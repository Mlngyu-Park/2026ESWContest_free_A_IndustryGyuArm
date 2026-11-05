"""ARM STUDIO 앱용 카메라 동기화 — 녹화 영상으로 카메라 위치 · 1인칭 · 판 위 물건(로봇·도킹)을 읽는다.

앱(arm_studio_web.py)은 이 모듈의 SYNC 하나만 쓴다:
    SYNC.load(path)               영상 분석 시작 (백그라운드, 수 초)
    SYNC.ctl(cmd)                 play / pause / fp(1인칭) / view(1인칭 보기: 영상+트윈 → 트윈만 → 영상만) /
                                  auto(자동 반영 켜기·끄기) / rot(판 90도) / x± y±(한 칸) / tx± ty±(한 타일) / reset
    SYNC.state(origin, ztop)      뷰어 /state 에 실을 dict (프레임 = 재생 시각)
    SYNC.jpeg(k)                  k 번째 프레임 JPEG

좌표
  판 좌표: pose.py — 판 30×30 의 첫 목심이 원점 (m, z 위). 첫 목심은 타일 경계에 맞춰져 있다.
  앱 월드: 판의 첫 목심을 앱 바닥판 '왼쪽 아래 타일' 의 첫 목심(origin)에 맞춘다.
           rot(판 가운데 기준 90도씩)·shift 로 사용자가 고칠 수 있다.

물건 (objects.py)
  '가려진 구멍' 으로 로봇(위치만)과 도킹(위치 + 색)을 읽는다. 영상을 처음부터 따라가며 판정을
  시간순으로 기록해 두고, 재생 시각까지 카메라가 본 만큼의 배치를 낸다 (녹화 영상에서의 실시간).
"""
import os, sys, threading, time, math
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path: sys.path.insert(0, HERE)
from grid import Board, detect_holes, pitch_of            # noqa: E402
from pose import PITCH, focal_from_H, K_of, camera_pose   # noqa: E402
from objects import Tracker, N, COLOR_HEX                 # noqa: E402

SIDE = 29 * PITCH                 # 첫 목심 ~ 마지막 목심 (580 mm)
JPEG_W = 960                      # 1인칭 배경으로 보내는 영상 폭
SNAP_EVERY = 4                    # 물건 판정을 몇 프레임마다 기록하나
PERSIST = 3                       # 연속 몇 번 같은 자리에 있어야 인정하나 (깜빡임 막기)


def robot_template(mesh_path):
    """로봇 받침 메쉬(base_link)에서 자세 맞추기에 쓸 것들 (로컬 m).
    위에서 본 높이 지도로: 날개 판 윗면 높이, 기둥 가운데, 판 구멍들, '틈 있는 가장자리' 의 구멍 한 쌍."""
    import trimesh
    from scipy import ndimage
    m = trimesh.load(mesh_path); v = np.asarray(m.vertices) * 1000; F = np.asarray(m.faces)
    lo = v[:, :2].min(0) - 5; hi = v[:, :2].max(0) + 5
    Hm = np.full((int(hi[1] - lo[1]) + 1, int(hi[0] - lo[0]) + 1), -1.0)
    for f in F:
        t = v[f]; z = t[:, 2].max()
        msk = np.zeros(Hm.shape, np.uint8); cv2.fillPoly(msk, [(t[:, :2] - lo).astype(np.int32)], 1)
        b = msk.astype(bool); Hm[b] = np.maximum(Hm[b], z)
    z0 = v[:, 2].min()
    lvl = float(np.median(Hm[(Hm > z0) & (Hm < z0 + 25)]))           # 낮은 판 윗면
    col = np.argwhere(Hm > lvl + 5).mean(0)[::-1] + lo
    inside = ndimage.binary_fill_holes(Hm > 0)
    dent = inside & ((Hm < 0) | (Hm < lvl - 2)) & ~(Hm > lvl + 5)
    dent &= ~ndimage.binary_dilation(Hm > lvl + 5, iterations=2)
    lab, n = ndimage.label(dent)
    holes = np.array([np.argwhere(lab == k).mean(0)[::-1] + lo for k in range(1, n + 1)
                      if 15 <= (lab == k).sum() <= 200])
    # 기둥에서 가장 먼 쪽 가장자리의 구멍 한 쌍 (실물에서는 '구멍 줄 + 틈' 이 있는 앞 가장자리)
    d = (holes - col) @ ((holes.mean(0) - col) / (np.linalg.norm(holes.mean(0) - col) + 1e-9))
    front = holes[np.argsort(d)[-2:]].mean(0)
    return {'hb': (lvl - z0) / 1000, 'col': col / 1000, 'front': front / 1000}


def robot_pose_from_plate(frames, Hs, poses, K, window, tp, near_b, tmpl):
    """받침 판 목심 구멍 줄 + 몸통 방향으로 로봇의 판 좌표 자세 (90도 단위).
    반환 {'R': 2x2, 't': (x,y) m, 'yaw': rad, 'n_holes': n}  —  p_판 = R p_로봇로컬 + t.  못 찾으면 None.

    ① 영상을 '받침 판 윗면 높이' 로 펴서 중앙값 → 받침 판 위 구멍이 또렷해진다 (판 구멍은 번진다)
    ② 판 목심 격자에 딱 맞는 어두운 점만 = 받침 구멍 (받침이 목심에 꽂히니 받침 구멍도 격자 위)
    ③ 가장 많이 늘어선 한 줄 = 앞 가장자리 구멍 줄. 몸통(흰 덩어리)은 그 반대편
    ④ 앞뒤 축 = 구멍 줄에 수직(판 격자 방향), 앞쪽 = 몸통 반대편 → 메쉬의 '기둥 → 앞 구멍쌍' 과 맞춰 각도
    ★ 받침 판도 비스듬히 찍혀 기울어져 보인다 → ① 에서 카메라 자세로 받침 판 높이에 맞춰 반듯하게 편다 ★"""
    from scipy import ndimage
    SZ = 620; hb = tmpl['hb']
    S = np.array([[1e-3, 0, -10e-3], [0, 1e-3, -10e-3], [0, 0, 0.0]])
    tops = []
    for f, ps in zip(frames, poses):
        if ps is None: continue
        C, Rwc = ps
        M = K @ Rwc.T @ (S + np.outer(np.array([0, 0, hb]) - C, [0, 0, 1]))
        tops.append(cv2.warpPerspective(f, M, (SZ, SZ), flags=cv2.WARP_INVERSE_MAP, borderValue=(0, 0, 0)))
    if not tops: return None
    st = np.stack(tops).astype(np.float32); st[st.sum(-1) == 0] = np.nan
    with np.errstate(all='ignore'):
        E = np.nan_to_num(np.nanmedian(st, 0)).astype(np.uint8)
    cu, cv = near_b[0] * 1000 + 10, near_b[1] * 1000 + 10                  # 받침 대략 자리 (그림 px = mm)
    x0, x1 = int(max(0, cu - 130)), int(min(SZ, cu + 130)); y0, y1 = int(max(0, cv - 130)), int(min(SZ, cv + 130))
    gray = cv2.cvtColor(E, cv2.COLOR_BGR2GRAY).astype(np.int16); hsv = cv2.cvtColor(E, cv2.COLOR_BGR2HSV)
    dark = np.clip(cv2.GaussianBlur(gray, (0, 0), 6) - gray, 0, 255).astype(np.uint8)
    wm = ((hsv[..., 1] < 50) & (hsv[..., 2] > 140)).astype(np.uint8)
    roi = np.zeros_like(dark); roi[y0:y1, x0:x1] = 1
    _, th = cv2.threshold(dark, 12, 255, cv2.THRESH_BINARY)
    nl, _, ss, cc = cv2.connectedComponentsWithStats(th * roi * cv2.dilate(wm, np.ones((15, 15), np.uint8)), 8)
    dots = np.array([cc[i] for i in range(1, nl) if 12 <= ss[i, 4] <= 120 and max(ss[i, 2], ss[i, 3]) <= 16]).reshape(-1, 2)
    if len(dots) < 2: return None
    on = dots[(np.abs(((dots - 10) % 20 + 10) % 20 - 10) < 3.5).all(1)]
    rows = {}
    for p in on:
        rows.setdefault(('y', int(round((p[1] - 10) / 20))), []).append(p)
        rows.setdefault(('x', int(round((p[0] - 10) / 20))), []).append(p)
    if not rows: return None
    (axis, _), line = max(rows.items(), key=lambda kv: len(kv[1]))
    if len(line) < 2: return None
    hole_mid = np.mean(line, 0)
    lab, n = ndimage.label(wm * roi)
    if n == 0: return None
    big = lab == (1 + int(np.argmax(ndimage.sum(wm * roi, lab, range(1, n + 1)))))
    pts = np.argwhere(big)[:, ::-1].astype(float)
    far = pts[np.linalg.norm(pts - hole_mid, axis=1) > 25]
    if len(far) == 0: return None
    # 구멍 줄은 판 격자를 따라 가로('y' 같음) 또는 세로('x' 같음)로 놓인다 → 앞뒤 축은 줄에 수직으로 딱 정해진다.
    # 몸통(흰 덩어리)은 그 축의 어느 쪽이 '뒤' 인지만 가른다 (몸통 번짐으로 생기는 각도 오차를 없앤다).
    body = far.mean(0); raw = hole_mid - body
    d_r = np.array([0.0, np.sign(raw[1]) or 1.0]) if axis == 'y' else np.array([np.sign(raw[0]) or 1.0, 0.0])
    d_l = (tmpl['front'] - tmpl['col']) * 1000
    yaw = np.arctan2(raw[1], raw[0]) - np.arctan2(d_l[1], d_l[0])            # 참고용 (몸통으로만 잰 각)
    yaw_axis = np.arctan2(d_r[1], d_r[0]) - np.arctan2(d_l[1], d_l[0])
    yaw90 = round(yaw_axis / (np.pi / 2)) * (np.pi / 2)
    c, s = np.cos(yaw90), np.sin(yaw90); R = np.array([[c, -s], [s, c]])
    t = (hole_mid - 10) / 1000 - R @ tmpl['front']
    return {'R': R, 't': t, 'yaw': float(yaw90), 'yaw_raw': float(yaw), 'n_holes': len(line)}


def analyse(path, progress=None, robot_mesh=None):
    """영상 하나를 끝까지 분석한다. 앱(CamSync)과 RViz(cam_rviz) 가 같이 쓴다."""
    cap = cv2.VideoCapture(path); fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
    B = Board(); frames, Hs = [], []
    while True:
        ok, f = cap.read()
        if not ok: break
        g = B.add(f); frames.append(f); Hs.append(B.H.copy() if g else None)
        if progress: progress(f'격자 추적 {len(frames)}프레임')
    if not B.holes: raise RuntimeError('판 격자를 한 프레임도 못 찾았습니다')
    i0, j0 = B.window(); tp = B.tile_phase(); h, w = frames[0].shape[:2]
    fs = [x for x in (focal_from_H(H, w, h) for H in Hs if H is not None) if x]
    f = float(np.median(fs)); K = K_of(f, w, h)
    poses = [camera_pose(H, K, i0, j0)[:2] if H is not None else None for H in Hs]

    # 판 크기: 타일(10×10) 마다 구멍을 30% 이상 봤으면 '있는 타일'
    tiles = []
    for u in range(3):
        for v in range(3):
            seen = sum(1 for (a, b) in B.holes
                       if i0 + 10 * u <= a < i0 + 10 * u + 10 and j0 + 10 * v <= b < j0 + 10 * v + 10)
            if seen >= 30: tiles.append((u, v))

    # 물건 판정을 시간순으로
    def board_color(i, j):                 # 판 무늬 — 파란 타일 위 파랑은 물건 색으로 안 친다
        even = ((i - tp['p']) // 10 + (j - tp['q']) // 10) % 2 == 0
        return tp['짝수타일색'] if even else ('W' if tp['짝수타일색'] == 'B' else 'B')
    T = Tracker(i0, j0, board_color, (tp['p'], tp['q'])); timeline = []
    for k, (fr, H) in enumerate(zip(frames, Hs)):
        if H is not None:
            T.update(fr, H, detect_holes(fr, pitch_of(H, fr.shape)))
        if k % SNAP_EVERY == SNAP_EVERY - 1 or k == len(frames) - 1:
            timeline.append((k, T.objects(), T.free().copy()))
            if progress: progress(f'물건 판정 {k + 1}/{len(frames)}프레임')
    # 로봇 받침 자세 (위치 + 방향): 받침 판의 목심 구멍 줄로. 메쉬를 모르면 건너뛴다.
    robot_pose = None
    last_rob = next((o for o in timeline[-1][1] if o['kind'] == 'robot'), None) if timeline else None
    if robot_mesh and last_rob is not None:
        try:
            tmpl = robot_template(robot_mesh)
            near = np.array(last_rob['base']) * PITCH
            robot_pose = robot_pose_from_plate(frames, Hs, poses, K, (i0, j0), tp, near, tmpl)
            if progress: progress('로봇 받침 자세 ' + ('찾음' if robot_pose else '못 찾음'))
        except Exception as e:
            if progress: progress(f'로봇 받침 자세 실패: {e}')
    return dict(frames=frames, Hs=Hs, poses=poses, K=K, f=f, fps=fps, wh=(w, h),
                window=(i0, j0), phase=tp, check=B.check(), tiles=tiles, timeline=timeline,
                robot_pose=robot_pose)


class CamSync:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = '영상 없음'
        self.busy = False; self.ready = False
        self.path = None; self.data = None; self.jpgs = []
        self.playing = False; self.t0 = 0.0; self.k_pause = 0
        self.fp = False                          # 1인칭
        self.view = 'both'                       # 1인칭 보기: both(영상+트윈) / twin(트윈만) / video(영상만)
        self.auto = True                         # 읽은 배치를 앱에 자동 반영
        self.rot = 0; self.shift = [0, 0]

    # ---------------------------------------------------------------- 분석
    robot_mesh = None                            # 앱이 넣어 준다 (로봇 받침 메쉬 경로)

    def load(self, path):
        path = os.path.expanduser(str(path or '').strip())
        if not os.path.isabs(path): path = os.path.join(HERE, path)
        if not os.path.exists(path): return f'영상 파일이 없습니다: {path}'
        if self.busy: return '이미 분석 중입니다'
        self.busy = True; self.ready = False; self.status = '분석 중...'

        def work():
            try:
                d = analyse(path, progress=lambda m: setattr(self, 'status', '분석 중... ' + m),
                            robot_mesh=self.robot_mesh)
                jpgs = []
                for fr in d['frames']:
                    s = JPEG_W / fr.shape[1]
                    jpgs.append(cv2.imencode('.jpg', cv2.resize(fr, (JPEG_W, int(fr.shape[0] * s))),
                                             [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes())
                d.pop('frames')                       # 원본 프레임은 메모리에서 내린다
                with self.lock:
                    self.path, self.data, self.jpgs = path, d, jpgs
                    self.playing, self.t0, self.k_pause = True, time.time(), 0
                    self.ready = True
                last = d['timeline'][-1][1] if d['timeline'] else []
                docks = ', '.join(o['color'] for o in last if o['kind'] == 'docking') or '없음'
                self.status = (f"{os.path.basename(path)} · {len(jpgs)}프레임 · 판 목심 {d['check']['판_안_구멍']} · "
                               f"판 {len(d['tiles'])}타일 · 로봇 {'있음' if any(o['kind'] == 'robot' for o in last) else '못 찾음'}"
                               f"{' (방향 %d°, 받침 구멍 %d개)' % (round(math.degrees(d['robot_pose']['yaw'])) % 360, d['robot_pose']['n_holes']) if d.get('robot_pose') else ''} · "
                               f"도킹 {docks}")
            except Exception as e:
                self.status = f'분석 실패: {e}'
            finally:
                self.busy = False
        threading.Thread(target=work, daemon=True).start()
        return '영상 분석을 시작했습니다 (수십 초 걸릴 수 있습니다)'

    # ---------------------------------------------------------------- 조작
    def ctl(self, cmd):
        cmd = str(cmd)
        with self.lock:
            if cmd == 'play':
                if not self.playing: self.playing, self.t0 = True, time.time() - self.k_pause / self._fps()
            elif cmd == 'pause':
                if self.playing: self.k_pause, self.playing = self._k(), False
            elif cmd == 'fp':   self.fp = not self.fp
            elif cmd in ('view', 'bg'):              # 보기 바꾸기: 영상+트윈 → 트윈만 → 영상만
                self.view = {'both': 'twin', 'twin': 'video', 'video': 'both'}[self.view]
            elif cmd == 'auto': self.auto = not self.auto
            elif cmd == 'rot':  self.rot = (self.rot + 1) % 4
            elif cmd in ('x+', 'x-', 'y+', 'y-', 'tx+', 'tx-', 'ty+', 'ty-'):
                step = 10 if cmd.startswith('t') else 1
                self.shift[0 if 'x' in cmd else 1] += step if cmd.endswith('+') else -step
            elif cmd == 'reset': self.rot, self.shift = 0, [0, 0]
            else: return f'모르는 명령: {cmd}'
        return self.summary()

    def summary(self):
        return (f"{self.status} | {'재생' if self.playing else '정지'} · 1인칭 {'켬' if self.fp else '끔'}"
                f"{'(' + {'both': '영상+트윈', 'twin': '트윈만', 'video': '영상만'}[self.view] + ')' if self.fp else ''} · 자동 반영 {'켬' if self.auto else '끔'} · "
                f"판 회전 {self.rot * 90}° · 이동 ({self.shift[0]},{self.shift[1]})칸")

    def _fps(self): return self.data['fps'] if self.data else 15.0

    def _k(self):
        n = len(self.jpgs)
        if n == 0: return 0
        if not self.playing: return self.k_pause % n
        return int((time.time() - self.t0) * self._fps()) % n

    # ---------------------------------------------------------------- 좌표
    def _to_world(self, origin, ztop):
        """판 좌표 -> 앱 월드: p_w = R p_b + t.  판 가운데 기준으로 rot, 그다음 shift."""
        a = self.rot * math.pi / 2
        R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
        c_b = np.array([SIDE / 2, SIDE / 2, 0.0])
        c_w = np.array([origin[0] + SIDE / 2 + self.shift[0] * PITCH,
                        origin[1] + SIDE / 2 + self.shift[1] * PITCH, ztop])
        return R, c_w - R @ c_b

    def _cell_world(self, R, t, ab):
        return R @ np.array([ab[0] * PITCH, ab[1] * PITCH, 0.0]) + t

    def layout(self, origin, ztop):
        """지금 재생 시각까지 읽은 배치 (앱 월드 좌표). 자동 반영이 쓴다."""
        if not self.ready: return None
        with self.lock:
            k = self._k(); R, t = self._to_world(origin, ztop)
            hist = [(objs, free) for kk, objs, free in self.data['timeline'] if kk <= k][-PERSIST:]
            if len(hist) < PERSIST: return {'k': k, 'robot': None, 'dockings': [], 'free': None, 'R': R, 't': t}
            objs, free = hist[-1]
            # 깜빡임 막기: 최근 PERSIST 번의 판정에 계속 (같은 색, 3칸 안) 있던 것만 인정
            def steady(o):
                for old, _ in hist[:-1]:
                    key = 'base' if o['kind'] == 'robot' else 'center'     # 로봇은 받침 위치로 비교
                    tol = 5 if o['kind'] == 'robot' else 3
                    if not any(p['kind'] == o['kind'] and p['color'] == o['color'] and
                               math.dist(p.get(key, p['center']), o.get(key, o['center'])) <= tol for p in old): return False
                return True
            objs = [o for o in objs if steady(o)]
            rob = next((o for o in objs if o['kind'] == 'robot'), None)
            if rob is not None:
                # 로봇은 움직이지 않는다 → 지금까지 '받침' 으로 가장 자주 나온 자리 (2칸 단위 다수결).
                # 팔이 판 위로 뻗으면 팔 밑이 더 크게 가려져 받침으로 잘못 뽑히는 때가 있다 (실측).
                bases = [next(x for x in oo if x['kind'] == 'robot')['base']
                         for kk, oo, _ in self.data['timeline']
                         if kk <= k and any(x['kind'] == 'robot' for x in oo)]
                # 3칸 안의 것끼리 묶어 가장 큰 묶음 (반올림 칸으로 묶으면 경계에서 표가 갈라진다, 실측)
                best = max(([b for b in bases if math.dist(b, c) <= 3] for c in bases), key=len)
                rob = dict(rob, base=[float(v) for v in np.mean(best, 0)])
            rp = self.data.get('robot_pose'); robot_pose = None
            if rp is not None:                                  # 판 자세 -> 월드: 기준점 위치 + 방향
                o = R @ np.array([rp['t'][0], rp['t'][1], 0.0]) + t
                robot_pose = {'base_off': [float(o[0]), float(o[1])], 'yaw': float(rp['yaw'] + self.rot * math.pi / 2)}
            return {'k': k, 'R': R, 't': t, 'free': free, 'robot_pose': robot_pose,
                    'robot': None if rob is None else [float(v) for v in self._cell_world(R, t, rob['base'])[:2]],
                    'dockings': [{'color': o['color'], 'hex': o['hex'], 'n': o['n'], 'cells': o['cells'],
                                  'xy': [float(v) for v in self._cell_world(R, t, o['center'])[:2]]}
                                 for o in objs if o['kind'] == 'docking']}

    def is_free_world(self, lay, xy, r_cells=1):
        """앱 월드 (x,y) 자리가 '또렷이 비어 있음' 인가 (뽑았다고 판단해도 되나)."""
        if lay is None or lay.get('free') is None: return False
        R, t = lay['R'], lay['t']
        b = R.T @ (np.array([xy[0], xy[1], t[2]]) - t)
        a0, b0 = int(round(b[0] / PITCH)), int(round(b[1] / PITCH))
        fr = lay['free']; cells = [(a0 + da, b0 + db) for da in range(-r_cells, r_cells + 1)
                                   for db in range(-r_cells, r_cells + 1) if 0 <= a0 + da < N and 0 <= b0 + db < N]
        return bool(cells) and sum(fr[c] for c in cells) >= 0.6 * len(cells)

    def state(self, origin, ztop):
        if not self.ready:
            return {'on': False, 'status': self.summary() if self.busy or self.path else self.status}
        lay = self.layout(origin, ztop)
        with self.lock:
            d = self.data; k = self._k(); ps = d['poses'][k] if d['poses'] else None
            R, t = self._to_world(origin, ztop); w, h = d['wh']
            out = {'on': True, 'k': k, 'n': len(self.jpgs), 'fp': self.fp, 'view': self.view, 'auto': self.auto,
                   'status': self.summary(), 'aspect': w / h,
                   'fovy': math.degrees(2 * math.atan(h / 2 / d['K'][1, 1])),
                   'center': [float(v) for v in (R @ np.array([SIDE / 2, SIDE / 2, 0.0]) + t)],
                   'corners': [[float(v) for v in (R @ np.array([x, y, 0.0]) + t)]
                               for x, y in ((-PITCH / 2, -PITCH / 2), (SIDE + PITCH / 2, -PITCH / 2),
                                            (SIDE + PITCH / 2, SIDE + PITCH / 2), (-PITCH / 2, SIDE + PITCH / 2))],
                   'tiles': len(d['tiles']),
                   'objs': ([{'kind': 'robot', 'xy': lay['robot']}] if lay and lay['robot'] else []) +
                           ([{'kind': 'docking', 'xy': o['xy'], 'hex': o['hex']} for o in lay['dockings']] if lay else [])}
            if ps is None:
                out['pos'] = None
            else:
                C, Rwc = ps
                Cw = R @ C + t; Rw = R @ Rwc
                dd = Cw - np.array(out['center'])
                out.update(pos=[float(v) for v in Cw], R=[float(v) for v in Rw.ravel()],
                           theta=float(math.degrees(math.asin(dd[2] / np.linalg.norm(dd)))),
                           phi=float(math.degrees(math.atan2(dd[1], dd[0]))), dist=float(np.linalg.norm(dd)))
            return out

    def tiles_needed(self):
        """영상에서 본 판 타일 (판 좌표의 타일 u,v). 앱 바닥판 맞추기 제안에 쓴다."""
        return list(self.data['tiles']) if self.ready else []

    def jpeg(self, k):
        with self.lock:
            if not self.jpgs: return None
            return self.jpgs[int(k) % len(self.jpgs)]


SYNC = CamSync()
