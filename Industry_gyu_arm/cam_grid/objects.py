"""판 위 물건 찾기 — '가려진 구멍' 으로 로봇과 도킹의 자리·색을 읽는다.

원리
  판 900개 목심의 화면 위치는 프레임마다 안다 (격자 추적).
  화면 안에 있는데 구멍이 안 보이면 그 프레임에선 '가려짐'.
  카메라가 움직이며 여러 방향에서 보므로, 한 방향에서만 가려진 구멍은 다른 방향에서 보인다.
  → 계속 가려진 구멍 = 물건이 올라간 자리.

  가려진 자리의 색(그 픽셀)을 모아 물건 색을 정한다.
  판 색(파랑·흰)과 같은 색은 물건 색으로 치지 않는다 — 파란 타일 위의 파란 물건은
  그 칸에서는 색 판정을 건너뛰고, 흰 타일 위에 걸친 부분에서 정한다.

실시간
  Tracker.update(frame, H) 를 프레임마다 부르면 상태가 갱신된다 (지수 이동 평균).
  물건을 뽑으면 그 자리 구멍이 다시 보이기 시작해 몇 초 안에 '빈 자리' 로 돌아간다.
"""
import cv2, numpy as np
from scipy import ndimage

N = 30                # 옛 3×3 판 (녹화 영상용 camsync). 휴대폰(livecam)은 Tracker(shape=(20, 40))
ALPHA = 0.15          # 이동 평균 — 한 번 볼 때마다 이만큼 새 값 쪽으로
OCC = 0.35            # 같은 종류 자리의 평소 보임 점수의 이 배보다 낮으면 '물건 있음'
MIN_SEEN = 6          # 이만큼은 화면 안에서 봐야 판정한다


def _hsv_name(h, s, v):
    """색 이름 (OpenCV HSV, h 0~180)."""
    if v < 60: return 'black'
    if s < 50: return 'white' if v > 150 else 'gray'
    if h < 8 or h >= 158: return 'red'          # 그늘진 빨강은 160 근처로 읽힌다 (보라로 잘못 읽힘, 실측)
    if h < 20: return 'orange'
    if h < 35: return 'yellow'
    if h < 85: return 'green'
    if h < 130: return 'blue'
    return 'purple' if s > 90 else 'gray'


COLOR_HEX = {'red': '#e6194b', 'orange': '#f58231', 'yellow': '#ffe119', 'green': '#3cb44b',
             'blue': '#4363d8', 'purple': '#911eb4'}
SAT_COLORS = set(COLOR_HEX)


class Tracker:
    def __init__(self, i0, j0, board_color=None, phase=(0, 0), shape=(N, N)):
        self.i0, self.j0 = i0, j0
        self.ni, self.nj = shape            # 판 칸 수 (i, j) — 2×4 판은 (20, 40) (11-03)
        ni, nj = shape
        self.phase = phase                  # 타일 경계 위상 (grid.Board.tile_phase 의 p, q)
        self.vis = np.ones((ni, nj))        # 보임 점수 (1 = 늘 보임)
        self.cnt = np.zeros((ni, nj), int)  # 화면 안에서 본 횟수
        self.hit = np.zeros((ni, nj), int)  # 구멍이 실제로 보인 횟수 ('비었음' 판정의 증거)
        self.colors = [[{} for _ in range(nj)] for _ in range(ni)]
        self.board_color = board_color      # (i,j) -> 'B'/'W' (판 무늬, 없으면 체크 위상으로)

    def expected(self, i, j):
        if self.board_color is not None: return self.board_color(i, j)
        return None

    def update(self, frame, H, holes_px):
        """holes_px: 이 프레임에서 찾은 구멍 중심들 (픽셀)."""
        h, w = frame.shape[:2]
        ni, nj = self.ni, self.nj
        I, J = np.meshgrid(np.arange(ni), np.arange(nj), indexing='ij')
        G = np.stack([I + self.i0, J + self.j0], -1).reshape(-1, 1, 2).astype(np.float32)
        P = cv2.perspectiveTransform(G, H).reshape(ni, nj, 2)
        # 이 프레임에서 보인 구멍 (격자좌표로 바꿔 반올림)
        seen = np.zeros((ni, nj), bool)
        if len(holes_px):
            ij = cv2.perspectiveTransform(holes_px.reshape(-1, 1, 2).astype(np.float32), np.linalg.inv(H)).reshape(-1, 2)
            r = np.round(ij); ok = np.hypot(*(ij - r).T) < 0.3
            for a, b in r[ok].astype(int):
                a -= self.i0; b -= self.j0
                if 0 <= a < ni and 0 <= b < nj: seen[a, b] = True
        # 화면 안 (가장자리 여유) + 구멍이 너무 작게 찍히지 않는 곳만
        q = cv2.perspectiveTransform(np.float32([[[self.i0 + 15, self.j0 + 15], [self.i0 + 16, self.j0 + 15]]]), H)[0]
        m = 12
        inv = (P[..., 0] > m) & (P[..., 0] < w - m) & (P[..., 1] > m) & (P[..., 1] < h - m)
        self.vis[inv] = (1 - ALPHA) * self.vis[inv] + ALPHA * seen[inv]
        self.cnt[inv] += 1
        self.hit[inv & seen] += 1
        # 가려진 자리의 색
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for a, b in zip(*np.where(inv & ~seen)):
            x, y = int(P[a, b, 0]), int(P[a, b, 1])
            hh, ss, vv = hsv[y - 2:y + 3, x - 2:x + 3].reshape(-1, 3).mean(0)
            name = _hsv_name(hh, ss, vv)
            exp = self.expected(a + self.i0, b + self.j0)
            if name == 'blue' and exp == 'B': continue          # 파란 타일 위 파랑은 물건색으로 안 친다
            if name == 'white' and exp == 'W': continue
            d = self.colors[a][b]; d[name] = d.get(name, 0) + 1

    def baseline(self, R=6, pct=75):
        """자리마다 '평소라면 이만큼은 보인다' 는 기준.
        같은 종류 자리(타일 가장자리 줄인가 · 안쪽인가)의 '가까운 이웃' 중 잘 보이는 쪽(상위 25%).
          - 가장자리 줄은 퍼즐 이음새에 걸려 원래 덜 잡힌다 (실측 0.2~0.3 vs 안쪽 0.55)
          - 카메라에서 먼 곳은 구멍이 작고 흐려 덜 잡힌다
          → 판 전체 평균이 아니라 '주변의 같은 종류' 와 비교해야 한다."""
        ok = self.cnt >= MIN_SEEN
        I, J = np.meshgrid(np.arange(self.ni), np.arange(self.nj), indexing='ij')
        ei = ((I + self.i0 - self.phase[0]) % 10 == 0) | ((I + self.i0 - self.phase[0]) % 10 == 9)
        ej = ((J + self.j0 - self.phase[1]) % 10 == 0) | ((J + self.j0 - self.phase[1]) % 10 == 9)
        cls = ei.astype(int) * 2 + ej.astype(int)
        base = np.ones((self.ni, self.nj))
        for a in range(self.ni):
            for b in range(self.nj):
                sl = (slice(max(0, a - R), a + R + 1), slice(max(0, b - R), b + R + 1))
                m = ok[sl] & (cls[sl] == cls[a, b])
                if m.sum() >= 4: base[a, b] = max(0.05, float(np.percentile(self.vis[sl][m], pct)))
        return base

    def edge_mask(self):
        """타일 경계 줄(타일 안 0·9번째). 퍼즐 이음새에 걸려 구멍이 한 번도 안 잡힌다 (실측 보임 0)
        → 정보가 없는 자리라 판단에서 빼고 이웃으로 채운다."""
        I, J = np.meshgrid(np.arange(self.ni), np.arange(self.nj), indexing='ij')
        ei = (I + self.i0 - self.phase[0]) % 10
        ej = (J + self.j0 - self.phase[1]) % 10
        return (ei == 0) | (ei == 9) | (ej == 0) | (ej == 9)

    def free(self, R=0.8, min_hit=4):
        """또렷이 보이는 자리 (물건이 없다고 확신). 물건을 뽑았는지 판단할 때 쓴다.
        ★ 구멍이 실제로 min_hit 번 이상 보였어야 한다 — 보임 점수는 1 에서 시작하므로,
          덜 본 자리는 막혀 있어도 점수가 높게 남아 '비었음' 으로 착각한다 (실측: 영상 반복 때 도킹 삭제)."""
        return ((self.cnt >= MIN_SEEN) & (self.hit >= min_hit) &
                (self.vis >= R * self.baseline()) & ~self.edge_mask())

    def occupied(self):
        """① 안쪽 자리: 주변 같은 종류보다 훨씬 안 보이면 '물건 있음'
           ② 경계 자리: 이웃 안쪽 자리 중 둘 이상이 '물건 있음' 이면 같이 '물건 있음'"""
        edge = self.edge_mask(); ok = self.cnt >= MIN_SEEN
        inner = (~edge) & ok & (self.vis < OCC * self.baseline())
        nb = ndimage.convolve(inner.astype(int), np.ones((3, 3), int), mode='constant')
        return inner | (edge & (nb >= 2))

    def cell_labels(self, min_votes=4, sat_share=0.4):
        """가려진 칸마다 색 이름 붙이기 (판 색은 이미 빼고 셌다).
        진한 색이 많으면 그 색(도킹), 흰·검·회색이면 'robot', 색 증거가 거의 없으면 None.
        ★ 색 증거가 없는 칸은 버린다 — 멀리 있는 파란 타일처럼 '원래 잘 안 보이는' 곳은
          가려진 게 아니라서 판 색 외의 색 기록이 거의 없다."""
        occ = self.occupied() & ~self.edge_mask()
        lab = np.full((self.ni, self.nj), None, object)
        for a, b in zip(*np.where(occ)):
            v = self.colors[a][b]; tot = sum(v.values())
            if tot < min_votes: continue
            sat = {c: n for c, n in v.items() if c in SAT_COLORS}
            if sat and sum(sat.values()) / tot >= sat_share: lab[a, b] = max(sat, key=sat.get)
            elif (v.get('black', 0) + v.get('gray', 0) + v.get('white', 0)) / tot >= 0.6: lab[a, b] = 'robot'
        return lab

    def objects(self, min_cells=3):
        """같은 이름 칸끼리 묶는다. 경계 줄(빈칸 1줄)을 건너 이어지도록 한 칸 넓혀서 묶는다.
        반환: [{'kind':'robot'|'docking','color','n','cells','center'(격자 i,j)}] (큰 것부터)"""
        lab = self.cell_labels(); out = []
        for name in set(x for x in lab.ravel() if x):
            m = (lab == name)
            grown = ndimage.binary_dilation(m, np.ones((3, 3)))          # 경계 줄 건너기
            comp, n = ndimage.label(grown, structure=np.ones((3, 3)))
            for k in range(1, n + 1):
                cells = np.argwhere((comp == k) & m)
                if len(cells) < min_cells: continue
                out.append({'kind': 'robot' if name == 'robot' else 'docking',
                            'color': None if name == 'robot' else name,
                            'hex': COLOR_HEX.get(name), 'n': int(len(cells)),
                            'cells': [(int(a), int(b)) for a, b in cells],
                            'center': [float(v) for v in cells.mean(0)]})
        # 로봇은 하나뿐 — 가장 큰 흰·검 덩어리만 로봇, 나머지 흰·검 조각은 버린다
        robots = sorted([o for o in out if o['kind'] == 'robot'], key=lambda o: -o['n'])
        if robots:
            # 위치 = 받침. 팔이 판 위로 뻗어 가린 자리는 듬성듬성하므로, 한 겹 깎아서
            # 남는 빽빽한 부분(받침)의 가운데를 쓴다. 다 깎이면 원래 덩어리 그대로.
            r = robots[0]; m = np.zeros((self.ni, self.nj), bool)
            for a, b in r['cells']: m[a, b] = True
            core = ndimage.binary_erosion(ndimage.binary_closing(m, np.ones((3, 3))), np.ones((3, 3)))
            comp, n = ndimage.label(core)
            if n:
                big = np.argwhere(comp == 1 + int(np.argmax(ndimage.sum(core, comp, range(1, n + 1)))))
                r['base'] = [float(v) for v in big.mean(0)]
            else:
                r['base'] = r['center']
        out = [o for o in out if o['kind'] != 'robot'] + robots[:1]
        out.sort(key=lambda o: -o['n'])
        return out
