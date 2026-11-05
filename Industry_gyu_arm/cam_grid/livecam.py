"""휴대폰 실시간 카메라 — 손에 든 휴대폰 영상으로 판 격자를 따라가며 카메라 위치를 트윈에 보인다.

흐름
  휴대폰 브라우저(/phone, https) 가 JPEG 프레임을 보낸다 -> push()
  일꾼 스레드가 '가장 최근 프레임 하나만' 처리한다 (밀리면 옛 프레임은 버린다 -> 늦어지지 않는다)
    격자 추적(grid.Board) -> 초점거리(최근 프레임 중앙값) -> 카메라 위치·방향(pose.camera_pose)
  state() 는 녹화 영상용 CamSync.state() 와 같은 모양이라, 뷰어가 그대로 카메라 위치·시야를 그린다.

판 좌표 -> 앱 좌표
  판 창(BI×BJ = 20×40, 2×4 타일 — 11-03)의 첫 목심 = 앱 바닥판 첫 목심 (origin) 에 놓고, 90° 회전(rot)·칸 이동(shift) 으로 맞춘다.
  (녹화 영상용과 같은 방식. 화면의 버튼으로 조정)
"""
import glob
import math
import os
import threading
import time
from collections import deque

import cv2
import numpy as np

import grid as _grid
from grid import Board, BI, BJ, TILE, detect_holes, pitch_of
from objects import Tracker, _hsv_name, COLOR_HEX
# ★ 납작한 타원 구멍 인정은 쓰지 않는다 (2026-10-22 실측) ★
#   받아 주면 큰 구멍 사이의 작은 배수 구멍까지 섞여 격자가 엉뚱한 무늬에 맞춰졌다
#   (초점거리 1166 -> 411px, 아이폰에서 517/3454px). 원래 기준으로도 비스듬한 아이폰 사진이 잘 잡힌다.
_grid.SHAPE_MAX_ASPECT = 2.0
_grid.FIT_EARLY = 0.75          # 다시 찾기: 충분히 맞으면 바로 멈춤 (1920px 에서 2~5초 -> 짧게)
_grid.RANSAC_ITERS = 300        # 다시 찾기 60번 × 다듬기 6번의 RANSAC 을 짧게 (어려운 장 4.6초 -> 짧게)
_grid.MOTION_SCALE = 0.5        # 화면 이동은 절반 크기로 잡는다 (특징점 32ms -> 12ms, 직전 장 것은 재사용)
_grid.ORB_N = 1500
_grid.CELL_OFS = 0.3            # 칸 색은 타일 안쪽에서 읽는다 (0.5 는 이음새 위 -> 판 무늬 1칸 어긋남)
_grid.CHECK_MIN = 0.85          # 다시 찾은 격자는 칸 색이 체크무늬와 85% 이상 맞아야 (구멍 4개 각도 검사; 실측 맞는 격자 중앙 99%, 틀린 격자 중앙 87%)
_grid.LOCAL_AREA = True         # 추적 중 구멍 크기를 자리마다 예상 (비스듬한 휴대폰 화면)
_grid.SHAPE_MIN_FILL = 0.40
_grid.SHAPE_MIN_AREA = 8
from pose import PITCH, focal_from_H, K_of, camera_pose

SIDE_I = (BI - 1) * PITCH           # 첫 목심 ~ 마지막 목심 (짧은 쪽 i)
SIDE_J = (BJ - 1) * PITCH           # 첫 목심 ~ 마지막 목심 (긴 쪽 j)
LIVE_TIMEOUT = 3.0                  # 이 시간(초) 동안 프레임이 안 오면 '연결 끊김'
F_KEEP = 40                         # 초점거리 중앙값에 쓰는 최근 프레임 수
GOOD_RATIO = 0.6                    # 찾은 구멍 중 격자에 맞는 비율이 이보다 낮으면 실패 (정상 0.8 이상)
GOOD_ERR_PX = 4.0                   # 격자 오차가 이보다 크면 실패 (정상 1px 안팎)
SAMPLE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'shots', 'live')   # 진단용 표본
SNAP_S = 1.0                        # 물건 판정을 몇 초마다 기록하나
PERSIST = 3                         # 연속 몇 번 같은 자리에 있어야 인정하나 (깜빡임 막기)
SLOW_S = 0.4                        # 느린 길(판 무늬·바닥색·물건·도킹)은 이 간격(초)마다
RESET_AFTER = 2                     # 이만큼 연속 실패하면 예전 격자를 버리고 처음부터 찾는다


MIN_BLUE_FRAC = 0.08                # 화면 속 판 칸 중 파란 칸이 이보다 적으면 판이 아니다 (키보드 등)
FOREIGN_ALPHA = 0.25                # 칸마다 '바닥색과 다름' 비율의 이동 평균
FOREIGN_ON = 0.6                    # 이 비율 이상이면 '물건이 서 있는 칸'
FOREIGN_MIN_SEEN = 4                # 이만큼은 봐야 판정
ROBOT_KEEPOUT = 6.0                 # 로봇 받침에서 이 칸 수 안의 덩어리는 도킹으로 안 본다
ROBOT_MAX_SAT = 70                  # 로봇(흰·회색) 칸의 채도 상한
ROBOT_MIN_CELLS = 5                 # 로봇 덩어리 최소 칸 수
ROBOT_MIN_FR = 0.4                  # 로봇 받침 자리의 '바닥색과 다름' 최소 (실측: 진짜 0.57~1.0, 흐린 판 0.0)
LEG_STEP = 2                        # 도킹 다리 간격 40mm = 목심 2칸
# 도킹 윗판의 'U 홈' (열린 쪽 = 도킹 로컬 -y). 다리 가운데 기준 mm, 새 도킹 STL 실측 (2026-10-22)
DOCK_TOP_Z = 0.095                  # 판 윗면에서 도킹 윗면까지 (STL 위 115mm - 판 20mm)
NOTCH = np.array([(-17, -25), (17, -25), (9, -6), (9, 5), (4, 10), (-4, 10), (-9, 5), (-9, -6)]) / 1000.0
YAW_ALPHA = 0.3                     # 방향 판정 이동 평균
YAW_MIN_N = 3                       # 이만큼 보고 나서 방향을 정한다
YAW_MARGIN = 0.12                   # 한 장에서 홈 자리(도킹 색이 가장 적은 쪽)가 둘째보다 이만큼 또렷해야 한 표
FIT_MIN = 0.19                      # 3D 모양 맞추기 점수 — 이 이상이어야 도킹 (실측: 진짜 0.21~0.35, 가짜 0~0.16)
SKIN_SAT = 140                      # 빨강·주황 도킹은 채도 중앙값이 이 이상 (손 피부: 실측 중앙 102, 90% 135)
YAW_VOTES = 2.5
YAW_MIN_ELEV = 50.0                 # 방향은 이 각도(°) 이상 '위에서 내려다본' 장에서만 판정한다
YAW_MIN_PX = 90                     # 그리고 윗판(55mm)이 화면에서 이만큼(px) 이상 크게 보일 때만
#   실측(10-22): 30° 쯤 비스듬히 보면 U 홈이 20~30px 인데 투영 오차가 ~10px 라 4방향 판정이 장마다 뒤집혔다.
#   그 전엔 앱의 도킹 방향을 그대로 둔다 (틀리게 돌리는 것보다 낫다).                     # 표가 이만큼 쌓여야 방향을 정한다 (그 전엔 앱의 방향을 그대로 둔다)


def _leg_square(occ, cells):
    """덩어리 칸들 근처에서 '다리 4개' 무늬(2칸 간격 정사각형 네 칸이 모두 가려짐)를 찾는다.
    여럿이면 덩어리 가운데에 가장 가까운 것. 반환: 정사각형 왼쪽 아래 칸 (a,b) 또는 None."""
    if not cells:
        return None
    c = np.mean(cells, 0)
    best = None
    a0, b0 = np.min(cells, 0) - LEG_STEP
    a1, b1 = np.max(cells, 0)
    for a in range(int(a0), int(a1) + 1):
        for b in range(int(b0), int(b1) + 1):
            pts = [(a, b), (a + LEG_STEP, b), (a, b + LEG_STEP), (a + LEG_STEP, b + LEG_STEP)]
            if all(0 <= x < occ.shape[0] and 0 <= y < occ.shape[1] and occ[x, y] for x, y in pts):
                d = math.dist((a + 1, b + 1), c)
                if best is None or d < best[0]:
                    best = (d, (a, b))
    return None if best is None else best[1]


class OneEuro:
    """손 떨림 거르기 (One Euro 필터). 느린 흔들림은 강하게 거르고, 크게 움직이면 바로 따라간다.
    min_cutoff 를 낮출수록 더 부드럽고(대신 조금 늦음), beta 를 높일수록 빠른 움직임에 덜 늦는다."""
    def __init__(self, min_cutoff=0.6, beta=4.0, d_cutoff=1.0):
        self.mc, self.beta, self.dc = min_cutoff, beta, d_cutoff
        self.x = self.dx = self.t = None

    @staticmethod
    def _a(cutoff, dt):
        r = 2 * math.pi * cutoff * dt
        return r / (r + 1)

    def __call__(self, x, t):
        x = np.asarray(x, float)
        if self.x is None or t <= self.t or t - self.t > 2.0:      # 처음이거나 오래 끊겼으면 그대로 시작
            self.x, self.dx, self.t = x, np.zeros_like(x), t
            return x
        dt = t - self.t
        dx = (x - self.x) / dt
        self.dx = self.dx + self._a(self.dc, dt) * (dx - self.dx)
        cutoff = self.mc + self.beta * float(np.linalg.norm(self.dx))
        self.x = self.x + self._a(cutoff, dt) * (x - self.x)
        self.t = t
        return self.x


def _orth(M):
    """거른 회전 행렬을 다시 올바른 회전으로 (SVD)."""
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


class LiveCam:
    def __init__(self):
        self.lock = threading.RLock()      # state() 안에서 summary() 가 다시 잡는다
        self.cv = threading.Condition(self.lock)
        self.latest = None              # (jpeg bytes, 받은 시각, 번호)
        self.n_in = 0                   # 받은 프레임 수
        self.n_done = 0                 # 처리한 프레임 수
        self.n_ok = 0                   # 격자를 찾은 프레임 수
        self.t_in = deque(maxlen=30)    # 받은 시각 (fps 계산)
        self.t_proc = deque(maxlen=30)  # 처리 시간
        self.jpg = None                 # 마지막으로 처리한 프레임 (1인칭 표시용)
        self.wh = None
        self.board = Board()
        self.fs = deque(maxlen=F_KEEP)
        self.f_mem = {}                 # (w,h) -> 초점거리 (휴대폰을 다시 연결해도 기억)
        self.K = None
        self.pose = None                # (C, Rwc) 판 좌표
        self.t_pose = 0.0
        self.window = None
        self.rot = 0                    # 판 -> 앱 90° 회전 수
        self.shift = [0, 0]             # 판 -> 앱 칸 이동
        self.fp = False                 # 1인칭
        self.view = 'both'
        self.msg = '휴대폰 연결을 기다리는 중'
        self.fails = 0                  # 연속 실패 수
        self.t_sample = 0.0             # 진단용 표본 저장 시각
        self.smooth = True              # 손 떨림 거르기
        self.f_pos = OneEuro(min_cutoff=0.6, beta=4.0)     # 위치 (m)
        self.f_rot = OneEuro(min_cutoff=0.6, beta=1.5)     # 방향 (회전 행렬 9칸)
        self.pose_s = None              # 거른 (C, Rwc)
        self.T = None                   # 물건 판정 (가려진 구멍 누적)
        self.reg = None                 # 앱이 정한 판 -> 앱 좌표 (R, t). 있으면 rot/shift 대신 쓴다
        self.rec_dir = None             # 녹화 중이면 모든 프레임을 여기 저장 (인식 맞추기용)
        self.T_win = None
        self.hist = deque(maxlen=8)     # (시각, objects, free)
        self.fr = np.zeros((BI, BJ))          # 칸마다 '바닥색과 다름' 이동 평균
        self.fr_n = np.zeros((BI, BJ), int)   # 본 횟수
        self.fr_cs = np.zeros((BI, BJ, 2))    # 다른 색의 색상(각도) 이동 평균 (cos, sin)
        self.fr_s = np.zeros((BI, BJ))        # 다른 색의 채도 이동 평균 (흰 로봇 = 낮음)
        self.fr_edge = np.zeros((BI, BJ), bool)   # 타일 가장자리 칸 (퍼즐 이음새 — 판정 안 함)
        self.fr_bluetile = np.zeros((BI, BJ), bool)   # 파란 타일 칸 (파란 다리는 여기서 안 보인다)
        self.cam_cells = deque(maxlen=15)           # 카메라 바닥점 (칸) — 다리 고를 때 '가까운 쪽'
        self.fr_win = None
        self.mirror = False                         # 판 격자 축이 거울상인가 (밖으로 낼 때 b -> BJ-1-b)
        self._turns = 0                             # 마지막으로 본 Board.turns (격자를 90° 돌리면 칸 번호가 바뀐다)
        self.cont = 0                               # 추적이 끊겼다 다시 찾은 횟수 (같으면 칸 번호 방향이 이어진다)
        self.board_id = 0                           # Board 를 새로 만든 횟수 (같으면 칸 번호가 이어진다)
        self.gen = 0                                # 판 좌표가 새로 잡힌 횟수 (앱의 판 맞춤이 이것과 짝)
        self.yaw_need = False                       # 방향을 못 정한 도킹이 있고 너무 비스듬히 보는 중
        self.ghosts = deque(maxlen=6)              # 최근 장마다 '앞 도킹이 가린 자리' 목록
        self.yaw_fr = {}                            # 도킹 (가운데 칸) -> [4방향 '홈 자리의 도킹색 비율' 평균, 본 횟수]
        self.t_snap = 0.0
        self.t_slow = 0.0                           # 느린 길(판 무늬·바닥색·물건)을 마지막으로 한 시각
        self.lost_hint = None
        self.worker = None

    # ------------------------------------------------------------ 받기
    def push(self, data):
        """휴대폰이 보낸 JPEG 한 장. 바로 돌아간다 (처리는 일꾼이)."""
        now = time.time()
        with self.cv:
            self.n_in += 1
            self.latest = (data, now, self.n_in)
            self.t_in.append(now)
            self.cv.notify()
            if self.worker is None or not self.worker.is_alive():
                self.worker = threading.Thread(target=self._loop, daemon=True)
                self.worker.start()
        return self.brief()

    def _loop(self):
        last = 0
        while True:
            with self.cv:
                while self.latest is None or self.latest[2] == last:
                    self.cv.wait(timeout=1.0)
                data, t_in, last = self.latest
            try:
                self._process(data)
            except Exception as e:
                self.msg = f'처리 오류: {type(e).__name__}: {e}'

    def _process(self, data):
        t0 = time.time()
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self.msg = '그림을 읽지 못했습니다'
            return
        h, w = img.shape[:2]
        if self.wh != (w, h):                       # 휴대폰을 돌리면 가로세로가 바뀐다 -> 처음부터
            self.wh = (w, h)
            self.board = Board()
            self.board_id += 1                      # 판 칸 번호가 완전히 새로 매겨진다
            self.fs.clear()
            self.K = None
        self._keep_sample(data, img)
        if self.rec_dir:
            try:
                with open(os.path.join(self.rec_dir, f'{self.n_done:05d}.jpg'), 'wb') as f:
                    f.write(data)
            except Exception:
                pass
        snap = (dict(self.board.holes), {k: dict(v) for k, v in self.board.color.items()})
        # ★ 빠른 길 / 느린 길 (10-26) ★ 매 장은 격자 추적 + 카메라 위치만. 판 무늬·바닥색·물건·도킹 판정은
        #   SLOW_S 마다 한 번 (카메라가 빨리 움직이지 않으니 충분). 다시 찾은 장은 늘 느린 길(판 확인 포함).
        slow = (t0 - self.t_slow >= SLOW_S) or self.board.H is None
        ok = self.board.add(img, colors=slow)
        # ★ 엉터리 격자는 실패로 (2026-10-22) ★
        #   너무 가까우면 구멍 몇 개로 '맞췄다' 고 나오는데(실측 23/46 맞음, 오차 13px), 그걸 믿으면
        #   다음 장부터 그 격자의 '구멍 크기 예상' 으로 찾아서, 정상 화면이 와도 영영 못 찾았다(멈춤).
        last = self.board.log[-1] if self.board.log else None

        def bad(lg):
            return bool(lg and len(lg) >= 4 and lg[2] and (lg[1] / lg[2] < GOOD_RATIO or lg[3] > GOOD_ERR_PX))
        if ok and bad(last) and str(last[0]).startswith('추적'):
            # 앞 장을 이어받은 추적이 엉터리면 같은 장에서 바로 처음부터 다시 찾는다 (다음 장까지 안 기다림)
            self.board.holes, self.board.color = snap
            self.board.H = None
            self.board.prev = None
            ok = self.board.add(img)
            last = self.board.log[-1] if self.board.log else None
            slow = True
        if ok and bad(last):
            ok = False
            self.board.holes, self.board.color = snap     # 엉터리 격자로 센 기록은 되돌린다
            self.board.H = None                            # 다음 장의 출발점으로도 쓰지 않는다
            self.board.prev = None
        if ok:
            self.fails = 0
        else:
            self.fails += 1
            hole_n = last[1] if last and len(last) >= 2 and last[0] == '구멍 부족' else None
            if self.fails >= RESET_AFTER:           # 예전 격자 정보를 버리고 처음부터 (구멍 크기도 다시 자동)
                self.board.H = None
                self.board.prev = None
            self.lost_hint = ('판이 안 보입니다 — 목심판 전체가 화면에 들어오게 비춰 주세요'
                              if hole_n is not None and hole_n < 12 else
                              '격자를 못 맞춤 — 너무 가깝거나 멀면 판 전체가 보이게 조금 움직여 주세요')
        if last and str(last[0]) == '재탐색':
            slow = True
            if ok:
                # 처음부터 다시 찾으면 칸 번호가 180° 돌아서 잡힐 수 있다 (2×4 체크무늬는 180° 돌려도 같아 보임)
                # -> 앱이 이전 자리(로봇·도킹)로 다시 맞추도록 이어짐 번호를 올린다
                self.cont += 1
                # ★ 쌓아 둔 증거(바닥색·가려진 구멍·도킹 방향)도 비운다 (10-26 실측) ★ 칸 번호가 돌았는데 예전
                #   증거가 예전 칸에 남아 엉뚱한 자리가 로봇으로 보였고, 앱 로봇이 잘못 옮겨졌다.
                self.fr_win = None
                self.T = None
                self.T_win = None
                self.hist.clear()
                # 판 칸 기록도 이번 장 것만 남긴다 (다시 찾기는 위치만 맞추고 방향은 못 맞춰 예전 번호와 섞일 수 있다)
                h0, c0 = snap
                self.board.holes = {k: v - h0.get(k, 0) for k, v in self.board.holes.items() if v - h0.get(k, 0) > 0}
                self.board.color = {k: {c: n - c0.get(k, {}).get(c, 0) for c, n in v.items()}
                                    for k, v in self.board.color.items()
                                    if sum(v.values()) - sum(c0.get(k, {}).values()) > 0}
                self.board.orient()                 # 이번 장 기록만으로 긴 쪽 방향을 다시 본다
        if self.board.turns != self._turns:
            # ★ 격자를 90° 돌려 '긴 쪽 = j' 로 맞췄다 (2×4 판, 11-03) ★ 칸 번호가 바뀌었으니 다시 찾은 것과 같게:
            #   앱이 판을 다시 맞추고(cont), 예전 칸 번호로 쌓인 증거는 버린다.
            self._turns = self.board.turns
            self.cont += 1
            self.fr_win = None
            self.T = None
            self.T_win = None
            self.hist.clear()
            slow = True
        pose = None
        if ok and self.board.H is not None:
            H = self.board.H
            f = focal_from_H(H, w, h)
            L = max(w, h)                           # 휴대폰 카메라: 긴 변 화각 55~80° 만 받는다
            if f and L / 2 / math.tan(math.radians(40)) < f < L / 2 / math.tan(math.radians(27.5)):
                self.fs.append(f)
                self.f_mem[(w, h)] = float(np.median(self.fs))
            # 비스듬한 장면에선 계산이 흔들린다 (실측 517~3454px). 모이기 전엔 기억값 또는 기본 70°
            f_use = (float(np.median(self.fs)) if len(self.fs) >= 5 else
                     self.f_mem.get((w, h), L / 2 / math.tan(math.radians(35))))
            if True:
                self.K = K_of(f_use, w, h)
                i0, j0 = self.board.window()
                self.window = (i0, j0)
                C, Rwc = camera_pose(H, self.K, i0, j0)[:2]
                # ★ 판 격자 축이 거울상(위에서 볼 때 왼손 좌표)으로 잡히는 경우가 있다 (2026-10-22 실측) ★
                #   camera_pose 는 그때 z 만 뒤집어서, 카메라 보는 방향이 거꾸로 그려지고(주황 사각형)
                #   판 -> 앱 맞춤도 거울상이 됐다. 진짜 카메라->판 변환(Rt)으로 바로잡고, 밖으로 내보낼 때는
                #   판 y 를 뒤집어(b -> BJ-1-b) 늘 오른손 좌표로 맞춘다.
                mir = bool(np.linalg.det(Rwc) < 0)
                Rt = Rwc @ np.diag([1.0, -1.0, -1.0]) if mir else Rwc
                blue = self._floor_color(img, H, (i0, j0), C) if slow else 1.0   # 빠른 길: 판 확인은 느린 길에서
                # ★ 판이 아닌 네모 무늬(키보드 등) 와 말이 안 되는 카메라 위치는 버린다 ★
                if blue < MIN_BLUE_FRAC or not (0.05 < C[2] < 3.0) or np.linalg.norm(C) > 3.0:
                    self.fails += 1
                    self.lost_hint = '판이 아닌 것 같습니다 — 파랑·흰 목심판이 보이게 비춰 주세요'
                else:
                    if mir != self.mirror:
                        self.gen += 1                   # 거울상 여부가 바뀌어도 판 좌표가 바뀐다
                    self.mirror = mir
                    pose = ((np.array([C[0], SIDE_J - C[1], C[2]]), np.diag([1.0, -1.0, 1.0]) @ Rt) if mir
                            else (C, Rwc))
                    if slow:
                        self._objects(img, H, (i0, j0))
                        self._dock_yaw(img, C, Rt)
                        self.t_slow = t0
        with self.lock:
            self.n_done += 1
            if pose is not None:
                self.n_ok += 1
                self.pose = pose
                self.t_pose = time.time()
                C, Rwc = pose
                self.pose_s = (self.f_pos(C, self.t_pose),
                               _orth(self.f_rot(np.asarray(Rwc).ravel(), self.t_pose).reshape(3, 3)))
            self.jpg = data
            self.t_proc.append(time.time() - t0)
            self.msg = ('격자 추적 중' if pose is not None else
                        (self.lost_hint or '판을 찾는 중 — 목심판이 화면에 크게 보이게 비춰 주세요'))

    def _keep_sample(self, data, img):
        """진단용 표본 저장: 못 찾는 중이면 3초마다, 찾는 중이면 10초마다 한 장 (최근 60장만)."""
        now = time.time()
        gap = 10.0 if self.fails == 0 and self.pose is not None else 3.0
        if now - self.t_sample < gap:
            return
        self.t_sample = now
        try:
            os.makedirs(SAMPLE_DIR, exist_ok=True)
            tag = 'ok' if self.fails == 0 and self.pose is not None else 'fail'
            fn = os.path.join(SAMPLE_DIR, time.strftime(f'%m%d_%H%M%S_{tag}.jpg'))
            with open(fn, 'wb') as f:
                f.write(data)
            old = sorted(glob.glob(os.path.join(SAMPLE_DIR, '*.jpg')))
            for o in old[:-60]:
                os.remove(o)
        except Exception:
            pass

    def _objects(self, img, H, win):
        """가려진 구멍으로 물건(로봇 받침·도킹)을 누적 판정한다. 판 창이 바뀌면 처음부터."""
        if self.T is None or self.T_win != win:
            tp = self.board.tile_phase()
            even = tp['짝수타일색']

            def board_color(i, j, tp=tp, even=even):      # 판 무늬 — 파란 타일 위 파랑은 물건 색으로 안 친다
                e = ((i - tp['p']) // TILE + (j - tp['q']) // TILE) % 2 == 0
                return even if e else ('W' if even == 'B' else 'B')
            self.T = Tracker(win[0], win[1], board_color, (tp['p'], tp['q']), shape=(BI, BJ))
            self.T_win = win
            self.hist.clear()
        holes = detect_holes(img, pitch_of(H, img.shape))
        self.T.update(img, H, holes)
        now = time.time()
        if now - self.t_snap >= SNAP_S:
            self.t_snap = now
            self.hist.append((now, self.T.objects(), self.T.free().copy(), self.T.occupied().copy()))

    def _floor_color(self, img, H, win, C):
        """목심 칸마다 원래 바닥색(파랑/흰)과 실제로 찍힌 색을 비교해 누적한다.
        반환: 화면 안 판 칸 중 '파란 바닥' 으로 보인 칸의 비율 (판인지 확인용)."""
        if self.fr_win != win:                  # 판 창이 바뀌면 처음부터
            self.fr[:] = 0; self.fr_n[:] = 0; self.fr_cs[:] = 0; self.fr_s[:] = 0; self.fr_win = win
            self.gen += 1                       # 판 번호가 새로 매겨졌다 -> 앱이 로봇 기준으로 다시 맞춘다
            self.yaw_fr.clear()
            self.ghosts.clear()
        i0, j0 = win
        tp = self.board.tile_phase(); even = tp['짝수타일색']
        h, w = img.shape[:2]
        I, J = np.meshgrid(np.arange(BI), np.arange(BJ), indexing='ij')
        G = np.stack([I + i0, J + j0], -1).reshape(-1, 1, 2).astype(np.float32)
        P = cv2.perspectiveTransform(G, H).reshape(BI, BJ, 2)
        r = max(2, int(0.22 * pitch_of(H, img.shape)))
        hsv = cv2.blur(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), (2 * r + 1, 2 * r + 1))
        x, y = P[..., 0].round().astype(int), P[..., 1].round().astype(int)
        inv = (x >= r) & (x < w - r) & (y >= r) & (y < h - r)
        xs, ys = np.clip(x, 0, w - 1), np.clip(y, 0, h - 1)
        hh, ss, vv = (hsv[ys, xs, k].astype(float) for k in range(3))
        e = (((I + i0 - tp['p']) // TILE + (J + j0 - tp['q']) // TILE) % 2) == 0
        tile_w = np.where(e, even == 'W', even != 'W')
        blue_floor = (~tile_w) & (hh > 95) & (hh < 130) & (ss > 60)
        # 흰 바닥 위: 색이 있으면(채도) 다른 것 — 구멍 그늘은 회색이라 색이 없다
        # 파란 바닥 위: 희거나 파랑이 아닌 색이면 다른 것 (파란 도킹은 파란 바닥 위에선 못 가른다)
        foreign = np.where(tile_w, ss > 80, ((ss < 60) & (vv > 150)) | ~((hh > 95) & (hh < 130)))
        # ★ 타일 가장자리 칸은 판정하지 않는다 (10-22 실측) ★ 퍼즐 이음새 톱니로 옆 타일 색이 들어와
        #   이음새를 따라 '다른 색' 줄이 생겼다. 이 칸은 '모름' — 도킹 다리 찾기에서 빈 다리로 봐준다.
        edge = (((I + i0 - tp['p']) % TILE == 0) | ((I + i0 - tp['p']) % TILE == TILE - 1) |
                ((J + j0 - tp['q']) % TILE == 0) | ((J + j0 - tp['q']) % TILE == TILE - 1))
        self.fr_edge = edge
        self.fr_bluetile = ~tile_w
        m = inv & ~edge
        self.fr[m] = (1 - FOREIGN_ALPHA) * self.fr[m] + FOREIGN_ALPHA * foreign[m]
        self.fr_n[m] += 1
        ang = hh * 2 * math.pi / 180.0
        fm = m & foreign
        self.fr_cs[fm, 0] = 0.7 * self.fr_cs[fm, 0] + 0.3 * np.cos(ang[fm])
        self.fr_cs[fm, 1] = 0.7 * self.fr_cs[fm, 1] + 0.3 * np.sin(ang[fm])
        self.fr_s[fm] = 0.7 * self.fr_s[fm] + 0.3 * ss[fm]
        self.cam_cells.append((C[0] / PITCH, C[1] / PITCH))
        n_blue_tiles = int(((~tile_w) & inv).sum())
        return float(blue_floor[inv].sum()) / max(1, int(inv.sum())) if n_blue_tiles else 0.0

    def _docks_by_color(self, rob, fit=True):
        """'바닥색과 꾸준히 다른' 칸에서 다리 4개(2칸 간격 정사각형)를 찾는다. 덩어리마다 카메라에 가장 가까운 것."""
        on = (self.fr >= FOREIGN_ON) & (self.fr_n >= FOREIGN_MIN_SEEN)
        on[0, :] = on[-1, :] = on[:, 0] = on[:, -1] = False      # 판 가장자리 한 줄은 책상·바닥이 비친다
        # 모름 = 타일 이음새 · 아직 덜 본 칸 · 파란 타일 칸(파란 다리는 파란 바닥에서 색으로 안 갈린다).
        # 다리 간격(40mm 정사각형)이 정해져 있으니 진짜 다른 색 다리 2개만 보여도 나머지 2개 자리를 안다.
        unk = self.fr_edge | (self.fr_n < FOREIGN_MIN_SEEN) | self.fr_bluetile
        unk[0, :] = unk[-1, :] = unk[:, 0] = unk[:, -1] = False
        sq = []
        for a in range(BI - LEG_STEP):
            for b in range(BJ - LEG_STEP):
                legs = [(a, b), (a + LEG_STEP, b), (a, b + LEG_STEP), (a + LEG_STEP, b + LEG_STEP)]
                if all(on[x, y] or unk[x, y] for x, y in legs) and sum(bool(on[x, y]) for x, y in legs) >= 2:
                    c = (a + 1.0, b + 1.0)
                    if rob is not None and math.dist(c, rob) < ROBOT_KEEPOUT:
                        continue
                    sq.append(c)
        if not sq:
            return []
        cam = np.median(np.array(self.cam_cells), 0) if self.cam_cells else np.array([BI / 2, -20.0])
        groups = []                               # 3칸 안끼리 한 도킹
        for c in sorted(sq, key=lambda c: math.dist(c, cam)):
            if all(math.dist(c, g) > 3.0 for g in groups):
                groups.append(c)
        out = []
        for c in groups:
            a, b = int(c[0] - 1), int(c[1] - 1)
            cells = [(a, b), (a + 2, b), (a, b + 2), (a + 2, b + 2)]
            if fit and self._is_ghost(c):          # 앞 도킹 윗판이 가린 바닥 — 도킹 아님
                continue
            fc = self._fit_center(c) if fit else None
            if fit and fc is None:                 # ★ 3D 모양이 맞아야만 도킹 (바닥 빛·그늘 가짜를 거른다) ★
                continue
            cs = np.mean([self.fr_cs[x, y] for x, y in cells if on[x, y]], 0)
            hue = (math.degrees(math.atan2(cs[1], cs[0])) % 360) / 2.0
            name = _hsv_name(hue, 200, 200)
            if name not in COLOR_HEX:
                continue
            if fc is not None:                     # 3D 모양 맞추기로 다듬은 자리 (다리가 한 칸 밀리는 것 바로잡기)
                c = (float(fc[0]), float(fc[1]))
                cells = [(fc[0] + x, fc[1] + y) for x in (-1, 1) for y in (-1, 1)]
            out.append({'color': name, 'hex': COLOR_HEX[name], 'n': 4, 'center': [c[0], c[1]], 'cells': cells,
                        'hue': hue, 'yaw': self._yaw_of(c) if fc is not None else None, 'fit': fc is not None})
        return out

    def _dock_yaw(self, img, C, Rt):
        """도킹 자리·방향을 3D 모양 맞추기로 다듬는다 (2026-10-22).
        ① 자리: 바닥색 차이로 찾은 다리 4개는 비스듬히 보면 한 칸쯤 밀린다 (다리가 뒤쪽 칸을 가린다).
           그 근처 ±2칸에 진짜 도킹 모양(다리 4개 + 윗판)을 세워 그려 보고, 도킹 색이 실루엣 안에
           가장 많고 바로 바깥에 가장 적은 자리를 고른다.
        ② 유령: 높이 있는 윗판이 뒤쪽 바닥을 가려 '도킹이 하나 더' 처럼 보인다. 다리 자리가
           카메라에 더 가까운 도킹의 실루엣 안에 들어가면 그 도킹이 가린 것 -> 버린다.
        ③ 방향: 윗판 U 홈이 열린 쪽. 홈 자리로는 바닥·그늘이 보여 도킹 색이 적다.
           또렷한 장(1·2등 차이 YAW_MARGIN 이상)만 한 표씩, 표가 쌓여야 정한다.
        Rt = 진짜 카메라 -> 판 회전 (판 격자 원 좌표)."""
        if self.K is None:
            return
        Rc = np.asarray(Rt)
        K = self.K
        h, w = img.shape[:2]

        def proj(X):
            Xc = (np.asarray(X, float) - C) @ Rc
            if (Xc[:, 2] <= 0.05).any():
                return None
            u = Xc @ K.T
            return u[:, :2] / u[:, 2:3]

        def parts(ca, cb):                       # 다리 4개 + 윗판 상자의 꼭짓점 (판 좌표 m)
            legs = [[(ca + lx + ex, cb + ly + ey, z) for ex in (-.005, .005) for ey in (-.005, .005)
                     for z in (0.0, DOCK_TOP_Z - 0.010)] for lx in (-.02, .02) for ly in (-.02, .02)]
            top = [(ca + ex, cb + ey, z) for ex in (-.027, .027) for ey in (-.025, .025)
                   for z in (DOCK_TOP_Z - 0.010, DOCK_TOP_Z)]
            return legs + [top]
        self.yaw_need = False
        seen, ghosts, hulls = set(), [], []      # hulls: 앞서 받아들인 도킹들의 실루엣 (화면 좌표)
        for d in self._docks_by_color(None, fit=False):          # 카메라에 가까운 것부터
            a0, b0 = d['center']
            P = proj(np.array([(x, y, z) for x in (-0.07, 0.07) for y in (-0.07, 0.07) for z in (0.0, DOCK_TOP_Z)])
                     + [a0 * PITCH, b0 * PITCH, 0])
            if P is None:
                continue
            x0, y0 = np.maximum(np.floor(P.min(0)).astype(int) - 6, 0)    # 화면 밖은 잘라내고 보이는 만큼만
            x1, y1 = np.minimum(np.ceil(P.max(0)).astype(int) + 6, [w, h])
            mid = proj([(a0 * PITCH, b0 * PITCH, 0.0), (a0 * PITCH, b0 * PITCH, DOCK_TOP_Z)])
            if (x1 - x0) * (y1 - y0) < 400 or mid is None or not all(
                    6 <= x < w - 6 and 6 <= y < h - 6 for x, y in mid):
                continue                                          # 도킹 가운데(바닥·윗면)가 화면 밖이면 판정하지 않는다
            hsv = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2HSV).astype(int)
            dh = np.abs(hsv[..., 0] - d['hue'])
            dh = np.minimum(dh, 180 - dh)
            # 그늘(어두운 곳)은 도킹 색으로 안 친다 — 홈 안쪽은 그늘이 진다
            mask = ((dh < 14) & (hsv[..., 1] > 70) & (hsv[..., 2] > 70)).astype(np.float32)
            ring_px = max(3, int(K[0, 0] * 0.008 / max(0.15, np.linalg.norm(C - [a0 * PITCH, b0 * PITCH, 0]))))
            best = None
            for da in range(-2, 3):
                for db in range(-2, 3):
                    ca, cb = (a0 + da) * PITCH, (b0 + db) * PITCH
                    hs = []
                    for X in parts(ca, cb):
                        Q = proj(X)
                        if Q is None:
                            break
                        hs.append(cv2.convexHull(np.float32(Q)))
                    if len(hs) < 5:
                        continue
                    sil = np.zeros(mask.shape, np.uint8)
                    for hl in hs:
                        cv2.fillConvexPoly(sil, np.int32(np.round(hl - [x0, y0])), 1)
                    ring = cv2.dilate(sil, np.ones((2 * ring_px + 1, 2 * ring_px + 1), np.uint8)) - sil
                    if sil.sum() < 30 or ring.sum() < 30:
                        continue
                    sc = float(mask[sil > 0].mean() - mask[ring > 0].mean())
                    if best is None or sc > best[0]:
                        best = (sc, (int(round(a0 + da)), int(round(b0 + db))), hs)
            if best is None:
                continue
            if d['hue'] < 25 or d['hue'] >= 158:                  # 빨강·주황 = 손(피부)과 헷갈린다
                sil = np.zeros(mask.shape, np.uint8)
                for hl in best[2]:
                    cv2.fillConvexPoly(sil, np.int32(np.round(hl - [x0, y0])), 1)
                sat = hsv[..., 1][(sil > 0) & (mask > 0)]
                if len(sat) and float(np.median(sat)) < SKIN_SAT:
                    continue                                      # 피부: 채도가 낮다 (실측 중앙 102)
            key = best[1]
            ca, cb = key[0] * PITCH, key[1] * PITCH
            feet = proj([(ca + lx, cb + ly, 0.0) for lx in (-.02, .02) for ly in (-.02, .02)])
            if feet is not None and hulls and sum(
                    any(cv2.pointPolygonTest(hl, (float(x), float(y)), False) >= 0 for hl in hulls)
                    for x, y in feet) >= 3:
                ghosts.append(d['center'])                        # 앞 도킹이 가린 자리
                continue
            hulls.extend(best[2])
            seen.add(key)
            v = self.yaw_fr.get(key)
            if v is None:
                v = self.yaw_fr[key] = [np.zeros(4), 0, 0.0, 0]  # [방향 표, 내려다본 장 수, 자리 점수 평균, 자리 본 수]
            v[2] = (1 - YAW_ALPHA) * v[2] + YAW_ALPHA * max(0.0, best[0])
            v[3] += 1
            dxy = math.hypot(C[0] - ca, C[1] - cb)
            elev = math.degrees(math.atan2(C[2] - DOCK_TOP_Z, dxy))
            size = K[0, 0] * 0.055 / max(0.05, math.hypot(dxy, C[2] - DOCK_TOP_Z))
            if elev < YAW_MIN_ELEV or size < YAW_MIN_PX:
                self.yaw_need = True                              # 방향은 아직 — 위에서 비춰 달라고 안내
                continue
            v[1] += 1
            fr = []
            for k in range(4):                                    # U 홈 4방향
                an = math.radians(90 * k)
                R2 = np.array([[math.cos(an), -math.sin(an)], [math.sin(an), math.cos(an)]])
                Q = proj(np.c_[NOTCH @ R2.T + [ca, cb], np.full(len(NOTCH), DOCK_TOP_Z)])
                m = np.zeros(mask.shape, np.uint8)
                cv2.fillPoly(m, [np.int32(np.round(Q - [x0, y0])).reshape(-1, 1, 2)], 1)
                fr.append(float(mask[m > 0].mean()) if m.sum() >= 8 else float('nan'))
            if not any(math.isnan(f) for f in fr):
                o = np.argsort(fr)
                if fr[o[1]] - fr[o[0]] >= YAW_MARGIN:             # 또렷한 장만 한 표
                    v[0] *= 0.9                                   # 옛 표는 서서히 잊는다 (돌려 꽂으면 따라가게)
                    v[0][o[0]] += 1.0
        for key in [k for k in self.yaw_fr if k not in seen]:   # 안 보인 자리는 서서히 잊는다
            v = self.yaw_fr[key]
            v[2] *= (1 - YAW_ALPHA)
            v[3] -= 1
            if v[3] <= 0:
                del self.yaw_fr[key]
        self.ghosts.append(ghosts)

    def _fit_center(self, c):
        """바닥색으로 찾은 가운데 c 근처(±2칸)에서 모양 맞추기 점수가 가장 높은 자리. 없으면 None."""
        cand = [(v[2], k) for k, v in self.yaw_fr.items()
                if abs(k[0] - c[0]) <= 2.5 and abs(k[1] - c[1]) <= 2.5 and v[3] >= YAW_MIN_N]
        if not cand:
            return None
        sc, k = max(cand)
        return k if sc >= FIT_MIN else None

    def _is_ghost(self, c):
        """최근 몇 장에서 '앞 도킹이 가린 자리' 로 판정된 적이 더 많으면 True."""
        n = sum(any(math.dist(c, g) <= 2.5 for g in gs) for gs in self.ghosts)
        return n >= max(2, len(self.ghosts) // 2)

    def _yaw_of(self, center):
        """도킹 가운데 칸 -> 홈 방향 {'k','dir'(판 좌표 단위벡터),'conf'} 또는 None (아직 모름).
        또렷한 장의 표가 YAW_VOTES 이상이고 2등의 두 배 이상일 때만."""
        v = self.yaw_fr.get((int(round(center[0])), int(round(center[1]))))
        if v is None:
            return None
        o = np.argsort(-v[0])
        if v[0][o[0]] < YAW_VOTES or v[0][o[0]] < 2 * v[0][o[1]]:
            return None
        k = int(o[0]); a = math.radians(90 * k)
        return {'k': k, 'dir': [round(math.sin(a), 6), round(-math.cos(a), 6)],
                'conf': round(float(v[0][o[0]] / (v[0].sum() + 1e-9)), 2)}

    def _robot_by_color(self):
        """바닥색으로 로봇 받침 찾기 — 흰·회색(채도 낮음)이 꾸준히 보이는 가장 큰 덩어리의 카메라 쪽 부분.
        키 큰 로봇은 뒤쪽 칸을 가려 덩어리가 카메라 반대쪽으로 길게 늘어지므로 가까운 쪽 40% 만 쓴다.
        판 맞춤은 200mm 타일 단위로 끌어당기므로 받침이 '어느 타일인지' 만 맞으면 된다."""
        on = (self.fr >= FOREIGN_ON) & (self.fr_n >= FOREIGN_MIN_SEEN) & (self.fr_s < ROBOT_MAX_SAT)
        if on.sum() < ROBOT_MIN_CELLS:
            return None
        from scipy import ndimage
        comp, n = ndimage.label(ndimage.binary_dilation(on, np.ones((3, 3))) , structure=np.ones((3, 3)))
        best = None
        for k in range(1, n + 1):
            cells = np.argwhere((comp == k) & on)
            if len(cells) >= ROBOT_MIN_CELLS and (best is None or len(cells) > len(best)):
                best = cells
        if best is None:
            return None
        cam = np.median(np.array(self.cam_cells), 0) if self.cam_cells else np.array([BI / 2, -20.0])
        dist = np.linalg.norm(best - cam, axis=1)
        near = best[dist <= np.percentile(dist, 40)]
        return [float(v) for v in near.mean(0)]

    def _robot_floor_ok(self, b):
        """로봇 받침 후보 자리 둘레(3×3 칸)에 '바닥색과 다름' 이 뚜렷한가."""
        a0, b0 = int(round(b[0])), int(round(b[1]))
        sl = self.fr[max(0, a0 - 1):a0 + 2, max(0, b0 - 1):b0 + 2]
        return sl.size > 0 and float(sl.max()) >= ROBOT_MIN_FR

    def tiles_seen(self):
        """판 창(20×40) 안에서 실제로 본 타일 (u,v) — 구멍을 30개 이상 본 타일. u = 0..1 (짧은 쪽), v = 0..3 (긴 쪽)."""
        if not self.window:
            return []
        i0, j0 = self.window
        out = []
        for u in range(BI // TILE):
            for v in range(BJ // TILE):
                n = sum(1 for (a, b) in self.board.holes
                        if i0 + TILE * u <= a < i0 + TILE * u + TILE and j0 + TILE * v <= b < j0 + TILE * v + TILE)
                if n >= 30:
                    out.append((u, v))
        return out

    def layout(self):
        """최근 PERSIST 번의 판정에 계속 나온 것만 (판 창 좌표, 칸 단위).
        반환: {'robot':[a,b]|None, 'dockings':[{color,hex,center,cells}], 'free':BI×BJ bool, 'window', 'tiles', 'size':(BI,BJ)}"""
        with self.lock:
            hist = list(self.hist)
        if len(hist) < PERSIST:
            return None
        recent = hist[-PERSIST:]
        objs, free = recent[-1][1], recent[-1][2]

        def steady(o):
            key = 'base' if o['kind'] == 'robot' else 'center'
            tol = 5 if o['kind'] == 'robot' else 3
            for _, old, _f, _o in recent[:-1]:
                if not any(p['kind'] == o['kind'] and p['color'] == o['color'] and
                           math.dist(p.get(key, p['center']), o.get(key, o['center'])) <= tol for p in old):
                    return False
            return True
        objs = [o for o in objs if steady(o)]
        rob = None
        # ★ 로봇 받침 자리는 바닥색도 달라야 한다 (2026-10-22 실측) ★ — 흰 로봇이 파란 판 위에 선다.
        #   휴대폰 바로 앞 판이 흔들려 흐리면 구멍이 안 보여 '가려진 덩어리' 로 잡혔다 (바닥색 차이 0.0,
        #   진짜 받침 0.57~1.0). 그걸 받침으로 믿어 판 맞춤이 180° 뒤집히고 도킹이 로봇 코앞에 놓였다.
        bases = [x['base'] for _, oo, _f, _o in hist for x in oo
                 if x['kind'] == 'robot' and 'base' in x and self._robot_floor_ok(x['base'])]
        rob = self._robot_by_color()          # ① 바닥색 (흰 받침이 파란 판 위)
        if rob is None and bases and any(o['kind'] == 'robot' and self._robot_floor_ok(o.get('base', o['center']))
                                         for o in objs):    # ② 가려진 구멍 덩어리 (바닥색도 달라야)
            best = max(([b for b in bases if math.dist(b, c) <= 3] for c in bases), key=len)
            rob = [float(v) for v in np.mean(best, 0)]
        # 도킹 = 바닥색과 꾸준히 다른 칸의 다리 4개 무늬 (사용자 제안: 바닥 색 차이로)
        docks = self._docks_by_color(rob)
        tiles = self.tiles_seen()
        if self.mirror:                               # 늘 오른손 좌표로 내보낸다 (위 _process 설명)
            m = BJ - 1
            rob = None if rob is None else [rob[0], m - rob[1]]
            free = None if free is None else np.asarray(free)[:, ::-1].copy()
            tiles = [(u, BJ // TILE - 1 - v) for u, v in tiles]
            for d in docks:
                d['center'] = [d['center'][0], m - d['center'][1]]
                d['cells'] = [(a, m - b) for a, b in d['cells']]
                if d['yaw']:
                    d['yaw'] = dict(d['yaw'], dir=[d['yaw']['dir'][0], -d['yaw']['dir'][1]])
        return {'robot': rob, 'free': free, 'window': self.window, 'tiles': tiles,
                'dockings': docks, 'mirror': self.mirror, 'gen': self.gen,
                'win': tuple(self.window) if self.window else None, 'board_id': self.board_id, 'cont': self.cont,
                'size': (BI, BJ)}

    # ------------------------------------------------------------ 상태
    def live(self):
        return bool(self.t_in) and time.time() - self.t_in[-1] < LIVE_TIMEOUT

    def fps(self):
        t = list(self.t_in)
        return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0.0

    def brief(self):
        """휴대폰 화면에 보일 짧은 상태."""
        with self.lock:
            p = self.pose if time.time() - self.t_pose < 3.0 else None   # 휴대폰 표시: 처리 중 잠깐 늦어도 노랑으로 깜빡이지 않게
            ms = 1000 * float(np.mean(self.t_proc)) if self.t_proc else 0.0
            out = {'ok': True, 'n': self.n_in, 'fps': round(self.fps(), 1), 'ms': round(ms),
                   'tracking': p is not None, 'msg': self.msg,
                   'f': round(float(np.median(self.fs))) if self.fs else None}
            if p is not None:
                out['cam_mm'] = [round(float(v) * 1000) for v in p[0]]
                if self.yaw_need and any(v[0].max() < YAW_VOTES for v in self.yaw_fr.values()):
                    out['hint'] = '도킹 방향을 보려면 도킹 위에서 내려다보듯 가까이 비춰 주세요'
            return out

    def summary(self):
        b = self.brief()
        if not self.live():
            return '📱 휴대폰 연결 안 됨 — 휴대폰에서 카메라 페이지를 여세요'
        s = f"📱 {b['fps']}fps · 처리 {b['ms']}ms · {b['msg']}" + (' · 🔴 녹화 중' if self.rec_dir else '')
        if b.get('cam_mm'):
            s += f" · 카메라 높이 {b['cam_mm'][2]}mm"
        if b.get('hint'):
            s += ' · ' + b['hint']
        return s

    def _to_world(self, origin, ztop):
        if self.reg is not None:                    # 로봇 받침으로 자동 맞춘 값
            return self.reg
        a = self.rot * math.pi / 2
        R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
        c_b = np.array([SIDE_I / 2, SIDE_J / 2, 0.0])
        ext = np.abs(R[:2, :2]) @ np.array([SIDE_I, SIDE_J])      # 돌린 판이 앱에서 차지하는 가로·세로
        c_w = np.array([origin[0] + ext[0] / 2 + self.shift[0] * PITCH,
                        origin[1] + ext[1] / 2 + self.shift[1] * PITCH, ztop])
        return R, c_w - R @ c_b

    def state(self, origin, ztop):
        """녹화 영상용 CamSync.state() 와 같은 모양 (뷰어가 그대로 그린다)."""
        if not self.live():
            return {'on': False, 'status': self.summary()}
        with self.lock:
            R, t = self._to_world(origin, ztop)
            w, h = self.wh or (16, 9)
            out = {'on': True, 'live': True, 'k': self.n_done, 'n': 0, 'fp': self.fp, 'view': self.view,
                   'auto': False, 'status': self.summary(), 'aspect': w / h,
                   'fovy': (math.degrees(2 * math.atan(h / 2 / self.K[1, 1])) if self.K is not None else 50.0),
                   'center': [float(v) for v in (R @ np.array([SIDE_I / 2, SIDE_J / 2, 0.0]) + t)],
                   'corners': [[float(v) for v in (R @ np.array([x, y, 0.0]) + t)]
                               for x, y in ((-PITCH / 2, -PITCH / 2), (SIDE_I + PITCH / 2, -PITCH / 2),
                                            (SIDE_I + PITCH / 2, SIDE_J + PITCH / 2), (-PITCH / 2, SIDE_J + PITCH / 2))],
                   'tiles': 0, 'objs': []}
            raw = self.fp and self.view == 'both'          # 영상과 겹칠 때는 영상과 같이 흔들려야 맞는다
            p = (self.pose if (raw or not self.smooth) else self.pose_s) if time.time() - self.t_pose < 1.5 else None
            if p is None:
                out['pos'] = None
            else:
                C, Rwc = p
                Cw = R @ C + t
                Rw = R @ Rwc
                dd = Cw - np.array(out['center'])
                out.update(pos=[float(v) for v in Cw], R=[float(v) for v in Rw.ravel()],
                           theta=float(math.degrees(math.asin(dd[2] / np.linalg.norm(dd)))),
                           phi=float(math.degrees(math.atan2(dd[1], dd[0]))), dist=float(np.linalg.norm(dd)))
            return out

    def ctl(self, cmd):
        """화면 버튼: 판 맞추기(rot, x±, y±), 1인칭(fp), 보기(view), 처음부터(reset)."""
        with self.lock:
            if cmd in ('rot', 'x+', 'x-', 'y+', 'y-'):
                self.reg = None                     # 손으로 맞추면 자동 맞춤은 끈다
            if cmd == 'rot':
                self.rot = (self.rot + 1) % 4
            elif cmd in ('x+', 'x-', 'y+', 'y-'):
                k = 0 if cmd[0] == 'x' else 1
                self.shift[k] += 1 if cmd[1] == '+' else -1
            elif cmd == 'fp':
                self.fp = not self.fp
            elif cmd == 'view':
                self.view = {'both': 'twin', 'twin': 'video', 'video': 'both'}[self.view]
            elif cmd == 'rec':
                if self.rec_dir:
                    self.rec_dir = None
                else:
                    self.rec_dir = os.path.join(SAMPLE_DIR, time.strftime('rec_%m%d_%H%M%S'))
                    os.makedirs(self.rec_dir, exist_ok=True)
            elif cmd == 'smooth':
                self.smooth = not self.smooth
            elif cmd == 'reset':
                self.reg = None
                self.T = None
                self.hist.clear()
                self.board = Board()
                self.board_id += 1
                self.fs.clear()
                self.K = None
                self.pose = None
                self.wh = None
        return (f"판 맞추기: 회전 {self.rot * 90}° · 이동 x{self.shift[0]:+d} y{self.shift[1]:+d}칸"
                f" · 1인칭 {'켬' if self.fp else '끔'} · 손떨림 거르기 {'켬' if self.smooth else '끔'}"
                + (f" · 🔴 녹화 중 ({os.path.basename(self.rec_dir)})" if self.rec_dir else '') + " | " + self.summary())

    def jpeg(self, k=None):
        with self.lock:
            return self.jpg


LIVE = LiveCam()
