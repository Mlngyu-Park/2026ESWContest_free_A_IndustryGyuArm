"""목심 격자 인식 — 사진 한 장에서, 그리고 움직이는 영상에서.

바탕 사실 (바뀌지 않는 것)
  목심 간격 20 mm · 타일 200 mm = 10×10 칸 · 판 2×4 타일 = 20×40 칸 (11-03, 예전 3×3 = 30×30)
  타일 색은 파랑·흰색이 번갈아 (체크무늬)

좌표
  H : 격자좌표 (i, j) -> 픽셀. 구멍 하나 = 정수 (i, j).
  i, j 의 '0' 은 처음엔 임의다. 판 끝이 보이면 확정된다.

처음 만든 곳: 2026-09-23 대화 (/tmp/fit*.py). 재부팅으로 사라져 여기로 옮겼다.
"""
import cv2, numpy as np
from scipy.spatial import cKDTree

TILE = 10          # 타일 한 변 칸 수
# ★ 판 = 타일 2×4 (사용자 11-03 — 앱 기본 바닥판 PLATE_LAYOUT 과 같다) ★ 예전엔 3×3 정사각형(30×30 칸)이었다.
#   격자 (i, j) 는 늘 '짧은 쪽 = i (20칸) · 긴 쪽 = j (40칸)' 로 맞춘다 (Board.orient).
#   2×4 체크무늬는 180° 돌려도 똑같아 보인다 → 그 방향은 앱이 로봇·도킹 자리로 가른다.
BOARD_TILES = (2, 4)
BI = BOARD_TILES[0] * TILE   # 판 짧은 쪽 칸 수 (i)
BJ = BOARD_TILES[1] * TILE   # 판 긴 쪽 칸 수 (j)
ORIENT_MARGIN = 1.1          # 반대 방향(긴 쪽 = i) 창이 이 배 넘게 구멍이 많아야 격자를 90° 돌린다 (반만 보일 땐 그대로)
TOL = 0.30         # 격자점과 이 칸 수 안이면 '맞음'


# ---------------------------------------------------------------- 한 장
def _blobs(img):
    """파랑·흰 판 위의 어두운 덩어리: (중심, 면적) — 크기로는 아직 안 거른다."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 파랑(H 96~129, S>80, V>50) 또는 흰(S<45, V>140) — numpy 비교 대신 inRange (같은 결과, 11ms -> 1ms)
    mat = cv2.bitwise_or(cv2.inRange(hsv, (96, 81, 51), (129, 255, 255)),
                         cv2.inRange(hsv, (0, 0, 141), (255, 44, 255)))
    mat = cv2.morphologyEx(mat, cv2.MORPH_CLOSE,
                           np.ones((9, 9), np.uint8), iterations=3)
    dark = cv2.subtract(cv2.GaussianBlur(gray, (0, 0), 9), gray)
    dark = cv2.bitwise_and(dark, dark, mask=mat)
    _, th = cv2.threshold(dark, 10, 255, cv2.THRESH_BINARY)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, s, c = cv2.connectedComponentsWithStats(th, 8)
    w, h, a = s[1:, 2], s[1:, 3], s[1:, 4]
    if SHAPE_MAX_ASPECT <= 2.0:                     # 녹화 영상용 (원래 기준): 동그란 것만
        ok = (w / np.maximum(h, 1) > 0.5) & (w / np.maximum(h, 1) < 2.0) & (a >= 12) & (np.maximum(w, h) ** 2 < 3 * a)
    else:                                           # 휴대폰(비스듬히): 납작한 타원도 구멍 — 기울어짐 정보다
        r = w / np.maximum(h, 1)
        ok = ((r > 1 / SHAPE_MAX_ASPECT) & (r < SHAPE_MAX_ASPECT) & (a >= SHAPE_MIN_AREA)
              & (a >= SHAPE_MIN_FILL * w * h))      # 타원은 상자의 약 0.79 를 채운다 (기운 타원은 덜)
    return c[1:][ok], a[ok]


# 구멍 모양 기준. 기본값 = 녹화 영상용 원래 기준. livecam(휴대폰) 이 넓혀 쓴다.
CELL_OFS = 0.5               # 칸 색을 읽는 자리 (구멍 i 에서 +이만큼). 0.5 = 구멍 사이 한가운데 → 타일 마지막 줄은
                             #   판 이음새 위라 판 무늬 위치가 1칸 어긋났다(10-26 실측). 휴대폰: 0.3 (구멍 반지름 ~0.18 칸 밖, 타일 안쪽)
CHECK_MIN = 0.0              # >0 이면 다시 찾은 격자로 칸 색이 체크무늬로 이만큼 안 맞으면 버린다 (휴대폰: 0.8)
RANSAC_ITERS = 2000          # grow 의 RANSAC 반복 한도 (휴대폰: 300 — 대응쌍이 대부분 맞아서 충분, 재탐색 3.6초 -> 짧게)
LOCAL_AREA = False           # True(휴대폰): 추적 중 구멍 크기를 자리마다 예상 (detect_holes 의 H)
SHAPE_MAX_ASPECT = 2.0       # 가로/세로 비 한계 (비스듬히 보면 구멍이 납작해진다)
SHAPE_MIN_FILL = 0.40        # 덩어리가 상자를 이만큼은 채워야 (선·틈 제외)
SHAPE_MIN_AREA = 8           # 이보다 작으면 잡티


# 큰 구멍 면적 / (격자 간격 px)^2 — 사진에서 잰 비율 (아래 _calib 로 확인)
AREA_PER_PITCH2 = 0.107      # 스크린샷 2026-09-23 에서 실측 (445개 중앙값)


def detect_holes(img, pitch_px=None, H=None):
    """큰 구멍(목심 자리) 중심들.
    H(예상 격자)를 주면 덩어리 자리마다 그 자리의 칸 넓이로 예상 크기를 따로 잡는다 (비스듬히 볼 때).
    pitch_px 를 알면(앞 프레임) 그 크기에 맞는 것만, 모르면 면적 분포를 둘로 나눠 큰 무리만."""
    c, a = _blobs(img)
    if len(a) == 0: return np.zeros((0, 2))
    if H is not None and AREA_PER_PITCH2:
        # ★ 비스듬히 보면 가까운 구멍은 크고 먼 구멍은 작다 (10-22 휴대폰 실측) ★ 화면 가운데 크기 하나로
        #   거르면 가까운 물빠짐 작은 구멍이 들어오고 먼 큰 구멍이 빠져 맞음 비율이 55% 로 떨어졌다.
        ij = _ij(H, c)
        q1 = cv2.perspectiveTransform((ij + [1, 0]).reshape(-1, 1, 2).astype(np.float32), H).reshape(-1, 2)
        q2 = cv2.perspectiveTransform((ij + [0, 1]).reshape(-1, 1, 2).astype(np.float32), H).reshape(-1, 2)
        u, v = q1 - c, q2 - c
        cell = np.abs(u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0])           # 그 자리 칸 하나의 화면 넓이
        m = AREA_PER_PITCH2 * cell
        keep = (a > 0.35 * m) & (a < 2.8 * m)
        return c[keep].astype(np.float64)
    if pitch_px is not None and AREA_PER_PITCH2:
        m = AREA_PER_PITCH2 * pitch_px ** 2
        keep = (a > 0.35 * m) & (a < 2.8 * m)
    else:
        la = np.log(a.astype(np.float64))
        best, cut = -1, np.median(la)
        for t in np.linspace(la.min(), la.max(), 60)[1:-1]:        # 오츠: 두 무리로 가장 잘 나뉘는 곳
            g1, g2 = la[la < t], la[la >= t]
            if len(g1) < 5 or len(g2) < 5: continue
            v = len(g1) * len(g2) * (g1.mean() - g2.mean()) ** 2
            if v > best: best, cut = v, t
        keep = la >= cut
    return c[keep].astype(np.float64)


def pitch_of(H, img_shape):
    """화면 가운데에서 격자 한 칸이 몇 px 인가."""
    h, w = img_shape[:2]
    ij = cv2.perspectiveTransform(np.float32([[[w / 2, h / 2]]]), np.linalg.inv(H))[0, 0]
    q = cv2.perspectiveTransform(np.float32([[ij, ij + [1, 0], ij + [0, 1]]]), H)[0]
    return float((np.hypot(*(q[1] - q[0])) + np.hypot(*(q[2] - q[0]))) / 2)


def _ij(H, pts):
    Hi = np.linalg.inv(H)
    return cv2.perspectiveTransform(pts.reshape(-1, 1, 2).astype(np.float32), Hi).reshape(-1, 2)


def grow(H, pts, iters=6):
    """H 로 예측한 격자점에 실제 구멍이 있으면 대응쌍으로 넣고 H 를 다시 푼다."""
    for _ in range(iters):
        ij = _ij(H, pts); r = np.round(ij)
        sel = np.hypot(*(ij - r).T) < TOL
        if sel.sum() < 12: return None
        H2, _ = cv2.findHomography(r[sel].astype(np.float32), pts[sel].astype(np.float32),
                                   cv2.RANSAC, 2.5, maxIters=RANSAC_ITERS)
        if H2 is None: return None
        H = H2
    return H


def score(H, pts):
    """(맞는 구멍 수, 재투영 오차 중앙값 px)"""
    ij = _ij(H, pts); r = np.round(ij)
    ok = np.hypot(*(ij - r).T) < TOL
    if ok.sum() == 0: return 0, 1e9
    pr = cv2.perspectiveTransform(r[ok].reshape(-1, 1, 2).astype(np.float32), H).reshape(-1, 2)
    return int(ok.sum()), float(np.median(np.hypot(*(pr - pts[ok]).T)))


def _basis(pts):
    """이웃 벡터에서 격자의 두 기저벡터."""
    t = cKDTree(pts); _, idx = t.query(pts, k=6)
    med = np.median(t.query(pts, k=2)[0][:, 1])
    vecs = np.array([pts[j] - pts[i] for i in range(len(pts)) for j in idx[i][1:]
                     if 0.7 * med < np.hypot(*(pts[j] - pts[i])) < 1.4 * med])
    ang = np.degrees(np.arctan2(vecs[:, 1], vecs[:, 0])) % 180
    hist, _ = np.histogram(ang, bins=180, range=(0, 180))
    hs = cv2.GaussianBlur(hist.astype(np.float32).reshape(-1, 1), (0, 0), 2).ravel()
    a1 = int(np.argmax(hs))
    band = [(a1 + 90 + k) % 180 for k in range(-12, 13)]
    a2 = band[int(np.argmax([hs[b] for b in band]))]
    def mv(a0):
        u = np.array([np.cos(np.radians(a0)), np.sin(np.radians(a0))])
        sel = vecs[np.abs(((ang - a0 + 90) % 180) - 90) < 7]
        return np.array([v if v @ u > 0 else -v for v in sel]).mean(0)
    return mv(a1), mv(a2)


def fit_full(pts, tries=60):
    """씨앗 없이 처음부터: 여러 구멍을 원점 후보로 두고 제일 많이 맞는 격자."""
    e1, e2 = _basis(pts)
    src = np.float32([[0, 0], [1, 0], [0, 1], [1, 1]])
    best, bn = None, 0
    order = np.random.default_rng(0).permutation(len(pts))[:tries]
    for k in order:
        o = pts[k]
        H0 = cv2.getPerspectiveTransform(src, np.float32([o, o + e1, o + e2, o + e1 + e2]))
        H = grow(H0, pts, 4)
        if H is not None:
            n, _ = score(H, pts)
            if n > bn: best, bn = H, n
            if FIT_EARLY and bn >= FIT_EARLY * len(pts):   # 충분히 좋은 답이면 나머지 후보는 안 본다
                break
    return best


FIT_EARLY = 0.0      # >0 이면 '찾은 구멍의 이 비율 이상이 맞는 격자' 가 나오는 즉시 멈춘다 (livecam: 0.75)


def cell_colors(img, H, lo, hi):
    """칸 (i,j) 가운데(i+0.5, j+0.5)의 색: 'B' 파랑 / 'W' 흰 / None.
    (한꺼번에 계산 — 칸마다 따로 하던 것과 같은 결과: 가운데 5×5 평균)"""
    h, w = img.shape[:2]
    if hi[0] - lo[0] > 4 * BJ or hi[1] - lo[1] > 4 * BJ:
        return {}                                   # 엉터리 격자 (칸 범위가 판의 몇 배) — 읽지 않는다
    I, J = np.meshgrid(np.arange(lo[0], hi[0]), np.arange(lo[1], hi[1]), indexing='ij')
    if I.size == 0: return {}
    G = np.stack([I + CELL_OFS, J + CELL_OFS], -1).reshape(-1, 1, 2).astype(np.float32)
    P = cv2.perspectiveTransform(G, H).reshape(-1, 2)
    x, y = P[:, 0].astype(int), P[:, 1].astype(int)
    ok = (x >= 3) & (x < w - 3) & (y >= 3) & (y < h - 3)
    if not ok.any(): return {}
    # 필요한 자리 둘레만 잘라서 평균 (화면 전체 HSV 를 만들지 않는다)
    x0, x1, y0, y1 = x[ok].min() - 2, x[ok].max() + 3, y[ok].min() - 2, y[ok].max() + 3
    hsv = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    m = cv2.boxFilter(hsv, cv2.CV_32F, (5, 5), normalize=True, borderType=cv2.BORDER_REPLICATE)
    v = m[y[ok] - y0, x[ok] - x0]
    Hh, S, V = v[:, 0], v[:, 1], v[:, 2]
    blue = (Hh > 95) & (Hh < 130) & (S > 80) & (V > 50)
    white = ~blue & (S < 45) & (V > 140)
    out = {}
    for i, j, b, wt in zip(I.ravel()[ok], J.ravel()[ok], blue, white):
        if b: out[(int(i), int(j))] = 'B'
        elif wt: out[(int(i), int(j))] = 'W'
    return out


def checker_agree(cols):
    """칸 색 {(i,j): 'B'|'W'} 이 10칸 타일 체크무늬와 가장 잘 맞을 때의 일치율 (0~1)."""
    if not cols: return 0.0
    P = np.array(list(cols.keys())); isb = np.array([c == 'B' for c in cols.values()])
    best = 0
    for p in range(TILE):
        for q in range(TILE):
            ev = ((P[:, 0] - p) // TILE + (P[:, 1] - q) // TILE) % 2 == 0
            a = int((ev == isb).sum()); best = max(best, a, len(isb) - a)
    return best / len(isb)


# ---------------------------------------------------------------- 영상
def frame_motion(prev, cur):
    """두 프레임 사이 화면 이동(호모그래피 prev->cur)을 '특징점' 으로 잡는다.

    ★ 격자 구멍만으로는 안 된다 ★ 구멍이 전부 똑같이 생겨서, 반 칸 넘게 움직이면
      옆 칸 구멍과 구별이 안 된다 (실측: 추적이 매 프레임 정확히 1칸씩 미끄러짐).
      로봇·도킹·물건·판 가장자리처럼 하나뿐인 모양으로 이동을 먼저 잡는다."""
    g0 = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY); g1 = cv2.cvtColor(cur, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(3000)
    k0, d0 = orb.detectAndCompute(g0, None); k1, d1 = orb.detectAndCompute(g1, None)
    if d0 is None or d1 is None: return None, 0
    m = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d0, d1, k=2)
    good = [a for a, b in (x for x in m if len(x) == 2) if a.distance < 0.75 * b.distance]
    if len(good) < 15: return None, len(good)
    p0 = np.float32([k0[g.queryIdx].pt for g in good]); p1 = np.float32([k1[g.trainIdx].pt for g in good])
    F, mask = cv2.findHomography(p0, p1, cv2.RANSAC, 3.0)
    return F, int(mask.sum()) if mask is not None else 0


MOTION_SCALE = 1.0           # 화면 이동 잡기용 축소 비율 (휴대폰: 0.5)
ORB_N = 3000                 # 특징점 수 (휴대폰: 1500)


def motion_feats(img):
    """화면 이동 잡기용 특징점 (축소 화면 기준 좌표, 기술자)."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if MOTION_SCALE != 1.0:
        g = cv2.resize(g, None, fx=MOTION_SCALE, fy=MOTION_SCALE, interpolation=cv2.INTER_AREA)
    k, d = cv2.ORB_create(ORB_N).detectAndCompute(g, None)
    return (np.float32([p.pt for p in k]) if k else np.zeros((0, 2), np.float32)), d


def motion_from_feats(f0, f1):
    """두 장의 특징점으로 화면 이동(원래 크기 기준 호모그래피). frame_motion 과 같은 판단."""
    (p0a, d0), (p1a, d1) = f0, f1
    if d0 is None or d1 is None: return None, 0
    m = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d0, d1, k=2)
    good = [a for a, b in (x for x in m if len(x) == 2) if a.distance < 0.75 * b.distance]
    if len(good) < 15: return None, len(good)
    p0 = p0a[[g.queryIdx for g in good]]; p1 = p1a[[g.trainIdx for g in good]]
    F, mask = cv2.findHomography(p0, p1, cv2.RANSAC, 3.0 * MOTION_SCALE)
    if F is None: return None, 0
    if MOTION_SCALE != 1.0:
        S = np.diag([MOTION_SCALE, MOTION_SCALE, 1.0])
        F = np.linalg.inv(S) @ F @ S
    return F, int(mask.sum()) if mask is not None else 0


class Board:
    """프레임을 이어 판(BI×BJ = 20×40 칸)을 채운다. 무한정 늘리지 않는다."""

    def __init__(self):
        self.prev = None           # 직전 프레임 (이동 예상용)
        self.H = None              # 직전 프레임의 격자 -> 픽셀
        self.holes = {}            # (i,j) -> 본 횟수
        self.color = {}            # (i,j) -> {'B':n,'W':n}
        self.log = []
        self.feat = None           # 직전 프레임 특징점 (매 장 두 번 뽑지 않게)
        self.feat_of = None        # self.feat 가 어느 프레임 것인가 (prev 와 같아야 쓴다)
        self.turns = 0             # 격자를 90° 돌려 '긴 쪽 = j' 로 맞춘 횟수 (바뀌면 칸 번호가 바뀐 것)

    def _offset_by_color(self, cols):
        """추적이 끊겼을 때: 새 프레임의 칸 색을 누적 판과 맞춰 정수 이동량을 찾는다.
        (배열로 한꺼번에 — 예전 이중 반복은 3~5초 걸렸다. 결과는 같다)"""
        if not cols or not self.color:
            return (0, 0)
        keys = np.array(list(self.color.keys())); lo = keys.min(0)
        A = np.zeros(tuple(keys.max(0) - lo + 1), np.int8)            # 누적 판: +1 파랑, -1 흰, 0 모름
        for (i, j), v in self.color.items():
            A[i - lo[0], j - lo[1]] = 1 if max(v, key=v.get) == 'B' else -1
        P = np.array(list(cols.keys())); c = np.array([1 if x == 'B' else -1 for x in cols.values()], np.int8)
        best, bs = (0, 0), -1
        for di in range(-BJ, BJ + 1):
            for dj in range(-BJ, BJ + 1):
                q = P + [di, dj] - lo
                inb = (q[:, 0] >= 0) & (q[:, 1] >= 0) & (q[:, 0] < A.shape[0]) & (q[:, 1] < A.shape[1])
                if inb.sum() < 20: continue
                a = A[q[inb, 0], q[inb, 1]]; known = a != 0
                n = int(known.sum())
                if n < 20: continue
                agree = int((a[known] == c[inb][known]).sum())
                if agree - (n - agree) > bs: best, bs = (di, dj), agree - (n - agree)
        return best

    def _motion(self, img):
        """직전 프레임 -> 이번 프레임 화면 이동. 특징점은 한 장에 한 번만 뽑는다."""
        f1 = motion_feats(img)
        f0 = self.feat if (self.feat is not None and self.feat_of is self.prev) else \
            (motion_feats(self.prev) if self.prev is not None else None)
        self._feat_new = f1
        return motion_from_feats(f0, f1) if f0 is not None else (None, 0)

    def add(self, img, colors=True):
        """colors=False 면 추적 중에는 칸 색 읽기를 건너뛴다 (휴대폰 빠른 길 — 색은 가끔만 읽어도 된다)."""
        F = seed = None
        self._feat_new = None
        if LOCAL_AREA and self.H is not None:               # 휴대폰: 이동을 먼저 잡고, 예상 격자로 자리별 구멍 크기
            F, nin = self._motion(img)
            seed = F @ self.H if F is not None else self.H
            pts = detect_holes(img, H=seed)
        else:
            pts = detect_holes(img, pitch_of(self.H, img.shape) if self.H is not None else None)
        if len(pts) < 12:
            self.log.append(('구멍 부족', len(pts))); return False
        H = None; how = '추적'
        if self.H is not None:
            if seed is None:
                F, nin = self._motion(img)                     # ① 카메라가 얼마나 움직였나
                seed = F @ self.H if F is not None else self.H  # ② 예상 격자
            H = grow(seed, pts)                             # ③ 실제 구멍에 맞춰 다듬기
            if F is None: how = '추적(이동예상 실패)'
        if H is None or score(H, pts)[0] < 0.5 * len(pts):
            H = fit_full(pts); how = '재탐색'
            if H is None: self.log.append(('맞춤 실패', len(pts))); return False
        n, err = score(H, pts)
        ij = np.round(_ij(H, pts)).astype(int)
        ok = np.hypot(*(_ij(H, pts) - ij).T) < TOL
        lo, hi = ij[ok].min(0) - 1, ij[ok].max(0) + 2
        cols = cell_colors(img, H, lo, hi) if (colors or how == '재탐색') else {}
        if how == '재탐색' and CHECK_MIN and len(cols) >= 40:
            # ★ 구멍 4개로 구한 각도로 판(타일) 경계를 그려 보고 실제 색이 체크무늬로 맞는지 (사용자 제안, 10-26) ★
            #   엉터리 4개(물빠짐 구멍 등)로 잡은 격자는 타일 경계가 색 경계와 안 맞는다 -> 버린다
            hole_cells = set(map(tuple, ij[ok]))                       # 실제로 구멍이 맞은 칸만 (판 밖 책상·벽 제외)
            chk = checker_agree({c: v for c, v in cols.items() if c in hole_cells})
            if chk < CHECK_MIN:
                self.log.append(('체크무늬 불일치', n, len(pts), round(chk, 2))); return False
        if how == '재탐색' and self.holes:
            di, dj = self._offset_by_color(cols)
            T = np.array([[1, 0, -di], [0, 1, -dj], [0, 0, 1]], float)   # (i+di) 로 옮긴다
            H = H @ T
            ij = ij + [di, dj]; cols = {(i + di, j + dj): c for (i, j), c in cols.items()}
        for p in map(tuple, ij[ok]): self.holes[p] = self.holes.get(p, 0) + 1
        for p, c in cols.items():
            d = self.color.setdefault(p, {'B': 0, 'W': 0}); d[c] += 1
        self.H = H; self.prev = img
        if self._feat_new is not None:
            self.feat, self.feat_of = self._feat_new, img
        self.log.append((how, n, len(pts), round(err, 2)))
        self.orient()
        return True

    def orient(self):
        """판이 긴 쪽 = i 로 더 잘 맞으면 격자를 90° 돌려 늘 '짧은 쪽 = i · 긴 쪽 = j' 로 맞춘다.
        새 (i', j') = (j, -i) — 제자리 회전이라 거울상 여부는 안 바뀐다. 돌렸으면 True (칸 번호가 바뀌었다).
        판 일부만 보여 두 방향이 비슷하면 그대로 둔다 (ORIENT_MARGIN)."""
        if BI == BJ or not self.holes:
            return False
        if self._win_best(BJ, BI)[0] <= ORIENT_MARGIN * self._win_best(BI, BJ)[0]:
            return False
        M = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)    # (i', j', 1) -> (i, j, 1) = (-j', i', 1)
        self.H = self.H @ M if self.H is not None else None
        self.holes = {(j, -i): n for (i, j), n in self.holes.items()}
        self.color = {(j, -i): c for (i, j), c in self.color.items()}
        self.turns += 1
        self.log.append(('90° 돌림', self.turns))
        return True

    def tile_phase(self):
        """10칸 타일 경계의 위상 (p, q) 와 체크무늬 일치율.
        칸 (i,j) 의 타일 = ((i-p)//10, (j-q)//10). 이웃 타일은 색이 달라야 한다."""
        i0, j0 = self.window()            # 판 밖(벽·로봇·상자)의 색은 빼고 본다
        cells = [(i, j, 'B' if v['B'] >= v['W'] else 'W') for (i, j), v in self.color.items()
                 if i0 <= i < i0 + BI and j0 <= j < j0 + BJ]
        best = None
        if cells:
            ci = np.array([c[0] for c in cells]); cj = np.array([c[1] for c in cells])
            isb = np.array([c[2] == 'B' for c in cells])
        for p in range(TILE):
            for q in range(TILE):
                ev = (((ci - p) // TILE + (cj - q) // TILE) % 2 == 0) if cells else np.zeros(0, bool)
                for c0 in 'BW':                       # (예전 3중 반복과 같은 순서·같은 결과, 배열로)
                    agree = int((ev == (isb if c0 == 'B' else ~isb)).sum()) if cells else 0
                    if best is None or agree > best[0]: best = (agree, p, q, c0)
        agree, p, q, c0 = best
        return {'p': p, 'q': q, '짝수타일색': c0, '일치': agree, '칸수': len(cells),
                '일치율': round(agree / max(len(cells), 1), 3)}

    def _win_best(self, ni, nj):
        """ni×nj 창 중 구멍을 가장 많이 본 창: (본 횟수 합, (i0, j0))."""
        a = np.array(list(self.holes)); lo = a.min(0)
        M = np.zeros(np.maximum(a.max(0) - lo + 1, (ni, nj)))
        for p in a: M[tuple(p - lo)] = self.holes[tuple(p)]
        C = np.pad(M.cumsum(0).cumsum(1), ((1, 0), (1, 0)))      # 창 합을 빠르게
        S = C[ni:, nj:] - C[:-ni, nj:] - C[ni:, :-nj] + C[:-ni, :-nj]
        i, j = np.unravel_index(np.argmax(S), S.shape)
        return float(S[i, j]), (int(i + lo[0]), int(j + lo[1]))

    def window(self):
        """판 = 가장 많이 본 BI×BJ(20×40) 창. 구멍은 반드시 판 안에 있다 (판 밖은 로봇·상자·벽의 가짜).
        실측(2026-10-06 영상, 3×3 판 시절): 창 안 구멍은 중앙값 29번, 창 밖은 2번 보였다."""
        return self._win_best(BI, BJ)[1]

    def inside(self, p):
        i0, j0 = self.window()
        return i0 <= p[0] < i0 + BI and j0 <= p[1] < j0 + BJ

    def extent(self):
        a = np.array(list(self.holes))
        return a.min(0), a.max(0), a.max(0) - a.min(0) + 1

    def check(self):
        """검산: 누적 범위가 판(20×40칸)을 넘으면 어딘가 잘못 이어붙인 것."""
        lo, hi, span = self.extent()
        i0, j0 = self.window()
        ins = [p for p in self.holes if i0 <= p[0] < i0 + BI and j0 <= p[1] < j0 + BJ]
        return {'판_원점(20x40창)': (i0, j0), '판_안_구멍': f"{len(ins)}/{BI * BJ}",
                '판_밖_버린것': len(self.holes) - len(ins),
                '전체_범위_칸수': (int(span[0]), int(span[1])), '본_구멍': len(self.holes)}
