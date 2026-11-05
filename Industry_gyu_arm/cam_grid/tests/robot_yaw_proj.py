"""로봇 받침 3D 모양을 카메라 시점으로 투영해 방향을 가른다 (시험).
방향·위치 후보마다: 받침 볼록 껍질을 프레임에 투영 → 그 안에 드는 판 구멍 = '가려져야 할 구멍'.
실제로 그 프레임에서 또렷이 보인 구멍이 그 안에 있으면 감점 (거기엔 받침이 없다는 증거)."""
import sys, numpy as np, trimesh, cv2
sys.path.insert(0, '.')
from grid import Board, detect_holes, pitch_of, TOL
from pose import PITCH, focal_from_H, K_of, camera_pose
from objects import N
m = trimesh.load('/home/drboom/2026_project/urdf/sts3215/Industry_gyu_arm_20260728_base/meshes/base_link.STL')
hull = np.asarray(m.convex_hull.vertices)                         # 받침 볼록 껍질 (로컬 m)
z0 = hull[:, 2].min(); PIN = np.array([0.210, -0.250])
c = cv2.VideoCapture('shots/KakaoTalk_20261006_123735928.mp4'); B = Board(); fr = []; Hs = []
while True:
    ok, f = c.read()
    if not ok: break
    g = B.add(f); fr.append(f); Hs.append(B.H.copy() if g else None)
i0, j0 = B.window(); h, w = fr[0].shape[:2]
f_ = float(np.median([x for x in (focal_from_H(H, w, h) for H in Hs if H is not None) if x])); K = K_of(f_, w, h)
I, J = np.meshgrid(np.arange(N), np.arange(N), indexing='ij')
holes_b = np.stack([I.ravel() * PITCH, J.ravel() * PITCH, np.zeros(N * N)], 1)
frames = []                                                        # (C, Rwc, 보인 구멍 bool[900], 화면안 bool[900], 구멍 픽셀)
for k in range(0, len(fr), 2):
    H = Hs[k]
    if H is None: continue
    C, Rwc = camera_pose(H, K, i0, j0)[:2]
    pts = detect_holes(fr[k], pitch_of(H, fr[k].shape))
    seen = np.zeros(N * N, bool)
    if len(pts):
        ij = cv2.perspectiveTransform(pts.reshape(-1, 1, 2).astype(np.float32), np.linalg.inv(H)).reshape(-1, 2)
        r = np.round(ij); ok = np.hypot(*(ij - r).T) < TOL
        for a, b in r[ok].astype(int):
            a -= i0; b -= j0
            if 0 <= a < N and 0 <= b < N: seen[a * N + b] = True
    Xc = (holes_b - C) @ Rwc; uv = Xc @ K.T; uv = uv[:, :2] / uv[:, 2:3]
    inv = (uv[:, 0] > 12) & (uv[:, 0] < w - 12) & (uv[:, 1] > 12) & (uv[:, 1] < h - 12)
    frames.append((C, Rwc, seen, inv, uv))
print(f'프레임 {len(frames)}개 사용')
ROT = lambda q: np.array([[np.cos(q), -np.sin(q), 0], [np.sin(q), np.cos(q), 0], [0, 0, 1.0]])
def evaluate(yaw, pa, pb):
    R = ROT(yaw * np.pi / 2)
    P = (hull - [PIN[0], PIN[1], z0]) @ R.T + [pa * PITCH, pb * PITCH, 0]   # 받침 껍질 -> 판 좌표
    bad = good = 0
    for C, Rwc, seen, inv, uv in frames:
        Xc = (P - C) @ Rwc
        if (Xc[:, 2] <= 0.01).any(): continue
        q = Xc @ K.T; q = (q[:, :2] / q[:, 2:3]).astype(np.float32)
        hullpx = cv2.convexHull(q)
        insil = np.array([cv2.pointPolygonTest(hullpx, (float(x), float(y)), False) > 0 if v else False
                          for (x, y), v in zip(uv, inv)])
        bad += int((insil & seen).sum()); good += int((insil & ~seen).sum())
    return good - 3 * bad, good, bad
base = (25.6, 14.3)
for yaw in range(4):
    best = max(((evaluate(yaw, pa, pb), pa, pb) for pa in range(int(base[0]) - 3, int(base[0]) + 4)
                for pb in range(int(base[1]) - 3, int(base[1]) + 4)), key=lambda x: x[0][0])
    (s, g, b), pa, pb = best
    print(f'방향 {yaw * 90:3d}°: 점수 {s:5d}  (가려져야 할 구멍 중 안 보임 {g}, 또렷이 보임 {b})  핀 칸 ({pa},{pb})', flush=True)
