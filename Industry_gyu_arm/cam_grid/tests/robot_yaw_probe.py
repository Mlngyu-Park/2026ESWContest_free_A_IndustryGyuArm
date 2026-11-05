"""받침 바닥 모양 끼워 맞추기로 로봇 방향을 가를 수 있는지 시험."""
import sys, time, numpy as np, trimesh, glob, cv2
sys.path.insert(0, '.')
from grid import Board, detect_holes, pitch_of
from objects import Tracker, N
from matplotlib.path import Path
# ① 받침 바닥 면 -> 받침 밑 목심 자리 (로봇 로컬, 핀이 목심에 꽂히는 격자)
pk = '/home/drboom/2026_project/urdf/sts3215/Industry_gyu_arm_20260728_base'
m = trimesh.load(pk + '/meshes/base_link.STL')
from scipy import ndimage
v = np.asarray(m.vertices); F = np.asarray(m.faces); n = m.face_normals
down = [f for f, nn in zip(F, n) if nn[2] < -0.9]                      # 아래를 보는 면 (받침 바닥)
O = np.array([0.15, -0.34]); mask = np.zeros((200, 200), np.uint8)       # 1mm 격자
for f in down:
    cv2.fillPoly(mask, [((v[f][:, [1, 0]] - O[[1, 0]]) * 1000).astype(np.int32)], 1)   # (행=x, 열=y)
mask = ndimage.binary_fill_holes(ndimage.binary_closing(mask, np.ones((5, 5)), iterations=2))   # 홈은 메운다
PIN = np.array([0.210, -0.250])                                          # robot_pins_fixed (m)
foot = []
for a in range(-6, 7):
    for b in range(-6, 7):
        q = ((PIN + 0.02 * np.array([a, b]) - O) * 1000).astype(int)
        # 판 구멍(지름 ~8mm)이 받침 테두리 안에 다 들어와야 '가려짐'
        if 5 <= q[0] < 195 and 5 <= q[1] < 195 and mask[q[0] - 5:q[0] + 6, q[1] - 5:q[1] + 6].T.all(): foot.append((a, b))
print('받침 밑 목심 자리', len(foot), '개 (핀 기준 칸):', foot)
# ② 영상 끝까지 추적해서 칸마다 '본 횟수', '실제로 보인 횟수'
c = cv2.VideoCapture('shots/KakaoTalk_20261006_123735928.mp4'); B = Board(); fr = []; Hs = []
while True:
    ok, f = c.read()
    if not ok: break
    g = B.add(f); fr.append(f); Hs.append(B.H.copy() if g else None)
i0, j0 = B.window(); tp = B.tile_phase()
Tr = Tracker(i0, j0, None, (tp['p'], tp['q']))
for f, H in zip(fr, Hs):
    if H is not None: Tr.update(f, H, detect_holes(f, pitch_of(H, f.shape)))
edge = Tr.edge_mask(); hit, cnt = Tr.hit, Tr.cnt
# ③ 4방향 × 위치 찾기 (받침 다수결 자리 주변)
base = np.array([25.6, 14.3])
def score(yaw, pa, pb):
    s = 0; seen = cov = unk = 0
    R = {0: ((1, 0), (0, 1)), 1: ((0, -1), (1, 0)), 2: ((-1, 0), (0, -1)), 3: ((0, 1), (-1, 0))}[yaw]
    for a, b in foot:
        ci = pa + R[0][0] * a + R[0][1] * b; cj = pb + R[1][0] * a + R[1][1] * b
        if not (0 <= ci < N and 0 <= cj < N) or edge[ci, cj] or cnt[ci, cj] < 6: unk += 1; continue
        if hit[ci, cj] >= 2: s -= 3; seen += 1
        elif hit[ci, cj] == 0: s += 1; cov += 1
        else: unk += 1
    return s, cov, seen, unk
res = []
for yaw in range(4):
    best = max(((score(yaw, pa, pb), pa, pb) for pa in range(int(base[0]) - 6, int(base[0]) + 7)
                for pb in range(int(base[1]) - 6, int(base[1]) + 7)), key=lambda x: x[0][0])
    res.append((yaw, best)); (s, cov, seen, unk), pa, pb = best
    print(f'방향 {yaw * 90:3d}°: 점수 {s:3d}  (받침 밑이 안 보임 {cov}, 또렷이 보임 {seen}, 모름 {unk})  핀 칸 ({pa},{pb})')
