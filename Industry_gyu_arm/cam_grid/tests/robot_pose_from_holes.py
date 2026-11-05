"""받침 판 목심 구멍 줄 + 몸통 방향으로 로봇 받침의 위치·방향(90도 단위) 정하기 — 시험."""
import sys, numpy as np, cv2
from scipy import ndimage
sys.path.insert(0, '.')
E = cv2.imread('shots/_top_plate.png')                      # 받침 판 높이(15mm)로 편 중앙값 (robot_plate_holes.py)
mh = np.load('tests/_mesh_holes.npy')                      # 메쉬 받침 판 구멍 (로컬 mm)
Hm = np.load('tests/_base_heightmap.npy'); O = np.array([180.0, -300.0])
lvl = np.median(Hm[(Hm > 30) & (Hm < 45)])
col = np.argwhere(Hm > lvl + 5).mean(0)[::-1] + O            # 기둥 가운데 (로컬 x, y)
front = mh[np.argsort(mh[:, 1])[-2:]].mean(0)               # 틈 있는 가장자리의 구멍 한 쌍 가운데
foot_l = np.array([239.3, -256.0])                          # 받침 바닥 가운데 (로컬)
d_l = front - col
print('메쉬: 기둥', np.round(col, 1), '앞 구멍쌍', np.round(front, 1), '방향', np.round(d_l / np.linalg.norm(d_l), 2))
# 영상: 받침 근처 어두운 점 중 '판 목심 격자에 딱 맞는' 것만 (받침 구멍은 판 격자 위에 있다)
gray = cv2.cvtColor(E, cv2.COLOR_BGR2GRAY).astype(np.int16)
hsv = cv2.cvtColor(E, cv2.COLOR_BGR2HSV)
dark = np.clip(cv2.GaussianBlur(gray, (0, 0), 6) - gray, 0, 255).astype(np.uint8)
white = cv2.dilate(((hsv[..., 1] < 50) & (hsv[..., 2] > 140)).astype(np.uint8), np.ones((15, 15), np.uint8))
x0, x1, y0, y1 = 420, 620, 200, 430
roi = np.zeros_like(dark); roi[y0:y1, x0:x1] = 1
_, th = cv2.threshold(dark, 12, 255, cv2.THRESH_BINARY)
nl, ll, ss, cc = cv2.connectedComponentsWithStats(th * roi * white, 8)
dots = np.array([cc[i] for i in range(1, nl) if 12 <= ss[i, 4] <= 120 and max(ss[i, 2], ss[i, 3]) <= 16])
off = np.abs(((dots - 10) % 20 + 10) % 20 - 10)              # 격자에서 벗어난 정도 (mm)
on = dots[(off < 3.5).all(1)]
print('영상 받침 근처 점', len(dots), '→ 판 격자 위', len(on), ':', np.round(on).astype(int).tolist())
# 가장 많은 점이 놓인 한 줄 (가로 또는 세로) = 받침 구멍 줄
rows = {}
for p in on:
    rows.setdefault(('y', int(round((p[1] - 10) / 20))), []).append(p)
    rows.setdefault(('x', int(round((p[0] - 10) / 20))), []).append(p)
key, line = max(rows.items(), key=lambda kv: len(kv[1]))
line = np.array(line); hole_mid = line.mean(0)
print('받침 구멍 줄:', key, len(line), '개, 가운데', np.round(hole_mid, 1))
# 몸통 = 받침 판 높이 그림에서 흰 덩어리 중 구멍 줄 반대편 (구멍 줄 근처 25mm 띠는 빼고)
wm = ((hsv[..., 1] < 50) & (hsv[..., 2] > 140)).astype(np.uint8) * roi
lab, n = ndimage.label(wm); big = lab == (1 + np.argmax(ndimage.sum(wm, lab, range(1, n + 1))))
pts = np.argwhere(big)[:, ::-1].astype(float)
far = pts[np.linalg.norm(pts - hole_mid, axis=1) > 25]
body = far.mean(0); d_r = hole_mid - body
print('영상: 몸통 가운데', np.round(body, 1), '방향(몸통→구멍줄)', np.round(d_r / np.linalg.norm(d_r), 2))
yaw = np.arctan2(d_r[1], d_r[0]) - np.arctan2(d_l[1], d_l[0])
yaw90 = round(yaw / (np.pi / 2)) * (np.pi / 2)
print(f'방향: 잰 값 {np.degrees(yaw) % 360:.1f}° → 90도 단위 {np.degrees(yaw90) % 360:.0f}°')
c, s = np.cos(yaw90), np.sin(yaw90); R = np.array([[c, -s], [s, c]])
foot_b = hole_mid + R @ (foot_l - front)
print('받침 바닥 가운데 (판 mm):', np.round(foot_b, 1), '  판 칸', np.round((foot_b - 10) / 20, 2))
# 그림: 메쉬 높이지도를 그 자세로 덮어 그린다
vis = E.copy()
ys, xs = np.nonzero(Hm > 0)
loc = np.stack([xs + O[0], ys + O[1]], 1)
wb = (loc - front) @ R.T + hole_mid
col_ = np.where(Hm[ys, xs] > lvl + 5, 0, 1)
for (u, v), k in zip(wb.astype(int), col_):
    if 0 <= u < 620 and 0 <= v < 620: vis[v, u] = (0, 0, 255) if k == 0 else (0, 200, 0)
for p in mh: q = (R @ (p - front) + hole_mid).astype(int); cv2.circle(vis, tuple(q), 5, (255, 0, 255), 1)
for p in on: cv2.circle(vis, tuple(p.astype(int)), 3, (0, 255, 255), -1)
blend = cv2.addWeighted(E, 0.55, vis, 0.45, 0)
cv2.imwrite('shots/_robot_pose_fit.png', cv2.resize(blend[y0:y1, x0:x1], None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
