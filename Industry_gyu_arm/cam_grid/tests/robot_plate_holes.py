"""로봇 받침 판의 목심 구멍으로 받침 위치·방향 맞추기.
받침 판 윗면은 판보다 높다 → 카메라 자세로 영상을 '받침 판 윗면 높이' 로 펴면 그 위 구멍이 또렷해진다."""
import sys, numpy as np, cv2
from scipy import ndimage
sys.path.insert(0, '.')
from grid import Board
from pose import PITCH, focal_from_H, K_of, camera_pose
# ① 메쉬 받침 판의 구멍 (위에서 본 높이 지도에서 판 높이보다 움푹한 동그라미)
Hm = np.load('tests/_base_heightmap.npy'); O = np.array([180.0, -300.0])
lvl = np.median(Hm[(Hm > 30) & (Hm < 45)])                      # 날개 판 윗면 높이 (mm, 절대)
inside = ndimage.binary_fill_holes(Hm > 0)
dent = inside & ((Hm < 0) | (Hm < lvl - 2)) & ~(Hm > lvl + 5)
dent &= ~ndimage.binary_dilation(Hm > lvl + 5, iterations=2)    # 기둥 옆 틈은 빼고
lab, n = ndimage.label(dent)
mh = []
for k in range(1, n + 1):
    a = np.argwhere(lab == k)
    if 15 <= len(a) <= 200: mh.append(a.mean(0)[::-1] + O)       # (x, y) 로컬 mm
mh = np.array(mh); HB = lvl - 20.0                              # 판 위 높이 (받침 바닥 = 판 윗면)
print(f'메쉬 받침 판 높이 판위 {HB:.1f}mm · 구멍 {len(mh)}개:', np.round(mh).astype(int).tolist())
# ② 영상을 받침 판 높이로 펴서 중앙값
c = cv2.VideoCapture('shots/KakaoTalk_20261006_123735928.mp4'); B = Board(); fr = []; Hs = []
while True:
    ok, f = c.read()
    if not ok: break
    g = B.add(f); fr.append(f); Hs.append(B.H.copy() if g else None)
i0, j0 = B.window(); h, w = fr[0].shape[:2]
K = K_of(float(np.median([x for x in (focal_from_H(H, w, h) for H in Hs if H is not None) if x])), w, h)
SZ = 620; hb = HB / 1000
S = np.array([[1e-3, 0, -10e-3], [0, 1e-3, -10e-3], [0, 0, 0.0]])
tops = []
for f, H in zip(fr, Hs):
    if H is None: continue
    C, Rwc = camera_pose(H, K, i0, j0)[:2]
    M = K @ Rwc.T @ (S + np.outer(np.array([0, 0, hb]) - C, [0, 0, 1]))
    tops.append(cv2.warpPerspective(f, M, (SZ, SZ), flags=cv2.WARP_INVERSE_MAP, borderValue=(0, 0, 0)))
st = np.stack(tops).astype(np.float32); st[st.sum(-1) == 0] = np.nan
E = np.nan_to_num(np.nanmedian(st, 0)).astype(np.uint8)
cv2.imwrite('shots/_top_plate.png', E)
# ③ 받침 근처에서 구멍 점 찾기 (흰 판 위의 어두운 동그라미)
gray = cv2.cvtColor(E, cv2.COLOR_BGR2GRAY).astype(np.int16)
hsv = cv2.cvtColor(E, cv2.COLOR_BGR2HSV)
dark = np.clip(cv2.GaussianBlur(gray, (0, 0), 6) - gray, 0, 255).astype(np.uint8)
white = cv2.dilate(((hsv[..., 1] < 50) & (hsv[..., 2] > 140)).astype(np.uint8), np.ones((15, 15), np.uint8))
x0, x1, y0, y1 = 440, 620, 230, 420                              # 로봇 받침 근처 (판 위 mm)
roi = np.zeros_like(dark); roi[y0:y1, x0:x1] = 1
_, th = cv2.threshold(dark, 12, 255, cv2.THRESH_BINARY)
th = th * roi * white
nl, ll, ss, cc = cv2.connectedComponentsWithStats(th, 8)
dots = np.array([cc[i] for i in range(1, nl) if 12 <= ss[i, 4] <= 120 and max(ss[i, 2], ss[i, 3]) <= 16])
print('영상 받침 근처 구멍 점', len(dots), '개:', np.round(dots).astype(int).tolist())
vis = E.copy()
for d in dots: cv2.circle(vis, (int(d[0]), int(d[1])), 5, (0, 0, 255), 1)
cv2.imwrite('shots/_top_plate_dots.png', cv2.resize(vis[y0:y1, x0:x1], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
np.save('tests/_plate_dots.npy', dots); np.save('tests/_mesh_holes.npy', mh)
