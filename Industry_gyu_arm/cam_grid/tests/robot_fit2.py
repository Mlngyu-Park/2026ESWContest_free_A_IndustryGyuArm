"""받침 윤곽을 '늘 판과 다른 색인 덩어리' 의 바깥 경계에 맞춘다.
받침 위쪽 경계는 몸통 덩어리에 묻혀도, 아래 세 변과 두 발 사이 홈은 덩어리 경계와 겹친다."""
import sys, numpy as np, cv2, trimesh
from scipy import ndimage
frac = np.load('shots/_top_frac.npy'); F90 = (frac >= 0.9).astype(np.uint8)
F90 = cv2.morphologyEx(F90, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
bd = cv2.morphologyEx(F90, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
dt = cv2.distanceTransform(1 - bd, cv2.DIST_L2, 3)
m = trimesh.load('/home/drboom/2026_project/urdf/sts3215/Industry_gyu_arm_20260728_base/meshes/base_link.STL')
v = np.asarray(m.vertices) * 1000; Fc = np.asarray(m.faces); n = m.face_normals
O = np.array([150.0, -340.0]); mask = np.zeros((200, 200), np.uint8)
for f in Fc[n[:, 2] < -0.9]: cv2.fillPoly(mask, [(v[f][:, :2] - O).astype(np.int32)], 1)
mask = ndimage.binary_fill_holes(mask).astype(np.uint8)
cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
outline = max(cnts, key=cv2.contourArea)[:, 0, :].astype(float) + O
FOOT = outline.mean(0); oc = outline - FOOT
inner = np.argwhere(mask)[:, ::-1].astype(float) + O - FOOT                  # 안쪽 점들 (x, y)
inner = inner[::7]
def score(th, tx, ty):
    c, s = np.cos(th), np.sin(th); R = np.array([[c, s], [-s, c]])
    p = (oc @ R + [tx, ty]).astype(int); q = (inner @ R + [tx, ty]).astype(int)
    H, W = F90.shape
    if p.min() < 0 or p[:, 0].max() >= W or p[:, 1].max() >= H: return -1, 0, 0
    on_edge = (dt[p[:, 1], p[:, 0]] <= 3).mean()
    filled = F90[q[:, 1], q[:, 0]].mean()
    return (on_edge if filled >= 0.85 else -1), on_edge, filled
cx, cy = 522, 296
res = []
for deg in range(0, 360, 3):
    th = np.radians(deg)
    r = max(((score(th, tx, ty), tx, ty) for tx in range(cx - 110, cx + 80, 2) for ty in range(cy - 90, cy + 140, 2)), key=lambda x: x[0][0])
    res.append((r[0][0], deg, r[1], r[2], r[0][1], r[0][2]))
res.sort(reverse=True)
for r in res[:6]: print(f'각도 {r[1]:3d}°  경계에 겹친 윤곽 {r[4]:.2f}  안쪽 채움 {r[5]:.2f}  가운데 ({r[2]},{r[3]})mm')
per = {}
for r in res: per.setdefault(int(round(r[1] / 90)) % 4, r)
print('90도 묶음별 최고:', {k * 90: (round(v[0], 3), v[1]) for k, v in sorted(per.items())})
s0, deg, tx, ty = res[0][:4]; th = np.radians(deg); c, s = np.cos(th), np.sin(th)
med = cv2.imread('shots/_top_median.png')
pp = (oc @ np.array([[c, s], [-s, c]]) + [tx, ty]).astype(np.int32)
out = med.copy(); cv2.polylines(out, [pp], True, (0, 0, 255), 2)
cv2.imwrite('shots/_top_fit2.png', cv2.resize(out[cy - 150:cy + 150, cx - 150:cx + 98], None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
np.save('shots/_robot_fit.npy', np.array([deg, tx, ty]))
