"""위에서 본 중앙값 그림에 받침 바닥 윤곽을 돌리고 옮겨 가며 맞춘다 (엣지 거리 = 챔퍼 정합)."""
import sys, numpy as np, cv2, trimesh
from scipy import ndimage
sys.path.insert(0, '.')
med = cv2.imread('shots/_top_median.png')
# ① 받침 바닥 윤곽 (로봇 로컬, mm). 아래를 보는 면을 1mm 격자에 칠하고 안쪽 홈만 메운다 (두 발 사이 틈은 열린 채)
m = trimesh.load('/home/drboom/2026_project/urdf/sts3215/Industry_gyu_arm_20260728_base/meshes/base_link.STL')
v = np.asarray(m.vertices) * 1000; F = np.asarray(m.faces); n = m.face_normals
O = np.array([150.0, -340.0]); mask = np.zeros((200, 200), np.uint8)       # (행 = y, 열 = x)
for f in F[n[:, 2] < -0.9]:
    cv2.fillPoly(mask, [(v[f][:, :2] - O).astype(np.int32)], 1)
mask = ndimage.binary_fill_holes(mask).astype(np.uint8)
cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
outline = max(cnts, key=cv2.contourArea)[:, 0, :].astype(float) + O        # 로컬 mm (x, y)
outline = outline[::2]
PIN = np.array([210.0, -250.0]); FOOT = outline.mean(0)
print('윤곽 점', len(outline), ' 바닥 가운데(로컬mm)', np.round(FOOT, 1))
# ② '판과 다른 색' 지도. 판 무늬(파랑·흰 타일)를 알고 있으니 자리마다 와야 할 색과 비교한다.
#    구멍 자리(판 구멍 지름 ~10mm)는 어느 쪽인지 애매하므로 판단에서 뺀다.
import pickle
hsv = cv2.cvtColor(med, cv2.COLOR_BGR2HSV).astype(int); Hh, Ss, Vv = hsv[..., 0], hsv[..., 1], hsv[..., 2]
is_blue = (Hh > 95) & (Hh < 130) & (Ss > 80)
is_white = (Ss < 45) & (Vv > 140)
tp = {'p': 8, 'q': 4, 'even': 'B'}; i0, j0 = -12, -26                   # 이 영상의 타일 위상 (grid.Board.tile_phase)
U, V = np.meshgrid(np.arange(med.shape[1]), np.arange(med.shape[0]))
ti = ((U - 10) // 20 + 0.5 + i0 - tp['p']) // 10; tj = ((V - 10) // 20 + 0.5 + j0 - tp['q']) // 10
exp_blue = ((ti + tj) % 2 == 0) == (tp['even'] == 'B')
foreign = np.where(exp_blue, ~is_blue, ~is_white) & (med.sum(-1) > 0)
holes = np.zeros(foreign.shape, np.uint8)
for a_ in range(30):
    for b_ in range(30): cv2.circle(holes, (10 + 20 * a_, 10 + 20 * b_), 6, 1, -1)
care = holes == 0
cv2.imwrite('shots/_top_foreign.png', (foreign * 255).astype(np.uint8))
tm = np.zeros((260, 260), np.uint8)                                    # 꽉 찬 받침 모양 (가운데 기준)
cv2.fillPoly(tm, [(outline - FOOT + 130).astype(np.int32)], 1)
ring = cv2.dilate(tm, np.ones((17, 17), np.uint8)) - tm                 # 바로 바깥 8mm 띠
def score(th, tx, ty):
    M = cv2.getRotationMatrix2D((130, 130), -np.degrees(th), 1.0)
    T = cv2.warpAffine(tm, M, (260, 260), flags=cv2.INTER_NEAREST).astype(bool)
    Rg = cv2.warpAffine(ring, M, (260, 260), flags=cv2.INTER_NEAREST).astype(bool)
    y0, x0 = ty - 130, tx - 130
    if y0 < 0 or x0 < 0 or y0 + 260 > foreign.shape[0] or x0 + 260 > foreign.shape[1]: return -9
    Fw = foreign[y0:y0 + 260, x0:x0 + 260]; Cw = care[y0:y0 + 260, x0:x0 + 260]
    ins = Fw[T & Cw].mean() if (T & Cw).any() else 0
    out = Fw[Rg & Cw].mean() if (Rg & Cw).any() else 1
    return ins - out
cx, cy = 10 + 25.6 * 20, 10 + 14.3 * 20
best = []
for deg in range(0, 360, 5):
    th = np.radians(deg)
    r = max(((score(th, tx, ty), tx, ty) for tx in range(int(cx) - 70, int(cx) + 71, 3) for ty in range(int(cy) - 70, int(cy) + 71, 3)))
    best.append((r[0], deg, r[1], r[2]))
best.sort(reverse=True)
for b in best[:6]: print(f'각도 {b[1]:3d}°  점수 {b[0]:.3f} (안=판과 다른 색 비율 - 바깥띠 비율)  가운데 ({b[2]},{b[3]})mm')
per90 = {}
for b in best:
    per90.setdefault(int(round(b[1] / 90)) % 4, b)
print('90도 묶음별 최고:', {k * 90: (round(v[0], 3), v[1]) for k, v in sorted(per90.items())})
best = [(b[0], b[1], b[2], b[3]) for b in best]
# ④ 그림
s0, deg, tx, ty = best[0]; th = np.radians(deg); c, s = np.cos(th), np.sin(th)
p = ((outline - FOOT) @ np.array([[c, s], [-s, c]]) + [tx, ty]).astype(np.int32)
out = med.copy(); cv2.polylines(out, [p], True, (0, 0, 255), 2)
cv2.imwrite('shots/_top_fit.png', cv2.resize(out[ty - 120:ty + 120, tx - 120:tx + 120], None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
