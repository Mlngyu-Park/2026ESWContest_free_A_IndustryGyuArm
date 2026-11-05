"""받침 판의 '점(뚫린 구멍)' 과 판 경계로 받침 위치·방향 맞추기.
메쉬를 위에서 본 높이 지도에서: 낮은 판 / 뚫린 구멍 / 키 큰 기둥(판단 제외) / 바깥 띠 를 뽑는다.
위에서 본 영상(중앙값)에서: 판과 다른 색 = 받침, 판 색 = 판이 보임."""
import sys, numpy as np, cv2
from scipy import ndimage
Hm = np.load('tests/_base_heightmap.npy')                 # 행 = 로컬 y+300, 열 = 로컬 x-180 (mm)
O = np.array([180.0, -300.0])
inside = ndimage.binary_fill_holes(Hm > 0)
plate = (Hm > 0) & (Hm <= 45)                             # 판 위 25mm 이하 (바닥 20mm 기준)
hole = inside & (Hm < 0)                                  # 뚫린 구멍 (위에서 판이 보임)
column = Hm > 45
ring = cv2.dilate(inside.astype(np.uint8), np.ones((15, 15), np.uint8)).astype(bool) & ~inside
print(f'판 {plate.sum()} · 구멍 {hole.sum()} · 기둥 {column.sum()} · 바깥띠 {ring.sum()} (mm²)')
med = cv2.imread('shots/_top_median.png'); hsv = cv2.cvtColor(med, cv2.COLOR_BGR2HSV).astype(int)
is_blue = (hsv[..., 0] > 95) & (hsv[..., 0] < 130) & (hsv[..., 1] > 80)
is_white = (hsv[..., 1] < 45) & (hsv[..., 2] > 140)
tp = {'p': 8, 'q': 4, 'even': 'B'}; i0, j0 = -12, -26
U, V = np.meshgrid(np.arange(620), np.arange(620))
ti = ((U - 10) // 20 + 0.5 + i0 - tp['p']) // 10; tj = ((V - 10) // 20 + 0.5 + j0 - tp['q']) // 10
exp_blue = ((ti + tj) % 2 == 0) == (tp['even'] == 'B')
board = np.where(exp_blue, is_blue, is_white)            # 이 자리에서 판이 보이나
foreign = ~board & (med.sum(-1) > 0)
cy0, cx0 = np.array(Hm.shape) / 2
P_plate = np.argwhere(plate)[::3] - [cy0, cx0]; P_hole = np.argwhere(hole) - [cy0, cx0]; P_ring = np.argwhere(ring)[::3] - [cy0, cx0]
def pts(P, th, tx, ty):
    c, s = np.cos(th), np.sin(th)
    yx = P @ np.array([[c, s], [-s, c]]).T                 # (y, x) 회전
    return np.clip((yx[:, 0] + ty).astype(int), 0, 619), np.clip((yx[:, 1] + tx).astype(int), 0, 619)
def score(th, tx, ty):
    y, x = pts(P_plate, th, tx, ty); a = foreign[y, x].mean()
    y, x = pts(P_hole, th, tx, ty); b = board[y, x].mean() if len(y) else 0
    y, x = pts(P_ring, th, tx, ty); c = board[y, x].mean()
    return a + b + c, a, b, c
res = []
for deg in range(0, 360, 3):
    th = np.radians(deg)
    r = max(((score(th, tx, ty), tx, ty) for tx in range(470, 610, 2) for ty in range(250, 400, 2)), key=lambda z: z[0][0])
    res.append((r[0][0], deg, r[1], r[2], *r[0][1:]))
res.sort(reverse=True)
for r in res[:5]: print(f'각도 {r[1]:3d}°  합 {r[0]:.3f} (판 {r[4]:.2f} 구멍 {r[5]:.2f} 바깥 {r[6]:.2f})  가운데 ({r[2]},{r[3]})mm')
per = {}
for r in res: per.setdefault(int(round(r[1] / 90)) % 4, r)
print('90도 묶음별 최고:', {k * 90: (round(v[0], 3), v[1]) for k, v in sorted(per.items())})
s0, deg, tx, ty = res[0][:4]; th = np.radians(deg)
out = med.copy()
for P, col in ((P_plate, (0, 255, 0)), (P_hole, (0, 0, 255)), (P_ring, (255, 255, 0))):
    y, x = pts(P, th, tx, ty); out[y, x] = col
cv2.imwrite('shots/_fit_holes.png', cv2.resize(out[ty - 90:ty + 90, tx - 90:tx + 90], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
np.save('shots/_robot_fit.npy', np.array([deg, tx, ty]))
