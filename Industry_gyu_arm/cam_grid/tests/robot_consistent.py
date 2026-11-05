"""프레임마다 '판과 다른 색' 지도(위에서 본 그림)를 만들고, 거의 모든 프레임에서 판과 다른 자리만 남긴다.
판에 붙은 받침 바닥은 어느 프레임에서나 같은 자리, 키 큰 몸통이 번진 자리는 일부 프레임에서만 덮인다."""
import sys, numpy as np, cv2
sys.path.insert(0, '.')
from grid import Board
from pose import PITCH
c = cv2.VideoCapture('shots/KakaoTalk_20261006_123735928.mp4'); B = Board(); fr = []; Hs = []
while True:
    ok, f = c.read()
    if not ok: break
    g = B.add(f); fr.append(f); Hs.append(B.H.copy() if g else None)
i0, j0 = B.window(); tp = B.tile_phase(); SZ = 620
A = np.array([[1 / 20, 0, i0 - 0.5], [0, 1 / 20, j0 - 0.5], [0, 0, 1]])
U, V = np.meshgrid(np.arange(SZ), np.arange(SZ))
ti = ((U - 10) // 20 + 0.5 + i0 - tp['p']) // 10; tj = ((V - 10) // 20 + 0.5 + j0 - tp['q']) // 10
exp_blue = ((ti + tj) % 2 == 0) == (tp['짝수타일색'] == 'B')
cnt = np.zeros((SZ, SZ)); fo = np.zeros((SZ, SZ))
for f, H in zip(fr, Hs):
    if H is None: continue
    t = cv2.warpPerspective(f, H @ A, (SZ, SZ), flags=cv2.WARP_INVERSE_MAP, borderValue=(0, 0, 0))
    hsv = cv2.cvtColor(t, cv2.COLOR_BGR2HSV).astype(int)
    blue = (hsv[..., 0] > 95) & (hsv[..., 0] < 130) & (hsv[..., 1] > 80)
    white = (hsv[..., 1] < 45) & (hsv[..., 2] > 140)
    valid = t.sum(-1) > 0
    cnt += valid; fo += valid & np.where(exp_blue, ~blue, ~white)
frac = np.where(cnt >= 15, fo / np.maximum(cnt, 1), 0)
np.save('shots/_top_frac.npy', frac)
cv2.imwrite('shots/_top_frac.png', (frac * 255).astype(np.uint8))
cx, cy = 522, 296
crop = cv2.cvtColor((frac[cy - 150:cy + 150, cx - 150:cx + 98] * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
crop90 = crop.copy(); crop90[frac[cy - 150:cy + 150, cx - 150:cx + 98] < 0.9] = 0
cv2.imwrite('shots/_top_frac_cmp.png', cv2.resize(np.hstack([crop, crop90]), None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
print('ok', int((frac >= 0.9).sum()), 'px 가 90% 이상')
