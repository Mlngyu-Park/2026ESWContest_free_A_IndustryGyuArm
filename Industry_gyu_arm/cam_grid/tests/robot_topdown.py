"""영상을 판 위에서 본 그림으로 펴고(1px = 1mm), 여러 프레임의 중앙값을 낸다.
판에 붙은 것(받침 바닥 윤곽)은 늘 같은 자리, 키 큰 몸통은 보는 방향마다 번진다 → 중앙값에 바닥 윤곽만 남는다."""
import sys, numpy as np, cv2
sys.path.insert(0, '.')
from grid import Board
from pose import PITCH
c = cv2.VideoCapture('shots/KakaoTalk_20261006_123735928.mp4'); B = Board(); fr = []; Hs = []
while True:
    ok, f = c.read()
    if not ok: break
    g = B.add(f); fr.append(f); Hs.append(B.H.copy() if g else None)
i0, j0 = B.window()
MM = 1.0                                             # 1px = 1mm
SZ = 620                                             # 판 600mm + 여유
def topdown(f, H):
    # 위 그림 픽셀 (u,v) mm -> 판 격자 (i,j) = (u-10)/20 + i0 ... -> 영상 픽셀 (H)
    A = np.array([[1 / (PITCH * 1000) * MM, 0, i0 - 10 / 20], [0, 1 / (PITCH * 1000) * MM, j0 - 10 / 20], [0, 0, 1]])
    return cv2.warpPerspective(f, H @ A, (SZ, SZ), flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR, borderValue=(0, 0, 0))
tops = [topdown(f, H) for f, H in zip(fr, Hs) if H is not None]
stack = np.stack(tops).astype(np.float32); valid = stack.sum(-1) > 0
med = np.zeros_like(tops[0])
for ch in range(3):
    s = np.where(valid, stack[..., ch], np.nan); med[..., ch] = np.nan_to_num(np.nanmedian(s, 0)).astype(np.uint8)
cv2.imwrite('shots/_top_median.png', med)
cv2.imwrite('shots/_top_one.png', tops[len(tops) // 2])
# 로봇 받침 근처 (판 칸 25.6,14.3 = mm (10+25.6*20, 10+14.3*20))
cx, cy = int(10 + 25.6 * 20), int(10 + 14.3 * 20)
crop = med[max(0, cy - 150):cy + 150, max(0, cx - 150):cx + 150]
cv2.imwrite('shots/_top_robot.png', cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
print('중앙값 그림', med.shape, '로봇 근처', crop.shape)
