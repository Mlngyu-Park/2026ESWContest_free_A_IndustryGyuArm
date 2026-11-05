"""가짜 영상 시험 — 사진 한 장에서 카메라가 움직이는 프레임을 만든다.
각 프레임의 정답 격자 = W @ H_full (W: 사진 -> 프레임). 그래서 추적이 준 칸 번호가
정답과 '항상 같은 정수만큼' 차이 나야 한다 (차이는 원점 선택일 뿐). 한 번이라도 바뀌면 잘못 이은 것."""
import cv2, numpy as np, sys
from grid import *
img = cv2.imread('/home/drboom/2026_project/스크린샷 2026-09-23 130622.png')
Hf = np.load('H_full.npy'); h, w = img.shape[:2]
FW, FH = 640, 480                     # 프레임 크기 (실제 카메라 해상도)
N = int(sys.argv[1]) if len(sys.argv) > 1 else 40
jump = '--jump' in sys.argv           # 중간에 프레임을 건너뛰어 추적을 일부러 끊는다

def W_at(t):
    """시각 t(0~1) 의 사진->프레임 변환: 판 위를 ㄹ자로 훑는다."""
    cx = w * (0.30 + 0.40 * t); cy = h * (0.30 + 0.40 * (0.5 - 0.5 * np.cos(2 * np.pi * t)))
    ang = np.radians(8 * np.sin(3 * t)); s = 0.9
    T1 = np.array([[1, 0, -cx], [0, 1, -cy], [0, 0, 1]])
    R = np.array([[np.cos(ang) * s, -np.sin(ang) * s, 0], [np.sin(ang) * s, np.cos(ang) * s, 0], [0, 0, 1]])
    P = np.array([[1, 0, 0], [0, 1, 0], [2e-4 * np.sin(5 * t), 1e-4 * np.cos(4 * t), 1]])   # 살짝 기울임
    T2 = np.array([[1, 0, FW / 2], [0, 1, FH / 2], [0, 0, 1]])
    return T2 @ P @ R @ T1

B = Board(); shifts = []; bad = 0
ts = np.linspace(0, 1, N)
if jump: ts = np.concatenate([ts[:N // 2], ts[N // 2 + N // 5:]])   # 가운데를 크게 건너뜀
for k, t in enumerate(ts):
    W = W_at(t); fr = cv2.warpPerspective(img, W, (FW, FH))
    ok = B.add(fr)
    if not ok: print(k, '실패', B.log[-1]); continue
    true_H = W @ Hf
    pts = detect_holes(fr)
    a = np.round(cv2.perspectiveTransform(pts.reshape(-1,1,2).astype(np.float32), np.linalg.inv(B.H)).reshape(-1,2))
    b = np.round(cv2.perspectiveTransform(pts.reshape(-1,1,2).astype(np.float32), np.linalg.inv(true_H)).reshape(-1,2))
    d = a - b; vals, cnt = np.unique(d, axis=0, return_counts=True); sh = tuple(vals[np.argmax(cnt)].astype(int))
    shifts.append(sh)
    if sh != shifts[0]: bad += 1
    if k % 5 == 0 or sh != shifts[0] or B.log[-1][0] != '추적':
        print(f"{k:3d} {B.log[-1]}  칸번호 차이 {sh}")
print('\n프레임', len(shifts), '/', len(ts), ' 칸번호가 처음과 달라진 프레임:', bad)
print('검산:', B.check())
