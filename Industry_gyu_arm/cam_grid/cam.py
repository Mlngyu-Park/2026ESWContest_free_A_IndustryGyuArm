#!/usr/bin/env python3
"""카메라로 목심 격자 확인하기 — 박민규님이 직접 실행.

  python cam.py snap                 한 장 찍어서 격자 맞추기 (결과 그림 저장)
  python cam.py rec 15               15초 녹화 (판 위를 천천히 훑으면서)
  python cam.py track shots/rec_XXXX.mp4   녹화한 영상으로 30×30 판 이어붙이기
  python cam.py show shots/rec_XXXX.mp4    영상 위에 판 목심(초록 원)을 덧그려 재생 + 저장
  python cam.py snap --image 사진.png      카메라 없이 사진으로 시험

결과는 cam_grid/shots/ 에 저장된다.
카메라: /dev/video0 (Logi C270, usbip 로 넘긴 것), MJPG 1280x720.
"""
import os, sys, time
import cv2, numpy as np
from grid import detect_holes, fit_full, score, Board, _ij, TOL

HERE = os.path.dirname(os.path.abspath(__file__))
SHOTS = os.path.join(HERE, 'shots')
DEV, W, H = 0, 1280, 720          # /dev/video0 — OpenCV 는 경로 이름으로는 못 열어서 번호로 연다


def open_cam():
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))   # usbip 로는 압축이 덜 끊긴다
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    if not cap.isOpened():
        sys.exit(f"카메라를 못 열었습니다: /dev/video{DEV} (usbipd attach 했는지 확인)")
    for _ in range(15):                     # 처음 몇 장은 노출이 안 맞아 버린다
        cap.read()
    return cap


def overlay(img, pts, Hm):
    """초록 = 격자 예측 자리, 빨강 = 실제로 찾은 구멍."""
    out = img.copy(); h, w = img.shape[:2]
    ij = np.round(_ij(Hm, pts)); ok = np.hypot(*(_ij(Hm, pts) - ij).T) < TOL
    lo, hi = ij[ok].min(0).astype(int) - 3, ij[ok].max(0).astype(int) + 4
    for i in range(lo[0], hi[0]):
        for j in range(lo[1], hi[1]):
            p = cv2.perspectiveTransform(np.float32([[[i, j]]]), Hm)[0, 0]
            if 0 <= p[0] < w and 0 <= p[1] < h:
                cv2.circle(out, (int(p[0]), int(p[1])), 6, (0, 255, 0), 1)
    for p in pts[ok]: cv2.circle(out, (int(p[0]), int(p[1])), 3, (0, 0, 255), -1)
    return out


def snap(image=None):
    os.makedirs(SHOTS, exist_ok=True)
    stamp = time.strftime('%m%d_%H%M%S')
    if image:
        img = cv2.imread(image)
        if img is None: sys.exit(f"사진을 못 읽었습니다: {image}")
    else:
        cap = open_cam(); ok, img = cap.read(); cap.release()
        if not ok: sys.exit("프레임을 못 받았습니다")
        cv2.imwrite(os.path.join(SHOTS, f'snap_{stamp}.png'), img)
    print(f"사진 {img.shape[1]}x{img.shape[0]}")
    pts = detect_holes(img)
    print(f"큰 구멍 후보 {len(pts)}개")
    if len(pts) < 12:
        sys.exit("구멍이 너무 적습니다 — 판이 화면에 크게 들어오게, 조명을 밝게 해 주세요")
    Hm = fit_full(pts, tries=min(len(pts), 120))
    if Hm is None: sys.exit("격자를 못 맞췄습니다")
    n, err = score(Hm, pts)
    print(f"격자에 맞음 {n}/{len(pts)} ({n / len(pts):.0%}), 재투영 오차 중앙값 {err:.2f}px")
    path = os.path.join(SHOTS, f'snap_{stamp}_grid.png')
    cv2.imwrite(path, overlay(img, pts, Hm))
    print(f"결과 그림: {path}  (초록 = 예측 자리, 빨강 = 찾은 구멍)")


def rec(sec):
    os.makedirs(SHOTS, exist_ok=True)
    path = os.path.join(SHOTS, f"rec_{time.strftime('%m%d_%H%M%S')}.mp4")
    cap = open_cam(); ok, f = cap.read()
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), 15, (f.shape[1], f.shape[0]))
    print(f"{sec}초 녹화 시작 — 판 위를 천천히 훑어 주세요")
    t0 = time.time(); n = 0
    while time.time() - t0 < sec:
        ok, f = cap.read()
        if not ok: break
        vw.write(f); n += 1
    cap.release(); vw.release()
    el = time.time() - t0
    print(f"끝: {n}프레임 / {el:.1f}초 = {n / el:.1f} fps → {path}")


def track(src):
    cap = cv2.VideoCapture(src)
    B = Board(); k = 0; t0 = time.time()
    while True:
        ok, f = cap.read()
        if not ok: break
        good = B.add(f)
        last = B.log[-1]
        if k % 10 == 0 or not good or last[0] != '추적':
            print(f"{k:4d} {last}")
        k += 1
    if not B.holes: sys.exit("한 프레임도 못 맞췄습니다")
    print(f"\n{k}프레임 / {time.time() - t0:.1f}초")
    print("검산:", B.check())
    print("타일:", B.tile_phase())
    import draw_board
    out = os.path.splitext(src)[0] + '_board.png'
    draw_board.draw(B, out)
    print(f"판 그림: {out}")


def board_overlay(f, Hm, i0, j0, phase=None):
    """판 900개 목심 자리(초록 점), 판 테두리(빨강), 타일 경계(노랑)를 그린다."""
    h, w = f.shape[:2]; out = f.copy()
    I, J = np.meshgrid(np.arange(i0, i0 + 30), np.arange(j0, j0 + 30), indexing='ij')
    P = cv2.perspectiveTransform(np.stack([I, J], -1).reshape(-1, 1, 2).astype(np.float32), Hm).reshape(30, 30, 2)
    q = cv2.perspectiveTransform(np.float32([[[i0 + 15, j0 + 15], [i0 + 16, j0 + 15]]]), Hm)[0]
    r = max(2, int(np.hypot(*(q[1] - q[0])) * 0.12))           # 점 크기 = 격자 간격의 약 1/8
    for p in P.reshape(-1, 2):
        if -r <= p[0] < w + r and -r <= p[1] < h + r:
            cv2.circle(out, (int(p[0]), int(p[1])), r, (0, 255, 0), -1)   # 속이 찬 점
    def line(a, b, col, t):
        pa = cv2.perspectiveTransform(np.float32([[a]]), Hm)[0, 0]; pb = cv2.perspectiveTransform(np.float32([[b]]), Hm)[0, 0]
        cv2.line(out, tuple(int(v) for v in pa), tuple(int(v) for v in pb), col, t)
    for k in (10, 20):                                         # 타일 경계 (구멍 사이 반 칸 자리)
        line((i0 + k - .5, j0 - .5), (i0 + k - .5, j0 + 29.5), (0, 255, 255), 2)
        line((i0 - .5, j0 + k - .5), (i0 + 29.5, j0 + k - .5), (0, 255, 255), 2)
    c = [(i0 - .5, j0 - .5), (i0 + 29.5, j0 - .5), (i0 + 29.5, j0 + 29.5), (i0 - .5, j0 + 29.5)]
    for a, b in zip(c, c[1:] + c[:1]): line(a, b, (0, 0, 255), 3)
    return out


def show(src, play=True):
    """녹화 영상에 판 목심(초록 원)을 덧그려 재생·저장. 1차: 추적, 2차: 그리기."""
    cap = cv2.VideoCapture(src); fps = cap.get(cv2.CAP_PROP_FPS) or 15
    B = Board(); frames, Hs = [], []
    while True:
        ok, f = cap.read()
        if not ok: break
        good = B.add(f); frames.append(f); Hs.append(B.H.copy() if good else None)
    if not B.holes: sys.exit("한 프레임도 못 맞췄습니다")
    i0, j0 = B.window()
    print("판:", B.check())
    out = os.path.splitext(src)[0] + '_overlay.mp4'
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (frames[0].shape[1], frames[0].shape[0]))
    for k, (f, Hm) in enumerate(zip(frames, Hs)):
        o = board_overlay(f, Hm, i0, j0) if Hm is not None else f.copy()
        mode = {'추적': 'track', '재탐색': 're-search'}.get(B.log[k][0] if k < len(B.log) else '', 'lost')
        cv2.putText(o, f"frame {k}  {mode}  board 30x30", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        vw.write(o)
        if play:
            cv2.imshow('board overlay', o)
            if (cv2.waitKey(int(1000 / fps)) & 0xFF) == 27: play = False; cv2.destroyAllWindows()
    vw.release()
    if play: cv2.waitKey(0); cv2.destroyAllWindows()
    print(f"저장: {out}")


if __name__ == '__main__':
    a = sys.argv[1:]
    if not a: sys.exit(__doc__)
    if a[0] == 'snap': snap(a[a.index('--image') + 1] if '--image' in a else None)
    elif a[0] == 'rec': rec(float(a[1]) if len(a) > 1 else 15)
    elif a[0] == 'track': track(a[1])
    elif a[0] == 'show': show(a[1], play='--noplay' not in a)
    else: sys.exit(__doc__)
