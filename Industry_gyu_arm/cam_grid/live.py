#!/usr/bin/env python3
"""실시간 카메라 미리보기 + 캡처 + 격자 판정 — 박민규님이 직접 실행.

    conda run --no-capture-output -n gyu_arm python live.py

  x (또는 ㅌ)  지금 화면 캡처        → shots/cap_시각.png
  ㅂ (또는 q)  마지막 캡처를 판정    → 격자 맞춤 결과 창 + shots/cap_시각_grid.png
               (캡처가 없으면 지금 화면을 캡처해서 판정)
  Esc / Ctrl+C 끝내기

키는 터미널에서 눌러도 되고(엔터 필요 없음), 영상 창을 클릭한 상태에서 눌러도 된다.
판정은 몇 초 걸리는데, 그동안에도 미리보기는 계속 돈다.
"""
import os, sys, time, threading, select, termios, tty
import cv2, numpy as np
from grid import detect_holes, fit_full, score
from cam import overlay, open_cam, SHOTS

CAP_KEYS = set('xXㅌ')
JUDGE_KEYS = set('qQㅂ')

state = {'frame': None, 'last_cap': None, 'last_path': None, 'result': None,
         'busy': False, 'quit': False, 'msg': 'x: 캡처   ㅂ: 판정   Esc: 끝'}
lock = threading.Lock()


def capture():
    with lock:
        f = None if state['frame'] is None else state['frame'].copy()
    if f is None: return None
    path = os.path.join(SHOTS, f"cap_{time.strftime('%m%d_%H%M%S')}.png")
    cv2.imwrite(path, f)
    with lock:
        state['last_cap'], state['last_path'] = f, path
        state['msg'] = f"캡처: {os.path.basename(path)}"
    print(f"\r[캡처] {path}", flush=True)
    return f


def judge():
    if state['busy']: print("\r[판정] 이미 판정 중입니다", flush=True); return
    with lock:
        img, path = state['last_cap'], state['last_path']
    if img is None:
        img = capture(); path = state['last_path']
        if img is None: return
    state['busy'] = True; state['msg'] = '판정 중...'

    def work():
        t0 = time.time()
        try:
            pts = detect_holes(img)
            if len(pts) < 12:
                msg = f"구멍이 너무 적습니다 ({len(pts)}개) — 판이 크게 들어오게, 밝게"
                res = img.copy()
            else:
                Hm = fit_full(pts, tries=min(len(pts), 120))
                if Hm is None:
                    msg = f"격자를 못 맞췄습니다 (구멍 후보 {len(pts)}개)"; res = img.copy()
                else:
                    n, err = score(Hm, pts)
                    msg = f"격자에 맞음 {n}/{len(pts)} ({n / len(pts):.0%})  오차 {err:.2f}px"
                    res = overlay(img, pts, Hm)
                    gp = os.path.splitext(path)[0] + '_grid.png'
                    cv2.imwrite(gp, res); msg += f"  → {os.path.basename(gp)}"
            msg += f"  ({time.time() - t0:.1f}초)"
        except Exception as e:
            msg = f"판정 실패: {e}"; res = img.copy()
        with lock:
            state['result'], state['msg'] = res, msg
        state['busy'] = False
        print(f"\r[판정] {msg}", flush=True)
    threading.Thread(target=work, daemon=True).start()


def key(ch):
    if ch in CAP_KEYS: capture()
    elif ch in JUDGE_KEYS: judge()
    elif ch == '\x1b': state['quit'] = True


def term_keys():
    """터미널 키를 엔터 없이 한 글자씩 받는다 (한글도)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not state['quit']:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key(sys.stdin.read(1))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    os.makedirs(SHOTS, exist_ok=True)
    cap = open_cam()
    if sys.stdin.isatty():
        threading.Thread(target=term_keys, daemon=True).start()
    print(__doc__.split('\n\n')[1], flush=True)
    t0, n, fps = time.time(), 0, 0.0
    try:
        while not state['quit']:
            ok, f = cap.read()
            if not ok:
                print("\r프레임을 못 받았습니다 — 연결이 끊겼는지 확인 (dmesg | tail)", flush=True)
                time.sleep(0.5); continue
            with lock: state['frame'] = f
            n += 1
            if time.time() - t0 >= 1.0: fps, n, t0 = n / (time.time() - t0), 0, time.time()
            show = f.copy()
            cv2.putText(show, f"{fps:.1f} fps   x: capture   q: judge   Esc: quit",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow('live (camera)', show)
            with lock: res = state['result']; state['result'] = None
            if res is not None: cv2.imshow('judge (grid)', res)
            k = cv2.waitKey(1) & 0xFF
            if k == 27: state['quit'] = True
            elif k != 255: key(chr(k))
    except KeyboardInterrupt:
        pass
    finally:
        state['quit'] = True
        cap.release(); cv2.destroyAllWindows()
        print("\n끝", flush=True)


if __name__ == '__main__':
    main()
