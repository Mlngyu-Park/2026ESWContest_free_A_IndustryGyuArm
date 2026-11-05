#!/usr/bin/env python
"""물컵 규칙(그리퍼 수평) 전용 미세조정 — 터미널 조그.

물컵 규칙:
  · j1  (Z축)      기울기에 영향 없음  -> 여기서 안 건드림
  · j2,j3,j4 (X축) 셋의 '합' 이 곧 기울기 -> 합이 0 이어야 수평
  · j5  (Y축)      돌리면 무조건 기울어짐 -> 항상 0 고정 (편집 금지)
  · j6             그리퍼 여닫기, 자세 무관

그래서 이 도구는 **j2 / j3 / j4 만** 고르게 한다.
한 관절을 움직이면 나머지 둘 중 하나로 자동 상쇄해 '합 = 0' 을 유지한다
(= 팔 모양은 바뀌지만 그리퍼는 계속 수평).

조작:
  ↑ / ↓      : 조정할 관절 고르기 (j2 / j3 / j4)
  ← / →      : 그 관절 각도 -/+  (상쇄 관절이 자동으로 반대로 움직임)
  Tab        : 상쇄를 어느 관절이 받을지 바꾸기
  [ / ]      : 스텝 크기 (0.1 / 0.5 / 1 / 2 / 5도)
  space      : 지금 기울기를 강제로 0 으로 (수평 맞추기)
  s          : 이 보정값을 자세에 저장 (calib.json)
  r          : 보정 초기화 (전부 0)
  p          : 지금 상태 자세히 출력
  q / Ctrl+C : 종료 (토크 해제)

사용:
    ./level_jog.sh            # 기본: base 자세
    ./level_jog.sh 픽업위치    # 특정 자세
"""
import os, sys, math, signal, termios, tty, select, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

EDIT = [1, 2, 3]          # 물컵 규칙에 영향을 주는 관절 = j2,j3,j4 (0-based)
STEPS = [0.1, 0.5, 1.0, 2.0, 5.0]

_orig_term = None
def restore_term():
    if _orig_term is not None:
        try: termios.tcsetattr(sys.stdin, termios.TCSADRAIN, _orig_term)
        except Exception: pass

def getkey(timeout=0.1):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r: return None
    c = sys.stdin.read(1)
    if c == '\x1b':                       # 화살표 = ESC [ A/B/C/D
        r2, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not r2: return 'ESC'
        if sys.stdin.read(1) != '[': return 'ESC'
        return {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}.get(
            sys.stdin.read(1), None)
    return c

def main():
    pose = next((a for a in sys.argv[1:] if not a.startswith('-')), 'base')
    print('앱 로직 로드 중... (20초쯤)')
    import arm_studio_web as m

    if m.get_angles(pose) is None:
        print(f"'{pose}' 자세가 없습니다. 있는 것: {', '.join(m.pose_names())}")
        sys.exit(1)
    if not m.ROBOT_STS.connect():
        print(f'SO-ARM 연결 실패 ({m.PORT}) — USB/전원을 확인하세요')
        sys.exit(2)

    off = m.get_offset(pose)              # 보정값 (rad, 6개)
    sel = 0                               # EDIT 안에서의 선택 위치
    comp = 2                              # 상쇄를 받을 관절 (EDIT 안 위치)
    si = 2                                # 스텝 = 1도

    def tilt():
        """지금 기울기(도) = (자세+보정)의 j2+j3+j4 합."""
        a = m.get_angles(pose)
        return math.degrees(sum(a[j] + off[j] for j in EDIT))

    def real_tilt():
        """FK 로 잰 실제 기울기(도) — 공식이 맞는지 교차 확인용."""
        a = m.get_angles(pose)
        q = [x + y for x, y in zip(a, off)]
        try:
            _, r = m.fk(q)
            return math.degrees(math.acos(max(-1, min(1, abs(float(r[:, 2][2]))))))
        except Exception:
            return float('nan')

    def send():
        """실물로 전송 (3D 도 함께). j5 는 절대 안 건드림."""
        a = m.get_angles(pose)
        q = [x + y for x, y in zip(a, off)]
        q[m.J5_IDX] = a[m.J5_IDX]         # 물컵 규칙: j5 고정
        q[5] = m.GRIP_STATE['rad']        # 그리퍼 현재 상태 유지
        m.ros_publish(q)
        m.ROBOT_STS.time_profile(True); m.ROBOT_STS.torque(True)
        m.ROBOT_STS.send(q)

    def draw(msg=''):
        os.system('clear')
        t = tilt(); rt = real_tilt()
        print('=' * 62)
        print(f"  물컵 규칙 미세조정   자세: {pose}")
        print('=' * 62)
        print(f"  기울기 = j2+j3+j4 합 = {t:+7.2f}도   "
              f"{'✅ 수평' if abs(t) < 0.05 else '⚠ 기울어짐'}")
        print(f"  (FK 실측 확인: {rt:5.2f}도)")
        print()
        for k, j in enumerate(EDIT):
            cur = math.degrees(m.get_angles(pose)[j] + off[j])
            o = math.degrees(off[j])
            mark = '▶' if k == sel else ' '
            tagc = ' [상쇄]' if k == comp else ''
            print(f"  {mark} j{j+1}   각도 {cur:+8.2f}도   보정 {o:+6.2f}도{tagc}")
        print(f"    j1   (기울기 무관 — 여기서 조정 안 함)")
        print(f"    j5   0 고정 (돌리면 무조건 기울어짐 — 편집 금지)")
        print()
        print(f"  스텝: {STEPS[si]}도    ↑↓ 관절 · ←→ 조정 · Tab 상쇄관절 · "
              f"[ ] 스텝")
        print(f"  space 수평 맞추기 · s 저장 · r 초기화 · p 출력 · q 종료")
        if msg: print('\n  ' + msg)

    def cleanup(*_):
        restore_term()
        try: m.ROBOT_STS.estop()
        except Exception: pass
        print('\n토크 해제됨. 종료합니다.')
        sys.exit(0)
    signal.signal(signal.SIGINT, cleanup)

    global _orig_term
    _orig_term = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    send(); draw('시작 — 실물이 이 자세로 이동했습니다')

    try:
        while True:
            k = getkey()
            if k is None: continue
            msg = ''
            if k in ('q', 'Q'): cleanup()
            elif k == 'UP':   sel = (sel - 1) % len(EDIT)
            elif k == 'DOWN': sel = (sel + 1) % len(EDIT)
            elif k == '\t':
                comp = (comp + 1) % len(EDIT)
                if comp == sel: comp = (comp + 1) % len(EDIT)
                msg = f'상쇄 관절 -> j{EDIT[comp]+1}'
            elif k == '[': si = max(0, si - 1)
            elif k == ']': si = min(len(STEPS) - 1, si + 1)
            elif k in ('LEFT', 'RIGHT'):
                if sel == comp:            # 같은 관절이면 상쇄를 다른 데로
                    comp = (comp + 1) % len(EDIT)
                d = math.radians(STEPS[si]) * (1 if k == 'RIGHT' else -1)
                j, jc = EDIT[sel], EDIT[comp]
                off[j] += d
                off[jc] -= d               # 합 유지 = 수평 유지
                send()
                msg = (f'j{j+1} {"+" if d>0 else "-"}{STEPS[si]}도 '
                       f'(j{jc+1} 자동 상쇄 — 수평 유지)')
            elif k == ' ':
                t = sum(m.get_angles(pose)[j] + off[j] for j in EDIT)
                off[EDIT[comp]] -= t       # 기울기를 상쇄 관절로 몰아 0 으로
                send()
                msg = f'기울기를 0 으로 맞췄습니다 (j{EDIT[comp]+1} 로 상쇄)'
            elif k in ('s', 'S'):
                m.set_offset(pose, off)
                m._macro_save()
                msg = (f"'{pose}' 보정 저장됨: "
                       + '  '.join(f'j{i+1}:{math.degrees(off[i]):+.2f}'
                                   for i in EDIT))
            elif k in ('r', 'R'):
                off = [0.0] * 6; send(); msg = '보정 초기화 (전부 0)'
            elif k in ('p', 'P'):
                a = m.get_angles(pose)
                q = [x + y for x, y in zip(a, off)]
                tp = m.tool_pos(q)
                msg = ('관절(도): '
                       + ' '.join(f'j{i+1}:{math.degrees(q[i]):+.1f}'
                                  for i in range(5))
                       + f'\n  툴 위치: ({tp[0]*1000:.1f}, {tp[1]*1000:.1f}, '
                         f'{tp[2]*1000:.1f})mm')
            draw(msg)
    finally:
        restore_term()

if __name__ == '__main__':
    main()
