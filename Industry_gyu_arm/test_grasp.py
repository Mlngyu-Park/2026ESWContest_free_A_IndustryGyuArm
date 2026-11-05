#!/usr/bin/env python
"""펜 잡기 검증 — 명령어 하나로 끝.

펜이 어디에 있든, 로봇이 어디에 있든:
  ① 배치 확인(비었으면 '이전 배치' 자동 로드)
  ② 예전에 정한 잡기 규칙 전부 확인·출력
  ③ 도달 불가면 -> 로봇 자리를 자동으로 찾아 최소 이동으로 옮김 (시뮬)
  ④ 잡기 5단계 실행

사용:
    ./test_grasp.sh              # 이거 하나면 됨 (시뮬레이션)
    ./test_grasp.sh --real       # 실물 로봇 (자리 이동은 제안만 하고 멈춤)
"""
import os, sys, time, math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def find_robot_spot(m, p):
    """타깃 p 를 잡을 수 있는 로봇 자리들(2핀=목심, 4회전) 중 최소 이동 순서로."""
    import numpy as np
    pins = m.robot_pins_local()
    pa = np.asarray(pins[0][:2], float); pb = np.asarray(pins[1][:2], float)
    pegs = m._pegs_all_world()
    pegset = {(round(x, 3), round(y, 3)) for x, y in pegs}
    cands = []
    for g1 in pegs:
        g1 = np.asarray(g1, float)
        for k in range(4):
            yaw = k * math.pi / 2
            g2 = g1 + m._rz2(yaw) @ (pb - pa)
            if (round(float(g2[0]), 3), round(float(g2[1]), 3)) not in pegset:
                continue
            off = g1 - m._rz2(yaw) @ pa
            if not m._reach_at(off, yaw, p):
                continue
            d = math.hypot(off[0]-m.BASE_OFF[0], off[1]-m.BASE_OFF[1])
            cands.append((d, float(off[0]), float(off[1]), float(yaw)))
    cands.sort()
    return cands

def main():
    real = '--real' in sys.argv
    want = next((a for a in sys.argv[1:] if not a.startswith('-')), None)

    print('앱 로직 로드 중... (20초쯤)')
    import arm_studio_web as m

    m.EXEC['mode'] = 'sts' if real else 'sim'
    print(f"\n모드: {'실물 (SO-ARM)' if real else '시뮬레이션'}"
          f"  ·  3D 뷰어: http://localhost:{m.VIEWER_PORT}/\n")

    if not any(b.get('pos') for b in m.BRICKS.values()):
        p = (m.S.get('factory_presets') or {}).get('이전 배치')
        if p:
            m._factory_apply(p)
            print("배치가 비어 있어 '이전 배치'를 불러왔습니다")

    placed = [bid for bid, b in m.BRICKS.items() if b.get('pos')]
    bid = want if (want and want in m.BRICKS) else (placed[0] if placed else None)
    if not bid or m.BRICKS[bid].get('pos') is None:
        print('배치된 물체가 없습니다. ARM STUDIO 에서 펜을 도킹에 꽂고 다시 실행하세요.')
        sys.exit(1)

    b = m.BRICKS[bid]
    gp = m.brick_grasp_point(bid)
    mate = m._dock_mate()
    print('=' * 60)
    print(f"대상: {bid}  위치 ({b['pos'][0]*1000:.0f},{b['pos'][1]*1000:.0f})mm"
          f"  바닥 z={b['pos'][2]*1000:.0f}mm")
    print('[규칙 확인 - 예전에 정한 것들]')
    print( "  · 잡는 높이  = [고정 치수] 도킹 결합 시 cr_7 = 도킹 윗면(115)"
           " +10 +37 = 162mm / 도킹 밖이면 바닥 +"
          f"{b['grasp_dz']*1000:.2f}mm")
    print(f"  · 도킹 메이트 = "
          + (f"소켓 {mate['socket_xy_mm']}mm · 꽂힘 깊이 {mate['seat_dz_mm']}mm OK"
             if mate else '없음(근사) - pick_mate.sh 로 정하면 정확해짐'))
    print(f"  · 잡는 점    = ({gp[0]*1000:.0f},{gp[1]*1000:.0f},{gp[2]*1000:.0f})mm")
    print( "  · 축 일치    = 펜 중심축 <-> 그리퍼 축(axis_7)   (잡기 ③단계)")
    print( "  · 수평 유지  = 물컵 규칙, 경로 전체")
    print(f"  · 접근 여유  = 상승/하강 {m.GRASP_LIFT*1000:.0f}mm")
    print( "  · 5단계      = 상승 -> 열기 -> 축일치 -> 수직하강 -> 닫기")
    print('=' * 60)

    ok, q = m.brick_graspable(bid)
    if not ok:
        print(f"현재 로봇 자리({m._robot_pos_str()})에선 도달 불가 -> 자리 탐색...")
        cands = find_robot_spot(m, [float(gp[0]), float(gp[1]), float(gp[2])])
        if not cands:
            print('어떤 로봇 자리에서도 이 펜에 도달할 수 없습니다.')
            sys.exit(2)
        moved = False
        for d, x, y, yaw in cands[:20]:        # 표 판정 후 IK 로 최종 확인
            m.set_robot_pose(x, y, yaw, save=False)
            ok, q = m.brick_graspable(bid)
            if ok:
                print(f"로봇 자동 이동(최소 {d*1000:.0f}mm): {m._robot_pos_str()}")
                moved = True
                break
        if not moved:
            print('표에서는 가능해 보였지만 IK 최종 확인 실패 — 펜을 옮겨보세요.')
            sys.exit(2)
        if real:
            print('\n[실물 모드] 실제 로봇을 위 자리로 옮긴 뒤 다시 실행하세요.'
                  ' (실물 베이스는 손으로 옮겨야 합니다)')
            sys.exit(0)

    if real:
        input('실물 로봇이 움직입니다. 엔터를 누르면 잡기 시작: ')

    # 시작 자세 정리 (시뮬): 임의 시작점(예: 기둥 바로 위)에서는 ① 수직 상승이
    # 기구학적으로 불가능할 수 있다. 실제 사용 흐름처럼 접근점(펜 위 여유높이)
    # 으로 EE 를 먼저 보내고 잡기 5단계를 시작한다.
    if not real:
        q0 = list(m.CURRENT_Q)
        appr = m.ik_tool((float(gp[0]), float(gp[1]),
                          float(gp[2]) + m.GRASP_LIFT),
                         m._seed_list(q0), q0[4], q0[5])
        if appr is not None:
            m.ros_publish(appr)
            print(f"시작 자세: 펜 위 {m.GRASP_LIFT*1000:.0f}mm 접근점으로 이동")
        else:
            print('경고: 접근점 IK 실패 - 현재 자세에서 바로 시도합니다')

    t0 = time.time()
    print('\n잡기 실행...')
    res = m.do_grasp(bid)
    dt = time.time() - t0
    held = (m.HELD.get('brick') == bid)
    print(f"\n결과 ({dt:.1f}초): {res}")
    print(f"들고 있음: {'예' if held else '아니오'}")
    print('성공! (시뮬이면 3D 뷰어에서 확인)' if held else '실패 - 위 메시지 확인')
    sys.exit(0 if held else 3)

if __name__ == '__main__':
    main()
