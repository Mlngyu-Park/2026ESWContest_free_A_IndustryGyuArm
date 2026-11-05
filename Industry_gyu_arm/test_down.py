#!/usr/bin/env python
"""하강 높이 검증 — 플레이트+도킹+펜 결합 상태에서 '하강 직후' 모습을 만든다.

사용자 고정 치수 (2026-08-14 확정):
    원점(플레이트 최하단) -> 도킹 윗면      = 115mm
    도킹 윗면 -> 그리퍼가 닿는 펜 면        = +10mm
    그 면 -> cr_7(그리퍼 기준점)            = +37mm
    => 하강했을 때 cr_7 높이 = 115+10+37 = 162mm

  ① 1번공정 배치 로드 (플레이트 + 도킹 + 펜 결합)
  ② 로봇 자리 확인 (안 닿으면 자동 이동)
  ③ 이동(접근점) -> 그리퍼 열기 -> 수직 하강까지만 실행하고 멈춘다
     (닫기 안 함 — 그리퍼가 펜 어디에 서 있는지 눈으로 확인하는 용도)

사용:
    ./test_down.sh     # 실행 후 3D 뷰어에서 확인 (주소는 아래 출력)
"""
import os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def main():
    print('앱 로직 로드 중... (20초쯤)')
    import arm_studio_web as m
    m.EXEC['mode'] = 'sim'

    p = (m.S.get('factory_presets') or {}).get('1번공정')
    if not p:
        print("'1번공정' 프리셋이 없습니다."); sys.exit(1)
    m._factory_apply(p)

    bid = 'ppap'
    b = m.BRICKS[bid]
    dock = 'Industry_gyu_arm_20260727_-_docking_station-5#3'
    dock_top = float(m.docking_top(dock)[2])
    gp = m.brick_grasp_point(bid)

    print('=' * 62)
    print('[치수표 — 전부 원점(플레이트 최하단) 기준, mm]')
    print(f"  도킹 윗면              : {dock_top*1000:7.2f}  (규칙: 115)")
    print(f"  그리퍼가 닿는 펜 면    : {(dock_top+m.GRASP_FACE_DZ)*1000:7.2f}  (도킹 윗면 +10)")
    print(f"  하강 목표 cr_7 높이    : {gp[2]*1000:7.2f}  (+37 => 규칙 162)")
    print(f"  펜 꼭대기              : {(b['pos'][2]+b['h'])*1000:7.2f}")
    print('=' * 62)

    ok, _ = m.brick_graspable(bid)
    if not ok:
        s = m.h_robot_suggest()
        print('자리 이동 필요 ->', s)
        print(m.h_robot_apply_suggest())

    q = m.goto_q(dock)
    if q is None:
        print('접근점 IK 실패 — [로봇 자리 추천] 필요'); sys.exit(2)
    m.ros_publish(q)
    print('\n① 접근점 도착:', [round(float(v)*1000,1) for v in m.tool_pos(m.CURRENT_Q)], 'mm')
    gq = list(m.CURRENT_Q); gq[5] = m._j6_rad(True)
    m.ros_publish(gq, with_grip=True)
    print('② 그리퍼 열기')
    msg = m.do_descend_here()
    print('③', msg)

    tp = m.tool_pos(m.CURRENT_Q)
    print('\n[하강 직후 상태]')
    print(f"  cr_7 실제 높이     : {tp[2]*1000:.2f} mm  (목표 162)")
    print(f"  도킹 윗면과 여유   : {(tp[2]-dock_top)*1000:+.2f} mm  (규칙 +47)")
    print('\n3D 뷰어에서 그리퍼가 펜 어디에 서 있는지 확인하세요:'
          f'  http://localhost:{m.VIEWER_PORT}/')
    print('끝내려면 Ctrl+C')
    try:
        while True: time.sleep(3600)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
