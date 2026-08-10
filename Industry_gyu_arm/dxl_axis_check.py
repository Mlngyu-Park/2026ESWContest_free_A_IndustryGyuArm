#!/usr/bin/env python3
"""Dynamixel 조인트↔모터 매핑과 회전 방향 확인 (안전 우선)

한 관절씩 아주 조금(기본 3도)만 움직여, 어느 모터가 어느 축을 담당하고
방향 부호가 맞는지 눈으로 확인한다. 매 단계마다 Enter 를 기다리므로
언제든 Ctrl+C 로 멈출 수 있다.

사용:
    conda run --no-capture-output -n gyu_arm python dxl_axis_check.py
    conda run --no-capture-output -n gyu_arm python dxl_axis_check.py --deg 5

확인 방법:
  - "j2 를 +3도" 라고 하면 그 관절만 움직여야 한다. 다른 데가 움직이면 매핑 오류
  - 화면 안내와 실제 방향(위/아래)이 반대면 DIR 부호가 틀린 것
결과는 dxl_profile.py 의 J2ID / DIR 을 고치는 근거가 된다.
"""
import sys, os, time, math, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dynamixel_sdk import PortHandler, PacketHandler
from dxl_profile import PORT, BAUD, J2ID, DIR, ZERO_RAW, LIMIT_RAW

A_TORQUE, A_GOAL, A_PRESENT, A_PVEL, A_PACC, A_DRIVE = 64, 116, 132, 112, 108, 10

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--deg', type=float, default=3.0, help='한 번에 움직일 각도')
    a = ap.parse_args()

    p = PortHandler(PORT)
    if not p.openPort(): raise SystemExit(f"포트 열기 실패: {PORT}")
    p.setBaudRate(BAUD)
    pk = PacketHandler(2.0)
    ids = sorted(set(J2ID))

    def w1(i, ad, v): pk.write1ByteTxRx(p, i, ad, int(v))
    def w4(i, ad, v): pk.write4ByteTxRx(p, i, ad, int(v) & 0xFFFFFFFF)
    def r4(i, ad):
        v, r, _ = pk.read4ByteTxRx(p, i, ad); return v if r == 0 else None

    print(f"포트 {PORT}\n조인트→모터 매핑 (dxl_profile.J2ID): "
          + '  '.join(f"j{k+1}→ID{J2ID[k]}" for k in range(len(J2ID))))
    print("방향 부호 (DIR): " + '  '.join(f"ID{i}:{DIR[i]:+d}" for i in ids))
    print(f"\n각 관절을 {a.deg}도씩만 움직입니다. 각 단계에서 Enter, 중단은 Ctrl+C\n")

    # 느리게 (시간기반 프로파일). 토크 끄기 전에 설정하고 곧바로 켠다.
    for i in ids:
        w1(i, A_TORQUE, 0); w1(i, A_DRIVE, 4)
        w4(i, A_PVEL, 1500); w4(i, A_PACC, 500)
    for i in ids: w1(i, A_TORQUE, 1)
    time.sleep(0.3)

    try:
        for k in range(len(J2ID) - 1):        # 마지막(그리퍼) 제외
            i = J2ID[k]
            cur = r4(i, A_PRESENT)
            if cur is None:
                print(f"j{k+1} (ID{i}): 읽기 실패 - 건너뜀"); continue
            step = int(DIR[i] * a.deg * 2048.0 / 180.0)
            tgt = cur + step
            lo, hi = LIMIT_RAW[i]
            if not (lo <= tgt <= hi):
                print(f"j{k+1} (ID{i}): 한계에 걸려 건너뜀 "
                      f"(현재 {cur}, 목표 {tgt}, 한계 {lo}~{hi})")
                continue
            input(f"j{k+1} (모터 ID{i}) 를 {a.deg:+.1f}도 → Enter: ")
            w4(i, A_GOAL, tgt)
            time.sleep(1.5)
            now = r4(i, A_PRESENT)
            print(f"   raw {cur} → {now}  (목표 {tgt})")
            input("   이 관절만 움직였나요? 방향이 맞나요? Enter 로 원위치: ")
            w4(i, A_GOAL, cur); time.sleep(1.5)
            print("   원위치 완료\n")
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        print("토크는 켜둡니다 (팔이 떨어지지 않게). 끄려면 웹에서 [토크 해제]")
        p.closePort()

if __name__ == '__main__':
    main()
