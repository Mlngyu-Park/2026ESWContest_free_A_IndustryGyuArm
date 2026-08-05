#!/usr/bin/env python3
"""
각 모터의 속도 한계(Velocity Limit) 읽기.
- Velocity Limit (addr 44): 이 모터가 낼 수 있는 최대 속도 (0.229 rev/min 단위)
- Profile Velocity (addr 112): 현재 이동 프로파일 속도 설정
XM/XC/XL 430 계열은 속도 단위 = 0.229 rev/min.
"""
from dynamixel_sdk import PortHandler, PacketHandler

PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_VELOCITY_LIMIT   = 44   # 4byte, 최대 속도 한계
ADDR_PROFILE_VELOCITY = 112  # 4byte, 현재 프로파일 속도
MOTORS={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5',5:'j6'}

UNIT_RPM = 0.229            # rev/min per unit (X-series)


def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v


def main():
    port=PortHandler(PORT); packet=PacketHandler(2.0)
    if not port.openPort(): print("포트 열기 실패"); return
    port.setBaudRate(BAUD)

    print("="*72)
    print(f"{'조인트':<6}{'ID':<4}{'VelLimit(raw)':>14}{'최대속도(rpm)':>14}{'rad/s':>10}{'Profile':>9}")
    print("-"*72)
    for i,name in MOTORS.items():
        vl,r1,_=packet.read4ByteTxRx(port,i,ADDR_VELOCITY_LIMIT)
        pv,r2,_=packet.read4ByteTxRx(port,i,ADDR_PROFILE_VELOCITY)
        if r1!=0:
            print(f"{name:<6}{i:<4} 읽기 실패"); continue
        vl=s32(vl)
        rpm = vl*UNIT_RPM
        rad_s = rpm*2*3.14159265/60.0
        pv_s = pv if r2==0 else '?'
        pv_note = "(0=제한없음)" if pv==0 else ""
        print(f"{name:<6}{i:<4}{vl:>14}{rpm:>14.1f}{rad_s:>10.2f}{str(pv_s):>9} {pv_note}")
    print("="*72)
    print("VelLimit = 모터가 낼 수 있는 최대 속도(하드웨어 상한).")
    print("Profile Velocity = 실제 이동 시 속도 (0이면 VelLimit까지 최대).")
    print("최저 속도는 별도 레지스터가 없음 - Profile Velocity를 작게 주면 느려짐(최저 사실상 0).")
    port.closePort()


if __name__=='__main__':
    main()
