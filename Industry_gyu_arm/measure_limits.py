#!/usr/bin/env python3
"""
관절 한계 측정.
- 시작 시 전체 모터 토크 OFF (손으로 돌릴 수 있게)
- 6개 모터를 실시간 추적하며 각 모터의 raw 최소/최대값 갱신
- 손으로 각 관절을 양방향 끝까지 천천히 왕복시키면 min/max가 기록됨
- Ctrl+C 로 종료하면 측정된 한계를 raw + rad로 출력

rad 변환은 ZERO_RAW(현재 영점) 기준. 영점이 안 맞으면 raw만 참고해도 됨.
"""
import sys
import time
import signal
from dynamixel_sdk import PortHandler, PacketHandler

# 실시간 출력이 버퍼에 안 갇히게 (conda run 등에서 멈춘 것처럼 보이는 문제 방지)
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64
ADDR_PRESENT_POSITION=132
MOTORS={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5',5:'j6'}

# 현재 확정 base 영점 (rad 해석용). raw 값만 봐도 됨.
ZERO_RAW={0:1517,1:2483,2:1562,3:2244,4:2545,5:2018}

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def raw_to_rad(z,raw): return (raw-z)*(3.14159265/2048.0)

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 열기 실패"); raise SystemExit
port.setBaudRate(BAUD)

# 토크 전부 OFF
for i in MOTORS:
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
print("토크 OFF. 이제 손으로 각 관절을 양쪽 끝까지 천천히 돌리세요.")
print("모든 관절을 다 돌린 뒤 Ctrl+C 로 종료 → 한계 출력.\n")

mn={i:None for i in MOTORS}
mx={i:None for i in MOTORS}

def finish(*_):
    print("\n\n================ 측정된 한계 ================")
    print(f"{'조인트':<6}{'min_raw':>9}{'max_raw':>9}{'범위(도)':>10}")
    for i,name in MOTORS.items():
        if mn[i] is None:
            print(f"{name:<6}  측정 안됨"); continue
        span=(mx[i]-mn[i])*360.0/4096.0
        print(f"{name:<6}{mn[i]:>9}{mx[i]:>9}{span:>10.1f}")
    print("\n--- URDF용 rad (현재 영점 기준, 참고) ---")
    for i,name in MOTORS.items():
        if mn[i] is None: continue
        lo=raw_to_rad(ZERO_RAW[i],mn[i]); hi=raw_to_rad(ZERO_RAW[i],mx[i])
        print(f"{name}: lower={lo:.3f}  upper={hi:.3f}   (raw {mn[i]}~{mx[i]})")
    port.closePort()
    raise SystemExit

signal.signal(signal.SIGINT, finish)

last_print=0
while True:
    line=[]
    for i,name in MOTORS.items():
        p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT_POSITION)
        if r!=0:
            line.append(f"{name}:--")
            continue
        p=s32(p)
        if mn[i] is None or p<mn[i]: mn[i]=p
        if mx[i] is None or p>mx[i]: mx[i]=p
        line.append(f"{name}:{p}({mn[i]}~{mx[i]})")
    now=time.time()
    if now-last_print>0.3:
        print("\r"+"  ".join(line)+"        ",end="",flush=True)
        last_print=now
    time.sleep(0.02)
