#!/usr/bin/env python3
"""
현재 자세가 어디든 -> base 자세(영점)로 천천히 이동.
- 안전속도 + 가감속 (부드럽게)
- Ctrl+C 누르면 즉시 토크 OFF (안전정지)
"""
import sys, time, signal
from dynamixel_sdk import PortHandler, PacketHandler

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112; ADDR_GOAL=116; ADDR_PRESENT=132

# base 자세 raw (= 영점, 2차 캡처값)
BASE_RAW={0:1517, 1:2483, 2:1562, 3:2244, 4:2545, 5:2018}

SAFE_VEL=25      # 안전속도 (작을수록 느림, 0.229rpm 단위. 25≈5.7rpm)
SAFE_ACCEL=8     # 부드러운 가감속
POS_TOL=20
TIMEOUT=25.0

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def get(i):
    p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT); return s32(p) if r==0 else None

def emergency_off(*_):
    print("\n[긴급정지] 토크 OFF")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for i in BASE_RAW: packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
    port.closePort()
    sys.exit(0)
signal.signal(signal.SIGINT, emergency_off)

# 속도/가감속/토크 ON
print(f"안전속도 {SAFE_VEL}, 가감속 {SAFE_ACCEL} 설정 + 토크 ON")
for i in BASE_RAW:
    packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,SAFE_ACCEL)
    packet.write4ByteTxRx(port,i,ADDR_P_VEL,SAFE_VEL)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)
time.sleep(0.3)

# base로 이동 (j1->j6 순서, 하나씩 천천히)
print("base 자세로 이동 시작 (Ctrl+C = 즉시 정지)")
for i in sorted(BASE_RAW):
    goal=BASE_RAW[i]
    cur=get(i)
    if cur is None: print(f"  j{i+1} 읽기실패 스킵"); continue
    print(f"  j{i+1}: {cur} -> {goal}")
    packet.write4ByteTxRx(port,i,ADDR_GOAL, goal & 0xFFFFFFFF)
    t0=time.time()
    while True:
        p=get(i)
        if p is not None and abs(p-goal)<=POS_TOL:
            print(f"    j{i+1} 도착"); break
        if time.time()-t0>TIMEOUT:
            print(f"    j{i+1} 타임아웃(현재 {p})"); break
        time.sleep(0.1)

print("\nbase 자세 완료. 토크 유지 중. Ctrl+C로 토크 해제.")
try:
    while True: time.sleep(1)
except KeyboardInterrupt:
    emergency_off()
