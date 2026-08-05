#!/usr/bin/env python3
"""
안전 속도로 기본자세(영점)로 이동.
- Profile Velocity 를 낮게 설정 (안전 속도)
- 각 모터를 영점(ZERO_RAW)으로 천천히 이동
- 경계(0/4095) 넘지 않게 가까운 방향으로 이동
Ctrl+C 시 즉시 정지.
"""
import sys, time, signal
from dynamixel_sdk import PortHandler, PacketHandler

sys.stdout.reconfigure(line_buffering=True)

PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64
ADDR_PROFILE_ACCEL=108
ADDR_PROFILE_VELOCITY=112
ADDR_GOAL_POSITION=116
ADDR_PRESENT_POSITION=132

MOTORS={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5',5:'j6'}

# 2026-07-20 캡처한 기본자세(영점)
HOME_RAW={0:2406,1:1441,2:1430,3:2319,4:2532,5:538}

SAFE_PROFILE_VELOCITY = 30   # 안전 속도 (작을수록 느림. 단위 0.229rpm. 30≈6.9rpm)
# Profile Acceleration: 출발 시 서서히 가속 + 도착 시 서서히 감속(사다리꼴).
# 작을수록 더 부드럽게 가감속. 0이면 급출발/급정지(충격 발생).
SAFE_PROFILE_ACCEL = 10
POSITION_TOLERANCE = 15
TIMEOUT_SEC = 20.0

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 열기 실패"); raise SystemExit
port.setBaudRate(BAUD)

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def get_pos(i):
    p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT_POSITION)
    return s32(p) if r==0 else None

def stop_and_exit(*_):
    print("\n[정지] 현재 위치 고정 후 종료")
    for i in MOTORS:
        p=get_pos(i)
        if p is not None:
            packet.write4ByteTxRx(port,i,ADDR_GOAL_POSITION,p & 0xFFFFFFFF)
    port.closePort()
    raise SystemExit
signal.signal(signal.SIGINT, stop_and_exit)

# 1) 안전 속도 + 가감속 프로파일 설정 + 토크 ON
print(f"속도={SAFE_PROFILE_VELOCITY}, 가감속={SAFE_PROFILE_ACCEL} 설정 + 토크 ON")
for i in MOTORS:
    packet.write4ByteTxRx(port,i,ADDR_PROFILE_ACCEL,SAFE_PROFILE_ACCEL)     # 부드러운 가감속
    packet.write4ByteTxRx(port,i,ADDR_PROFILE_VELOCITY,SAFE_PROFILE_VELOCITY)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)
time.sleep(0.3)

# 2) 기본자세로 이동 (한 모터씩 순서대로, 안전하게)
print("기본자세로 이동 시작 (j1→j6)...")
for i in MOTORS:
    goal=HOME_RAW[i]
    cur=get_pos(i)
    if cur is None:
        print(f"  {MOTORS[i]} 읽기 실패, 스킵"); continue
    print(f"  {MOTORS[i]}: {cur} → {goal}")
    packet.write4ByteTxRx(port,i,ADDR_GOAL_POSITION,goal & 0xFFFFFFFF)
    start=time.time()
    while True:
        p=get_pos(i)
        if p is not None and abs(p-goal)<=POSITION_TOLERANCE:
            print(f"    {MOTORS[i]} 도착"); break
        if time.time()-start>TIMEOUT_SEC:
            print(f"    {MOTORS[i]} 타임아웃(현재 {p})"); break
        time.sleep(0.1)

print("\n기본자세 이동 완료. 토크는 유지됨(자세 고정).")
print("종료하려면 Ctrl+C. (원하면 토크 해제 코드 추가 가능)")
# 자세 유지하며 대기
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    stop_and_exit()
