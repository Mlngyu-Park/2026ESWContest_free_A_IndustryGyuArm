#!/usr/bin/env python3
"""
base -> 1-1 -> 1 -> 1-1 -> base 시퀀스를 실제 로봇으로 실행.
- 영점(BASE_RAW) 기준 관절각(rad) -> raw -> 모터
- 안전속도 + 가감속, 부드러운 보간 이동
- Ctrl+C = 즉시 토크 OFF (긴급정지)
- 각 관절 방향부호 DIR (실물이 RViz와 반대면 -1)
- 측정 한계로 클램프 (안전)

실행: python play_seq_real.py         # 1회
      python play_seq_real.py --loop  # 반복
"""
import sys, time, math, signal
from dynamixel_sdk import PortHandler, PacketHandler

sys.stdout.reconfigure(line_buffering=True)
LOOP="--loop" in sys.argv

PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112; ADDR_GOAL=116; ADDR_PRESENT=132
SAFE_VEL=25; SAFE_ACCEL=8

MOTORS={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5',5:'j6'}
# 모터 ID -> POSES 조인트 index. j4<->j5 실제 배선 교차:
#   ID3 모터는 실제 j5 관절 -> j5각(index4), ID4 모터는 실제 j4 관절 -> j4각(index3)
ID2JOINT={0:0, 1:1, 2:2, 3:4, 4:3, 5:5}
# 영점 (base 자세 = 관절 0 rad), 2차 캡처값
ZERO_RAW={0:1517, 1:2483, 2:1562, 3:2244, 4:2545, 5:2018}
# 관절 방향부호 (실물이 RViz와 반대면 -1). 일단 전부 +1, 확인 후 조정.
DIR={0:-1, 1:1, 2:1, 3:1, 4:1, 5:1}   # j1 반전 (브리지 확인)
# 측정 한계 (raw) - 안전 클램프
# 2026-07-24 재측정 (j1~j4=ID0~3). j5,j6(ID4,5)는 미측정이라 기존값 유지.
LIMIT_RAW={0:(380,2571),1:(1479,3555),2:(1334,3382),3:(1271,3153),4:(1599,3524),5:(1970,3525)}

# 자세 관절각 (rad) - RViz IK 계산값
POSES={
    "base":[0.0,    0.0,    0.0,    0.0, 0.0,    0.0],
    # 1st_position 역산 후 jog로 교정한 1번 자세 (j1 -14.9->-18.1도)
    "1":   [-0.3161,0.3773,-0.3053,0.0,-0.6826, 0.0],
}
SEQ=["1","base"]   # 시작시 base로 간 뒤 base->1->base 반복
STEPS=50; STEP_DT=0.05   # 2.5초/구간

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def rad_to_raw(i,rad):
    raw=int(ZERO_RAW[i]+DIR[i]*rad*2048.0/math.pi)
    lo,hi=LIMIT_RAW[i]
    return max(lo,min(hi,raw))   # 안전 클램프
def get(i):
    p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT); return s32(p) if r==0 else None

def emergency_off(*_):
    print("\n[긴급정지] 토크 OFF")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for i in MOTORS: packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
    port.closePort(); sys.exit(0)
signal.signal(signal.SIGINT, emergency_off)

def ease(t): return t*t*(3-2*t)

def send(q):
    for i in MOTORS:
        packet.write4ByteTxRx(port,i,ADDR_GOAL, rad_to_raw(i,q[ID2JOINT[i]]) & 0xFFFFFFFF)

def move(a,b):
    """관절각 a -> b 를 조인트 순서대로(j1->j6) 하나씩 이동."""
    q=list(a)
    for j in range(6):
        if abs(b[j]-q[j])<0.005:      # 변화 거의 없으면 스킵 (약 0.3도)
            q[j]=b[j]; continue
        print(f"      j{j+1} 이동")
        start=q[j]
        for k in range(STEPS+1):
            t=ease(k/STEPS)
            q[j]=start+(b[j]-start)*t
            send(q)
            time.sleep(STEP_DT)
        time.sleep(0.2)

# 속도/가감속/토크
print(f"안전속도 {SAFE_VEL}, 가감속 {SAFE_ACCEL} + 토크 ON")
for i in MOTORS:
    packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,SAFE_ACCEL)
    packet.write4ByteTxRx(port,i,ADDR_P_VEL,SAFE_VEL)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)
time.sleep(0.3)

# 시작하면 무조건 먼저 base 자세로 이동 (현재 실제각 읽어서 base로)
print("먼저 base 자세로 이동 (Ctrl+C = 즉시 정지)")
cur_read=[0.0]*6
for i in MOTORS:
    p=get(i)
    ang=DIR[i]*((p if p is not None else ZERO_RAW[i])-ZERO_RAW[i])*math.pi/2048.0
    cur_read[ID2JOINT[i]]=ang   # 모터 ID -> 조인트 index 위치에 저장
move(cur_read, POSES["base"])
time.sleep(0.6)

print("시퀀스 시작: base->1->base" + (" (반복)" if LOOP else ""))
cur=POSES["base"]
while True:
    for name in SEQ:
        print(f"  -> {name}")
        move(cur, POSES[name]); cur=POSES[name]
        time.sleep(0.6)
    if not LOOP: break

print("완료. Ctrl+C로 토크 해제.")
try:
    while True: time.sleep(1)
except KeyboardInterrupt:
    emergency_off()
