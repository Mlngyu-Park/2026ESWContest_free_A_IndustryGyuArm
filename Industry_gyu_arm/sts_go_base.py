#!/usr/bin/env python3
"""
어떤 자세에 있든 base(sts_zero.py) 로 천천히 복귀. RViz 불필요.

안전:
  - 이동 전 각 관절의 이동량을 먼저 보여주고, 큰 이동이 있으면 확인을 받음
  - 관절 하나씩 순서대로, 아주 천천히 (사이클당 약 1.3도)
  - 측정 한계 안으로 클램프 (모터 펌웨어 한계도 이중으로 걸려 있음)
  - Ctrl+C = 즉시 토크 OFF

실행: python sts_go_base.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
from sts_zero import ZERO_RAW
from sts_limits import LIMIT_RAW

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_TORQUE=40; ADDR_ACC=41; ADDR_GOAL=42; ADDR_SPEED=46
ADDR_TORQUE_LIMIT=48; ADDR_PRESENT=56; ADDR_MODE=33
IDS=[1,2,3,4,5,6]

SAFE_SPEED=150      # 1.5배속
SAFE_ACC=8
TORQUE_LIMIT=250
STEP=15                 # 사이클당 raw (약 1.3도)
DT=0.06
LIMIT_MARGIN=25
BIG_MOVE=350            # 이보다 크면(약 30도) 확인 요청
MAX_HOLD_ERR=10

port=PortHandler(PORT)
if not port.openPort(): print(f"포트 실패: {PORT}"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(i,addr,size=2,tries=4):
    for _ in range(tries):
        try:
            time.sleep(0.05)
            v,r,_=(packet.read2ByteTxRx(port,i,addr) if size==2
                   else packet.read1ByteTxRx(port,i,addr))
            if r==COMM_SUCCESS: return v
        except Exception: pass
        time.sleep(0.05)
    return None
def wr2(i,addr,v):
    for _ in range(3):
        try:
            if packet.write2ByteTxRx(port,i,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.05)
    return False
def wr1(i,addr,v):
    for _ in range(3):
        try:
            if packet.write1ByteTxRx(port,i,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.05)
    return False

_stop=False
def emergency_off(*_):
    global _stop
    _stop=True
    print("\n[긴급정지] 토크 OFF")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for i in IDS: wr1(i,ADDR_TORQUE,0)
    try: port.closePort()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT, emergency_off)

# 1) 현재 위치 읽고 이동량 미리 보고
print("현재 위치 확인 중...\n")
cur={}; plan=[]
for i in IDS:
    p=rd(i,ADDR_PRESENT)
    if p is None:
        print(f"  ID{i}: 읽기 실패 - 중단"); emergency_off()
    cur[i]=p
    lo,hi=LIMIT_RAW[i]
    tgt=max(lo+LIMIT_MARGIN,min(hi-LIMIT_MARGIN,ZERO_RAW[i]))
    d=tgt-p
    plan.append((i,p,tgt,d))
    print(f"  ID{i}: {p} -> {tgt}   이동 {d:+5d} raw ({d*180/2048:+6.1f}도)"
          + ("   <-- 큰 이동" if abs(d)>BIG_MOVE else ""))

big=[i for i,_,_,d in plan if abs(d)>BIG_MOVE]
print()
if big:
    print(f"주의: ID {big} 의 이동량이 {BIG_MOVE*180/2048:.0f}도를 넘습니다.")
    print("팔 주변을 비우고, 케이블이 꼬이지 않는지 확인하세요.")
try:
    input("이동하려면 Enter (취소는 Ctrl+C): ")
except KeyboardInterrupt:
    emergency_off()

# 2) 관절별: 안전설정 -> 현재위치 즉시 고정 -> 토크 ON
print("\n현재 자세로 고정 중...")
held=[]
for i in IDS:
    wr1(i,ADDR_MODE,0)
    wr2(i,ADDR_TORQUE_LIMIT,TORQUE_LIMIT)
    wr1(i,ADDR_ACC,SAFE_ACC)
    wr2(i,ADDR_SPEED,SAFE_SPEED)
    p=rd(i,ADDR_PRESENT)
    if p is None: print(f"  ID{i}: 읽기 실패 - 건너뜀"); continue
    wr2(i,ADDR_GOAL,p)
    p2=rd(i,ADDR_PRESENT)
    err=abs(p2-p) if p2 is not None else 999
    if err>MAX_HOLD_ERR:
        print(f"  ID{i}: 흔들림 {err} raw - 토크 안 켬"); continue
    wr1(i,ADDR_TORQUE,1)
    cur[i]=p; held.append(i)
    print(f"  ID{i}: 고정 (pos {p})")

if not held:
    print("\n고정된 관절이 없습니다."); emergency_off()

# 3) 관절 순서대로 base 로 천천히
print(f"\nbase 로 이동 (관절 순서대로). Ctrl+C = 즉시 정지\n")
for i in held:
    p0=cur[i]
    lo,hi=LIMIT_RAW[i]
    tgt=max(lo+LIMIT_MARGIN,min(hi-LIMIT_MARGIN,ZERO_RAW[i]))
    diff=tgt-p0
    if abs(diff)<5:
        print(f"  ID{i}: 이미 base"); continue
    n=max(1,int(abs(diff)/STEP))
    print(f"  ID{i}: 이동 {diff*180/2048:+.1f}도 ...",end="",flush=True)
    for k in range(1,n+1):
        if _stop: emergency_off()
        wr2(i,ADDR_GOAL,int(p0+diff*k/n))
        time.sleep(DT)
    wr2(i,ADDR_GOAL,tgt)
    time.sleep(0.4)
    p=rd(i,ADDR_PRESENT)
    print(f" 도착 (pos {p}, 목표 {tgt})")

print("\nbase 자세 완료. 토크 유지 중.")
print("Ctrl+C = 토크 해제\n")
try:
    while True:
        s=[]
        for i in IDS:
            p=rd(i,ADDR_PRESENT)
            s.append(f"ID{i}:{p if p is not None else '--'}({(p-ZERO_RAW[i]) if p is not None else 0:+d})")
        print("\r  "+"  ".join(s)+"   ",end="",flush=True)
        time.sleep(0.5)
except KeyboardInterrupt:
    emergency_off()
