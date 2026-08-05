#!/usr/bin/env python3
"""
ID6 회전 방향 실측 (아주 작게만 움직임, 안전).

현재 위치에서 +40 raw(약 3.5도) 목표를 주고 실제로 어느 쪽으로 갔는지 측정.
-> 목표를 올렸는데 위치가 내려가면 '방향 반대' 확정.

한계는 넉넉히 열어두고, 즉시 토크 OFF. 이동량 40 raw 로 제한.
실행: python sts_g6_dirtest.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
GID=6
ADDR_TORQUE=40; ADDR_ACC=41; ADDR_GOAL=42; ADDR_SPEED=46
ADDR_TQLIMIT=48; ADDR_PRESENT=56; ADDR_MODE=33
ADDR_MIN=9; ADDR_MAX=11; ADDR_LOCK=55
DELTA=40           # 시험 이동량 (약 3.5도)

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(a,sz=2,tries=6):
    for _ in range(tries):
        time.sleep(0.1)
        try:
            v,r,_=(packet.read2ByteTxRx(port,GID,a) if sz==2 else packet.read1ByteTxRx(port,GID,a))
            if r==COMM_SUCCESS: return v
        except Exception: pass
        time.sleep(0.1)
    return None
def wr2(a,v):
    for _ in range(4):
        try:
            if packet.write2ByteTxRx(port,GID,a,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False
def wr1(a,v):
    for _ in range(4):
        try:
            if packet.write1ByteTxRx(port,GID,a,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False
def off(*_):
    wr1(ADDR_TORQUE,0)
    try: port.closePort()
    except Exception: pass
    print("\n토크 OFF"); sys.exit(0)
signal.signal(signal.SIGINT, off)

wr1(ADDR_TORQUE,0); time.sleep(0.2)
p0=rd(ADDR_PRESENT)
lo=rd(ADDR_MIN); hi=rd(ADDR_MAX)
if p0 is None: print("응답 없음"); off()
print(f"현재 위치 {p0},  한계 {lo}~{hi}")

# 시험 목표가 한계 안에 들도록 방향 선택
tgt=p0+DELTA
if hi is not None and tgt>hi-10: tgt=p0-DELTA
if lo is not None and tgt<lo+10:
    print("한계가 좁아 시험 불가. 한계를 임시로 넓힙니다.")
    wr1(ADDR_LOCK,0); time.sleep(0.05)
    wr2(ADDR_MIN,max(0,p0-200)); time.sleep(0.05)
    wr2(ADDR_MAX,min(4095,p0+200)); time.sleep(0.05)
    wr1(ADDR_LOCK,1); time.sleep(0.05)
    tgt=p0+DELTA

print(f"시험: 목표 {tgt} ({tgt-p0:+d} raw) 로 이동합니다.")
print("팔/그리퍼 주변을 비우세요. Ctrl+C = 즉시 정지")
try: input("Enter: ")
except KeyboardInterrupt: off()

wr1(ADDR_MODE,0)
wr2(ADDR_TQLIMIT,200)     # 낮은 토크
wr1(ADDR_ACC,5)
wr2(ADDR_SPEED,80)        # 매우 느리게
wr2(ADDR_GOAL,tgt)
wr1(ADDR_TORQUE,1)

for k in range(12):
    time.sleep(0.25)
    p=rd(ADDR_PRESENT)
    print(f"  t={k*0.25:.1f}s  pos={p}")
    if p is not None and abs(p-tgt)<8: break

p1=rd(ADDR_PRESENT)
wr1(ADDR_TORQUE,0)
moved=p1-p0 if p1 is not None else 0
want=tgt-p0
print(f"\n결과: 목표 {want:+d},  실제 이동 {moved:+d}")
if moved==0:
    print("-> 안 움직임 (한계에 막혔거나 토크 부족)")
elif (moved>0)==(want>0):
    print("-> 방향 정상 (목표와 같은 쪽으로 이동)")
else:
    print("-> ***방향 반대*** (목표와 반대쪽으로 이동)")
    print("   sts_gripper.py 의 CLOSE/OPEN 값을 서로 바꾸면 해결됩니다.")
off()
