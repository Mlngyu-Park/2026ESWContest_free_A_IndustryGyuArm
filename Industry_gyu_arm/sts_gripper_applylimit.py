#!/usr/bin/env python3
"""
sts_gripper.py 의 GRIPPER_LIMIT 을 ID6 모터 펌웨어 한계로 적용.
토크 안 켬 (모터 안 움직임).

실행: python sts_gripper_applylimit.py
"""
import sys, time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
from sts_gripper import GRIPPER_CLOSE, GRIPPER_OPEN, GRIPPER_LIMIT

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
GID=6
ADDR_TORQUE=40; ADDR_PRESENT=56
ADDR_MIN_ANGLE=9; ADDR_MAX_ANGLE=11; ADDR_LOCK=55

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(addr,tries=5):
    for _ in range(tries):
        try:
            time.sleep(0.06)          # 응답 밀림 방지 (간격 충분히)
            v,r,_=packet.read2ByteTxRx(port,GID,addr)
            if r==COMM_SUCCESS: return v
        except Exception: pass
        time.sleep(0.08)
    return None
def wr2(addr,v):
    for _ in range(4):
        try:
            if packet.write2ByteTxRx(port,GID,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False
def wr1(addr,v):
    for _ in range(4):
        try:
            if packet.write1ByteTxRx(port,GID,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False

wr1(ADDR_TORQUE,0); time.sleep(0.2)
lo,hi=GRIPPER_LIMIT
print(f"현재: pos={rd(ADDR_PRESENT)}  limit={rd(ADDR_MIN_ANGLE)}~{rd(ADDR_MAX_ANGLE)}")
print(f"적용: CLOSE={GRIPPER_CLOSE}  OPEN={GRIPPER_OPEN}  limit={lo}~{hi}")

wr1(ADDR_LOCK,0); time.sleep(0.04)
wr2(ADDR_MIN_ANGLE,lo); time.sleep(0.04)
wr2(ADDR_MAX_ANGLE,hi); time.sleep(0.04)
wr1(ADDR_LOCK,1); time.sleep(0.04)

r_lo=rd(ADDR_MIN_ANGLE); r_hi=rd(ADDR_MAX_ANGLE); p=rd(ADDR_PRESENT)
ok=(r_lo==lo and r_hi==hi)
print(f"확인: limit={r_lo}~{r_hi}  pos={p}   {'OK' if ok else '쓰기 실패'}")
if p is not None and not (r_lo<=p<=r_hi):
    print("주의: 현재 위치가 한계 밖입니다. 토크를 켜면 한계 안으로 이동합니다.")
wr1(ADDR_TORQUE,0)
port.closePort()
