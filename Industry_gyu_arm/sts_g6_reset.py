#!/usr/bin/env python3
"""
ID6(그리퍼) 설정만 초기화. 다른 모터는 건드리지 않음.
  - Position Offset(31) = 0
  - Min Angle(9) = 0, Max Angle(11) = 4095  (한계 해제)
  - Torque(40) = 0
토크 안 켬 (모터 스스로 안 움직임).

실행: python sts_g6_reset.py
"""
import sys, time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
GID=6
ADDR_TORQUE=40; ADDR_PRESENT=56; ADDR_GOAL=42
ADDR_MIN=9; ADDR_MAX=11; ADDR_OFS=31; ADDR_LOCK=55

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(a,sz=2,tries=6):
    for _ in range(tries):
        time.sleep(0.1)
        try:
            v,r,_=(packet.read2ByteTxRx(port,GID,a) if sz==2
                   else packet.read1ByteTxRx(port,GID,a))
            if r==COMM_SUCCESS: return v
        except Exception: pass
        time.sleep(0.1)
    return None
def wr2(a,v):
    for _ in range(4):
        try:
            if packet.write2ByteTxRx(port,GID,a,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.04)
    return False
def wr1(a,v):
    for _ in range(4):
        try:
            if packet.write1ByteTxRx(port,GID,a,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.04)
    return False
def dec(v): return -(v&0x7FF) if (v&0x800) else (v&0x7FF)

wr1(ADDR_TORQUE,0); time.sleep(0.2)
print("초기화 전:")
print(f"  offset = {dec(rd(ADDR_OFS) or 0)}")
print(f"  한계   = {rd(ADDR_MIN)} ~ {rd(ADDR_MAX)}")
print(f"  위치   = {rd(ADDR_PRESENT)}")
print(f"  목표   = {rd(ADDR_GOAL)}")

print("\n초기화 중 (offset=0, 한계=0~4095, 토크 OFF)...")
wr1(ADDR_LOCK,0); time.sleep(0.06)
wr2(ADDR_OFS,0);  time.sleep(0.08)
wr2(ADDR_MIN,0);  time.sleep(0.06)
wr2(ADDR_MAX,4095); time.sleep(0.06)
wr1(ADDR_LOCK,1); time.sleep(0.06)
wr1(ADDR_TORQUE,0); time.sleep(0.1)

# 목표를 현재 위치로 맞춰 둠 (다음에 토크 켜도 안 튐)
p=rd(ADDR_PRESENT)
if p is not None: wr2(ADDR_GOAL,p); time.sleep(0.06)

print("\n초기화 후:")
o=rd(ADDR_OFS); mn=rd(ADDR_MIN); mx=rd(ADDR_MAX)
pos=rd(ADDR_PRESENT); goal=rd(ADDR_GOAL); tq=rd(ADDR_TORQUE,1); tmp=rd(63,1)
print(f"  offset = {dec(o) if o is not None else '?'}")
print(f"  한계   = {mn} ~ {mx}")
print(f"  위치   = {pos}")
print(f"  목표   = {goal}")
print(f"  토크   = {tq}")
print(f"  온도   = {tmp} C")
ok = (o==0 and mn==0 and mx==4095 and tq==0)
print("\n"+("초기화 완료 - 이제 손으로 자유롭게 움직입니다."
      if ok else "일부 값이 예상과 다릅니다 - 출력을 알려주세요"))
port.closePort()
