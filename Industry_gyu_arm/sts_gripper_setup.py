#!/usr/bin/env python3
"""
그리퍼(ID6) 열림/닫힘 위치 설정. 각도 개념 없이 두 위치만 기록.

1) ID6 각도 한계를 임시 해제 (0~4095) -> 손으로 자유롭게
2) 완전히 닫은 상태에서 Enter -> CLOSE 기록
3) 완전히 연 상태에서 Enter   -> OPEN 기록
4) 두 위치 안쪽으로 한계 재설정 + sts_gripper.py 저장

토크는 켜지 않음 (안 움직임).
실행: python sts_gripper_setup.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
GID=5      # 그리퍼 (10-29 번호 6 -> 5)
ADDR_TORQUE=40; ADDR_PRESENT=56
ADDR_MIN_ANGLE=9; ADDR_MAX_ANGLE=11; ADDR_LOCK=55
MARGIN=15
OUT='/home/drboom/2026_project/Industry_gyu_arm/sts_gripper.py'

port=PortHandler(PORT)
if not port.openPort(): print(f"포트 실패: {PORT}"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(addr,tries=4):
    for _ in range(tries):
        try:
            time.sleep(0.05)
            v,r,_=packet.read2ByteTxRx(port,GID,addr)
            if r==COMM_SUCCESS: return v
        except Exception: pass
        time.sleep(0.05)
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
def bye(*_):
    wr1(ADDR_TORQUE,0)
    try: port.closePort()
    except Exception: pass
    print("\n종료 (토크 꺼짐)"); sys.exit(0)
signal.signal(signal.SIGINT,bye)

wr1(ADDR_TORQUE,0); time.sleep(0.2)
print(f"ID6 현재: pos={rd(ADDR_PRESENT)}  limit={rd(ADDR_MIN_ANGLE)}~{rd(ADDR_MAX_ANGLE)}")

# 한계 임시 해제
wr1(ADDR_LOCK,0); time.sleep(0.04)
wr2(ADDR_MIN_ANGLE,0); time.sleep(0.04)
wr2(ADDR_MAX_ANGLE,4095); time.sleep(0.04)
wr1(ADDR_LOCK,1); time.sleep(0.04)
print("한계 임시 해제 (0~4095). 이제 손으로 자유롭게 움직입니다.\n")

import threading
stop=False; cur=[rd(ADDR_PRESENT)]
def disp():
    while not stop:
        p=rd(ADDR_PRESENT)
        if p is not None:
            cur[0]=p
            print(f"\r  현재 pos={p:4d}   ",end="",flush=True)
        time.sleep(0.15)
t=threading.Thread(target=disp,daemon=True); t.start()

print("[1/2] 그리퍼를 '완전히 닫은' 상태로 두고 Enter")
try: input()
except KeyboardInterrupt: bye()
close_pos=cur[0]
print(f"\n  CLOSE = {close_pos}\n")

print("[2/2] 그리퍼를 '완전히 연' 상태로 두고 Enter")
try: input()
except KeyboardInterrupt: bye()
open_pos=cur[0]
stop=True; time.sleep(0.3)
print(f"\n  OPEN  = {open_pos}\n")

if abs(open_pos-close_pos)<50:
    print("두 위치가 너무 가깝습니다. 다시 실행하세요."); bye()

lo=min(close_pos,open_pos); hi=max(close_pos,open_pos)
lim_lo=max(0,lo-MARGIN); lim_hi=min(4095,hi+MARGIN)
wr1(ADDR_LOCK,0); time.sleep(0.04)
wr2(ADDR_MIN_ANGLE,lim_lo); time.sleep(0.04)
wr2(ADDR_MAX_ANGLE,lim_hi); time.sleep(0.04)
wr1(ADDR_LOCK,1); time.sleep(0.04)

open(OUT,'w').write(
 "# sts_gripper_setup.py 로 저장한 그리퍼 위치 (ID6)\n"
 f"GRIPPER_CLOSE={close_pos}\n"
 f"GRIPPER_OPEN={open_pos}\n"
 f"GRIPPER_LIMIT=({lim_lo}, {lim_hi})\n")

print(f"저장: {OUT}")
print(f"  CLOSE={close_pos}  OPEN={open_pos}  한계={lim_lo}~{lim_hi}")
print(f"  확인: limit={rd(ADDR_MIN_ANGLE)}~{rd(ADDR_MAX_ANGLE)}")
print("\n이제 그리퍼는 각도 대신 열림/닫힘 두 위치로 제어합니다.")
bye()
