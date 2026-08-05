#!/usr/bin/env python3
"""
STS3215 오프셋/각도한계 초기화 (공장 기본 상태로).
  - Position Offset(31) = 0
  - Min Angle(9) = 0, Max Angle(11) = 4095  (제한 해제)
  - 토크는 켜지 않음 (모터 안 움직임)

이후 손으로 각 관절을 원하는 범위 안으로 직접 돌려놓으면 됩니다.

실행: python sts_reset.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_TORQUE=40; ADDR_PRESENT=56
ADDR_MIN_ANGLE=9; ADDR_MAX_ANGLE=11; ADDR_OFS=31; ADDR_LOCK=55
IDS=[1,2,3,4,5,6]

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
    for _ in range(4):
        try:
            if packet.write2ByteTxRx(port,i,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False
def wr1(i,addr,v):
    for _ in range(4):
        try:
            if packet.write1ByteTxRx(port,i,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False
def dec(v): return -(v&0x7FF) if (v&0x800) else (v&0x7FF)
def bye(*_):
    for i in IDS: wr1(i,ADDR_TORQUE,0)
    try: port.closePort()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT,bye)

# 토크 확실히 OFF
for i in IDS: wr1(i,ADDR_TORQUE,0)
time.sleep(0.2)

print("초기화 전 상태:")
before={}
for i in IDS:
    o=rd(i,ADDR_OFS); lo=rd(i,ADDR_MIN_ANGLE); hi=rd(i,ADDR_MAX_ANGLE); p=rd(i,ADDR_PRESENT)
    before[i]=(dec(o) if o is not None else None,lo,hi,p)
    print(f"  ID{i}: offset={dec(o) if o is not None else '?':+5}  limit={lo}~{hi}  pos={p}")

print("\n오프셋 0, 한계 0~4095 로 초기화합니다.")
print("(위치 표시값이 오프셋만큼 바뀝니다. 물리적으로는 안 움직입니다)")
try:
    input("Enter 로 진행 (취소는 Ctrl+C): ")
except KeyboardInterrupt:
    bye()

print()
allok=True
for i in IDS:
    wr1(i,ADDR_LOCK,0); time.sleep(0.04)
    ok1=wr2(i,ADDR_OFS,0); time.sleep(0.06)
    ok2=wr2(i,ADDR_MIN_ANGLE,0); time.sleep(0.04)
    ok3=wr2(i,ADDR_MAX_ANGLE,4095); time.sleep(0.04)
    wr1(i,ADDR_LOCK,1); time.sleep(0.04)
    o=rd(i,ADDR_OFS); lo=rd(i,ADDR_MIN_ANGLE); hi=rd(i,ADDR_MAX_ANGLE); p=rd(i,ADDR_PRESENT)
    good = (o==0 and lo==0 and hi==4095)
    if not good: allok=False
    print(f"  ID{i}: offset={dec(o) if o is not None else '?'}  limit={lo}~{hi}  pos={p}   {'OK' if good else '확인필요'}")

print("\n"+("전체 초기화 완료." if allok else "일부 실패 - 출력을 알려주세요"))
print("이제 손으로 각 관절을 원하는 자세로 돌려놓으세요. (토크 꺼짐)")
print("현재 위치 확인: python sts_scan.py")
bye()
