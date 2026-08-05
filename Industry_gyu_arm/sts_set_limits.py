#!/usr/bin/env python3
"""
측정된 한계(sts_limits.py)를 모터 펌웨어의 각도 한계(Min/Max Angle)로 설정.
-> 코드에 버그가 있어도 모터가 이 범위 밖으로 물리적으로 못 나감.

토크 절대 안 켬 (안 움직임). 여유(MARGIN)만큼 안쪽으로 좁혀서 설정.

실행: python sts_set_limits.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
from sts_limits import LIMIT_RAW

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_TORQUE=40; ADDR_PRESENT=56
ADDR_MIN_ANGLE=9; ADDR_MAX_ANGLE=11; ADDR_LOCK=55
IDS=[1,2,3,4,5,6]
MARGIN=20      # 측정 끝단에서 안쪽으로 여유 (raw). 끝단 충돌 방지

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(i,addr,size=2,tries=4):
    for _ in range(tries):
        try:
            time.sleep(0.05)          # 응답 밀림 방지 (연속 읽기 간격 확보)
            v,r,_=(packet.read2ByteTxRx(port,i,addr) if size==2
                   else packet.read1ByteTxRx(port,i,addr))
            if r==COMM_SUCCESS: return v
        except Exception: pass
        time.sleep(0.05)
    return None
def wr2(i,addr,v,tries=4):
    for _ in range(tries):
        try:
            if packet.write2ByteTxRx(port,i,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False
def wr1(i,addr,v,tries=4):
    for _ in range(tries):
        try:
            if packet.write1ByteTxRx(port,i,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False
def bye(*_):
    for i in IDS: wr1(i,ADDR_TORQUE,0)
    try: port.closePort()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT,bye)

for i in IDS: wr1(i,ADDR_TORQUE,0)
time.sleep(0.2)

print("펌웨어 각도 한계 설정 (토크 OFF, 안 움직임)\n")
out=["# sts_set_limits.py 로 모터에 설정한 각도 한계 (raw)","LIMIT_RAW={"]
allok=True
for i in IDS:
    mn,mx=LIMIT_RAW[i]
    lo=max(0,min(4095,mn+MARGIN))
    hi=max(0,min(4095,mx-MARGIN))
    if lo>=hi: print(f"ID{i}: 범위 이상 ({mn}~{mx}) - 건너뜀"); allok=False; continue
    wr1(i,ADDR_LOCK,0); time.sleep(0.03)
    wr2(i,ADDR_MIN_ANGLE,lo); time.sleep(0.03)
    wr2(i,ADDR_MAX_ANGLE,hi); time.sleep(0.03)
    wr1(i,ADDR_LOCK,1); time.sleep(0.03)
    r_lo=rd(i,ADDR_MIN_ANGLE); r_hi=rd(i,ADDR_MAX_ANGLE); p=rd(i,ADDR_PRESENT)
    ok = (r_lo==lo and r_hi==hi)
    inside = p is not None and r_lo<=p<=r_hi
    if not ok: allok=False
    print(f"ID{i}: 설정 {lo}~{hi} / 확인 {r_lo}~{r_hi}  현재 pos={p}  "
          f"{'OK' if ok else '쓰기실패'}{'' if inside else '  [현재 위치가 한계 밖!]'}")
    out.append(f"    {i}: ({lo}, {hi}),")
out.append("}")
open('/home/drboom/2026_project/Industry_gyu_arm/sts_limits_applied.py','w').write("\n".join(out)+"\n")
print("\n저장: sts_limits_applied.py")
print("전체 성공 - 다음: python sts_capture_zero.py (base 자세 잡고 Enter)"
      if allok else "일부 실패 - 출력을 알려주세요")
bye()
