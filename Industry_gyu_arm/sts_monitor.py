#!/usr/bin/env python3
"""
STS3215 실시간 위치 모니터 (읽기 전용, 절대 안 움직임).

표시: 각 모터의 raw 위치, 각도, 영점 대비 차이, 0/4095 경계 근접 경고
종료: Ctrl+C

실행: python sts_monitor.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_PRESENT=56
IDS=[1,2,3,4,5,6]
EDGE=150        # 0 또는 4095 에서 이만큼 이내면 경고

try:
    from sts_zero import ZERO_RAW
except Exception:
    ZERO_RAW=None

port=PortHandler(PORT)
if not port.openPort(): print(f"포트 실패: {PORT}"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(i,addr,tries=3):
    for _ in range(tries):
        try:
            time.sleep(0.01)
            v,r,_=packet.read2ByteTxRx(port,i,addr)
            if r==COMM_SUCCESS: return v
        except Exception: pass
        time.sleep(0.03)
    return None

def bye(*_):
    try: port.closePort()
    except Exception: pass
    print("\n종료 (읽기만 했습니다)")
    sys.exit(0)
signal.signal(signal.SIGINT,bye)

print("실시간 모니터 (읽기 전용). Ctrl+C 종료")
print("raw 0~4095 = 0~360도.  ! 표시 = 0/4095 경계 근접\n")
if ZERO_RAW: print("영점:",ZERO_RAW,"\n")

try:
    while True:
        parts=[]
        for i in IDS:
            p=rd(i,ADDR_PRESENT)
            if p is None:
                parts.append(f"ID{i}: ----"); continue
            deg=p*360.0/4096.0
            warn="!" if (p<EDGE or p>4095-EDGE) else " "
            if ZERO_RAW and i in ZERO_RAW:
                d=p-ZERO_RAW[i]
                parts.append(f"ID{i}:{p:4d}{warn}({deg:5.1f}d {d:+5d})")
            else:
                parts.append(f"ID{i}:{p:4d}{warn}({deg:5.1f}d)")
        print("\r "+" ".join(parts)+"  ",end="",flush=True)
        time.sleep(0.2)
except KeyboardInterrupt:
    bye()
