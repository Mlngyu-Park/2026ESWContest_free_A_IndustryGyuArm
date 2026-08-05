#!/usr/bin/env python3
"""
STS3215 관절 한계 측정 (이전 measure_limits.py 와 동일한 방식).
- 시작 시 전체 토크 OFF (손으로 돌릴 수 있게)
- 6개 모터를 실시간 추적하며 min/max 갱신 (0/4095 랩 보정 포함)
- 손으로 각 관절을 양방향 끝까지 천천히 왕복
- Ctrl+C 로 종료 -> 측정된 한계 출력 + sts_limits.py 저장

실행: python sts_measure_limits.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_TORQUE=40; ADDR_PRESENT=56
IDS=[1,2,3,4,5,6]
OUT='/home/drboom/2026_project/Industry_gyu_arm/sts_limits.py'

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def read_pos(i):
    p,r,_=packet.read2ByteTxRx(port,i,ADDR_PRESENT)
    return p if r==COMM_SUCCESS else None

# 토크 전부 OFF
for i in IDS: packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
print("토크 OFF. 손으로 각 관절을 양쪽 끝까지 천천히 왕복하세요.")
print("(0/4095 경계를 넘어도 자동 보정합니다)")
print("전부 다 돌렸으면 Ctrl+C -> 한계 출력\n")

# 랩 보정 추적 초기화
last={}; un={}; mn={}; mx={}
for i in IDS:
    p=read_pos(i)
    if p is None: print(f"ID {i} 응답 없음"); raise SystemExit
    last[i]=p; un[i]=p; mn[i]=p; mx[i]=p

def finish(*_):
    print("\n\n================ 측정된 한계 (raw) ================")
    print(f"{'ID':<4}{'min':>7}{'max':>7}{'범위(도)':>10}")
    for i in IDS:
        span=(mx[i]-mn[i])*360.0/4096.0
        print(f"{i:<4}{mn[i]:>7}{mx[i]:>7}{span:>10.1f}")
    lines=["# sts_measure_limits.py 측정 결과 (raw, 랩 보정된 연속값)",
           "LIMIT_RAW={"]
    for i in IDS: lines.append(f"    {i}: ({mn[i]}, {mx[i]}),")
    lines.append("}")
    open(OUT,'w').write("\n".join(lines)+"\n")
    print(f"\n저장됨: {OUT}")
    print("(토크는 꺼진 상태 유지)")
    try: port.closePort()
    except Exception: pass
    raise SystemExit
signal.signal(signal.SIGINT, finish)

t0=0
while True:
    line=[]
    for i in IDS:
        p=read_pos(i)
        if p is None:
            line.append(f"ID{i}:--"); continue
        d=p-last[i]
        if d>2048: d-=4096
        elif d<-2048: d+=4096
        un[i]+=d; last[i]=p
        mn[i]=min(mn[i],un[i]); mx[i]=max(mx[i],un[i])
        line.append(f"ID{i}:{un[i]}({mn[i]}~{mx[i]})")
    now=time.time()
    if now-t0>0.3:
        print("\r"+"  ".join(line)+"    ",end="",flush=True)
        t0=now
    time.sleep(0.05)
