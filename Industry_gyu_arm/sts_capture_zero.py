#!/usr/bin/env python3
"""
STS3215 영점(base 자세) 캡처. 모터를 '전혀 움직이지 않습니다'.

동작:
  1) 6개 모터 토크 OFF  -> 손으로 자유롭게 움직일 수 있음
  2) 현재 위치를 실시간 표시
  3) 로봇을 RViz 의 base 자세(모든 관절 0)와 똑같이 맞춘 뒤
     Enter 를 누르면 그 순간 위치를 ZERO_RAW 로 저장

출력: sts_zero.py  (다른 스크립트가 import 해서 사용)

실행: python sts_capture_zero.py
종료: Ctrl+C (토크는 계속 꺼진 상태)
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_TORQUE=40
ADDR_PRESENT_POSITION=56
IDS=[1,2,3,4,5,6]
OUT='/home/drboom/2026_project/Industry_gyu_arm/sts_zero.py'

port=PortHandler(PORT)
if not port.openPort(): print(f"포트 실패: {PORT}"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def torque_off():
    for i in IDS: packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
def read_pos(i):
    p,r,_=packet.read2ByteTxRx(port,i,ADDR_PRESENT_POSITION)
    return p if r==COMM_SUCCESS else None

def bye(*_):
    print("\n종료 (토크는 꺼진 상태 유지)")
    try: port.closePort()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT, bye)

# 연결 확인
missing=[i for i in IDS if read_pos(i) is None]
if missing:
    print(f"응답 없는 ID: {missing} - 연결 확인 후 재시도"); bye()

torque_off()
print("="*60)
print(" 토크 OFF 완료 - 이제 손으로 로봇을 움직일 수 있습니다.")
print(" RViz 의 base 자세(모든 슬라이더 0)와 똑같이 맞춰주세요.")
print(" 맞추는 동안 현재 위치가 아래에 계속 표시됩니다.")
print("="*60)
print(" 다 맞췄으면 Enter, 취소는 Ctrl+C\n")

try:
    from sts_limits import LIMIT_RAW as _LIM
except Exception:
    _LIM=None
stop_disp=False
import threading
def disp():
    while not stop_disp:
        vals=[]
        for i in IDS:
            p=read_pos(i)
            if p is None: vals.append(f"ID{i}:  --"); continue
            mark=""
            if _LIM and i in _LIM:
                lo,hi=_LIM[i]
                if (p-lo)<115 or (hi-p)<115: mark="!"   # 한계 10도 이내
            vals.append(f"ID{i}:{p:>5}{mark}")
        print("\r  "+"  ".join(vals)+"    (! = 한계 10도 이내)  ",end="",flush=True)
        time.sleep(0.25)
t=threading.Thread(target=disp,daemon=True); t.start()

try:
    input()
except KeyboardInterrupt:
    bye()
stop_disp=True; time.sleep(0.4)

# 캡처 (여러 번 읽어 안정된 값 사용)
print("\n\n캡처 중...")
zero={}
for i in IDS:
    reads=[]
    for _ in range(5):
        p=read_pos(i)
        if p is not None: reads.append(p)
        time.sleep(0.03)
    if not reads:
        print(f"  ID {i} 읽기 실패"); bye()
    zero[i]=sorted(reads)[len(reads)//2]      # 중앙값

print("\n=== 영점 (base 자세) ===")
try:
    from sts_limits import LIMIT_RAW
except Exception:
    LIMIT_RAW=None
warn=[]
for i in IDS:
    if LIMIT_RAW and i in LIMIT_RAW:
        lo,hi=LIMIT_RAW[i]
        a=(zero[i]-lo)*180/2048; b=(hi-zero[i])*180/2048
        flag=""
        if a<10 or b<10:
            flag="   <-- 한계에 너무 가까움!"; warn.append(i)
        print(f"  ID {i}: {zero[i]}   여유 아래 {a:.0f}도 / 위 {b:.0f}도{flag}")
    else:
        print(f"  ID {i}: {zero[i]}")
if warn:
    print(f"\n주의: ID {warn} 는 한계에 붙어 있어 그 방향으로 못 움직입니다.")
    print("      그 관절을 가동 범위 중간쯤으로 옮기고 다시 캡처하는 게 좋습니다.")

lines=["# sts_capture_zero.py 로 저장한 영점 (base 자세)",
       "# 로봇을 RViz base 자세로 맞춘 뒤 캡처한 raw 값",
       "ZERO_RAW={"]
for i in IDS: lines.append(f"    {i}: {zero[i]},")
lines.append("}")
open(OUT,'w').write("\n".join(lines)+"\n")
print(f"\n저장됨: {OUT}")
print("이제 방향 확인 단계로 넘어갈 수 있습니다. (토크는 꺼진 상태)")
port.closePort()
