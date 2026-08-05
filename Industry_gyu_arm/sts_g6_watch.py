#!/usr/bin/env python3
"""
ID6 위치 추적 - 경계 넘어가는지 확인 (읽기 전용, 토크 안 켬).

손으로 그리퍼를 닫힘<->열림 왕복시키면서
  - raw 위치
  - 연속각(랩 보정): 4095->0 을 넘어가면 이어서 계산
  - 최소/최대
를 실시간 표시. Ctrl+C 종료 시 요약.

실행: python sts_g6_watch.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
GID=6
ADDR_TORQUE=40; ADDR_PRESENT=56; ADDR_MIN=9; ADDR_MAX=11

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(addr,tries=4):
    for _ in range(tries):
        try:
            time.sleep(0.04)
            v,r,_=packet.read2ByteTxRx(port,GID,addr)
            if r==COMM_SUCCESS: return v
        except Exception: pass
        time.sleep(0.05)
    return None
def wr1(addr,v):
    for _ in range(3):
        try:
            if packet.write1ByteTxRx(port,GID,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False

wr1(ADDR_TORQUE,0)      # 토크 OFF - 손으로 움직일 수 있게
lo=rd(ADDR_MIN); hi=rd(ADDR_MAX)
p0=rd(ADDR_PRESENT)
if p0 is None: print("ID6 응답 없음"); raise SystemExit

print(f"ID6 한계: {lo} ~ {hi}   시작 위치: {p0}")
print("토크 OFF. 손으로 그리퍼를 닫힘<->열림 왕복시켜 주세요.")
print("경계(0/4095)를 넘으면 'WRAP!' 이 표시됩니다. Ctrl+C 로 종료\n")

last=p0; un=p0; mn=p0; mx=p0; wraps=0
def finish(*_):
    print("\n\n===== 요약 =====")
    print(f"raw 관찰 범위(연속각): {mn} ~ {mx}   폭 {mx-mn} raw ({(mx-mn)*360/4096:.0f}도)")
    print(f"경계 넘김(WRAP) 횟수: {wraps}")
    print(f"모터 한계 설정: {lo} ~ {hi}")
    if wraps>0:
        print("\n-> 그리퍼 가동 범위가 0/4095 경계를 걸치고 있습니다.")
        print("   이 경우 한계 설정만으로는 막을 수 없고, 오프셋으로 좌표를 옮겨야 합니다.")
    else:
        print("\n-> 경계를 넘지 않습니다. 한계 안에서만 움직입니다.")
    try: port.closePort()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT, finish)

t0=0
while True:
    p=rd(ADDR_PRESENT)
    if p is not None:
        d=p-last
        w=""
        if d>2048:
            d-=4096; wraps+=1; w="  <-- WRAP!"
        elif d<-2048:
            d+=4096; wraps+=1; w="  <-- WRAP!"
        un+=d; last=p
        mn=min(mn,un); mx=max(mx,un)
        out=(p<lo or p>hi)
        now=time.time()
        if now-t0>0.15 or w:
            print(f"\r raw={p:4d}{'!한계밖' if out else '      '}  연속={un:6d}  범위=[{mn}~{mx}]  wrap={wraps}{w}   ",
                  end="" if not w else "\n",flush=True)
            t0=now
    time.sleep(0.03)
