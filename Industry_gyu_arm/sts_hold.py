#!/usr/bin/env python3
"""
STS3215 안전 홀드 - 현재 자세를 그대로 유지 (움직임 없음).
방향 확인 전에 '토크를 켜도 안 움직이는지' 검증하는 용도.

안전 설계:
  - 관절 하나씩 처리: 위치 읽기 -> 즉시 목표로 쓰기 -> 오차 확인 -> 토크 ON
  - 오차가 크면 그 관절은 건너뜀 (움직일 위험이 있으므로)
  - 토크 제한 낮게, 속도/가속 최저
  - Ctrl+C = 즉시 토크 OFF

실행: python sts_hold.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_TORQUE=40; ADDR_ACC=41; ADDR_GOAL=42; ADDR_SPEED=46
ADDR_TORQUE_LIMIT=48; ADDR_PRESENT=56
IDS=[1,2,3,4,5]        # j6(그리퍼)는 원인 파악 전까지 제외 - 절대 건드리지 않음
WATCH_ONLY=[6]         # 읽기만 (쓰기 없음)

SAFE_SPEED=100      # 매우 느리게
SAFE_ACC=5          # 부드럽게
TORQUE_LIMIT=250    # 25% 제한
MAX_ERR=8           # 목표-실제 오차가 이보다 크면 토크 안 켬 (raw)

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def read_pos(i):
    p,r,_=packet.read2ByteTxRx(port,i,ADDR_PRESENT)
    return p if r==COMM_SUCCESS else None

def torque_off_all():
    for i in IDS+WATCH_ONLY:      # 6번도 '끄기'만은 함 (안전)
        try: packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
        except Exception: pass

def bye(*_):
    print("\n[정지] 토크 OFF")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    torque_off_all()
    try: port.closePort()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT, bye)

print("전체 토크 OFF 후 시작합니다...")
torque_off_all()
time.sleep(0.3)

print("\n관절별로 '현재 위치 = 목표'로 맞춘 뒤 토크를 켭니다.")
print("팔을 손으로 받치고 계세요. Ctrl+C = 즉시 정지\n")

held=[]
for i in IDS:
    # 안전 파라미터 먼저 (토크 꺼진 상태에서)
    packet.write2ByteTxRx(port,i,ADDR_TORQUE_LIMIT,TORQUE_LIMIT)
    packet.write1ByteTxRx(port,i,ADDR_ACC,SAFE_ACC)
    packet.write2ByteTxRx(port,i,ADDR_SPEED,SAFE_SPEED)
    # 읽자마자 즉시 목표로 쓰기 (사이에 시간 안 둠)
    p=read_pos(i)
    if p is None:
        print(f"  ID {i}: 읽기 실패 - 건너뜀"); continue
    packet.write2ByteTxRx(port,i,ADDR_GOAL,p)
    # 검증: 다시 읽어 오차 확인
    time.sleep(0.05)
    p2=read_pos(i)
    err=abs(p2-p) if p2 is not None else 999
    if err>MAX_ERR:
        print(f"  ID {i}: 위치 흔들림 {err} raw - 토크 안 켬 (팔을 고정하고 재시도)")
        continue
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)
    held.append(i)
    print(f"  ID {i}: 유지 시작 (pos {p}, 오차 {err})")

if not held:
    print("\n토크를 켠 관절이 없습니다. 팔을 고정하고 다시 실행하세요.")
    bye()

print(f"\n토크 ON: {held}")
print("이 상태에서 로봇이 '가만히 있으면' 정상입니다.")
print("움직이면 즉시 Ctrl+C 하세요.\n")

try:
    while True:
        vals=[]
        for i in IDS:
            p=read_pos(i)
            vals.append(f"ID{i}:{p if p is not None else '--':>5}")
        for i in WATCH_ONLY:
            p=read_pos(i)
            vals.append(f"ID{i}(감시):{p if p is not None else '--':>5}")
        print("\r  "+"  ".join(vals)+"   ",end="",flush=True)
        time.sleep(0.4)
except KeyboardInterrupt:
    bye()
