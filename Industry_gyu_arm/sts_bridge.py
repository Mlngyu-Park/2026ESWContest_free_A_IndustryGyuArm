#!/usr/bin/env python3
"""
RViz 슬라이더 -> 실물(STS3215) 브리지. 방향(DIR) 확인용.

전제:
  - sts_zero.py    : base 영점 (캡처 완료)
  - sts_limits.py  : 관절 한계 (측정 완료, 모터 펌웨어에도 설정됨)

안전 설계:
  - 시작 시 '현재 위치'를 목표로 즉시 고정 -> 안 튐 (관절별 오차 검증)
  - 슬라이더는 '시작 시점 대비 변화량'만 반영
  - 한 사이클 최대 이동량 제한 + 측정 한계 클램프
  - 토크 제한 낮음, 속도/가속 최저
  - Ctrl+C = 즉시 토크 OFF

사용:
  터미널1: ros2 launch Industry_gyu_arm_20260727_base view.launch.py
  터미널2: python sts_bridge.py

확인법: 슬라이더를 조금씩 올렸을 때 실물이 RViz와 같은 방향이면 OK,
        반대면 그 관절 번호를 기록 -> DIR 을 -1 로 바꾸면 됩니다.
"""
import sys, math, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from sts_zero import ZERO_RAW
from sts_limits import LIMIT_RAW
from sts_gripper import GRIPPER_CLOSE, GRIPPER_OPEN

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_TORQUE=40; ADDR_ACC=41; ADDR_GOAL=42; ADDR_SPEED=46
ADDR_TORQUE_LIMIT=48; ADDR_PRESENT=56; ADDR_MODE=33

JOINTS=['j1','j2','j3','j4','j5']        # 슬라이더 연동은 j1~j5
NAME2ID={f'j{k}':k for k in range(1,6)}  # j6 제외 (열기/닫기 버튼으로 제어)
IDS=[1,2,3,4,5,6]                        # 토크/고정은 6개 전부
GRIP_ID=6

# 방향부호: 실물이 RViz와 반대면 -1
# 2026-07-27 확인: j1~j5 전부 반대로 확인됨 -> 전부 -1 (j6도 동일 규약으로 -1)
DIR={1:-1, 2:-1, 3:-1, 4:-1, 5:-1, 6:-1}
RAW_PER_RAD=4096.0/(2*math.pi)      # STS3215: 4096 = 360도

# 안전 파라미터
SAFE_SPEED=150      # 1.5배속
SAFE_ACC=8
TORQUE_LIMIT=250        # 25%
MAX_STEP_RAW=60         # 한 사이클 최대 이동 (약 5도)
MAX_HOLD_ERR=10         # 시작 시 위치 흔들림 허용치
LIMIT_MARGIN=25         # 측정 한계에서 안쪽 여유

# 관절별 예외: j6(그리퍼)는 기어비/부하 때문에 낮은 속도로는 안 움직임
PER_ID_SPEED={6:300}    # 그리퍼는 낮은 속도로는 정지마찰을 못 이김
PER_ID_ACC={6:10}
PER_ID_TQ={6:250}       # 발열 억제 (350 -> 250)
NO_LIMIT_IDS={6}        # j6 는 모터 한계를 해제해 뒀으므로 소프트 클램프도 완화

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
    for _ in range(3):
        try:
            if packet.write2ByteTxRx(port,i,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.05)
    return False
def wr1(i,addr,v):
    for _ in range(3):
        try:
            if packet.write1ByteTxRx(port,i,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.05)
    return False
def read_pos(i): return rd(i,ADDR_PRESENT)

_stop=False
def emergency_off(*_):
    global _stop
    _stop=True
    print("\n[긴급정지] 토크 OFF")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for i in IDS: wr1(i,ADDR_TORQUE,0)
    try: port.closePort()
    except Exception: pass
    try: rclpy.shutdown()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT, emergency_off)

print("="*58)
print(" RViz 슬라이더 -> 실물 브리지 (방향 확인)")
print(" 영점:", ZERO_RAW)
print(" 팔을 손으로 받치고, 손을 Ctrl+C 위에 두세요.")
print("="*58)

for i in IDS: wr1(i,ADDR_TORQUE,0)
time.sleep(0.3)

# 관절별: 안전설정 -> 현재위치 즉시 목표 -> 오차검증 -> 토크 ON
print("\n현재 자세로 고정 중...")
START_RAW={}; held=[]
for i in IDS:
    wr1(i,ADDR_MODE,0)
    wr2(i,ADDR_TORQUE_LIMIT,PER_ID_TQ.get(i,TORQUE_LIMIT))
    wr1(i,ADDR_ACC,PER_ID_ACC.get(i,SAFE_ACC))
    wr2(i,ADDR_SPEED,PER_ID_SPEED.get(i,SAFE_SPEED))
    p=read_pos(i)
    if p is None: print(f"  ID{i}: 읽기 실패 - 건너뜀"); continue
    wr2(i,ADDR_GOAL,p)
    p2=read_pos(i)
    err=abs(p2-p) if p2 is not None else 999
    if err>MAX_HOLD_ERR:
        print(f"  ID{i}: 흔들림 {err} raw - 토크 안 켬"); continue
    wr1(i,ADDR_TORQUE,1)
    START_RAW[i]=p; held.append(i)
    d=p-ZERO_RAW[i]
    print(f"  ID{i}: pos {p} (영점 대비 {d:+d} raw = {d*180/2048:+.1f}도)")

if not held:
    print("\n고정된 관절이 없습니다. 팔을 고정하고 재실행하세요."); emergency_off()
print(f"\n토크 ON: {held}")

# --- 이동 계획 미리 보고 확인 ---
print("\nbase 이동 계획:")
big=[]
for i in held:
    if i==GRIP_ID:
        tgt=GRIPPER_CLOSE                     # 그리퍼는 '닫힘' 이 base
    elif i in NO_LIMIT_IDS:
        tgt=max(30,min(4065,ZERO_RAW[i]))
    else:
        lo,hi=LIMIT_RAW[i]
        tgt=max(lo+LIMIT_MARGIN,min(hi-LIMIT_MARGIN,ZERO_RAW[i]))
    d=tgt-START_RAW[i]
    if abs(d)>350: big.append(i)
    print(f"  ID{i}: {START_RAW[i]} -> {tgt}  이동 {d:+5d} raw ({d*180/2048:+6.1f}도)"
          + ("   <-- 큰 이동" if abs(d)>350 else ""))
if big: print(f"\n주의: ID {big} 이동량이 30도를 넘습니다. 주변을 확인하세요.")
try:
    input("\nEnter 로 base 이동 시작 (취소는 Ctrl+C): ")
except KeyboardInterrupt:
    emergency_off()

# --- base 자세로 천천히 이동 (관절 순서대로, 소량씩) ---
print("\nbase 자세로 이동합니다 (Ctrl+C = 즉시 정지)")
STEP=15          # 사이클당 raw (약 1.3도) - 아주 천천히
for i in held:
    p0=START_RAW[i]; tgt=ZERO_RAW[i]
    if i==GRIP_ID:
        tgt=GRIPPER_CLOSE
    elif i in NO_LIMIT_IDS:
        tgt=max(30,min(4065,tgt))
    else:
        lo,hi=LIMIT_RAW[i]
        tgt=max(lo+LIMIT_MARGIN,min(hi-LIMIT_MARGIN,tgt))
    diff=tgt-p0
    if abs(diff)<5:
        print(f"  ID{i}: 이미 base"); continue
    n=max(1,int(abs(diff)/STEP))
    print(f"  ID{i}: {p0} -> {tgt} ({diff*180/2048:+.1f}도)")
    for k in range(1,n+1):
        if _stop: emergency_off()
        wr2(i,ADDR_GOAL,int(p0+diff*k/n))
        time.sleep(0.06)
    wr2(i,ADDR_GOAL,tgt)
    time.sleep(0.3)
print("base 도착.\n")
print("슬라이더 0 = 이 자세(손으로 캡처한 base) 입니다.")
print("j1~j5: RViz 슬라이더로 제어   |   j6: 별도 창의 [열기]/[닫기] 버튼")
print("Ctrl+C = 전체 토크 OFF\n")

goal_raw={i:ZERO_RAW[i] for i in held}   # base 도착 상태에서 시작
last_print=0.0
_temp_idx=0; _last_temp=0.0              # 과열 감시용

def cb(msg):
    """이전(0724) 방식: 슬라이더 절대각 -> raw = ZERO_RAW + DIR*rad*2048/pi
       슬라이더 0 = 손으로 캡처한 base 자세."""
    global last_print
    if _stop: return
    for name,pos in zip(msg.name,msg.position):
        if name not in NAME2ID: continue
        i=NAME2ID[name]
        if i not in START_RAW: continue
        target=int(ZERO_RAW[i]+DIR[i]*pos*RAW_PER_RAD)
        if i in NO_LIMIT_IDS:
            target=max(30,min(4065,target))          # 물리 범위만 (모터 한계 해제됨)
        else:
            lo,hi=LIMIT_RAW[i]
            target=max(lo+LIMIT_MARGIN,min(hi-LIMIT_MARGIN,target))
        cur=goal_raw[i]
        if target-cur>MAX_STEP_RAW: target=cur+MAX_STEP_RAW
        elif cur-target>MAX_STEP_RAW: target=cur-MAX_STEP_RAW
        if target!=cur:
            wr2(i,ADDR_GOAL,target); goal_raw[i]=target
    now=time.time()
    if now-last_print>1.0:
        last_print=now
        s=[f"j{k}:{goal_raw[k]-ZERO_RAW[k]:+5d}" for k in sorted(goal_raw)]
        # 과열 감시 (5초마다 한 관절씩 순회 검사)
        global _temp_idx,_last_temp
        if now-_last_temp>5.0 and held:
            _last_temp=now
            i=held[_temp_idx % len(held)]; _temp_idx+=1
            t=rd(i,63,1)
            if t is not None:
                if t>=65:
                    print(f"\n[과열 {t}C] ID{i} - 토크 OFF")
                    emergency_off()
                elif t>=55:
                    s.append(f"  [경고 ID{i} {t}C]")
        print("영점대비  "+"  ".join(s))

rclpy.init(); node=Node("sts_bridge")

# 슬라이더가 0 근처인지 먼저 확인 (0이 아니면 base 에서 그만큼 튐)
_first={}
def _peek(msg):
    for n,p in zip(msg.name,msg.position):
        if n in NAME2ID: _first[n]=p
_sub0=node.create_subscription(JointState,"joint_states",_peek,10)
print("RViz 슬라이더 확인 중...")
t0=time.time()
while len(_first)<len(JOINTS) and time.time()-t0<6:
    rclpy.spin_once(node,timeout_sec=0.1)
node.destroy_subscription(_sub0)
if _first:
    off=[f"{n}={v*180/3.14159:+.0f}도" for n,v in _first.items() if abs(v)>0.02]
    if off:
        print("주의: 슬라이더가 0이 아닙니다 ->",", ".join(off))
        print("      연동을 켜면 실물이 그 각도로 움직입니다.")
        print("      RViz 슬라이더를 전부 0으로 내린 뒤 Enter 를 누르세요.")
        try: input("Enter: ")
        except KeyboardInterrupt: emergency_off()
    else:
        print("슬라이더 전부 0 - 정상")
else:
    print("주의: /joint_states 미수신 (RViz 슬라이더 GUI 확인)")

def grip_move(target,label):
    """그리퍼를 target 으로. 토크/설정을 확실히 보장한 뒤 이동."""
    wr1(GRIP_ID,ADDR_MODE,0)
    wr2(GRIP_ID,ADDR_TORQUE_LIMIT,PER_ID_TQ.get(GRIP_ID,TORQUE_LIMIT))
    wr1(GRIP_ID,ADDR_ACC,PER_ID_ACC.get(GRIP_ID,SAFE_ACC))
    wr2(GRIP_ID,ADDR_SPEED,PER_ID_SPEED.get(GRIP_ID,SAFE_SPEED))
    wr2(GRIP_ID,ADDR_GOAL,target)
    wr1(GRIP_ID,ADDR_TORQUE,1)          # 토크가 꺼져 있었어도 확실히 켬
    p=rd(GRIP_ID,ADDR_PRESENT)
    print(f"[그리퍼] {label} -> 목표 {target} (현재 {p})")

def grip_cb(msg):
    """패널의 그리퍼 명령 처리 (/gripper_cmd)."""
    if _stop: return
    c=msg.data.strip().lower()
    if c=='open':
        grip_move(GRIPPER_OPEN,"열기")
    elif c=='close':
        grip_move(GRIPPER_CLOSE,"닫기")
    elif c=='torque_off':
        wr1(GRIP_ID,ADDR_TORQUE,0); print("[그리퍼] 토크 OFF")
    elif c=='torque_on':
        p=rd(GRIP_ID,ADDR_PRESENT)
        if p is not None: wr2(GRIP_ID,ADDR_GOAL,p)
        wr1(GRIP_ID,ADDR_TORQUE,1); print("[그리퍼] 토크 ON")
    else:
        print(f"[그리퍼] 알 수 없는 명령: {c}")

node.create_subscription(JointState,"joint_states",cb,10)
node.create_subscription(String,"gripper_cmd",grip_cb,10)
print("그리퍼 명령 대기 중 (/gripper_cmd) - RViz 옆 패널의 버튼 사용\n")
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    emergency_off()
