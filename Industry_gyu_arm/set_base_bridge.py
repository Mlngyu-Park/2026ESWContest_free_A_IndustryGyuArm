#!/usr/bin/env python3
"""
RViz 슬라이더로 실물을 조금씩 움직이다가, 터미널에 's' + Enter 를 치면
그 순간의 실제 모터 자세를 '기본자세(base)' = 영점(ZERO_RAW)으로 저장.

- /joint_states 구독 -> 실물 모터로 전송 (RViz와 실물 같이 움직임)
- 's' Enter : 현재 실제 모터 raw 를 읽어 base로 저장 (화면 출력 + 파일 저장)
- 'q' Enter 또는 Ctrl+C : 토크 OFF 종료
- j4<->j5 실제 배선 교차 매핑 반영

안전: 안전속도 + 가감속, 측정 한계 클램프, Ctrl+C = 즉시 토크 OFF.
"""
import sys, math, signal, threading
from dynamixel_sdk import PortHandler, PacketHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112; ADDR_GOAL=116; ADDR_PRESENT=132
SAFE_VEL=25; SAFE_ACCEL=8
SAVE_PATH='/home/drboom/2026_project/Industry_gyu_arm/base_captured.py'

MOTORS={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5',5:'j6'}
# j4<->j5 실제 배선 교차: RViz j4 -> 실물 ID4, RViz j5 -> 실물 ID3
NAME2ID={'j1':0,'j2':1,'j3':2,'j4':4,'j5':3,'j6':5}
ZERO_RAW={0:1517,1:2483,2:1562,3:2244,4:2545,5:2018}
DIR={0:-1,1:1,2:1,3:1,4:1,5:1}   # j1 반전 (브리지 확인)
LIMIT_RAW={0:(380,2571),1:(1479,3555),2:(1334,3382),3:(1271,3153),4:(1599,3524),5:(1970,3525)}

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def rad_to_raw(i,rad):
    raw=int(ZERO_RAW[i]+DIR[i]*rad*2048.0/math.pi)
    lo,hi=LIMIT_RAW[i]
    return max(lo,min(hi,raw))
def get(i):
    p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT); return s32(p) if r==0 else None

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)

_stop=False
def emergency_off(*_):
    global _stop
    _stop=True
    print("\n[긴급정지] 토크 OFF")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for i in MOTORS: packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
    port.closePort()
    try: rclpy.shutdown()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT, emergency_off)

# 속도/가감속/토크
for i in MOTORS:
    packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,SAFE_ACCEL)
    packet.write4ByteTxRx(port,i,ADDR_P_VEL,SAFE_VEL)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)

print("="*50)
print("RViz 슬라이더로 실물을 조금씩 움직이세요.")
print("  s + Enter : 현재 자세를 base(영점)로 저장")
print("  q + Enter : 종료 (토크 OFF)")
print("  Ctrl+C    : 즉시 정지 (토크 OFF)")
print("="*50)

def save_base():
    cur={}
    for i in MOTORS:
        p=get(i)
        cur[i]= p if p is not None else ZERO_RAW[i]
    lines=["# set_base_bridge.py 로 저장한 base 자세 (ZERO_RAW)","ZERO_RAW={"]
    for i in sorted(cur): lines.append(f"    {i}: {cur[i]},")
    lines.append("}")
    text="\n".join(lines)
    print("\n"+"-"*50)
    print("[저장됨] 현재 자세 -> base (영점):")
    print(text)
    print("-"*50)
    try:
        with open(SAVE_PATH,'w') as f: f.write(text+"\n")
        print(f"파일 저장: {SAVE_PATH}")
    except Exception as e:
        print(f"파일 저장 실패: {e}")
    print("계속 조정하거나 q로 종료하세요.\n")

def key_loop():
    global _stop
    for line in sys.stdin:
        c=line.strip().lower()
        if c=='s': save_base()
        elif c=='q':
            emergency_off()
        elif c:
            print("  (s=저장, q=종료)")
        if _stop: break

rclpy.init(); node=Node("set_base_bridge")

def cb(msg):
    for name, pos in zip(msg.name, msg.position):
        if name in NAME2ID:
            i=NAME2ID[name]
            packet.write4ByteTxRx(port,i,ADDR_GOAL, rad_to_raw(i,pos) & 0xFFFFFFFF)

node.create_subscription(JointState,"joint_states",cb,10)

t=threading.Thread(target=key_loop, daemon=True)
t.start()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    emergency_off()
