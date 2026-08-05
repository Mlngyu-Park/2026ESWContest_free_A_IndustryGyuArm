#!/usr/bin/env python3
"""
엔드 이펙터 x/y/z 직선 이동 (IK). 실물 + RViz 동시.
- 시작 시 실물 현재 자세를 읽어 그 위치부터 시작 (튀지 않음)
- 이동 중 엔드 이펙터 방향(자세)은 고정 -> 평행이동 (그리퍼 안 비틀림)
- IK 해가 나쁘면(오차 큼) 그 스텝 거부 -> 작업범위 밖으로 안 나감

조작:
  a / d : x- / x+
  w / s : y+ / y-
  9 / 0 : z+ / z-
  - / = : 스텝 크기 (2/5/10/20mm)
  f     : 좌표계 전환 (base축 <-> 엔드이펙터축)
  b     : base 자세로 복귀
  p     : 현재 EE 위치 출력
  q / Ctrl+C : 종료 (토크 OFF)

안전: 안전속도+가감속, 관절한계 클램프, IK 오차 거부, Ctrl+C = 즉시 토크 OFF.
RViz: ros2 launch Industry_gyu_arm_20260724_base_position jog.launch.py
"""
import sys, math, signal, termios, tty, select, time
import numpy as np
import xml.etree.ElementTree as ET
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from dynamixel_sdk import PortHandler, PacketHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112; ADDR_GOAL=116; ADDR_PRESENT=132
SAFE_VEL=25; SAFE_ACCEL=8
URDF='/home/drboom/2026_project/urdf/Industry_gyu_arm_20260724_base_position/urdf/Industry_gyu_arm_20260724_base_position.urdf'

JOINTS=['j1','j2','j3','j4','j5','j6']
NAME2ID={'j1':0,'j2':1,'j3':2,'j4':4,'j5':3,'j6':5}   # j4<->j5 배선 교차
J2ID=[0,1,2,4,3,5]
ZERO_RAW={0:1517,1:2483,2:1562,3:2244,4:2545,5:2018}
DIR={0:-1,1:1,2:1,3:1,4:1,5:1}                          # j1 반전
LIMIT_RAW={0:(380,2571),1:(1479,3555),2:(1334,3382),3:(1271,3153),4:(1599,3524),5:(1970,3525)}

STEPS_MM=[2,5,10,20]; step_i=1     # 기본 5mm
IK_TOL=0.004                        # IK 위치오차 허용 4mm
MOVE_T=0.25                         # 스텝당 이동시간(s)

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def jr_limits(jidx):
    i=J2ID[jidx]; lo,hi=LIMIT_RAW[i]
    a=DIR[i]*(lo-ZERO_RAW[i])*math.pi/2048.0
    b=DIR[i]*(hi-ZERO_RAW[i])*math.pi/2048.0
    return (min(a,b),max(a,b))
def rad_to_raw(jidx,rad):
    i=J2ID[jidx]
    raw=int(ZERO_RAW[i]+DIR[i]*rad*2048.0/math.pi)
    lo,hi=LIMIT_RAW[i]; return max(lo,min(hi,raw)), i

# --- ikpy 체인 (URDF j1~j6 수동 구성) ---
def build_chain():
    root=ET.parse(URDF).getroot()
    jm={j.get('name'):j for j in root.findall('joint')}
    links=[OriginLink()]; active=[False]
    for k,jn in enumerate(JOINTS):
        j=jm[jn]; o=j.find('origin')
        xyz=[float(x) for x in o.get('xyz').split()]
        rpy=[float(x) for x in (o.get('rpy') or '0 0 0').split()]
        ax=[float(x) for x in j.find('axis').get('xyz').split()]
        links.append(URDFLink(name=jn,origin_translation=xyz,origin_orientation=rpy,
                              rotation=ax,bounds=jr_limits(k),joint_type='revolute'))
        active.append(True)
    return Chain(name='arm',links=links,active_links_mask=active)
CHAIN=build_chain()

def fk(q):
    T=CHAIN.forward_kinematics([0]+list(q))
    return T[:3,3].copy(), T[:3,:3].copy()
def ik(pos,rot,q0):
    sol=CHAIN.inverse_kinematics(target_position=pos,target_orientation=rot,
                                 orientation_mode='all',initial_position=[0]+list(q0))
    return list(sol[1:])

# --- 모터 ---
port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)

_orig_term=None
def restore_term():
    if _orig_term is not None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, _orig_term)
def emergency_off(*_):
    restore_term()
    print("\n[긴급정지] 토크 OFF")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for i in ZERO_RAW: packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
    port.closePort()
    try: rclpy.shutdown()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT, emergency_off)

for i in ZERO_RAW:
    packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,SAFE_ACCEL)
    packet.write4ByteTxRx(port,i,ADDR_P_VEL,SAFE_VEL)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)

def read_joints():
    q=[0.0]*6
    for jidx in range(6):
        i=J2ID[jidx]
        p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT)
        if r==0: q[jidx]=DIR[i]*(s32(p)-ZERO_RAW[i])*math.pi/2048.0
    return q
def send_joints(q):
    for jidx in range(6):
        raw,i=rad_to_raw(jidx,q[jidx])
        packet.write4ByteTxRx(port,i,ADDR_GOAL, raw & 0xFFFFFFFF)

# --- ROS ---
rclpy.init(); node=Node('ee_jog')
pub=node.create_publisher(JointState,'joint_states',10)
def publish(q):
    m=JointState(); m.header.stamp=node.get_clock().now().to_msg()
    m.name=list(JOINTS); m.position=[float(x) for x in q]; pub.publish(m)

# --- 시작: 실물 현재 자세에서 출발 ---
q=read_joints()
pos,rot=fk(q)
publish(q)
ee_frame=False   # False=base축, True=EE축
msg=""

def render():
    sys.stdout.write("\033[2J\033[H")
    p,_=fk(q)
    frame="EE축" if ee_frame else "base축"
    print(f"EE 위치: x={p[0]*1000:+8.1f}  y={p[1]*1000:+8.1f}  z={p[2]*1000:+8.1f} (mm)")
    print(f"스텝: {STEPS_MM[step_i]}mm   좌표계: {frame}")
    print("관절(deg):", "  ".join(f"j{k+1}:{math.degrees(v):+.1f}" for k,v in enumerate(q)))
    print("-"*56)
    print("a/d=x-+  w/s=y+-  9/0=z+-  -/= 스텝  f=좌표계  b=base  p=위치  q=종료")
    if msg: print(msg)

def smooth_goto(q_from,q_to,T=MOVE_T,n=6):
    for k in range(1,n+1):
        t=k/n; t=t*t*(3-2*t)
        qq=[a+(b-a)*t for a,b in zip(q_from,q_to)]
        send_joints(qq); publish(qq)
        time.sleep(T/n)

def try_move(d_base):
    """d_base: base프레임 이동벡터(m). ee_frame이면 EE축 벡터를 회전."""
    global q,pos,rot,msg
    p_cur,r_cur=fk(q)
    d = r_cur@d_base if ee_frame else np.array(d_base)
    target=p_cur+d
    q_new=ik(target,r_cur,q)
    p_chk,_=fk(q_new)
    err=np.linalg.norm(p_chk-target)
    if err>IK_TOL:
        msg=f"[거부] IK 오차 {err*1000:.1f}mm - 작업범위 한계"
        return
    # 관절 급변 방지 (한 스텝에 25도 이상 뛰면 거부)
    if max(abs(a-b) for a,b in zip(q_new,q))>math.radians(25):
        msg="[거부] 관절 급변 - 다른 방향으로 시도"
        return
    smooth_goto(q,q_new)
    q=q_new; msg=f"이동 OK (IK오차 {err*1000:.2f}mm)"

_orig_term=termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)
render()
try:
    while True:
        r,_,_=select.select([sys.stdin],[],[],0.1)
        if not r:
            rclpy.spin_once(node,timeout_sec=0); continue
        c=sys.stdin.read(1)
        s=STEPS_MM[step_i]/1000.0
        if c=='q': emergency_off()
        elif c=='a': try_move([-s,0,0])
        elif c=='d': try_move([+s,0,0])
        elif c=='w': try_move([0,+s,0])
        elif c=='s': try_move([0,-s,0])
        elif c=='9': try_move([0,0,+s])
        elif c=='0': try_move([0,0,-s])
        elif c=='-': step_i=max(0,step_i-1)
        elif c=='=': step_i=min(len(STEPS_MM)-1,step_i+1)
        elif c=='f': ee_frame=not ee_frame
        elif c=='p':
            p,_=fk(q); msg=f"EE=({p[0]*1000:.1f}, {p[1]*1000:.1f}, {p[2]*1000:.1f})mm"
        elif c=='b':
            tgt=[0.0]*6
            # base로는 조인트 순서대로 천천히
            qq=list(q)
            for jidx in range(6):
                if abs(qq[jidx])<0.005: qq[jidx]=0.0; continue
                start=qq[jidx]
                for k in range(31):
                    t=k/30; t=t*t*(3-2*t)
                    qq[jidx]=start*(1-t)
                    send_joints(qq); publish(qq); time.sleep(0.05)
            q=qq; msg="base 도착"
        render()
except KeyboardInterrupt:
    emergency_off()
finally:
    restore_term()
