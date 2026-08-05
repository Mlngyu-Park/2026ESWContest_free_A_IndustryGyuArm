#!/usr/bin/env python3
"""
base -> 0725_1st -> 0725_seed -> 0725_1st -> base 실물 실행.
엔드 이펙터 수직/수평 축 이동 (내려갈 땐 z 마지막, 올라올 땐 z 먼저).
- 경로 전체 사전 IK 검증 후에만 이동
- 시간 기반 프로파일 (부드러움), 종료 시 원복
- Ctrl+C = 즉시 토크 OFF

실행: conda run --no-capture-output -n gyu_arm python play_seq_0725_real.py [--loop]
RViz 같이 보려면: ros2 launch Industry_gyu_arm_20260724_base_position jog.launch.py
"""
import sys, math, time, signal, threading
import numpy as np
import xml.etree.ElementTree as ET
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from dynamixel_sdk import PortHandler, PacketHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
LOOP="--loop" in sys.argv

PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112; ADDR_GOAL=116; ADDR_PRESENT=132
ADDR_DRIVE_MODE=10
SAFE_VEL=25; SAFE_ACCEL=8
PROFILE_MS=110; ACCEL_MS=35
URDF='/home/drboom/2026_project/urdf/Industry_gyu_arm_20260724_base_position/urdf/Industry_gyu_arm_20260724_base_position.urdf'

JOINTS=['j1','j2','j3','j4','j5','j6']
J2ID=[0,1,2,4,3,5]                                    # j4<->j5 배선 교차
ZERO_RAW={0:1517,1:2483,2:1562,3:2244,4:2545,5:2018}
DIR={0:-1,1:1,2:1,3:1,4:1,5:1}                         # j1 반전
LIMIT_RAW={0:(380,2571),1:(1479,3555),2:(1334,3382),3:(1271,3153),4:(1599,3524),5:(1970,3525)}

POSES={
    "base":     [0.0,    0.0,    0.0,    0.0, 0.0,    0.0],
    "0725_1st": [-0.26,  0.3551,-0.2928, 0.0,-0.648,  0.0],
    "0725_seed":[ 0.924, 0.5701,-0.3497, 0.0,-1.0521, 0.0],  # j3 한계 클램프(-20deg)
}
# (목표자세, 축순서): 내려갈 땐 z 마지막, 올라올 땐 z 먼저
SEQ=[("0725_1st",[0,1,2]), ("0725_seed",[0,1,2]),
     ("0725_1st",[2,1,0]), ("base",[2,0,1])]
WAY_DT=0.08

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
CHAIN=Chain(name='arm',links=links,active_links_mask=active)
def fk(q):
    T=CHAIN.forward_kinematics([0]+list(q)); return T[:3,3].copy(), T[:3,:3].copy()
def rot_aa(a,th):
    a=a/np.linalg.norm(a); x,y,z=a; c,s=np.cos(th),np.sin(th); C=1-c
    return np.array([[c+x*x*C,x*y*C-z*s,x*z*C+y*s],
                     [y*x*C+z*s,c+y*y*C,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,c+z*z*C]])
def plan_ee_axis(qa,qb,order):
    p0,r0=fk(qa); p1,r1=fk(qb)
    d=p1-p0; total=sum(abs(d[i]) for i in range(3))
    if total<0.003:
        return [[a+(b-a)*k/30 for a,b in zip(qa,qb)] for k in range(31)]
    z0=r0[:,2]; z1=r1[:,2]
    cz=np.clip(np.dot(z0,z1),-1,1); angz=np.arccos(cz)
    axz=np.cross(z0,z1)
    axz=axz/np.linalg.norm(axz) if np.linalg.norm(axz)>1e-9 else np.array([1,0,0.0])
    way=[list(qa)]; q=list(qa); cur=p0.copy(); traveled=0.0
    for ai in order:
        dist=d[ai]
        if abs(dist)<0.002:
            cur[ai]+=dist; traveled+=abs(dist); continue
        n=max(6,int(abs(dist)/0.003))
        for k in range(1,n+1):
            t=k/n; te=t*t*(3-2*t)
            pt=cur.copy(); pt[ai]+=dist*te
            frac=(traveled+abs(dist)*te)/total
            zt=rot_aa(axz,angz*frac)@z0
            sol=CHAIN.inverse_kinematics(target_position=pt,target_orientation=zt,
                                         orientation_mode='Z',initial_position=[0]+q)
            qn=list(sol[1:]); pchk,_=fk(qn)
            if np.linalg.norm(pchk-pt)>0.004: return None
            if max(abs(a-b) for a,b in zip(qn,q))>math.radians(15): return None
            way.append(qn); q=qn
        cur[ai]+=dist; traveled+=abs(dist)
    way[-1]=list(qb); return way

# --- 모터 ---
port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
def set_time_profile(on):
    for i in ZERO_RAW: packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
    for i in ZERO_RAW:
        dm,r,_=packet.read1ByteTxRx(port,i,ADDR_DRIVE_MODE)
        if r!=0: continue
        ndm=(dm|0x04) if on else (dm & ~0x04)
        if ndm!=dm: packet.write1ByteTxRx(port,i,ADDR_DRIVE_MODE,ndm)
    for i in ZERO_RAW:
        if on:
            packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,ACCEL_MS)
            packet.write4ByteTxRx(port,i,ADDR_P_VEL,PROFILE_MS)
        else:
            packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,SAFE_ACCEL)
            packet.write4ByteTxRx(port,i,ADDR_P_VEL,SAFE_VEL)
def emergency_off(*_):
    print("\n[긴급정지] 토크 OFF + 프로파일 원복")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for i in ZERO_RAW: packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
    try: set_time_profile(False)
    except Exception: pass
    port.closePort()
    try: rclpy.shutdown()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT, emergency_off)

def send_joints(q):
    for jidx in range(6):
        raw,i=rad_to_raw(jidx,q[jidx])
        packet.write4ByteTxRx(port,i,ADDR_GOAL, raw & 0xFFFFFFFF)
def read_joints():
    q=[0.0]*6
    for jidx in range(6):
        i=J2ID[jidx]
        p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT)
        if r==0: q[jidx]=DIR[i]*(s32(p)-ZERO_RAW[i])*math.pi/2048.0
    return q

# --- ROS (RViz 미러) ---
rclpy.init(); node=Node('play_seq_0725')
pub=node.create_publisher(JointState,'joint_states',10)
def publish(q):
    m=JointState(); m.header.stamp=node.get_clock().now().to_msg()
    m.name=list(JOINTS); m.position=[float(x) for x in q]; pub.publish(m)

# --- 경로 사전 계산/검증 (로봇 안 움직인 상태) ---
print("경로 계산 중 (수직/수평 축 이동)...")
paths=[]; cur="base"
for tgt,order in SEQ:
    w=plan_ee_axis(POSES[cur],POSES[tgt],order)
    if w is None:
        print(f"경로 실패: {cur}->{tgt} - 중단 (로봇 안 움직임)"); emergency_off()
    axes=''.join('xyz'[i] for i in order)
    print(f"  {cur}->{tgt} ({axes}) {len(w)}점 OK")
    paths.append((f"{cur}->{tgt} ({axes})",w)); cur=tgt
print("경로 전부 OK. 이동 시작.")

# --- 시간 프로파일 + 토크 ---
set_time_profile(True)
for i in ZERO_RAW: packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)

# 먼저 base로 (관절 순서대로, 안전)
print("먼저 base 자세로 (Ctrl+C = 즉시 정지)")
q=read_joints()
for jidx in range(6):
    if abs(q[jidx])<0.005: q[jidx]=0.0; continue
    print(f"  j{jidx+1} 이동")
    start=q[jidx]
    for k in range(51):
        t=k/50; t=t*t*(3-2*t)
        q[jidx]=start*(1-t)
        send_joints(q); publish(q); time.sleep(0.05)
    time.sleep(0.2)
time.sleep(0.6)

print("시퀀스: base->1st->seed->1st->base (수직/수평)" + (" 반복" if LOOP else ""))
while True:
    for label,way in paths:
        print("  ->",label)
        for qq in way:
            send_joints(qq); publish(qq); time.sleep(WAY_DT)
        time.sleep(0.8)
    if not LOOP: break

print("완료. Ctrl+C로 토크 해제.")
try:
    while True: time.sleep(1)
except KeyboardInterrupt:
    emergency_off()
