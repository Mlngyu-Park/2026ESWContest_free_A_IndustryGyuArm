#!/usr/bin/env python3
"""
base -> 0725_1st -> 0725_seed -> 0725_1st -> base 애니메이션 (RViz).
엔드 이펙터 '수직/수평 축 이동'만 사용:
  내려가는 구간 = 수평(x,y) 먼저, z 마지막
  올라오는 구간 = z 먼저, 수평 나중
반복. Ctrl+C 종료.
"""
import sys, time, math
import numpy as np
import xml.etree.ElementTree as ET
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
URDF='/home/drboom/2026_project/urdf/Industry_gyu_arm_20260724_base_position/urdf/Industry_gyu_arm_20260724_base_position.urdf'
JOINTS=['j1','j2','j3','j4','j5','j6']
J2ID=[0,1,2,4,3,5]
ZERO_RAW={0:1517,1:2483,2:1562,3:2244,4:2545,5:2018}
DIR={0:-1,1:1,2:1,3:1,4:1,5:1}
LIMIT_RAW={0:(380,2571),1:(1479,3555),2:(1334,3382),3:(1271,3153),4:(1599,3524),5:(1970,3525)}

POSES={
    "base":     [0.0,    0.0,    0.0,    0.0, 0.0,    0.0],
    "0725_1st": [-0.26,  0.3551,-0.2928, 0.0,-0.648,  0.0],
    "0725_seed":[ 0.924, 0.5701,-0.3497, 0.0,-1.0521, 0.0],  # j3 한계 클램프(-20deg)
}
# (목표자세, 축순서) - 내려갈 땐 z 마지막, 올라올 땐 z 먼저
SEQ=[("0725_1st",[0,1,2]), ("0725_seed",[0,1,2]),
     ("0725_1st",[2,1,0]), ("base",[2,0,1])]
DT=0.05

def jr_limits(jidx):
    i=J2ID[jidx]; lo,hi=LIMIT_RAW[i]
    a=DIR[i]*(lo-ZERO_RAW[i])*math.pi/2048.0
    b=DIR[i]*(hi-ZERO_RAW[i])*math.pi/2048.0
    return (min(a,b),max(a,b))
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

# ROS 먼저 시작 - 경로 계산 동안에도 base 자세를 발행해 RViz에 로봇이 보이게
rclpy.init(); node=Node("animate_seq_0725")
pub=node.create_publisher(JointState,"joint_states",10)
def publish(q):
    m=JointState(); m.header.stamp=node.get_clock().now().to_msg()
    m.name=list(JOINTS); m.position=[float(x) for x in q]; pub.publish(m)

import threading
_planning=True
def _heartbeat():
    while _planning:
        publish(POSES["base"]); time.sleep(0.3)
threading.Thread(target=_heartbeat,daemon=True).start()

# 경로 전부 미리 계산
print("경로 계산 중 (수직/수평 축 이동)... RViz에는 base 자세 표시 중")
paths=[]; cur="base"
for tgt,order in SEQ:
    w=plan_ee_axis(POSES[cur],POSES[tgt],order)
    if w is None:
        print(f"경로 실패: {cur}->{tgt} {order}"); sys.exit(1)
    axes=''.join('xyz'[i] for i in order)
    print(f"  {cur}->{tgt} ({axes}) {len(w)}점 OK")
    paths.append((f"{cur}->{tgt} ({axes})",w)); cur=tgt
_planning=False
print("경로 계산 완료. 애니메이션 시작.")
time.sleep(0.5)
try:
    while True:
        for label,way in paths:
            print("  ->",label)
            for q in way: publish(q); time.sleep(DT)
            time.sleep(0.5)
except KeyboardInterrupt:
    print("\n종료"); node.destroy_node(); rclpy.shutdown()
