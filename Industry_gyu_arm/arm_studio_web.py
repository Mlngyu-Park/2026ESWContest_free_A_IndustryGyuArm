#!/usr/bin/env python3
"""
ARM STUDIO WEB - 브라우저 통합 UI (Gradio). 파일 하나로 전부.

실행:
  cd /home/drboom/2026_project/Industry_gyu_arm
  conda run --no-capture-output -n gyu_arm python arm_studio_web.py
  -> 브라우저에서 http://localhost:7860 열림 (내부망 전용)

기능:
  - URDF 업로드(드래그앤드롭) -> 관절각 자동 역산 (축+xyz+메쉬 방법)
  - 웹 3D 뷰어로 로봇 자세 표시 (RViz 없이도 됨. RViz 켜져있으면 같이 갱신)
  - 관절 슬라이더 미세조정 (j6=그리퍼 포함)
  - 자세 복제 -> 그리퍼만 다른 변형 생성
  - 시퀀스 편집 + 이동방식 4종 + 속도 + 루프
  - PREVIEW(3D 애니메이션) / RUN(실물) / STOP(즉시 토크 OFF)
"""
import os, sys, json, math, time, struct, threading
import logging

# 브라우저가 업로드 중 연결을 끊으면 나는 무해한 예외 로그 억제
class _DropClientDisconnect(logging.Filter):
    def filter(self,rec):
        ei=getattr(rec,'exc_info',None)
        if ei and ei[0] is not None:
            # 예외 체인 어딘가에 ClientDisconnect 가 있으면 억제
            e=ei[1]
            while e is not None:
                if type(e).__name__=='ClientDisconnect': return False
                e=e.__cause__ or e.__context__
        try:
            return 'ClientDisconnect' not in rec.getMessage()
        except Exception:
            return True
for _n in ('uvicorn.error','uvicorn','starlette'):
    logging.getLogger(_n).addFilter(_DropClientDisconnect())

import numpy as np
import xml.etree.ElementTree as ET
import gradio as gr
import plotly.graph_objects as go
import trimesh, fast_simplification
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

# ROS(RViz 연동)는 선택사항 - 없어도 웹 3D로 동작
try:
    import rclpy
    from rclpy.node import Node as RosNode
    from sensor_msgs.msg import JointState
    rclpy.init()
    _ros=RosNode('arm_studio_web')
    _pub=_ros.create_publisher(JointState,'joint_states',10)
except Exception:
    _ros=None; _pub=None

# ================= 캘리브레이션 (2026-07-24 확정) =================
PORT='/dev/ttyACM0'; BAUD=1000000       # Feetech STS3215 (SO-ARM101)
# STS3215 레지스터
ADDR_TORQUE=40; ADDR_P_ACCEL=41; ADDR_P_VEL=46; ADDR_GOAL=42; ADDR_PRESENT=56
ADDR_KP=21; ADDR_KD=22; ADDR_KI=23     # 위치 PID (공장값 32/32/0 = 이 팔엔 약함)
# 측정(j2, 2왕복): Kp32/Kd32/Ki0 -> 정지오차 18raw(1.58도)
#                 Kp64/Kd64/Ki4 -> 정지오차  4raw(0.35도), 진동 0
# Ki 가 중력으로 남는 정상편차(내릴 때만 커지는 오차)를 없애준다.
GAIN_KP=64; GAIN_KD=64; GAIN_KI=4
ADDR_TORQUE_LIMIT=48; ADDR_MODE=33
SAFE_VEL=150; SAFE_ACCEL=8; TORQUE_LIMIT=250   # 1.5배속 (100->150)
# 웨이포인트 추종: 실물이 이만큼 근접하면 다음 목표로 (너무 좁으면 느리고 넓으면 뒤처짐)
FOLLOW_TOL=60          # raw (약 5도)
FOLLOW_TIMEOUT=1.5     # 한 웨이포인트 최대 대기 (초)
# 구간 끝(자세 도달) 허용치. 작을수록 정확하지만 도달에 시간이 더 걸림
#   20 raw = 1.76도 -> EE 약 10mm / 6 raw = 0.53도 -> EE 약 3mm
END_TOL=6              # 정밀 모드 기본
END_TIMEOUT=8.0
END_SETTLE=0.25        # 도착 후 진동이 가라앉기를 기다리는 시간(초)
# 웨이포인트 간 최대 관절 변화. 크면 경로가 잘 풀리지만 급가속 위험
JUMP_LIMIT_DEG=20
PER_ID_VEL={6:300}; PER_ID_TQ={6:250}                 # 그리퍼는 저속에선 안 움직임

# ===== base URDF 자동 탐색 (하드코딩 없음) =====
# 우선순위: 1) 환경변수 ARM_BASE_URDF  2) base_urdf.txt 에 적힌 경로
#           3) urdf/ 아래에서 이름에 'base' 가 들어간 최신 패키지
URDF_ROOT='/home/drboom/2026_project/urdf'
# 로봇 패밀리별 URDF 디렉토리 분할 (2026-08-07):
#   urdf/sts3215/    SO-ARM101 (Feetech STS3215) - 현재 사용
#   urdf/dynamixel/  옛 Dynamixel 로봇 (XC430/XM430)
# base 탐색은 활성 패밀리 폴더 안에서만 한다 (양쪽에 base 가 있어 섞이면 위험)
ROBOT_FAMILY=os.environ.get('ARM_ROBOT_FAMILY','sts3215')
BASE_URDF_CFG=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'base_urdf.txt')
def _find_base_urdf():
    p=os.environ.get('ARM_BASE_URDF')
    if p and os.path.exists(p): return p
    if os.path.exists(BASE_URDF_CFG):
        p=open(BASE_URDF_CFG).read().strip()
        if p and os.path.exists(p): return p
    import glob as _g
    # 1순위: urdf/<패밀리>/<패키지>/urdf/*.urdf   2순위(구버전 호환): urdf/<패키지>/urdf/*.urdf
    for pat in (os.path.join(URDF_ROOT,ROBOT_FAMILY,'*','urdf','*.urdf'),
                os.path.join(URDF_ROOT,'*','urdf','*.urdf')):
        cands=[u for u in _g.glob(pat) if not u.endswith('_limits.urdf')]
        if cands: break
    if not cands: return None
    # '패키지 폴더명' 또는 파일명에 base 가 들어간 것을 우선 (자세 URDF 오선택 방지)
    def _is_base(u):
        pkg=os.path.basename(os.path.dirname(os.path.dirname(u))).lower()
        return 'base' in pkg or 'base' in os.path.basename(u).lower()
    base=[u for u in cands if _is_base(u)]
    if len(base)>1:
        print(f"주의: base 후보가 {len(base)}개 - 최신 것을 사용합니다")
        for u in base: print("   ",u)
    elif not base:
        print("주의: 이름에 'base' 가 있는 URDF 가 없어 최신 파일을 사용합니다")
        print("      의도한 게 아니면 base_urdf.txt 에 경로를 지정하세요")
    pick=base or cands
    return max(pick,key=os.path.getmtime)      # 가장 최근 것

BASE_URDF=_find_base_urdf()
if BASE_URDF is None:
    print("="*60)
    print(" base URDF 를 찾지 못했습니다.")
    print(f" {URDF_ROOT}/{ROBOT_FAMILY}/<패키지>/urdf/*.urdf 를 넣거나,")
    print(f" {BASE_URDF_CFG} 에 경로를 적어주세요.")
    print("="*60)
    raise SystemExit(1)
print(f"base URDF: {BASE_URDF}")
JOINTS=['j1','j2','j3','j4','j5','j6']
J2ID=[1,2,3,4,5,6]                                    # j1~j6 = 모터 ID 1~6 (교차 없음)
# 2026-07-27 STS3215 캘리브레이션
from sts_zero import ZERO_RAW as _Z
from sts_limits import LIMIT_RAW as _L
from sts_gripper import GRIPPER_CLOSE, GRIPPER_OPEN
ZERO_RAW=dict(_Z)
LIMIT_RAW=dict(_L)
DIR={1:-1, 2:-1, 3:-1, 4:-1, 5:-1, 6:-1}              # 전 관절 반전 (실측)
GRIP_ID=6
# 프로젝트(자세/시퀀스/보정/녹화)는 URDF 패키지 폴더 안에 저장한다.
#   -> URDF 를 바꾸면 그 URDF 의 데이터가 따라오고, 서로 섞이지 않는다.
#   -> SolidWorks 에서 URDF 를 재익스포트해도 calib.json 은 지워지지 않는다.
URDF_PKG_DIR=os.path.dirname(os.path.dirname(BASE_URDF))   # .../<패키지>/
PROJECT_FILE=os.path.join(URDF_PKG_DIR,'calib.json')
_LEGACY_PROJECT='/home/drboom/2026_project/Industry_gyu_arm/arm_studio_project.json'
print(f"보정 데이터: {PROJECT_FILE}")

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

# ================= 기구학 =================
def _R(rpy):
    r,p,y=rpy
    Rx=np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry=np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz=np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return Rz@Ry@Rx
def _rot(axis,th):
    a=np.asarray(axis,float); a=a/np.linalg.norm(a)
    x,y,z=a; c,s=np.cos(th),np.sin(th); C=1-c
    return np.array([[c+x*x*C,x*y*C-z*s,x*z*C+y*s],
                     [y*x*C+z*s,c+y*y*C,y*z*C-x*s],
                     [z*x*C-y*s,z*y*C+x*s,c+z*z*C]])
def _parse_urdf(path):
    root=ET.parse(path).getroot(); d={}
    for j in root.findall('joint'):
        o=j.find('origin'); a=j.find('axis')
        rpy=np.array([float(x) for x in (o.get('rpy') or '0 0 0').split()])
        xyz=np.array([float(x) for x in (o.get('xyz') or '0 0 0').split()])
        axis=np.array([float(x) for x in (a.get('xyz') if a is not None else '0 0 1').split()])
        d[j.get('name')]=dict(rpy=rpy,xyz=xyz,axis=axis)
    return d
BASE_DEF=_parse_urdf(BASE_URDF)
def build_chain():
    links=[OriginLink()]; active=[False]
    for k,jn in enumerate(JOINTS):
        b=BASE_DEF[jn]
        links.append(URDFLink(name=jn,origin_translation=list(b['xyz']),
                              origin_orientation=list(b['rpy']),rotation=list(b['axis']),
                              bounds=jr_limits(k),joint_type='revolute'))
        active.append(True)
    return Chain(name='arm',links=links,active_links_mask=active)
CHAIN=build_chain()
def fk(q):
    T=CHAIN.forward_kinematics([0]+list(q)); return T[:3,3].copy(), T[:3,:3].copy()
def fk_points(q):
    frames=CHAIN.forward_kinematics([0]+list(q),full_kinematics=True)
    return np.array([f[:3,3] for f in frames])

def _read_stl(path,maxtri=20000):
    with open(path,'rb') as f:
        f.read(80); n=struct.unpack('<I',f.read(4))[0]; n=min(n,maxtri)
        pts=np.zeros((n,3))
        for i in range(n):
            data=f.read(50); pts[i]=struct.unpack('<3f',data[12:24])
    return pts

def solve_pose_from_urdf(urdf_path,l6_path=None):
    new=_parse_urdf(urdf_path)
    M=np.eye(3); thetas=[]
    for k,nk in enumerate(JOINTS):
        b=BASE_DEF[nk]; a=b['axis']/np.linalg.norm(b['axis'])
        C=_R(b['rpy']).T@M@_R(b['rpy'])
        if k+1<len(JOINTS):
            nk1=JOINTS[k+1]
            pairs=[(BASE_DEF[nk1]['axis']/np.linalg.norm(BASE_DEF[nk1]['axis']),
                    new[nk1]['axis']/np.linalg.norm(new[nk1]['axis']))]
            vb=BASE_DEF[nk1]['xyz']; vn=new[nk1]['xyz']
            if np.linalg.norm(vb)>1e-6:
                pairs.append((vb/np.linalg.norm(vb),vn/np.linalg.norm(vn)))
            num=den=0.0
            for v,t_raw in pairs:
                t=C.T@t_raw
                vp=v-a*np.dot(a,v); tp=t-a*np.dot(a,t)
                if np.linalg.norm(vp)<1e-6: continue
                num+=np.dot(a,np.cross(vp,tp)); den+=np.dot(vp,tp)
            th=math.atan2(num,den) if (abs(num)>1e-12 or abs(den)>1e-12) else 0.0
        else:
            th=0.0
        M=C@_rot(a,th); thetas.append(th)
    warn=""
    try:
        base_mesh=os.path.join(os.path.dirname(BASE_URDF),'..','meshes','l6.STL')
        new_mesh=l6_path or os.path.join(os.path.dirname(urdf_path),'..','meshes','l6.STL')
        vb=_read_stl(base_mesh); vn=_read_stl(new_mesh)
        if len(vb)==len(vn):
            cb=vb.mean(0); cn=vn.mean(0)
            H=(vb-cb).T@(vn-cn); U,S,Vt=np.linalg.svd(H)
            D=np.diag([1,1,np.sign(np.linalg.det(Vt.T@U.T))])
            Rm=Vt.T@D@U.T
            M5=np.eye(3)
            for k,nk in enumerate(JOINTS[:-1]):
                b=BASE_DEF[nk]; a=b['axis']/np.linalg.norm(b['axis'])
                C=_R(b['rpy']).T@M5@_R(b['rpy']); M5=C@_rot(a,thetas[k])
            b6=BASE_DEF['j6']; a6=b6['axis']/np.linalg.norm(b6['axis'])
            C6=_R(b6['rpy']).T@M5@_R(b6['rpy'])
            Rj=C6.T@Rm
            ang=np.arccos(np.clip((np.trace(Rj)-1)/2,-1,1))
            vec=np.array([Rj[2,1]-Rj[1,2],Rj[0,2]-Rj[2,0],Rj[1,0]-Rj[0,1]])
            sign=np.sign(np.dot(vec,a6)) if np.linalg.norm(vec)>1e-9 else 1
            thetas[5]=float(ang*sign)
        else: warn="l6 mesh mismatch: j6=0"
    except Exception as e:
        warn=f"j6 mesh skip: {e}"
    return [float(t) for t in thetas], warn

# ================= 경로 계획 =================
def plan_joint_seq(qa,qb,steps=50,level=True):
    """관절 순서대로 이동.
    level=True 면 j2,j3,j4 를 한 묶음으로 동시에 움직인다.
    (세 관절 축이 같아 합이 그리퍼 기울기를 결정하므로, 하나씩 돌리면
     중간에 합이 깨져 기울어짐 = 물컵을 쏟음)"""
    way=[list(qa)]; q=list(qa)
    groups=[[0],[1,2,3],[4],[5]] if level else [[j] for j in range(6)]
    for grp in groups:
        if all(abs(qb[j]-q[j])<0.005 for j in grp):
            for j in grp: q[j]=qb[j]
            continue
        st={j:q[j] for j in grp}
        for k in range(1,steps+1):
            t=k/steps; t=t*t*(3-2*t)
            for j in grp: q[j]=st[j]+(qb[j]-st[j])*t
            way.append(list(q))
    way[-1]=list(qb); return way,0.05
def plan_joint_all(qa,qb,steps=50):
    way=[]
    for k in range(steps+1):
        t=k/steps; t=t*t*(3-2*t)
        way.append([a+(b-a)*t for a,b in zip(qa,qb)])
    way[-1]=list(qb); return way,0.05
def _ee_common(qa,qb):
    p0,r0=fk(qa); p1,r1=fk(qb)
    z0=r0[:,2]; z1=r1[:,2]
    cz=np.clip(np.dot(z0,z1),-1,1); angz=np.arccos(cz)
    axz=np.cross(z0,z1)
    axz=axz/np.linalg.norm(axz) if np.linalg.norm(axz)>1e-9 else np.array([1,0,0.0])
    return p0,p1,z0,axz,angz
def plan_ee_line(qa,qb,dens=1,rotmode='blend'):
    p0,p1,z0,axz,angz=_ee_common(qa,qb)
    d=p1-p0; L=np.linalg.norm(d)
    if L<0.003: return plan_joint_all(qa,qb)
    # 거리 기준 + 자세 회전 기준 중 촘촘한 쪽 (회전이 크면 점프 방지)
    rot_n=int(math.degrees(angz)/1.5) if rotmode=='blend' else 0
    n=max(10,int(L/0.003),rot_n)*dens
    way=[list(qa)]; q=list(qa)
    if rotmode=='before':                 # 이동 전에 손목 회전을 끝냄
        q=_rot_only(way,q,p0,z0,axz,angz,0.0,1.0,dens)
        if q is None: return None
    for k in range(1,n+1):
        t=k/n; te=t*t*(3-2*t)
        pt=p0+d*te
        if rotmode=='before':   frac=1.0
        elif rotmode=='after':  frac=0.0
        else:                   frac=te
        zt=_rot(axz,angz*frac)@z0
        sol=CHAIN.inverse_kinematics(target_position=pt,target_orientation=zt,
                                     orientation_mode='Z',initial_position=[0]+q)
        qn=list(sol[1:]); pchk,_=fk(qn)
        if np.linalg.norm(pchk-pt)>0.004: return None
        if max(abs(a-b) for a,b in zip(qn,q))>math.radians(JUMP_LIMIT_DEG):
            return None
        way.append(qn); q=qn
    if rotmode=='after':                  # 이동을 마친 뒤 손목 회전
        q=_rot_only(way,q,p1,z0,axz,angz,0.0,1.0,dens)
        if q is None: return None
    way[-1]=list(qb)
    if rotmode=='lock':                   # j5 모터만 안 돌림 (경로는 그대로)
        for fr in way: fr[4]=qa[4]
    return way,0.08
def _rot_only(way,q,pos,z0,axz,angz,f0,f1,dens):
    """제자리에서 그리퍼 방향만 f0->f1 로 회전시키는 구간을 way 에 추가.
    실패 시 None, 성공 시 마지막 관절각 반환."""
    if abs(angz*(f1-f0))<math.radians(0.5): return q
    n=max(6,int(math.degrees(abs(angz*(f1-f0)))/1.5))*dens
    for k in range(1,n+1):
        t=k/n; te=t*t*(3-2*t)
        zt=_rot(axz,angz*(f0+(f1-f0)*te))@z0
        sol=CHAIN.inverse_kinematics(target_position=pos,target_orientation=zt,
                                     orientation_mode='Z',initial_position=[0]+q)
        qn=list(sol[1:]); pchk,_=fk(qn)
        if np.linalg.norm(pchk-pos)>0.004: return None
        if max(abs(a-b) for a,b in zip(qn,q))>math.radians(JUMP_LIMIT_DEG):
            return None
        way.append(qn); q=qn
    return q

def plan_ee_axis(qa,qb,order,dens=1,rotmode='blend'):
    p0,p1,z0,axz,angz=_ee_common(qa,qb)
    d=p1-p0; total=sum(abs(d[i]) for i in range(3))
    if total<0.003: return plan_joint_all(qa,qb)
    # lock 은 IK 제약이 아니라, 계획을 끝낸 뒤 j5 값만 덮어쓰는 방식(아래)
    way=[list(qa)]; q=list(qa); cur=p0.copy(); traveled=0.0
    if rotmode=='before':                 # 이동 전에 손목 회전을 끝냄
        q=_rot_only(way,q,p0,z0,axz,angz,0.0,1.0,dens)
        if q is None: return None
    for ai in order:
        dist=d[ai]
        if abs(dist)<0.002:
            cur[ai]+=dist; traveled+=abs(dist); continue
        # 이 축 구간이 담당하는 자세 회전량만큼도 촘촘히 (관절 점프 방지)
        rot_share=(math.degrees(angz)*abs(dist)/total
                   if rotmode=='blend' else 0.0)
        n=max(6,int(abs(dist)/0.003),int(rot_share/1.5))*dens
        for k in range(1,n+1):
            t=k/n; te=t*t*(3-2*t)
            pt=cur.copy(); pt[ai]+=dist*te
            if rotmode=='before':   frac=1.0      # 이미 회전 완료 -> 유지
            elif rotmode=='after':  frac=0.0      # 아직 회전 안 함 -> 유지
            else:                   frac=(traveled+abs(dist)*te)/total
            zt=_rot(axz,angz*frac)@z0
            sol=CHAIN.inverse_kinematics(target_position=pt,target_orientation=zt,
                                         orientation_mode='Z',initial_position=[0]+q)
            qn=list(sol[1:]); pchk,_=fk(qn)
            if np.linalg.norm(pchk-pt)>0.004: return None
            if max(abs(a-b) for a,b in zip(qn,q))>math.radians(JUMP_LIMIT_DEG):
                return None
            way.append(qn); q=qn
        cur[ai]+=dist; traveled+=abs(dist)
    if rotmode=='after':                  # 이동을 마친 뒤 손목 회전
        q=_rot_only(way,q,p1,z0,axz,angz,0.0,1.0,dens)
        if q is None: return None
    way[-1]=list(qb)
    if rotmode=='lock':                   # j5 모터만 안 돌림 (경로는 그대로)
        for fr in way: fr[4]=qa[4]
    return way,0.08
AXIS_ORDERS={'x-y-z':[0,1,2],'x-z-y':[0,2,1],'y-x-z':[1,0,2],
             'y-z-x':[1,2,0],'z-x-y':[2,0,1],'z-y-x':[2,1,0]}
# ===== 그리퍼 수평 유지 (물컵 규칙) =====
# 관절 축:  j1=(0,0,1) Z회전 -> 기울기 무관
#           j2,j3,j4=(1,0,0) 같은 X축 -> 3개 조합으로 수평 유지 가능
#           j5=(0,1,0) Y축 -> 돌리면 무조건 기울어짐 => 항상 고정
J5_IDX=4                      # j5 = 인덱스 4
LEVEL_ALWAYS=True             # 모든 이동에서 그리퍼 수평 유지

def level_offset(off):
    """미세조정 오프셋이 그리퍼 수평을 깨지 않게 보정한다.

    j2,j3,j4 는 축이 같으므로 '합'이 기울기를 만든다.
    사용자가 j3 에 +5도를 주면 그리퍼가 5도 기운다 -> 그만큼 j4 로 되돌린다.
    j5 는 회전하면 무조건 기울므로 오프셋을 쓰지 않는다.
    """
    if not LEVEL_ALWAYS: return list(off)
    o=list(off)
    tilt=o[1]+o[2]+o[3]          # j2+j3+j4 합 = 기울기
    o[3]-=tilt                   # j4 로 상쇄 -> 합 0
    o[J5_IDX]=0.0                # j5 오프셋 금지
    return o

def lock_j5(way,qa):
    """경로 전체에서 j5 를 출발 각도로 고정 (기울기 원인 제거)."""
    for f in way: f[J5_IDX]=qa[J5_IDX]
    return way

def plan_ee_level(qa,qb,order,dens=1,down=None):
    """그리퍼를 x-y 평면에 평행(수직 유지)하게 고정한 채 축별 이동.
    down=None 이면 출발 자세의 z 방향(위/아래)을 그대로 유지."""
    p0,r0=fk(qa); p1,_=fk(qb)
    z_target=np.array([0.0,0.0,1.0])
    if down is None:
        down = r0[:,2][2] < 0          # 출발 시 아래를 보고 있으면 계속 아래로
    if down: z_target=-z_target
    d=p1-p0; total=sum(abs(d[i]) for i in range(3))
    if total<0.003: return plan_joint_all(qa,qb)
    way=[list(qa)]; q=list(qa); cur=p0.copy()
    for ai in order:
        dist=d[ai]
        if abs(dist)<0.002:
            cur[ai]+=dist; continue
        n=max(6,int(abs(dist)/0.003))*dens
        for k in range(1,n+1):
            t=k/n; te=t*t*(3-2*t)
            pt=cur.copy(); pt[ai]+=dist*te
            sol=CHAIN.inverse_kinematics(target_position=pt,
                                         target_orientation=z_target,
                                         orientation_mode='Z',
                                         initial_position=[0]+q)
            qn=list(sol[1:]); pchk,_=fk(qn)
            if np.linalg.norm(pchk-pt)>0.004: return None
            if max(abs(a-b) for a,b in zip(qn,q))>math.radians(JUMP_LIMIT_DEG):
                return None
            way.append(qn); q=qn
        cur[ai]+=dist
    way[-1]=list(q)          # 목표 자세로 덮어쓰지 않음 (수직 유지가 우선)
    return way,0.08

# 모든 모드에서 j5 고정 -> 그리퍼 수평 유지가 기본
MODES=['관절 순서대로 (수평유지)','관절 동시 (수평유지)',
       'EE 직선 (수평유지)','EE 축별 이동 (수평유지)',
       'EE 축별 · 자세까지 수평보정']

# ================= 간편 이동 (뽑기 -> 수평 -> 하강) =================
# 규칙
#   j5/j6 은 IK 변수에서 아예 제외(4축 체인) -> 수평이 절대 깨지지 않음
#   ① 뽑기 : 현재 x,y 유지한 채 도달 가능한 최대 z 로 수직 상승
#   ② 수평 : x-y 경로 이동. 높이 = 경로 전체에서 유지 가능한 최소 z
#   ③ 하강 : 목표 x,y 에서 목표 z 로 수직 하강
#   이동높이 < 목표z+여유 이면: 하강 여유가 확보되는 지점에서 미리 상승
MODE_SIMPLE='간편 이동 (뽑기→수평→하강)'

_CHAIN_L=None
def _chain_level():
    """j5,j6 을 잠근 4축 체인 (수평 유지 전용 IK)."""
    global _CHAIN_L
    if _CHAIN_L is None:
        links=[OriginLink()]; active=[False]
        for k,jn in enumerate(JOINTS):
            b=BASE_DEF[jn]
            links.append(URDFLink(name=jn,origin_translation=list(b['xyz']),
                                  origin_orientation=list(b['rpy']),
                                  rotation=list(b['axis']),
                                  bounds=jr_limits(k),joint_type='revolute'))
            active.append(k<4)               # j1~j4 만 활성
        _CHAIN_L=Chain(name='arm_level',links=links,active_links_mask=active)
    return _CHAIN_L

_UPV=np.array([0.,0.,1.])
def _seed_list(extra=None):
    seeds=[]
    if extra is not None: seeds.append(list(extra))
    for p in S['poses']: seeds.append(list(p['angles']))
    return seeds or [[0.0]*6]

def ik_level(pt,seeds,j5,j6):
    """수평 유지 IK (j5/j6 잠금). 초기값 여러 개 시도 -> 관절6개 or None.
    (초기값이 나쁘면 도달 가능한 점도 실패 판정이 나므로 다중 시도 필수)"""
    ch=_chain_level(); tgt=np.asarray(pt,float)
    for s in seeds:
        init=[0.0]+list(s); init[5]=j5; init[6]=j6
        try:
            sol=ch.inverse_kinematics(target_position=tgt,
                                      target_orientation=_UPV,
                                      orientation_mode='Z',
                                      initial_position=init)
        except Exception:
            continue
        q=list(sol[1:7]); got,R=fk(q)
        if np.linalg.norm(got-tgt)<0.004 and R[2,2]>0.999:
            return q
    return None

_MAXZ_R={}                    # (x,y 1cm 반올림) -> 최대 z
# 주의: 반경만으로 캐시하면 안 된다 - 손목 오프셋 때문에 방향마다 상한이 다르다
_Z_LO,_Z_HI=0.03,0.55
def maxz_at(x,y,seed=None,j5=None,j6=0.0,hint=None):
    """(x,y) 에서 수평 유지로 도달 가능한 최대 z. 없으면 None.
    hint: 이웃 지점의 상한 (거기서부터 스캔해 IK 횟수 절약)."""
    rk=(round(x,2),round(y,2))
    if rk in _MAXZ_R: return _MAXZ_R[rk]
    seeds=_seed_list(seed)
    if j5 is None: j5=seeds[0][4]
    z=min(_Z_HI,hint+0.06) if hint else _Z_HI
    found=None
    while z>=_Z_LO:                      # 위에서부터 2cm 스캔
        if ik_level((x,y,z),seeds,j5,j6) is not None: found=z; break
        z-=0.02
    if found is None:
        _MAXZ_R[rk]=None; return None
    lo,hi=found,min(found+0.02,_Z_HI)    # 이분탐색으로 다듬기
    for _ in range(4):
        mid=(lo+hi)/2
        if ik_level((x,y,mid),seeds,j5,j6) is not None: lo=mid
        else: hi=mid
    _MAXZ_R[rk]=lo; return lo

# 주의: 팔 기둥(j1 회전축)은 base_link 원점이 아니라 j1 origin 에 있다!
_J1_XY=tuple(BASE_DEF['j1']['xyz'][:2])      # (0.240, -0.235)

# j1 방위각 한계: 로봇 뒤쪽(j1 이 못 도는 방향)은 어떤 z 로도 도달 불가
# 방위각은 반드시 '기둥' 기준으로 잰다 (원점 기준 아님)
_AZIM0=None
def _azim_ok(x,y,margin=math.radians(8)):
    global _AZIM0
    cx,cy=_J1_XY
    if _AZIM0 is None:
        p,_=fk([0.0]*6); _AZIM0=math.atan2(p[1]-cy,p[0]-cx)  # j1=0 EE 방위각
    lo,hi=jr_limits(0)
    az=math.atan2(y-cy,x-cx)-_AZIM0
    az=(az+math.pi)%(2*math.pi)-math.pi
    return lo-margin<=az<=hi+margin

# ---- 2D 장애물 격자 (위에서 본 투영, 1cm 셀) ----
G2_MIN,G2_MAX,G2_RES=-0.45,0.45,0.01
G2_N=int(round((G2_MAX-G2_MIN)/G2_RES))
def _cell(x,y):
    return (max(0,min(G2_N-1,int((x-G2_MIN)/G2_RES))),
            max(0,min(G2_N-1,int((y-G2_MIN)/G2_RES))))
def _cxy(i,j): return (G2_MIN+(i+0.5)*G2_RES, G2_MIN+(j+0.5)*G2_RES)

def _pt_in_tri(p,tri):
    (x,y)=p; (x1,y1),(x2,y2),(x3,y3)=tri
    d1=(x-x2)*(y1-y2)-(x1-x2)*(y-y2)
    d2=(x-x3)*(y2-y3)-(x2-x3)*(y-y3)
    d3=(x-x1)*(y3-y1)-(x3-x1)*(y-y1)
    neg=(d1<0)or(d2<0)or(d3<0); pos=(d1>0)or(d2>0)or(d3>0)
    return not(neg and pos)

_OBS={'grid':None,'dist':None,'base':None,'ids':{}}
def _fill_mesh2d(g,path,scale):
    """메시를 위에서 본 모양으로 격자 g 에 채운다."""
    try: mm=trimesh.load(path)
    except Exception: return
    v=np.asarray(mm.vertices,float)*scale
    f=np.asarray(mm.faces,int)
    for tri in f:
        pts=v[tri][:,:2]
        i0,j0=_cell(*pts.min(0)); i1,j1=_cell(*pts.max(0))
        for i in range(i0,i1+1):
            for j in range(j0,j1+1):
                if not g[i,j] and _pt_in_tri(_cxy(i,j),pts): g[i,j]=True
        g[_cell(*pts[0])]=True             # 셀보다 작은 삼각형 보강

def build_obs_grid(force=False):
    """플레이트/로봇 제외 환경 STL 을 위에서 본 모양으로 격자에 채운다.
    로봇 base(받침대) 는 별도 격자로 -> 경로 차단 + 맵에 경계 표시."""
    if _OBS['grid'] is not None and not force: return _OBS['grid']
    g=np.zeros((G2_N,G2_N),bool); per={}
    for e in ENV_SHOW:
        mid=e['id']
        if 'plate_bottom' in mid.lower(): continue     # 바닥은 장애물 아님
        path,scale=MESH_REG[mid]
        mk=np.zeros((G2_N,G2_N),bool)
        _fill_mesh2d(mk,path,scale)
        per[mid]=mk; g|=mk
    _OBS['ids']=per
    # 로봇 base 발자국 (m 단위 링크 메시, base_link 프레임 그대로)
    gb=np.zeros((G2_N,G2_N),bool)
    bp=os.path.join(MESH_DIR,'base_link.STL')
    if os.path.exists(bp): _fill_mesh2d(gb,bp,1.0)
    _OBS['base']=gb
    # 거리장 (BFS, 칸 단위) - "장애물에서 먼 칸" 선호에 사용. base 도 포함
    import collections
    dist=np.full((G2_N,G2_N),10**9,int)
    dq=collections.deque()
    for i in range(G2_N):
        for j in range(G2_N):
            if g[i,j] or gb[i,j]: dist[i,j]=0; dq.append((i,j))
    while dq:
        i,j=dq.popleft()
        for a,b in ((i+1,j),(i-1,j),(i,j+1),(i,j-1)):
            if 0<=a<G2_N and 0<=b<G2_N and dist[a,b]>dist[i,j]+1:
                dist[a,b]=dist[i,j]+1; dq.append((a,b))
    _OBS['grid'],_OBS['dist']=g,dist
    return g

def _corners(cells):
    """격자 경로 -> 방향이 꺾이는 모서리만 남긴 (x,y) 목록."""
    if len(cells)<2: return [_cxy(*c) for c in cells]
    out=[_cxy(*cells[0])]
    for k in range(1,len(cells)-1):
        d0=(cells[k][0]-cells[k-1][0],cells[k][1]-cells[k-1][1])
        d1=(cells[k+1][0]-cells[k][0],cells[k+1][1]-cells[k][1])
        if d0!=d1: out.append(_cxy(*cells[k]))
    out.append(_cxy(*cells[-1])); return out

def _feas2d(x,y):
    """(x,y) 가 팔의 물리적 도달 범위(기둥 기준 반경+방위각) 안인가."""
    cx,cy=_J1_XY; r=math.hypot(x-cx,y-cy)
    return 0.06<=r<=0.32 and _azim_ok(x,y)

def grid_path(pxy,qxy,clear_m=0.05,prefer='x',obstacles=True):
    """직교 격자 경로 탐색. -> (모서리 목록, 설명) / (None, 사유)
    obstacles=False 면 장애물은 무시하고 '도달 가능 영역'만으로 경로를 찾는다.
    우선순위: ① z 높게 유지(반경 작은 쪽) ② 이동량 최소 ③ 랜덤"""
    import heapq,random
    build_obs_grid(); d=_OBS['dist']
    ca,cb=_cell(*pxy),_cell(*qxy)
    cc=max(1,int(round(clear_m/G2_RES)))
    blocked=(d<cc).copy() if obstacles else np.zeros((G2_N,G2_N),bool)
    cx,cy=_J1_XY
    for i in range(G2_N):                   # 도달 불가 영역은 항상 금지
        for j in range(G2_N):
            if blocked[i,j]: continue
            if not _feas2d(*_cxy(i,j)): blocked[i,j]=True
    for (ci,cj) in (ca,cb):                 # 시작/도착 주변은 작업 지점 -> 해제
        r=cc+2
        for i in range(max(0,ci-r),min(G2_N,ci+r+1)):
            for j in range(max(0,cj-r),min(G2_N,cj+r+1)):
                if (i-ci)**2+(j-cj)**2<=r*r: blocked[i,j]=False
    if _OBS['base'] is not None: blocked|=_OBS['base']   # 로봇 base 는 항상 금지
    if blocked[ca] or blocked[cb]: return None,'시작/도착이 금지 영역'
    if ca==cb: return [pxy,qxy],'제자리'
    # 단순 L자 경로(선호축 먼저)가 뚫려 있으면 그대로 사용
    def _l_cells(o):
        cells=[ca]; cur=list(ca)
        for ax in o:
            t=cb[ax]
            while cur[ax]!=t:
                cur[ax]+=1 if t>cur[ax] else -1; cells.append(tuple(cur))
        return cells
    for o in (((0,1),(1,0)) if prefer=='x' else ((1,0),(0,1))):
        cells=_l_cells(o)
        if not any(blocked[c] for c in cells):
            return _corners(cells),('x먼저' if o==(0,1) else 'y먼저')+' 직행'
    # 다익스트라: z 유지(반경) > 거리 > 랜덤, 꺾임 최소화
    rmax=abs(G2_MAX)*1.42
    def cellcost(c):
        x,y=_cxy(*c); r=math.hypot(x-cx,y-cy)   # 기둥에서의 거리 (가까울수록 높은 z)
        near=max(0,(cc+3)-d[c])            # 장애물 근접 벌점 -> 통로 중앙 선호
        return 1.0 + 2.0*(r/rmax) + 0.6*near + random.random()*1e-6
    pq=[(0.0,ca,None)]; best={}; par={}
    while pq:
        cost,c,pdir=heapq.heappop(pq)
        if c==cb: break
        if best.get((c,pdir),1e18)<cost: continue
        for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
            n=(c[0]+di,c[1]+dj)
            if not(0<=n[0]<G2_N and 0<=n[1]<G2_N) or blocked[n]: continue
            nc=cost+cellcost(n)+(0.3 if pdir is not None and pdir!=(di,dj) else 0)
            key=(n,(di,dj))
            if nc<best.get(key,1e18):
                best[key]=nc; par[key]=(c,pdir)
                heapq.heappush(pq,(nc,n,(di,dj)))
    ends=[k for k in best if k[0]==cb]
    if not ends: return None,'우회 경로 없음 (여유 거리를 줄여보세요)'
    k=min(ends,key=lambda k:best[k])
    cells=[]
    while k is not None:
        cells.append(k[0]); k=par.get(k)
        if k is not None and k[1] is None and k[0]==ca:
            cells.append(ca); break
    cells.reverse()
    return _corners(cells),'우회 경로'

def _orthize(pts):
    """연속 점이 축 정렬이 아니면 L자 모서리를 끼워 직교 경로로 만든다.
    (마지막 점 -> 도착점 연결이 대각선이 되는 것 방지)"""
    out=[tuple(pts[0])]
    for p in pts[1:]:
        q=out[-1]
        if abs(p[0]-q[0])>1e-6 and abs(p[1]-q[1])>1e-6:
            out.append((p[0],q[1]))          # x 먼저 꺾고
        if abs(p[0]-out[-1][0])>1e-9 or abs(p[1]-out[-1][1])>1e-9:
            out.append(tuple(p))
    return out

def _resample2d(corners,step=0.01):
    """모서리 목록 -> step 간격 점 목록 (직교 구간별)."""
    out=[tuple(corners[0])]
    for a,b in zip(corners[:-1],corners[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        n=max(1,int(math.hypot(dx,dy)/step))
        for k in range(1,n+1):
            out.append((a[0]+dx*k/n,a[1]+dy*k/n))
    return out

def simple_cfg(a=None,b=None):
    """간편 이동 설정 (전역 + 구간별 덮어쓰기)."""
    g=S.setdefault('simple',{'prefer':'x','clear_cm':5.0,'extra_cm':5.0})
    cfg=dict(prefer=g.get('prefer','x'),clear_cm=float(g.get('clear_cm',5.0)),
             extra_cm=float(g.get('extra_cm',5.0)),user_path=None)
    if a and b:
        k=seg_key(a,b)
        sg=S.setdefault('simple_seg',{}).get(k)
        if sg and sg.get('prefer'): cfg['prefer']=sg['prefer']
        p=S.setdefault('paths2d',{}).get(k)
        if p: cfg['user_path']=[tuple(pt) for pt in p]
    return cfg

def plan_simple(qa,qb,prefer='x',clear_cm=5.0,extra_cm=5.0,user_path=None,
                a='',b='',ignore=()):
    """간편 이동 계획. -> ((way,dt), 라벨) / (None, 사유)
    장애물은 회피하지 않는다 - 겹치면 라벨에 ⚠ 경고만 붙이고 그대로 진행.
    ignore: 이 이동에서 무시할 장애물 id 들 (자세가 그 위로 올라가는 경우 등)."""
    qa=list(qa); qb=list(qb)
    j5,j6=qa[4],qa[5]
    pa,Ra=fk(qa); pb,Rb=fk(qb)
    if Ra[2,2]<0.999: return None,'출발 자세가 수평이 아닙니다'
    if Rb[2,2]<0.999: return None,'도착 자세가 수평이 아닙니다'
    seeds=_seed_list(qa)+[list(qb)]
    mz=lambda x,y: maxz_at(x,y,seed=qa,j5=j5,j6=j6)
    # ① 뽑기 높이
    PROG(f"[간편] {a}->{b} 뽑기 높이 계산")
    za=mz(pa[0],pa[1])
    if za is None: return None,'출발 지점 상승 불가'
    za=max(pa[2],za-0.005)
    # ② 2D 경로: 사용자가 그린 경로 or 직교(L자). 장애물은 회피하지 않음
    pa2,pb2=tuple(pa[:2]),tuple(pb[:2])
    def _caps_for(pts):
        """경로 각 점의 도달 상한. 하나라도 불가면 None."""
        caps=[]; prev=za
        for (x,y) in pts:
            c=maxz_at(x,y,seed=qa,j5=j5,j6=j6,hint=prev)
            if c is None: return None
            caps.append(c-0.006); prev=c
        return caps
    if user_path:
        corners=_orthize([pa2]+[tuple(p) for p in user_path]+[pb2])
        how='사용자 경로'; pts2=_resample2d(corners)
    else:
        # L자 후보 중 물리적으로 도달 가능한 쪽 선택 (선호축 먼저)
        cands=[('x먼저',[pa2,(pb2[0],pa2[1]),pb2]),
               ('y먼저',[pa2,(pa2[0],pb2[1]),pb2])]
        if prefer=='y': cands.reverse()
        pts2=None
        for hname,cn in cands:
            pts=_resample2d(cn)
            if all(_feas2d(x,y) for x,y in pts[1:-1]):
                how,pts2=hname,pts; break
        if pts2 is None:                     # 직선 불가 -> 도달 영역 안에서 우회
            PROG(f"[간편] {a}->{b} 직선 불가, 도달 가능 경로 탐색")
            cn,how=grid_path(pa2,pb2,0.0,prefer,obstacles=False)
            if cn is None: return None,f'도달 가능한 경로 없음 ({how})'
            how+='(도달범위 우회)'; pts2=_resample2d(cn)
    # 경로 전체에서 유지 가능한 높이 (이웃 힌트로 스캔 절약)
    PROG(f"[간편] {a}->{b} 경로 높이 계산 ({len(pts2)}점)")
    caps=_caps_for(pts2)
    if caps is None and not user_path:
        cn,how2=grid_path(pa2,pb2,0.0,prefer,obstacles=False)
        if cn is not None:
            pts2=_resample2d(cn); caps=_caps_for(pts2)
            how=how2+'(도달범위 우회)'
    if caps is None: return None,'경로상 도달 불가 지점이 있습니다'
    # 충돌 검사: 경고만 붙이고 그대로 진행 (무시 목록 제외)
    build_obs_grid(); ign=set(ignore or ())
    hitn=sorted(mid for mid,mk in _OBS['ids'].items()
                if mid not in ign and any(mk[_cell(x,y)] for x,y in pts2))
    if hitn: how+=' ⚠충돌:'+','.join(hitn)
    h=min(min(caps),za)
    zb_cap=mz(pb[0],pb[1])
    needed=min(pb[2]+extra_cm*0.01,(zb_cap or pb[2])-0.004)
    needed=max(needed,pb[2])
    # 이동높이가 하강 여유보다 낮으면: 여유 확보 가능한 지점부터 미리 상승
    heights=[h]*len(pts2); rise_at=None
    if h<needed:
        suf=[0.0]*len(caps); m=1e9
        for i in range(len(caps)-1,-1,-1):
            m=min(m,caps[i]); suf[i]=m
        for i,sm in enumerate(suf):
            if sm>=needed: rise_at=i; break
        if rise_at is None: return None,'하강 여유 높이를 확보할 수 없습니다'
        for i in range(rise_at,len(heights)): heights[i]=needed
    # ---- 웨이포인트 생성 (1cm 스텝, 직전 해 웜스타트) ----
    way=[list(qa)]; q=list(qa)
    lifted=[0]                # 높이 재시도 횟수 (로그용)
    def _go(x,y,z,retry=True):
        nonlocal q
        # 몸통 근처 등 일부 지점은 낮은 z 가 도달 불가 -> 필요한 만큼만 올려서 통과
        for dz in ((0,0.01,0.02,0.03,0.045,0.06) if retry else (0,)):
            sol=ik_level((x,y,z+dz),[q]+seeds,j5,j6)
            if sol is not None:
                if dz>0: lifted[0]+=1
                sol[4]=j5; sol[5]=j6
                way.append(sol); q=sol; return True
        return False
    def _vert(x,y,z0,z1):
        n=max(1,int(abs(z1-z0)/0.01))
        for k in range(1,n+1):
            if not _go(x,y,z0+(z1-z0)*k/n,retry=False): return False
        return True
    PROG(f"[간편] {a}->{b} ① 뽑기 {pa[2]:.2f}->{h:.2f}m")
    if not _vert(pa[0],pa[1],pa[2],min(h,za)): return None,'뽑기(상승) IK 실패'
    PROG(f"[간편] {a}->{b} ② 수평 이동 ({how})")
    for i,(x,y) in enumerate(pts2):
        if rise_at is not None and i==rise_at:
            if not _vert(x,y,h,needed): return None,'중간 상승 IK 실패'
        if not _go(x,y,heights[i]): return None,f'수평 이동 IK 실패 @({x:.2f},{y:.2f})'
    PROG(f"[간편] {a}->{b} ③ 하강 ->{pb[2]:.2f}m")
    if not _vert(pb[0],pb[1],heights[-1],pb[2]): return None,'하강 IK 실패'
    # 마지막: 목표 관절각으로 수렴 (j2+j3+j4 합이 같아 보간 중에도 수평 유지)
    if max(abs(x-y) for x,y in zip(q[:4],qb[:4]))>0.01:
        st=list(q)
        for k in range(1,11):
            t=k/10.0
            f=[st[i]+(qb[i]-st[i])*t for i in range(6)]
            f[4]=j5; f[5]=j6
            way.append(f)
    way[-1]=list(qb); way[-1][4]=j5; way[-1][5]=j6
    if lifted[0]: how+=f' · 저지대 {lifted[0]}점 상향'
    return (way,0.06),how

# ================= 터미널 로그 =================
LOG_ON=True
def LOG(msg):
    if LOG_ON: print(msg,flush=True)

# 진행 상황: 콘솔 + 웹 상태창(폴링)에 동시 표시
PROG_STATE={'text':'대기 중','t':0.0}
def PROG(msg):
    PROG_STATE['text']=msg
    LOG(f"  ... {msg}")
def BTN(name,detail=''):
    """버튼 클릭 로그. 무엇을 눌렀고 무엇을 하는지 남긴다."""
    LOG(f"\n>>> [버튼] {name}" + (f"  ({detail})" if detail else ""))
    PROG_STATE['text']=f"[{name}] {detail}" if detail else f"[{name}]" 
def LOG_TARGET(raws,tag=''):
    s='  '.join(f"j{J2ID.index(i)+1}:{v}" for i,v in sorted(raws.items()))
    LOG(f"  [목표] {tag} {s}")
def LOG_COMPARE(want,got,tag=''):
    """목표 raw 와 실제 raw 비교 출력."""
    parts=[]
    for i in sorted(want):
        w=want[i]; g=got.get(i)
        if g is None: parts.append(f"j{J2ID.index(i)+1}:{w}/--")
        else:
            d=g-w
            mark='' if abs(d)<=25 else ('!' if abs(d)<=100 else '!!')
            parts.append(f"j{J2ID.index(i)+1}:{w}->{g}({d:+d}){mark}")
    LOG(f"  [비교] {tag} "+'  '.join(parts))

# ================= 로봇 =================
IO_LOCK=threading.RLock()   # 시리얼 포트 동시 접근 방지 (녹화 스레드 vs 실행)

class Robot:
    """Feetech STS3215 (SO-ARM101)"""
    def __init__(self):
        self.port=None; self.packet=None
        self.last_want={}; self.last_got={}
    def connect(self):
        if self.port: return True
        try:
            import scservo_sdk.port_handler as _ph
            _ph.LATENCY_TIMER=16
            from scservo_sdk import PortHandler, PacketHandler
            p=PortHandler(PORT)
            if not p.openPort(): return False
            p.setBaudRate(BAUD)
            self.port=p; self.packet=PacketHandler(0); return True
        except Exception: return False
    def _w1(self,i,addr,v):
      with IO_LOCK:
        for _ in range(3):
            try:
                if self.packet.write1ByteTxRx(self.port,i,addr,int(v))[0]==0: return True
            except Exception: pass
            time.sleep(0.03)
        return False
    def _w2(self,i,addr,v):
      with IO_LOCK:
        for _ in range(3):
            try:
                if self.packet.write2ByteTxRx(self.port,i,addr,int(v))[0]==0: return True
            except Exception: pass
            time.sleep(0.03)
        return False
    def _r2(self,i,addr):
      with IO_LOCK:
        for _ in range(4):
            try:
                time.sleep(0.04)
                v,r,_=self.packet.read2ByteTxRx(self.port,i,addr)
                if r==0: return v
            except Exception: pass
            time.sleep(0.05)
        return None
    def time_profile(self,on,sp=1.0):
        """STS 는 시간기반 프로파일이 없으므로 속도/가속 설정으로 대체.
        sp: 속도 배수 (UI 선택). 모터 속도에도 반영해야 실제로 빨라짐."""
        for i in J2ID:
            v=int(min(3000,max(30,PER_ID_VEL.get(i,SAFE_VEL)*sp)))
            a=int(min(60,max(2,SAFE_ACCEL*sp)))
            self._w1(i,ADDR_MODE,0)
            self._w2(i,ADDR_TORQUE_LIMIT,PER_ID_TQ.get(i,TORQUE_LIMIT))
            self._w1(i,ADDR_P_ACCEL,a)
            self._w2(i,ADDR_P_VEL,v)
            if i!=GRIP_ID:                 # 그리퍼는 물체를 무는 용도라 제외
                self._w1(i,ADDR_KP,GAIN_KP)
                self._w1(i,ADDR_KD,GAIN_KD)
                self._w1(i,ADDR_KI,GAIN_KI)   # 중력으로 남는 편차를 없앰
    def torque(self,on):
        for i in J2ID: self._w1(i,ADDR_TORQUE,1 if on else 0)
    def send(self,q,log=False,tag='',with_grip=False):
        """관절 목표 전송.
        그리퍼(j6)는 기본적으로 '보내지 않는다' -> URDF/자세 값과 무관하게
        ◆열기/닫기 블록으로만 바뀌고 그 상태를 유지한다.
        with_grip=True 일 때만 j6 도 함께 보낸다(그리퍼 블록 전용)."""
        raws={}
        for jidx in range(6):
            if jidx==5 and not with_grip: continue   # j6 제외
            raw,i=rad_to_raw(jidx,q[jidx])
            self._w2(i,ADDR_GOAL,raw); raws[i]=raw
        if log: LOG_TARGET(raws,tag)
        return raws
    def read(self):
        q=[0.0]*6
        for jidx in range(6):
            i=J2ID[jidx]
            p=self._r2(i,ADDR_PRESENT)
            if p is not None: q[jidx]=DIR[i]*(p-ZERO_RAW[i])*math.pi/2048.0
        return q
    def _r2fast(self,i,addr):
        """빠른 1회 읽기 (추종 확인용)."""
        with IO_LOCK:
          try:
            v,r,_=self.packet.read2ByteTxRx(self.port,i,addr)
            return v if r==0 else None
          except Exception:
            return None
    def wait_reach(self,q,tol=25,timeout=6.0,ids=None):
        """실물이 목표 q 에 도달할 때까지 대기. (도달여부, 최대오차raw)
        움직이는 관절만 검사해 불필요한 통신을 줄임."""
        t0=time.time(); worst=0
        want={}
        for jidx in range(6):
            if jidx==5 and ids is None: continue   # j6 는 검사 제외 (그리퍼 블록 전용)
            raw,i=rad_to_raw(jidx,q[jidx])
            if ids is None or i in ids: want[i]=raw
        last_got={}
        while time.time()-t0<timeout:
            if STOP.is_set():
                self.last_want,self.last_got=want,last_got
                return False,worst
            worst=0; ok=True; last_got={}
            for i,tgt in want.items():
                p=self._r2fast(i,ADDR_PRESENT)
                if p is None: continue
                last_got[i]=p
                d=abs(p-tgt); worst=max(worst,d)
                if d>tol: ok=False
            if ok:
                self.last_want,self.last_got=want,last_got
                return True,worst
            time.sleep(0.02)
        self.last_want,self.last_got=want,last_got
        return False,worst
    def settle(self,q,tol=END_TOL,timeout=END_TIMEOUT,retry=4):
        """목표에 정밀하게 수렴시킴.
        같은 목표 재전송은 의미가 없다(이미 그 값) - 잔여 오차만큼 목표를
        반대 방향으로 살짝 밀어(과보정) 마찰/데드밴드를 뚫는다."""
        best=None
        for k in range(retry):
            ok,err=self.wait_reach(q,tol=tol,timeout=timeout/retry)
            best=err if best is None else min(best,err)
            if ok: return True,err
            if STOP.is_set(): return False,err
            # --- 오차 보정 (Ki 가 대부분 잡아주므로 소폭만) ---
            #  중력 반대쪽으로 목표를 살짝 밀어 마찰/데드밴드를 넘긴다
            for jidx in range(5):                      # j6 제외
                raw,i=rad_to_raw(jidx,q[jidx])
                p=self._r2(i,ADDR_PRESENT)
                if p is None: continue
                d=raw-p
                if abs(d)>tol:
                    push=raw+max(-15,min(15,d))        # 최대 ±15 (약 1.3도)
                    lo,hi=LIMIT_RAW[i]
                    self._w2(i,ADDR_GOAL,max(lo,min(hi,push)))
                    LOG(f"  [보정] j{jidx+1}: 오차{d:+d} -> 목표 {raw}->{max(lo,min(hi,push))}")
            time.sleep(END_SETTLE)
        return False,(best if best is not None else 9999)
    def gripper(self,open_,tq=None):
        i=GRIP_ID
        self._w1(i,ADDR_MODE,0)
        if open_:
            # 물체를 조인 채 이동하면 과부하 보호가 걸릴 수 있다
            # -> 토크를 껐다 켜서 보호 해제 후 연다
            self._w1(i,ADDR_TORQUE,0); time.sleep(0.05)
        self._w2(i,ADDR_TORQUE_LIMIT,tq or PER_ID_TQ.get(i,TORQUE_LIMIT))
        self._w1(i,ADDR_P_ACCEL,10)
        self._w2(i,ADDR_P_VEL,PER_ID_VEL.get(i,SAFE_VEL))
        self._w1(i,ADDR_TORQUE,1)
        self._w2(i,ADDR_GOAL,GRIPPER_OPEN if open_ else GRIPPER_CLOSE)
    def estop(self):
        try: self.torque(False)
        except Exception: pass
ROBOT=Robot()
STOP=threading.Event()
RUNNING=threading.Event()

# ================= 상태 =================
S={'poses':[{'name':'base','angles':[0.0]*6}], 'seq':['base']}
LOCK=threading.Lock()
def pose_names(): return [p['name'] for p in S['poses']]
def get_angles(name):
    for p in S['poses']:
        if p['name']==name: return list(p['angles'])
    return None
def get_offset(name):
    for p in S['poses']:
        if p['name']==name: return list(p.get('offset',[0.0]*6))
    return [0.0]*6
def set_offset(name,off):
    for p in S['poses']:
        if p['name']==name: p['offset']=list(off); return
def seg_key(a,b): return f"{a}=>{b}"
def get_moves(): return S.setdefault('moves',{})     # {"A=>B": [frames...]}
def get_axorders(): return S.setdefault('axorders',{})  # {"A=>B": "x-z-y"}
def seg_axorder(a,b,default):
    """구간별 축 순서. 지정 없으면 기본값 사용."""
    return get_axorders().get(seg_key(a,b),default)

# 손목(j5) 회전 타이밍
#   blend  : 이동하며 함께 회전 (기본)
#   before : 회전을 먼저 끝내고 이동
#   after  : 이동을 먼저 하고 회전
#   lock   : j5 를 출발 각도로 고정 (목표 자세의 j5 는 무시)
ROT_MODES={'동시(기본)':'blend','회전 먼저':'before','이동 먼저':'after',
           'j5 고정(회전 안함)':'lock'}
ROT_LABEL={v:k for k,v in ROT_MODES.items()}
def get_modes(): return S.setdefault('modes',{})       # {"A=>B": 이동방식이름}
def seg_mode(a,b,default):
    return get_modes().get(seg_key(a,b),default)

def get_rotmodes(): return S.setdefault('rotmodes',{})   # {"A=>B": "before"}
def seg_rotmode(a,b,default='blend'):
    return get_rotmodes().get(seg_key(a,b),default)

# 시퀀스 항목: 자세는 문자열("base"), 그리퍼는 "@open" / "@close"
GRIP_OPEN_ITEM='@open'; GRIP_CLOSE_ITEM='@close'
def is_grip_item(x): return isinstance(x,str) and x.startswith('@')
def pose_seq():
    """시퀀스에서 자세 항목만 (경로 계획용)."""
    return [x for x in S['seq'] if not is_grip_item(x)]

def seq_labels(mode_name=None):
    """파이프라인 블록: ■자세 / ┃이동 / ◆그리퍼"""
    out=[]; mv=get_moves()
    prev_pose=None; seg_i=0
    for idx,n in enumerate(S['seq']):
        if is_grip_item(n):
            act='열기 ✋' if n==GRIP_OPEN_ITEM else '닫기 ✊'
            out.append(f"◆ {idx}. 그리퍼 {act}")
            continue
        if prev_pose is not None:
            k=seg_key(prev_pose,n)
            rec=mv.get(k)
            mm=get_modes().get(k)          # 블록별 이동방식 지정 (없으면 간편)
            if rec:
                tag=f"● 녹화 {len(rec)}프"   # 녹화가 있으면 그 경로가 우선 실행됨
            elif mm and mm!=MODE_SIMPLE:
                # 복잡 이동을 명시한 블록만 세부 설정 표시
                tag=mm.split(' (')[0]
                ax=get_axorders().get(k)
                if ax: tag+=f" · 축 {ax}"
                rm=get_rotmodes().get(k)
                if rm and rm!='blend': tag+=f" · {ROT_LABEL[rm]}"
            else:
                # 기본 = 간편 이동 (뽑기->수평->하강)
                tag='간편'
                if S.get('paths2d',{}).get(k): tag+=' · 🗺그린 경로'
                else: tag+=' · 자동 경로'
            out.append(f"┃ {seg_i}↓ 이동 · {tag}")
            seg_i+=1
        out.append(f"■ {idx}. {n}")
        prev_pose=n
    return out
def seq_index(label):
    """블록 라벨 -> (종류,인덱스). 종류: 'pose'(자세) 또는 'move'(이동)"""
    if label is None: return None,None
    s=str(label)
    if s.lstrip().startswith('┃'):          # 이동 블록: "┃ 0↓ 이동 · ..."
        try:
            return 'move',int(s.split('┃')[1].split('↓')[0].strip())
        except Exception:
            return None,None
    if s.lstrip().startswith('■'):          # 자세 블록 (S['seq'] 인덱스)
        try: return 'pose',int(s.split('.')[0].replace('■','').strip())
        except Exception: return None,None
    if s.lstrip().startswith('◆'):          # 그리퍼 블록 (S['seq'] 인덱스)
        try: return 'grip',int(s.split('.')[0].replace('◆','').strip())
        except Exception: return None,None
    return None,None

def move_seg_poses(seg_i):
    """이동 블록 인덱스 -> (앞 자세, 뒤 자세). 없으면 (None,None)"""
    ps=pose_seq()
    if 0<=seg_i<len(ps)-1: return ps[seg_i],ps[seg_i+1]
    return None,None

CURRENT_Q=[0.0]*6   # 웹 뷰어가 폴링하는 현재 자세
GRIP_STATE={'rad':0.0}          # 3D 뷰용 그리퍼 상태 (◆블록으로만 변경)
def ros_publish(q,with_grip=False):
    """3D 뷰 갱신. 그리퍼(j6)는 자세 값을 무시하고 현재 상태를 유지한다."""
    global CURRENT_Q
    q=list(q)
    if with_grip: GRIP_STATE['rad']=q[5]
    else:         q[5]=GRIP_STATE['rad']
    CURRENT_Q=q
    if _pub is None: return
    m=JointState(); m.header.stamp=_ros.get_clock().now().to_msg()
    m.name=list(JOINTS); m.position=[float(x) for x in q]; _pub.publish(m)

# ================= 3D 뷰 (RViz처럼 STL 메쉬 렌더링) =================
MESH_DIR=os.path.join(os.path.dirname(BASE_URDF),'..','meshes')
ROBOT_LINKS=['base_link','l1','l2','l3','l4','l5','l6']
ROBOT_COLORS=['#8a8a8a','#5b8dbf','#6a9bcf','#5b8dbf','#6a9bcf','#5b8dbf','#e0a13a']
ENV_MESHES=[]        # 환경은 /home/drboom/2026_project/stl 자동 스캔으로 채움

def _load_dec(path,reduction=0.85,scale=1.0):
    m=trimesh.load(path)
    v=np.asarray(m.vertices,dtype=np.float32)*scale
    f=np.asarray(m.faces,dtype=np.int64)
    if len(f)>500:
        v,f=fast_simplification.simplify(v,f,target_reduction=reduction)
    return np.asarray(v,dtype=np.float32),np.asarray(f,dtype=np.int32)

# 로봇 링크 메쉬 (m 단위, 링크 프레임)
_LINK_MESH=[]
for ln in ROBOT_LINKS:
    _LINK_MESH.append(_load_dec(os.path.join(MESH_DIR,ln+'.STL')))
# 환경 메쉬 (mm -> m, base_link 프레임 고정) - trace 미리 생성해 재사용
_ENV_TRACES=[]
_env_pts=[]
for fn,col in ENV_MESHES:
    p=os.path.join(MESH_DIR,fn)
    if not os.path.exists(p): continue
    v,f=_load_dec(p,scale=0.001)
    _env_pts.append(v)
    _ENV_TRACES.append(go.Mesh3d(x=v[:,0],y=v[:,1],z=v[:,2],
        i=f[:,0],j=f[:,1],k=f[:,2],color=col,opacity=1.0,flatshading=True,
        lighting=dict(ambient=0.55,diffuse=0.8,specular=0.15),
        hoverinfo='skip',showscale=False))
# 뷰 범위: 환경+로봇 작업영역 포함
if _env_pts:
    allp=np.vstack(_env_pts)
    lo=np.minimum(allp.min(0),[-0.35,-0.05,0.0])-0.03
    hi=np.maximum(allp.max(0),[0.35,0.55,0.55])+0.03
else:
    lo=np.array([-0.4,-0.1,0.0]); hi=np.array([0.4,0.6,0.6])
_span=float(max(hi-lo))
_mid=(hi+lo)/2
_RANGES=[[_mid[k]-_span/2,_mid[k]+_span/2] for k in range(3)]
_RANGES[2][0]=min(0.0,lo[2])   # 바닥은 0부터

def link_frames(q):
    return CHAIN.forward_kinematics([0]+list(q),full_kinematics=True)

def robot_fig(q):
    fig=go.Figure()
    for t in _ENV_TRACES: fig.add_trace(t)
    frames=link_frames(q)
    # frames[0]=base_link(원점), frames[k]=조인트k 이후 = l_k 프레임
    for k,(v,f) in enumerate(_LINK_MESH):
        T=np.eye(4) if k==0 else frames[k]
        R=T[:3,:3]; p=T[:3,3]
        w=v@R.T+p
        fig.add_trace(go.Mesh3d(x=w[:,0],y=w[:,1],z=w[:,2],
            i=f[:,0],j=f[:,1],k=f[:,2],color=ROBOT_COLORS[k],
            flatshading=True,
            lighting=dict(ambient=0.45,diffuse=0.9,specular=0.3),
            hoverinfo='skip',showscale=False))
    p_ee,_=fk(q)
    fig.add_trace(go.Scatter3d(x=[p_ee[0]],y=[p_ee[1]],z=[p_ee[2]],mode='markers',
        marker=dict(size=6,color='#dd2222',symbol='diamond'),name='EE'))
    fig.update_layout(
        scene=dict(xaxis=dict(range=_RANGES[0],title='x',showbackground=False),
                   yaxis=dict(range=_RANGES[1],title='y',showbackground=False),
                   zaxis=dict(range=_RANGES[2],title='z',showbackground=True,
                              backgroundcolor='#f0f0f0'),
                   aspectmode='cube',
                   camera=dict(eye=dict(x=1.3,y=-1.5,z=0.9))),
        margin=dict(l=0,r=0,t=28,b=0),height=460,showlegend=False,
        paper_bgcolor='#ffffff',
        title=f"EE: x={p_ee[0]*1000:.1f} y={p_ee[1]*1000:.1f} z={p_ee[2]*1000:.1f} mm",
        uirevision='keep')   # 마우스 회전 시점 유지
    return fig

# ================= 핸들러 =================
def h_select(name):
    q=get_angles(name) or [0.0]*6
    ros_publish(q)
    # 슬라이더는 j1~j5 만
    return [robot_fig(q)]+[round(math.degrees(v),1) for v in q[:5]]

def h_slider(name,*degs):
    """j1~j5 슬라이더. j6 는 기존 값 유지 (그리퍼 버튼으로만 변경)."""
    if name is None: return gr.update(), "먼저 자세를 선택하세요"
    cur=get_angles(name) or [0.0]*6
    q=[math.radians(d) for d in degs]+[cur[5]]
    with LOCK:
        for p in S['poses']:
            if p['name']==name: p['angles']=q
    _clear_plan_cache()
    ros_publish(q)
    return robot_fig(q), f"'{name}' 갱신"

# ---- 그리퍼 (j6): 각도 대신 열림/닫힘 ----
def _j6_rad(open_):
    i=GRIP_ID
    raw=GRIPPER_OPEN if open_ else GRIPPER_CLOSE
    return DIR[i]*(raw-ZERO_RAW[i])*math.pi/2048.0

def h_grip(name,open_):
    """3D 뷰 갱신 + 자세에 j6 반영 + (연결돼 있으면) 실물 그리퍼 동작."""
    lbl='열기' if open_ else '닫기'
    rad=_j6_rad(open_)
    if name:
        with LOCK:
            for p in S['poses']:
                if p['name']==name:
                    a=list(p['angles']); a[5]=rad; p['angles']=a
        q=get_angles(name)
    else:
        q=list(CURRENT_Q); q[5]=rad
    ros_publish(q,with_grip=True)
    msg=f"그리퍼 {lbl}"
    if ROBOT.connect():                     # 아직 안 열려 있으면 여기서 연결
        try:
            ROBOT.gripper(open_)
            raw=GRIPPER_OPEN if open_ else GRIPPER_CLOSE
            time.sleep(0.3)
            p=ROBOT._r2(GRIP_ID,ADDR_PRESENT)
            msg+=f" -> 실물 목표 {raw} (현재 {p})"
        except Exception as e:
            msg+=f" (실물 실패: {e})"
    else:
        msg+=f" (3D만 - {PORT} 연결 실패)"
    return robot_fig(q), msg

def _add_urdf_pose(urdf_path,l6_path=None):
    """URDF 경로 -> 자세 역산해 추가. (추가된이름 or None, 메시지)"""
    try: angles,warn=solve_pose_from_urdf(urdf_path,l6_path)
    except Exception as e:
        return None,f"FAIL {os.path.basename(urdf_path)}: {e}"
    # 관절 한계로 클램프 (IK bounds 오류/실물 한계 초과 방지)
    clamped=[]
    for j,a in enumerate(angles):
        lo,hi=jr_limits(j)
        c=max(lo,min(hi,a))
        if abs(c-a)>0.002: clamped.append(f"j{j+1} {math.degrees(a):+.1f}->{math.degrees(c):+.1f}deg")
        angles[j]=c
    if clamped:
        warn=(warn+" | " if warn else "")+"LIMIT clamp: "+", ".join(clamped)
    name=os.path.basename(urdf_path).replace('.urdf','')
    for tag in ['Industry_gyu_arm_20260724_','Industry_gyu_arm_']:
        name=name.replace(tag,'')
    base=name; k=2
    while name in pose_names(): name=f'{base}_{k}'; k+=1
    # 기존에 보정된 자세가 있으면 가장 가까운 것의 보정값을 자동 적용
    off,src,dist=nearest_offset(angles)
    with LOCK:
        S['poses'].append({'name':name,'angles':angles,
                           'offset':off if off else [0.0]*6})
    deg=', '.join(f'{math.degrees(a):+.1f}' for a in angles)
    msg=f"OK {name}: [{deg}]"+(f" WARN:{warn}" if warn else "")
    if off: msg+=f"\n  base 보정 자동 적용 (관절각 차이 {dist:.1f}도)"
    return name,msg

def h_upload_dir(files):
    """URDF 패키지 디렉토리 통째 드롭: 안에서 .urdf와 l6.STL을 찾아 처리."""
    if not files: return gr.update(),gr.update(),"no folder"
    paths=[f if isinstance(f,str) else f.name for f in files]
    urdfs=[p for p in paths if p.lower().endswith('.urdf')]
    l6=next((p for p in paths if os.path.basename(p).lower()=='l6.stl'),None)
    if not urdfs:
        return gr.update(),gr.update(),"no .urdf found in folder"
    msgs=[]; added=None
    for u in urdfs:
        # _limits.urdf 같은 변형은 건너뛰고 원본 우선
        if u.lower().endswith('_limits.urdf') and len(urdfs)>1: continue
        name,msg=_add_urdf_pose(u,l6)
        msgs.append(msg)
        if name: added=name
    upd=(gr.update(choices=pose_names(),value=added) if added
         else gr.update(choices=pose_names()))
    return upd, gr.update(choices=pose_names()), "\n".join(msgs)

def h_add_path(pathtext):
    """로컬 경로 직접 입력: 디렉토리(패키지) 또는 .urdf 파일."""
    p=(pathtext or '').strip().rstrip('/')
    if not p: return gr.update(),gr.update(),"enter a path"
    if not os.path.exists(p): return gr.update(),gr.update(),f"not found: {p}"
    if os.path.isdir(p):
        import glob as _g
        urdfs=[u for u in _g.glob(os.path.join(p,'**','*.urdf'),recursive=True)
               if not u.lower().endswith('_limits.urdf')] \
              or _g.glob(os.path.join(p,'**','*.urdf'),recursive=True)
        if not urdfs: return gr.update(),gr.update(),f"no .urdf in {p}"
        urdf=urdfs[0]
        l6c=_g.glob(os.path.join(p,'**','l6.STL'),recursive=True)
        l6=l6c[0] if l6c else None
    else:
        urdf=p; l6=None
    name,msg=_add_urdf_pose(urdf,l6)
    upd=(gr.update(choices=pose_names(),value=name) if name
         else gr.update(choices=pose_names()))
    return upd, gr.update(choices=pose_names()), msg

def h_plate_edit(on):
    PLATE_EDIT['on']=bool(on)
    if on:
        g=ghost_tiles()
        return (f"바닥 확장 모드 ON — 3D 뷰의 반투명 초록 타일을 클릭하면 그 자리에 "
                f"플레이트가 추가됩니다. 현재 플레이트 {len(PLATES)}장, 확장 가능 자리 {len(g)}곳.")
    return f"바닥 확장 모드 OFF (플레이트 {len(PLATES)}장)"
def h_plate_list():
    rows=[]
    for t in plate_tiles():
        info=PLATES[t]
        rows.append(f"{tile_label(t)} {'[추가됨]' if info.get('added') else '[원본]'}")
    return "플레이트 "+str(len(PLATES))+"장: "+", ".join(rows)
def added_tile_labels():
    return [tile_label(t) for t in plate_tiles() if PLATES[t].get('added')]
def h_plate_remove(lbl):
    if not lbl: return "삭제할 플레이트를 선택하세요", gr.update()
    t=tuple(int(v) for v in lbl.strip('()').split(','))
    ok=remove_plate(t)
    msg=f"{lbl} 삭제됨" if ok else f"{lbl} 삭제 불가 (원본 플레이트는 삭제할 수 없습니다)"
    return msg, gr.update(choices=added_tile_labels(),value=None)
def h_plate_refresh():
    return h_plate_list(), gr.update(choices=added_tile_labels(),value=None)

def env_ids(): return [e['id'] for e in ENV_SHOW]
def env_visible_ids(): return [e['id'] for e in ENV_SHOW if e.get('visible',True)]
def h_env(selected):
    selected=selected or []
    with LOCK:
        for e in ENV_SHOW: e['visible']=(e['id'] in selected)
    return "STL visible: "+(", ".join(selected) if selected else "(none)")

def h_upload(files):
    if not files: return gr.update(),gr.update(),gr.update(),"no file"
    msgs=[]
    added=None
    for f in (files if isinstance(files,list) else [files]):
        path=f if isinstance(f,str) else f.name
        low=path.lower()
        if low.endswith('.stl'):
            # STL -> 환경 오브젝트로 뷰어에 추가 (mm/m 자동 감지)
            try:
                mm=trimesh.load(path)
                span=float(max(mm.bounds[1]-mm.bounds[0]))
                scale=0.001 if span>10 else 1.0     # 10 넘으면 mm 단위로 판단
                mid=os.path.basename(path).replace('.STL','').replace('.stl','')
                base=mid; k=2
                while mid in MESH_REG: mid=f'{base}_{k}'; k+=1
                MESH_REG[mid]=(path,scale)
                ENV_SHOW.append({'id':mid,'color':'#b8865b','visible':True})
                msgs.append(f"OK env '{mid}' (scale {scale})")
            except Exception as e:
                msgs.append(f"FAIL {os.path.basename(path)}: {e}")
            continue
        if not low.endswith('.urdf'):
            msgs.append(f"skip {os.path.basename(path)}"); continue
        name,msg=_add_urdf_pose(path)
        msgs.append(msg)
        if name: added=name
    upd=(gr.update(choices=pose_names(),value=added) if added
         else gr.update(choices=pose_names()))
    return upd, gr.update(choices=pose_names()), \
           gr.update(choices=env_ids(),value=env_visible_ids()), "\n".join(msgs)

def h_duplicate(name,newname):
    if not name: return gr.update(),gr.update(),"select a pose"
    newname=(newname or '').strip() or name+'_copy'
    base=newname; k=2
    while newname in pose_names(): newname=f'{base}_{k}'; k+=1
    with LOCK: S['poses'].append({'name':newname,'angles':get_angles(name),
                                  'offset':get_offset(name)})
    return gr.update(choices=pose_names(),value=newname), gr.update(choices=pose_names()), \
           f"duplicated '{name}' -> '{newname}' (adjust j6 for gripper)"

def h_delete(name):
    if not name or name=='base':
        return gr.update(),gr.update(),gr.update(),"cannot delete base"
    with LOCK:
        S['poses']=[p for p in S['poses'] if p['name']!=name]
        S['seq']=[s for s in S['seq'] if s!=name]
    return gr.update(choices=pose_names(),value='base'), gr.update(choices=pose_names()), \
           gr.update(choices=seq_labels(),value=None), f"deleted '{name}'"

def h_seq_add(name):
    if not name: return gr.update(),"자세를 선택하세요"
    with LOCK: S['seq'].append(name)
    return gr.update(choices=seq_labels(),value=None), f"시퀀스: {' -> '.join(S['seq'])}"

def h_seq_grip(open_):
    """파이프라인에 그리퍼 블록 추가."""
    with LOCK: S['seq'].append(GRIP_OPEN_ITEM if open_ else GRIP_CLOSE_ITEM)
    return gr.update(choices=seq_labels(),value=None), \
           f"그리퍼 {'열기' if open_ else '닫기'} 블록 추가"

def h_seg_axorder(sel,ax):
    """선택한 이동 블록에 축 순서 지정."""
    BTN('구간 축순서 적용',f'{sel} -> {ax}')
    kind,i=seq_index(sel)
    if kind!='move':
        return gr.update(),"┃ 로 시작하는 [이동] 블록을 선택하세요"
    a,b=move_seg_poses(i)
    if a is None: return gr.update(choices=seq_labels(),value=None),"구간을 찾을 수 없습니다"
    k=seg_key(a,b)
    if ax=='(기본값 사용)':
        get_axorders().pop(k,None); msg=f"[{a} → {b}] 축 순서 지정 해제"
    else:
        get_axorders()[k]=ax; msg=f"[{a} → {b}] 축 순서 = {ax}"
    _clear_plan_cache()
    try:
        with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    except Exception: pass
    return gr.update(choices=seq_labels(),value=None), msg

def h_seg_rotmode(sel,label):
    """선택한 이동 블록에 손목(j5) 회전 타이밍 지정."""
    BTN('구간 회전타이밍 적용',f'{sel} -> {label}')
    kind,i=seq_index(sel)
    if kind!='move':
        return gr.update(),"┃ 로 시작하는 [이동] 블록을 선택하세요"
    a,b=move_seg_poses(i)
    if a is None:
        return gr.update(choices=seq_labels(),value=None),"구간을 찾을 수 없습니다"
    mode=ROT_MODES.get(label,'blend')
    k=seg_key(a,b)
    if mode=='blend': get_rotmodes().pop(k,None)
    else:             get_rotmodes()[k]=mode
    _clear_plan_cache()
    try:
        with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    except Exception: pass
    return gr.update(choices=seq_labels(),value=None), \
           f"[{a} → {b}] 손목 회전 = {label}"

def h_seg_mode(sel,mname):
    """선택한 이동 블록에 이동 방식 지정. '(기본값 사용)' 이면 전역값을 따른다."""
    BTN('구간 이동방식 적용',f'{sel} -> {mname}')
    kind,i=seq_index(sel)
    if kind!='move':
        return gr.update(),"┃ 로 시작하는 [이동] 블록을 선택하세요"
    a,b=move_seg_poses(i)
    if a is None:
        return gr.update(choices=seq_labels(),value=None),"구간을 찾을 수 없습니다"
    k=seg_key(a,b)
    if mname=='(기본값 사용)':
        get_modes().pop(k,None); msg=f"[{a} → {b}] 이동 방식 지정 해제"
    else:
        get_modes()[k]=mname; msg=f"[{a} → {b}] 이동 방식 = {mname}"
    _clear_plan_cache()
    try:
        with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    except Exception: pass
    return gr.update(choices=seq_labels(),value=None), msg

def h_seq_del(sel):
    kind,i=seq_index(sel)
    if kind is None: return gr.update(),"항목을 선택하세요"
    if kind=='move':
        a,b=move_seg_poses(i)
        if a is None:
            return gr.update(choices=seq_labels(),value=None),\
                   "이동 구간을 찾을 수 없습니다 (목록 갱신됨)"
        get_moves().pop(seg_key(a,b),None)
        return gr.update(choices=seq_labels(),value=None), \
               f"[이동] {a} -> {b} 녹화 삭제 (자동 경로로 복귀)"
    if not (0<=i<len(S['seq'])):        # 목록이 바뀌어 인덱스가 낡은 경우
        return gr.update(choices=seq_labels(),value=None),\
               "목록이 변경되었습니다. 다시 선택하세요."
    with LOCK: del S['seq'][i]          # 자세 또는 그리퍼 블록 삭제
    return gr.update(choices=seq_labels(),value=None), \
           f"시퀀스: {' -> '.join(S['seq'])}"

def h_seq_move(sel,d):
    kind,i=seq_index(sel)
    if kind not in ('pose','grip'):
        return gr.update(),"자세 또는 그리퍼 블록을 선택하세요"
    if not (0<=i<len(S['seq'])):
        return gr.update(choices=seq_labels(),value=None),\
               "목록이 변경되었습니다. 다시 선택하세요."
    j=i+d
    if not (0<=j<len(S['seq'])): return gr.update(),"더 이동할 수 없습니다"
    with LOCK: S['seq'][i],S['seq'][j]=S['seq'][j],S['seq'][i]
    return gr.update(choices=seq_labels(),value=None), \
           f"시퀀스: {' -> '.join(S['seq'])}"

# ===== 충돌 검사 (계획 경로 vs 환경 STL, fcl) =====
from trimesh.collision import CollisionManager
import trimesh as _tm
_COLL_LINKS=None
_COLL_ENV={}
COLL_EXCLUDE={'base_link','l1'}   # 바닥판 위에 서 있는 부분은 제외
def _coll_robot_mgr():
    global _COLL_LINKS
    if _COLL_LINKS is None:
        _COLL_LINKS=[]
        for k,ln in enumerate(ROBOT_LINKS):
            v,f=_LINK_MESH[k]
            _COLL_LINKS.append((ln,_tm.Trimesh(vertices=v,faces=f,process=False)))
    m=CollisionManager()
    for ln,mesh in _COLL_LINKS:
        if ln in COLL_EXCLUDE: continue
        m.add_object(ln,mesh)
    return m
def _coll_env_mgr():
    m=CollisionManager(); n=0
    for e in ENV_SHOW:
        if not e.get('visible',True): continue   # 숨긴 STL은 충돌검사도 제외
        mid=e['id']
        if mid not in _COLL_ENV:
            path,scale=MESH_REG[mid]
            v,f=_load_dec(path,scale=scale)
            _COLL_ENV[mid]=_tm.Trimesh(vertices=v,faces=f,process=False)
        m.add_object(mid,_COLL_ENV[mid]); n+=1
    return m if n else None
def check_collision(way,step=2):
    """경로 웨이포인트들에서 로봇(l2~l6) vs 환경 STL 충돌 검사. [(링크,STL)] 반환."""
    env=_coll_env_mgr()
    if env is None: return []
    rob=_coll_robot_mgr()
    hits=set()
    for idx in list(range(0,len(way),step))+[len(way)-1]:
        frames=link_frames(way[idx])
        for k,ln in enumerate(ROBOT_LINKS):
            if ln in COLL_EXCLUDE: continue
            T=np.eye(4) if k==0 else frames[k]
            rob.set_transform(ln,T)
        try:
            ok,names=rob.in_collision_other(env,return_names=True)
        except Exception:
            return []
        if ok:
            for a,b in names: hits.add((a,b))
    return sorted(hits)

_PLAN_CACHE={}          # (구간키, 설정) -> 계획 결과. 같은 조건이면 재계산 안 함
def _clear_plan_cache(): _PLAN_CACHE.clear()

def plan_all(mode,axorder,check_coll=False):
    ps=pose_seq()                                   # 그리퍼 블록 제외
    if len(ps)<2: return None,"시퀀스에 자세가 2개 이상 필요합니다",""
    segs=[]
    mv=get_moves()
    carry=None          # 앞 구간의 '실제 끝 자세' (j5 고정 등이 반영된 값)
    mode0=mode; si=1; ntot=max(1,len(ps)-1)
    for a,b in zip(ps[:-1],ps[1:]):
        qa=get_angles(a); qb=get_angles(b)
        if carry is not None:
            qa=list(carry)          # 이전 구간에서 실제로 끝난 자세에서 출발
        used=""
        mode=seg_mode(a,b,mode0)          # 구간별 이동 방식 (없으면 전역값)
        PROG(f"[경로 계산 {si}/{ntot}] {a} -> {b} · {mode}")
        si+=1
        # 이 구간에 녹화가 있으면 그 경로를 그대로 사용
        rec=mv.get(seg_key(a,b))
        if rec:
            way=[list(f) for f in rec]
            way[0]=list(qa); way[-1]=list(qb)
            # 그리퍼(j6)는 이동/도착 전 구간 고정 -> ◆그리퍼 블록에서만 변경
            for f in way: f[5]=qa[5]
            if LEVEL_ALWAYS: lock_j5(way,qa)     # j5 고정 (물컵 규칙)
            segs.append((f'{a} -> {b} [녹화 {len(way)}프레임]',way,0.06,
                         get_offset(a),get_offset(b)))
            carry=list(way[-1])          # 실제 끝 자세를 다음 구간 출발점으로
            continue
        # --- 캐시 조회 (EE 모드는 계산이 비싸므로) ---
        ck=None
        if mode in (MODES[2],MODES[3],MODES[4],MODE_SIMPLE):
            xtra=()
            if mode==MODE_SIMPLE:
                c=simple_cfg(a,b)
                ig=S.setdefault('ignore_obs',{})
                xtra=(c['prefer'],c['clear_cm'],c['extra_cm'],
                      tuple(map(tuple,c['user_path'] or [])),
                      tuple(sorted(set(ig.get(a,[]))|set(ig.get(b,[])))))
            ck=(mode,seg_axorder(a,b,axorder),seg_rotmode(a,b),
                tuple(round(v,6) for v in qa),tuple(round(v,6) for v in qb),
                bool(check_coll))+xtra
            hit=_PLAN_CACHE.get(ck)
            if hit is not None:
                r,used=hit
                if r is None and mode==MODE_SIMPLE:
                    return None,f"간편 이동 실패: {a} -> {b} · {used}",""
                if r is not None:
                    way=[list(f) for f in r[0]]
                    way[-1]=list(qb)
                    for f in way: f[5]=qa[5]
                    if LEVEL_ALWAYS or seg_rotmode(a,b)=='lock': lock_j5(way,qa)
                    tag=f" [{used}]" if used else ""
                    segs.append((f'{a} -> {b}{tag}',way,r[1],
                                 get_offset(a),get_offset(b)))
                    carry=list(way[-1]); continue
        if mode==MODE_SIMPLE:
            c=simple_cfg(a,b)
            ig=S.setdefault('ignore_obs',{})
            r,used=plan_simple(qa,qb,a=a,b=b,
                               ignore=set(ig.get(a,[]))|set(ig.get(b,[])),**c)
            if r is None:
                if ck is not None: _PLAN_CACHE[ck]=(None,used)
                return None,f"간편 이동 실패: {a} -> {b} · {used}",""
        elif mode==MODES[0]: r=plan_joint_seq(qa,qb)
        elif mode==MODES[1]: r=plan_joint_all(qa,qb)
        elif mode==MODES[2]:
            myrot=seg_rotmode(a,b)
            r=None
            for dens in (1,2,4):                  # 실패하면 더 촘촘히 재시도
                r=plan_ee_line(qa,qb,dens,myrot)
                if r is not None:
                    used=(f"조밀x{dens}" if dens>1 else "")
                    if myrot!='blend':
                        used=(used+" " if used else "")+ROT_LABEL[myrot]
                    break
        elif mode==MODES[4]:              # 그리퍼 수직 유지
            myax=seg_axorder(a,b,axorder)
            r=None
            for dens in (1,2,4):
                r=plan_ee_level(qa,qb,AXIS_ORDERS[myax],dens)
                if r is not None:
                    used=myax+"·수직유지"+(f" 조밀x{dens}" if dens>1 else ""); break
            if r is None:                 # 다른 축 순서로 재시도
                for on,o in AXIS_ORDERS.items():
                    if on==myax: continue
                    for dens in (1,2,4):
                        r=plan_ee_level(qa,qb,o,dens)
                        if r is not None: break
                    if r is not None:
                        used=on+"·수직유지(자동)"; break
        else:
            # 구간별 축 순서(있으면) 우선. 충돌/IK 실패면 다른 순서를 자동 탐색
            myax=seg_axorder(a,b,axorder); myrot=seg_rotmode(a,b)
            rtag=("·"+ROT_LABEL[myrot]) if myrot!='blend' else ""
            r=None
            for dens in (1,2,4):
                r=plan_ee_axis(qa,qb,AXIS_ORDERS[myax],dens,myrot)
                if r is not None:
                    used=myax+rtag+(f" 조밀x{dens}" if dens>1 else ""); break
            if r is not None and check_coll and check_collision(r[0]):
                r=None                      # 충돌 -> 다른 순서 시도
                used=''
            if r is None:
                fallback=None; fb_name=''
                for on,o in AXIS_ORDERS.items():
                    if on==myax: continue
                    cand=None
                    for dens in (1,2,4):
                        PROG(f"[경로 계산 {si-1}/{ntot}] {a} -> {b} · "
                             f"축순서 {on} 시도 · 밀도 {dens}배")
                        cand=plan_ee_axis(qa,qb,o,dens,myrot)
                        if cand is not None: break
                    if cand is None: continue
                    if check_coll and check_collision(cand[0]):
                        if fallback is None:      # 충돌하지만 IK 는 되는 후보 보관
                            fallback,fb_name=cand,on+"(충돌)"
                        continue
                    r=cand; used=on+rtag+"(자동)"; break
                if r is None and fallback is not None:
                    r,used=fallback,fb_name     # 전부 충돌이면 그나마 가능한 것
        if ck is not None: _PLAN_CACHE[ck]=(r,used)     # 실패도 캐시 (재시도 낭비 방지)
        if r is None: return None,f"IK 경로 실패: {a} -> {b} (관절 모드로 시도해보세요)",""
        tag=f" [{used}]" if used else ""
        # 그리퍼(j6)는 이동 중 변하지 않게 고정 -> 목적지에서만 열고 닫음
        way=[list(f) for f in r[0]]
        way[-1]=list(qb)
        # 그리퍼(j6)는 전 구간 고정 -> ◆그리퍼 블록에서만 변경
        for f in way: f[5]=qa[5]
        # j5 는 항상 고정 -> 이동 중 그리퍼가 기울지 않음 (물컵 규칙)
        if LEVEL_ALWAYS or seg_rotmode(a,b)=='lock':
            lock_j5(way,qa)
        segs.append((f'{a} -> {b}{tag}',way,r[1],get_offset(a),get_offset(b)))
        carry=list(way[-1])              # 실제 끝 자세를 다음 구간 출발점으로
    warns=[]
    for label,_w,_d,_oa,_ob in segs:
        if '⚠' in label:
            warns.append(f"[충돌 경고] {label} - 무시할 장애물이면 해당 ■자세에 등록하세요")
    if check_coll:
        for label,way,dt,offa,offb in segs:
            hits=check_collision(way)
            if hits:
                warns.append(f"[충돌 경고] {label}: "
                             +", ".join(f"{a}↔{b}" for a,b in hits))
    return segs,None,("\n".join(warns))

def h_preview(kind,mode,axorder,speed,loop,coll=False):
    mode=resolve_mode(kind,mode)
    BTN('3D 미리보기',f'{mode} · {speed}')
    if RUNNING.is_set():
        yield gr.update(),"다른 동작 실행 중"; return
    # 계산은 스레드에서 돌리고, 진행 상황을 실시간으로 흘려보낸다
    box={}
    def _work():
        try: box['r']=plan_all(mode,axorder,coll)
        except Exception as e: box['e']=e
    th=threading.Thread(target=_work,daemon=True); th.start()
    t0=time.time()
    while th.is_alive():
        yield gr.update(),f"{PROG_STATE['text']}   ({time.time()-t0:.0f}초)"
        time.sleep(0.4)
    th.join()
    if 'e' in box: yield gr.update(),f"계산 오류: {box['e']}"; return
    segs,err,cwarn=box['r']
    LOG(f"  경로 계산 완료: {time.time()-t0:.1f}초")
    if err: yield gr.update(),err; return
    pre=(cwarn+"\n") if cwarn else ""
    sp=float(speed.replace('x','')); STOP.clear(); RUNNING.set()
    try:
        while True:
            for label,way,dt,offa,offb in segs:
                dec=max(1,int(0.2/dt))          # 화면은 ~5fps (메쉬 렌더링)
                for k,q in enumerate(way):
                    if STOP.is_set():
                        yield robot_fig(q),pre+"미리보기 정지"; return
                    ros_publish(q)
                    if k%dec==0 or k==len(way)-1:
                        yield robot_fig(q),pre+f"미리보기: {label}"
                    time.sleep(dt/sp)
                time.sleep(0.4)
            if not loop: break
        yield gr.update(),pre+"미리보기 완료"
    finally:
        RUNNING.clear()

def h_run(kind,mode,axorder,speed,loop,coll=False):
    mode=resolve_mode(kind,mode)
    BTN('실물 실행',f'{mode} · {speed} · 반복{"ON" if loop else "OFF"} · 충돌검사{"ON" if coll else "OFF"}')
    if RUNNING.is_set():
        yield gr.update(),"다른 동작 실행 중"; return
    # 계산은 스레드에서 돌리고, 진행 상황을 실시간으로 흘려보낸다
    box={}
    def _work():
        try: box['r']=plan_all(mode,axorder,coll)
        except Exception as e: box['e']=e
    th=threading.Thread(target=_work,daemon=True); th.start()
    t0=time.time()
    while th.is_alive():
        yield gr.update(),f"{PROG_STATE['text']}   ({time.time()-t0:.0f}초)"
        time.sleep(0.4)
    th.join()
    if 'e' in box: yield gr.update(),f"계산 오류: {box['e']}"; return
    segs,err,cwarn=box['r']
    LOG(f"  경로 계산 완료: {time.time()-t0:.1f}초")
    if err: yield gr.update(),err; return
    if cwarn:
        yield gr.update(),cwarn+"\n(충돌 경고 있음 - 계속 진행합니다. 멈추려면 정지)"
    if not ROBOT.connect():
        yield gr.update(),f"로봇 연결 실패 ({PORT})"; return
    sp=float(speed.replace('x','')); STOP.clear(); RUNNING.set()
    try:
        ROBOT.time_profile(True,sp); ROBOT.torque(True)
        ps=pose_seq()
        LOG("\n"+"="*66)
        LOG(f"[실행 시작] 속도 {sp}x, 반복 {'ON' if loop else 'OFF'}")
        LOG(f"  파이프라인: {' -> '.join(S['seq'])}")
        LOG("="*66)
        q0=ROBOT.read()
        first=get_angles(ps[0]); off1=get_offset(ps[0])
        first_c=[a+o for a,o in zip(first,level_offset(off1))]
        first_c[5]=q0[5]                           # 그리퍼는 현재 상태 유지
        if LEVEL_ALWAYS: first_c[J5_IDX]=q0[J5_IDX]   # j5 고정
        way,dt=plan_joint_seq(q0,first_c)          # 실물은 보정 적용 목표로
        for f in way: f[5]=q0[5]                   # 이동 중에도 고정
        if LEVEL_ALWAYS: lock_j5(way,q0)
        ros_publish(first)                         # 3D 뷰는 정답(이상 자세) 표시
        LOG(f"\n[첫 자세로 이동] {ps[0]}  ({len(way)}점)")
        yield gr.update(),"첫 자세로 이동 중..."
        for q in way:
            if STOP.is_set(): raise KeyboardInterrupt
            ROBOT.send(q); time.sleep(dt/sp)
        ok,err_raw=ROBOT.settle(first_c)           # 실물 정밀 도착 대기
        LOG_COMPARE(ROBOT.last_want,ROBOT.last_got,
                    f"{ps[0]} {'도착' if ok else '타임아웃'}")
        yield gr.update(),(f"첫 자세 도착 (오차 {err_raw})" if ok
                           else f"첫 자세 근접 (오차 {err_raw}, 타임아웃)")
        while True:
            seg_i=0
            # 첫 자세는 위에서 이미 이동함 -> 그 '자세 블록' 하나만 건너뛰고
            # 나머지는 전부 순서대로 (첫 자세 앞의 그리퍼 블록도 실행되어야 함)
            first_pose_i=next(i for i,x in enumerate(S['seq'])
                              if not is_grip_item(x))
            items=[x for i,x in enumerate(S['seq']) if i!=first_pose_i]
            for item in items:
                if STOP.is_set(): raise KeyboardInterrupt
                if is_grip_item(item):
                    op = (item==GRIP_OPEN_ITEM)
                    tgt=GRIPPER_OPEN if op else GRIPPER_CLOSE
                    LOG(f"\n[그리퍼 {'열기' if op else '닫기'}] 목표 {tgt}")
                    ROBOT.gripper(op)
                    gq=list(CURRENT_Q); gq[5]=_j6_rad(op)
                    ros_publish(gq,with_grip=True)      # 3D 그리퍼도 반영
                    # 도착까지 대기 (스트로크 1200step / 300step/s = 4초 -> 8초 여유)
                    #  닫기: 위치가 멈추면 '물체 잡음' -> 목표를 현재 근처로 낮춰
                    #        살짝만 조인다 (조인 채 이동 시 과부하 보호 방지)
                    #  열기: 멈춤 감지 없음. 안 움직이면 보호 해제+토크 올려 재시도
                    t0=time.time(); p=None; lastp=None; p_start=None
                    still=0; retried=False; why='타임아웃'
                    while time.time()-t0<8.0:
                        if STOP.is_set(): raise KeyboardInterrupt
                        p=ROBOT._r2(GRIP_ID,ADDR_PRESENT)   # 안정 읽기 (밀림 방지)
                        if p is not None:
                            if p_start is None: p_start=p
                            if abs(p-tgt)<25: why='도착'; break
                            if op:
                                # 열기가 1.2초 넘게 꿈쩍 안 하면 보호 걸림 -> 재시도
                                if (not retried and time.time()-t0>1.2
                                        and abs(p-p_start)<15):
                                    LOG('  [그리퍼] 안 움직임 -> 보호 해제 + 토크 상향 재시도')
                                    ROBOT.gripper(True,tq=500); retried=True
                            else:
                                if lastp is not None and abs(p-lastp)<8:
                                    still+=1                # 닫기 중 멈춤 = 잡음
                                    if still>=4:
                                        why='물체 잡음'
                                        # 살짝만 조인 목표로 재설정 (과부하 방지)
                                        hold=max(GRIPPER_CLOSE,p-25)
                                        ROBOT._w2(GRIP_ID,ADDR_GOAL,hold)
                                        LOG(f'  [그리퍼] 잡음 감지 @{p} -> 유지 목표 {hold}')
                                        break
                                else: still=0
                            lastp=p
                        time.sleep(0.12)
                    LOG_COMPARE({GRIP_ID:tgt},{GRIP_ID:p} if p is not None else {},
                                f'그리퍼 {why} ({time.time()-t0:.1f}초)')
                    yield gr.update(),f"그리퍼 {'열기' if op else '닫기'}"
                    time.sleep(0.2)
                    continue
                if seg_i>=len(segs): continue
                label,way,dt,offa,offb=segs[seg_i]; seg_i+=1
                offa=level_offset(offa); offb=level_offset(offb)  # 수평 보정
                LOG(f"\n[이동] {label}  ({len(way)}점)")
                yield robot_fig(way[0]),f"실행: {label}"
                n=max(1,len(way)-1)
                log_every=max(1,len(way)//8)       # 구간당 약 8회만 로그
                for k,q in enumerate(way):
                    if STOP.is_set(): raise KeyboardInterrupt
                    t=k/n
                    qc=[v+oa+(ob-oa)*t for v,oa,ob in zip(q,offa,offb)]
                    ROBOT.send(qc)                 # 실물: 보정 적용
                    # 실물이 이 웨이포인트에 충분히 근접할 때까지 대기
                    # 고속일수록 허용치를 넓혀 부드럽게 (정지-출발 반복 방지)
                    ROBOT.wait_reach(qc,tol=int(FOLLOW_TOL*max(1.0,sp)),
                                     timeout=FOLLOW_TIMEOUT)
                    if k%log_every==0:
                        LOG_COMPARE(ROBOT.last_want,ROBOT.last_got,
                                    f"{k+1}/{len(way)}")
                    ros_publish(q)                 # 3D: 실물과 같은 속도로 진행
                    if STOP.is_set(): raise KeyboardInterrupt
                # --- 구간 끝: 정확히 도착할 때까지 (좁은 허용치) ---
                q_end=[v+o for v,o in zip(way[-1],offb)]
                ok,err_raw=ROBOT.settle(q_end)      # 정밀 수렴 (재전송 포함)
                LOG_COMPARE(ROBOT.last_want,ROBOT.last_got,
                            f"구간끝 {'도착' if ok else '타임아웃'} 오차{err_raw}")
                if STOP.is_set(): raise KeyboardInterrupt
                ros_publish(way[-1])
                yield robot_fig(way[-1]),(f"도착: {label} (오차 {err_raw})" if ok
                                          else f"근접: {label} (오차 {err_raw}, 타임아웃)")
                time.sleep(0.3)
            if not loop: break
        yield gr.update(),"실행 완료 (토크 ON - 해제하려면 [토크 해제])"
    except KeyboardInterrupt:
        ROBOT.estop(); yield gr.update(),"STOPPED - torque off"
    except Exception as e:
        ROBOT.estop(); yield gr.update(),f"ERROR: {e} - torque off"
    finally:
        RUNNING.clear()

# ===== 미세조정 (실물 보정) — 3D 뷰는 정답 유지, 실물만 움직임 =====
FT={'pose':None}
def _off_str(off):
    return '  '.join(f"j{j+1}:{math.degrees(o):+.1f}" for j,o in enumerate(off))
def h_ft_goto(name):
    if not name: return "왼쪽 [자세] 탭에서 자세를 먼저 선택하세요"
    if RUNNING.is_set(): return "다른 동작 실행 중 - [정지] 후 다시"
    if not ROBOT.connect():
        return (f"로봇 연결 실패 ({PORT}) — 확인: "
                "1) USB/전원 연결  2) 다른 프로그램(sts_bridge 등)이 포트를 쓰는 중인지")
    STOP.clear(); RUNNING.set()
    try:
        ROBOT.time_profile(True); ROBOT.torque(True)
        q0=ROBOT.read()
        off=level_offset(get_offset(name))     # 수평 유지 보정
        tgt=[a+o for a,o in zip(get_angles(name),off)]
        tgt[5]=q0[5]                           # 그리퍼는 현재 상태 유지
        if LEVEL_ALWAYS: tgt[J5_IDX]=q0[J5_IDX]
        way,dt=plan_joint_seq(q0,tgt)
        for f in way: f[5]=q0[5]
        if LEVEL_ALWAYS: lock_j5(way,q0)
        ros_publish(get_angles(name))          # 3D 뷰 = 정답 자세
        for q in way:
            if STOP.is_set(): break
            ROBOT.send(q); time.sleep(dt)
        FT['pose']=name
        return f"'{name}' 도착. 관절 선택 후 +/- 로 미세조정하세요. 현재 보정: "+_off_str(off)
    except Exception as e:
        ROBOT.estop(); return f"오류: {e} - 토크 해제됨"
    finally:
        RUNNING.clear()
def h_ft_step(pose_name,joint_label,step_deg,sign):
    """[로봇 이동] 없이도 바로 보정 가능. 선택된 자세를 그대로 사용."""
    name=FT['pose'] or pose_name
    if not name: return "왼쪽 [자세] 탭에서 자세를 먼저 선택하세요"
    FT['pose']=name
    if not ROBOT.connect(): return f"로봇 연결 실패 ({PORT}) - 다른 프로그램이 포트를 쓰고 있는지 확인"
    j=int(joint_label[1])-1
    off=get_offset(name)
    off[j]+=math.radians(float(step_deg))*sign
    set_offset(name,off)
    tgt=[a+o for a,o in zip(get_angles(name),off)]
    try:
        ROBOT.time_profile(True); ROBOT.torque(True)
        ROBOT.send(tgt)                        # 실물만 이동 (3D는 그대로)
    except Exception as e:
        return f"전송 실패: {e}"
    d='+' if sign>0 else '-'
    return f"[{name}] {joint_label} {d}{step_deg}도  |  보정: "+_off_str(off)
def calibrated_poses():
    """보정값이 실제로 들어간 자세 목록."""
    out=[]
    for p in S['poses']:
        off=p.get('offset',[0.0]*6)
        if any(abs(o)>1e-6 for o in off): out.append(p)
    return out

REF_POSE='base'          # 보정 기준 자세 (다른 자세와 섞지 않음)

def nearest_offset(angles,exclude=None):
    """보정 기준은 base 하나만 사용 (자세마다 섞이지 않게).
    반환: (offset, 기준자세명, 관절각차이도) / 없으면 (None,None,None)"""
    if exclude==REF_POSE: return None,None,None
    ref=None
    for p in S['poses']:
        if p['name']==REF_POSE: ref=p; break
    if ref is None: return None,None,None
    off=ref.get('offset',[0.0]*6)
    if not any(abs(o)>1e-6 for o in off): return None,None,None
    d=math.sqrt(sum((a-b)**2 for a,b in zip(angles[:5],ref['angles'][:5])))
    return list(off), REF_POSE, math.degrees(d)

def h_apply_nearest(pose_name):
    """선택 자세에 base 의 보정값을 적용."""
    name=FT['pose'] or pose_name
    if not name: return "자세를 먼저 선택하세요"
    if name==REF_POSE:
        return f"'{REF_POSE}' 는 기준 자세입니다. 직접 보정하세요."
    ang=get_angles(name)
    if ang is None: return "자세를 찾을 수 없습니다"
    off,src,dist=nearest_offset(ang,exclude=name)
    if off is None:
        return f"'{REF_POSE}' 에 보정값이 없습니다. 먼저 {REF_POSE} 를 보정하세요."
    set_offset(name,off)
    return (f"'{name}' <- '{src}' 보정 적용 (관절각 차이 {dist:.1f}도)\n"
            +_off_str(off))

def h_ft_grip(pose_name,open_):
    """미세조정 탭: 그리퍼 상태를 '이 자세'에 저장 + 실물도 동작."""
    name=FT['pose'] or pose_name
    if not name: return "자세를 먼저 선택하세요"
    FT['pose']=name
    rad=_j6_rad(open_)
    with LOCK:
        for p in S['poses']:
            if p['name']==name:
                a=list(p['angles']); a[5]=rad; p['angles']=a
    ros_publish(get_angles(name),with_grip=True)
    lbl='열기' if open_ else '닫기'
    if ROBOT.connect():
        try:
            ROBOT.gripper(open_)
            raw=GRIPPER_OPEN if open_ else GRIPPER_CLOSE
            return f"[{name}] 그리퍼 {lbl} -> 자세에 저장 (실물 {raw})"
        except Exception as e:
            return f"[{name}] 그리퍼 {lbl} 저장됨 (실물 실패: {e})"
    return f"[{name}] 그리퍼 {lbl} -> 자세에 저장 (실물 미연결)"

def h_ft_reset(pose_name):
    name=FT['pose'] or pose_name
    if not name: return "자세를 먼저 선택하세요"
    set_offset(name,[0.0]*6)
    if ROBOT.connect():
        try: ROBOT.send(get_angles(name))
        except Exception: pass
    return f"'{name}' 보정 초기화 (실물 = 정답 자세로)"
def h_ft_save(pose_name):
    name=FT['pose'] or pose_name
    if not name: return "자세를 먼저 선택하세요"
    with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    return f"'{name}' 보정 저장됨: "+_off_str(get_offset(name))+"  (실행 시 자동 적용)"

# ===== 텔레오퍼레이션: 손으로 움직여 녹화 -> 재생 =====
REC={'on':False,'frames':[],'thread':None}
REC_FILE='/home/drboom/2026_project/Industry_gyu_arm/arm_studio_record.json'
REC_HZ=15                 # 초당 샘플 수

def _rec_loop():
    dt=1.0/REC_HZ
    while REC['on']:
        try:
            q=ROBOT.read()
            REC['frames'].append([float(v) for v in q])
        except Exception:
            pass
        time.sleep(dt)

REC_SETTLE=2.0          # 시작 자세 도착 후 토크를 끄기까지 대기(초)
REC_TAIL_STEPS=25       # 녹화 끝 -> 목표 자세로 맞추는 보정 프레임 수

def h_rec_toggle(seg_label):
    BTN('구간 녹화',str(seg_label))
    """녹화 버튼(토글).
    1회차: 시작 자세로 이동 -> 2초 대기 -> 토크 OFF -> 녹화 시작
    2회차: 녹화 중지 -> 끝점을 목표 자세로 최소 보정해서 저장"""
    # ---------- 중지 ----------
    if REC['on']:
        REC['on']=False
        time.sleep(0.3)
        frames=list(REC['frames'])
        seg=REC.get('seg')
        if not seg: return "구간 정보가 없습니다"
        a,b=seg
        if len(frames)<2:
            return f"[{a} → {b}] 녹화가 너무 짧습니다. 다시 시도하세요."
        qb=get_angles(b)
        last=frames[-1]
        # 끝점이 목표와 다르므로 짧게 보간해서 정확히 b 로 맞춤
        gap=max(abs(x-y) for x,y in zip(last[:5],qb[:5]))
        n=REC_TAIL_STEPS if gap>math.radians(0.5) else 0
        for k in range(1,n+1):
            t=k/n; t=t*t*(3-2*t)
            frames.append([l+(g-l)*t for l,g in zip(last,qb)])
        get_moves()[seg_key(a,b)]=frames
        # 갱신된 라벨을 기억 -> 새로고침 후 그 블록이 다시 선택되게
        try:
            si=pose_seq().index(a)           # 이동 블록 인덱스 = 자세 순번
            REC['last_sel']=next(l for l in seq_labels()
                                 if l.lstrip().startswith(f'┃ {si}↓'))
        except Exception:
            REC['last_sel']=None
        try:
            with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
        except Exception as e:
            return f"저장 실패: {e}"
        return (f"[{a} → {b}] 녹화 완료: {len(frames)}프레임 "
                f"({len(frames)/REC_HZ:.1f}초)\n"
                f"끝점 보정 {n}프레임 (차이 {math.degrees(gap):.1f}도) → '{b}' 로 정확히 맞춤\n"
                "이 구간은 이제 녹화 경로로 실행됩니다. (토크 꺼진 상태)")

    # ---------- 시작 ----------
    if RUNNING.is_set():
        return "다른 동작이 실행 중입니다. [정지]를 누른 뒤 다시 시도하세요."
    kind,i=seq_index(seg_label)
    if kind is None:
        return "블록을 선택하세요 (파이프라인에서 ┃ 이동 항목)"
    if kind!='move':
        return f"'{kind}' 블록입니다. ┃ 로 시작하는 [이동] 항목을 선택하세요."
    a,b=move_seg_poses(i)                    # 그리퍼 블록을 건너뛴 자세 쌍
    if a is None or b is None:
        return "이동 구간을 찾을 수 없습니다. 목록이 바뀌었으니 다시 선택하세요."
    if not ROBOT.connect(): return f"로봇 연결 실패 ({PORT})"
    STOP.clear()                             # 이전 정지 상태 해제
    LOG(f"\n[녹화 준비] {a} -> {b} : '{a}' 자세로 이동")
    try:
        ROBOT.time_profile(True); ROBOT.torque(True)
        q0=ROBOT.read(); qa=get_angles(a); offa=get_offset(a)
        tgt=[v+o for v,o in zip(qa,level_offset(offa))]
        tgt[5]=q0[5]                         # 그리퍼는 현재 상태 유지
        if LEVEL_ALWAYS: tgt[J5_IDX]=q0[J5_IDX]
        way,dt=plan_joint_seq(q0,tgt)
        for f in way: f[5]=q0[5]
        if LEVEL_ALWAYS: lock_j5(way,q0)
        for q in way:
            if STOP.is_set(): ROBOT.estop(); return "정지됨"
            ROBOT.send(q); time.sleep(dt)
        ok,err=ROBOT.wait_reach(tgt,tol=25,timeout=8.0)
        ros_publish(qa)
        LOG_COMPARE(ROBOT.last_want,ROBOT.last_got,f"{a} {'도착' if ok else '타임아웃'}")
        time.sleep(REC_SETTLE)              # 도착 후 2초 대기
        ROBOT.torque(False)                 # 손으로 움직일 수 있게
        LOG(f"[녹화 시작] 토크 OFF - 손으로 '{b}' 까지 움직이세요")
    except Exception as e:
        try: ROBOT.estop()
        except Exception: pass
        return f"시작 자세 이동 실패: {e}"
    REC['frames']=[]; REC['on']=True; REC['seg']=(a,b)
    REC['last_sel']=seg_label
    REC['thread']=threading.Thread(target=_rec_loop,daemon=True)
    REC['thread'].start()
    return (f"● 녹화 중 [{a} → {b}]\n"
            f"'{a}' 도착 후 토크를 껐습니다. 손으로 '{b}' 자세까지 움직이세요.\n"
            "끝나면 같은 버튼을 한 번 더 누르세요.")

def h_rec_play(speed,loop):
    """녹화 경로 재생. 실물이 따라오길 기다리며 3D도 함께."""
    if RUNNING.is_set():
        yield gr.update(),"다른 동작 실행 중"; return
    if REC['on']:
        yield gr.update(),"녹화 중입니다 - [녹화 정지] 먼저"; return
    frames=REC['frames']
    if not frames:
        try:
            with open(REC_FILE) as f: frames=json.load(f)['frames']
        except Exception:
            yield gr.update(),"녹화 데이터가 없습니다. 먼저 [녹화 시작]"; return
    if not ROBOT.connect():
        yield gr.update(),f"로봇 연결 실패 ({PORT})"; return
    sp=float(speed.replace('x','')); STOP.clear(); RUNNING.set()
    try:
        ROBOT.time_profile(True,sp); ROBOT.torque(True)
        # 첫 프레임으로 안전 이동
        q0=ROBOT.read()
        way,dt0=plan_joint_seq(q0,frames[0])
        yield gr.update(),"녹화 시작 자세로 이동 중..."
        for q in way:
            if STOP.is_set(): raise KeyboardInterrupt
            ROBOT.send(q); time.sleep(dt0/sp)
        ROBOT.wait_reach(frames[0],tol=30,timeout=6.0)
        step=max(1,int(REC_HZ/10))          # 화면은 ~10fps
        while True:
            for k,q in enumerate(frames):
                if STOP.is_set(): raise KeyboardInterrupt
                ROBOT.send(q)
                ROBOT.wait_reach(q,tol=FOLLOW_TOL,timeout=FOLLOW_TIMEOUT)
                ros_publish(q)
                if k%step==0:
                    yield robot_fig(q),f"재생 {k+1}/{len(frames)}"
            if not loop: break
        yield gr.update(),f"재생 완료 ({len(frames)}프레임)"
    except KeyboardInterrupt:
        ROBOT.estop(); yield gr.update(),"정지 - 토크 해제"
    except Exception as e:
        ROBOT.estop(); yield gr.update(),f"오류: {e} - 토크 해제"
    finally:
        RUNNING.clear()

def h_rec_load():
    try:
        with open(REC_FILE) as f: d=json.load(f)
        REC['frames']=d['frames']
        n=len(REC['frames'])
        return f"녹화 불러옴: {n}프레임 ({n/d.get('hz',REC_HZ):.1f}초)"
    except Exception as e:
        return f"불러오기 실패: {e}"

def h_stop():
    BTN('정지','토크 해제')
    STOP.set()
    REC['on']=False              # 녹화 중이면 함께 정지
    if ROBOT.port: ROBOT.estop()
    return "정지 - 토크 해제됨"

def h_torque_off():
    BTN('토크 해제')
    if ROBOT.connect(): ROBOT.estop(); return "torque off"
    return "robot not connected"

def h_save():
    BTN('저장')
    with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    return f"saved -> {PROJECT_FILE}"

def h_load():
    BTN('불러오기')
    global S
    try:
        with open(PROJECT_FILE) as f: data=json.load(f)
        with LOCK:
            S['poses']=data['poses']; S['seq']=data['seq']
            if not any(p['name']=='base' for p in S['poses']):
                S['poses'].insert(0,{'name':'base','angles':[0.0]*6})
        return gr.update(choices=pose_names(),value='base'), gr.update(choices=pose_names()), \
               gr.update(choices=seq_labels(),value=None), "loaded"
    except FileNotFoundError:
        return gr.update(),gr.update(),gr.update(),"no project file"

# 시작 시 자동 로드 (URDF 패키지 안의 calib.json)
try:
    with open(PROJECT_FILE) as f: _d=json.load(f)
    S['poses']=_d['poses']; S['seq']=_d['seq']
    S['moves']=_d.get('moves',{})
    S['axorders']=_d.get('axorders',{})
    S['rotmodes']=_d.get('rotmodes',{})
    if not any(p['name']=='base' for p in S['poses']):
        S['poses'].insert(0,{'name':'base','angles':[0.0]*6,'offset':[0.0]*6})
    print(f"  자세 {len(S['poses'])}개, 녹화 {len(S['moves'])}건 불러옴")
except FileNotFoundError:
    print("  (새 URDF - 보정 데이터 없음, base 부터 시작)")

# ===== RViz 스타일 네이티브 웹 뷰어 (three.js, VNC보다 빠름) =====
# 메쉬는 브라우저가 1번만 로드, 이후 관절값만 폴링 -> 60fps 부드러움
import http.server, socketserver
VIEWER_PORT=7870
MESH_REG={}          # id -> (path, scale)
ENV_SHOW=[]          # [{'id','scale','color'}]
for ln in ROBOT_LINKS:
    MESH_REG[ln]=(os.path.join(MESH_DIR,ln+'.STL'),1.0)
for fn,col in ENV_MESHES:
    p=os.path.join(MESH_DIR,fn)
    if not os.path.exists(p): continue
    mid=fn.replace('.STL','')
    MESH_REG[mid]=(p,0.001)
    ENV_SHOW.append({'id':mid,'color':col,'visible':True})
# /home/drboom/2026_project/stl 자동 스캔 (환경 STL, mm 단위)
EXTRA_STL_DIR='/home/drboom/2026_project/stl'
import glob as _glob
for p in sorted(_glob.glob(os.path.join(EXTRA_STL_DIR,'*.STL'))
                +_glob.glob(os.path.join(EXTRA_STL_DIR,'*.stl'))):
    mid=os.path.basename(p).rsplit('.',1)[0]
    mid=mid.replace('Industry_gyu_arm - ','').replace(' ','_')
    if mid in MESH_REG: continue
    n=mid.lower()
    col='#6699e6' if 'docking' in n else ('#80cc80' if 'seed' in n else '#999999')
    MESH_REG[mid]=(p,0.001)
    ENV_SHOW.append({'id':mid,'color':col,'visible':True})

# ===== 바닥 플레이트 타일 관리 (200mm 격자 스냅) =====
PLATE_SIZE=0.200          # m (200mm)
PLATE_SRC=None            # 복제에 쓸 원본 플레이트 STL id
for e in ENV_SHOW:
    if 'plate_bottom' in e['id'].lower(): PLATE_SRC=e['id']; break
PLATE_MESH_PATH=MESH_REG[PLATE_SRC][0] if PLATE_SRC else None

def _plate_bounds_m(path):
    mm=trimesh.load(path)
    return mm.bounds*0.001   # mm -> m

# 격자 원점: 실제 플레이트가 -0.1 배수 위치에 놓여 있으므로 그 기준으로 인덱싱
GRID_ORG=(-0.100,-0.100)
def tile_of(ox,oy):
    return (int(round((ox-GRID_ORG[0])/PLATE_SIZE)),
            int(round((oy-GRID_ORG[1])/PLATE_SIZE)))
def tile_origin(t):
    return (GRID_ORG[0]+t[0]*PLATE_SIZE, GRID_ORG[1]+t[1]*PLATE_SIZE)

# 기존 플레이트들의 타일 좌표(정수 격자) 파악
PLATES={}                 # (ix,iy) -> {'id','origin'(m), 'src'}
def _register_existing_plates():
    for e in list(ENV_SHOW):
        if 'plate_bottom' not in e['id'].lower(): continue
        b=_plate_bounds_m(MESH_REG[e['id']][0])
        ox,oy=b[0][0],b[0][1]                    # 좌하단(원점)
        ix,iy=tile_of(ox,oy)
        PLATES[(ix,iy)]={'id':e['id'],'origin':(ox,oy),'added':False}
        e['tile']=(ix,iy)
_register_existing_plates()

PLATE_EDIT={'on':False,'sel':None}    # 편집 모드 / 선택 타일
def plate_tiles(): return sorted(PLATES.keys())
def ghost_tiles():
    """기존 플레이트 4방위 중 비어있는 자리."""
    g=set()
    for (ix,iy) in PLATES:
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            t=(ix+dx,iy+dy)
            if t not in PLATES: g.add(t)
    return sorted(g)
def tile_label(t):
    return f"({t[0]},{t[1]})"
def add_plate(tile):
    """빈 타일에 플레이트 추가 (원본 STL을 오프셋만 다르게 복제)."""
    if tile in PLATES or PLATE_SRC is None: return False
    ix,iy=tile
    mid=f"plate_tile_{ix}_{iy}"
    k=2; base=mid
    while mid in MESH_REG: mid=f"{base}_{k}"; k+=1
    # 원본 플레이트 STL의 현재 원점 -> 목표 타일 원점으로 평행이동
    b=_plate_bounds_m(PLATE_MESH_PATH)
    tox,toy=tile_origin(tile)
    off=(tox-b[0][0], toy-b[0][1], 0.0)
    MESH_REG[mid]=(PLATE_MESH_PATH,0.001)
    ENV_SHOW.append({'id':mid,'color':'#8f8f8f','visible':True,
                     'tile':tile,'offset':off,'added':True})
    PLATES[tile]={'id':mid,'origin':(tox,toy),'added':True}
    return True
def remove_plate(tile):
    if tile not in PLATES: return False
    info=PLATES[tile]
    if not info.get('added'): return False      # 원본 STL은 삭제 금지
    mid=info['id']
    ENV_SHOW[:] = [e for e in ENV_SHOW if e['id']!=mid]
    MESH_REG.pop(mid,None); _COLL_ENV.pop(mid,None)
    del PLATES[tile]
    return True

VIEWER_HTML="""<!DOCTYPE html><html><head><meta charset="utf-8">
<style>body{margin:0;overflow:hidden;background:#303030}
#lg{position:fixed;left:8px;top:8px;color:#ddd;font:12px sans-serif;
    background:rgba(0,0,0,.45);padding:6px 9px;border-radius:5px;line-height:1.5}
#lg b{font-weight:700}</style>
<script type="importmap">{"imports":{
 "three":"https://unpkg.com/three@0.160.0/build/three.module.js",
 "three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}</script>
</head><body>
<div id="lg">축 (base_link 고정)<br>
<b style="color:#ff5555">X</b> <b style="color:#55dd55">Y</b> <b style="color:#55aaff">Z</b>
&nbsp;· 큰축=원점, 작은축=EE</div>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
const scene=new THREE.Scene(); scene.background=new THREE.Color(0x303030);
// 부모(스튜디오)의 다크/라이트 토글을 따라간다
const _THEMES={dark:{bg:0x303030,fg:'#ddd',pane:'rgba(0,0,0,.45)'},
               light:{bg:0xe9ecf0,fg:'#222',pane:'rgba(255,255,255,.65)'}};
window.addEventListener('message',e=>{
  const t=e.data&&e.data.armTheme; if(!t||!_THEMES[t])return;
  scene.background=new THREE.Color(_THEMES[t].bg);
  const lg=document.getElementById('lg');
  if(lg){lg.style.color=_THEMES[t].fg;lg.style.background=_THEMES[t].pane;}
});
const cam=new THREE.PerspectiveCamera(50,innerWidth/innerHeight,0.01,10);
cam.position.set(0.55,0.45,0.75);
const ren=new THREE.WebGLRenderer({antialias:true});
ren.setSize(innerWidth,innerHeight); document.body.appendChild(ren.domElement);
const ctl=new OrbitControls(cam,ren.domElement); ctl.target.set(0,0.15,0.25);
scene.add(new THREE.HemisphereLight(0xffffff,0x445566,1.0));
const dl=new THREE.DirectionalLight(0xffffff,1.2); dl.position.set(1,2,1); scene.add(dl);
// z-up(로봇) -> y-up(three.js)
const world=new THREE.Group(); world.rotation.x=-Math.PI/2; scene.add(world);
const grid=new THREE.GridHelper(2,20,0x666666,0x444444); scene.add(grid);

// ---- 좌표축 (base_link 원점, 월드 고정) : x=빨강 y=초록 z=파랑 ----
function axisLabel(txt,color,pos){
  const c=document.createElement('canvas'); c.width=c.height=64;
  const g=c.getContext('2d');
  g.fillStyle=color; g.font='bold 44px sans-serif';
  g.textAlign='center'; g.textBaseline='middle'; g.fillText(txt,32,32);
  const sp=new THREE.Sprite(new THREE.SpriteMaterial({
    map:new THREE.CanvasTexture(c),depthTest:false,transparent:true}));
  sp.position.copy(pos); sp.scale.set(0.05,0.05,0.05);
  return sp;
}
function makeAxes(len,lw){
  const g=new THREE.Group();
  const defs=[[new THREE.Vector3(len,0,0),0xff3333,'X'],
              [new THREE.Vector3(0,len,0),0x33cc33,'Y'],
              [new THREE.Vector3(0,0,len),0x3399ff,'Z']];
  for(const [v,col,name] of defs){
    const geo=new THREE.BufferGeometry().setFromPoints(
      [new THREE.Vector3(0,0,0),v]);
    g.add(new THREE.Line(geo,new THREE.LineBasicMaterial(
      {color:col,linewidth:lw,depthTest:false})));
    g.add(axisLabel(name,'#'+col.toString(16).padStart(6,'0'),
                    v.clone().multiplyScalar(1.12)));
  }
  return g;
}
world.add(makeAxes(0.20,3));          // base_link 원점 축 (크게)

// ---- EE 좌표축 (엔드이펙터 위치, 방향은 월드축과 동일) ----
const eeAxes=makeAxes(0.08,2); world.add(eeAxes);
const loader=new STLLoader(); const objs={};
function ensure(id,color,scale){
  if(objs[id])return;
  objs[id]={mesh:null,tp:new THREE.Vector3(),tq:new THREE.Quaternion(),
            cp:new THREE.Vector3(),cq:new THREE.Quaternion(),s:scale,init:false};
  loader.load('/mesh/'+id,g=>{
    g.computeVertexNormals();
    const m=new THREE.Mesh(g,new THREE.MeshPhongMaterial({color:color,shininess:60,specular:0x333333}));
    m.matrixAutoUpdate=false;
    world.add(m); objs[id].mesh=m;
  });
}
const _m4=new THREE.Matrix4();
function setTarget(id,a){ // a: row-major 16 -> 목표 pos/quat 저장 (보간용)
  const o=objs[id]; if(!o)return;
  _m4.set(a[0],a[1],a[2],a[3], a[4],a[5],a[6],a[7],
          a[8],a[9],a[10],a[11], 0,0,0,1);
  const sc=new THREE.Vector3();
  _m4.decompose(o.tp,o.tq,sc);
  if(!o.init){o.cp.copy(o.tp);o.cq.copy(o.tq);o.init=true;}
}
// ---- 고스트 플레이트 (반투명, 클릭하면 확장) ----
const ghostGroup=new THREE.Group(); world.add(ghostGroup);
const ghostMat=new THREE.MeshPhongMaterial({color:0x44ff88,transparent:true,
  opacity:0.28,side:THREE.DoubleSide,depthWrite:false});
const ghostMatHi=new THREE.MeshPhongMaterial({color:0x66ffaa,transparent:true,
  opacity:0.55,side:THREE.DoubleSide,depthWrite:false});
let ghostMeshes=[];
function buildGhosts(list,psize){
  const key=JSON.stringify(list.map(g=>g.tile));
  if(ghostGroup.userData.key===key) return;
  ghostGroup.userData.key=key;
  ghostGroup.clear(); ghostMeshes=[];
  for(const g of list){
    const geo=new THREE.BoxGeometry(psize*0.96,psize*0.96,0.02);
    const m=new THREE.Mesh(geo,ghostMat);
    m.position.set(g.x+psize/2,g.y+psize/2,0.01);
    m.userData.tile=g.tile;
    ghostGroup.add(m); ghostMeshes.push(m);
    // 테두리
    const eg=new THREE.LineSegments(new THREE.EdgesGeometry(geo),
      new THREE.LineBasicMaterial({color:0x33ff99}));
    eg.position.copy(m.position); ghostGroup.add(eg);
  }
}
const ray=new THREE.Raycaster(), mouse=new THREE.Vector2();
let hover=null;
ren.domElement.addEventListener('pointermove',ev=>{
  const r=ren.domElement.getBoundingClientRect();
  mouse.x=((ev.clientX-r.left)/r.width)*2-1;
  mouse.y=-((ev.clientY-r.top)/r.height)*2+1;
  ray.setFromCamera(mouse,cam);
  const hits=ray.intersectObjects(ghostMeshes,false);
  if(hover) hover.material=ghostMat;
  hover=hits.length?hits[0].object:null;
  if(hover){ hover.material=ghostMatHi; ren.domElement.style.cursor='pointer'; }
  else ren.domElement.style.cursor='';
});
ren.domElement.addEventListener('click',async ev=>{
  if(!hover) return;
  const t=hover.userData.tile;
  await fetch('/click/'+t[0]+','+t[1]);
  ghostGroup.userData.key=null;      // 다음 폴링에서 갱신
});

async function poll(){
  try{
    const st=await (await fetch('/state')).json();
    for(const l of st.links){ ensure(l.id,l.color,1.0); setTarget(l.id,l.m); }
    const alive=new Set();
    for(const e of st.env){ ensure(e.id,e.color,e.scale); alive.add(e.id);
      const o=e.off||[0,0,0];
      setTarget(e.id,[1,0,0,o[0], 0,1,0,o[1], 0,0,1,o[2], 0,0,0,1]);
      objs[e.id].vis=e.v; objs[e.id].env=true; }
    // 삭제된 환경 오브젝트 정리
    for(const id in objs){
      if(id.startsWith('plate_tile_')&&!alive.has(id)&&objs[id].mesh){
        world.remove(objs[id].mesh); delete objs[id];
      }
    }
    buildGhosts(st.ghosts||[],st.psize||0.2);
    if(st.eexyz) eeAxes.position.set(st.eexyz[0],st.eexyz[1],st.eexyz[2]);
    document.title='EE '+st.ee;
  }catch(err){}
  setTimeout(poll,50);
}
poll();
addEventListener('resize',()=>{cam.aspect=innerWidth/innerHeight;
  cam.updateProjectionMatrix();ren.setSize(innerWidth,innerHeight);});
let last=performance.now();
const _sm=new THREE.Matrix4(), _ss=new THREE.Vector3(1,1,1);
(function anim(now){
  requestAnimationFrame(anim);
  const dt=Math.min((now-last)/1000||0.016,0.1); last=now;
  const a=1-Math.exp(-dt*18);          // 부드러운 추종 (약 55ms 시정수)
  for(const id in objs){
    const o=objs[id];
    if(!o.mesh||!o.init)continue;
    if(o.env){ o.cp.copy(o.tp); o.cq.copy(o.tq); }   // 환경: 즉시
    else { o.cp.lerp(o.tp,a); o.cq.slerp(o.tq,a); }  // 로봇: 부드럽게
    _ss.set(o.s,o.s,o.s);
    o.mesh.matrix.compose(o.cp,o.cq,_ss);
    o.mesh.visible=(o.vis!==false);
  }
  ctl.update();ren.render(scene,cam);
})(performance.now());
</script></body></html>"""

ROBOT_COLOR_MAP=dict(zip(ROBOT_LINKS,ROBOT_COLORS))
class _ViewerHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self,*a): pass
    def _send(self,code,ctype,body):
        self.send_response(code)
        self.send_header('Content-Type',ctype)
        self.send_header('Content-Length',str(len(body)))
        self.end_headers(); self.wfile.write(body)
    def do_GET(self):
        try:
            if self.path=='/' or self.path.startswith('/index'):
                self._send(200,'text/html',VIEWER_HTML.encode())
            elif self.path=='/state':
                frames=link_frames(CURRENT_Q)
                links=[]
                for k,ln in enumerate(ROBOT_LINKS):
                    T=np.eye(4) if k==0 else frames[k]
                    links.append({'id':ln,'color':ROBOT_COLOR_MAP[ln],
                                  'm':[float(x) for x in T[:3,:].flatten()]+[0,0,0,1]})
                p,_=fk(CURRENT_Q)
                env=[]
                for e in ENV_SHOW:
                    env.append({'id':e['id'],'color':e['color'],
                                'scale':MESH_REG[e['id']][1],
                                'v':e.get('visible',True),
                                'off':list(e.get('offset',(0,0,0))),
                                'tile':list(e['tile']) if e.get('tile') else None,
                                'plate':'plate' in e['id'].lower()})
                ghosts=[{'tile':list(t),
                         'x':tile_origin(t)[0],'y':tile_origin(t)[1]}
                        for t in ghost_tiles()] if PLATE_EDIT['on'] else []
                body=json.dumps({'links':links,'env':env,'ghosts':ghosts,
                    'psize':PLATE_SIZE,'sel':PLATE_EDIT.get('sel'),
                    'ee':f"x={p[0]*1000:.0f} y={p[1]*1000:.0f} z={p[2]*1000:.0f}mm",
                    'eexyz':[float(v) for v in p]}).encode()
                self._send(200,'application/json',body)
            elif self.path.startswith('/click/'):
                # 뷰어에서 고스트 클릭 -> 플레이트 추가
                try:
                    ix,iy=self.path[len('/click/'):].split(',')
                    ok=add_plate((int(ix),int(iy)))
                    self._send(200,'application/json',
                               json.dumps({'ok':ok}).encode())
                except Exception:
                    self._send(400,'text/plain',b'bad')
            elif self.path.startswith('/mesh/'):
                mid=self.path[len('/mesh/'):]
                if mid in MESH_REG:
                    with open(MESH_REG[mid][0],'rb') as f: data=f.read()
                    self._send(200,'application/octet-stream',data)
                else: self._send(404,'text/plain',b'not found')
            else: self._send(404,'text/plain',b'not found')
        except Exception:
            try: self._send(500,'text/plain',b'err')
            except Exception: pass
class _TS(socketserver.ThreadingMixIn,http.server.HTTPServer):
    daemon_threads=True; allow_reuse_address=True
def _start_viewer():
    global VIEWER_PORT
    for p in range(7870,7890):
        try:
            srv=_TS(('0.0.0.0',p),_ViewerHandler)
            VIEWER_PORT=p
            threading.Thread(target=srv.serve_forever,daemon=True).start()
            print(f'Viewer: http://localhost:{p}')
            return True
        except OSError: continue
    return False
_start_viewer()

# ===== 진짜 RViz 웹 스트리밍 자동 기동 (Xvfb + x11vnc + noVNC) =====
import shutil, subprocess, socket
RVIZ_WEB_URL='http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
def _port_used(p):
    s=socket.socket(); s.settimeout(0.3)
    try: s.connect(('127.0.0.1',p)); s.close(); return True
    except Exception: return False
RVIZ_WEB_OK=False
if _port_used(6080):
    RVIZ_WEB_OK=True                      # 이미 떠 있음
elif all(shutil.which(b) for b in ['Xvfb','x11vnc','websockify']) and \
     os.path.exists('/usr/share/novnc/vnc.html'):
    subprocess.Popen(['bash','/home/drboom/2026_project/Industry_gyu_arm/rviz_web.sh'],
                     stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
                     start_new_session=True)
    for _ in range(40):
        if _port_used(6080): RVIZ_WEB_OK=True; break
        time.sleep(0.5)
if RVIZ_WEB_OK:
    print('RViz web: ON ->',RVIZ_WEB_URL)
else:
    print('RViz web: OFF (sudo apt install -y xvfb x11vnc novnc websockify 후 재실행)')

# ================= 간편 이동 UI (2D 맵 + 경로 그리기) =================
DRAW2D={'pts':[]}          # 지금 그리는 중인 경로 (저장 전)

def _sel_seg(sel):
    """선택한 블록이 ┃이동이면 (a,b), 아니면 (None,None)."""
    kind,i=seq_index(sel) if sel else (None,None)
    if kind!='move': return None,None
    return move_seg_poses(i)

MAP_W=1040                     # 2배 해상도 렌더 -> 화면에서 선명 (사진느낌 방지)
_MAP_FONTS={}
def _map_font(sz):
    """한글 지원 폰트 (없으면 기본). 크기별 캐시."""
    if sz in _MAP_FONTS: return _MAP_FONTS[sz]
    from PIL import ImageFont
    f=None
    import glob as _g
    for pat in ('/usr/share/fonts/truetype/nanum/NanumGothic*.ttf',
                '/usr/share/fonts/truetype/nanum/*.ttf'):
        c=sorted(_g.glob(pat))
        if c:
            try: f=ImageFont.truetype(c[0],sz); break
            except Exception: pass
    if f is None: f=ImageFont.load_default()
    _MAP_FONTS[sz]=f; return f

def _dash(dr,p0,p1,fill,width,dash=14):
    """점선 (벡터 느낌의 안내선)."""
    import math as _m
    x0,y0=p0; x1,y1=p1
    L=_m.hypot(x1-x0,y1-y0)
    if L<1: return
    n=max(1,int(L/dash))
    for k in range(n+1):
        t0=k/(n+1); t1=(k+0.55)/(n+1)
        dr.line([x0+(x1-x0)*t0,y0+(y1-y0)*t0,
                 x0+(x1-x0)*t1,y0+(y1-y0)*t1],fill=fill,width=width)

def render_map2d_png(sel=None):
    """(구) PIL 렌더러 - SVG 로 대체됨. 보존용."""
    from PIL import Image as PImage, ImageDraw
    W=MAP_W; sc=W/(G2_MAX-G2_MIN)
    F=_map_font(22); FS=_map_font(19)
    def PX(x,y): return ((x-G2_MIN)*sc, W-(y-G2_MIN)*sc)
    img=PImage.new('RGB',(W,W),(253,253,251))
    dr=ImageDraw.Draw(img)
    build_obs_grid()
    cell_px=G2_RES*sc
    def _fill_cells(mask,col):
        for i,j in zip(*np.nonzero(mask)):
            x,y=_cxy(i,j); px,py=PX(x,y)
            dr.rectangle([px-cell_px/2,py-cell_px/2,
                          px+cell_px/2,py+cell_px/2],fill=col)
    # 선택한 ■자세의 무시 장애물은 연두색으로 (경고 대상에서 빠짐)
    ign=set()
    kind0,i0=seq_index(sel) if sel else (None,None)
    if kind0=='pose':
        ign=set(S.setdefault('ignore_obs',{}).get(S['seq'][i0],[]))
    for mid,mk in _OBS['ids'].items():
        _fill_cells(mk,(150,210,150) if mid in ign else (90,90,90))
    if _OBS['base'] is not None:
        _fill_cells(_OBS['base'],(70,90,160))    # 로봇 base
    for v in np.arange(G2_MIN,G2_MAX+1e-9,0.05):  # 5cm 격자선
        p=(v-G2_MIN)*sc
        w=3 if abs(v)<1e-9 else 1
        gc=(140,140,140) if abs(v)<1e-9 else (228,228,228)
        dr.line([p,0,p,W],fill=gc,width=w); dr.line([0,W-p,W,W-p],fill=gc,width=w)
    dr.text((W-64,PX(0,0)[1]-30),'+x',fill=(160,30,30),font=F)
    dr.text((PX(0,0)[0]+10,8),'+y',fill=(30,130,30),font=F)
    ox,oy=PX(0,0)                                 # base_link 원점 (좌표 기준)
    dr.line([ox-9,oy,ox+9,oy],fill=(120,120,120),width=3)
    dr.line([ox,oy-9,ox,oy+9],fill=(120,120,120),width=3)
    dr.text((ox+12,oy+4),'원점',fill=(120,120,120),font=FS)
    jx,jy=PX(*_J1_XY)                             # 팔 기둥 (j1 회전축)
    dr.ellipse([jx-11,jy-11,jx+11,jy+11],fill=(40,40,200))
    dr.text((jx+15,jy+4),'로봇 base (j1 축)',fill=(40,40,200),font=F)
    for p in S['poses']:                          # 자세 위치
        pos,_=fk(p['angles']); px,py=PX(pos[0],pos[1])
        dr.ellipse([px-9,py-9,px+9,py+9],outline=(200,120,0),width=5)
        dr.text((px+13,py-12),p['name'][:14],fill=(160,90,0),font=F)
    a,b=_sel_seg(sel)
    p0=p1=None
    if a and b:
        p0,_=fk(get_angles(a)); p1,_=fk(get_angles(b))
        saved=S.setdefault('paths2d',{}).get(seg_key(a,b))
        if saved:                                  # 저장된 경로 (초록)
            pts=_orthize([tuple(p0[:2])]+[tuple(x) for x in saved]
                         +[tuple(p1[:2])])
            dr.line([PX(x,y) for x,y in pts],fill=(30,160,60),width=8)
    # 그리는 중 경로 (주황) - 블록 선택 여부와 무관하게 항상 표시
    if DRAW2D['pts']:
        pts=list(DRAW2D['pts'])
        if p0 is not None: pts=[tuple(p0[:2])]+pts
        if len(pts)>1:
            dr.line([PX(x,y) for x,y in _orthize(pts)],fill=(255,140,0),width=9)
        for x,y in DRAW2D['pts']:
            px,py=PX(x,y)
            dr.ellipse([px-10,py-10,px+10,py+10],fill=(255,150,20),
                       outline=(185,95,0),width=3)
    if p0 is not None:
        # 마지막 점(없으면 출발점) -> 도착점 안내 점선: 남은 구간을 항상 보여줌
        tail=DRAW2D['pts'][-1] if DRAW2D['pts'] else tuple(p0[:2])
        if math.hypot(tail[0]-p1[0],tail[1]-p1[1])>0.005:
            seg=_orthize([tail,(p1[0],p1[1])])
            for u,v in zip(seg[:-1],seg[1:]):
                _dash(dr,PX(*u),PX(*v),(230,120,120),5)
        for pos,col,t in ((p0,(0,110,220),'출발'),(p1,(220,60,60),'도착')):
            px,py=PX(pos[0],pos[1])
            dr.ellipse([px-12,py-12,px+12,py+12],fill=col,
                       outline=(255,255,255),width=3)
            dr.text((px+15,py+6),t,fill=col,font=F)
    # 사용 안내 (좌상단)
    if a and b:
        hint=f"[{a} → {b}]  클릭=점 추가 · 점선=도착점까지 남은 구간 (자동 연결)"
    else:
        hint="클릭=점 추가 · ┃이동 블록을 선택하면 출발/도착점과 연결됩니다"
    tw=dr.textlength(hint,font=F)
    dr.rectangle([8,8,tw+28,46],fill=(255,255,240),outline=(200,200,160),width=2)
    dr.text((18,14),hint,fill=(90,90,40),font=F)
    return np.array(img)

def h_simple_set(prefer,clear,extra):
    """간편 이동 전역 설정 저장."""
    S.setdefault('simple',{})
    S['simple'].update(prefer=('x' if prefer.startswith('x') else 'y'),
                       clear_cm=float(clear),extra_cm=float(extra))
    _clear_plan_cache()
    try:
        with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    except Exception: pass
    return render_map2d(),f"간편 이동 설정: {prefer} · 여유 {clear:.0f}cm · 하강여유 {extra:.0f}cm"

# ===== SVG 벡터 맵 (사진이 아니라 브라우저가 직접 그림 - 항상 선명) =====
MAP_VB=900          # SVG viewBox 크기
def render_map2d(sel=None):
    """위에서 본 맵을 SVG 로 렌더. 클릭하면 좌표가 숨은 텍스트박스로 전달된다."""
    build_obs_grid()
    sc=MAP_VB/(G2_MAX-G2_MIN); cell=G2_RES*sc
    def SX(x,y): return ((x-G2_MIN)*sc, MAP_VB-(y-G2_MIN)*sc)
    E=[]
    # 5cm 격자 + 축
    for v in np.arange(G2_MIN,G2_MAX+1e-9,0.05):
        p=(v-G2_MIN)*sc
        col='#8c8c8c' if abs(v)<1e-9 else '#e6e6e6'
        w=2.5 if abs(v)<1e-9 else 1
        E.append(f'<line x1="{p:.0f}" y1="0" x2="{p:.0f}" y2="{MAP_VB}" stroke="{col}" stroke-width="{w}"/>')
        E.append(f'<line x1="0" y1="{MAP_VB-p:.0f}" x2="{MAP_VB}" y2="{MAP_VB-p:.0f}" stroke="{col}" stroke-width="{w}"/>')
    ox,oy=SX(0,0)
    E.append(f'<text x="{MAP_VB-46}" y="{oy-10:.0f}" fill="#a02020" font-size="22">+x</text>')
    E.append(f'<text x="{ox+8:.0f}" y="26" fill="#1e8236" font-size="22">+y</text>')
    # 장애물 / base 셀
    ign=set()
    kind0,i0=seq_index(sel) if sel else (None,None)
    if kind0=='pose':
        ign=set(S.setdefault('ignore_obs',{}).get(S['seq'][i0],[]))
    def cells(mask,col,op='1'):
        for i,j in zip(*np.nonzero(mask)):
            x,y=_cxy(i,j); px,py=SX(x,y)
            E.append(f'<rect x="{px-cell/2:.1f}" y="{py-cell/2:.1f}" '
                     f'width="{cell:.1f}" height="{cell:.1f}" fill="{col}" opacity="{op}"/>')
    for mid,mk in _OBS['ids'].items():
        cells(mk,'#96d296' if mid in ign else '#5a5a5a')
    if _OBS['base'] is not None: cells(_OBS['base'],'#4666a0')
    # 원점 + 기둥
    E.append(f'<path d="M{ox-8:.0f} {oy:.0f} h16 M{ox:.0f} {oy-8:.0f} v16" stroke="#909090" stroke-width="2.5"/>')
    E.append(f'<text x="{ox+10:.0f}" y="{oy+20:.0f}" fill="#909090" font-size="17">원점</text>')
    jx,jy=SX(*_J1_XY)
    E.append(f'<circle cx="{jx:.0f}" cy="{jy:.0f}" r="9" fill="#2828c8"/>')
    E.append(f'<text x="{jx+13:.0f}" y="{jy+6:.0f}" fill="#2828c8" font-size="20">로봇 base (j1 축)</text>')
    # 자세 위치
    for p in S['poses']:
        pos,_=fk(p['angles']); px,py=SX(pos[0],pos[1])
        E.append(f'<circle cx="{px:.0f}" cy="{py:.0f}" r="8" fill="none" stroke="#c87800" stroke-width="4"/>')
        E.append(f'<text x="{px+12:.0f}" y="{py-8:.0f}" fill="#a05a00" font-size="20">{p["name"][:14]}</text>')
    # 선택 구간: 저장 경로(초록) / 그리는 중(주황) / 도착 안내 점선
    a,b=_sel_seg(sel); p0=p1=None
    def poly(pts,col,w,dash=''):
        pt=' '.join(f'{x:.0f},{y:.0f}' for x,y in (SX(u,v) for u,v in pts))
        d=f' stroke-dasharray="{dash}"' if dash else ''
        E.append(f'<polyline points="{pt}" fill="none" stroke="{col}" stroke-width="{w}" stroke-linejoin="round" stroke-linecap="round"{d}/>')
    if a and b:
        p0,_=fk(get_angles(a)); p1,_=fk(get_angles(b))
        saved=S.setdefault('paths2d',{}).get(seg_key(a,b))
        if saved:
            poly(_orthize([tuple(p0[:2])]+[tuple(x) for x in saved]
                          +[tuple(p1[:2])]),'#1ea03c',7)
    if DRAW2D['pts']:
        pts=list(DRAW2D['pts'])
        if p0 is not None: pts=[tuple(p0[:2])]+pts
        if len(pts)>1: poly(_orthize(pts),'#ff8c00',8)
        for x,y in DRAW2D['pts']:
            px,py=SX(x,y)
            E.append(f'<circle cx="{px:.0f}" cy="{py:.0f}" r="9" fill="#ff9614" stroke="#b95f00" stroke-width="2.5"/>')
    if p0 is not None:
        tail=DRAW2D['pts'][-1] if DRAW2D['pts'] else tuple(p0[:2])
        if math.hypot(tail[0]-p1[0],tail[1]-p1[1])>0.005:
            poly(_orthize([tail,(float(p1[0]),float(p1[1]))]),'#e67878',5,'12 9')
        # 출발 = 분홍 (깜빡이는 링으로 강조), 도착 = 빨강
        sx,sy=SX(p0[0],p0[1])
        E.append(f'<circle cx="{sx:.0f}" cy="{sy:.0f}" r="14" fill="none" '
                 'stroke="#ff2d96" stroke-width="4">'
                 '<animate attributeName="r" values="12;24;12" dur="1.6s" repeatCount="indefinite"/>'
                 '<animate attributeName="opacity" values="0.9;0;0.9" dur="1.6s" repeatCount="indefinite"/>'
                 '</circle>')
        E.append(f'<circle cx="{sx:.0f}" cy="{sy:.0f}" r="13" fill="#ff2d96" stroke="#fff" stroke-width="3.5"/>')
        E.append(f'<text x="{sx+17:.0f}" y="{sy+8:.0f}" fill="#e01e82" font-size="23" font-weight="bold">출발 ({a})</text>')
        ex,ey=SX(p1[0],p1[1])
        E.append(f'<circle cx="{ex:.0f}" cy="{ey:.0f}" r="11" fill="#dc3c3c" stroke="#fff" stroke-width="3"/>')
        E.append(f'<text x="{ex+14:.0f}" y="{ey+7:.0f}" fill="#dc3c3c" font-size="21" font-weight="bold">도착 ({b})</text>')
    # 안내
    hint=(f'[{a} → {b}]  클릭=점 추가 · 점선=도착까지 남은 구간(자동 연결)'
          if a and b else
          '클릭=점 추가 · ┃이동 블록을 선택하면 출발/도착점과 연결됩니다')
    E.append(f'<rect x="6" y="{MAP_VB-40}" width="{MAP_VB-12}" height="34" fill="#fffff0" opacity="0.92" stroke="#c8c8a0"/>')
    E.append(f'<text x="16" y="{MAP_VB-16}" fill="#5a5a28" font-size="19">{hint}</text>')
    js=("const r=this.getBoundingClientRect();"
        "const fx=(event.clientX-r.left)/r.width;"
        "const fy=(event.clientY-r.top)/r.height;"
        "const t=document.querySelector('#mapxy textarea,#mapxy input');"
        "if(t){t.value=fx.toFixed(4)+','+fy.toFixed(4);"
        "t.dispatchEvent(new Event('input',{bubbles:true}));}")
    return ('<div style="width:100%;max-width:660px">'
            f'<svg viewBox="0 0 {MAP_VB} {MAP_VB}" '
            'style="width:100%;background:#fdfdfb;border:1.5px solid #bbb;'
            'border-radius:8px;cursor:crosshair;display:block" '
            f'onclick="{js}">'+''.join(E)+'</svg></div>')

def h_map_xy(sel,xystr):
    """SVG 클릭 좌표(0~1 비율) -> 경로 점 추가."""
    try:
        fx,fy=[float(v) for v in str(xystr).split(',')]
    except Exception:
        return gr.update(),gr.update()
    x=G2_MIN+fx*(G2_MAX-G2_MIN)
    y=G2_MIN+(1-fy)*(G2_MAX-G2_MIN)
    return _add_path_pt(sel,x,y)

def _add_path_pt(sel,x,y):
    a,b=_sel_seg(sel)
    snapped_end=False
    if b:                                           # 도착점 2cm 이내 -> 스냅
        pe,_=fk(get_angles(b))
        if math.hypot(x-pe[0],y-pe[1])<0.02:
            x,y=float(pe[0]),float(pe[1]); snapped_end=True
    if not snapped_end:
        prev=None
        if DRAW2D['pts']: prev=DRAW2D['pts'][-1]
        elif a:
            p0,_=fk(get_angles(a)); prev=(p0[0],p0[1])
        if prev is not None:                        # 직교 스냅
            if abs(x-prev[0])>=abs(y-prev[1]): y=prev[1]
            else: x=prev[0]
    DRAW2D['pts'].append((round(float(x),3),round(float(y),3)))
    BTN('맵 클릭',f'점 추가 ({x:.2f},{y:.2f})')
    n=len(DRAW2D['pts'])
    if snapped_end: tail="  · 도착점 연결됨! [경로 저장]을 누르세요"
    else: tail="" if a else "  (저장하려면 ┃이동 블록을 선택하세요)"
    return render_map2d(sel),f"경로 점 {n}개 · 마지막 ({x:.2f},{y:.2f}){tail}"

def h_map_click(sel,evt:gr.SelectData):
    """맵 클릭 -> 경로 점 추가. 블록 미선택이어도 일단 그려진다.
    직전 점(없으면 출발점)에 직교 스냅. 도착점 2cm 이내 클릭 = 도착점에 스냅."""
    a,b=_sel_seg(sel)
    W=MAP_W; sc=W/(G2_MAX-G2_MIN)
    px,py=evt.index
    x=px/sc+G2_MIN; y=(W-py)/sc+G2_MIN
    snapped_end=False
    if b:                                           # 도착점 근처 -> 정확히 스냅
        pe,_=fk(get_angles(b))
        if math.hypot(x-pe[0],y-pe[1])<0.02:
            x,y=float(pe[0]),float(pe[1]); snapped_end=True
    if not snapped_end:
        prev=None
        if DRAW2D['pts']: prev=DRAW2D['pts'][-1]
        elif a:
            p0,_=fk(get_angles(a)); prev=(p0[0],p0[1])
        if prev is not None:                        # 직교 스냅
            if abs(x-prev[0])>=abs(y-prev[1]): y=prev[1]
            else: x=prev[0]
    DRAW2D['pts'].append((round(x,3),round(y,3)))
    BTN('맵 클릭',f'점 추가 ({x:.2f},{y:.2f})')
    n=len(DRAW2D['pts'])
    if snapped_end:
        tail="  · 도착점 연결됨! [경로 저장]을 누르세요"
    else:
        tail="" if a else "  (저장하려면 ┃이동 블록을 선택하세요)"
    return render_map2d(sel),f"경로 점 {n}개 · 마지막 ({x:.2f},{y:.2f}){tail}"

def h_map_undo(sel):
    if DRAW2D['pts']: DRAW2D['pts'].pop()
    return render_map2d(sel),f"점 취소 (남은 {len(DRAW2D['pts'])}개)"

def h_map_clear(sel):
    DRAW2D['pts']=[]
    return render_map2d(sel),"그리던 경로 지움"

def h_path_save(sel):
    """그린 경로를 선택한 ┃이동 블록에 저장."""
    BTN('경로 저장',str(sel))
    a,b=_sel_seg(sel)
    if not a: return gr.update(),render_map2d(sel),"┃이동 블록을 선택하세요"
    if not DRAW2D['pts']:
        return gr.update(),render_map2d(sel),"먼저 맵을 클릭해 경로를 그리세요"
    S.setdefault('paths2d',{})[seg_key(a,b)]=[list(p) for p in DRAW2D['pts']]
    DRAW2D['pts']=[]
    _clear_plan_cache()
    try:
        with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    except Exception: pass
    return (gr.update(choices=seq_labels(),value=None),render_map2d(sel),
            f"[{a} → {b}] 사용자 경로 저장됨")

def h_path_del(sel):
    BTN('경로 삭제',str(sel))
    a,b=_sel_seg(sel)
    if not a: return gr.update(),render_map2d(sel),"┃이동 블록을 선택하세요"
    S.setdefault('paths2d',{}).pop(seg_key(a,b),None)
    _clear_plan_cache()
    try:
        with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    except Exception: pass
    return (gr.update(choices=seq_labels(),value=None),render_map2d(sel),
            f"[{a} → {b}] 사용자 경로 삭제 (자동 경로 사용)")

def h_seg_prefer(sel,prefer):
    """선택 ┃블록에만 x/y 우선 축 저장."""
    BTN('구간 축 저장',f'{sel} -> {prefer}')
    a,b=_sel_seg(sel)
    if not a: return gr.update(),"┃이동 블록을 선택하세요"
    S.setdefault('simple_seg',{})[seg_key(a,b)]={
        'prefer':'x' if prefer.startswith('x') else 'y'}
    _clear_plan_cache()
    try:
        with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    except Exception: pass
    return gr.update(choices=seq_labels(),value=None),f"[{a} → {b}] {prefer} 이동"

def resolve_mode(kind,mode):
    return MODE_SIMPLE if kind=='간편 이동' else mode

# ---- 자세별 무시 장애물 (그 자세가 올라가거나 만지는 장애물) ----
def obstacle_ids():
    return [e['id'] for e in ENV_SHOW if 'plate_bottom' not in e['id'].lower()]

def h_ign_load(sel):
    """■자세 블록 선택 시 그 자세의 무시 목록을 체크박스에 반영."""
    kind,i=seq_index(sel) if sel else (None,None)
    if kind!='pose': return gr.update(choices=obstacle_ids())
    name=S['seq'][i]
    return gr.update(choices=obstacle_ids(),
                     value=S.setdefault('ignore_obs',{}).get(name,[]))

def h_ign_save(sel,checks):
    BTN('무시 장애물 저장',str(sel))
    kind,i=seq_index(sel) if sel else (None,None)
    if kind!='pose':
        return gr.update(),"파이프라인에서 ■ 자세 블록을 먼저 선택하세요"
    name=S['seq'][i]
    S.setdefault('ignore_obs',{})[name]=list(checks or [])
    _clear_plan_cache()
    try:
        with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    except Exception: pass
    return (render_map2d(sel),
            f"'{name}' 무시 장애물: {', '.join(checks) if checks else '없음'}")

# ================= 산업용 테마 (다크/라이트) =================
# 색 규칙:
#   안전 주황 = 실물 로봇이 움직이는 버튼 전용 (편집 버튼과 절대 혼용 금지)
#   강철 파랑 = 편집/저장 등 소프트웨어만 건드리는 primary
#   신호 빨강 = 정지
try:
    THEME=gr.themes.Base(primary_hue='blue',neutral_hue='slate')
except Exception:
    THEME=None

THEME_CSS="""
/* ---------- 라이트 (기본) ---------- */
:root{
  --as-accent:#e8720c; --as-accent-dim:#b85a0a;   /* 안전 주황 */
  --as-stop:#d1322e; --as-blue:#3d6fa3;
  --body-background-fill:#e8eaee;
  --background-fill-primary:#f4f5f7;
  --background-fill-secondary:#e8eaee;
  --block-background-fill:#f4f5f7;
  --block-border-color:#c3c8d2; --border-color-primary:#c3c8d2;
  --input-background-fill:#ffffff;
  --body-text-color:#20242c; --block-label-text-color:#5a6172;
  --button-primary-background-fill:var(--as-blue);
  --button-primary-background-fill-hover:#34608f;
  --button-primary-text-color:#fff;
  --button-cancel-background-fill:var(--as-stop);
  --button-cancel-background-fill-hover:#b52824;
  --button-cancel-text-color:#fff;
  --color-accent:var(--as-blue); --color-accent-soft:#dbe5f0;
}
/* ---------- 다크 (html.dark = gradio 기본 .dark 보다 우선) ---------- */
html.dark, html.dark body{
  --body-background-fill:#14161a;
  --background-fill-primary:#1c1f26;
  --background-fill-secondary:#171a20;
  --block-background-fill:#1c1f26;
  --block-border-color:#343945; --border-color-primary:#343945;
  --input-background-fill:#12151a;
  --body-text-color:#d6dae2; --block-label-text-color:#8f97a8;
  --color-accent-soft:#243447;
}
/* ---------- 헤더 ---------- */
#hdr{align-items:center;border-bottom:2px solid var(--as-accent);
     padding:2px 0 8px;margin-bottom:6px}
#brand{font-family:ui-monospace,Consolas,monospace;font-size:1.3em;
       font-weight:800;letter-spacing:.14em;display:flex;align-items:center;
       gap:10px;color:var(--body-text-color)}
#brand-mark{width:13px;height:13px;background:var(--as-accent);
            display:inline-block}
#brand-sub{font-size:.55em;font-weight:600;letter-spacing:.3em;
           color:var(--block-label-text-color);margin-left:6px;margin-top:5px}
/* ---------- 탭 ---------- */
button[role="tab"]{font-weight:700;letter-spacing:.06em}
button[role="tab"][aria-selected="true"]{
  color:var(--as-accent)!important;border-color:var(--as-accent)!important}
/* ---------- 버튼 위계 ---------- */
/* 실물 동작 = 안전 주황 (하드웨어가 움직임을 한눈에) */
button.act-robot{
  background:linear-gradient(180deg,var(--as-accent),var(--as-accent-dim))!important;
  color:#fff!important;border:1px solid #8a4407!important;
  font-weight:800!important;letter-spacing:.03em}
button.act-robot:hover{filter:brightness(1.1)}
/* 실물 미세 동작(조그/그리퍼/토크) = 주황 외곽선 */
button.act-jog{background:transparent!important;color:var(--as-accent)!important;
  border:1.5px solid var(--as-accent)!important;font-weight:700!important}
button.act-jog:hover{background:rgba(232,114,12,.12)!important}
/* 정지 (gradio 6 은 cancel 변수를 안 쓰므로 직접 칠한다) */
button.act-stop{background:#d1322e!important;color:#fff!important;
  border:1px solid #a02320!important;
  font-weight:800!important;letter-spacing:.1em}
button.act-stop:hover{background:#b52824!important}
/* ---------- 파이프라인 = 진짜 블록 (스크래치처럼 꽉 찬 색) ---------- */
#pipeline label{flex:1 1 100%;display:flex;align-items:center;margin:3px 0;
  padding:12px 14px;border-radius:7px;cursor:pointer;border:none;
  position:relative;font-weight:700;font-size:.95em;transition:box-shadow .12s}
#pipeline input{position:absolute;opacity:0}
/* ■ 자세 = 강철 파랑 꽉 찬 블록 */
#pipeline label.blk-pose{background:var(--as-blue);color:#fff}
#pipeline label.blk-pose span{color:#fff!important}
/* ┃ 이동 = 들여쓰기 + 점선 테두리 + 연결선 */
#pipeline label.blk-move{background:transparent;
  border:1.5px dashed #7c8798;color:var(--body-text-color);
  flex-basis:calc(100% - 38px);margin-left:38px;padding:8px 12px;
  font-weight:600;font-size:.86em}
/* 이동/그리퍼 블록 왼쪽에 위아래 블록을 잇는 세로 연결선 */
#pipeline label.blk-move::before, #pipeline label.blk-grip::before{
  content:'';position:absolute;left:-21px;top:-12px;bottom:-12px;width:3px;
  background:#7c8798;border-radius:2px}
/* ◆ 그리퍼 = 호박색 꽉 찬 블록 */
#pipeline label.blk-grip{background:#d9a012;color:#231a00;
  flex-basis:calc(100% - 38px);margin-left:38px;padding:9px 12px;
  font-size:.88em}
#pipeline label.blk-grip span{color:#231a00!important}
/* 선택 = 안전 주황 링 (채운 블록 위에서도 확실히 보이게) */
#pipeline label.selected{box-shadow:0 0 0 3px var(--as-accent)!important}
/* ---------- 세그먼트 (조인트 선택 등) ---------- */
.segmented label{border:1px solid var(--block-border-color);padding:6px 14px;
  border-radius:4px;font-weight:600;cursor:pointer}
.segmented label.selected{background:var(--color-accent);color:#fff;
  border-color:var(--color-accent)}
.segmented input{position:absolute;opacity:0}
/* ---------- 상태 콘솔 (양쪽 테마 모두 어두운 HMI 콘솔) ---------- */
#statusbox textarea{
  font-family:ui-monospace,'Cascadia Mono',Consolas,monospace!important;
  background:#101318!important;color:#c9d4e3!important;
  border:1px solid #2c3340!important;
  border-left:4px solid var(--as-accent)!important;
  border-radius:4px!important;font-size:.85em!important}
/* ---------- 3D 뷰어 / 2D 맵 ---------- */
#viewer3d{width:100%;height:calc(100vh - 250px);min-height:470px;
  border:1px solid var(--block-border-color);border-radius:6px;
  background:#303030;display:block}
#mapwrap{background:var(--background-fill-primary);
  border:1px solid var(--block-border-color);border-radius:6px;padding:8px}
/* ---------- 정렬 수정 ---------- */
/* 모든 gr.Button 높이/글자 통일 (lg/md/sm 크기 차이 제거) */
button.lg, button.md, button.sm{min-height:44px!important;height:44px!important;
  font-size:.92rem!important;padding-top:0!important;padding-bottom:0!important}
/* 아이콘 버튼 (↻): 빈 회색 바 금지, 작은 정사각 */
button.iconbtn{max-width:44px!important;min-width:44px!important;
  width:44px!important;height:44px!important;padding:0!important;
  font-size:1.05em;align-self:flex-end;flex-grow:0!important}
/* 라벨 있는 입력과 버튼이 한 행일 때: 아래 기준선 정렬 */
.row-end{align-items:flex-end!important}
/* 같은 행 버튼 높이 통일 */
.btnrow{align-items:stretch!important}
/* 칩 라디오 (자세 목록): 동그라미 숨기고 칩으로 */
.chips label{border:1px solid var(--block-border-color);border-radius:5px;
  padding:7px 14px;font-weight:600;cursor:pointer;
  background:var(--background-fill-primary)}
.chips label.selected{background:var(--color-accent);color:#fff;
  border-color:var(--color-accent)}
.chips input{position:absolute;opacity:0}
/* 헤더 토글 (순수 HTML 버튼) */
#themebtn-html{margin-left:auto;background:transparent;cursor:pointer;
  border:1px solid var(--block-border-color);border-radius:5px;
  color:var(--body-text-color);padding:7px 14px;font-weight:600;
  font-size:.85em;letter-spacing:.05em}
#themebtn-html:hover{border-color:var(--as-accent);color:var(--as-accent)}
"""

# 테마 적용/토글 + 파이프라인 블록 분류.
# 주의: gradio 6 은 launch(js=...) 를 실행하지 않으므로 head= 로 주입한다.
HEAD_HTML="""
<script>
(function(){
  function ready(fn){
    if(document.readyState!=='loading') fn();
    else document.addEventListener('DOMContentLoaded',fn);
  }
  ready(function(){
    var apply=function(t){
      document.documentElement.classList.toggle('dark',t==='dark');
      document.body.classList.toggle('dark',t==='dark');
      try{localStorage.setItem('arm_theme',t);}catch(e){}
      var f=document.getElementById('viewer3d');
      if(f&&f.contentWindow) f.contentWindow.postMessage({armTheme:t},'*');
    };
    window._armSendTheme=function(){
      var t=document.documentElement.classList.contains('dark')?'dark':'light';
      var f=document.getElementById('viewer3d');
      if(f&&f.contentWindow) f.contentWindow.postMessage({armTheme:t},'*');
    };
    window._armToggleTheme=function(){
      apply(document.documentElement.classList.contains('dark')?'light':'dark');
    };
    var saved='dark';
    try{saved=localStorage.getItem('arm_theme')||'dark';}catch(e){}
    apply(saved);
    /* 파이프라인 Radio 항목을 기호(■┃◆)로 분류해 블록처럼 칠한다 */
    var classify=function(){
      document.querySelectorAll('#pipeline label').forEach(function(lb){
        var t=(lb.textContent||'').trim();
        var want=t.indexOf('\\u25a0')===0?'blk-pose'
               :(t.indexOf('\\u2503')===0?'blk-move'
               :(t.indexOf('\\u25c6')===0?'blk-grip':''));
        if(want&&!lb.classList.contains(want)) lb.classList.add(want);
      });
    };
    new MutationObserver(classify).observe(document.body,
      {childList:true,subtree:true});
    classify();
  });
})();
</script>
"""

# ================= UI =================
with gr.Blocks(title='ARM STUDIO') as demo:
    with gr.Row(elem_id='hdr'):
        gr.HTML('<div id="brand"><span id="brand-mark"></span>ARM STUDIO'
                '<span id="brand-sub">ROBOT ARM CONTROL</span>'
                '<button id="themebtn-html" '
                'onclick="window._armToggleTheme&&window._armToggleTheme()">'
                '◐ 다크/라이트</button></div>')
    with gr.Row():
        # ================= 왼쪽 사이드바 (자세 / 이동 / 환경) =================
        with gr.Column(scale=3):
            with gr.Tabs():
                # ---- 1. 자세: 미리보기 + 미세조정 + 불러오기 ----
                with gr.Tab('자세'):
                    with gr.Row():
                        pose_sel=gr.Radio(choices=pose_names(),value='base',
                                          label='자세 목록 (클릭 = 3D 미리보기)',
                                          scale=4,elem_classes=['chips'])
                        ft_refresh=gr.Button('↻',scale=0,size='sm',
                                             elem_classes=['iconbtn'])
                    ft_joint=gr.Radio(choices=['J1','J2','J3','J4','J5'],
                                      value='J1',label='조인트 선택',
                                      elem_classes=['segmented'])
                    with gr.Accordion('미세조정 (실물만 보정, 3D는 정답 고정)',
                                      open=True):
                        ft_go=gr.Button('로봇을 이 자세로',variant='primary',
                                        elem_classes=['act-robot'])
                        with gr.Row(elem_classes=['row-end']):
                            ft_step=gr.Dropdown(choices=['0.5','1','2','5'],
                                                value='1',label='스텝(도)',scale=2)
                            ft_minus=gr.Button('−',variant='secondary',scale=1,
                                               elem_classes=['act-jog'])
                            ft_plus=gr.Button('+',variant='secondary',scale=1,
                                              elem_classes=['act-jog'])
                        with gr.Row():
                            ft_gopen=gr.Button('그리퍼 열기',size='sm',
                                               elem_classes=['act-jog'])
                            ft_gclose=gr.Button('그리퍼 닫기',size='sm',
                                                elem_classes=['act-jog'])
                        with gr.Row():
                            ft_apply=gr.Button('base 보정 가져오기',size='sm')
                            ft_reset=gr.Button('보정 초기화',size='sm')
                            ft_savebtn=gr.Button('보정 저장',variant='primary',
                                                 size='sm')
                    with gr.Accordion('URDF / STL 불러오기',open=False):
                        up_kind=gr.Radio(['URDF (자세)','STL (환경)'],
                                         value='URDF (자세)',show_label=False,
                                         elem_classes=['segmented'])
                        up_dir=gr.File(label='URDF 패키지 폴더 드롭',
                                       file_count='directory')
                        up=gr.File(label='STL 파일',
                                   file_types=['.urdf','.stl'],
                                   file_count='multiple',visible=False)
                        with gr.Row():
                            btn_dup=gr.Button('자세 복제',size='sm')
                            btn_del=gr.Button('자세 삭제',size='sm')
                        newname=gr.Textbox(label='새 이름 (복제용)',
                                           placeholder='예: 1st_그리퍼열림')
                        with gr.Row():
                            btn_save=gr.Button('프로젝트 저장',size='sm')
                            btn_load=gr.Button('불러오기',size='sm')
                    # (숨김) 이전 UI 와의 배선 호환용 - 기능은 유지
                    with gr.Group(visible=False):
                        sliders=[]
                        for j in range(5):
                            lo,hi=jr_limits(j)
                            sliders.append(gr.Slider(
                                minimum=round(math.degrees(lo),1),
                                maximum=round(math.degrees(hi),1),
                                value=0.0,step=0.1,label=f'j{j+1}'))
                        grip_open=gr.Button('열 기')
                        grip_close=gr.Button('닫 기')
                        path_in=gr.Textbox(label='폴더 경로')
                        btn_path=gr.Button('경로에서 추가')

                # ---- 2. 이동: 파이프라인 + 간편 이동 ----
                with gr.Tab('이동'):
                    with gr.Row():
                        mv_pose=gr.Dropdown(choices=pose_names(),value='base',
                                            label='추가할 자세',scale=3)
                        mv_refresh=gr.Button('↻',scale=0,size='sm',
                                             elem_classes=['iconbtn'])
                    with gr.Row(elem_classes=['btnrow']):
                        btn_sadd=gr.Button('■ 자세 추가',variant='primary')
                        btn_gadd_open=gr.Button('◆ 그리퍼 열기',size='sm')
                        btn_gadd_close=gr.Button('◆ 그리퍼 닫기',size='sm')
                    gr.Markdown('**파이프라인** (■자세 / ┃이동 / ◆그리퍼)')
                    seq_sel=gr.Radio(choices=seq_labels(),
                                     label='블록 선택',show_label=False,
                                     elem_id='pipeline')
                    with gr.Row():
                        btn_sdel=gr.Button('삭제',size='sm')
                        btn_sup=gr.Button('▲',size='sm')
                        btn_sdn=gr.Button('▼',size='sm')
                    with gr.Accordion('선택한 ■자세가 무시할 장애물',open=False):
                        gr.Markdown('그 자세가 장애물 위로 올라가거나 만져야 하면 '
                                    '체크 → 충돌 경고에서 제외')
                        ign_chk=gr.CheckboxGroup(choices=obstacle_ids(),
                                                 label='체크 = 무시')
                        btn_ign=gr.Button('무시 목록 저장',size='sm')
                    with gr.Row():
                        speed=gr.Dropdown(choices=['0.5x','1.0x','1.5x','2.0x','3.0x','4.0x'],
                                          value='1.5x',label='속도')
                        loop=gr.Checkbox(label='반복',value=False)
                    with gr.Row(elem_classes=['btnrow']):
                        btn_prev=gr.Button('미리보기',variant='secondary')
                        btn_run=gr.Button('실물 실행',variant='primary',
                                          elem_classes=['act-robot'])
                        btn_stop=gr.Button('정지',variant='stop',
                                           elem_classes=['act-stop'])
                    btn_toff=gr.Button('토크 해제',size='sm',
                                       elem_classes=['act-jog'])
                    gr.Markdown('**위에서 본 맵** — ┃이동 블록 선택 후 클릭하면 '
                                '경로 점이 직교로 추가됩니다')
                    map_img=gr.HTML(value='',elem_id='mapwrap')
                    mapxy=gr.Textbox(visible=False,elem_id='mapxy')
                    with gr.Row():
                        btn_mundo=gr.Button('점 취소',size='sm')
                        btn_mclear=gr.Button('전부 지움',size='sm')
                        btn_mref=gr.Button('맵 새로고침',size='sm')
                    with gr.Row():
                        btn_psave=gr.Button('선택한 ┃블록에 경로 저장',
                                            variant='primary',size='sm')
                        btn_pdel=gr.Button('블록 경로 삭제',size='sm')
                    # (숨김) 간편 이동만 사용 - 복잡 이동/세부 설정은 보류
                    with gr.Group(visible=False):
                        movekind=gr.Radio(['간편 이동','복잡 이동'],
                                          value='간편 이동',label='기본 이동 종류')
                        seg_mv=gr.Dropdown(
                            choices=['(기본값 사용)',MODE_SIMPLE]+list(MODES),
                            value='(기본값 사용)',label='블록 이동 방식')
                        btn_segmv=gr.Button('적용')
                        _sp=S.setdefault('simple',{'prefer':'x','clear_cm':5.0,'extra_cm':5.0})
                        sp_prefer=gr.Radio(['x 먼저','y 먼저'],
                                           value=('x 먼저' if _sp.get('prefer','x')=='x' else 'y 먼저'))
                        sp_clear=gr.Slider(1,15,value=float(_sp.get('clear_cm',5)),step=1)
                        sp_extra=gr.Slider(0,15,value=float(_sp.get('extra_cm',5)),step=1)
                        btn_segpref=gr.Button('블록 축 적용')
                        mode=gr.Dropdown(choices=MODES,value=MODES[0])
                        axorder=gr.Dropdown(choices=list(AXIS_ORDERS),value='x-z-y')
                        coll=gr.Checkbox(label='충돌 검사',value=False)
                        seg_ax=gr.Dropdown(
                            choices=['(기본값 사용)']+list(AXIS_ORDERS),
                            value='(기본값 사용)')
                        btn_segax=gr.Button('적용')
                        seg_rot=gr.Dropdown(choices=list(ROT_MODES),value='동시(기본)')
                        btn_segrot=gr.Button('적용')
                        btn_rec=gr.Button('녹화')
                    ft_pose=pose_sel          # 미세조정 대상 = 자세 목록 선택
                    pose_for_seq=mv_pose      # 파이프라인 추가 = 이동 탭 선택

                # ---- 3. 환경 ----
                with gr.Tab('환경'):
                    gr.Markdown('**바닥 플레이트**')
                    plate_edit=gr.Checkbox(
                        label='바닥 확장 모드 (3D에서 초록 타일 클릭)',value=False)
                    plate_info=gr.Textbox(label='현재 배치',value='',lines=2,
                                          interactive=False)
                    with gr.Row():
                        plate_sel=gr.Dropdown(choices=[],label='추가한 플레이트',
                                              scale=2)
                        plate_del=gr.Button('삭제',scale=1,size='sm')
                    plate_ref=gr.Button('목록 새로고침',size='sm')
                    gr.Markdown('**환경 STL**')
                    env_chk=gr.CheckboxGroup(choices=env_ids(),
                                             value=env_visible_ids(),
                                             label='체크 = 표시')

        # ================= 오른쪽: 3D 뷰 + 상태 =================
        with gr.Column(scale=5):
            gr.HTML(f'<iframe id="viewer3d" src="http://localhost:{VIEWER_PORT}/" '
                    'onload="window._armSendTheme&&window._armSendTheme()">'
                    '</iframe>')
            status=gr.Textbox(label='상태',lines=3,elem_id='statusbox',
                              value='준비됨. [자세]에서 미세조정, [이동]에서 실행.')
            with gr.Group(visible=False):        # (숨김) plotly 폴백 출력용
                plot=gr.Plot(value=robot_fig([0.0]*6),label='plotly 3D')

    # 배선
    pose_sel.change(h_select,[pose_sel],[plot]+sliders)
    for sl in sliders:
        sl.release(h_slider,[pose_sel]+sliders,[plot,status])
    grip_open.click(lambda n:h_grip(n,True),[pose_sel],[plot,status])
    grip_close.click(lambda n:h_grip(n,False),[pose_sel],[plot,status])
    up.upload(h_upload,[up],[pose_sel,pose_for_seq,env_chk,status])
    up_dir.upload(h_upload_dir,[up_dir],[pose_sel,pose_for_seq,status])
    btn_path.click(h_add_path,[path_in],[pose_sel,pose_for_seq,status])
    env_chk.change(h_env,[env_chk],[status])
    plate_edit.change(h_plate_edit,[plate_edit],[plate_info])
    plate_del.click(h_plate_remove,[plate_sel],[plate_info,plate_sel])
    plate_ref.click(h_plate_refresh,None,[plate_info,plate_sel])
    demo.load(h_plate_refresh,None,[plate_info,plate_sel])
    def _ft_pick(n):
        FT['pose']=n
        return f"보정 대상: {n}  |  현재 보정: "+_off_str(get_offset(n))
    ft_pose.change(_ft_pick,[ft_pose],[status])
    def _refresh_poses(cur):
        """자세 목록 갱신. 현재 선택이 사라졌으면 첫 항목으로 되돌린다.
        (value 를 안 주면 Radio 가 update dict 를 값으로 물고 있다가 터짐)"""
        names=pose_names()
        val=cur if cur in names else (names[0] if names else None)
        return gr.update(choices=names,value=val)
    ft_refresh.click(_refresh_poses,[pose_sel],[pose_sel])
    ft_go.click(h_ft_goto,[ft_pose],[status])
    ft_minus.click(lambda p,j,s:h_ft_step(p,j,s,-1),
                   [ft_pose,ft_joint,ft_step],[status])
    ft_plus.click(lambda p,j,s:h_ft_step(p,j,s,+1),
                  [ft_pose,ft_joint,ft_step],[status])
    ft_gopen.click(lambda p:h_ft_grip(p,True),[ft_pose],[status])
    ft_gclose.click(lambda p:h_ft_grip(p,False),[ft_pose],[status])
    def _refresh_seq():
        """라벨이 바뀌었을 수 있으므로 choices 와 선택값을 함께 갱신."""
        labs=seq_labels()
        keep=REC.get('last_sel')
        val=keep if keep in labs else None
        return gr.update(choices=labs,value=val)
    btn_rec.click(h_rec_toggle,[seq_sel],[status]).then(
        _refresh_seq,None,[seq_sel])
    ft_apply.click(h_apply_nearest,[ft_pose],[status])
    ft_reset.click(h_ft_reset,[ft_pose],[status])
    ft_savebtn.click(h_ft_save,[ft_pose],[status])
    btn_dup.click(h_duplicate,[pose_sel,newname],[pose_sel,pose_for_seq,status])
    btn_del.click(h_delete,[pose_sel],[pose_sel,pose_for_seq,seq_sel,status])
    btn_sadd.click(h_seq_add,[pose_for_seq],[seq_sel,status])
    btn_gadd_open.click(lambda:h_seq_grip(True),None,[seq_sel,status])
    btn_gadd_close.click(lambda:h_seq_grip(False),None,[seq_sel,status])
    btn_segax.click(h_seg_axorder,[seq_sel,seg_ax],[seq_sel,status])
    btn_segrot.click(h_seg_rotmode,[seq_sel,seg_rot],[seq_sel,status])
    btn_segmv.click(h_seg_mode,[seq_sel,seg_mv],[seq_sel,status])
    btn_sdel.click(h_seq_del,[seq_sel],[seq_sel,status])
    btn_sup.click(lambda s:h_seq_move(s,-1),[seq_sel],[seq_sel,status])
    btn_sdn.click(lambda s:h_seq_move(s,1),[seq_sel],[seq_sel,status])
    btn_prev.click(h_preview,[movekind,mode,axorder,speed,loop,coll],[plot,status])
    btn_run.click(h_run,[movekind,mode,axorder,speed,loop,coll],[plot,status])
    # ---- 간편 이동 (2D 맵) ----
    for _c in (sp_prefer,sp_clear,sp_extra):
        _c.change(h_simple_set,[sp_prefer,sp_clear,sp_extra],[map_img,status])
    mapxy.input(h_map_xy,[seq_sel,mapxy],[map_img,status])
    btn_mundo.click(h_map_undo,[seq_sel],[map_img,status])
    btn_mclear.click(h_map_clear,[seq_sel],[map_img,status])
    btn_mref.click(lambda s:(render_map2d(s),'맵 갱신'),[seq_sel],[map_img,status])
    btn_psave.click(h_path_save,[seq_sel],[seq_sel,map_img,status])
    btn_pdel.click(h_path_del,[seq_sel],[seq_sel,map_img,status])
    btn_segpref.click(h_seg_prefer,[seq_sel,sp_prefer],[seq_sel,status])
    seq_sel.change(render_map2d,[seq_sel],[map_img])
    seq_sel.change(h_ign_load,[seq_sel],[ign_chk])
    btn_ign.click(h_ign_save,[seq_sel,ign_chk],[map_img,status])
    mv_refresh.click(_refresh_poses,[mv_pose],[mv_pose])
    up_kind.change(lambda k:(gr.update(visible=k.startswith('URDF')),
                             gr.update(visible=not k.startswith('URDF'))),
                   [up_kind],[up_dir,up])
    demo.load(render_map2d,None,[map_img])
    btn_stop.click(h_stop,None,[status])
    btn_toff.click(h_torque_off,None,[status])
    btn_save.click(h_save,None,[status])
    btn_load.click(h_load,None,[pose_sel,pose_for_seq,seq_sel,status])

if __name__=='__main__':
    demo.queue(default_concurrency_limit=8)
    # 7860부터 빈 포트 자동 탐색 (이전 실행이 포트를 잡고 있어도 OK)
    for port in range(7860,7880):
        try:
            demo.launch(server_name='0.0.0.0',server_port=port,share=False,
                        inbrowser=False,show_error=True,
                        theme=THEME,css=THEME_CSS,head=HEAD_HTML)
            break
        except OSError:
            print(f"port {port} busy, trying {port+1}...")
