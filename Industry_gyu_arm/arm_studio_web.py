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
# ===== USB 자동 인식 =====
# /dev/ttyACM0 같은 번호는 뽑았다 꽂으면 1, 2 로 밀린다(usbip 재연결도 마찬가지).
# 대신 /dev/serial/by-id/ 에는 칩 종류와 시리얼번호가 박힌 고정 경로가 있어
# 번호가 바뀌어도 늘 같은 장치를 가리킨다. 그걸로 어느 로봇인지 자동 판별한다.
#   SO-ARM(STS3215) : CH343 칩  -> by-id 에 '1a86' 또는 'CH343'
#   Dynamixel(U2D2) : FTDI 칩   -> by-id 에 'FTDI'
# 환경변수 ARM_STS_PORT / ARM_DXL_PORT 로 수동 지정도 가능.
BYID_DIR='/dev/serial/by-id'

def scan_serial():
    """연결된 시리얼 어댑터를 종류별로 찾는다. {'sts':경로,'dxl':경로}"""
    found={'sts':None,'dxl':None}
    try: names=sorted(os.listdir(BYID_DIR))
    except Exception: names=[]
    for n in names:
        p=os.path.join(BYID_DIR,n); low=n.lower()
        if ('ftdi' in low or '0403' in low) and not found['dxl']:
            found['dxl']=p
        elif ('1a86' in low or 'ch343' in low or 'ch340' in low) and not found['sts']:
            found['sts']=p
    # by-id 가 없는 환경(예: 일부 SBC)에서는 예전 방식으로 폴백
    if not found['sts']:
        for c in ('/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2'):
            if os.path.exists(c): found['sts']=c; break
    if not found['dxl']:
        for c in ('/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2'):
            if os.path.exists(c): found['dxl']=c; break
    return found

def port_of(rk):
    """그 로봇의 현재 포트. 매번 스캔하므로 도중에 다시 꽂아도 따라간다."""
    env=os.environ.get('ARM_STS_PORT' if rk=='sts' else 'ARM_DXL_PORT')
    if env: return env
    return scan_serial().get(rk)

_SCAN0=scan_serial()
PORT=_SCAN0['sts'] or '/dev/ttyACM0'    # Feetech STS3215 (SO-ARM101)
BAUD=1000000
print(f"USB 자동 인식: SO-ARM={PORT}")
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
# ===== 툴 중심 (axis_7) =====
# 사용자가 SolidWorks 에서 그리퍼 회전 원의 중심에 잡은 좌표계.
# 2026-08-09 URDF(20260809_base)의 j7 에서 읽은 값. j1~j6/메쉬는 이전 URDF 와
# 완전히 동일함을 확인했으므로(차이 0), URDF 는 그대로 두고 이 값만 쓴다.
#   -> j7 이 들어있는 URDF 를 base 로 쓰면 그쪽 값이 자동으로 우선한다.
TOOL_OFF_DEFAULT=np.array([-0.009, 0.0395, 0.0393351262037889])
def _tool_offset():
    try:
        root=ET.parse(BASE_URDF).getroot()
        for j in root.findall('joint'):
            if j.get('name')=='j7':
                o=j.find('origin')
                v=np.array([float(x) for x in (o.get('xyz') or '0 0 0').split()])
                if np.linalg.norm(v)>1e-9: return v
    except Exception: pass
    return TOOL_OFF_DEFAULT.copy()
TOOL_OFF=_tool_offset()          # l6(=j6 프레임) 기준
TOOL_HAS=bool(np.linalg.norm(TOOL_OFF)>1e-9)
if TOOL_HAS:
    print(f"툴 중심(axis_7): j6 에서 {np.round(TOOL_OFF*1000,1)} mm")

def tool_pos(q):
    """월드 기준 툴 중심 좌표 (axis_7). j6 프레임에서 TOOL_OFF 만큼 이동."""
    T=CHAIN.forward_kinematics([0]+list(q))
    return T[:3,3]+T[:3,:3]@TOOL_OFF+BASE_OFF
# ===== 로봇 베이스 위치 (플레이트 홈 스냅) =====
# 로봇을 플레이트 위 원하는 타일에 재배치할 수 있다.
#   - 내부 기구학(CHAIN)은 항상 로봇 로컬 좌표로 푼다
#   - fk 결과는 월드로 (+BASE_OFF), IK 목표는 로컬로 (-BASE_OFF) 변환
#   -> 나머지 코드(맵/충돌/간편이동)는 전부 월드 좌표라 수정 불필요
BASE_OFF=np.zeros(3)          # 월드 기준 로봇 base 오프셋 (z 는 항상 0)
def _base_T():
    T=np.eye(4); T[:3,3]=BASE_OFF; return T
def fk(q):
    T=CHAIN.forward_kinematics([0]+list(q))
    return T[:3,3]+BASE_OFF, T[:3,:3].copy()
def fk_points(q):
    frames=CHAIN.forward_kinematics([0]+list(q),full_kinematics=True)
    return np.array([f[:3,3] for f in frames])+BASE_OFF

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
        sol=CHAIN.inverse_kinematics(target_position=pt-BASE_OFF,target_orientation=zt,
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
        sol=CHAIN.inverse_kinematics(target_position=pos-BASE_OFF,target_orientation=zt,
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
            sol=CHAIN.inverse_kinematics(target_position=pt-BASE_OFF,target_orientation=zt,
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

# ===== Dynamixel 의 '수평' 기준 =====
# SO-ARM 은 j2+j3+j4 합으로 기울기를 계산한다(위 level_offset). 그런데 DXL 은
# 축 구성이 달라(수평축이 j1,j2,j3,j5) 그 수식이 맞지 않는다.
# 대신 사용자가 손으로 물컵 규칙에 맞게 잡아둔 base 자세를 기준으로 쓴다.
#   -> base 를 다시 잡으면 이 기준도 자동으로 따라간다 (코드 수정 불필요)
def dxl_level_axes():
    """DXL 에서 '수평을 만드는' 관절 인덱스들 (URDF 축 구조에서 자동 판별)."""
    if not DXL_OK: return []
    try:
        lvl=_level_mask(DXL_DEF)
        return [k for k in range(6) if lvl[k]]
    except Exception:
        return []

def dxl_base_pose():
    """DXL 관절 공간에서의 base 자세. 영점이 곧 base 이므로 전부 0.
    (dxl_zero.py 를 손으로 다시 잡으면 그 자세가 새 기준이 된다)"""
    return [0.0]*6

def dxl_apply_level(qd):
    """DXL 자세에 '수평 유지' 를 적용한다.

    수평을 깨는 축(수평 그룹에 속하지 않는 회전축)은 base 값으로 고정한다.
    SO-ARM 의 j5 고정과 같은 개념이지만, 어느 축인지는 DXL URDF 에서 판별한다.
    """
    if not qd: return qd
    q=list(qd); base=dxl_base_pose(); lv=set(dxl_level_axes())
    for k in range(5):
        if k not in lv: q[k]=base[k]      # 수평을 깨는 축 -> base 로 고정
    return q

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
            sol=CHAIN.inverse_kinematics(target_position=pt-BASE_OFF,
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

# ===== Dynamixel(옛 로봇) 체인 + STS<->DXL EE 매핑 =====
# 원리: 두 로봇이 공유하는 것은 관절각이 아니라 "EE 경로"다.
#   - 각 로봇의 base 자세(q=0)를 기준으로 한 EE 변위를 (앞,위) 축 정렬로 옮긴다
#   - 자세(수평)는 각 로봇이 자기 base 자세의 기울기를 그대로 유지
#   주의: DXL URDF 는 SolidWorks 특성상 y-up (j1 축이 ±Y), STS 는 z-up
DXL_OK=False
try:
    import glob as _g
    from dxl_profile import (PORT as DXL_PORT, BAUD as DXL_BAUD,
        J2ID as DXL_J2ID, DIR as DXL_DIR, ZERO_RAW as DXL_ZERO,
        LIMIT_RAW as DXL_LIMIT, ZERO_VERIFIED as DXL_ZERO_VERIFIED)
    _c=_g.glob(os.path.join(URDF_ROOT,'dynamixel','*base*','urdf','*.urdf'))
    if _c:
        DXL_URDF=_c[0]; DXL_DEF=_parse_urdf(DXL_URDF); DXL_OK=True
        print(f"Dynamixel URDF: {DXL_URDF}")
        if not DXL_ZERO_VERIFIED:
            print("  !! DXL 영점 미검증 - 실행 전 dxl_capture_zero.py 필수 !!")
except Exception as _e:
    print(f"Dynamixel 프로필 로드 실패 (기능 비활성): {_e}")

def dxl_jr_limits(jidx):
    i=DXL_J2ID[jidx]; lo,hi=DXL_LIMIT[i]; z=DXL_ZERO[i]; d=DXL_DIR[i]
    a=d*(lo-z)*math.pi/2048.0; b=d*(hi-z)*math.pi/2048.0
    return (min(a,b),max(a,b))
def dxl_rad_to_raw(jidx,rad):
    i=DXL_J2ID[jidx]
    raw=int(DXL_ZERO[i]+DXL_DIR[i]*rad*2048.0/math.pi)
    lo,hi=DXL_LIMIT[i]; return max(lo,min(hi,raw)), i

def _world_axes(defs):
    """q=0 에서 각 조인트 회전축의 베이스(월드) 좌표계 방향."""
    R=np.eye(3); out=[]
    for jn in JOINTS:
        b=defs[jn]
        R=R@_R(b['rpy'])
        a=R@b['axis']; out.append(a/np.linalg.norm(a))
    return out

def _level_mask(defs):
    """수평 유지 4축 선택을 로봇 구조에서 자동 판별.
    yaw(수직축과 평행) 1개 + 서로 평행한 pitch(수직에 직교) 최대 그룹.
    (STS: j1+j2j3j4 / DXL: j1+j2j3j5 - SolidWorks 네이밍이 꼬여 있어도 동작)"""
    ax=_world_axes(defs)
    up=np.abs(ax[0])                       # j1 = 베이스 요 축이 수직
    yaw=[k for k in range(5) if abs(np.dot(ax[k],up))>0.99]
    pitch=[k for k in range(5) if abs(np.dot(ax[k],up))<0.1]
    best=[]
    for k in pitch:
        grp=[g for g in pitch if abs(np.dot(ax[k],ax[g]))>0.99]
        if len(grp)>len(best): best=grp
    mask=[False]*6
    for k in yaw[:1]+best: mask[k]=True
    return mask

_DXL_CHAINS={}
def _dxl_chain(level=True):
    if not DXL_OK: return None
    key='L' if level else 'F'
    if key not in _DXL_CHAINS:
        lvl=_level_mask(DXL_DEF)
        links=[OriginLink()]; active=[False]
        for k,jn in enumerate(JOINTS):
            b=DXL_DEF[jn]
            links.append(URDFLink(name=jn,origin_translation=list(b['xyz']),
                                  origin_orientation=list(b['rpy']),
                                  rotation=list(b['axis']),
                                  bounds=dxl_jr_limits(k),joint_type='revolute'))
            active.append(lvl[k] if level else True)
        _DXL_CHAINS[key]=Chain(name='dxl'+key,links=links,active_links_mask=active)
        if level: print(f"DXL 수평 체인 활성 관절: "
                        f"{[JOINTS[k] for k in range(6) if lvl[k]]}")
    return _DXL_CHAINS[key]

def _chain_ee(chain,q):
    T=chain.forward_kinematics([0]+list(q)); return T[:3,3].copy(),T[:3,:3].copy()

# 로봇별 프레임 파라미터: base 자세(q=0)에서 자동 산출
#   up   : 수직축 (j1 회전축 방향, EE 가 위에 있는 쪽)
#   p0   : base 자세의 EE 위치 (로컬)
#   emode: EE 프레임에서 수직과 정렬된 축 ('X'/'Y'/'Z') -> ikpy orientation_mode
#   evec : 그 축이 향해야 하는 방향 (base 자세 그대로 유지)
#   M    : (앞,옆,위) -> 로컬 좌표 회전행렬 (변위 변환용)
def _frame_params(chain,j1_axis):
    p0,R0=_chain_ee(chain,[0.0]*6)
    up=np.abs(np.asarray(j1_axis,float)); up=up/np.linalg.norm(up)
    if np.dot(p0,up)<0: up=-up                    # EE 가 있는 쪽 = 위
    k=int(np.argmax([abs(np.dot(R0[:,c],up)) for c in range(3)]))
    emode='XYZ'[k]
    evec=R0[:,k].copy()                           # base 자세의 방향 그대로 유지
    h=p0-np.dot(p0,up)*up
    n=np.linalg.norm(h)
    fwd=h/n if n>1e-6 else np.array([1.,0.,0.])
    side=np.cross(up,fwd)
    M=np.column_stack([fwd,side,up])              # (앞,옆,위) -> 로컬
    return {'p0':p0,'up':up,'emode':emode,'evec':evec,'M':M}

_FRM={'sts':None,'dxl':None}
def _frames_init():
    if _FRM['sts'] is None:
        _FRM['sts']=_frame_params(CHAIN,BASE_DEF['j1']['axis'])
    if DXL_OK and _FRM['dxl'] is None:
        _FRM['dxl']=_frame_params(_dxl_chain(False),DXL_DEF['j1']['axis'])

_MAP_LAST={'s2d':None,'d2s':None}
def _map_pair(q_src,src):
    """src 로봇 관절 -> 같은 'base 기준 EE 변위'를 만드는 상대 로봇 관절.
    실패 시 None (해당 웨이포인트는 상대 로봇 유지)."""
    if not DXL_OK: return None
    _frames_init()
    dst='dxl' if src=='sts' else 'sts'
    key='s2d' if src=='sts' else 'd2s'
    cs=CHAIN if src=='sts' else _dxl_chain(False)
    cd=_dxl_chain(True) if dst=='dxl' else _chain_level()
    fs,fd=_FRM[src],_FRM[dst]
    p,_=_chain_ee(cs,q_src)
    d_local=fs['M'].T@(p-fs['p0'])               # (앞,옆,위) 변위
    pt=fd['p0']+fd['M']@d_local                  # 상대 로봇 로컬 목표
    seeds=[]
    if _MAP_LAST[key] is not None: seeds.append(list(_MAP_LAST[key]))
    seeds.append([0.0]*6)
    for s in seeds:
        try:
            sol=cd.inverse_kinematics(target_position=pt,
                target_orientation=fd['evec'],
                orientation_mode=fd['emode'],
                initial_position=[0.0]+list(s))
        except Exception:
            continue
        qd=list(sol[1:7])
        pc,Rc=_chain_ee(cd,qd)
        kc='XYZ'.index(fd['emode'])
        if (np.linalg.norm(pc-pt)<0.004
                and np.dot(Rc[:,kc],fd['evec'])>0.999):
            _MAP_LAST[key]=qd
            return qd
    return None

def map_sts_to_dxl(q): return _map_pair(q,'sts')
def map_dxl_to_sts(q): return _map_pair(q,'dxl')

# ===== 해석적 IK (닫힌 해) =====
# 수치 솔버는 "각도를 조금씩 바꿔보며 목표에 다가가는" 반복이라 느리다.
# 이 로봇은 구조가 정해져 있으므로 공식으로 한 번에 풀 수 있다:
#   j1(Z축) = 방위만 결정  /  j2,j3,j4(X축 평행) = 그 평면 안의 (거리,높이)
#   수평 유지 -> j2+j3+j4 합이 고정 -> 미지수 2개, 식 2개 = 2링크 역기구학
# 준비: q=0 에서 평면 팔의 링크 길이와 오프셋을 실측해 둔다.
_AN={'ok':False}
def _analytic_setup():
    if _AN.get('done'): return _AN['ok']
    _AN['done']=True
    try:
        ax=_world_axes(BASE_DEF)
        up=np.array([0.,0.,1.])
        if abs(np.dot(ax[0],up))<0.99: return False      # j1 이 Z 축이 아님
        pl=[k for k in (1,2,3) if abs(np.dot(ax[k],up))<0.01]
        if len(pl)!=3: return False                      # j2,j3,j4 평면 아님
        if not (abs(np.dot(ax[1],ax[2]))>0.99 and abs(np.dot(ax[1],ax[3]))>0.99):
            return False                                 # 세 축이 평행해야 함
        fr=CHAIN.forward_kinematics([0]*7,full_kinematics=True)
        p=[f[:3,3] for f in fr]
        # j1 축 위치(월드 xy) 와 각 조인트의 상대 위치
        _AN['o1']=p[1][:2].copy()                        # j1 회전축의 xy
        _AN['z1']=float(p[2][2])                         # 어깨(j2) 높이
        # j2->j3, j3->j4 링크 길이 (평면 팔 2링크)
        _AN['L1']=float(np.linalg.norm(p[3]-p[2]))
        _AN['L2']=float(np.linalg.norm(p[4]-p[3]))
        # j4 이후 손목~툴까지는 수평 유지 상태에서 고정 벡터가 된다.
        # q=0 기준 j4 원점 -> 툴 중심
        T6=fr[6]
        tool0=T6[:3,3]+T6[:3,:3]@TOOL_OFF
        _AN['w']=tool0-p[4]                              # j4 -> 툴 (q=0)
        # 어깨 기준 평면 좌표 정의를 위한 전방 방향(q=0 에서 j2->j4 투영)
        _AN['ok']=True
        return True
    except Exception:
        return False

def ik_tool_fast(pt,j5,j6):
    """해석적 IK. 수평유지 + 툴 중심 기준. 실패하면 None (호출측이 수치로 폴백).
    반복 없음 - 공식 한 번."""
    if not _analytic_setup(): return None
    tgt=np.asarray(pt,float)-BASE_OFF
    o1=_AN['o1']; d=tgt[:2]-o1
    r=float(np.linalg.norm(d))
    if r<1e-6: return None
    # ① j1: 툴이 j1 축에서 벗어나 있으므로 그만큼 보정한 방위각
    #    수평 유지 상태에서 j4->툴 벡터의 '옆(측면)' 성분이 곧 오프셋
    w=_AN['w']; side=float(w[0])                # q=0 에서 x 가 측면 방향
    if abs(side)>=r: return None                # 축에 너무 가까움 - 해 없음
    j1=math.atan2(d[1],d[0])-math.atan2(side,math.sqrt(r*r-side*side))
    j1-=math.atan2(1.0,0.0)                     # q=0 기준(+y 전방) 보정
    # ② 평면 2링크: 어깨에서 (수평거리 R, 높이 H) 인 손목점 구하기
    fwd=math.sqrt(max(r*r-side*side,0.0))       # 전방 성분
    R=fwd-float(w[1]); H=tgt[2]-_AN['z1']-float(w[2])
    L1,L2=_AN['L1'],_AN['L2']
    D=math.hypot(R,H)
    if D>L1+L2 or D<abs(L1-L2): return None     # 도달 불가
    c=(L1*L1+L2*L2-D*D)/(2*L1*L2)
    c=max(-1.0,min(1.0,c))
    beta=math.acos(c)                           # 팔꿈치 내각
    a=math.atan2(H,R)+math.acos(
        max(-1.0,min(1.0,(L1*L1+D*D-L2*L2)/(2*L1*D))))
    j2=a; j3=beta-math.pi
    j4=-(j2+j3)                                 # 합=0 -> 수평 유지
    q=[j1,j2,j3,j4,j5,j6]
    # 검산: 공식이 맞았는지 실제 FK 로 확인 (틀리면 폴백)
    if np.linalg.norm(tool_pos(q)-np.asarray(pt,float))>0.001: return None
    for k in range(5):
        lo,hi=jr_limits(k)
        if not (lo-1e-6<=q[k]<=hi+1e-6): return None
    return q

def ik_tool(pt,seeds,j5,j6,iters=6):
    """툴 중심(axis_7)이 pt 에 오도록 푸는 수평유지 IK.

    IK 는 j6 원점을 푸므로 목표에서 툴 오프셋을 빼야 하는데, 그 오프셋의
    월드 방향이 j1 회전에 따라 달라진다(z 성분만 일정). 그래서 한 번에
    못 빼고, '풀고 -> 실제 툴 위치 오차만큼 목표를 밀어주고' 를 반복해
    수렴시킨다. 보통 2~3회면 0.1mm 이내.
    """
    tgt=np.asarray(pt,float)
    if not TOOL_HAS: return ik_level(tgt,seeds,j5,j6)
    aim=tgt.copy(); best=None
    for _ in range(iters):
        q=ik_level(aim,seeds,j5,j6)
        if q is None: return None
        err=tgt-tool_pos(q)
        best=q
        # 반복마다 오차가 ~7배씩 줄어든다(56mm -> 1.2 -> 0.16 -> 0.02 ...).
        # 0.01mm 까지 조여도 4ms 밖에 안 늘어나므로 여기서 멈춘다.
        if np.linalg.norm(err)<0.00001: break
        aim=aim+err
        seeds=[q]+list(seeds)                    # 다음 반복은 이 해에서 출발
    return best

def ik_level(pt,seeds,j5,j6):
    """수평 유지 IK (j5/j6 잠금). 초기값 여러 개 시도 -> 관절6개 or None.
    (초기값이 나쁘면 도달 가능한 점도 실패 판정이 나므로 다중 시도 필수)"""
    ch=_chain_level(); tgt=np.asarray(pt,float)
    for s in seeds:
        init=[0.0]+list(s); init[5]=j5; init[6]=j6
        try:
            sol=ch.inverse_kinematics(target_position=tgt-BASE_OFF,
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
def _fill_mesh2d(g,path,scale,off=(0.0,0.0)):
    """메시를 위에서 본 모양으로 격자 g 에 채운다. off=월드 평행이동(x,y)."""
    try: mm=trimesh.load(path)
    except Exception: return
    v=np.asarray(mm.vertices,float)*scale
    v[:,0]+=off[0]; v[:,1]+=off[1]
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
    if os.path.exists(bp): _fill_mesh2d(gb,bp,1.0,off=(BASE_OFF[0],BASE_OFF[1]))
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
            pt=port_of('sts') or PORT       # 재연결로 번호가 밀려도 따라감
            p=PortHandler(pt)
            if not p.openPort(): return False
            p.setBaudRate(BAUD)
            globals()['PORT']=pt
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
    def send(self,q,log=False,tag='',with_grip=False,only=None):
        """관절 목표 전송.
        그리퍼(j6)는 기본적으로 '보내지 않는다' -> URDF/자세 값과 무관하게
        ◆열기/닫기 블록으로만 바뀌고 그 상태를 유지한다.
        with_grip=True 일 때만 j6 도 함께 보낸다(그리퍼 블록 전용).
        only=[관절인덱스] 면 그 관절에만 보낸다(미세조정 - 한 축만 움직이기)."""
        raws={}
        for jidx in range(6):
            if jidx==5 and not with_grip: continue   # j6 제외
            if only is not None and jidx not in only: continue
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
ROBOT_STS=Robot()

# Dynamixel 이동 시간(ms). 값이 클수록 느리다.
# 기본 600 은 실물이 너무 빨라 1200 으로 = 속도 절반 (사용자 요청)
DXL_MOVE_MS=1200
DXL_STEP_SLOW=2.0          # 웨이포인트 간 대기도 이 배수만큼 늘린다

class DXLRobot:
    """옛 Dynamixel 로봇 (XC/XM430, Protocol 2.0, U2D2).
    j4<->j5 배선 교차/방향/영점은 dxl_profile 에서. 관절값은 DXL 로컬 rad."""
    A_TORQUE=64; A_GOAL=116; A_PRESENT=132
    A_PVEL=112; A_PACC=108; A_DRIVE=10
    # 이동 시간(ms). 크면 느리다. 600 -> 1200 = 속도 절반
    def __init__(self):
        self.port=None; self.packet=None; self.lock=threading.RLock()
    def connect(self):
        if not DXL_OK: return False
        if self.port: return True
        try:
            from dynamixel_sdk import PortHandler,PacketHandler
            pt=port_of('dxl') or DXL_PORT
            p=PortHandler(pt)
            if not p.openPort(): return False
            p.setBaudRate(DXL_BAUD)
            globals()['DXL_PORT']=pt
            pk=PacketHandler(2.0)
            _,r,_=pk.ping(p,DXL_J2ID[0])
            if r!=0:
                p.closePort(); return False
            self.port=p; self.packet=pk; return True
        except Exception:
            return False
    def _w1(self,i,a,v):
        with self.lock:
            try: return self.packet.write1ByteTxRx(self.port,i,a,int(v))[0]==0
            except Exception: return False
    def _w4(self,i,a,v):
        with self.lock:
            try:
                return self.packet.write4ByteTxRx(
                    self.port,i,a,int(v)&0xFFFFFFFF)[0]==0
            except Exception: return False
    def _r4(self,i,a):
        with self.lock:
            try:
                v,r,_=self.packet.read4ByteTxRx(self.port,i,a)
                return s32(v) if r==0 else None
            except Exception: return None
    def _r1(self,i,a):
        with self.lock:
            try:
                v,r,_=self.packet.read1ByteTxRx(self.port,i,a)
                return v if r==0 else None
            except Exception: return None
    _prof=None                       # 이미 설정된 프로파일 (재설정 방지)
    def time_profile(self,on,sp=1.0):
        """Drive Mode bit2=1 시간기반 프로파일 (덜덜거림 방지, 이전 실측).

        !! Drive Mode 는 토크가 켜져 있으면 못 바꾼다 -> 잠깐 토크를 꺼야 하는데,
        그 순간 팔이 중력으로 떨어진다(실제로 바닥에 부딪힌 사고 발생).
        그래서 값이 실제로 바뀔 때만 건드리고, 끈 토크는 즉시 되살린다.
        """
        key=(bool(on),round(float(sp),3))
        if self._prof==key: return          # 이미 같은 설정 - 토크 건드리지 않음
        was_on=False
        try:                                # 현재 토크 상태 확인
            v=self._r1(DXL_J2ID[0],self.A_TORQUE)
            was_on=bool(v)
        except Exception: pass
        for i in set(DXL_J2ID):
            self._w1(i,self.A_TORQUE,0)
            self._w1(i,self.A_DRIVE,4 if on else 0)
        if was_on:                          # 켜져 있었으면 곧바로 복구
            for i in set(DXL_J2ID): self._w1(i,self.A_TORQUE,1)
        self._prof=key
        # 시간기반 프로파일: ms 가 클수록 느리다.
        # 기본 600ms 는 실물이 너무 빨라 1200ms 로 (= 속도 절반, 사용자 요청)
        ms=int(DXL_MOVE_MS/max(sp,0.1))
        for i in set(DXL_J2ID):
            self._w4(i,self.A_PVEL,ms if on else 0)
            self._w4(i,self.A_PACC,ms//3 if on else 0)
    def torque(self,on):
        for i in set(DXL_J2ID): self._w1(i,self.A_TORQUE,1 if on else 0)
    def send(self,q_dxl,log=False,tag='',only=None):
        """only=[관절인덱스] 를 주면 그 관절에만 명령을 보낸다.
        미세조정에서 한 축만 움직이려면 나머지에는 아예 명령을 보내면 안 된다
        (같은 값이라도 명령을 받으면 미세하게 움직인다)."""
        raws={}
        for jidx in range(5):                  # 그리퍼(j6) 자동제어 안 함(값 미측정)
            if only is not None and jidx not in only: continue
            raw,i=dxl_rad_to_raw(jidx,q_dxl[jidx])
            self._w4(i,self.A_GOAL,raw); raws[f'D{i}']=raw
        if log: LOG_TARGET(raws,tag)
        return raws
    def read(self):
        q=[0.0]*6
        for jidx in range(6):
            i=DXL_J2ID[jidx]
            p=self._r4(i,self.A_PRESENT)
            if p is not None: q[jidx]=DXL_DIR[i]*(p-DXL_ZERO[i])*math.pi/2048.0
        return q
    def estop(self):
        try: self.torque(False)
        except Exception: pass
ROBOT_DXL=DXLRobot()

# ===== 실행 로봇 선택 파사드 =====
# 기존 핸들러들은 전부 ROBOT.* 를 부른다. q 는 항상 STS 관절 공간이고,
# DXL 쪽은 send 순간에 EE 매핑(map_sts_to_dxl)으로 변환된다.
EXEC={'mode':'sts','miss':0}
class RobotMux:
    # mode='sim' = 시뮬레이션. 실물 통신을 전부 건너뛰고 3D/RViz 만 움직인다.
    #   -> _sts()/_dxl() 이 모두 False 가 되므로 아래 메서드들이 자동으로 무동작
    def _sim(self): return EXEC['mode']=='sim'
    def _sts(self): return EXEC['mode'] in ('sts','both')
    def _dxl(self): return EXEC['mode'] in ('dxl','both') and DXL_OK
    def connect(self):
        if self._sim(): return True          # 시뮬은 항상 '연결됨'
        ok=True
        if self._sts(): ok=ROBOT_STS.connect() and ok
        if self._dxl(): ok=ROBOT_DXL.connect() and ok
        return ok
    def time_profile(self,on,sp=1.0):
        if self._sts(): ROBOT_STS.time_profile(on,sp)
        if self._dxl(): ROBOT_DXL.time_profile(on,sp)
    def torque(self,on):
        if self._sts(): ROBOT_STS.torque(on)
        if self._dxl(): ROBOT_DXL.torque(on)
    def send(self,q,log=False,tag='',with_grip=False):
        if self._dxl():
            qd=map_sts_to_dxl(q)
            if qd is not None:
                # DXL 의 수평 기준은 사용자가 손으로 잡은 base 자세다
                # (SO-ARM 의 j2+j3+j4 합 수식은 DXL 축 구성에 맞지 않음)
                ROBOT_DXL.send(dxl_apply_level(qd),log=log,tag=tag)
            else:
                EXEC['miss']+=1                # 도달 불가: DXL 은 그 자리 유지
                if EXEC['miss']%25==1:
                    print(f"[DXL] 도달 불가 웨이포인트 스킵 (누적 {EXEC['miss']})"
                          " - 영점 재캡처/한계 재측정 후 다시 평가 필요")
        if self._sts():
            return ROBOT_STS.send(q,log=log,tag=tag,with_grip=with_grip)
        return {}
    def read(self):
        if self._sim(): return list(CURRENT_Q)   # 시뮬: 3D 뷰의 현재 자세
        if self._sts(): return ROBOT_STS.read()
        m=map_dxl_to_sts(ROBOT_DXL.read())
        return m if m is not None else list(CURRENT_Q)
    def wait_reach(self,q,tol=25,timeout=6.0,ids=None):
        if self._sts(): return ROBOT_STS.wait_reach(q,tol,timeout,ids)
        time.sleep(0.05); return True,0        # DXL 단독: 시간 페이싱
    def settle(self,q,tol=END_TOL,timeout=END_TIMEOUT,retry=4):
        if self._sts(): return ROBOT_STS.settle(q,tol,timeout,retry)
        time.sleep(END_SETTLE); return True,0
    def gripper(self,open_,tq=None):
        if self._sim(): time.sleep(0.3); return None
        if self._sts(): return ROBOT_STS.gripper(open_,tq)
        return None                            # DXL 그리퍼 값 미측정
    def estop(self):
        if self._sim(): return
        ROBOT_STS.estop()
        if DXL_OK: ROBOT_DXL.estop()           # 정지는 모드 무관 전체
    # 추종 비교 로그(LOG_COMPARE)가 읽는 값. STS 가 wait_reach 에서 채운다.
    # 시뮬/DXL 단독일 때는 비어 있어 로그가 그냥 생략된다.
    @property
    def last_want(self): return ROBOT_STS.last_want
    @property
    def last_got(self): return ROBOT_STS.last_got
    @property
    def port(self):
        """열려 있는 포트가 하나라도 있으면 참 (종료 시 안전정지 판단용)."""
        return ROBOT_STS.port or (ROBOT_DXL.port if DXL_OK else None)
    def _w1(self,*a): return ROBOT_STS._w1(*a)
    def _w2(self,*a): return ROBOT_STS._w2(*a)
    def _r2(self,*a): return ROBOT_STS._r2(*a)
ROBOT=RobotMux()
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
# ===== 보정값은 '자세 × 로봇' 마다 따로 =====
# 보정값 = URDF 이론값과 실물의 차이. 로봇이 다르면 조립오차·링크휨이 달라
# 같은 자세라도 값이 달라야 한다.
#   저장: p['offsets'] = {'sts':[...], 'dxl':[...]}
#   예전 형식 p['offset'] 은 STS 로 보고 자동 이관한다.
# 보정 대상 로봇은 [자세] 탭에서 직접 고른다.
# 실행 로봇(EXEC)과는 별개다 - 실물이 연결돼 있지 않아도 보정값을 보고
# 고칠 수 있어야 하고, "지금 실행할 로봇" 과 "지금 보정하는 로봇" 은 다른 개념.
CAL={'robot':'sts'}

def offset_robot():
    """지금 보정 대상 로봇 키 ('sts' | 'dxl')."""
    return CAL.get('robot','sts')

def robot_njoints(rk=None):
    """그 로봇의 조정 가능한 관절 수를 URDF 에서 읽는다 (그리퍼 제외).

    로봇마다 축 수가 다르므로 하드코딩하지 않는다. URDF 의 revolute 조인트를
    세고, 마지막 하나는 그리퍼(열기/닫기만)로 보아 제외한다.
    """
    rk=rk or offset_robot()
    try:
        path=DXL_URDF if (rk=='dxl' and DXL_OK) else BASE_URDF
        root=ET.parse(path).getroot()
        n=sum(1 for j in root.findall('joint')
              if (j.get('type') or '').lower() in ('revolute','continuous'))
        return max(1,n-1)                 # 마지막 = 그리퍼
    except Exception:
        return 5

def _pose_rec(name):
    for p in S['poses']:
        if p['name']==name: return p
    return None

def get_offset(name,rk=None):
    p=_pose_rec(name)
    if not p: return [0.0]*6
    rk=rk or offset_robot()
    d=p.get('offsets')
    if d is None:                      # 예전 형식 -> STS 보정으로 이관
        d={'sts':list(p.get('offset',[0.0]*6))}
        p['offsets']=d
    return list(d.get(rk,[0.0]*6))

def set_offset(name,off,rk=None):
    p=_pose_rec(name)
    if not p: return
    rk=rk or offset_robot()
    d=p.setdefault('offsets',{'sts':list(p.get('offset',[0.0]*6))})
    d[rk]=list(off)
    if rk=='sts': p['offset']=list(off)     # 구버전 호환 필드도 갱신
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
# 물체 잡기/놓기 블록. '@grasp:<물체>' / '@release:<도킹>'
GRASP_PREFIX='@grasp:'; RELEASE_PREFIX='@release:'
def is_grasp_item(n): return isinstance(n,str) and n.startswith(GRASP_PREFIX)
def is_release_item(n): return isinstance(n,str) and n.startswith(RELEASE_PREFIX)
def is_brick_item(n): return is_grasp_item(n) or is_release_item(n)
def is_grip_item(x):
    # '@' 로 시작하는 다른 블록(@grasp:/@release:)도 생겼으므로
    # 그리퍼 블록은 정확히 이 두 값일 때만이다.
    return x in (GRIP_OPEN_ITEM,GRIP_CLOSE_ITEM)
def pose_seq():
    """시퀀스에서 자세 항목만 (경로 계획용).
    그리퍼·물체 블록은 자세가 아니므로 제외한다."""
    return [x for x in S['seq']
            if not is_grip_item(x) and not is_brick_item(x)]

def seq_labels(mode_name=None):
    """파이프라인 블록: ■자세 / ┃이동 / ◆그리퍼"""
    out=[]; mv=get_moves()
    prev_pose=None; seg_i=0
    for idx,n in enumerate(S['seq']):
        if is_grip_item(n):
            act='열기 ✋' if n==GRIP_OPEN_ITEM else '닫기 ✊'
            out.append(f"◆ {idx}. 그리퍼 {act}")
            continue
        if is_brick_item(n):
            # 물체 블록은 자세 사이에 끼어들 뿐 '이동 구간' 을 만들지 않는다
            if is_grasp_item(n):
                out.append(f"◈ {idx}. 잡기: {brick_label(n[len(GRASP_PREFIX):])}")
            else:
                out.append(f"◈ {idx}. 놓기: {dock_label(n[len(RELEASE_PREFIX):])}")
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
    fr=[f.copy() for f in
        CHAIN.forward_kinematics([0]+list(q),full_kinematics=True)]
    for f in fr: f[:3,3]+=BASE_OFF
    return fr

def robot_fig(q):
    fig=go.Figure()
    for t in _ENV_TRACES: fig.add_trace(t)
    frames=link_frames(q)
    # frames[0]=base_link(원점), frames[k]=조인트k 이후 = l_k 프레임
    for k,(v,f) in enumerate(_LINK_MESH):
        T=_base_T() if k==0 else frames[k]
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
    if on: ROBOT_MOVE['on']=False          # 두 클릭 모드 동시 금지
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
    """파이프라인에 자세 블록 추가. 추가한 자세를 3D 로 바로 보여준다."""
    if not name: return gr.update(),"자세를 선택하세요"
    with LOCK: S['seq'].append(name)
    if not RUNNING.is_set():
        _peg_sim_stop()                  # 목심 미리보기가 돌고 있으면 양보
        ros_publish(get_angles(name))    # 방금 추가한 자세 미리보기
    return (gr.update(choices=seq_labels(),value=None),
            f"'{name}' 추가 (3D 미리보기) — 시퀀스: {' -> '.join(S['seq'])}")

def h_seq_grasp(bid):
    """파이프라인에 잡기 블록 추가."""
    if not bid: return gr.update(),"물체를 선택하세요"
    with LOCK: S['seq'].append(GRASP_PREFIX+bid)
    _macro_save()
    return gr.update(choices=seq_labels(),value=None), \
           f"◈ 잡기: {brick_label(bid)} 블록 추가"

def h_seq_release(did):
    """파이프라인에 놓기 블록 추가."""
    if not did: return gr.update(),"도킹을 선택하세요"
    with LOCK: S['seq'].append(RELEASE_PREFIX+did)
    _macro_save()
    return gr.update(choices=seq_labels(),value=None), \
           f"◈ 놓기: {dock_label(did)} 블록 추가"

def h_set_name(oid,name,color):
    """물체/도킹에 이름과 색을 지정."""
    BTN('이름 지정',f'{oid} -> {name}')
    if not oid:
        return gr.update(),gr.update(),gr.update(),"대상을 선택하세요"
    set_name(oid,name,color)
    labs=[f"{disp_name(o)}  ({o})" for o in list(BRICKS)+env_ids()]
    return (gr.update(choices=list(BRICKS)+env_ids(),value=oid),
            gr.update(choices=list(BRICKS),
                      value=(oid if oid in BRICKS else None)),
            gr.update(choices=docking_ids(),
                      value=(oid if oid in docking_ids() else None)),
            f"'{disp_name(oid)}' 로 지정 (색 {color})")

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
    if len(ps)<2:
        # 자세가 1개여도 잡기/놓기 블록만으로 이뤄진 파이프라인은 유효하다
        # (그 블록들이 스스로 이동까지 하므로 자세 사이 경로가 필요 없다)
        if any(is_brick_item(x) for x in S['seq']):
            return [],None,""
        return None,"시퀀스에 자세가 2개 이상 필요합니다",""
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
                              if not is_grip_item(x) and not is_brick_item(x))
            items=[x for i,x in enumerate(S['seq']) if i!=first_pose_i]
            for item in items:
                if STOP.is_set(): raise KeyboardInterrupt
                if is_brick_item(item):
                    # 잡기/놓기 블록: 상승 -> 축 일치 -> 하강 -> 그리퍼 순으로
                    # 자체 완결된 동작이라 경로 계획과 별개로 실행한다.
                    RUNNING.clear()            # do_grasp 가 자체적으로 잡는다
                    if is_grasp_item(item):
                        b=item[len(GRASP_PREFIX):]
                        LOG(f"\n[잡기] {brick_label(b)}")
                        msg=do_grasp(b)
                    else:
                        d=item[len(RELEASE_PREFIX):]
                        LOG(f"\n[놓기] {dock_label(d)}")
                        msg=do_release(d)
                    RUNNING.set()
                    LOG(f"  {msg}")
                    yield gr.update(),msg
                    if '오류' in msg or '없습니다' in msg or '불가' in msg:
                        raise RuntimeError(msg)
                    continue
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
ROBOT_LABEL={'sts':'SO-ARM','dxl':'Dynamixel','both':'SO-ARM'}
_CAL_KEYS={'SO-ARM (STS3215)':'sts','Dynamixel':'dxl'}
def _off_str(off,rk=None):
    rk=rk or offset_robot()
    n=robot_njoints(rk)
    body='  '.join(f"j{j+1}:{math.degrees(o):+.1f}" for j,o in enumerate(off[:n]))
    return f"[{ROBOT_LABEL.get(rk,rk)}] "+body
def h_ft_goto(name):
    """[자세] 탭의 '로봇을 이 자세로'.
    [이동] 탭의 실행 로봇이 아니라 이 탭에서 고른 '보정할 로봇' 으로 보낸다
    (같은 탭의 미세조정 +/- 와 같은 로봇이어야 보정 작업이 성립한다)."""
    if not name: return "왼쪽 [자세] 탭에서 자세를 먼저 선택하세요"
    if RUNNING.is_set(): return "다른 동작 실행 중 - [정지] 후 다시"
    rk=offset_robot(); dev=ROBOT_DXL if rk=='dxl' else ROBOT_STS
    if rk=='dxl' and not DXL_OK: return "Dynamixel 프로필이 없습니다"
    if not dev.connect():
        return (f"{ROBOT_LABEL.get(rk)} 연결 실패 — 확인: "
                "1) USB/전원  2) 다른 프로그램이 포트를 쓰는 중인지")
    STOP.clear(); RUNNING.set()
    try:
        q0=list(CURRENT_Q)                     # 3D 기준 자세에서 출발
        # 보정값을 그대로 쓴다. level_offset 은 j2/j3 에 준 값을 j4 로 상쇄해
        # '하나 눌렀는데 둘이 움직이는' 현상을 만든다 -> 보정 작업에는 쓰지 않는다
        off=get_offset(name)
        tgt=[a+o for a,o in zip(get_angles(name),off)]
        tgt[5]=q0[5]                           # 그리퍼는 현재 상태 유지
        if LEVEL_ALWAYS: tgt[J5_IDX]=q0[J5_IDX]
        way,dt=plan_joint_seq(q0,tgt)
        for f in way: f[5]=q0[5]
        if LEVEL_ALWAYS: lock_j5(way,q0)
        ros_publish(get_angles(name))          # 3D 뷰 = 정답 자세
        # DXL 은 모터 프로파일이 느리므로(1200ms) 전송 간격도 함께 늘린다.
        # 안 그러면 명령만 앞질러 쌓이고 실제 동작은 뚝뚝 끊긴다.
        step=dt*(DXL_STEP_SLOW if rk=='dxl' else 1.0)
        for q in way:
            if STOP.is_set(): break
            e=_cal_send(q)                     # 고른 로봇에만 전송
            if e: return e
            time.sleep(step)
        FT['pose']=name
        return (f"[{ROBOT_LABEL.get(rk)}] '{name}' 도착. "
                "관절 선택 후 +/- 로 미세조정하세요. 현재 보정: "+_off_str(off))
    except Exception as e:
        dev.estop(); return f"오류: {e} - 토크 해제됨"
    finally:
        RUNNING.clear()
def _cal_send(q_sts,direct=False,only=None):
    """보정 대상 로봇에 자세를 보낸다 (실행 로봇 설정과 무관).

    direct=True 면 관절값을 '그 로봇의 관절로 그대로' 보낸다.
      미세조정은 관절 하나만 건드려야 하는데, EE 매핑(map_sts_to_dxl)을 거치면
      IK 가 모든 관절을 다시 풀어 여러 축이 함께 움직인다 -> 미세조정 불가.
    direct=False 는 '이 자세로 이동' 처럼 EE 를 맞춰야 할 때 쓴다.
    """
    rk=offset_robot()
    if rk=='dxl':
        if not DXL_OK or not ROBOT_DXL.connect(): return "Dynamixel 연결 실패"
        qd=list(q_sts) if direct else map_sts_to_dxl(q_sts)
        if qd is None: return "그 자세는 Dynamixel 로 옮길 수 없습니다"
        ROBOT_DXL.time_profile(True); ROBOT_DXL.torque(True)
        ROBOT_DXL.send(qd,only=only)
        return None
    if not ROBOT_STS.connect(): return f"SO-ARM 연결 실패 ({PORT})"
    ROBOT_STS.time_profile(True); ROBOT_STS.torque(True)
    ROBOT_STS.send(q_sts,only=only)
    return None

def h_ft_step(pose_name,joint_label,step_deg,sign):
    """[로봇 이동] 없이도 바로 보정 가능. 선택된 자세를 그대로 사용."""
    name=FT['pose'] or pose_name
    if not name: return "왼쪽 [자세] 탭에서 자세를 먼저 선택하세요"
    FT['pose']=name
    j=int(joint_label[1])-1
    off=get_offset(name)
    off[j]+=math.radians(float(step_deg))*sign
    set_offset(name,off)
    tgt=[a+o for a,o in zip(get_angles(name),off)]
    try:
        # 미세조정은 고른 관절 하나만 움직여야 한다 -> direct 전송
        # (EE 매핑을 거치면 IK 가 전 관절을 다시 풀어 여러 축이 함께 움직인다)
        e=_cal_send(tgt,direct=True,only=[j])   # 고른 관절 하나만
        if e: return e
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
    rk=offset_robot()                       # 이 탭에서 고른 로봇으로
    if rk=='dxl':
        return (f"[{name}] 그리퍼 {lbl} -> 자세에 저장 "
                "(Dynamixel 그리퍼는 열림/닫힘 값이 미측정이라 실물은 동작 안 함)")
    if ROBOT_STS.connect():
        try:
            ROBOT_STS.gripper(open_)
            raw=GRIPPER_OPEN if open_ else GRIPPER_CLOSE
            return f"[SO-ARM] [{name}] 그리퍼 {lbl} -> 자세에 저장 (실물 {raw})"
        except Exception as e:
            return f"[{name}] 그리퍼 {lbl} 저장됨 (실물 실패: {e})"
    return f"[{name}] 그리퍼 {lbl} -> 자세에 저장 (SO-ARM 미연결)"

def h_ft_reset(pose_name):
    name=FT['pose'] or pose_name
    if not name: return "자세를 먼저 선택하세요"
    set_offset(name,[0.0]*6)
    try: _cal_send(get_angles(name),direct=True)
    except Exception: pass
    return f"'{name}' {ROBOT_LABEL.get(offset_robot())} 보정 초기화 (실물 = 정답 자세로)"
def h_ft_save(pose_name):
    name=FT['pose'] or pose_name
    if not name: return "자세를 먼저 선택하세요"
    with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    return f"'{name}' 보정 저장됨: "+_off_str(get_offset(name))+"  (그 로봇에만 적용)"

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

# ===== 실행 로봇 선택 / 연결 상태 =====
def _robot_status_str():
    if EXEC['mode']=='sim':
        return "시뮬레이션 모드 — 실물은 움직이지 않고 3D/RViz 만 재생됩니다"
    sc=scan_serial()
    def short(p): return os.path.basename(p) if p else '어댑터 없음'
    s='● 연결됨' if ROBOT_STS.connect() else '○ 미연결'
    out=f"SO-ARM [{short(sc['sts'])}]: {s}"
    if DXL_OK:
        d='● 연결됨' if ROBOT_DXL.connect() else '○ 미연결'
        out+=f"\nDynamixel [{short(sc['dxl'])}]: {d}"
        if not DXL_ZERO_VERIFIED:
            out+="  [경고: 영점 미검증]"
    else:
        out+="\nDynamixel: 프로필/URDF 없음"
    if not sc['sts'] and not sc['dxl']:
        out+="\n어댑터가 하나도 안 보입니다 — usbipd attach 후 다시 [확인]"
    return out

_EXEC_LABELS={'시뮬레이션 (로봇 없이)':'sim','SO-ARM (STS)':'sts',
              'Dynamixel':'dxl','둘 다 (동시)':'both'}
def h_exec_robot(sel):
    """실행 로봇 변경. 보정값도 그 로봇 것으로 바뀌므로 미세조정 UI 도 갱신."""
    BTN(f'실행로봇 {sel}')
    EXEC['mode']=_EXEC_LABELS.get(sel,'sts'); EXEC['miss']=0
    warn=''
    if EXEC['mode'] in ('dxl','both') and DXL_OK and not DXL_ZERO_VERIFIED:
        warn='\n!! DXL 영점 미검증 상태 - 실물 실행 전 dxl_capture_zero.py 필수 !!'
    return f"실행 로봇: {sel}{warn}\n{_robot_status_str()}"
def h_robot_status():
    BTN('연결 확인')
    return _robot_status_str()

def h_cal_torque_off(rlabel):
    """보정 대상 로봇의 토크를 풀어 손으로 자세를 잡을 수 있게 한다."""
    rk=_CAL_KEYS.get(rlabel,'sts'); CAL['robot']=rk
    BTN('토크 해제',rlabel)
    if rk=='dxl':
        if not DXL_OK: return "Dynamixel 프로필이 없습니다"
        if not ROBOT_DXL.connect():
            return f"Dynamixel 연결 실패 ({DXL_PORT}) — usbipd 연결/전원 확인"
        ROBOT_DXL.torque(False)
    else:
        if not ROBOT_STS.connect():
            return f"SO-ARM 연결 실패 ({PORT}) — USB/전원, 포트 점유 확인"
        ROBOT_STS.torque(False)
    return (f"{ROBOT_LABEL.get(rk)} 토크 해제 — 손으로 base 자세를 잡으세요.\n"
            "  * 그리퍼가 지면과 수평이 되게 (물컵 규칙의 기준 자세)\n"
            "  자세를 잡았으면 [이 자세를 base 로 저장] 을 누르세요")

def h_cal_capture_zero(rlabel):
    """지금 손으로 잡아둔 자세를 그 로봇의 영점(base)으로 저장."""
    rk=_CAL_KEYS.get(rlabel,'sts'); CAL['robot']=rk
    BTN('base 저장',rlabel)
    here=os.path.dirname(os.path.abspath(__file__))
    if rk=='dxl':
        if not DXL_OK: return "Dynamixel 프로필이 없습니다"
        if not ROBOT_DXL.connect(): return f"Dynamixel 연결 실패 ({DXL_PORT})"
        ids=sorted(set(DXL_J2ID)); rd=lambda i: ROBOT_DXL._r4(i,ROBOT_DXL.A_PRESENT)
        path=os.path.join(here,'dxl_zero.py'); title='Dynamixel'
    else:
        if not ROBOT_STS.connect(): return f"SO-ARM 연결 실패 ({PORT})"
        ids=sorted(set(J2ID)); rd=lambda i: ROBOT_STS._r2(i,ADDR_PRESENT)
        path=os.path.join(here,'sts_zero.py'); title='SO-ARM'
    vals={}
    for i in ids:
        got=[]
        for _ in range(30):
            v=rd(i)
            if v is not None: got.append(int(v))
            if len(got)>=5: break
            time.sleep(0.04)
        if not got: return f"ID{i} 읽기 실패 - 배선/전원 확인"
        got.sort(); vals[i]=got[len(got)//2]        # 중앙값 (노이즈 제거)
    # 덮어쓰기 전에 이전 값을 백업해 둔다 (잘못 저장했을 때 되돌리기용)
    try:
        if os.path.exists(path):
            import shutil
            shutil.copy2(path,path+'.bak')
    except Exception: pass
    with open(path,'w') as f:
        f.write(f"# {title} 영점 raw - 웹에서 손으로 잡아 캡처\n")
        f.write(f"# 캡처: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"ZERO_RAW={vals}\n")
        f.write("VERIFIED=True\n")
    rng=[]; nj=robot_njoints(rk)
    if rk=='dxl':
        DXL_ZERO.clear(); DXL_ZERO.update(vals); _DXL_CHAINS.clear()
        for k in range(nj):
            lo,hi=dxl_jr_limits(k)
            rng.append(f"j{k+1} {math.degrees(lo):+.0f}~{math.degrees(hi):+.0f}")
    else:
        ZERO_RAW.clear(); ZERO_RAW.update(vals)
        globals()['CHAIN']=build_chain(); globals()['_CHAIN_L']=None
        _AN['done']=False                            # 해석 파라미터 재산출
        for k in range(nj):
            lo,hi=jr_limits(k)
            rng.append(f"j{k+1} {math.degrees(lo):+.0f}~{math.degrees(hi):+.0f}")
    return (f"{title} base 저장됨 ({os.path.basename(path)})\n  "
            +'  '.join(f"ID{i}:{v}" for i,v in vals.items())
            +"\n  가동범위: "+'  '.join(rng)
            +"\n  * 범위가 좌우 대칭에 가까우면 기계적 중립에 잘 앉은 것입니다")

def h_cal_robot(rlabel):
    """보정 대상 로봇 변경 -> 미세조정 축 수와 현재 보정값 표시를 갱신."""
    rk=_CAL_KEYS.get(rlabel,'sts'); CAL['robot']=rk
    BTN('보정 대상',rlabel)
    n=robot_njoints(rk); jl=[f'J{i+1}' for i in range(n)]
    name=FT.get('pose')
    cur=("  현재 보정: "+_off_str(get_offset(name),rk)) if name else ""
    return (gr.update(choices=jl,value=jl[0]),
            f"보정 대상: {ROBOT_LABEL.get(rk)} (J1~J{n}) — "
            f"보정값은 로봇마다 따로 저장됩니다\n{cur}")

# ===== 매크로 버튼 1·2·3 (좌표 기록 -> 원버튼 재생) =====
# 웹 버전 먼저. 나중에 물리 버튼(아두이노/RP2040/조이스틱)이 생겨도
# 같은 h_macro_go(slot) 를 부르면 되도록 입력과 동작을 분리해 둔다.
MACRO_SLOTS=['1','2','3']

def _macro_btn_label(slot):
    m=S.get('macros',{}).get(slot)
    return f"▶ M{slot} · {m['name']}" if m else f"▶ M{slot} (빈)"

def _macro_btn_updates():
    return [gr.update(value=_macro_btn_label(s)) for s in MACRO_SLOTS]

def _macro_save():
    with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)

def h_macro_rec(slot):
    """현재 위치를 슬롯에 기록. 실물 연결 시 실물 좌표, 아니면 3D 자세."""
    BTN(f'매크로{slot} 기록')
    slot=str(slot)
    if ROBOT.connect():
        q=ROBOT.read(); src='실물'
    else:
        q=list(CURRENT_Q); src='3D(시뮬)'
    name=time.strftime('%H:%M')
    with LOCK:
        S.setdefault('macros',{})[slot]={'name':name,'angles':list(q),'src':src}
    _macro_save()
    deg=', '.join(f'{math.degrees(a):+.0f}' for a in q[:5])
    return _macro_btn_updates()+[f"M{slot} 기록됨 ({src}, {name}) [{deg}]"]

def h_macro_clear(slot):
    BTN(f'매크로{slot} 비움')
    slot=str(slot)
    with LOCK: S.get('macros',{}).pop(slot,None)
    _macro_save()
    return _macro_btn_updates()+[f"M{slot} 비움"]

def h_macro_go(slot):
    """기록된 좌표로 이동. 수평 유지(j5 고정)·그리퍼 상태 보존 = 물컵 규칙."""
    BTN(f'매크로{slot} 실행')
    slot=str(slot)
    m=S.get('macros',{}).get(slot)
    if not m: return f"M{slot} 이 비어있습니다 - 먼저 [현재 위치 기록]을 하세요"
    if RUNNING.is_set(): return "다른 동작 실행 중 - [정지] 후 다시"
    if not ROBOT.connect():
        return (f"로봇 연결 실패 ({PORT}) — 확인: "
                "1) USB/전원 연결  2) 다른 프로그램이 포트를 쓰는 중인지")
    STOP.clear(); RUNNING.set()
    try:
        ROBOT.time_profile(True); ROBOT.torque(True)
        q0=ROBOT.read()
        tgt=list(m['angles'])
        tgt[5]=q0[5]                           # 그리퍼는 이동 중 상태 불변
        if LEVEL_ALWAYS: tgt[J5_IDX]=q0[J5_IDX]   # j5 고정 = 수평 유지
        way,dt=plan_joint_seq(q0,tgt)
        for f in way: f[5]=q0[5]
        if LEVEL_ALWAYS: lock_j5(way,q0)
        ros_publish(tgt)
        for q in way:
            if STOP.is_set(): break
            ROBOT.send(q,log=True,tag=f'M{slot}'); time.sleep(dt)
        return f"M{slot} ({m['name']}) 도착"
    except Exception as e:
        ROBOT.estop(); return f"오류: {e} - 토크 해제됨"
    finally:
        RUNNING.clear()

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
    S['macros']=_d.get('macros',{})
    if _d.get('robot_xy'): S['robot_xy']=_d['robot_xy']
    S['names']=_d.get('names',{})
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

# ===== 로봇 위치 이동 (플레이트 타일에 스냅) =====
ROBOT_MOVE={'on':False}

def _robot_pos_str():
    return f"x={BASE_OFF[0]*1000:+.0f}mm  y={BASE_OFF[1]*1000:+.0f}mm"

def set_robot_xy(x,y,save=True):
    """로봇 base 를 월드 (x,y) 로 이동. 캐시/격자 전부 무효화."""
    BASE_OFF[0]=float(x); BASE_OFF[1]=float(y)
    _MAXZ_R.clear(); _clear_plan_cache()
    _OBS['grid']=None                      # 다음 사용 때 재구축
    if save:
        with LOCK: S['robot_xy']=[float(x),float(y)]
        _macro_save()

def robot_to_tile(t):
    """타일 중앙으로 로봇 이동. 플레이트가 있는 타일만 허용."""
    if t not in PLATES: return False
    if RUNNING.is_set(): return False      # 동작 중 이동 금지
    ox,oy=tile_origin(t)
    set_robot_xy(ox+PLATE_SIZE/2, oy+PLATE_SIZE/2)
    print(f"[로봇 위치] 타일 {t} 중앙으로 -> {_robot_pos_str()}")
    return True

# 시작 시 저장된 로봇 위치 복원
if S.get('robot_xy'):
    set_robot_xy(S['robot_xy'][0],S['robot_xy'][1],save=False)
    print(f"로봇 위치 복원: {_robot_pos_str()}")

# ===== 공간 좌표 (목심 격자) =====
# 바닥 플레이트의 목심(구멍) 중심을 격자 좌표로 삼는다.
#   열=A,B,C... (x)  행=1,2,3... (y)  높이=mm
# STL 자기상관으로 측정한 실제 간격 = 20mm, 플레이트(200mm)당 10x10
PEG_PITCH=0.020
# 높이 입력의 기준면 = 바닥 플레이트 윗면.
#   사용자가 말하는 "높이 80mm" = 플레이트 윗면에서 j6 축 원점까지 80mm
#   (IK 가 푸는 점이 j6 축 원점이므로 그 점이 기준. 그리퍼 손가락 끝이 아님)
PLATE_TOP=0.020        # 플레이트 두께 = 윗면의 절대 z (STL 실측)
def peg_z(height_mm): return PLATE_TOP+float(height_mm)/1000.0
def peg_cols(): return [chr(65+i) for i in range(10)]   # A~J (타일 1장 기준)
def peg_names():
    """현재 배치된 플레이트 전체의 목심 이름 목록. 타일마다 A1~J10."""
    out=[]
    for t in sorted(PLATES.keys()):
        for i in range(10):
            for j in range(10):
                out.append(f"{t[0]},{t[1]}:{chr(65+i)}{j+1}")
    return out
def peg_xy(name):
    """'0,0:C4' -> 월드 (x,y). 형식이 틀리면 None."""
    try:
        tp,cell=name.split(':')
        ix,iy=(int(v) for v in tp.split(','))
        col=ord(cell[0].upper())-65; row=int(cell[1:])-1
        ox,oy=tile_origin((ix,iy))
        return ox+PEG_PITCH/2+col*PEG_PITCH, oy+PEG_PITCH/2+row*PEG_PITCH
    except Exception:
        return None

# --- 도달성 계산 가속 ---
# IK 는 수치 최적화라 GPU 가 못 돕는다. 대신 세 가지로 줄인다:
#   1) 팔이 닿을 수 없는 거리는 IK 없이 즉시 탈락 (도달 불가가 IK 2배로 느림)
#   2) 남은 것만 코어 수만큼 병렬 (코어 수는 실행 시점에 자동 감지)
#   3) (높이, 로봇위치) 캐시
def _worker_count():
    """가용 CPU 로 워커 수 결정. PC 를 바꿔도 자동 적응.
    ARM_IK_WORKERS 로 수동 지정 가능(0=자동)."""
    env=int(os.environ.get('ARM_IK_WORKERS','0') or 0)
    if env>0: return env
    try: n=len(os.sched_getaffinity(0))      # 실제 할당된 코어
    except AttributeError: n=os.cpu_count() or 4
    return max(1,min(32,n-1))                # 1개는 UI 응답용으로 남김

_ARM_REACH=None
def _arm_reach():
    """base 원점에서 EE 까지 최대 도달 반경(대략). 링크 길이 합."""
    global _ARM_REACH
    if _ARM_REACH is None:
        s=0.0
        for jn in JOINTS: s+=float(np.linalg.norm(BASE_DEF[jn]['xyz']))
        _ARM_REACH=s
    return _ARM_REACH

def reach_ring(z):
    """그 높이에서 툴이 닿을 수 있는 수평거리 구간 (min,max). 없으면 None.

    평면 팔(j2,j3,j4) 구조라 어깨에서 손목까지는 L1+L2 안이어야 한다.
    -> 3D IK 를 돌리기 전에 (x,y) 만으로 대부분을 걸러낼 수 있다.
    (링크 길이 단순 합은 765mm 지만 실제 도달은 190~360mm 수준)
    """
    if not _analytic_setup(): return None
    L1,L2,w,z1=_AN['L1'],_AN['L2'],_AN['w'],_AN['z1']
    H=z-z1-float(w[2])                      # 어깨 기준 손목 높이
    lo=abs(L1-L2); hi=L1+L2
    if abs(H)>hi: return None               # 그 높이 자체가 팔 밖
    # 손목의 수평거리 R 이 sqrt(D^2-H^2) 형태로 가능한 범위
    Rmax=math.sqrt(max(hi*hi-H*H,0.0))
    Rmin=math.sqrt(max(lo*lo-H*H,0.0)) if abs(H)<lo else 0.0
    fwd=float(w[1]); side=float(w[0])       # 손목->툴 (전방, 측면)
    # 툴까지의 수평거리 = sqrt((R+fwd)^2 + side^2)
    def tot(R): return math.hypot(R+fwd,side)
    return (tot(Rmin),tot(Rmax))

def _peg_ik_ok(args):
    nm,x,y,z,seeds,j5,j6=args
    return nm if ik_tool((x,y,z),seeds,j5,j6) is not None else None

# 프로세스 워커용 - fork 로 모듈 상태를 물려받아 그대로 계산한다
# (spawn 방식 OS 라면 ProcessPool 이 실패하고 순차로 폴백됨)
def _peg_ik_worker(a):
    nm,x,y,z=a
    q0=list(CURRENT_Q)
    return nm if ik_tool((x,y,z),_seed_list(q0),q0[4],q0[5]) is not None else None

# ===== 목심 도달 구간 표 (한 번 만들고 계속 조회) =====
# 각 목심의 도달 가능 높이가 '연속 구간' 임을 실측으로 확인했다(중간 끊김 0).
# 그래서 목심마다 [zmin,zmax] 두 값만 저장하면 어떤 높이든 즉시 판정된다.
#   -> 1mm 는 물론 소수점 높이까지 조회 가능(해상도 제한 없음)
# 표는 (로봇위치, URDF) 가 바뀌면 무효화된다.
PEG_H_MAX=250          # 표가 담는 최대 높이(mm). 슬라이더 상한과 같아야 한다
PEG_TABLE={'key':None,'span':{}}
_PEG_SCAN={'busy':False,'msg':''}

def _peg_span_one(args):
    """한 목심의 도달 가능 높이 구간들 [(a,b),...] (mm).

    주의: 도달 영역이 항상 하나의 연속 구간은 아니다. 관절 한계 때문에
    중간이 끊기는 목심이 실제로 존재한다(예: 0~0 가능, 2~38 불가, 40~232 가능).
    그래서 하나의 [min,max] 가 아니라 '구간 목록' 으로 저장한다.
    1mm 전수 검사가 가장 확실하고, 목심당 251회면 병렬로 충분히 빠르다.
    """
    nm,x,y=args
    q0=list(CURRENT_Q); seeds=_seed_list(q0)
    hits=[h for h in range(0,PEG_H_MAX+1)
          if ik_tool((x,y,peg_z(h)),seeds,q0[4],q0[5]) is not None]
    if not hits: return (nm,None)
    spans=[]; a=p=hits[0]
    for h in hits[1:]:
        if h==p+1: p=h; continue
        spans.append((a,p)); a=p=h
    spans.append((a,p))
    return (nm,spans)

def peg_table_key():
    return (round(BASE_OFF[0],4),round(BASE_OFF[1],4),BASE_URDF,
            tuple(sorted(PLATES.keys())))

def build_peg_table():
    """모든 목심의 도달 높이 구간을 계산해 표로 만든다 (약 7~15초)."""
    key=peg_table_key()
    jobs=[(nm,*peg_xy(nm)) for nm in peg_names() if peg_xy(nm)]
    t0=time.time()
    try:
        import multiprocessing as _mp
        ctx=_mp.get_context('fork')
        with ctx.Pool(_worker_count()) as pool:
            res=pool.map(_peg_span_one,jobs,chunksize=4)
    except Exception as e:
        print(f"[목심표] 병렬 실패({type(e).__name__}) - 순차")
        res=[_peg_span_one(j) for j in jobs]
    span={nm:v for nm,v in res if v}
    PEG_TABLE['key']=key; PEG_TABLE['span']=span
    el=time.time()-t0
    print(f"[목심표] {len(span)}/{len(jobs)}개 도달 가능, {el:.1f}초")
    try:                                   # 다음 실행에서 바로 쓰도록 저장
        with open(peg_table_file(),'w') as f:
            json.dump({'key':[key[0],key[1],key[2],[list(t) for t in key[3]]],
                       'max_h':PEG_H_MAX,'pitch':PEG_PITCH,
                       'built':time.strftime('%Y-%m-%d %H:%M:%S'),
                       'span':{nm:[list(s) for s in sp]
                               for nm,sp in span.items()}},f)
        print(f"[목심표] 저장: {peg_table_file()}")
    except Exception as e:
        print(f"[목심표] 저장 실패: {e}")
    return span,el

def peg_table_file():
    return os.path.join(URDF_PKG_DIR,'peg_table.json')

def load_peg_table():
    """peg_table_build.py 가 미리 만들어 둔 표를 불러온다.
    로봇 위치/URDF/플레이트 배치가 그때와 다르면 무시한다."""
    p=peg_table_file()
    if not os.path.exists(p): return False
    try:
        with open(p) as f: d=json.load(f)
        k=peg_table_key()
        saved=d.get('key') or []
        same=(len(saved)==4
              and abs(float(saved[0])-k[0])<1e-6
              and abs(float(saved[1])-k[1])<1e-6
              and saved[2]==k[2]
              and [tuple(t) for t in saved[3]]==list(k[3]))
        if not same:
            print("  (peg_table.json 은 다른 배치의 것이라 무시 — "
                  "peg_table_build.py 를 다시 돌리세요)")
            return False
        PEG_TABLE['span']={nm:[tuple(s) for s in sp]
                           for nm,sp in d['span'].items()}
        PEG_TABLE['key']=k
        globals()['PEG_H_MAX']=int(d.get('max_h',PEG_H_MAX))
        print(f"목심 도달표 불러옴: {len(PEG_TABLE['span'])}개 "
              f"(생성 {d.get('built','?')})")
        return True
    except Exception as e:
        print(f"  (peg_table.json 읽기 실패: {e})")
        return False

def peg_table_ready():
    return PEG_TABLE['key']==peg_table_key() and PEG_TABLE['span']

def peg_reachable_fast(height_mm):
    """표에서 즉시 조회 (IK 없음). 표가 없으면 None."""
    if not peg_table_ready(): return None
    h=int(round(float(height_mm)))
    return sorted(nm for nm,sp in PEG_TABLE['span'].items()
                  if any(a<=h<=b for a,b in sp))

_PEG_CACHE={}
def peg_reachable(height_mm,use_cache=True):
    """그 높이에서 IK 가 풀리는 목심 이름 목록."""
    fast=peg_reachable_fast(height_mm)      # 표가 있으면 즉시
    if fast is not None: return fast
    z=peg_z(height_mm)
    key=(round(z,4),round(BASE_OFF[0],4),round(BASE_OFF[1],4))
    if use_cache and key in _PEG_CACHE: return _PEG_CACHE[key]
    q0=list(CURRENT_Q); seeds=_seed_list(q0)
    # ① xy 선별: 그 높이에서 가능한 수평거리 링 밖은 IK 없이 탈락
    ring=reach_ring(z)
    if ring is not None:
        o1=CHAIN.forward_kinematics([0]*7,full_kinematics=True)[1][:3,3][:2]
        cx,cy=o1[0]+BASE_OFF[0],o1[1]+BASE_OFF[1]
        rlo,rhi=ring[0]-0.02,ring[1]+0.02    # 여유 2cm (경계 손실 방지)
    jobs=[]
    for nm in peg_names():
        xy=peg_xy(nm)
        if not xy: continue
        if ring is not None:
            d=math.hypot(xy[0]-cx,xy[1]-cy)
            if d<rlo or d>rhi: continue      # ② 링 밖 - IK 생략
        jobs.append((nm,xy[0],xy[1],z,seeds,q0[4],q0[5]))
    # ikpy(scipy)는 GIL 을 놓지 않아 스레드로는 오히려 느려진다 -> 프로세스 병렬.
    # 프로세스가 안 되는 환경(피클 실패 등)이면 순차로 안전하게 폴백.
    out=[]
    if jobs:
        try:
            import multiprocessing as _mp
            ctx=_mp.get_context('fork')      # fork = 모듈 상태를 그대로 물려받음
            with ctx.Pool(_worker_count()) as pool:
                res=pool.map(_peg_ik_worker,
                             [(j[0],j[1],j[2],z) for j in jobs],chunksize=8)
            out=[r for r in res if r]
        except Exception as e:
            print(f"[목심] 병렬 실패({type(e).__name__}) - 순차 계산")
            out=[r for r in map(_peg_ik_ok,jobs) if r]
    out.sort()
    _PEG_CACHE[key]=out
    return out

# --- 3D 목심 표시/선택 상태 + 이동 시뮬레이션(반복 재생) ---
PEG_VIEW={'on':False,'ok':[],'sel':None,'h':80,'dirty':False}
_PEG_SIM={'th':None,'stop':threading.Event(),'name':None}

def _peg_sim_loop(name,height_mm,stop,q_home):
    """선택한 목심으로 가는 경로를 3D 에서 반복 재생 (실물은 안 움직임).
    실제 실행 중이거나 다른 목심이 선택되면 즉시 멈춘다.

    q_home: 재생의 '출발 자세'. CURRENT_Q 를 그때그때 읽으면 안 된다 -
    미리보기가 스스로 움직여 놓은 중간 자세를 다시 출발점으로 삼게 되어
    (0도 -> 97도 -> 101도 ...) 경로가 매번 바뀌고 튀는 느낌이 난다."""
    xy=peg_xy(name)
    if xy is None: return
    q0=list(q_home)
    tgt=ik_tool((xy[0],xy[1],peg_z(height_mm)),_seed_list(q0),q0[4],q0[5])
    if tgt is None: return
    tgt[5]=q0[5]
    if LEVEL_ALWAYS: tgt[J5_IDX]=q0[J5_IDX]
    way,_=plan_joint_seq(q0,tgt)
    if LEVEL_ALWAYS: lock_j5(way,q0)
    # 프레임을 솎아내면 프레임당 관절 변화가 커져 '확확' 튄다(측정 3.2도/프레임).
    # 웨이포인트를 그대로 쓰고, 대신 간격을 줄여 같은 시간에 더 부드럽게 재생.
    frames=way
    dt=max(0.02,min(0.05,2.5/max(len(frames),1)))    # 편도 약 2.5초 목표
    while not stop.is_set():
        for seq in (frames,frames[::-1]):            # 갔다가 되돌아오기(왕복)
            for q in seq:
                if stop.is_set() or RUNNING.is_set(): return
                ros_publish(q); time.sleep(dt)
            if stop.is_set(): return
            time.sleep(0.4)

def _peg_home_pose():
    """미리보기의 기준 자세. 재생으로 오염되지 않은 값을 쓴다.
    (재생 중이면 재생 시작 때 기억해 둔 자세, 아니면 현재 자세)"""
    if _PEG_SIM.get('home') is not None and _PEG_SIM['th'] \
            and _PEG_SIM['th'].is_alive():
        return list(_PEG_SIM['home'])
    return list(CURRENT_Q)

def _peg_sim_stop(restore=True):
    """미리보기 정지. restore=True 면 3D 를 원래 자세로 되돌린다
    (안 되돌리면 재생 중간 자세가 그대로 남아 다음 계산의 출발점이 오염된다)."""
    home=_PEG_SIM.get('home')
    if _PEG_SIM['th'] and _PEG_SIM['th'].is_alive():
        _PEG_SIM['stop'].set(); _PEG_SIM['th'].join(timeout=1.0)
        if restore and home is not None:
            ros_publish(list(home))
    _PEG_SIM['th']=None; _PEG_SIM['name']=None; _PEG_SIM['home']=None

def _peg_sim_start(name):
    """목심 선택 시 미리보기 반복 재생 시작 (이전 재생은 중단)."""
    home=_peg_home_pose()                 # 오염되지 않은 기준 자세를 먼저 확보
    _peg_sim_stop()
    if not name or name not in PEG_VIEW['ok']: return   # 도달 불가는 재생 안 함
    ev=threading.Event(); _PEG_SIM['stop']=ev; _PEG_SIM['name']=name
    _PEG_SIM['home']=list(home)
    th=threading.Thread(target=_peg_sim_loop,
                        args=(name,PEG_VIEW['h'],ev,home),daemon=True)
    _PEG_SIM['th']=th; th.start()

load_peg_table()          # 미리 만들어 둔 표가 있으면 시작할 때 불러온다

def h_peg_view(on,height_mm):
    """3D 목심 표시 on/off. 켤 때 그 높이의 도달성을 계산해 둔다."""
    BTN('목심 표시',str(bool(on)))
    PEG_VIEW['on']=bool(on); PEG_VIEW['h']=float(height_mm)
    if not on:
        _peg_sim_stop(); PEG_VIEW['sel']=None
        return gr.update(),'목심 표시 끔'
    note=''
    if not peg_table_ready():
        # 표는 1mm 전수라 6분쯤 걸린다 -> 백그라운드로 만들고, 그동안은
        # 기존 방식(그때그때 IK)으로 동작한다. 완성되면 자동으로 표를 쓴다.
        if not _PEG_SCAN['busy']:
            _PEG_SCAN['busy']=True
            def _bg():
                try: build_peg_table()
                finally: _PEG_SCAN['busy']=False
            threading.Thread(target=_bg,daemon=True).start()
        note=(" (정밀 도달표를 백그라운드에서 만드는 중 — 완성되면 높이 변경이"
              " 즉시 반영됩니다. 그 전까지는 조금 느립니다)")
    t0=time.time(); lst=peg_reachable(height_mm); PEG_VIEW['ok']=lst
    return (gr.update(choices=lst,value=(lst[0] if lst else None)),
            f"목심 표시 ON — 높이 {height_mm}mm 기준 도달 가능 {len(lst)}개"
            f"{note}. 3D에서 구멍 위에 마우스를 올리면 "
            "노랑(가능)/빨강(불가), 클릭하면 파랑(선택)+이동 미리보기가 반복됩니다")

def h_peg_height(height_mm):
    """높이 변경 -> 도달성 자동 재계산 (표시 중일 때만)."""
    PEG_VIEW['h']=float(height_mm)
    if not PEG_VIEW['on']:
        return gr.update(), f"높이 {height_mm}mm (목심 표시를 켜면 계산합니다)"
    t0=time.time(); lst=peg_reachable(height_mm); PEG_VIEW['ok']=lst
    sel=PEG_VIEW.get('sel')
    if sel not in lst: sel=(lst[0] if lst else None); PEG_VIEW['sel']=sel
    _peg_sim_start(sel)
    return (gr.update(choices=lst,value=sel),
            f"높이 {height_mm}mm — 도달 가능 {len(lst)}개 "
            f"({time.time()-t0:.1f}초)")

def h_peg_pick(name):
    """드롭다운에서 고른 경우도 3D 선택/미리보기와 동기화."""
    PEG_VIEW['sel']=name
    _peg_sim_start(name)
    return f"선택: {name} (3D에서 파란색, 이동 미리보기 반복 중)" if name else ""

def h_peg_goto(name,height_mm):
    """목심 위 특정 높이로 이동. URDF 자세 없이 좌표만으로."""
    BTN('목심 이동',f'{name} {height_mm}mm')
    if not name: return "먼저 [도달 가능 목심 찾기] 를 눌러 목심을 고르세요"
    xy=peg_xy(name)
    if xy is None: return f"목심 이름 형식 오류: {name}"
    if RUNNING.is_set(): return "다른 동작 실행 중 - [정지] 후 다시"
    # 표(호버 색)가 불가라고 한 곳은 실행도 막는다.
    # 표는 현재 CURRENT_Q 기준, 실행은 실물 자세 기준이라 미세하게 다를 수 있는데
    # 그 차이로 '빨간데 움직이는' 일이 생기면 안 된다 -> 표를 기준으로 통일.
    _ok=peg_reachable_fast(height_mm)     # 그 높이 기준으로 다시 판정
    if _ok is not None and name not in _ok:
        return (f"{name} 은(는) 높이 {height_mm}mm 에서 도달 불가입니다 "
                "(3D에서 빨간색). 높이를 바꾸거나 다른 목심을 고르세요")
    _peg_sim_stop()                       # 미리보기 반복 재생 중단
    z=peg_z(height_mm)
    q0=ROBOT.read() if ROBOT.connect() else list(CURRENT_Q)
    tgt=ik_tool((xy[0],xy[1],z),_seed_list(q0),q0[4],q0[5])
    if tgt is None:
        return (f"{name} 의 {height_mm}mm 높이는 도달 불가 - "
                "높이를 올리거나 다른 목심을 고르세요")
    if not ROBOT.connect():
        return f"로봇 연결 실패 - [이동] 탭에서 '시뮬레이션' 을 고르면 3D로 볼 수 있습니다"
    STOP.clear(); RUNNING.set()
    try:
        ROBOT.time_profile(True); ROBOT.torque(True)
        tgt[5]=q0[5]                          # 그리퍼 상태 유지
        if LEVEL_ALWAYS: tgt[J5_IDX]=q0[J5_IDX]
        way,dt=plan_joint_seq(q0,tgt)
        for f in way: f[5]=q0[5]
        if LEVEL_ALWAYS: lock_j5(way,q0)
        ros_publish(tgt)
        for q in way:
            if STOP.is_set(): break
            ROBOT.send(q,log=True,tag=name); time.sleep(dt)
        p=tool_pos(tgt) if TOOL_HAS else fk(tgt)[0]
        return (f"{name} 위 {height_mm}mm 도착  "
                f"툴중심(x={p[0]*1000:.0f} y={p[1]*1000:.0f} z={p[2]*1000:.0f}mm)")
    except Exception as e:
        ROBOT.estop(); return f"오류: {e} - 토크 해제됨"
    finally:
        RUNNING.clear()

def h_peg_save_pose(name,height_mm,newname):
    """목심 위치를 자세로 등록 -> [이동] 탭 파이프라인에서 블록으로 쓸 수 있다."""
    BTN('목심 자세 등록',f'{name} {height_mm}mm')
    if not name: return gr.update(),gr.update(),"먼저 목심을 선택하세요"
    xy=peg_xy(name)
    if xy is None: return gr.update(),gr.update(),f"목심 이름 오류: {name}"
    q0=list(CURRENT_Q)
    q=ik_tool((xy[0],xy[1],peg_z(height_mm)),_seed_list(q0),q0[4],q0[5])
    if q is None:
        return gr.update(),gr.update(),f"{name} 의 {height_mm}mm 는 도달 불가"
    q[5]=q0[5]
    if LEVEL_ALWAYS: q[J5_IDX]=q0[J5_IDX]
    nm=(newname or '').strip() or f"{name}@{int(height_mm)}mm"
    base=nm; k=2
    while nm in pose_names(): nm=f"{base}_{k}"; k+=1
    with LOCK:
        S['poses'].append({'name':nm,'angles':list(q),'offset':[0.0]*6})
    _macro_save()
    ros_publish(q)                      # 등록한 자세를 바로 3D 로 보여줌
    return (gr.update(choices=pose_names(),value=nm),
            gr.update(choices=pose_names(),value=nm),
            f"자세 '{nm}' 등록 — [이동] 탭에서 파이프라인 블록으로 쓸 수 있습니다")

def h_robot_move_mode(on):
    ROBOT_MOVE['on']=bool(on)
    if on: PLATE_EDIT['on']=False          # 두 클릭 모드 동시 금지
    msg=('로봇 이동 모드 ON - 3D에서 초록 타일을 클릭하면 로봇이 그 타일 '
         '중앙으로 이동합니다' if on else '로봇 이동 모드 OFF')
    return _robot_pos_str(), msg

def h_robot_home():
    if RUNNING.is_set(): return _robot_pos_str(),"동작 실행 중 - [정지] 후 다시"
    set_robot_xy(0.0,0.0)
    return _robot_pos_str(), "로봇 위치 원점 복귀"
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
# ===== 잡을 수 있는 물체 (bricks) =====
# 잡기 조건은 두 가지뿐이다:
#   ① 물체 중심축(axis_ppap)과 툴 중심축(axis_7)이 x-y 에서 일치
#   ② 두 축의 z 방향 수직 거리 = 물체마다 정해진 값 (PPAP 는 36.37mm)
# 원통이라 회전 방향은 무관하고, 수평 유지는 기존 IK 가 이미 보장한다.
BRICK_DIR='/home/drboom/2026_project/urdf/bricks'
# PPAP 실측은 36.37mm 지만 37mm 로 고정한다.
# 정확히 맞추면 오차가 아래로 날 때 그리퍼가 물체를 파고들어 이상하게 잡힌다
# -> 0.63mm 여유를 위로 두는 편이 안전 (사용자 결정)
GRASP_DZ_DEFAULT=0.037            # PPAP: 바닥에서 37mm

BRICKS={}                         # id -> {mesh, dia, height, grasp_dz, pos}
def _load_bricks():
    import glob as _g
    for pkg in sorted(_g.glob(os.path.join(BRICK_DIR,'*'))):
        if not os.path.isdir(pkg): continue
        name=os.path.basename(pkg).split('_')[-1]     # ..._ppap -> ppap
        cands=sorted(_g.glob(os.path.join(pkg,'meshes','*l1*.STL'))) \
              or sorted(_g.glob(os.path.join(pkg,'meshes','*.STL')))
        if not cands: continue
        try:
            mm=trimesh.load(cands[0])
            v=np.asarray(mm.vertices)                 # 이 패키지는 m 단위
            b=np.array([v.min(0),v.max(0)])
            rad=float(np.median(np.linalg.norm(v[:,:2],axis=1)))
            BRICKS[name]={'mesh':cands[0],'scale':1.0,
                          'dia':rad*2,'h':float(b[1][2]-b[0][2]),
                          'z_bottom':float(b[0][2]),
                          'grasp_dz':GRASP_DZ_DEFAULT,
                          'pos':None}                  # 아직 배치 안 됨
            print(f"물체 등록: {name}  지름 {rad*2*1000:.1f}mm "
                  f"높이 {(b[1][2]-b[0][2])*1000:.1f}mm  "
                  f"잡는 높이 {GRASP_DZ_DEFAULT*1000:.2f}mm")
        except Exception as e:
            print(f"물체 로드 실패 {pkg}: {e}")
_load_bricks()

def brick_grasp_point(bid):
    """그 물체를 잡을 때 axis_7(툴 중심)이 놓여야 할 월드 좌표.
    = 물체 중심축 위, 바닥에서 grasp_dz 만큼 올라간 점."""
    b=BRICKS.get(bid)
    if not b or b.get('pos') is None: return None
    x,y,z_bottom=b['pos']
    return np.array([x,y,z_bottom+b['grasp_dz']])

def place_brick(bid,x,y,z_bottom):
    """물체를 (x,y) 에 세우고 바닥 높이를 z_bottom 으로 둔다."""
    if bid not in BRICKS: return False
    BRICKS[bid]['pos']=(float(x),float(y),float(z_bottom))
    return True

def docking_ids():
    """환경 STL 중 도킹 스테이션 목록."""
    return [e['id'] for e in ENV_SHOW if 'docking' in e['id'].lower()]

def docking_top(did):
    """도킹 스테이션의 (중심 x, 중심 y, 윗면 z). 물체를 세울 자리."""
    if did not in MESH_REG: return None
    path,scale=MESH_REG[did]
    try: mm=trimesh.load(path)
    except Exception: return None
    v=np.asarray(mm.vertices)*scale
    off=np.zeros(3)
    for e in ENV_SHOW:
        if e['id']==did: off=np.asarray(e.get('offset',(0,0,0)),float)
    return (float(v[:,0].mean()+off[0]), float(v[:,1].mean()+off[1]),
            float(v[:,2].max()+off[2]))

def place_brick_on_docking(bid,did):
    """물체를 도킹 스테이션 위에 세운다 (중심축 일치, 바닥=도킹 윗면)."""
    t=docking_top(did)
    if t is None: return False,f'도킹 스테이션을 찾을 수 없습니다: {did}'
    if not place_brick(bid,t[0],t[1],t[2]):
        return False,f'물체를 찾을 수 없습니다: {bid}'
    p=brick_grasp_point(bid)
    return True,(f"{bid} 를 {did} 위에 세움 "
                 f"(중심 {t[0]*1000:.0f},{t[1]*1000:.0f}mm, 바닥 z={t[2]*1000:.0f}mm)\n"
                 f"  잡는 점: z={p[2]*1000:.1f}mm "
                 f"(바닥 +{BRICKS[bid]['grasp_dz']*1000:.2f}mm)")

def brick_graspable(bid):
    """지금 그 물체를 잡을 수 있는지 (수평 유지 IK 로 그 점에 도달 가능한지).
    반환: (가능여부, 관절해 또는 사유)"""
    p=brick_grasp_point(bid)
    if p is None: return False,'물체가 배치되지 않았습니다'
    q0=list(CURRENT_Q)
    q=ik_tool(p,_seed_list(q0),q0[4],q0[5])
    if q is None:
        return False,(f'그 지점({p[0]*1000:.0f},{p[1]*1000:.0f},'
                      f'{p[2]*1000:.0f}mm)은 수평 유지로 도달 불가')
    return True,q

GRASP_LIFT=0.06        # 원하는 접근/상승 여유 높이 (m)
HELD={'brick':None}    # 지금 그리퍼가 들고 있는 물체

def _max_z_over(x,y,z_from,z_to=0.30):
    """(x,y) 축 위에서 도달 가능한 가장 높은 z. 없으면 None."""
    q0=list(CURRENT_Q); seeds=_seed_list(q0)
    z=z_to
    while z>=z_from-1e-9:
        if ik_tool((x,y,z),seeds,q0[4],q0[5]) is not None: return z
        z-=0.005
    return None

def _travel_z(p_from,p_to,z_min,z_max=0.30,n=8):
    """출발점 -> 목표점 '경로 전체' 에서 공통으로 갈 수 있는 최고 z.

    한쪽에서만 높이 올라갈 수 있어도 소용없다. 중간에 막히면 수평 이동이
    안 되므로, 경로를 나눠 각 지점의 최고 높이 중 '가장 낮은 값' 을 쓴다.
    (간편 이동의 '뽑기 -> 수평 -> 하강' 과 같은 원리)
    """
    a=np.asarray(p_from,float); b=np.asarray(p_to,float)
    best=z_max
    for k in range(n+1):
        c=a+(b-a)*(k/n)
        z=_max_z_over(float(c[0]),float(c[1]),z_min,best)
        if z is None: return None
        best=min(best,z)
    return best

def _descend(x,y,z_hi,z_lo,tag,steps=None):
    """축을 맞춘 채 수직으로만 내려간다 (x,y 고정 = 두 축 일치 유지).
    중간 지점마다 IK 를 풀어 보내므로 경로가 옆으로 새지 않는다."""
    n=steps or max(6,int(abs(z_hi-z_lo)/0.005))
    q0=ROBOT.read() if ROBOT.connect() else list(CURRENT_Q)
    for k in range(1,n+1):
        if STOP.is_set(): return f"{tag}: 정지됨"
        z=z_hi+(z_lo-z_hi)*k/n
        q=ik_tool((x,y,z),_seed_list(q0),q0[4],q0[5])
        if q is None: return f"{tag}: z={z*1000:.0f}mm 도달 불가"
        q[5]=q0[5]
        if LEVEL_ALWAYS: q[J5_IDX]=q0[J5_IDX]
        ros_publish(q); ROBOT.send(q,log=True,tag=tag)
        q0=q
        time.sleep(0.05*(DXL_STEP_SLOW if EXEC.get('mode')=='dxl' else 1.0))
    return None

def _brick_move_to(p,tag):
    """툴 중심을 월드 좌표 p 로 (수평 유지). 실패 시 사유 문자열 반환."""
    q0=ROBOT.read() if ROBOT.connect() else list(CURRENT_Q)
    q=ik_tool(np.asarray(p,float),_seed_list(q0),q0[4],q0[5])
    if q is None:
        return f"{tag}: 그 지점에 도달할 수 없습니다"
    q[5]=q0[5]
    if LEVEL_ALWAYS: q[J5_IDX]=q0[J5_IDX]
    way,dt=plan_joint_seq(q0,q)
    for f in way: f[5]=q0[5]
    if LEVEL_ALWAYS: lock_j5(way,q0)
    ros_publish(q)
    for f in way:
        if STOP.is_set(): return f"{tag}: 정지됨"
        ROBOT.send(f,log=True,tag=tag); time.sleep(dt)
    return None

def do_grasp(bid):
    """잡기 (사용자 정의 순서)
      ① z 상승  - 현재 위치에서 목표 축까지 '경로 전체' 에서 갈 수 있는 최고 z
      ② 그리퍼 열기 (안전 높이에서)
      ③ 축 일치  - 그 높이를 유지한 채 수평 이동, axis_ppap = axis_7
      ④ 수직 하강 - 축 맞춘 채 z 만 내려 바닥 +37mm 까지
      ⑤ 닫기 = 잡힘
    """
    BTN('잡기',bid)
    p=brick_grasp_point(bid)
    if p is None: return f"{bid} 가 배치되지 않았습니다"
    if RUNNING.is_set(): return "다른 동작 실행 중 - [정지] 후 다시"
    if not ROBOT.connect():
        return "로봇 연결 실패 — [이동] 탭에서 '시뮬레이션' 을 고르면 3D로 볼 수 있습니다"
    x,y,zg=float(p[0]),float(p[1]),float(p[2])
    now=tool_pos(ROBOT.read() if ROBOT.connect() else CURRENT_Q)
    STOP.clear(); RUNNING.set()
    try:
        ROBOT.time_profile(True); ROBOT.torque(True)
        ztr=_travel_z(now,p,zg)                              # 경로 공통 최고 z
        if ztr is None:
            return f"{bid} 의 축({x*1000:.0f},{y*1000:.0f}) 까지 갈 수 없습니다"
        e=_descend(float(now[0]),float(now[1]),float(now[2]),ztr,'잡기① 상승')
        if e: return e                                       # ① 제자리 상승
        ROBOT.gripper(True); time.sleep(0.3)                 # ② 열기
        e=_brick_move_to((x,y,ztr),'잡기③ 축 일치')          # ③ 수평 이동
        if e: return e
        e=_descend(x,y,ztr,zg,'잡기④ 수직하강')              # ④ 수직 하강
        if e: return e
        ROBOT.gripper(False); time.sleep(0.5)                # ⑤ 닫기 = 잡힘
        HELD['brick']=bid
        return (f"{bid} 잡음 — 상승 z{ztr*1000:.0f}mm → 축 일치 → "
                f"수직하강 {zg*1000:.0f}mm (바닥 +{BRICKS[bid]['grasp_dz']*1000:.0f}mm) → 닫기")
    except Exception as ex:
        ROBOT.estop(); return f"오류: {ex} - 토크 해제됨"
    finally:
        RUNNING.clear()

def do_release(did):
    """놓기: 목표 도킹 위로 이동 -> 하강 -> 열기 -> 상승. 잡기의 역순."""
    BTN('놓기',did)
    bid=HELD['brick']
    if bid is None: return "들고 있는 물체가 없습니다 (먼저 [잡기])"
    t=docking_top(did)
    if t is None: return f"도킹을 찾을 수 없습니다: {did}"
    if RUNNING.is_set(): return "다른 동작 실행 중 - [정지] 후 다시"
    if not ROBOT.connect(): return "로봇 연결 실패"
    # 놓기 = 잡기와 같은 구조 (물체를 든 채이므로 열기가 하강 뒤에 온다)
    #  ① z 상승  ② 축 일치(수평)  ③ 수직 하강  ④ 열기=놓임  ⑤ 수직 상승
    x,y=float(t[0]),float(t[1])
    zg=float(t[2]+BRICKS[bid]['grasp_dz'])
    now=tool_pos(ROBOT.read() if ROBOT.connect() else CURRENT_Q)
    STOP.clear(); RUNNING.set()
    try:
        ROBOT.time_profile(True); ROBOT.torque(True)
        ztr=_travel_z(now,(x,y,zg),zg)
        if ztr is None:
            return f"{did} 축까지 갈 수 없습니다"
        e=_descend(float(now[0]),float(now[1]),float(now[2]),ztr,'놓기① 상승')
        if e: return e
        e=_brick_move_to((x,y,ztr),'놓기② 축 일치')
        if e: return e
        e=_descend(x,y,ztr,zg,'놓기③ 수직하강')
        if e: return e
        ROBOT.gripper(True); time.sleep(0.5)          # ④ 열기 = 놓임
        place_brick(bid,t[0],t[1],t[2])               # 물체 위치 갱신
        HELD['brick']=None
        e=_descend(x,y,zg,ztr,'놓기⑤ 수직상승')       # ⑤ 같은 축으로 상승
        if e: return e
        return (f"{bid} 를 {did} 위에 놓음 — 상승 z{ztr*1000:.0f}mm → 축 일치 → "
                f"수직하강 {zg*1000:.0f}mm → 열기 → 상승")
    except Exception as ex:
        ROBOT.estop(); return f"오류: {ex} - 토크 해제됨"
    finally:
        RUNNING.clear()

# ===== 물체·도킹 이름 (사람이 부르는 말) =====
# "분홍색 펜", "도킹 4번" 처럼 지칭하려면 파일명 말고 이름이 필요하다.
# calib.json 에 저장되어 재시작해도 유지된다.
def _names():
    return S.setdefault('names',{})          # id -> {'name':..,'color':..}

def set_name(oid,name=None,color=None):
    d=_names().setdefault(oid,{})
    if name is not None: d['name']=name.strip()
    if color: d['color']=color
    if color:                                # 3D 색도 함께 바꾼다
        for e in ENV_SHOW:
            if e['id']==oid: e['color']=color
    _macro_save()
    return d

def disp_name(oid):
    """등록된 이름이 있으면 그것, 없으면 파일명에서 뽑은 짧은 이름."""
    d=_names().get(oid) or {}
    if d.get('name'): return d['name']
    s=str(oid)
    for tag in ('Industry_gyu_arm_','20260721_-_','20260727_-_','_-_'):
        s=s.replace(tag,'')
    return s

def brick_label(bid): return disp_name(bid)
def dock_label(did):  return disp_name(did)

def _peg_layout_changed():
    """플레이트 배치가 바뀌면 목심 구성도 바뀐다 -> 도달표/캐시 무효화.
    (안 하면 새로 생긴 목심이 옛 목록에 없어 전부 빨강으로 표시된다)"""
    PEG_TABLE['key']=None; PEG_TABLE['span']={}
    _PEG_CACHE.clear()
    PEG_VIEW['dirty']=True

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
    _peg_layout_changed()
    return True
def remove_plate(tile):
    if tile not in PLATES: return False
    info=PLATES[tile]
    if not info.get('added'): return False      # 원본 STL은 삭제 금지
    mid=info['id']
    ENV_SHOW[:] = [e for e in ENV_SHOW if e['id']!=mid]
    MESH_REG.pop(mid,None); _COLL_ENV.pop(mid,None)
    del PLATES[tile]
    _peg_layout_changed()
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
<div id="lg"><b style="color:#ff44dd">●</b> 자홍색 = 툴 중심(axis_7)</div>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
const scene=new THREE.Scene(); scene.background=new THREE.Color(0x303030);
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
// base_link 원점 축은 표시하지 않는다 (플레이트 위가 지저분해 보임).
// 다시 켜려면 아래 주석을 해제.
// world.add(makeAxes(0.20,3));

// ---- EE 좌표축 (엔드이펙터 위치, 방향은 월드축과 동일) ----
// EE(j6) 축은 표시하지 않는다 - 기준점은 툴 중심(axis_7) 이므로 혼동 방지.
// 위치는 계속 갱신하되 화면에는 숨김 (다시 켜려면 visible=true)
const eeAxes=makeAxes(0.08,2); eeAxes.visible=false; world.add(eeAxes);
// ---- 툴 중심 (axis_7) : 자홍색 구 + 작은 축. j6 와 구분되게 ----
const toolMark=new THREE.Group();
toolMark.add(new THREE.Mesh(new THREE.SphereGeometry(0.006,16,12),
  new THREE.MeshBasicMaterial({color:0xff00cc,depthTest:false})));
toolMark.add(makeAxes(0.05,2));
toolMark.visible=false; world.add(toolMark);
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
// ---- 목심(구멍) 레이어 ----
// 평소엔 안 보이고, 마우스를 올린 그 구멍만 색이 뜬다.
//   호버: 도달 가능=노랑 / 불가=빨강    클릭: 파랑(선택)
const pegGroup=new THREE.Group(); world.add(pegGroup);
const PEG_INVIS=new THREE.MeshBasicMaterial({visible:false});
const PEG_OK  =new THREE.MeshBasicMaterial({color:0xffdd00,transparent:true,opacity:0.85});
const PEG_NG  =new THREE.MeshBasicMaterial({color:0xff3333,transparent:true,opacity:0.75});
const PEG_SEL =new THREE.MeshBasicMaterial({color:0x2299ff,transparent:true,opacity:0.95});
let pegMeshes=[], pegHover=null, pegSelName=null;
function buildPegs(list,ptop){
  const key=JSON.stringify([list.length,list[0]&&list[0].n,ptop]);
  if(pegGroup.userData.key===key) return;
  pegGroup.userData.key=key;
  pegGroup.clear(); pegMeshes=[];
  const geo=new THREE.CylinderGeometry(0.008,0.008,0.004,16);
  geo.rotateX(Math.PI/2);                 // z 축을 위로
  for(const p of list){
    const m=new THREE.Mesh(geo,PEG_INVIS);
    m.position.set(p.x,p.y,ptop+0.003);
    m.userData={name:p.n,ok:p.ok};
    pegGroup.add(m); pegMeshes.push(m);
  }
}
function pegPaint(m){
  if(!m) return;
  m.material = (m.userData.name===pegSelName) ? PEG_SEL
             : (m===pegHover ? (m.userData.ok?PEG_OK:PEG_NG) : PEG_INVIS);
}
const ray=new THREE.Raycaster(), mouse=new THREE.Vector2();
let hover=null;
ren.domElement.addEventListener('pointermove',ev=>{
  const r=ren.domElement.getBoundingClientRect();
  mouse.x=((ev.clientX-r.left)/r.width)*2-1;
  mouse.y=-((ev.clientY-r.top)/r.height)*2+1;
  ray.setFromCamera(mouse,cam);
  // 목심이 타일보다 우선 (목심 위에 있으면 타일 호버는 끔)
  const ph=ray.intersectObjects(pegMeshes,false);
  const prev=pegHover;
  pegHover=ph.length?ph[0].object:null;
  if(prev!==pegHover){ pegPaint(prev); pegPaint(pegHover); }
  const hits=pegHover?[]:ray.intersectObjects(ghostMeshes,false);
  if(hover) hover.material=ghostMat;
  hover=hits.length?hits[0].object:null;
  if(hover) hover.material=ghostMatHi;
  ren.domElement.style.cursor=(hover||pegHover)?'pointer':'';
});
ren.domElement.addEventListener('click',async ev=>{
  if(pegHover){                            // 목심 선택 (도달 가능한 것만)
    if(!pegHover.userData.ok) return;      // 빨강은 클릭 무시
    const prev=pegMeshes.find(m=>m.userData.name===pegSelName);
    pegSelName=pegHover.userData.name;
    pegPaint(prev); pegPaint(pegHover);
    await fetch('/peg/'+encodeURIComponent(pegSelName));
    return;
  }
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
    buildPegs(st.pegs||[],st.ptop||0.02);
    if(st.pegsel!==pegSelName){          // 서버(드롭다운)에서 바뀐 선택 반영
      const a=pegMeshes.find(m=>m.userData.name===pegSelName);
      pegSelName=st.pegsel; pegPaint(a);
      pegPaint(pegMeshes.find(m=>m.userData.name===pegSelName));
    }
    if(st.eexyz) eeAxes.position.set(st.eexyz[0],st.eexyz[1],st.eexyz[2]);
    if(st.toolxyz){ toolMark.visible=true;
      toolMark.position.set(st.toolxyz[0],st.toolxyz[1],st.toolxyz[2]); }
    else toolMark.visible=false;
    document.title=(st.tool?('툴 '+st.tool):('EE '+st.ee));
  }catch(err){}
  setTimeout(poll,25);   // 미리보기 재생(25ms)과 맞춰야 프레임이 안 버려진다
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
                    T=_base_T() if k==0 else frames[k]
                    links.append({'id':ln,'color':ROBOT_COLOR_MAP[ln],
                                  'm':[float(x) for x in T[:3,:].flatten()]+[0,0,0,1]})
                p,_=fk(CURRENT_Q)
                _tp=tool_pos(CURRENT_Q) if TOOL_HAS else p
                env=[]
                for e in ENV_SHOW:
                    env.append({'id':e['id'],'color':e['color'],
                                'scale':MESH_REG[e['id']][1],
                                'v':e.get('visible',True),
                                'off':list(e.get('offset',(0,0,0))),
                                'tile':list(e['tile']) if e.get('tile') else None,
                                'plate':'plate' in e['id'].lower()})
                if PLATE_EDIT['on']:
                    ghosts=[{'tile':list(t),
                             'x':tile_origin(t)[0],'y':tile_origin(t)[1]}
                            for t in ghost_tiles()]
                elif ROBOT_MOVE['on']:
                    # 로봇 이동 모드: 실존 플레이트 타일이 클릭 대상
                    ghosts=[{'tile':list(t),
                             'x':tile_origin(t)[0],'y':tile_origin(t)[1]}
                            for t in plate_tiles()]
                else:
                    ghosts=[]
                pegs=[]
                if PEG_VIEW['on']:
                    # 표가 있으면 '현재 슬라이더 높이' 로 그때그때 판정한다.
                    # (드래그 중에는 gradio 핸들러가 아직 안 도는데, 뷰어는 계속
                    #  폴링하므로 옛 높이의 색이 남아 실제와 어긋났었다)
                    live=peg_reachable_fast(PEG_VIEW['h'])
                    if live is None and PEG_VIEW.get('dirty'):
                        # 플레이트 추가/삭제 등으로 목심 구성이 바뀌었는데
                        # 표가 아직 없다 -> 옛 목록을 쓰면 새 목심이 전부
                        # 빨강으로 뜬다. 한 번만 다시 계산해 반영한다.
                        PEG_VIEW['ok']=peg_reachable(PEG_VIEW['h'])
                        PEG_VIEW['dirty']=False
                    ok=set(live if live is not None else PEG_VIEW['ok'])
                    for nm in peg_names():
                        xy=peg_xy(nm)
                        if xy: pegs.append({'n':nm,'x':xy[0],'y':xy[1],
                                            'ok':nm in ok})
                body=json.dumps({'pegs':pegs,'ptop':PLATE_TOP,
                    'pegsel':PEG_VIEW.get('sel'),
                    'links':links,'env':env,'ghosts':ghosts,
                    'psize':PLATE_SIZE,'sel':PLATE_EDIT.get('sel'),
                    'ee':f"x={p[0]*1000:.0f} y={p[1]*1000:.0f} z={p[2]*1000:.0f}mm",
                    'eexyz':[float(v) for v in p],
                    'toolxyz':([float(v) for v in _tp] if TOOL_HAS else None),
                    'tool':(f"x={_tp[0]*1000:.0f} y={_tp[1]*1000:.0f} "
                            f"z={_tp[2]*1000:.0f}mm" if TOOL_HAS else None)
                    }).encode()
                self._send(200,'application/json',body)
            elif self.path.startswith('/peg/'):
                # 3D 에서 목심 클릭 -> 선택 + 이동 시뮬 예약
                from urllib.parse import unquote
                nm=unquote(self.path[len('/peg/'):])
                _live=peg_reachable_fast(PEG_VIEW['h'])
                _okset=set(_live if _live is not None else PEG_VIEW['ok'])
                if nm in _okset:                  # 빨강(도달 불가)은 선택 불가
                    PEG_VIEW['sel']=nm
                    _peg_sim_start(nm)
                    self._send(200,'application/json',b'{"ok":true}')
                else:
                    self._send(200,'application/json',b'{"ok":false}')
            elif self.path.startswith('/click/'):
                # 뷰어에서 고스트 클릭 -> 플레이트 추가
                try:
                    ix,iy=self.path[len('/click/'):].split(',')
                    t=(int(ix),int(iy))
                    ok=robot_to_tile(t) if ROBOT_MOVE['on'] else add_plate(t)
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

# ================= UI =================
with gr.Blocks(title='ARM STUDIO') as demo:
    gr.Markdown('## ARM STUDIO')
    with gr.Row():
        # ================= 왼쪽 사이드바 (자세 / 이동 / 환경) =================
        with gr.Column(scale=3):
            with gr.Tabs():
                # ---- 1. 자세: 미리보기 + 미세조정 + 불러오기 ----
                with gr.Tab('자세'):
                    with gr.Row():
                        pose_sel=gr.Radio(choices=pose_names(),value='base',
                                          label='자세 목록 (클릭 = 3D 미리보기)',
                                          scale=4)
                        ft_refresh=gr.Button('↻',scale=1,size='sm')
                    cal_robot=gr.Radio(choices=list(_CAL_KEYS),
                                       value=list(_CAL_KEYS)[0],
                                       label='보정할 로봇 (실행 로봇과 별개)')
                    with gr.Accordion('영점(base) 손으로 맞추기',open=False):
                        gr.Markdown('토크를 풀고 손으로 base 자세를 잡은 뒤 '
                                    '저장합니다. 그리퍼가 지면과 수평이 되게.')
                        with gr.Row():
                            btn_cal_off=gr.Button('① 토크 해제',size='sm')
                            btn_cal_cap=gr.Button('② 이 자세를 base 로 저장',
                                                  size='sm',variant='primary')
                    ft_joint=gr.Radio(choices=['J1','J2','J3','J4','J5'],
                                      value='J1',label='조인트 선택')
                    with gr.Accordion('미세조정 (실물만 보정, 3D는 정답 고정)',
                                      open=True):
                        ft_go=gr.Button('로봇을 이 자세로',variant='primary')
                        with gr.Row():
                            ft_step=gr.Dropdown(choices=['0.5','1','2','5'],
                                                value='1',label='스텝(도)',scale=2)
                            ft_minus=gr.Button('−',variant='secondary',scale=1)
                            ft_plus=gr.Button('+',variant='secondary',scale=1)
                        with gr.Row():
                            ft_gopen=gr.Button('그리퍼 열기',size='sm')
                            ft_gclose=gr.Button('그리퍼 닫기',size='sm')
                        with gr.Row():
                            ft_apply=gr.Button('base 보정 가져오기',size='sm')
                            ft_reset=gr.Button('보정 초기화',size='sm')
                            ft_savebtn=gr.Button('보정 저장',variant='primary',
                                                 size='sm')
                    with gr.Accordion('URDF / STL 불러오기',open=False):
                        up_kind=gr.Radio(['URDF (자세)','STL (환경)'],
                                         value='URDF (자세)',show_label=False)
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
                        mv_refresh=gr.Button('↻',scale=1,size='sm')
                    with gr.Row():
                        btn_sadd=gr.Button('■ 자세 추가',variant='primary')
                        btn_gadd_open=gr.Button('◆ 그리퍼 열기',size='sm')
                        btn_gadd_close=gr.Button('◆ 그리퍼 닫기',size='sm')
                    gr.Markdown('**파이프라인** (■자세 / ┃이동 / ◆그리퍼)')
                    seq_sel=gr.Radio(choices=seq_labels(),
                                     label='블록 선택',show_label=False)
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
                    exec_robot=gr.Radio(['시뮬레이션 (로봇 없이)','SO-ARM (STS)',
                                         'Dynamixel','둘 다 (동시)'],
                                        value='SO-ARM (STS)',label='실행 로봇')
                    with gr.Row():
                        robot_stat=gr.Textbox(label='연결 상태',value='(확인 전)',
                                              interactive=False,scale=4,lines=2)
                        btn_rstat=gr.Button('확인',scale=1,size='sm')
                    with gr.Row():
                        speed=gr.Dropdown(choices=['0.5x','1.0x','1.5x','2.0x','3.0x','4.0x'],
                                          value='1.5x',label='속도')
                        loop=gr.Checkbox(label='반복',value=False)
                    with gr.Row():
                        btn_prev=gr.Button('미리보기',variant='secondary')
                        btn_run=gr.Button('실물 실행',variant='primary')
                        btn_stop=gr.Button('정지',variant='stop')
                    btn_toff=gr.Button('토크 해제',size='sm')
                    gr.Markdown('**매크로** — 로봇을 손/조그로 옮겨놓고 기록 → '
                                '버튼 하나로 그 위치로 이동 (수평 유지)')
                    with gr.Row():
                        macro_btns=[gr.Button(_macro_btn_label(s),
                                              variant='primary')
                                    for s in MACRO_SLOTS]
                    with gr.Row():
                        macro_slot=gr.Dropdown(choices=MACRO_SLOTS,value='1',
                                               label='슬롯',scale=1)
                        btn_mrec=gr.Button('현재 위치 기록',scale=2)
                        btn_mclr=gr.Button('비우기',scale=1)
                    gr.Markdown('**위에서 본 맵** — ┃이동 블록 선택 후 클릭하면 '
                                '경로 점이 직교로 추가됩니다')
                    map_img=gr.HTML(value='')
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
                    gr.Markdown('**로봇 위치** — 플레이트 타일에 스냅')
                    robot_move_chk=gr.Checkbox(
                        label='로봇 이동 모드 (3D에서 초록 타일 클릭 = 그 타일 중앙으로)',
                        value=False)
                    robot_pos_txt=gr.Textbox(label='현재 로봇 위치',
                                             value=_robot_pos_str(),
                                             interactive=False)
                    btn_robot_home=gr.Button('원점 복귀',size='sm')

                # ---- 4. 공간 선택 (목심 좌표로 직접 이동) ----
                with gr.Tab('공간 선택'):
                    gr.Markdown(
                        '**목심 좌표로 이동** — URDF 자세 없이 바닥 목심 위치와 '
                        '높이만으로 이동합니다.\n\n'
                        f'목심 간격 {PEG_PITCH*1000:.0f}mm · 플레이트당 10×10 · '
                        '이름은 `타일:열행` (예: `0,0:C4`)')
                    peg_view=gr.Checkbox(
                        label='3D 목심 선택 모드 (구멍에 마우스를 올리면 '
                              '노랑=가능 / 빨강=불가, 클릭=파랑 선택)',
                        value=False)
                    peg_h=gr.Slider(0,250,value=80,step=1,
                        label='높이 (mm) — 플레이트 윗면에서 그리퍼 축까지')
                    peg_sel=gr.Dropdown(choices=[],label='선택한 목심',
                                        info='도달 가능한 것만 표시됩니다')
                    btn_peggo=gr.Button('이 목심 위로 이동 (실물)',
                                        variant='primary')
                    gr.Markdown('---\n**이 위치를 자세로 등록** — 등록하면 '
                                '[이동] 탭 파이프라인에서 블록으로 쓸 수 있습니다')
                    peg_pname=gr.Textbox(label='자세 이름 (비우면 자동)',
                                         placeholder='예: 픽업위치')
                    btn_pegpose=gr.Button('자세로 등록')
                    gr.Markdown('---\n**물체 잡기/놓기** — 물체 중심축과 '
                                '그리퍼 축(axis_7)을 맞추고 정해진 높이에서 '
                                '닫으면 잡힙니다')
                    with gr.Row():
                        brick_sel=gr.Dropdown(choices=list(BRICKS),
                                              value=(list(BRICKS)[0] if BRICKS else None),
                                              label='물체')
                        dock_sel=gr.Dropdown(choices=docking_ids(),
                                             value=(docking_ids()[0] if docking_ids() else None),
                                             label='도킹 스테이션')
                    btn_place=gr.Button('여기에 세우기 (시뮬)')
                    with gr.Row():
                        btn_grasp=gr.Button('잡기',variant='primary')
                        btn_release=gr.Button('놓기',variant='primary')
                    with gr.Row():
                        btn_gr_add=gr.Button('◈ 잡기 블록 추가',size='sm')
                        btn_rl_add=gr.Button('◈ 놓기 블록 추가',size='sm')
                    gr.Markdown('---\n**이름 붙이기** — 물체·도킹에 부르는 이름과 '
                                '색을 지정하면 3D와 블록에 그 이름이 표시됩니다')
                    nm_target=gr.Dropdown(
                        choices=list(BRICKS)+env_ids(),
                        value=(list(BRICKS)[0] if BRICKS else None),
                        label='대상')
                    with gr.Row():
                        nm_text=gr.Textbox(label='이름',placeholder='예: 분홍 펜')
                        nm_color=gr.ColorPicker(label='색',value='#ff69b4')
                    btn_nm=gr.Button('이름·색 적용')
                    gr.Markdown('로봇이 없으면 [이동] 탭에서 **시뮬레이션** 을 '
                                '고르면 3D로 확인할 수 있습니다.')

        # ================= 오른쪽: 3D 뷰 + 상태 =================
        with gr.Column(scale=5):
            gr.HTML(f'<iframe src="http://localhost:{VIEWER_PORT}/" '
                    'style="width:100%;height:560px;border:1px solid #bbb;'
                    'border-radius:6px;background:#303030;"></iframe>')
            status=gr.Textbox(label='상태',lines=3,
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
    robot_move_chk.change(h_robot_move_mode,[robot_move_chk],
                          [robot_pos_txt,status])
    btn_robot_home.click(h_robot_home,None,[robot_pos_txt,status])
    peg_view.change(h_peg_view,[peg_view,peg_h],[peg_sel,status])
    peg_h.change(h_peg_height,[peg_h],[peg_sel,status])
    peg_sel.change(h_peg_pick,[peg_sel],[status])
    btn_peggo.click(h_peg_goto,[peg_sel,peg_h],[status])
    btn_pegpose.click(h_peg_save_pose,[peg_sel,peg_h,peg_pname],
                      [pose_sel,pose_for_seq,status])
    btn_place.click(lambda b,d:place_brick_on_docking(b,d)[1],
                    [brick_sel,dock_sel],[status])
    btn_grasp.click(do_grasp,[brick_sel],[status])
    btn_release.click(do_release,[dock_sel],[status])
    btn_gr_add.click(h_seq_grasp,[brick_sel],[seq_sel,status])
    btn_rl_add.click(h_seq_release,[dock_sel],[seq_sel,status])
    btn_nm.click(h_set_name,[nm_target,nm_text,nm_color],
                 [nm_target,brick_sel,dock_sel,status])
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
    cal_robot.change(h_cal_robot,[cal_robot],[ft_joint,status])
    btn_cal_off.click(h_cal_torque_off,[cal_robot],[status])
    btn_cal_cap.click(h_cal_capture_zero,[cal_robot],[status])
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
    exec_robot.change(h_exec_robot,[exec_robot],[status])
    btn_rstat.click(h_robot_status,None,[robot_stat])
    for _s,_b in zip(MACRO_SLOTS,macro_btns):
        _b.click(lambda s=_s: h_macro_go(s),None,[status])
    btn_mrec.click(h_macro_rec,[macro_slot],macro_btns+[status])
    btn_mclr.click(h_macro_clear,[macro_slot],macro_btns+[status])
    btn_save.click(h_save,None,[status])
    btn_load.click(h_load,None,[pose_sel,pose_for_seq,seq_sel,status])

if __name__=='__main__':
    demo.queue(default_concurrency_limit=8)
    # 7860부터 빈 포트 자동 탐색 (이전 실행이 포트를 잡고 있어도 OK)
    for port in range(7860,7880):
        try:
            demo.launch(server_name='0.0.0.0',server_port=port,share=False,
                        inbrowser=False,show_error=True)
            break
        except OSError:
            print(f"port {port} busy, trying {port+1}...")
