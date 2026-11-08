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
import os, sys, json, math, time, struct, threading, re
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
print(f"USB 자동 인식: SO-ARM={_SCAN0['sts'] or '없음 (꽂으면 연결할 때 다시 찾음)'}")
# STS3215 레지스터
ADDR_TORQUE=40; ADDR_P_ACCEL=41; ADDR_P_VEL=46; ADDR_GOAL=42; ADDR_PRESENT=56
ADDR_KP=21; ADDR_KD=22; ADDR_KI=23     # 위치 PID (공장값 32/32/0 = 이 팔엔 약함)
# 측정(j2, 2왕복): Kp32/Kd32/Ki0 -> 정지오차 18raw(1.58도)
#                 Kp64/Kd64/Ki4 -> 정지오차  4raw(0.35도), 진동 0
# Ki 가 중력으로 남는 정상편차(내릴 때만 커지는 오차)를 없애준다.
# 서보 PID. 가속을 올리면 관성이 커져 목표를 지나쳤다 돌아오는(오버슛)
# 현상이 생긴다 -> 감쇠(KD)를 함께 올려 잡는다. KI 는 중력으로 남는
# 마지막 오차를 없애는 용도(과하면 목표 근처에서 떠는 헌팅이 생김).
GAIN_KP=64; GAIN_KD=64; GAIN_KI=4      # 8/13 백업값 (그때 부드러웠다)
# ===== 이동 중 / 도착 직전 게인을 따로 쓴다 =====
# 위 값(Kp64/Ki6)은 '그 자리를 정확히 지키는' 데 맞춘 값이다. 이동 중에는
# 목표가 계속 앞서 나가 오차가 항상 남는데, Ki 가 그 오차를 쌓아 점점 세게
# 밀어붙이고 Kp 도 강해서 '따라잡았다 지나쳤다' 를 반복한다 = 덜덜 떨림.
#   -> 이동 중엔 부드럽게(낮은 Kp, Ki=0), 도착 직전에만 정밀하게(원래 값).
# 실측 비교(하강 60mm, 물컵 최대오차 / 떨림):
#   Kp64 Ki6 : 1.23도 / 6회   (예전값)
#   Kp32 Ki0 : 1.41도 / 6회   ← 약하면 중력에 처져서 더 나쁘다
#   Kp48 Ki2 : 0.97도 / 6회   ← 가장 좋음
#   Kp64 Ki0 : 1.58도 / 10회  ← Ki 를 빼면 떨림이 늘어난다
# 결론: '약하게' 가 아니라 '적당히' 가 답. Ki 는 조금 있어야 떨림이 준다.
MOVE_KP=64; MOVE_KD=64; MOVE_KI=4      # 8/13 백업값
HOLD_KP=64; HOLD_KD=64; HOLD_KI=4      # 8/13 백업값
ADDR_TORQUE_LIMIT=48; ADDR_MODE=33
SAFE_VEL=150; SAFE_ACCEL=8; TORQUE_LIMIT=250   # 1.5배속 (100->150)
# 가속 24 -> 8 (8/13 백업값). 24 는 관성이 커져 목표를 지나쳤다 돌아온다.
# ===== 부드럽게 움직이기 (덜덜거림 방지) =====
# 실측: STS3215 는 이 로봇 부하에서 약 23도/초 밖에 못 낸다.
# 그런데 경로는 33ms 마다 약 2도(=60도/초)씩 목표를 앞세운다.
# 모터가 못 따라가 오차가 쌓이고, 급하게 따라잡으며 덜덜 떤다.
#   -> 한 프레임의 이동량을 모터가 소화할 만큼으로 줄인다.
MAX_JOINT_DPS=20.0     # 실측 23도/초 에 여유를 둔 값
# 걸음 쪼개기는 껐다. 쪼갤수록 명령이 잦아져 모터가 매번 가속/감속을
# 다시 하며 오히려 더 떤다. (2026-08-23 사용자: "미친듯이 떨린다")
DENSIFY_ON=[False]
# 하강이 항상 이만큼 덜 내려간다(실측 +1.8mm) -> 그만큼 더 내려가게 보정
DESCEND_BIAS_MM=1.8
# 가속 8 -> 24 (2026-08-21): 명령을 50ms 마다 새로 주는데 가속 8 이면
# 최고속까지 1.5초가 걸려 '출발-중단-재출발' 을 초당 20번 반복 = 덜덜 떨림.
# 24 면 약 0.5초에 붙어 훨씬 부드럽다. 더 올리면(40+) 부드럽지만 관성이
# 커져 정밀도가 떨어질 수 있어 중간값부터 시작한다. (원래 값 8)
# 웨이포인트 추종: 실물이 이만큼 근접하면 다음 목표로 (너무 좁으면 느리고 넓으면 뒤처짐)
FOLLOW_TOL=60          # raw (약 5도)
FOLLOW_TIMEOUT=1.5     # 한 웨이포인트 최대 대기 (초)
# 구간 끝(자세 도달) 허용치. 작을수록 정확하지만 도달에 시간이 더 걸림
#   20 raw = 1.76도 -> EE 약 10mm / 6 raw = 0.53도 -> EE 약 3mm
# ★ STS3215 는 '데드존' 이 약 10카운트(0.88도) 다 — 그보다 작은 명령은
#   모터가 아예 무시한다. (실측: 2카운트 명령 -> 0 이동, 10 -> 9 이동)
#   END_TOL 을 데드존보다 작게 잡으면 영원히 도달하지 못하고, settle 이
#   계속 밀어붙이며 덜덜 떤다. 그래서 데드존보다 크게 잡는다.
DEADZONE=10            # 실측한 모터 데드존(카운트)
END_TOL=6              # 8/13 백업값
END_TIMEOUT=8.0
END_SETTLE=0.25        # 도착 후 진동이 가라앉기를 기다리는 시간(초)
# 웨이포인트 간 최대 관절 변화. 크면 경로가 잘 풀리지만 급가속 위험
JUMP_LIMIT_DEG=20
PER_ID_VEL={5:600}; PER_ID_TQ={5:250}                 # 그리퍼(ID 5, 10-29 까지 6)는 저속에선 안 움직임 · 11-06 (사용자) 열고 닫기 2배: 300->600
GRIP_ACCEL=40                 # 그리퍼 가속 (11-06: 10->40 — 속도 2배와 함께 올려야 걸리는 시간이 딱 절반)

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
def _urdf_joints(path):
    """URDF 에서 움직이는 관절 이름을 순서대로 읽는다.
    로봇마다 축 수가 다르므로(SO-ARM 6축 / UR15 7축+end) 하드코딩하지 않는다."""
    try:
        root=ET.parse(path).getroot()
        out=[]
        for j in root.findall('joint'):
            if (j.get('type') or '').lower() in ('revolute','continuous'):
                out.append(j.get('name'))
        return out or ['j1','j2','j3','j4','j5','j6']
    except Exception:
        return ['j1','j2','j3','j4','j5','j6']

JOINTS=_urdf_joints(BASE_URDF)
NJOINT=len(JOINTS)
DEAD_ID=0   # 모터가 없는 관절 자리표 (버스에 보내지 않는다)
def _make_j2id(urdf,n):
    """관절 -> 모터 ID.
    ★ SO-ARM 2026-10-29 재조립 ★ 실물 모터는 5개: j1~j4 = ID 1~4, 그리퍼 = ID 5.
      URDF 는 그대로 6관절이라 5번째 관절(옛 j5)은 0도 기계 고정 — 모터 없음(DEAD_ID).
      (j4 번호 5 -> 4, 그리퍼 번호 6 -> 5: sts_grip_id_6to5.py · 한계·위치 보정: sts_apply_1029.py)"""
    fam=os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(urdf))))
    if fam=='sts3215' and n==6: return [1,2,3,4,DEAD_ID,5]
    return list(range(1,n+1))
J2ID=_make_j2id(BASE_URDF,NJOINT)
print(f"관절 {NJOINT}개: {', '.join(JOINTS)}")
# 2026-07-27 STS3215 캘리브레이션
from sts_zero import ZERO_RAW as _Z
from sts_limits import LIMIT_RAW as _L
from sts_gripper import GRIPPER_CLOSE, GRIPPER_OPEN
ZERO_RAW=dict(_Z)
LIMIT_RAW=dict(_L)
# 영점이 관절 한계 밖이면 base(0도)가 갈 수 없는 자세가 된다 -> 즉시 경고
for _i,_v in list(ZERO_RAW.items()):
    _lim=LIMIT_RAW.get(_i)
    if _lim and not (_lim[0]<=_v<=_lim[1]):
        print(f"!! 영점 경고: ID{_i} 영점 {_v} 이 관절 한계 "
              f"{_lim[0]}~{_lim[1]} 밖입니다 — base 자세가 도달 불가입니다."
              " sts_zero.py.bak 으로 되돌리거나 영점을 다시 잡으세요")
DIR={1:-1, 2:-1, 3:-1, 4:-1, 5:-1, 6:-1}
from collections import defaultdict as _dd
DIR=_dd(lambda:1,DIR)   # 없는 축은 +1 (다른 로봇 대비)              # 전 관절 반전 (실측)
GRIP_ID=5        # 2026-10-29 그리퍼 번호 6 -> 5 (사용자: "j5 와 5 는 둘 다 그리퍼")
# ★ 모터가 없는 관절 (2026-10-28, 사용자 / 10-29 번호 정리) ★
#   옛 j5 자리는 0도에 기계적으로 고정한 '연결 블록' 이다. 형상(URDF)·기구학은 그대로 6관절이고
#   계산상 그 관절 = 0 이지만, 실물 통신에서는 아무것도 보내지 않고 읽지도 않는다 (ID = DEAD_ID).
#   (응답 없는 ID 를 읽으면 4번 재시도 ≈ 0.36초씩 멈추고, 영점 저장은 'ID 읽기 실패' 로 막혔다)
STS_DEAD_IDS={DEAD_ID}
_ARM_A_GRIP=(GRIP_ID,GRIPPER_CLOSE,GRIPPER_OPEN)   # A팔 그리퍼 번호·값 (B팔로 바꿨다가 돌아올 때 쓴다)
# ★ 2번 팔(B팔) 실물 (2026-11-02) ★ — 같은 선에 모터 11~15. 값은 sts_arm2.py (영점·범위·방향·그리퍼)
#   모터 번호로 나뉘어 있으니 영점·범위·방향 표에 그대로 더한다 (A팔 1~5 와 안 겹친다).
#   팔을 바꾸면(arm_switch) 관절→모터 번호(J2ID)·그리퍼 번호·그리퍼 값만 그 팔 것으로 갈아끼운다 (_arm_motors).
try:
    import sts_arm2 as _ARM2
    ZERO_RAW.update(_ARM2.ZERO_RAW); LIMIT_RAW.update(_ARM2.LIMIT_RAW); DIR.update(_ARM2.DIR)
    PER_ID_VEL.update(_ARM2.PER_ID_VEL); PER_ID_TQ.update(_ARM2.PER_ID_TQ)
except Exception as _e:
    _ARM2=None; print("2번 팔 값(sts_arm2.py) 없음 — B팔은 3D 만:",_e)
def jname(k):
    """화면·로그에 보일 관절 이름. SO-ARM 은 모터 번호와 같게 (j1~j4, 그리퍼 j5), 고정 관절은 '고정'.
    다른 로봇은 그대로 j1, j2, ..."""
    if J2ID==list(range(1,len(J2ID)+1)) or k>=len(J2ID): return f"j{k+1}"
    if J2ID[k] in STS_DEAD_IDS: return '고정'
    i=J2ID[k]
    if _ARM2 is not None and i>_ARM2.ID_OFFSET: i-=_ARM2.ID_OFFSET   # B팔(11~15)도 j1~j4 · 그리퍼 j5 로 (11-02)
    return f"j{i}"
if J2ID!=list(range(1,NJOINT+1)):      # SO-ARM: URDF 이름과 모터 번호가 다르다
    print("  실물 모터: "+', '.join(f"{jname(k)}{'(그리퍼)' if k==NJOINT-1 else ''}=ID{i}"
                                   for k,i in enumerate(J2ID) if i not in STS_DEAD_IDS)
          +f" · URDF {JOINTS[J2ID.index(DEAD_ID)]} 는 0도 고정 (모터 없음)")
# 프로젝트(자세/시퀀스/보정/녹화)는 URDF 패키지 폴더 안에 저장한다.
#   -> URDF 를 바꾸면 그 URDF 의 데이터가 따라오고, 서로 섞이지 않는다.
#   -> SolidWorks 에서 URDF 를 재익스포트해도 calib.json 은 지워지지 않는다.
URDF_PKG_DIR=os.path.dirname(os.path.dirname(BASE_URDF))   # .../<패키지>/
PROJECT_FILE=os.path.join(URDF_PKG_DIR,'calib.json')
_LEGACY_PROJECT='/home/drboom/2026_project/Industry_gyu_arm/arm_studio_project.json'
print(f"보정 데이터: {PROJECT_FILE}")

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def _urdf_limits(path):
    """URDF 에 적힌 관절 한계 (rad). 실물 모터가 없는 로봇(UR15 등)에 쓴다."""
    out={}
    try:
        root=ET.parse(path).getroot()
        for j in root.findall('joint'):
            if (j.get('type') or '').lower() not in ('revolute','continuous'): continue
            l=j.find('limit')
            if l is None: out[j.get('name')]=(-math.pi,math.pi); continue
            lo=float(l.get('lower',-math.pi)); hi=float(l.get('upper',math.pi))
            if abs(hi-lo)<1e-6: lo,hi=-math.pi,math.pi
            out[j.get('name')]=(min(lo,hi),max(lo,hi))
    except Exception: pass
    return out
URDF_LIMITS=_urdf_limits(BASE_URDF)

def jr_limits(jidx):
    """관절 가동범위(rad).
    실물 모터(STS3215) 캘리브레이션이 있는 축은 그 값을,
    없는 축(다른 로봇의 7·8번 등)은 URDF 에 적힌 한계를 쓴다."""
    # ★ 실물이 있는 로봇에서만 모터 캘리브레이션을 쓴다 ★
    #   UR15 는 실물이 없는데 모터 ID(1~6)가 SO-ARM 과 겹쳐서, SO-ARM 의
    #   가동범위(242도)를 뒤집어써 '360도 로봇인데 못 도는' 문제가 났다.
    #   (사용자 지적: "오른쪽으로 갈 때 쓸데없이 x 막힘이 뜬다")
    nm=JOINTS[jidx] if jidx<len(JOINTS) else ''
    # real_robot_ok() 는 아래에서 정의되므로 같은 판단을 직접 한다
    _fam=os.path.basename(os.path.dirname(URDF_PKG_DIR))
    if _fam not in ('sts3215','dynamixel'):
        return URDF_LIMITS.get(nm,(-math.pi,math.pi))
    i=J2ID[jidx]
    if i not in LIMIT_RAW or i not in ZERO_RAW:
        return URDF_LIMITS.get(nm,(-math.pi,math.pi))
    lo,hi=LIMIT_RAW[i]
    a=DIR[i]*(lo-ZERO_RAW[i])*math.pi/2048.0
    b=DIR[i]*(hi-ZERO_RAW[i])*math.pi/2048.0
    return (min(a,b),max(a,b))
def rad_to_raw(jidx,rad):
    i=J2ID[jidx]
    if i not in ZERO_RAW or i not in LIMIT_RAW:
        return 0,i                     # 실물 모터가 없는 축 (시뮬 전용)
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
def _dead_joint(k):
    """선을 연결하지 않은 고정 관절인가 (SO-ARM 의 j5). 관절 가동범위는 그대로 두고 IK 에서만 잠근다
    (가동범위를 0~0 으로 하면 슬라이더·도달표 등 10여 곳이 최소=최대로 깨진다 — 시작 시 오류 실측)."""
    fam=os.path.basename(os.path.dirname(URDF_PKG_DIR))
    return fam=='sts3215' and k<len(J2ID) and J2ID[k] in STS_DEAD_IDS

def build_chain():
    links=[OriginLink()]; active=[False]
    for k,jn in enumerate(JOINTS):
        b=BASE_DEF[jn]
        links.append(URDFLink(name=jn,origin_translation=list(b['xyz']),
                              origin_orientation=list(b['rpy']),rotation=list(b['axis']),
                              bounds=jr_limits(k),joint_type='revolute'))
        active.append(not _dead_joint(k))   # 선 없는 j5 는 IK 가 돌리지 않는다 (초기값 = 0 유지)
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

J6_TOOL_REF=0.0   # 툴 중심 평가용 그리퍼 기준각 (axis_7 은 URDF 0자세에서 잰 값)
def tool_pos(q):
    """월드 기준 툴 중심 좌표 (axis_7). j6 프레임에서 TOOL_OFF 만큼 이동.

    axis_7 은 '움직이는 죠' 링크에 붙어 있어 그리퍼를 열면 실제로는
    60mm 나 옆으로 돌아간다. 하지만 잡는 축(펜이 올 자리)은 다물었을 때
    기준이므로, 그리퍼가 열려 있든 닫혀 있든 j6 를 기준각으로 고정해
    평가한다. (안 그러면 '열기' 뒤의 이동/하강이 전부 60mm 어긋난다)"""
    q=list(q); q[GRIP_IDX]=J6_TOOL_REF
    T=CHAIN.forward_kinematics([0]+list(q))
    return l2w(T[:3,3]+T[:3,:3]@TOOL_OFF)
# ===== 로봇 베이스 위치 (플레이트 홈 스냅) =====
# 로봇을 플레이트 위 원하는 타일에 재배치할 수 있다.
#   - 내부 기구학(CHAIN)은 항상 로봇 로컬 좌표로 푼다
#   - fk 결과는 월드로 (+BASE_OFF), IK 목표는 로컬로 (-BASE_OFF) 변환
#   -> 나머지 코드(맵/충돌/간편이동)는 전부 월드 좌표라 수정 불필요
BASE_OFF=np.zeros(3)          # 월드 기준 로봇 base 오프셋 (z 는 항상 0)
BASE_YAW=[0.0]                # 월드 기준 로봇 회전 (rad, 90도 단위. 2핀 배치용)
def _yawR():
    c,si=math.cos(BASE_YAW[0]),math.sin(BASE_YAW[0])
    return np.array([[c,-si,0],[si,c,0],[0,0,1]])
def l2w(p):  return _yawR()@np.asarray(p,float)+BASE_OFF
def w2l(p):  return _yawR().T@(np.asarray(p,float)-BASE_OFF)
def w2l_v(v):return _yawR().T@np.asarray(v,float)   # 방향 벡터 (평행이동 없음)
def _base_T():
    T=np.eye(4); T[:3,:3]=_yawR(); T[:3,3]=BASE_OFF; return T
def fk(q):
    T=CHAIN.forward_kinematics([0]+list(q))
    return l2w(T[:3,3]), _yawR()@T[:3,:3]
def fk_points(q):
    frames=CHAIN.forward_kinematics([0]+list(q),full_kinematics=True)
    return np.array([l2w(f[:3,3]) for f in frames])

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
    groups=[[0],[1,2,3],[4],[5]] if level else [[j] for j in range(NJOINT)]
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
        sol=CHAIN.inverse_kinematics(target_position=w2l(pt),target_orientation=w2l_v(zt),
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
        sol=CHAIN.inverse_kinematics(target_position=w2l(pos),target_orientation=w2l_v(zt),
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
            sol=CHAIN.inverse_kinematics(target_position=w2l(pt),target_orientation=w2l_v(zt),
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
# 물컵 규칙에서 '돌리면 무조건 기울어지는 축'.
#   SO-ARM: j5 = 인덱스 4.
#   다른 로봇(UR15 등)은 축 구성이 달라 이 규칙을 그대로 못 쓴다
#   -> 축이 6개가 아니면 물컵 규칙을 끄고 IK 가 자유롭게 풀게 둔다
#      (UR15 는 axis_end 방향을 직접 맞추는 방식으로 따로 제어한다)
J5_IDX=4
# 그리퍼(마지막 축) 인덱스. 로봇마다 축 수가 달라 고정할 수 없다.
#   SO-ARM: j6 = 5   ·   UR15: end = 7
GRIP_IDX=NJOINT-1
LEVEL_ALWAYS=(NJOINT==6)      # 6축(SO-ARM)에서만 물컵 규칙 적용
if not LEVEL_ALWAYS:
    print(f"  축 {NJOINT}개 로봇 — 물컵 규칙(j5 고정)은 끕니다 "
          "(축 구성이 달라 그대로 쓸 수 없음)")

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
        return [k for k in range(NJOINT) if lvl[k]]
    except Exception:
        return []

def dxl_base_pose():
    """DXL 관절 공간에서의 base 자세. 영점이 곧 base 이므로 전부 0.
    (dxl_zero.py 를 손으로 다시 잡으면 그 자세가 새 기준이 된다)"""
    return [0.0]*NJOINT

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

def densify(way,dt,sp=1.0):
    """모터가 못 따라갈 만큼 큰 걸음은 잘게 쪼갠다.

    실측: STS3215 는 이 로봇 부하에서 약 23도/초. 그런데 경로는
    한 프레임(≈33ms)에 2도씩 목표를 앞세워 60도/초를 요구한다.
    모터가 뒤처지다 급히 따라잡으며 '덜덜' 떨고, 그 사이 j2+j3+j4 합도
    흐트러진다(물컵이 기운다).
      -> 한 걸음이 (모터가 갈 수 있는 양) 이하가 되도록 사이를 채운다.
    반환: (새 경로, 새 프레임간격)
    """
    if not DENSIFY_ON[0] or len(way)<2: return way,dt
    # 한 프레임에 갈 수 있는 양. 단, 모터 데드존(10카운트)보다 확실히 커야
    # 한다. 데드존 언저리(11 같은)로 쪼개면 매번 조금씩 뒤처졌다 툭 따라잡아
    # 미세하게 떤다. 그래서 아래 하한을 둔다.
    lim=math.radians(MAX_JOINT_DPS)*(dt/max(0.1,sp))   # 한 프레임에 갈 수 있는 rad
    lim=max(lim,DEADZONE*2.5*math.pi/2048.0)           # 데드존의 2.5배 이상
    if lim<=0: return way,dt
    out=[list(way[0])]
    for a,b in zip(way[:-1],way[1:]):
        mx=max(abs(y-x) for x,y in zip(a,b))
        n=int(math.ceil(mx/lim)) if mx>lim else 1
        n=min(n,12)                                    # 너무 잘게 쪼개지 않게
        for k in range(1,n+1):
            t=k/n
            out.append([x+(y-x)*t for x,y in zip(a,b)])
    # 프레임이 늘어난 만큼 간격을 줄인다. 안 그러면 전체가 그 배수만큼
    # 느려지고, 명령 간격이 벌어져 오히려 뚝뚝 끊긴다.
    if len(out)>len(way):
        dt=dt*(len(way)-1)/max(1,len(out)-1)
    return out,dt

def lock_j5(way,qa):
    """경로 전체에서 j5 를 출발 각도로 고정 (기울기 원인 제거)."""
    for f in way: f[J5_IDX]=qa[J5_IDX]
    return way

def ramp_j5(way,qa,qb):
    """물컵 규칙을 지키면서 '목표 자세의 j5' 로 가는 방법.

    lock_j5 는 j5 를 출발값에 묶는다. 경로 중 기울지 않게 하려는 것이지만,
    등록 자세로 이동할 때 쓰면 그 자세의 j5 에 영영 도달할 수 없다
    (base 로 갔는데 j5 가 191도 어긋나던 버그).

    등록 자세들은 애초에 j5=0(수평)으로 저장되므로, 출발 j5 에서 목표 j5 로
    부드럽게 보간하면 물컵 규칙을 지키면서 목표에도 도달한다.
    목표가 수평이 아닌 자세(j5≠0)면 그건 사용자가 의도한 것이므로 그대로 따른다.
    """
    if not way: return way
    a=qa[J5_IDX]; b=qb[J5_IDX]
    n=max(1,len(way)-1)
    for k,f in enumerate(way):
        t=k/n; t=t*t*(3-2*t)              # 부드럽게 (급격한 기울기 변화 방지)
        f[J5_IDX]=a+(b-a)*t
    way[-1][J5_IDX]=b
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
            sol=CHAIN.inverse_kinematics(target_position=w2l(pt),
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
SEED_MAX=6      # IK 초기값 개수 상한 (많을수록 정확하지만 느려진다)

def _seed_list(extra=None):
    """IK 초기값 목록. 등록 자세가 많아질수록 IK 가 느려지므로
    (블록 자세 등록 기능 때문에 수십 개가 될 수 있다) 가까운 것 위주로 자른다.
    - extra(직전 자세)가 가장 좋은 씨앗이라 항상 맨 앞
    - base 는 안전한 기본값이라 항상 포함
    - 나머지는 extra 와 관절각이 가까운 순서로 SEED_MAX 까지"""
    seeds=[]
    if extra is not None: seeds.append(list(extra))
    rest=[list(p['angles']) for p in S['poses']]
    base=next((list(p['angles']) for p in S['poses'] if p['name']=='base'),None)
    if base is not None: seeds.append(base)
    if extra is not None and len(rest)>SEED_MAX:
        e=list(extra)
        rest.sort(key=lambda a: sum((x-y)**2 for x,y in zip(a[:5],e[:5])))
    for a in rest:
        if len(seeds)>=SEED_MAX: break
        if not any(all(abs(x-y)<1e-9 for x,y in zip(a[:5],b[:5])) for b in seeds):
            seeds.append(a)
    return seeds or [[0.0]*NJOINT]

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
    mask=[False]*NJOINT
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
                        f"{[JOINTS[k] for k in range(NJOINT) if lvl[k]]}")
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
    p0,R0=_chain_ee(chain,[0.0]*NJOINT)
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
    seeds.append([0.0]*NJOINT)
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
    tgt=np.asarray(w2l(pt),float)
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
    # 수렴 검증: 3mm 넘게 어긋난 해를 '성공'이라고 돌려주면 로봇이 허공에
    # 서는 사고가 난다 (경계 밖 목표에서 실제 발생) -> 정직하게 실패 처리
    if best is not None and float(np.linalg.norm(tgt-tool_pos(best)))>0.003:
        return None
    return best

# ===== 변형 물컵 규칙 (UR15 등) =====
# SO-ARM 은 'j5 를 안 돌린다' 로 수평을 지켰지만, 축 구성이 다른 로봇에는
# 그 방법을 못 쓴다. 대신 **끝단(axis_end)의 회전축을 바닥과 평행하게**
# 유지한다 — 사용자가 정한 이 로봇의 규칙.
#   URDF 상 end 조인트 축 = (0,-1,0) 이고 전 관절 0도에서 바닥과 평행하다.
END_AXIS_LOCAL=np.array([0.0,-1.0,0.0])   # end 프레임에서의 회전축
LEVEL_END=(NJOINT!=6)                     # 6축(SO-ARM)이 아니면 이 규칙 사용

def end_axis_world(q):
    """지금 자세에서 axis_end 회전축의 월드 방향."""
    T=CHAIN.forward_kinematics([0]+list(q)[:len(CHAIN.links)-1])
    return T[:3,:3]@END_AXIS_LOCAL

def end_tilt_deg(q):
    """axis_end 가 바닥(XY평면)에서 몇 도 벗어났는지. 0 이면 완전 평행."""
    v=end_axis_world(q)
    n=np.linalg.norm(v)
    if n<1e-9: return 0.0
    return abs(math.degrees(math.asin(max(-1,min(1,v[2]/n)))))

def ik_general(pt,seeds):
    """축 구성이 SO-ARM 과 다른 로봇(UR15 등)을 위한 IK.

    변형 물컵 규칙: **axis_end 회전축을 바닥과 평행하게** 유지한다.
    ikpy 로 위치를 풀되, 끝단이 기울어진 해는 버리고 다른 초기값으로 다시
    시도한다 (여러 해 중 규칙을 지키는 것을 고른다)."""
    tgt=np.asarray(pt,float)
    ch=CHAIN
    n=len(ch.links)
    TOL=8.0                       # 이 각도 안이면 '평행' 으로 본다

    # !! 7축 같은 여유자유도 로봇은 같은 위치에 갈 수 있는 자세가 무수히 많다.
    #    매번 아무 해나 고르면 5mm 움직였는데 팔 모양이 통째로 바뀐다
    #    (실측: 관절이 한 번에 74도씩 점프).
    #    -> '지금 자세에서 가장 조금 움직이는 해' 를 고른다.
    cur=list(CURRENT_Q)[:NJOINT] if len(CURRENT_Q)>=NJOINT else [0.0]*NJOINT
    def move_cost(q):
        return sum(abs(a-b) for a,b in zip(q[:NJOINT],cur))

    # 현재 자세를 첫 씨앗으로 (여기서 출발해야 '조금만 움직이는' 해가 나온다)
    seeds=[cur]+[x for x in seeds]
    cands=[]
    for sd in seeds:
        init=[0.0]*n
        for k,v in enumerate(list(sd)[:NJOINT]):
            if k+1<n: init[k+1]=float(v)
        try:
            sol=ch.inverse_kinematics(target_position=w2l(tgt),
                                      initial_position=init)
        except Exception:
            continue
        q=list(sol[1:1+NJOINT])
        got,_=fk(q)
        if np.linalg.norm(got-tgt)>0.005: continue
        t=end_tilt_deg(q) if LEVEL_END else 0.0
        cands.append((q,t,move_cost(q)))
    if not cands: return None
    # 규칙(끝단 평행)을 지키는 해 중에서 '가장 조금 움직이는' 것
    ok=[c for c in cands if c[1]<TOL]
    pick=min(ok or cands,key=lambda c:(0 if c[1]<TOL else c[1])+c[2]*0.6)
    return pick[0]

def ik_level(pt,seeds,j5,j6):
    """수평 유지 IK (j5/j6 잠금). 초기값 여러 개 시도 -> 관절6개 or None.
    (초기값이 나쁘면 도달 가능한 점도 실패 판정이 나므로 다중 시도 필수)"""
    if not LEVEL_ALWAYS:              # 물컵 규칙을 안 쓰는 로봇
        return ik_general(pt,seeds)
    ch=_chain_level(); tgt=np.asarray(pt,float)
    for s in seeds:
        init=[0.0]+list(s); init[5]=j5; init[6]=j6
        try:
            sol=ch.inverse_kinematics(target_position=w2l(tgt),
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
# 이 표를 매번 새로 만들면 경로 계산이 10초 넘게 걸린다 (IK 1,000회+).
# 로봇 자세(위치+회전)별로 디스크에 보관해 재시작해도 즉시 쓴다.
_MAXZ_FILE=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'maxz_cache.json')
_MAXZ_KEY=[None]
_MAXZ_DIRTY=[False]
def _maxz_key():
    # 로봇 이름을 반드시 넣는다. 안 넣으면 SO-ARM 과 UR15 가 같은 자리에서
    # 서로의 도달상한을 읽어가 엉뚱한 높이로 판단한다.
    _r=os.path.basename(URDF_PKG_DIR) if 'URDF_PKG_DIR' in globals() else '?'
    # 관절 가동범위도 넣는다 — 영점·한계를 다시 잡으면 (10-29 j4 교체) 예전 표가 틀리므로 새로 계산
    _lim=','.join(f"{math.degrees(a):.0f}:{math.degrees(b):.0f}" for a,b in (jr_limits(k) for k in range(NJOINT)))
    return f"{_r}|{BASE_OFF[0]:.4f},{BASE_OFF[1]:.4f},{BASE_YAW[0]:.4f}|{_lim}"
def _maxz_load():
    """지금 로봇 자세에 해당하는 도달상한 표를 디스크에서 불러온다."""
    k=_maxz_key()
    if _MAXZ_KEY[0]==k: return
    _MAXZ_KEY[0]=k; _MAXZ_R.clear()
    try:
        with open(_MAXZ_FILE) as f: d=json.load(f)
        for sk,v in (d.get(k) or {}).items():
            x,y=sk.split(','); _MAXZ_R[(float(x),float(y))]=v
        if _MAXZ_R: print(f"도달상한 캐시 {len(_MAXZ_R)}칸 불러옴 (경로 계산 빨라짐)")
    except Exception: pass
def _maxz_save():
    try:
        d={}
        try:
            with open(_MAXZ_FILE) as f: d=json.load(f)
        except Exception: pass
        d[_maxz_key()]={f"{k[0]},{k[1]}":v for k,v in _MAXZ_R.items()}
        with open(_MAXZ_FILE,'w') as f: json.dump(d,f)
    except Exception as e:
        print('도달상한 캐시 저장 실패:',e)
# 주의: 반경만으로 캐시하면 안 된다 - 손목 오프셋 때문에 방향마다 상한이 다르다
_Z_LO,_Z_HI=0.03,0.55
def maxz_at(x,y,seed=None,j5=None,j6=0.0,hint=None):
    """(x,y) 에서 수평 유지로 도달 가능한 최대 z. 없으면 None.
    hint: 이웃 지점의 상한 (거기서부터 스캔해 IK 횟수 절약)."""
    _maxz_load()
    rk=(round(x,2),round(y,2))
    if rk in _MAXZ_R: return _MAXZ_R[rk]
    _MAXZ_DIRTY[0]=True
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
        p,_=fk([0.0]*NJOINT); _AZIM0=math.atan2(p[1]-cy,p[0]-cx)  # j1=0 EE 방위각
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
def _fill_mesh2d(g,path,scale,off=(0.0,0.0),yaw=0.0):
    """메시를 위에서 본 모양으로 격자 g 에 채운다. off=월드 평행이동(x,y)."""
    try: mm=trimesh.load(path)
    except Exception: return
    v=np.asarray(mm.vertices,float)*scale
    if yaw:
        c,si=math.cos(yaw),math.sin(yaw)
        v[:,:2]=v[:,:2]@np.array([[c,si],[-si,c]])
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
        _off=e.get('offset',(0,0,0))
        _fill_mesh2d(mk,path,scale,off=(_off[0],_off[1]),yaw=e.get('yaw',0.0))
        per[mid]=mk; g|=mk
    _OBS['ids']=per
    # 로봇 base 발자국 (m 단위 링크 메시, base_link 프레임 그대로)
    gb=np.zeros((G2_N,G2_N),bool)
    bp=os.path.join(MESH_DIR,'base_link.STL')
    if os.path.exists(bp): _fill_mesh2d(gb,bp,1.0,off=(BASE_OFF[0],BASE_OFF[1]),yaw=BASE_YAW[0])
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
    #   출발지·목적지 자신은 장애물이 아니다 — 그 위에서 집고 놓아야 하므로
    #   반드시 그 자리를 지난다. 예전엔 "⚠충돌:목적지" 가 매번 떴다.
    build_obs_grid(); ign=set(ignore or ())
    for _ep in (a,b):
        _oid=_ep[len(GOTO_PREFIX):] if str(_ep).startswith(GOTO_PREFIX) else str(_ep)
        ign.add(_oid)
        for _d in docking_ids():            # 물체 이름이면 그 물체가 놓인 도킹도
            _t=dock_socket_world(_d)
            _bp=(BRICKS.get(_oid) or {}).get('pos')
            if _bp is not None and _t is not None and \
               math.hypot(_bp[0]-_t[0],_bp[1]-_t[1])<0.025:
                ign.add(_d)
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
            f=[st[i]+(qb[i]-st[i])*t for i in range(NJOINT)]
            f[4]=j5; f[GRIP_IDX]=j6
            way.append(f)
    way[-1]=list(qb); way[-1][4]=j5; way[-1][5]=j6
    if lifted[0]: how+=f' · 저지대 {lifted[0]}점 상향'
    return (way,0.06),how

# ================= 터미널 로그 =================
LOG_ON=True
def LOG(msg):
    """콘솔 로그. start_pro.sh 가 tee 로 /tmp/arm_studio.log 에도 남긴다.
    버튼 클릭 시각을 알 수 있게 앞에 시간을 붙인다 (원인 추적용)."""
    if not LOG_ON: return
    t=time.strftime('%H:%M:%S')
    for i,line in enumerate(str(msg).split('\n')):
        print((f"{t} {line}" if i==0 and line.strip() else line),flush=True)

# 진행 상황: 콘솔 + 웹 상태창(폴링)에 동시 표시
PROG_STATE={'text':'대기 중','t':0.0}

# ===== 실행 위치 추적 (타임바 / 블록 하이라이트용) =====
# blk  = 지금 실행 중인 S['seq'] 인덱스 (없으면 -1)
# sub  = 그 블록 안의 세부 단계 이름 ('열기'/'하강'/'닫기'/'상승' 등)
# t0   = 이번 재생 시작 시각, done = 끝난 블록 수, total = 전체 블록 수
# elapsed_at = 블록별 누적 경과(초) — 슬라이더가 시간축을 그릴 수 있게
RUN_AT={'blk':-1,'sub':'','t0':0.0,'done':0,'total':0,'running':False,
        'marks':[],'speed':1.0}
def _run_begin(total,speed=1.0):
    RUN_AT.update({'blk':-1,'sub':'','t0':time.time(),'done':0,
                   'total':int(total),'running':True,'marks':[],
                   'speed':float(speed)})
def _run_at(blk,sub=''):
    """지금 실행 중인 블록/세부 단계를 기록 (화면이 폴링해서 읽는다)."""
    if RUN_AT.get('blk')!=blk and RUN_AT.get('blk',-1)>=0:
        RUN_AT['marks'].append({'blk':RUN_AT['blk'],
                                'at':round(time.time()-RUN_AT['t0'],2)})
        RUN_AT['done']=RUN_AT.get('done',0)+1
    RUN_AT['blk']=blk; RUN_AT['sub']=sub
def _run_end():
    RUN_AT.update({'blk':-1,'sub':'','running':False})
def PROG(msg):
    PROG_STATE['text']=msg
    LOG(f"  ... {msg}")
def BTN(name,detail=''):
    """버튼 클릭 로그. 무엇을 눌렀고 무엇을 하는지 남긴다."""
    LOG(f"\n>>> [버튼] {name}" + (f"  ({detail})" if detail else ""))
    PROG_STATE['text']=f"[{name}] {detail}" if detail else f"[{name}]" 
def LOG_TARGET(raws,tag=''):
    s='  '.join(f"{jname(J2ID.index(i))}:{v}" for i,v in sorted(raws.items()))
    LOG(f"  [목표] {tag} {s}")
def LOG_COMPARE(want,got,tag=''):
    """목표 raw 와 실제 raw 비교 출력."""
    parts=[]
    for i in sorted(want):
        w=want[i]; g=got.get(i)
        if g is None: parts.append(f"{jname(J2ID.index(i))}:{w}/--")
        else:
            d=g-w
            mark='' if abs(d)<=25 else ('!' if abs(d)<=100 else '!!')
            parts.append(f"{jname(J2ID.index(i))}:{w}->{g}({d:+d}){mark}")
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
      if i in STS_DEAD_IDS: return True        # 선 없는 모터: 보내지 않는다 (기다림 없음)
      with IO_LOCK:
        for _ in range(3):
            try:
                if self.packet.write1ByteTxRx(self.port,i,addr,int(v))[0]==0: return True
            except Exception: pass
            time.sleep(0.03)
        return False
    def _w2(self,i,addr,v):
      if i in STS_DEAD_IDS: return True
      if addr==ADDR_GOAL and 'ACTIVE_ARM' in globals() and _arm_block_real(): return False   # 3D 지금 팔이 실물 팔이 아니면 움직이지 않는다 (10-30)
      with IO_LOCK:
        for _ in range(3):
            try:
                if self.packet.write2ByteTxRx(self.port,i,addr,int(v))[0]==0: return True
            except Exception: pass
            time.sleep(0.03)
        return False
    def _r2(self,i,addr):
      if i in STS_DEAD_IDS: return None        # 선 없는 모터: 읽지 않는다 (read() 는 0도로 둔다)
      with IO_LOCK:
        for _ in range(4):
            try:
                time.sleep(0.04)
                v,r,_=self.packet.read2ByteTxRx(self.port,i,addr)
                if r==0: return v
            except Exception: pass
            time.sleep(0.05)
        return None
    def set_gain(self,kp,kd,ki):
        """위치 PID 게인을 바꾼다 (그리퍼 제외).
        같은 값이면 통신하지 않는다(불필요한 지연 방지)."""
        if getattr(self,'_gain',None)==(kp,kd,ki): return
        for i in J2ID:
            if i==GRIP_ID: continue
            self._w1(i,ADDR_KP,kp); self._w1(i,ADDR_KD,kd); self._w1(i,ADDR_KI,ki)
        self._gain=(kp,kd,ki)

    def move_gain(self):
        """이동 중: 부드럽게 (덜덜 떨림 방지)"""
        self.set_gain(MOVE_KP,MOVE_KD,MOVE_KI)

    def hold_gain(self):
        """도착 직전/정지: 정밀하게 (자세 유지)"""
        self.set_gain(HOLD_KP,HOLD_KD,HOLD_KI)

    def time_profile(self,on,sp=1.0):
        """STS 는 시간기반 프로파일이 없으므로 속도/가속 설정으로 대체.
        sp: 속도 배수 (UI 선택). 모터 속도에도 반영해야 실제로 빨라짐.

        ★ 덜덜거림 ★ 웨이포인트는 33ms 마다 약 2도씩 온다(=약 60도/초).
        모터 속도를 그보다 훨씬 크게 두면 2도를 순식간에 가고 남는 시간은
        멈춰 있다가 다음 목표에 또 급출발한다 -> 33ms 주기의 '가다서다'.
        그래서 속도를 '한 프레임에 갈 만큼' 으로 맞춰 이어지게 한다."""
        for i in J2ID:
            v=int(min(3000,max(30,PER_ID_VEL.get(i,SAFE_VEL)*sp)))
            a=int(min(60,max(2,SAFE_ACCEL*sp)))
            self._w1(i,ADDR_MODE,0)
            self._w2(i,ADDR_TORQUE_LIMIT,PER_ID_TQ.get(i,TORQUE_LIMIT))
            self._w1(i,ADDR_P_ACCEL,a)
            self._w2(i,ADDR_P_VEL,v)
        # 게인은 move_gain()/hold_gain() 이 관리한다 (여기서 덮어쓰지 않는다)
        self._gain=None
    def torque(self,on):
        for i in J2ID: self._w1(i,ADDR_TORQUE,1 if on else 0)
    def send(self,q,log=False,tag='',with_grip=False,only=None):
        """관절 목표 전송.
        그리퍼(j6)는 기본적으로 '보내지 않는다' -> URDF/자세 값과 무관하게
        ◆열기/닫기 블록으로만 바뀌고 그 상태를 유지한다.
        with_grip=True 일 때만 j6 도 함께 보낸다(그리퍼 블록 전용).
        only=[관절인덱스] 면 그 관절에만 보낸다(미세조정 - 한 축만 움직이기).

        ★ 모터 5개에 '한 번에'(SyncWrite) 보낸다 ★
        하나씩 보내면 j1 이 움직이기 시작할 때 j4 는 아직 옛 목표라
        그 사이 j2+j3+j4 합이 깨진다(=물컵이 기운다).
        한 패킷으로 동시에 보내면 모두 같은 순간에 새 목표를 받는다."""
        raws={}
        for jidx in range(NJOINT):
            if jidx==5 and not with_grip: continue   # j6 제외
            if only is not None and jidx not in only: continue
            raw,i=rad_to_raw(jidx,q[jidx])
            if i in STS_DEAD_IDS: continue           # 선 없는 모터 (j5 고정)
            raws[i]=raw
        if raws:
            if only is None: self.move_gain()   # 이동 중엔 부드럽게
            self._sync_goal(raws)
        if log: LOG_TARGET(raws,tag)
        return raws

    def glide(self,q_from,q_to,secs=None,watch=None):
        """★ 부드럽게 이동 ★ 목표를 '한 번만' 주고 모터가 알아서 가게 둔다.

        50ms 마다 새 목표를 던지면 모터는 그때마다 가속을 다시 시작한다.
        초당 20번 가속/감속을 반복 = 미친듯이 덜덜 떨린다.
        STS3215 는 내부에 사다리꼴 속도 프로파일이 있어서, 목표 하나만
        주면 스스로 부드럽게 가속->등속->감속 한다. 그걸 살린다.

        모든 축이 같이 도착하도록 이동량에 비례해 속도를 나눈다
        (물컵 규칙: j2+j3+j4 합이 이동 내내 유지된다).
        """
        d=[abs(rad_to_raw(j,q_to[j])[0]-rad_to_raw(j,q_from[j])[0])
           for j in range(min(5,NJOINT))]
        mx=max(d) if d else 0
        if mx<2: return True
        # 실측(2026-08-23): 속도값 ≈ 카운트/초 (100 -> 약 300, 800 -> 약 410)
        # 너무 느리면(50) 급격히 느려지므로 하한을 둔다.
        cps=SAFE_VEL*SIM_SPEED[0]*2.0           # 기준 속도(카운트/초)
        secs=secs or max(0.35,mx/max(50.0,cps))
        for j in range(min(5,NJOINT)):
            i=J2ID[j]
            if i==GRIP_ID or i in STS_DEAD_IDS: continue
            # 이 축이 secs 안에 d[j] 카운트를 가려면 필요한 속도(≈카운트/초)
            # 실측: 속도 50 -> 54카운트/초, 100 -> 297카운트/초.
            # 50~60 근처가 절벽이라 그 아래로 내려가면 급격히 느려져
            # 팔을 끄는 느낌(덜덜)이 난다. 하한을 절벽 위로 둔다.
            v=int(max(120,min(1500,d[j]/max(0.05,secs))))
            self._w2(i,ADDR_P_VEL,v)
            self._w1(i,ADDR_P_ACCEL,SAFE_ACCEL)
        # 목표는 딱 한 번 — 위치와 속도를 함께 보낸다
        raws={}; vel={}
        for j in range(min(5,NJOINT)):
            i=J2ID[j]
            if i==GRIP_ID or i in STS_DEAD_IDS: continue
            raws[i]=rad_to_raw(j,q_to[j])[0]
            vel[i]=int(max(120,min(1500,d[j]/max(0.05,secs))))
        if not self._sync_goal_timed(raws,vel):
            self._sync_goal(raws)
        # 도착까지 기다리며 3D 화면만 갱신
        t0=time.time()
        while time.time()-t0<secs*1.8+0.5:
            if STOP.is_set(): return False
            el=min(1.0,(time.time()-t0)/max(0.05,secs))
            if watch: watch(el)
            if el>=1.0:
                p=self._r2fast(J2ID[0],ADDR_PRESENT)
                if p is not None and abs(p-raws.get(J2ID[0],p))<DEADZONE*2: break
            time.sleep(0.03)
        return True

    def sync_speed(self,q_from,q_to,dt):
        """★ 물컵 규칙의 핵심 ★ 모든 축이 '같이 출발해 같이 도착' 하게
        이동량에 비례해 속도를 나눈다.

        같은 속도를 주면 많이 가는 축(j2)은 오래 걸리고 적게 가는 축(j4)은
        먼저 도착한다. j2+j3+j4 합이 이동 내내 달라져 그리퍼가 기운다.
        (실측: j2 는 106카운트 뒤처지는데 j4 는 36카운트만 뒤처짐)
        """
        d=[abs(rad_to_raw(j,q_to[j])[0]-rad_to_raw(j,q_from[j])[0])
           for j in range(min(5,NJOINT))]
        mn=min([x for x in d if x>0] or [0])
        if mn<1: return
        base=SAFE_VEL*SIM_SPEED[0]
        for j in range(min(5,NJOINT)):
            i=J2ID[j]
            if i==GRIP_ID or i in STS_DEAD_IDS: continue
            # ★ 느리게 낮추지 않고, 많이 가는 축을 '올린다' ★
            #  낮추는 방식은 느린 축이 속도 50 근처(절벽 구간)에 빠져
            #  급격히 느려지고 팔을 끄는 느낌(=덜덜)이 난다.
            #  가장 적게 가는 축을 기준으로, 많이 가는 축만 비율만큼 빠르게.
            v=int(max(base,min(2000,base*(d[j]/mn))))
            if getattr(self,'_vcache',{}).get(i)!=v:
                self._w2(i,ADDR_P_VEL,v)
                if not hasattr(self,'_vcache'): self._vcache={}
                self._vcache[i]=v

    def _sync_goal_timed(self,raws,vel):
        if 'ACTIVE_ARM' in globals() and _arm_block_real(): return False
        return self._sync_goal_timed0(raws,vel)
    def _sync_goal_timed0(self,raws,vel):
        """위치+시간+속도(6바이트)를 한 패킷으로 — 예전 Gradio 방식.
        11-06: 비상정지 버튼이 눌려 있으면 보내지 않는다.

        STS3215 는 0x2A 부터 [위치2 · 시간2 · 속도2] 를 연속으로 받는다.
        속도를 지정해 보내면 모터 내부 프로파일이 부드럽게 가속/감속한다.
        축마다 속도를 달리 주면 '같이 출발해 같이 도착' 한다(물컵 유지).
        """
        if ESTOP['held']: return False
        try:
            gw=getattr(self,'_gsw6',None)
            if gw is None:
                from scservo_sdk import GroupSyncWrite
                gw=self._gsw6=GroupSyncWrite(self.port,self.packet,ADDR_GOAL,6)
            gw.clearParam()
            for i,raw in raws.items():
                if i in STS_DEAD_IDS: continue
                v=int(raw); sp=int(max(10,min(3000,vel.get(i,SAFE_VEL))))
                data=bytes([v & 0xFF,(v>>8)&0xFF, 0,0, sp & 0xFF,(sp>>8)&0xFF])
                if not gw.addParam(i,data): raise RuntimeError('addParam')
            with IO_LOCK:
                if gw.txPacket()!=0: raise RuntimeError('tx')
            return True
        except Exception:
            return False

    def _sync_goal(self,raws):
        """여러 모터의 목표를 한 패킷으로 동시에 보낸다.
        실패하면 예전처럼 하나씩 보낸다(안전)."""
        if 'ACTIVE_ARM' in globals() and _arm_block_real(): return False
        if ESTOP['held']: return False           # 11-06: 비상정지 버튼이 눌려 있는 동안 움직이지 않는다
        try:
            gw=getattr(self,'_gsw',None)
            if gw is None:
                from scservo_sdk import GroupSyncWrite
                gw=self._gsw=GroupSyncWrite(self.port,self.packet,ADDR_GOAL,2)
            gw.clearParam()
            for i,raw in raws.items():
                if i in STS_DEAD_IDS: continue
                v=int(raw)
                if not gw.addParam(i,bytes([v & 0xFF,(v>>8) & 0xFF])):
                    raise RuntimeError('addParam 실패')
            with IO_LOCK:
                if gw.txPacket()!=0: raise RuntimeError('txPacket 실패')
            return True
        except Exception:
            for i,raw in raws.items():          # 폴백: 하나씩
                self._w2(i,ADDR_GOAL,raw)
            return False
    def read(self):
        q=[0.0]*NJOINT
        for jidx in range(NJOINT):
            i=J2ID[jidx]
            p=self._r2(i,ADDR_PRESENT)
            if p is not None: q[jidx]=DIR[i]*(p-ZERO_RAW[i])*math.pi/2048.0
        return q
    def _r2fast(self,i,addr):
        """빠른 1회 읽기 (추종 확인용)."""
        if i in STS_DEAD_IDS: return None
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
        for jidx in range(NJOINT):
            if jidx==5 and ids is None: continue   # j6 는 검사 제외 (그리퍼 블록 전용)
            raw,i=rad_to_raw(jidx,q[jidx])
            if i in STS_DEAD_IDS: continue
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
        """목표에 도착할 때까지 기다린다.

        ★ 과보정(밀었다 되돌리기)은 하지 않는다 ★
          예전에는 잔여 오차만큼 목표를 반대로 밀어 마찰을 뚫으려 했다.
          그런데 그게 진동의 주범이었다 — 밀면 반대로 넘어가고 또 되민다.
          실측 로그: j3 +20 -> 2532, 곧이어 -10 -> 2502 (되돌림)
                     j4 -20 -> 2239, 곧이어 +10 -> 2269 (되돌림)
          구형(그라디오) 프로그램에는 이 과보정이 아예 없었고 잘 돌아갔다.
          8/23 기록에도 "하지 말 것: settle 에서 작은 오차 강제로 밀기".
          -> 기다리기만 하고, 못 맞춘 오차는 그대로 둔다(모터가 알아서 온다).
        """
        best=None
        self.hold_gain()          # 도착 직전 — 정밀하게 잡는다
        pushed={}                 # 관절별로 이미 민 방향 (되돌리기 방지)
        for k in range(retry):
            ok,err=self.wait_reach(q,tol=tol,timeout=timeout/retry)
            best=err if best is None else min(best,err)
            if ok: return True,err
            if STOP.is_set(): return False,err
            if not SETTLE_PUSH[0]:
                time.sleep(END_SETTLE)      # 밀지 않고 조금 더 기다린다
                continue
            # --- 오차 보정 (Ki 가 대부분 잡아주므로 소폭만) ---
            #  중력 반대쪽으로 목표를 살짝 밀어 마찰/데드밴드를 넘긴다
            #
            #  ★ 물컵 규칙 ★ j2,j3,j4 는 축이 같아서 '합' 이 그리퍼 기울기다.
            #  한 관절만 밀면 합이 바뀌어 물컵이 쏟아진다(=이동 중 기울어짐).
            #  그래서 j2/j3/j4 는 밀어준 양의 총합을 j4 로 되돌려 합을 지킨다.
            pushes={}
            for jidx in range(5):                      # j6 제외
                raw,i=rad_to_raw(jidx,q[jidx])
                if i in STS_DEAD_IDS: continue
                p=self._r2(i,ADDR_PRESENT)
                if p is None: continue
                d=raw-p
                # ★ 작은 오차는 건드리지 않는다 ★
                #  데드존(10) 안쪽 오차는 모터가 어차피 못 맞춘다. 그런데
                #  억지로 10씩 밀면 반대로 넘어가고 또 되밀어 '덜덜' 떤다.
                #  (사용자 지적: "오차 1~2 를 강하게 맞추는 느낌")
                #  -> 데드존보다 확실히 큰 오차만, 그것도 절반만 민다.
                if abs(d)>max(tol,DEADZONE*1.5):
                    step=int(d*0.5)
                    if abs(step)<DEADZONE: continue      # 밀어도 소용없다
                    # ★ 되돌리지 않는다 ★
                    #   +20 밀고 다음 회차에 -10 되미는 게 진동이었다
                    #   (실측: j3 +20 -> 2532, 곧이어 -10 -> 2502).
                    #   한 관절은 처음 민 방향으로만 민다.
                    sgn=1 if step>0 else -1
                    if pushed.get(jidx) not in (None,sgn): continue
                    pushed[jidx]=sgn
                    pushes[jidx]=(raw,i,max(-20,min(20,step)))
            if LEVEL_ALWAYS:
                # j2+j3+j4 로 준 보정량의 합을 구해 j4 에서 상쇄한다
                tilt=sum(v[2] for k,v in pushes.items() if k in (1,2,3))
                if tilt and 3 in pushes:
                    raw4,i4,p4=pushes[3]
                    pushes[3]=(raw4,i4,p4-tilt)
                elif tilt:
                    raw4,i4=rad_to_raw(3,q[3])
                    pushes[3]=(raw4,i4,-tilt)          # j4 만으로 되돌린다
            for jidx,(raw,i,dpush) in sorted(pushes.items()):
                push=raw+dpush
                lo,hi=LIMIT_RAW[i]
                self._w2(i,ADDR_GOAL,max(lo,min(hi,push)))
                LOG(f"  [보정] {jname(jidx)}: {dpush:+d} -> 목표 {raw}->{max(lo,min(hi,push))}")
            time.sleep(END_SETTLE)
        return False,(best if best is not None else 9999)
    def gripper(self,open_,tq=None):
        if ESTOP['held']: return                 # 11-06: 비상정지 버튼이 눌려 있는 동안 그리퍼도 안 움직인다
        i=GRIP_ID
        self._w1(i,ADDR_MODE,0)
        if open_:
            # 물체를 조인 채 이동하면 과부하 보호가 걸릴 수 있다
            # -> 토크를 껐다 켜서 보호 해제 후 연다
            self._w1(i,ADDR_TORQUE,0); time.sleep(0.05)
        self._w2(i,ADDR_TORQUE_LIMIT,tq or PER_ID_TQ.get(i,TORQUE_LIMIT))
        self._w1(i,ADDR_P_ACCEL,GRIP_ACCEL)
        self._w2(i,ADDR_P_VEL,PER_ID_VEL.get(i,SAFE_VEL))
        self._w1(i,ADDR_TORQUE,1)
        self._w2(i,ADDR_GOAL,GRIPPER_OPEN if open_ else GRIPPER_CLOSE)
    def estop(self):
        if ESTOP['hold']: return               # 11-06: 비상정지 버튼 '1' = 토크 유지 정지 (끄지 않는다)
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
        q=[0.0]*NJOINT
        for jidx in range(NJOINT):
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
FORCE_SIM=[False]   # True 동안은 실행 로봇 설정과 무관하게 '무조건 시뮬'
                    # (② 시뮬레이션 탭 미리보기가 켠다. 실물로는 절대 안 나감)
SIM_SPEED=[1.0]     # 미리보기 배속: 잡기/하강 등 내부 재생도 이 값으로 나눠
                    # 빨리 감고, 공정 시간은 (경과×배속)으로 정직하게 환산한다
class RobotMux:
    # mode='sim' = 시뮬레이션. 실물 통신을 전부 건너뛰고 3D/RViz 만 움직인다.
    #   -> _sts()/_dxl() 이 모두 False 가 되므로 아래 메서드들이 자동으로 무동작
    def _sim(self): return FORCE_SIM[0] or EXEC['mode']=='sim'
    def _sts(self): return (not FORCE_SIM[0]) and EXEC['mode'] in ('sts','both')
    def _dxl(self): return (not FORCE_SIM[0]) and EXEC['mode'] in ('dxl','both') and DXL_OK
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
        # ★ 최종 안전장치 ★ 실물 모터로 나가는 길목은 여기 하나뿐이다.
        # 3D 에서 고른 로봇이 실물과 다르면(예: UR15 8축) 축 수·기구학이
        # 달라 엉뚱한 각도가 모터에 들어간다 -> 사람이 다친다. 무조건 막는다.
        _ok,_why=real_robot_ok()
        if not _ok:
            if not _SENT_WARN[0]:
                _SENT_WARN[0]=True
                LOG('  [안전] 실물 전송 차단 — '+_why.splitlines()[0])
            return {}
        _SENT_WARN[0]=False
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
# ===== 아두이노 비상정지 버튼 (11-06 사용자) =====
#   아두이노가 9600bps 로 평소 '0', 누르면 '1' 을 줄마다 보낸다 (Arduino Due, /dev/serial/by-id/*Arduino*).
#   '1' 이 들어오는 순간 = 지금 동작을 멈추고 두 팔 모터를 지금 자리에 세운다 (토크 유지).
#     눌려 있는 동안은 모터 목표·그리퍼 명령을 보내지 않는다 (실행도 거절).
#   '1' 다음 '0' 이 들어오는 순간 = 토크를 끈다 (화면 [■ 정지] 와 같은 동작).
#   평소의 '0' 은 아무것도 안 한다. 포트는 ARM_ESTOP_PORT 로 바꿀 수 있다. 1200bps 로는 절대 열지 않는다 (Due 지워짐).
ESTOP={'held':False,'hold':False,'port':None}
ESTOP_BAUD=int(os.environ.get('ARM_ESTOP_BAUD','9600'))

def _estop_port():
    p=os.environ.get('ARM_ESTOP_PORT')
    if p: return p
    try:
        for n in sorted(os.listdir('/dev/serial/by-id')):
            if 'arduino' in n.lower(): return os.path.join('/dev/serial/by-id',n)
    except Exception: pass
    return None

def _estop_press():
    """'1': 멈추고 토크 유지."""
    ESTOP['held']=True; ESTOP['hold']=True
    STOP.set()
    try: PAUSE.clear()
    except Exception: pass
    held=0
    try:
        if ROBOT_STS.port is not None:
            for i in sorted(ZERO_RAW):
                if i in STS_DEAD_IDS: continue
                p=ROBOT_STS._r2fast(i,ADDR_PRESENT)
                if p is None: continue
                with IO_LOCK:
                    ROBOT_STS.packet.write2ByteTxRx(ROBOT_STS.port,i,ADDR_GOAL,int(p))
                held+=1
    except Exception as e:
        print('[비상정지] 자리 잡기 실패:',e)
    LOG(f"⛔ [비상정지] 버튼 — 토크 유지한 채 그 자리에 멈춤 (모터 {held}개)")

def _estop_release(reason=None):
    """'1' 다음 '0': 토크 끄기 (화면 [■ 정지] 와 같음). reason = 버튼을 뗀 게 아닐 때 (연결 끊김 등)."""
    ESTOP['held']=False; ESTOP['hold']=False
    STOP.set()
    try: PAUSE.clear()
    except Exception: pass
    off=0
    try:
        if ROBOT_STS.port is not None:
            ROBOT.estop()                        # 지금 팔 (예전과 같음)
            # 11-06 (사용자: A팔은 토크가 안 풀림) — 지금 팔만 끄면 쉬는 팔(기본 자세)이 그대로 켜져 있었다 → 두 팔 전부
            for i in sorted(ZERO_RAW):
                if i in STS_DEAD_IDS: continue
                if ROBOT_STS._w1(i,ADDR_TORQUE,0): off+=1
    except Exception as e:
        print('[비상정지] 토크 끄기 실패:',e)
    LOG(f"[비상정지] {reason or '버튼 풀림'} — 두 팔 토크 끔 (모터 {off}개)")

def _estop_loop():
    try:
        import serial
    except Exception:
        return
    last=None
    while True:
        p=_estop_port()
        if not p:
            time.sleep(2); continue
        try:
            s=serial.Serial(p,ESTOP_BAUD,timeout=0.5)
        except Exception:
            time.sleep(2); continue
        ESTOP['port']=p
        print(f"비상정지 버튼: {p} ({ESTOP_BAUD}bps)",flush=True)
        buf=b''
        try:
            while True:
                buf+=s.read(64)
                while b'\n' in buf:
                    ln,buf=buf.split(b'\n',1)
                    v=ln.strip()
                    if v==b'1' and last!='1':
                        last='1'; _estop_press()
                    elif v==b'0':
                        if last=='1': _estop_release()
                        last='0'
                if len(buf)>256: buf=buf[-64:]
        except Exception as e:
            print('[비상정지] 버튼 연결 끊김:',e,flush=True)
            ESTOP['port']=None
            try: s.close()
            except Exception: pass
            # 11-06 (사용자: 버튼 빼도 비상정지 눌렸다고 함) — '1'(눌림) 뒤 '0' 없이 아두이노가 빠지면 눌린 채로 남아
            #   모든 동작이 막혔다 → 연결이 끊기면 버튼을 뗀 것과 같게 (비상정지 풀고 두 팔 토크 끔)
            if last=='1' or ESTOP['held']:
                last='0'
                _estop_release('버튼(아두이노) 연결이 끊겨 비상정지를 풂')
            time.sleep(2)

threading.Thread(target=_estop_loop,daemon=True,name='estop').start()
# ===== 일시 정지 (11-06 사용자: 실행 도중 멈추고, 3D 에서 도킹을 옮긴 뒤 [계속] 하면 옮긴 자리로) =====
#   멈추는 곳 = 다음 모터 목표를 보내기 직전 (_send_real · 블록 사이 · 구간 이동). 팔은 토크를 켠 채 그 자리에 선다.
#   [계속] 때 지금 가던 목표(이동 대상 · 잡기 펜 · 놓기 도킹)가 옮겨졌으면 _TargetLost 로 그 자리에서 다시 계산한다.
#   실물 실행 중일 때만 걸린다 (RUN_ACTIVE, h_run 이 켜고 끈다) — 미리보기·미세조정 등은 영향 없음.
PAUSE=threading.Event()
PAUSE_STATE={'on':False}       # 지금 실제로 멈춰 서 있는지 (화면 표시용)
RUN_ACTIVE=[False]             # 실물 실행 중 (h_run)
PAUSE_MOVE_TOL=0.002           # 목표가 이만큼(m) 넘게 옮겨졌으면 다시 계산

def h_pause():
    """[⏸ 일시 정지] / [▶ 계속] (같은 버튼)."""
    if not RUN_ACTIVE[0]:
        PAUSE.clear(); return "실물 실행 중이 아닙니다 (일시 정지는 실물 실행 중에만)"
    if PAUSE.is_set():
        PAUSE.clear(); BTN('계속','일시 정지 풀기')
        return "계속 — 옮긴 도킹이 있으면 그 자리로 다시 계산해 갑니다"
    PAUSE.set(); BTN('일시 정지','다음 걸음에서 멈춤')
    return "일시 정지 — 팔이 지금 걸음을 마치면 그 자리에서 멈춥니다 (그리퍼가 움직이는 중이면 끝난 뒤)"

def _pause_point():
    """일시 정지면 풀릴 때까지 기다린다. 정지 버튼으로 끝나면 'stop'.
    풀린 뒤 지금 목표가 옮겨졌으면 _TargetLost (이동·잡기·놓기가 그 자리에서 다시 계산)."""
    if not PAUSE.is_set(): return None
    if not RUN_ACTIVE[0]:
        PAUSE.clear(); return None
    PAUSE_STATE['on']=True
    LOG("\n[일시 정지] 팔은 지금 자리에서 힘을 준 채 기다립니다 — 3D 에서 도킹을 옮겨도 됩니다. "
        "[▶ 계속] 을 누르면 이어 갑니다")
    try:
        while PAUSE.is_set() and not STOP.is_set():
            time.sleep(0.1)
    finally:
        PAUSE_STATE['on']=False
    if STOP.is_set(): return 'stop'
    LOG("[계속] 이어 갑니다")
    g=globals().get('GOTO_GUARD') or {}
    oid=g.get('poid')
    if oid is not None:
        p=goto_point(oid)
        if p is None: raise _TargetLost('사라짐')
        if g.get('pxy') is not None and math.hypot(float(p[0])-g['pxy'][0],float(p[1])-g['pxy'][1])>PAUSE_MOVE_TOL:
            raise _TargetLost('옮겨짐')
    return None

# ================= 상태 =================
S={'poses':[{'name':'base','angles':[0.0]*NJOINT}], 'seq':['base']}
LOCK=threading.Lock()
def pose_names(): return [p['name'] for p in S['poses']]
def get_angles(name):
    # 사물 중심 블록(@goto:): 등록된 자세가 아니라 '실행 순간 IK' 로 계산
    if isinstance(name,str) and name.startswith(GOTO_PREFIX):
        q=goto_q(name[len(GOTO_PREFIX):])
        return list(q) if q is not None else None
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

def _arm_off_key(rk):
    """보정 키: B팔 실물 보정은 A팔 것과 따로 둔다 (11-02). A팔은 예전 그대로 'sts'.
    (자세마다 저장된 보정은 A팔 실물에 맞춘 값이라 B팔에 더하면 엉뚱한 데로 간다)"""
    L=globals().get('ACTIVE_ARM',['A'])[0]
    return f"{rk}_{L}" if rk=='sts' and L!='A' else rk

def get_offset(name,rk=None):
    p=_pose_rec(name)
    if not p: return [0.0]*NJOINT
    rk=_arm_off_key(rk or offset_robot())
    d=p.get('offsets')
    if d is None:                      # 예전 형식 -> STS 보정으로 이관
        d={'sts':list(p.get('offset',[0.0]*NJOINT))}
        p['offsets']=d
    return list(d.get(rk,[0.0]*NJOINT))

def set_offset(name,off,rk=None):
    p=_pose_rec(name)
    if not p: return
    rk=_arm_off_key(rk or offset_robot())
    d=p.setdefault('offsets',{'sts':list(p.get('offset',[0.0]*NJOINT))})
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
GOTO_PREFIX='@goto:'          # 사물 위로 이동 (자세 등록 없이 즉석 IK)
def is_goto_item(x): return isinstance(x,str) and x.startswith(GOTO_PREFIX)
DOWN_ITEM='@down'             # 하강: 현재 XY 유지, 잡는 높이까지 수직
def is_down_item(x): return x==DOWN_ITEM
UP_ITEM='@up'                 # 상승: 하강의 반대, 접근 높이까지 수직
def is_up_item(x): return x==UP_ITEM
# ★ 두 팔 블록 (2026-10-30) — 더블 레이첼 출력과 같은 꼴 ★
#   '@arm:A'  = 이 뒤의 블록은 A팔이 한다 (스크래치의 '스프라이트 고르기' 처럼)
#   '@wait:10' = 10초 기다리기
#   둘 다 '그 자리 동작' 이라 경로 계획(자세 사이 이동)에는 끼지 않는다.
ARM_PREFIX='@arm:'; WAIT_PREFIX='@wait:'
def is_arm_item(x): return isinstance(x,str) and x.startswith(ARM_PREFIX)
def is_wait_item(x): return isinstance(x,str) and x.startswith(WAIT_PREFIX)
def is_meta_item(x): return is_arm_item(x) or is_wait_item(x)
def wait_secs(x):
    try: return max(0.0,min(600.0,float(x[len(WAIT_PREFIX):])))
    except Exception: return 0.0
def wait_arm(x):
    """'@wait:arm:A' → 'A' — 'A팔이 끝날 때까지' 기다리기 (11-03). 시간 기다리기면 None."""
    r=x[len(WAIT_PREFIX):] if is_wait_item(x) else ''
    return r[4:] if r.startswith('arm:') else None
def wait_text(x):
    L=wait_arm(x)
    return f"{L}팔 끝날 때까지" if L else f"{wait_secs(x):g}초"
ARMS_DEFAULT={'A':'빨강','B':'파랑'}       # 팔 이름 -> 색 (사용자가 색으로 부른다: 빨간 팔·파란 팔)
REAL_ARM='A'                                # 기본 실물 팔 (영점 저장·공장 배치는 이 팔 기준)
REAL_ARMS={'A','B'} if _ARM2 is not None else {'A'}   # 11-02: B팔(모터 11~15)도 실물 — sts_arm2.py 가 있을 때
def arms_cfg():
    a=S.setdefault('arms',dict(ARMS_DEFAULT))
    return a if isinstance(a,dict) and a else dict(ARMS_DEFAULT)
def arm_label(L):
    c=arms_cfg().get(L)
    return f"{L}팔 · {c} 팔" if c else f"{L}팔"
def is_grasp_item(n): return isinstance(n,str) and n.startswith(GRASP_PREFIX)
def is_release_item(n): return isinstance(n,str) and n.startswith(RELEASE_PREFIX)
def is_brick_item(n): return is_grasp_item(n) or is_release_item(n)
def is_grip_item(x):
    # '@' 로 시작하는 다른 블록(@grasp:/@release:)도 생겼으므로
    # 그리퍼 블록은 정확히 이 두 값일 때만이다.
    return x in (GRIP_OPEN_ITEM,GRIP_CLOSE_ITEM)
def _brick_home(bid):
    """그 물체가 지금 꽂혀 있는 도킹 (없으면 None). 잡기 앞에 넣을 이동 대상."""
    try:
        b=BRICKS.get(bid) or BRICKS.get(_clone_base(bid))
        if not b or b.get('pos') is None: return None
        bp=b['pos']
        for dk in docking_ids():
            t=dock_socket_world(dk)
            if t and math.hypot(bp[0]-t[0],bp[1]-t[1])<0.025: return dk
    except Exception: pass
    return None

def seq_fix_moves(seq=None):
    """★ 잡기/놓기 앞에 [이동] 이 없으면 넣어 준다 ★

    잡기·놓기는 좌표를 갖지 않는다(제자리 동작). 그래서 그 앞에 반드시
    [이동] 이 있어야 어디서 할지가 정해진다.
    예전 파이프라인은 잡기/놓기가 스스로 좌표를 계산했으므로 이동 없이도
    돌아갔다 -> 그대로 두면 엉뚱한 자리에서 집게만 여닫는다.
    여기서 옛 파이프라인을 자동으로 고쳐 준다."""
    seq=list(S['seq'] if seq is None else seq)
    out=[]; changed=0
    # ★ 물체 위치를 파이프라인을 따라 추적한다 (10-30) ★
    #   예전엔 잡기마다 '물체의 지금(시작) 자리' 를 썼다 → 2단계 이송(놓은 뒤 다시 잡기)이면
    #   두 번째 잡기 앞에 '처음 자리' 로 가는 이동을 끼워 넣어 엉뚱한 곳에서 집었다 (재시작 때마다).
    home={}; held=None
    for n in seq:
        need=None; bid=None
        if is_grasp_item(n):
            bid=n[len(GRASP_PREFIX):]
            if bid not in home: home[bid]=_brick_home(bid)
            need=home[bid]                              # 물체가 '그 시점에' 있는 도킹으로
        elif is_release_item(n):
            need=n[len(RELEASE_PREFIX):]                # 그 도킹으로
        if need:
            # 바로 앞이 이미 그 자리(또는 그 물체)로 가는 이동이면 넣지 않는다
            # 제자리 동작(그리퍼·하강·상승·팔·기다리기)은 건너뛰고 '진짜 이동' 을 찾는다
            prev=next((x for x in reversed(out)
                       if not (is_grip_item(x) or is_down_item(x)
                               or is_up_item(x) or is_meta_item(x))), None)
            ok_prev=is_goto_item(prev) and prev[len(GOTO_PREFIX):] in (need,bid)
            if not ok_prev:
                out.append(GOTO_PREFIX+need); changed+=1
        if is_grasp_item(n): held=bid
        elif is_release_item(n) and held:
            home[held]=n[len(RELEASE_PREFIX):]; held=None
        out.append(n)
    return out,changed

def pose_seq():
    """시퀀스에서 자세 항목만 (경로 계획용).
    그리퍼·물체 블록은 자세가 아니므로 제외한다."""
    return [x for x in S['seq']
            if not is_grip_item(x) and not is_brick_item(x)
            and not is_down_item(x) and not is_up_item(x) and not is_meta_item(x)]

def seq_labels(mode_name=None):
    """파이프라인 블록: ■자세 / ┃이동 / ◆그리퍼"""
    out=[]; mv=get_moves()
    prev_pose=None; seg_i=0
    for idx,n in enumerate(S['seq']):
        if is_arm_item(n):
            out.append(f"◉ {idx}. 팔: {arm_label(n[len(ARM_PREFIX):])}")
            continue
        if is_wait_item(n):
            out.append(f"⧗ {idx}. 기다리기 {wait_text(n)}")
            continue
        if is_grip_item(n):
            act='열기 ✋' if n==GRIP_OPEN_ITEM else '닫기 ✊'
            out.append(f"◆ {idx}. 그리퍼 {act}")
            continue
        if is_down_item(n):
            out.append(f"▼ {idx}. 하강 (그 자리 수직)")
            continue
        if is_up_item(n):
            out.append(f"▲ {idx}. 상승 (그 자리 수직)")
            continue
        if is_brick_item(n):
            # 물체 블록은 자세 사이에 끼어들 뿐 '이동 구간' 을 만들지 않는다
            if is_grasp_item(n):
                out.append(f"◈ {idx}. 잡기")      # 제자리 동작 — 대상 없음
            else:
                out.append(f"◈ {idx}. 놓기")      # 제자리 동작 — 대상 없음
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
        if is_goto_item(n):                # 사물 중심 이동 블록
            out.append(f"➜ {idx}. 이동: {disp_name(n[len(GOTO_PREFIX):])}")
        else:
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
    if s.lstrip().startswith('➜'):          # 사물 이동 블록 (자세처럼 취급)
        try: return 'pose',int(s.split('.')[0].replace('➜','').strip())
        except Exception: return None,None
    if s.lstrip().startswith('▼'):          # 하강 블록 (그 자리 동작)
        try: return 'grip',int(s.split('.')[0].replace('▼','').strip())
        except Exception: return None,None
    if s.lstrip().startswith('▲'):          # 상승 블록 (그 자리 동작)
        try: return 'grip',int(s.split('.')[0].replace('▲','').strip())
        except Exception: return None,None
    for _c in ('◉','⧗'):                     # 팔 바꾸기 · 기다리기 (그 자리 동작)
        if s.lstrip().startswith(_c):
            try: return 'grip',int(s.split('.')[0].replace(_c,'').strip())
            except Exception: return None,None
    return None,None

def move_seg_poses(seg_i):
    """이동 블록 인덱스 -> (앞 자세, 뒤 자세). 없으면 (None,None)"""
    ps=pose_seq()
    if 0<=seg_i<len(ps)-1: return ps[seg_i],ps[seg_i+1]
    return None,None

CURRENT_Q=[0.0]*NJOINT   # 웹 뷰어가 폴링하는 현재 자세
GRIP_STATE={'rad':0.0}          # 3D 뷰용 그리퍼 상태 (◆블록으로만 변경)
# ===== RViz 부드럽게 보내기 =====
# 웨이포인트는 초당 6~19개밖에 안 나온다(실물 모터로 값을 쓰는 시간이 대부분).
# 그대로 RViz 에 보내면 뚝뚝 끊겨 보인다. 그래서 뒤에서 50Hz 로
# '직전 자세 -> 목표 자세' 사이를 채워 보낸다. 실물 전송과는 무관하다.
_ROS_TGT=[None]          # 보내야 할 목표 자세
_ROS_CUR=[None]          # 지금 RViz 에 보이는 자세
_ROS_HZ=50.0
_ROS_SMOOTH=[True]

def _ros_send(q):
    if _pub is None: return
    m=JointState(); m.header.stamp=_ros.get_clock().now().to_msg()
    m.name=list(JOINTS); m.position=[float(x) for x in q]; _pub.publish(m)

def _ros_smooth_loop():
    """목표까지 조금씩 따라가며 RViz 로 보낸다 (끊김 없이 보이게)."""
    dt=1.0/_ROS_HZ
    while True:
        time.sleep(dt)
        tgt=_ROS_TGT[0]
        if tgt is None or _pub is None: continue
        cur=_ROS_CUR[0]
        if cur is None or len(cur)!=len(tgt):
            _ROS_CUR[0]=list(tgt); _ros_send(tgt); continue
        # 남은 거리의 35% 씩 좁힌다 (지수 보간 - 자연스럽게 감속)
        far=False
        nxt=[]
        for a,b in zip(cur,tgt):
            d=b-a
            if abs(d)>0.0005: far=True
            nxt.append(a+d*0.35)
        if far:
            _ROS_CUR[0]=nxt; _ros_send(nxt)
        elif cur!=tgt:
            _ROS_CUR[0]=list(tgt); _ros_send(tgt)

if _pub is not None:
    threading.Thread(target=_ros_smooth_loop,daemon=True).start()

def ros_publish(q,with_grip=False):
    """3D 뷰 갱신. 그리퍼(j6)는 자세 값을 무시하고 현재 상태를 유지한다."""
    global CURRENT_Q
    q=list(q)
    # ★ 그리퍼가 있는 로봇에서만 마지막 축을 '그리퍼 상태' 로 덮어쓴다 ★
    #   UR15 의 마지막 축(end)은 그리퍼가 아니라 손목 방향을 만드는 축이다.
    #   여기서 덮어쓰면 IK 가 0.02도로 맞춘 방향이 13.58도로 깨진다.
    if LEVEL_ALWAYS:                    # 6축(SO-ARM)만 그리퍼 개념이 있다
        if with_grip: GRIP_STATE['rad']=q[GRIP_IDX]
        else:         q[GRIP_IDX]=GRIP_STATE['rad']
    CURRENT_Q=q
    if _pub is None: return
    if _ROS_SMOOTH[0]:
        _ROS_TGT[0]=list(q)      # 뒤에서 부드럽게 따라간다
    else:
        _ros_send(q)

# ================= 3D 뷰 (RViz처럼 STL 메쉬 렌더링) =================
MESH_DIR=os.path.join(os.path.dirname(BASE_URDF),'..','meshes')
def _urdf_links(path):
    """URDF 에서 링크 이름을 순서대로 (메쉬가 있는 것만)."""
    try:
        root=ET.parse(path).getroot()
        out=[]
        for l in root.findall('link'):
            if l.find('visual') is not None: out.append(l.get('name'))
        return out or ['base_link','l1','l2','l3','l4','l5','l6']
    except Exception:
        return ['base_link','l1','l2','l3','l4','l5','l6']
ROBOT_LINKS=_urdf_links(BASE_URDF)
ROBOT_COLORS=['#8a8a8a','#5b8dbf','#6a9bcf','#5b8dbf','#6a9bcf','#5b8dbf','#e0a13a']
# 링크 수가 더 많은 로봇(UR15 등)도 색이 모자라지 않게 채운다 (마지막 = 툴)
while len(ROBOT_COLORS)<len(ROBOT_LINKS):
    ROBOT_COLORS.insert(-1,'#6a9bcf' if len(ROBOT_COLORS)%2 else '#5b8dbf')
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
    R=_yawR()
    fr=[]
    for f in CHAIN.forward_kinematics([0]+list(q),full_kinematics=True):
        g=f.copy(); g[:3,:3]=R@g[:3,:3]; g[:3,3]=R@g[:3,3]+BASE_OFF
        fr.append(g)
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
    q=get_angles(name) or [0.0]*NJOINT
    ros_publish(q)
    # 슬라이더는 j1~j5 만
    return [robot_fig(q)]+[round(math.degrees(v),1) for v in q[:5]]

def h_slider(name,*degs):
    """j1~j5 슬라이더. j6 는 기존 값 유지 (그리퍼 버튼으로만 변경)."""
    if name is None: return gr.update(), "먼저 자세를 선택하세요"
    cur=get_angles(name) or [0.0]*NJOINT
    _r=_pose_rec(name)
    if _r is not None and _is_slot_pose(_r):     # 11-06 (사용자): 보정 자세는 [보정 저장] 으로만 바뀐다
        ros_publish(cur)
        return gr.update(), f"'{name}' 은 보정 자세라 슬라이더로 바꾸지 않습니다 — 미세조정 후 [보정 저장] 으로만"
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
        q=list(CURRENT_Q); q[GRIP_IDX]=rad
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
        if abs(c-a)>0.002: clamped.append(f"{jname(j)} {math.degrees(a):+.1f}->{math.degrees(c):+.1f}deg")
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
                           'offset':off if off else [0.0]*NJOINT})
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
        S['env_vis']={e['id']:bool(e.get('visible',True)) for e in ENV_SHOW}
    try: _macro_save()
    except Exception: pass
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
    try:                                           # 11-06: 보정 자세가 사라진 이유를 나중에 알 수 있게 기록
        _r=_pose_rec(name)
        if _r is not None:
            LOG(f"[자세 삭제] {name} (팔 {_r.get('arm') or 'A'}) — 지운 각도 "
                +' '.join(f"j{j+1} {math.degrees(v):+.2f}°" for j,v in enumerate(list(_r.get('angles') or [])[:4])))
    except Exception: pass
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
        _yw=float(e.get('yaw',0.0)); _of=e.get('offset',(0,0,0))
        _c,_s2=math.cos(_yw),math.sin(_yw)
        m.add_object(mid,_COLL_ENV[mid],transform=np.array(
            [[_c,-_s2,0,_of[0]],[_s2,_c,0,_of[1]],[0,0,1,_of[2]],[0,0,0,1]])); n+=1
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

def _maxz_flush():
    """새로 계산한 도달상한이 있으면 디스크에 저장 (다음부터 즉시 로드)."""
    if _MAXZ_DIRTY[0]:
        _MAXZ_DIRTY[0]=False
        _maxz_save()

def plan_all(mode,axorder,check_coll=False):
    ps=pose_seq()                                   # 그리퍼/하강 블록 제외
    if any(is_arm_item(x) and x!=ARM_PREFIX+'A' for x in S['seq']) and all(is_goto_item(p) for p in ps):
        # 두 팔 (10-30): 이동 블록은 실행 시점에 '그 팔' 기준으로 다시 계산하므로 미리 계산이 필요 없다
        #   (A팔 자리로 B팔 구간을 계산하면 엉뚱하게 '도달 불가' 가 난다)
        return [],None,""
    if len(ps)<2:
        # 자세가 1개(또는 0개)여도 잡기/놓기/그리퍼/하강 같은 '그 자리 동작'
        # 블록이 있으면 유효한 파이프라인이다 (자세 사이 경로가 필요 없음 -
        # 첫 자세 이동은 실행기가 따로 처리한다)
        if any(is_brick_item(x) or is_grip_item(x) or is_down_item(x)
               or is_up_item(x) or is_meta_item(x) for x in S['seq']) or len(ps)==1:
            return [],None,""
        return None,"시퀀스에 자세가 2개 이상 필요합니다",""
    segs=[]
    mv=get_moves()
    carry=None          # 앞 구간의 '실제 끝 자세' (j5 고정 등이 반영된 값)
    mode0=mode; si=1; ntot=max(1,len(ps)-1)
    for a,b in zip(ps[:-1],ps[1:]):
        qa=get_angles(a); qb=get_angles(b)
        if qa is None or qb is None:
            bad=a if qa is None else b
            return None,(f"'{bad}' 에 도달할 자세를 만들 수 없습니다 — "
                         "[로봇 자리 추천]을 눌러 로봇을 옮겨보세요"),""
        if carry is not None:
            qa=list(carry)          # 이전 구간에서 실제로 끝난 자세에서 출발
        # ★ 같은 자리로의 이동은 제자리 (2026-11-04) ★
        #   [이동 #2 → 잡기 → 이동 #2] 처럼 같은 대상이 이어지면 출발=도착이라 간편 이동이
        #   '시작/도착이 금지 영역' 으로 실패해 파이프라인 전체가 막혔다. 이동 블록은 실행 때
        #   어차피 다시 계산하므로(미리 계산분은 버림) 여기서는 빈 구간으로 둔다.
        # ★ 이동 블록으로 가는 구간은 미리 풀지 않는다 (11-04) ★
        #   미리보기·실물 둘 다 이동 블록은 실행하는 순간 goto_move 로 다시 풀고 여기 계산은 버린다.
        #   그런데 이 버려질 계산이 '시작/도착이 금지 영역' 으로 실패해 (#2 -> #2, 펜이 꽂힌 #2 -> #8)
        #   파이프라인 전체를 막았다. 실행 시점 계산(goto_move)은 같은 구간을 문제없이 간다.
        #   도달 여부는 위 get_angles(=도달 IK) 와 블록의 ! 표시(seq_reach)가 미리 알려 준다.
        if is_goto_item(b):
            PROG(f"[경로 계산 {si}/{ntot}] {a} -> {b} · 실행 때 계산")
            si+=1
            segs.append((f'{a} -> {b} [실행 때 계산]',[list(qa),list(qb)],0.05,
                         get_offset(a),get_offset(b)))
            carry=list(qb); continue
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
            for f in way: f[GRIP_IDX]=qa[5]
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
                    for f in way: f[GRIP_IDX]=qa[5]
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
        for f in way: f[GRIP_IDX]=qa[5]
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
    _maxz_flush()          # 새로 계산한 도달상한 저장 (다음엔 즉시)
    return segs,None,("\n".join(warns))

def _cr7_mm():
    """지금 cr_7(그리퍼 기준점)의 원점(플레이트 최하단) 기준 절대 높이."""
    return f"cr_7 높이 {float(tool_pos(CURRENT_Q)[2])*1000:.1f}mm"

def h_preview(kind,mode,axorder,speed,loop,coll=False,end_base=True):
    """② 시뮬레이션 미리보기: 파이프라인 전체(첫 자세 이동 + 그리퍼 +
    잡기/놓기 + 자세 간 경로)를 3D 에서만 재생한다.
    FORCE_SIM 을 켜므로 실행 로봇이 무엇이든 실물로는 절대 안 나간다."""
    mode=resolve_mode(kind,mode)
    BTN('3D 미리보기',f'{mode} · {speed}')
    if RUNNING.is_set():
        yield gr.update(),"다른 동작 실행 중"; return
    # 계산은 스레드에서 돌리고, 진행 상황을 실시간으로 흘려보낸다
    box={}
    def _work():
        try: box['r']=plan_all(mode,axorder,coll)
        except Exception as e:
            box['e']=e
            import traceback as _tb
            LOG("  [경로 계산 예외] "+repr(e))
            _tb.print_exc()
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
    FORCE_SIM[0]=True; SIM_SPEED[0]=sp
    _t_run=time.time()          # 공정 시간 측정 (경로 계산 제외, 재생만)
    SEQ_SEEK['blk']=-1          # 재생 시작 -> 타임바 '고른 블록' 표시 해제
    _run_begin(len(S['seq']),sp)     # 타임바 시작
    # 미리보기용 세계 스냅샷: 시뮬이 옮긴 물체/잡음 상태는 끝나면 원위치
    _snap_pos={b:v.get('pos') for b,v in BRICKS.items()}
    _snap_held=HELD.get('brick')
    try:
        # 11-05 충돌 방지: 쉬는 팔은 기본 자세에서 시작한다
        _bq_all=get_angles('base')
        if _bq_all is not None:
            for _Lr in arms_cfg():
                if _Lr!=ACTIVE_ARM[0]:
                    _st=ARM_STATE.setdefault(_Lr,{}); _st['q']=list(_bq_all)
                    _st.setdefault('grip',GRIP_STATE['rad'])        # arm_switch 가 q·grip 을 같이 읽는다
        # 시작은 무조건 base 자세 (블록으로 표시하진 않지만 반드시 거친다)
        _bq=get_angles('base')
        if _bq is not None:
            _way,_dt=plan_joint_all(list(CURRENT_Q),_bq,steps=30)
            for _k,_q in enumerate(_way):
                if STOP.is_set(): yield gr.update(),pre+"미리보기 정지"; return
                ros_publish(_q)
                if _k%max(1,int(0.2/_dt))==0:
                    yield gr.update(),pre+"시작: 기본 자세(base)로"
                time.sleep(_dt/sp)
        ps=pose_seq()
        while True:
            seg_i=0; pose_i=0
            for _bi,item in enumerate(S['seq']):
                if STOP.is_set():
                    yield gr.update(),pre+"미리보기 정지"; return
                _run_at(_bi)                     # 타임바/하이라이트용 위치
                if is_brick_item(item):
                    # 잡기/놓기: 실물 실행과 같은 함수를 FORCE_SIM 으로 재생
                    RUNNING.clear()          # do_grasp 가 자체적으로 잡는다
                    if is_grasp_item(item):
                        b=item[len(GRASP_PREFIX):]
                        yield gr.update(),pre+f"미리보기: 잡기 {brick_label(b)}"
                        msg=do_grasp(b)
                    else:
                        d=item[len(RELEASE_PREFIX):]
                        yield gr.update(),pre+f"미리보기: 놓기 {dock_label(d)}"
                        msg=do_release(d)
                    RUNNING.set()
                    yield gr.update(),pre+f"미리보기: {msg} — {_cr7_mm()}"
                    if ('오류' in msg or '없습니다' in msg or '불가' in msg
                            or '아닙니다' in msg):
                        yield gr.update(),pre+f"미리보기 중단: {msg}"; return
                    continue
                if is_meta_item(item):                 # 두 팔 블록 (10-30) — 3D 는 아직 한 대
                    if is_arm_item(item):
                        _L=item[len(ARM_PREFIX):]
                        if _L!=ACTIVE_ARM[0]:
                            # ★ 충돌 방지 (11-05 사용자) ★ 다른 팔이 움직이기 전에 지금 팔을 기본 자세로.
                            #   기본 자세의 팔은 다른 팔이 절대 건드릴 수 없다 (B팔이 놓고 떠나야 A팔이 그 도킹에 간다)
                            _bq=get_angles('base')
                            if _bq is not None:
                                yield gr.update(),pre+f"미리보기: {arm_label(ACTIVE_ARM[0])} 기본 자세로 (다른 팔이 움직이기 전에)"
                                _g0=CURRENT_Q[GRIP_IDX]
                                _way,_dt=plan_joint_all(list(CURRENT_Q),_bq,steps=30)
                                for _q in _way:
                                    if STOP.is_set(): yield gr.update(),pre+"미리보기 정지"; return
                                    _q=list(_q); _q[GRIP_IDX]=_g0
                                    ros_publish(_q); time.sleep(_dt/sp)
                        arm_switch(_L)
                        yield gr.update(),pre+f"미리보기: 팔 {arm_label(item[len(ARM_PREFIX):])}"
                    else:
                        if wait_arm(item):     # 'A팔 끝날 때까지' (11-03): 블록을 차례대로 하므로 그 팔 블록은 이미 끝났다
                            yield gr.update(),pre+f"미리보기: 기다리기 — {wait_text(item)} (끝났음 → 다음으로)"
                            continue
                        _w=wait_secs(item)
                        yield gr.update(),pre+f"미리보기: 기다리기 {_w:g}초"
                        _t0=time.time()
                        while time.time()-_t0<_w/max(sp,0.1):
                            if STOP.is_set(): break
                            time.sleep(0.1)
                    continue
                if is_down_item(item):
                    yield gr.update(),"하강: 그 자리에서 수직으로"
                    msg=do_descend_here()
                    yield gr.update(),f"{msg} — {_cr7_mm()}"
                    if '불가' in msg or '오류' in msg or '정지' in msg:
                        yield gr.update(),f"중단: {msg}"; return
                    continue
                if is_up_item(item):
                    yield gr.update(),"상승: 그 자리에서 수직으로"
                    msg=do_ascend_here()
                    yield gr.update(),f"{msg} — {_cr7_mm()}"
                    if '불가' in msg or '오류' in msg or '정지' in msg:
                        yield gr.update(),f"중단: {msg}"; return
                    continue
                if is_grip_item(item):
                    op=(item==GRIP_OPEN_ITEM)
                    gq=list(CURRENT_Q); gq[GRIP_IDX]=_j6_rad(op)
                    ros_publish(gq,with_grip=True)
                    yield gr.update(),(pre+f"미리보기: 그리퍼 "
                                       f"{'열기' if op else '닫기'} — {_cr7_mm()}")
                    time.sleep(0.4/sp)
                    continue
                # 자세 블록
                if is_goto_item(item) and pose_i>0:
                    # 이동 블록은 '실행 시점' 재계산: 파이프라인 중간에 물체가
                    # 옮겨졌으면 미리 계산한 경로는 옛 위치로 가버린다
                    if seg_i<len(segs): seg_i+=1        # 미리 계산분은 버림
                    pose_i+=1
                    oid=item[len(GOTO_PREFIX):]
                    yield gr.update(),pre+f"미리보기: 이동 → {disp_name(oid)}"
                    msg=goto_move(oid,'이동')
                    if msg:
                        yield gr.update(),pre+f"중단: {msg}"; return
                    yield gr.update(),(pre+f"미리보기: 이동 → {disp_name(oid)} "
                                       f"완료 — {_cr7_mm()}")
                    time.sleep(0.4)
                    continue
                if pose_i==0:
                    # 첫 자세: 지금 시뮬 자세에서 그 자세까지 움직여 보여준다
                    first=get_angles(item)
                    if first is None:
                        yield gr.update(),(f"중단: '{item}' 에 도달할 자세를 "
                            "만들 수 없습니다 — [로봇 자리 추천]을 눌러보세요")
                        return
                    way,dt=plan_joint_seq(list(CURRENT_Q),first)
                    lab=f"첫 자세로 이동: {item}"
                else:
                    if seg_i>=len(segs): pose_i+=1; continue
                    lab,way,dt,offa,offb=segs[seg_i]; seg_i+=1
                pose_i+=1
                dec=max(1,int(0.2/dt))          # 화면은 ~5fps (메쉬 렌더링)
                for k,q in enumerate(way):
                    if STOP.is_set():
                        yield robot_fig(q),pre+"미리보기 정지"; return
                    ros_publish(q)
                    if k%dec==0 or k==len(way)-1:
                        yield robot_fig(q),pre+f"미리보기: {lab}"
                    time.sleep(dt/sp)
                yield gr.update(),pre+f"미리보기: {lab} 완료 — {_cr7_mm()}"
                time.sleep(0.4)
            if not loop: break
        # 끝도 무조건 base 복귀 (단 end_base=False 면 그 자리에 멈춘다 —
        # 타임바/우클릭 '이 자세로 이동' 은 거기서 미세조정해야 하므로)
        _bq=get_angles('base') if end_base else None
        if _bq is not None and not STOP.is_set():
            _way,_dt=plan_joint_all(list(CURRENT_Q),_bq,steps=30)
            for _q in _way:
                if STOP.is_set(): break
                ros_publish(_q); time.sleep(_dt/sp)
        _cyc=(time.time()-_t_run)*sp    # 1x 실물 기준으로 환산
        yield gr.update(),(pre+("미리보기 완료 (기본 자세 복귀)" if end_base
                                else "그 자리에 멈춤 — 여기서 자세 등록/미세조정 가능")
                           +f" — {_cr7_mm()} · 공정 시간 약 {_cyc:.0f}초 (실물 1x 기준)")
    finally:
        FORCE_SIM[0]=False; SIM_SPEED[0]=1.0
        _run_end()
        RUNNING.clear()
        if ACTIVE_ARM[0]!='A':
            try: arm_switch('A')              # 미리보기가 끝나면 A팔이 다시 지금 팔
            except Exception as e: LOG(f"  [팔] A팔로 돌아가기 실패: {e}")
        # 시뮬이 옮긴 물체/잡음 상태 원위치 (화면 시뮬 != 실제 배치)
        for _b,_p in _snap_pos.items():
            if _b in BRICKS: BRICKS[_b]['pos']=_p
        for _b in [x for x in list(BRICKS) if x not in _snap_pos]:
            BRICKS.pop(_b,None); MESH_REG.pop(_b,None)
        HELD['brick']=_snap_held

def h_preview_upto(pos,speed='1.5x',end_base=True):
    """블록을 놓은 직후: 파이프라인을 '그 블록까지'만 미리보기.
    (시작 base 포함 - h_preview 를 잘린 시퀀스로 재생하고 원상복구)"""
    orig=list(S['seq'])
    try: pos=int(pos)
    except Exception: pos=len(orig)-1
    pos=max(0,min(pos,len(orig)-1))
    trunc=orig[:pos+1]
    with LOCK: S['seq'][:]=trunc
    _clear_plan_cache()
    try:
        for out in h_preview('간편 이동',MODES[0],'x-z-y',str(speed),
                             False,False,end_base=end_base):
            yield out
    finally:
        with LOCK:
            cur=list(S['seq'])
            if cur==trunc:
                S['seq'][:]=orig          # 재생 중 편집 없었음 -> 원상복구
            else:
                # 재생 중에 사용자가 블록을 놓거나 지웠다 -> 그 편집을 살리고
                # 미리보기 때문에 잘라뒀던 뒷부분만 도로 붙인다
                # (예전엔 무조건 orig 로 덮어써서 새 블록이 사라지는 버그)
                S['seq'][:]=cur+orig[pos+1:]
                _macro_save()
        _clear_plan_cache()

def h_run(kind,mode,axorder,speed,loop,coll=False):
    mode=resolve_mode(kind,mode)
    BTN('실물 실행',f'{mode} · {speed} · 반복{"ON" if loop else "OFF"} · 충돌검사{"ON" if coll else "OFF"}')
    _ok,_why=real_robot_ok()          # 고른 로봇과 실물이 다르면 시작조차 막는다
    if not _ok:
        yield gr.update(),'실물 실행 불가 — '+_why; return
    _other=sorted({x[len(ARM_PREFIX):] for x in S['seq'] if is_arm_item(x)}-REAL_ARMS)
    if _other:                        # 두 팔 (10-30): 실물이 연결되지 않은 팔은 3D 미리보기만
        yield gr.update(),('실물 실행 불가 — '+', '.join(arm_label(L) for L in _other)
                           +' 는 아직 실물이 연결되지 않았습니다 (3D 미리보기는 됩니다)'); return
    if RUNNING.is_set():
        yield gr.update(),"다른 동작 실행 중"; return
    # 계산은 스레드에서 돌리고, 진행 상황을 실시간으로 흘려보낸다
    box={}
    def _work():
        try: box['r']=plan_all(mode,axorder,coll)
        except Exception as e:
            box['e']=e
            import traceback as _tb
            LOG("  [경로 계산 예외] "+repr(e))
            _tb.print_exc()
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
    if ESTOP['held']:                          # 11-06: 비상정지 버튼이 눌려 있으면 시작하지 않는다
        yield gr.update(),"비상정지 버튼이 눌려 있습니다 — 버튼을 풀고 다시 실행하세요"; return
    ESTOP['hold']=False
    sp=float(speed.replace('x','')); STOP.clear(); RUNNING.set()
    PAUSE.clear(); RUN_ACTIVE[0]=True           # 11-06 일시 정지 가능
    try:
        # 두 팔 (11-02): 첫 이동은 아래에서 블록 루프보다 먼저 가므로, 그 앞의 팔 블록대로 팔(=모터 번호)을 먼저 정한다
        _f0=next((i for i,x in enumerate(S['seq']) if not is_grip_item(x) and not is_brick_item(x)
                  and not is_down_item(x) and not is_up_item(x) and not is_meta_item(x)),len(S['seq']))
        _lead=[x[len(ARM_PREFIX):] for x in S['seq'][:_f0] if is_arm_item(x)]
        # ★ 충돌 방지 (11-05 사용자) ★ 첫 팔이 움직이기 전에 나머지 실물 팔은 기본 자세로
        #   (지난 실행이 끝난 자리 — 도킹 위 등 — 에 남아 있으면 첫 팔과 부딪힐 수 있다)
        _lead_arm=_lead[-1] if _lead else ACTIVE_ARM[0]
        for _Lp in sorted((set(arms_cfg())&REAL_ARMS)-{_lead_arm}):
            if _Lp!=ACTIVE_ARM[0]: arm_switch(_Lp)
            ROBOT.time_profile(True,sp); ROBOT.torque(True)
            LOG(f"\n[충돌 방지] {arm_label(_Lp)} 먼저 기본 자세로 (실행 시작 전)")
            yield gr.update(),f"{arm_label(_Lp)} — 기본 자세로 (다른 팔이 움직이기 전에)"
            _real_arm_to_base(sp)
        if _lead_arm!=ACTIVE_ARM[0]:
            arm_switch(_lead_arm); LOG(f"\n[팔] {arm_label(_lead_arm)} (첫 이동 전)")
        ROBOT.time_profile(True,sp); ROBOT.torque(True)
        _run_begin(len(S['seq']),sp)            # 타임바 시작 (실물)
        ps=pose_seq()
        LOG("\n"+"="*66)
        LOG(f"[실행 시작] 속도 {sp}x, 반복 {'ON' if loop else 'OFF'}")
        LOG(f"  파이프라인: {' -> '.join(S['seq'])}")
        LOG("="*66)
        q0=ROBOT.read()
        # 첫 블록이 '사물로 이동' 이면 실행 시점 재계산으로 간다 (미리보기와 동일).
        # 관절 보간으로 대충 보내면 축이 어긋나 뒤의 잡기가 "축 위가 아닙니다"로
        # 실패한다 (실물에서만 나던 버그).
        if ps and is_goto_item(ps[0]):
            oid0=ps[0][len(GOTO_PREFIX):]
            # 실물도 미리보기처럼 '기본 자세'에서 출발한다 (어느 자세에서
            # 눌러도 같은 경로가 나오게 — 접힌 자세에서 상승 불가 방지)
            _bq0=get_angles('base')
            if _bq0 is not None:
                # !! 실물로 보내는 값에는 반드시 '보정값'을 더해야 한다.
                #    (사용자가 손으로 맞춰 저장한 base 오프셋. 이걸 빼먹으면
                #     실물이 엉뚱한 자세로 튀어 사람이 다친다 - 실제 사고 발생)
                _off0=level_offset(get_offset('base'))
                _bq0c=[a+o for a,o in zip(_bq0,_off0)]
                _bq0c[GRIP_IDX]=q0[GRIP_IDX]                       # 그리퍼 현재 상태 유지
                if LEVEL_ALWAYS: _bq0c[J5_IDX]=q0[J5_IDX]
                LOG("\n[시작] 기본 자세(base)로 (보정 적용: "
                    +_off_str(get_offset('base'))+")")
                yield gr.update(),"시작: 기본 자세(base)로"
                _w0,_dt0=plan_joint_all(list(q0),_bq0c,steps=30)
                for _q in _w0: _q[GRIP_IDX]=q0[GRIP_IDX]
                if LEVEL_ALWAYS: lock_j5(_w0,q0)
                ros_publish(_bq0)                    # 3D 는 이론 자세 표시
                for _q in _w0:
                    if STOP.is_set(): raise KeyboardInterrupt
                    ROBOT.send(_q); time.sleep(_dt0/sp)
                _settle_or_rest(_bq0c)
            LOG(f"\n[첫 이동] → {disp_name(oid0)} (실행 시점 재계산)")
            yield gr.update(),f"첫 이동 → {disp_name(oid0)}"
            msg0=goto_move(oid0,'첫 이동')
            if msg0: raise RuntimeError(msg0)
            yield gr.update(),f"첫 이동 완료 — {_cr7_mm()}"
            first_done=True
        else:
            first_done=False
        # 이동 블록이 하나도 없어도 된다 (11-04: 잡기·놓기가 스스로 대상 위로 간다)
        if ps:
            first=get_angles(ps[0]); off1=get_offset(ps[0])
            if first is None:
                raise RuntimeError(f"'{ps[0]}' 에 도달할 자세를 만들 수 없습니다"
                                   " — [로봇 자리 추천]을 눌러보세요")
            first_c=[a+o for a,o in zip(first,level_offset(off1))]
            first_c[GRIP_IDX]=q0[GRIP_IDX]                           # 그리퍼는 현재 상태 유지
            # 등록 자세로 갈 때는 그 자세의 j5 를 써야 한다 (현재값에 묶으면
            # 그 자세에 영영 못 간다 — base 로 갔는데 j5 가 어긋나던 버그)
            way,dt=plan_joint_seq(q0,first_c)          # 실물은 보정 적용 목표로
            for f in way: f[GRIP_IDX]=q0[GRIP_IDX]                   # 그리퍼 고정
            if LEVEL_ALWAYS: ramp_j5(way,q0,first_c)   # j5 는 목표까지 부드럽게
            way,dt=densify(way,dt,sp)                  # 모터가 따라올 만큼 잘게
            if not first_done:                         # 등록 자세만 관절 보간으로
                ros_publish(first)                     # 3D 뷰는 정답(이상 자세) 표시
                LOG(f"\n[첫 자세로 이동] {ps[0]}  ({len(way)}점)")
                yield gr.update(),"첫 자세로 이동 중..."
                for q in way:
                    if STOP.is_set(): raise KeyboardInterrupt
                    ROBOT.send(q); time.sleep(dt/sp)
                ok,err_raw=_settle_or_rest(first_c)    # 구형: 쉬기만
                LOG_COMPARE(ROBOT.last_want,ROBOT.last_got,
                            f"{ps[0]} {'도착' if ok else '타임아웃'}")
                yield gr.update(),(f"첫 자세 도착 (오차 {err_raw})" if ok
                                   else f"첫 자세 근접 (오차 {err_raw}, 타임아웃)")
        while True:
            seg_i=0
            # 첫 자세는 위에서 이미 이동함 -> 그 '자세 블록' 하나만 건너뛰고
            # 나머지는 전부 순서대로 (첫 자세 앞의 그리퍼 블록도 실행되어야 함)
            first_pose_i=next((i for i,x in enumerate(S['seq'])
                              if not is_grip_item(x) and not is_brick_item(x)
                              and not is_down_item(x) and not is_up_item(x)
                              and not is_meta_item(x)),None)
            items=[(i,x) for i,x in enumerate(S['seq']) if i!=first_pose_i]
            for _bi,item in items:
                if STOP.is_set() or ESTOP['held']: raise KeyboardInterrupt
                if _pause_point()=='stop': raise KeyboardInterrupt     # 11-06 블록 사이 일시 정지
                _run_at(_bi)                    # 타임바/하이라이트용 위치
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
                    if ('오류' in msg or '없습니다' in msg or '불가' in msg
                            or '아닙니다' in msg):
                        raise RuntimeError(msg)
                    continue
                if is_meta_item(item):                 # 두 팔 블록 (10-30)
                    if is_arm_item(item):
                        _L=item[len(ARM_PREFIX):]
                        LOG(f"\n[팔] {arm_label(_L)}")
                        if _L not in REAL_ARMS:
                            raise RuntimeError(f"{arm_label(_L)} 는 아직 앱에 실물로 연결되지 않았습니다 "
                                               "— 3D 미리보기는 됩니다")
                        if _L!=ACTIVE_ARM[0]:          # 두 팔 (11-02): 그 팔 모터로 바꾸고, 그 팔도 기본 자세에서 출발
                            # ★ 충돌 방지 (11-05 사용자) ★ 먼저 지금 팔을 기본 자세로 — 기본 자세의 팔은 다른 팔이 건드릴 수 없다
                            LOG(f"\n[충돌 방지] {arm_label(ACTIVE_ARM[0])} 먼저 기본 자세로 (다른 팔이 움직이기 전에)")
                            yield gr.update(),f"팔: {arm_label(ACTIVE_ARM[0])} — 기본 자세로 (다른 팔이 움직이기 전에)"
                            _real_arm_to_base(sp)
                            arm_switch(_L)             #   (쉬는 팔은 기본 자세에서 토크를 켠 채 기다린다)
                            ROBOT.time_profile(True,sp); ROBOT.torque(True)
                            yield gr.update(),f"팔: {arm_label(_L)} — 기본 자세로"
                            _real_arm_to_base(sp)
                        yield gr.update(),f"팔: {arm_label(_L)}"
                    else:
                        if wait_arm(item):     # 'A팔 끝날 때까지' (11-03): 지금은 블록을 차례대로 하므로 이미 끝났다
                            LOG(f"\n[기다리기] {wait_text(item)} — 끝났음 (블록을 차례대로 한다)")   #   두 팔을 동시에 돌리게 되면 여기서 기다린다
                            yield gr.update(),f"기다리기 — {wait_text(item)} (끝났음)"
                            continue
                        _w=wait_secs(item); LOG(f"\n[기다리기] {_w:g}초")
                        yield gr.update(),f"기다리기 {_w:g}초"
                        _t0=time.time()
                        while time.time()-_t0<_w:
                            if STOP.is_set(): raise KeyboardInterrupt
                            time.sleep(0.1)
                    continue
                if is_down_item(item):
                    yield gr.update(),"하강: 그 자리에서 수직으로"
                    msg=do_descend_here()
                    yield gr.update(),f"{msg} — {_cr7_mm()}"
                    if '불가' in msg or '오류' in msg or '정지' in msg:
                        yield gr.update(),f"중단: {msg}"; return
                    continue
                if is_up_item(item):
                    yield gr.update(),"상승: 그 자리에서 수직으로"
                    msg=do_ascend_here()
                    yield gr.update(),f"{msg} — {_cr7_mm()}"
                    if '불가' in msg or '오류' in msg or '정지' in msg:
                        yield gr.update(),f"중단: {msg}"; return
                    continue
                if is_grip_item(item):
                    op = (item==GRIP_OPEN_ITEM)
                    tgt=GRIPPER_OPEN if op else GRIPPER_CLOSE
                    LOG(f"\n[그리퍼 {'열기' if op else '닫기'}] 목표 {tgt}")
                    ROBOT.gripper(op)
                    gq=list(CURRENT_Q); gq[GRIP_IDX]=_j6_rad(op)
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
                if is_goto_item(item):
                    # 이동 블록은 실행 시점 재계산 (물체가 옮겨졌을 수 있음)
                    if seg_i<len(segs): seg_i+=1
                    oid=item[len(GOTO_PREFIX):]
                    LOG(f"\n[이동] → {disp_name(oid)} (실행 시점 재계산)")
                    yield gr.update(),f"이동 → {disp_name(oid)}"
                    msg=goto_move(oid,'이동')
                    if msg: raise RuntimeError(msg)
                    yield gr.update(),f"이동 → {disp_name(oid)} 완료 — {_cr7_mm()}"
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
                    if _pause_point()=='stop': raise KeyboardInterrupt   # 11-06 일시 정지
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
                ok,err_raw=_settle_or_rest(q_end)   # 구형: 쉬기만
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
        _run_end()
        ROBOT.estop(); yield gr.update(),"STOPPED - torque off"
    except Exception as e:
        ROBOT.estop(); yield gr.update(),f"ERROR: {e} - torque off"
    finally:
        RUNNING.clear()
        RUN_ACTIVE[0]=False; PAUSE.clear()

# ===== 미세조정 (실물 보정) — 3D 뷰는 정답 유지, 실물만 움직임 =====
FT={'pose':None}
ROBOT_LABEL={'sts':'SO-ARM','dxl':'Dynamixel','both':'SO-ARM'}
_CAL_KEYS={'SO-ARM (STS3215)':'sts','Dynamixel':'dxl'}
def _off_str(off,rk=None):
    rk=rk or offset_robot()
    n=robot_njoints(rk)
    body='  '.join(f"{jname(j) if rk!='dxl' else f'j{j+1}'}:{math.degrees(o):+.1f}"
                   for j,o in enumerate(off[:n]) if rk=='dxl' or not _dead_joint(j))
    return f"[{ROBOT_LABEL.get(rk,rk)}] "+body
def _ensure_pose_arm(name):
    """11-05 (사용자): 자세는 자기 팔이 있다 — 그 자세를 고르거나 보내기 전에 지금 팔을 그 팔로 맞춘다.
    (B팔 자세를 A팔이 걸린 채 [로봇을 이 자세로] 하면 B팔 각도가 A팔 모터로 갈 수 있었다)
    base 는 두 팔 공통이라 바꾸지 않는다 (화면의 'A팔 base / B팔 base' 로 고른다). 팔 표시가 없는 예전 자세는 A팔.
    바꿨으면 그 팔 이름."""
    if not name or name=='base' or RUNNING.is_set(): return None
    rec=_pose_rec(name)
    if rec is None: return None
    L=rec.get('arm') or 'A'
    if L!=ACTIVE_ARM[0] and (L=='A' or L in arms_cfg()):
        arm_switch(L); return L
    return None

def h_ft_goto(name):
    """[자세] 탭의 '로봇을 이 자세로'.
    [이동] 탭의 실행 로봇이 아니라 이 탭에서 고른 '보정할 로봇' 으로 보낸다
    (같은 탭의 미세조정 +/- 와 같은 로봇이어야 보정 작업이 성립한다)."""
    if not name: return "왼쪽 [자세] 탭에서 자세를 먼저 선택하세요"
    if RUNNING.is_set(): return "다른 동작 실행 중 - [정지] 후 다시"
    _ensure_pose_arm(name)                     # 11-05: 그 자세의 팔 모터로
    rk=offset_robot(); dev=ROBOT_DXL if rk=='dxl' else ROBOT_STS
    if rk=='dxl' and not DXL_OK: return "Dynamixel 프로필이 없습니다"
    if not dev.connect():
        return (f"{ROBOT_LABEL.get(rk)} 연결 실패 — 확인: "
                "1) USB/전원  2) 다른 프로그램이 포트를 쓰는 중인지")
    STOP.clear(); RUNNING.set()
    try:
        q0=list(CURRENT_Q)                     # 3D 기준 자세에서 출발
        # '이 자세로 이동' 은 실제 이동이므로 물컵 규칙(수평 유지)을 지킨다.
        #   보정을 생으로 더하면 j2+j3+j4 합이 틀어져 그리퍼가 꺾인다
        #   (실측: base 보정 4/4/3/1 -> 합 +8도 기울어짐)
        #   -> level_offset 으로 j4 상쇄해서 합 0 을 유지한다.
        # (미세조정 +/- 버튼(h_ft_step)은 한 관절만 움직여야 하므로 거기서는
        #  여전히 생값을 쓴다 — 용도가 다르다)
        # 잡기/놓기 자리(상승 전)로 갈 때는 그리퍼를 먼저 연다.
        # 닫힌 채로 내려가면 물체에 부딪힌다 (미세조정용 자세는 물체 바로 위).
        if ('_잡기(' in name or '_놓기(' in name) and rk!='dxl':
            try:
                ROBOT_STS.gripper(True); time.sleep(0.3)
                gq=list(CURRENT_Q); gq[GRIP_IDX]=_j6_rad(True)
                ros_publish(gq,with_grip=True); q0=list(CURRENT_Q)
            except Exception: pass
        off_raw=get_offset(name)
        off=_off_for_send(name,off_raw)            # 11-06: 자리 보정 자세는 사용자가 맞춘 기울기 그대로
        tgt=[a+o for a,o in zip(get_angles(name),off)]
        tgt[GRIP_IDX]=q0[GRIP_IDX]                           # 그리퍼는 현재 상태 유지
        # !! j5 고정(물컵 규칙)은 '경로 중 자세가 기울지 않게' 하는 장치다.
        #    그런데 등록 자세로 갈 때까지 j5 를 현재값에 묶어 버리면 그 자세의
        #    j5 로 절대 못 간다 — base 로 갔는데 j5 가 191도 어긋나던 원인.
        #    목표 자세의 j5 는 그 자세 값 그대로 쓰고, 경로 중에만 부드럽게
        #    보간한다.
        way,dt=plan_joint_seq(q0,tgt)
        for f in way: f[GRIP_IDX]=q0[GRIP_IDX]
        if LEVEL_ALWAYS: ramp_j5(way,q0,tgt)   # j5 는 목표(=수평)까지 부드럽게
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
        _tilt=math.degrees(off_raw[1]+off_raw[2]+off_raw[3])
        return (f"[{ROBOT_LABEL.get(rk)}] '{name}' 도착. "
                "관절 선택 후 +/- 로 미세조정하세요. 현재 보정: "+_off_str(off_raw)
                +(f"\n  (수평 유지: j2+j3+j4 합 {_tilt:+.1f}도를 j4 로 상쇄해 "
                  "보냈습니다 — 물컵 규칙)" if abs(_tilt)>0.1 else ""))
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
    _ensure_pose_arm(name)                     # 11-05: 그 자세의 팔 보정·모터로
    j=int(joint_label[1])-1
    if CAL.get('robot','sts')=='sts' and j<len(J2ID) and J2ID[j] in STS_DEAD_IDS:
        return f"{joint_label} 는 선을 연결하지 않은 고정 관절입니다 (0도 고정, 미세조정 없음)"
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
def seq_expand():
    """파이프라인을 '펼친' 형태로 (보기 전용).

    잡기/놓기는 원초 블록 4개가 뭉친 것이다. 우클릭 펼치기에서 그 속을
    보여주되 편집은 하지 않는다 — 펼친 단계도 파이프라인의 일부로 취급해
    타임바/하이라이트가 세부 단계까지 따라가게 한다.
    반환: [{'blk':원본인덱스,'sub':단계이름 or '','label':표시글}]
    """
    GRASP_STEPS=['열기','하강','닫기','상승']      # 잡기 = 열기→하강→닫기→상승
    RELEASE_STEPS=['하강','열기','상승','닫기']    # 놓기 = 하강→열기→상승→닫기
    out=[]
    for i,it in enumerate(S['seq']):
        if is_grasp_item(it):
            nm=brick_label(it[len(GRASP_PREFIX):])
            out.append({'blk':i,'sub':'','label':f'잡기: {nm}',
                        'steps':GRASP_STEPS})
        elif is_release_item(it):
            nm=dock_label(it[len(RELEASE_PREFIX):])
            out.append({'blk':i,'sub':'','label':f'놓기: {nm}',
                        'steps':RELEASE_STEPS})
        else:
            out.append({'blk':i,'sub':'','label':'','steps':[]})
    return out

SEQ_SEEK={'blk':-1}       # 타임바로 고른 블록 (화면 강조용)

def _seq_pose_at(pos):
    """블록 pos 를 '끝냈을 때' 로봇이 서 있어야 할 관절값을 계산한다.
    재생하지 않고 앞 블록들의 효과만 따져서 바로 구한다 (타임바 점프용).
    반환: (관절값 or None, 설명)"""
    seq=S['seq']
    if not seq: return None,'파이프라인이 비어 있습니다'
    pos=max(0,min(int(pos),len(seq)-1))
    q=get_angles('base') or [0.0]*NJOINT
    held=None                      # 그 시점에 들고 있는 물체
    pos_map={b:(v.get('pos') if v.get('pos') else None)
             for b,v in BRICKS.items()}
    what=''
    for i in range(pos+1):
        it=seq[i]
        if is_goto_item(it):
            oid=it[len(GOTO_PREFIX):]
            # 트윈은 이론 각도 (자리 보정은 실물 전송 때 _ik_off 가 더한다)
            qq=goto_q(oid)
            if qq is None: return None,f'{disp_name(oid)} 접근점 도달 불가'
            q=list(qq); what=f'이동 → {disp_name(oid)}'
        elif is_grasp_item(it):
            b=it[len(GRASP_PREFIX):]
            p=brick_grasp_point(b)
            if p is None: return None,f'{b} 가 배치되지 않았습니다'
            # 잡기 끝 = 상승까지 마친 상태
            top=float(BRICKS[b]['pos'][2])+float(BRICKS[b].get('h',0.0))
            z=max(float(p[2])+GRASP_LIFT,top+0.02)
            qq=ik_tool((float(p[0]),float(p[1]),z),_seed_list(q),q[4],q[GRIP_IDX])
            if qq is None: return None,f'{disp_name(b)} 잡고 상승 불가'
            q=list(qq); held=b; what=f'잡기: {disp_name(b)} (상승 완료)'
        elif is_release_item(it):
            d=it[len(RELEASE_PREFIX):]
            t=dock_socket_world(d)
            if t is None: return None,f'{d} 소켓을 찾을 수 없습니다'
            zg=float(t[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT
            qq=ik_tool((float(t[0]),float(t[1]),zg+GRASP_LIFT),
                       _seed_list(q),q[4],q[GRIP_IDX])
            if qq is None: return None,f'{disp_name(d)} 놓고 상승 불가'
            q=list(qq); held=None; what=f'놓기: {disp_name(d)} (상승 완료)'
        elif is_down_item(it) or is_up_item(it):
            tp=tool_pos(q)
            x,y=float(tp[0]),float(tp[1])
            tz=None
            for ob,bb in BRICKS.items():
                bp=bb.get('pos')
                if bp and ob!=held and math.hypot(bp[0]-x,bp[1]-y)<0.025:
                    tz=float(brick_grasp_point(ob)[2]); break
            if tz is None:
                for dk in docking_ids():
                    t=dock_socket_world(dk)
                    if t and math.hypot(t[0]-x,t[1]-y)<0.025:
                        dt=docking_top(dk)
                        tz=float(dt[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT; break
            if tz is None: tz=PLATE_TOP+GRASP_DZ_DEFAULT
            if is_up_item(it): tz=tz+GRASP_LIFT
            qq=ik_tool((x,y,tz),_seed_list(q),q[4],q[GRIP_IDX])
            if qq is None:
                return None,f"{'하강' if is_down_item(it) else '상승'} 도달 불가"
            q=list(qq); what='하강' if is_down_item(it) else '상승'
        elif is_grip_item(it):
            what='그리퍼 '+('열기' if it==GRIP_OPEN_ITEM else '닫기')
        elif is_meta_item(it):
            what=(arm_label(it[len(ARM_PREFIX):]) if is_arm_item(it)
                  else f'기다리기 {wait_text(it)}')
        else:
            qq=get_angles(it)
            if qq is None: return None,f"'{it}' 자세를 만들 수 없습니다"
            q=list(qq); what=str(it)
    return q,what

def h_seq_seek(pos):
    """타임바를 끌었을 때: **처음부터 재생하지 않고** 그 블록의 자세로
    바로 점프한다. 화면에서는 그 블록이 강조된다."""
    try: _switch_to_block_arm(int(pos))       # 11-05: 그 블록의 팔로 (B팔 블록을 A팔로 풀지 않게)
    except Exception: pass
    q,what=_seq_pose_at(pos)
    if q is None:
        SEQ_SEEK['blk']=-1
        return f'그 지점으로 갈 수 없습니다 — {what}'
    SEQ_SEEK['blk']=int(pos)
    ros_publish(q)                       # 3D 만 (실물은 안 움직임)
    tp=tool_pos(q)
    return (f"{int(pos)+1}번 블록 자세 — {what} · cr_7 "
            f"({tp[0]*1000:.0f},{tp[1]*1000:.0f},{tp[2]*1000:.0f})mm")

def h_preview_from(pos,speed='1.5x'):
    """타임바로 고른 지점부터 이어서 재생 (처음부터 다시 안 함).

    ① 그 지점의 자세/세계 상태를 먼저 만들고(_seq_pose_at + 물체 위치 반영)
    ② 나머지 블록만 재생한다."""
    orig=list(S['seq'])
    try: pos=int(pos)
    except Exception: pos=0
    pos=max(0,min(pos,len(orig)-1))
    # ① 그 지점 상태 만들기 (실물은 안 움직임 — 시뮬 전용)
    q,what=_seq_pose_at(pos)
    if q is None:
        yield gr.update(),f'그 지점에서 시작할 수 없습니다 — {what}'; return
    _snap_pos={b:v.get('pos') for b,v in BRICKS.items()}
    _snap_held=HELD.get('brick')
    FORCE_SIM[0]=True
    try:
        # 앞 블록들의 '물체 이동' 효과를 세계에 반영 (놓기까지 진행된 상태)
        for i in range(pos+1):
            it=orig[i]
            if is_release_item(it) and HELD.get('brick'):
                place_brick_on_docking(HELD['brick'],it[len(RELEASE_PREFIX):])
                HELD['brick']=None
            elif is_grasp_item(it):
                HELD['brick']=it[len(GRASP_PREFIX):]
        ros_publish(q)
    finally:
        FORCE_SIM[0]=False
    # ② 나머지만 재생
    rest=orig[pos+1:]
    if not rest:
        yield gr.update(),f'{pos+1}번 블록이 마지막입니다 (재생할 게 없습니다)'
        return
    with LOCK: S['seq'][:]=rest
    _clear_plan_cache()
    try:
        for out in h_preview('간편 이동',MODES[0],'x-z-y',str(speed),
                             False,False,end_base=False):
            yield out
    finally:
        with LOCK: S['seq'][:]=orig
        _clear_plan_cache()
        for _b,_p in _snap_pos.items():
            if _b in BRICKS: BRICKS[_b]['pos']=_p
        HELD['brick']=_snap_held
        SEQ_SEEK['blk']=-1

def h_seq_goto(pos,speed='1.5x'):
    """타임바 / 우클릭 [이 자세로 이동]: '이 블록까지' 재생하고
    **그 자리에 멈춘다** (base 복귀 안 함 — 거기서 자세 등록·미세조정해야 하므로)."""
    for out in h_preview_upto(pos,speed,end_base=False):
        yield out

# ===== 조이콘 조종 (IMU) =====
# 브라우저(윈도우)가 조이콘 기울기를 읽어 '이만큼 움직여라'(dx,dy,dz)를 보낸다.
# 여기서는 그 값을 axis_7(cr_7) 목표점에 더하고 수평 유지 IK 로 자세를 만든다.
#   · 물컵 규칙: ik_tool 이 4축 체인으로 풀고 j5 를 고정하므로 자동 보장
#   · 실물로는 안 나간다 (3D 만). 실물 연결은 다음 단계
JOY={'on':False,'tgt':None,'grip':False,'warp':None}

# 로봇 목록에서 뺄 폴더 — 로봇이 아닌 것들
_NOT_ROBOT={'bricks'}
# 자세 URDF 를 걸러내는 표시 (1st/2nd/3rd position 등)
_POSE_MARK=('_1st','_2nd','_3rd','_4th','position','_pose')

def _calib_poses(pkg_dir):
    """그 패키지에 저장된 자세 개수 (보정을 얼마나 해뒀는지)."""
    try:
        with open(os.path.join(pkg_dir,'calib.json')) as f:
            return len(json.load(f).get('poses') or [])
    except Exception:
        return 0

def _rank(r):
    return (r['cur'], r['base'], r['npose'], r['mtime'])

def _robot_label(fam,pkg):
    """사람이 알아볼 이름. 날짜/군더더기를 뗀다."""
    if fam=='sts3215':   return 'SO-ARM (STS3215)'
    if fam=='dynamixel': return 'SO-ARM 구형 (Dynamixel)'
    if pkg.lower().startswith('ur15'): return 'UR15'
    return pkg

def list_robots():
    """쓸 수 있는 로봇 목록 — **로봇 한 종류당 딱 하나(base)** 만 준다.
    자세 URDF(1st/2nd/3rd position)와 물체(bricks)는 목록에 넣지 않는다.
    반환: [{'name','urdf','family','active'}]"""
    import glob as _g
    cur=os.path.abspath(BASE_URDF or '')
    byfam={}
    for u in sorted(_g.glob(os.path.join(URDF_ROOT,'*','*','urdf','*.urdf'))
                    +_g.glob(os.path.join(URDF_ROOT,'*','urdf','*.urdf'))):
        if u.endswith('_limits.urdf'): continue
        pkg=os.path.basename(os.path.dirname(os.path.dirname(u)))
        fam=os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(u))))
        if fam=='urdf': fam=''
        key=fam or pkg
        if key in _NOT_ROBOT: continue
        low=pkg.lower()
        is_base='base' in low
        # 자세 URDF 는 버린다 (단, 그게 지금 쓰는 로봇이면 남긴다)
        if not is_base and any(t in low for t in _POSE_MARK) \
           and os.path.abspath(u)!=cur:
            continue
        # 같은 로봇이 여러 개면 고르는 순서:
        #   ① 지금 쓰는 것  ② base  ③ 보정 데이터(자세)가 많은 것  ④ 최신
        # ③ 이 중요하다 — 사장님이 실제로 보정해 둔 패키지를 골라야 한다.
        rec={'urdf':u,'pkg':pkg,'fam':fam,'base':is_base,
             'cur':os.path.abspath(u)==cur,
             'npose':_calib_poses(os.path.dirname(os.path.dirname(u))),
             'mtime':os.path.getmtime(u)}
        old=byfam.get(key)
        if old is None or _rank(rec)>_rank(old):
            byfam[key]=rec
    out=[]
    for key,v in byfam.items():
        out.append({'name':_robot_label(v['fam'],v['pkg']),
                    'pkg':v['pkg'],'urdf':v['urdf'],'family':v['fam'],
                    'active':os.path.abspath(v['urdf'])==cur})
    out.sort(key=lambda r:r['name'])
    return out

def h_robot_list():
    """로봇 목록 (화면 표시용 문자열 포함)."""
    rs=list_robots()
    if not rs: return {'robots':[]},'쓸 수 있는 로봇이 없습니다'
    cur=next((r['name'] for r in rs if r['active']),'?')
    return {'robots':rs},f"로봇 {len(rs)}종 · 지금: {cur}"

def _reload_robot(urdf_path):
    """URDF 를 **앱 재시작 없이** 통째로 갈아끼운다.
    시작할 때 한 번만 계산하던 값들을 전부 다시 계산한다."""
    global BASE_URDF, URDF_PKG_DIR, PROJECT_FILE, MESH_DIR
    global JOINTS, NJOINT, CHAIN, TOOL_OFF, GRIP_IDX
    global LEVEL_ALWAYS, LEVEL_END, CURRENT_Q, _JOY_TBL_FILE
    global J2ID, URDF_LIMITS, BASE_DEF
    global ROBOT_LINKS, ROBOT_COLORS, ROBOT_COLOR_MAP, _LINK_MESH, _COLL_LINKS

    if ACTIVE_ARM[0]!='A': arm_switch('A')   # 두 팔 (11-02): 로봇을 바꾸기 전에 A팔 번호로 돌려놓는다
    BASE_URDF=urdf_path
    URDF_PKG_DIR=os.path.dirname(os.path.dirname(BASE_URDF))
    PROJECT_FILE=os.path.join(URDF_PKG_DIR,'calib.json')
    MESH_DIR=os.path.join(os.path.dirname(BASE_URDF),'..','meshes')

    BASE_DEF=_parse_urdf(BASE_URDF)     # 관절 기하(위치·축) — 로봇마다 다르다
    JOINTS=_urdf_joints(BASE_URDF)
    NJOINT=len(JOINTS)
    J2ID=_make_j2id(BASE_URDF,NJOINT)   # 관절 -> 모터 ID (축 수가 바뀌면 같이 바뀐다)
    _ARM_A['j2id']=list(J2ID)           # 두 팔 (11-02): 새 로봇의 A팔 번호 (바꾸기 전에 A팔로 돌려놨다)
    URDF_LIMITS=_urdf_limits(BASE_URDF) # 가동범위도 이 URDF 것으로
    CHAIN=build_chain()
    TOOL_OFF=_tool_offset()
    GRIP_IDX=NJOINT-1
    LEVEL_ALWAYS=(NJOINT==6)      # 6축(SO-ARM)만 원래 물컵 규칙
    LEVEL_END=(NJOINT!=6)         # 그 외는 변형 물컵 규칙(끝단 수평)
    CURRENT_Q=[0.0]*NJOINT        # 자세는 URDF 기본(base)으로

    # ===== 3D 모양(STL) 도 이 로봇 것으로 =====
    # 두 로봇이 링크 이름을 똑같이 쓴다(l1~l6). 여기서 새로 등록하지 않으면
    # 이름만 맞고 파일은 예전 로봇 것이라 '옛 부품으로 조립된' 3D 가 나온다.
    _old_links=list(ROBOT_LINKS)
    ROBOT_LINKS=_urdf_links(BASE_URDF)
    ROBOT_COLORS=['#8a8a8a','#5b8dbf','#6a9bcf','#5b8dbf','#6a9bcf','#5b8dbf','#e0a13a']
    while len(ROBOT_COLORS)<len(ROBOT_LINKS):
        ROBOT_COLORS.insert(-1,'#6a9bcf' if len(ROBOT_COLORS)%2 else '#5b8dbf')
    ROBOT_COLOR_MAP=dict(zip(ROBOT_LINKS,ROBOT_COLORS))
    _env_ids={e['id'] for e in ENV_SHOW}       # 설비 STL 은 건드리면 안 된다
    for _ln in _old_links:                     # 예전 로봇 링크 등록 지우기
        if _ln not in _env_ids: MESH_REG.pop(_ln,None)
    for _ln in ROBOT_LINKS:                    # 새 로봇 링크 등록 (이 로봇 폴더)
        MESH_REG[_ln]=(os.path.join(MESH_DIR,_ln+'.STL'),1.0)
    # 링크 메쉬를 실제로 다시 읽는다 (충돌검사가 이 목록을 순서대로 쓴다)
    _LINK_MESH=[_load_dec(os.path.join(MESH_DIR,_ln+'.STL')) for _ln in ROBOT_LINKS]
    _COLL_LINKS=None                           # 충돌검사용 캐시 무효화
    ROBOT_GEN[0]+=1                            # 브라우저가 STL 을 새로 받게 함

    # 조종표도 이 로봇 것으로 (로봇마다 표가 다르다)
    _HEAD_R[0]=None; _HEAD_CHAIN[0]=None   # 닭 머리 기준/사슬 다시 계산
    _HEAD_CACHE.clear()
    try: head_load_any()                   # 이 로봇의 저장된 표가 있으면
    except Exception: pass
    _JOY_TBL_FILE=os.path.join(URDF_PKG_DIR,'joy_table.json')
    JOY_TABLE.clear(); JOY['tgt']=None; JOY['on']=False

    # 이 로봇의 프로젝트(자세·보정·공장 배치)를 통째로 불러온다.
    # ★ 예전엔 poses/seq/macros 만 읽었다. 그러면 names·env_clones·
    #   plates_added 같은 '공장 꾸미기' 항목이 메모리에서 비어버리고,
    #   그 상태로 저장이 일어나면 calib.json 에서 통째로 지워진다.
    #   (2026-08-24 실제로 설비·이름·자세가 날아갔다)
    #   -> 시작할 때 읽는 항목을 하나도 빠짐없이 똑같이 읽는다.
    S['poses']=[{'name':'base','angles':[0.0]*NJOINT}]
    S['seq']=['base']
    for _k,_v in (('moves',{}),('axorders',{}),('rotmodes',{}),('macros',{}),
                  ('names',{}),('brick_pos',{}),('env_clones',{}),
                  ('env_removed',[]),('env_vis',{}),('factory_presets',{}),
                  ('plates_added',[]),('env_off',{}),('slot_off',{})):
        S[_k]=type(_v)()
    try:
        with open(PROJECT_FILE) as f: d=json.load(f)
        if d.get('poses'):
            S['poses']=d['poses']
            for _p in S['poses']:                    # 축 수가 다르면 길이 맞추기
                a=list(_p.get('angles') or [])
                _p['angles']=(a+[0.0]*NJOINT)[:NJOINT]
        S['seq']=d.get('seq') or ['base']
        for _k in ('moves','axorders','rotmodes','macros','names','brick_pos',
                   'env_clones','env_removed','env_vis','factory_presets',
                   'plates_added','env_off','slot_off','arm_pose','arms','level_trim'):   # 11-05: 두 팔 자리·이름·수평 보정도
            if d.get(_k): S[_k]=d[_k]
        if d.get('robot_xy'): S['robot_xy']=d['robot_xy']
        if d.get('robot_yaw') is not None: S['robot_yaw']=d['robot_yaw']
        if not any(p['name']=='base' for p in S['poses']):
            S['poses'].insert(0,{'name':'base','angles':[0.0]*NJOINT})
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f'  (프로젝트 읽기 실패: {e} — 기본값으로 시작)')

    # ★ 공장 배치를 화면(ENV_SHOW)에 실제로 다시 올린다 ★
    #   S['env_clones'] 만 읽어놓고 화면 목록을 안 만들면, 데이터는 있는데
    #   3D 에 아무것도 안 보인다 (2026-08-24: 복제 설비가 사라져 보였다).
    #   시작할 때 하는 일과 똑같이 해준다.
    for _cid,_cbase in (S.get('env_clones') or {}).items():
        if _cbase in MESH_REG and not any(e['id']==_cid for e in ENV_SHOW):
            MESH_REG[_cid]=MESH_REG[_cbase]
            _ccol=next((e['color'] for e in ENV_SHOW if e['id']==_cbase),'#b8865b')
            ENV_SHOW.append({'id':_cid,'color':_ccol,'visible':True})
    for _rid in (S.get('env_removed') or []):
        ENV_SHOW[:]=[e for e in ENV_SHOW if e['id']!=_rid]
    for _e in ENV_SHOW:                      # 표시 여부 복원
        _v=(S.get('env_vis') or {}).get(_e['id'])
        if _v is not None: _e['visible']=bool(_v)
    if '_OBS' in globals(): _OBS['grid']=None    # 장애물 격자 다시 계산

    # ===== 옛 로봇 기준으로 만든 표/캐시는 전부 무효 =====
    # 로봇이 바뀌면 '관절각 -> 실제 위치' 관계가 통째로 달라진다.
    # 안 지우면 옛 로봇 기준으로 도달 판정·경로를 계산해 엉뚱하게 움직인다.
    _GOTO_Q.clear(); _clear_plan_cache()
    _MAXZ_R.clear(); _MAXZ_KEY[0]=None          # 도달상한
    globals()['_ARM_REACH']=None                # 팔 도달범위
    globals()['_CHAIN_L']=None                  # 수평유지용 사슬
    globals()['_RPINS']=None                    # 로봇 발(핀) 위치
    if 'PEG_TABLE' in globals():                # 목심 도달표
        PEG_TABLE['key']=None; PEG_TABLE['span']={}
    if '_PEG_CACHE' in globals(): _PEG_CACHE.clear()
    if 'PEG_VIEW' in globals(): PEG_VIEW['dirty']=True
    if '_OBS' in globals(): _OBS['grid']=None   # 장애물 격자(로봇 발자국 포함)

    try: load_peg_table()         # 이 로봇에 저장된 목심 도달표가 있으면 사용
    except Exception: pass
    # 조종표(수백 MB)를 미리 뒤에서 읽어둔다. 조이콘을 잡은 뒤에 읽기
    # 시작하면 15초쯤 반응이 없다 (UR15 표는 263MB).
    try: _joy_table_load_bg()
    except Exception: pass
    _maxz_load()                  # 이 로봇의 도달상한 캐시 불러오기
    ros_publish(CURRENT_Q)        # 3D 를 기본 자세로
    return NJOINT


def h_robot_pick(name):
    """로봇 바꾸기 — **앱을 다시 켜지 않고 바로** 바뀐다.
    자세 같은 건 안 따지고 무조건 그 로봇의 base 자세로 시작한다."""
    for r in list_robots():
        if r['name']==name or r.get('pkg')==name:
            if os.path.abspath(r['urdf'])==os.path.abspath(BASE_URDF or ''):
                return f"이미 '{name}' 입니다"
            try:
                n=_reload_robot(r['urdf'])
            except Exception as e:
                return f"로봇을 바꾸지 못했습니다: {e}"
            with open(BASE_URDF_CFG,'w') as f: f.write(r['urdf'])   # 다음에도 이 로봇
            BTN('로봇 선택',name)
            rule='물컵 규칙' if LEVEL_ALWAYS else '변형 물컵 규칙(끝단 수평)'
            return (f"로봇을 '{name}' 로 바꿨습니다 — {n}축 · {rule} · base 자세\n"
                    f"  조종표: {'있음' if os.path.exists(_JOY_TBL_FILE) else '없음'}")
    return f"그런 로봇이 없습니다: {name}"

def h_home_zero(preset=''):
    """로봇을 URDF 기본 자세(전 관절 0도)로 되돌린다.
    조이콘 조종 등으로 자세가 꼬였을 때 기준으로 돌아오는 용도.
    3D 만 움직인다 (실물은 안 나감).

    ★ 물체(펜)도 배치 프리셋 위치로 (2026-10-20, 사용자) ★
      실물 실행이 펜을 다른 도킹에 놓고 끝나면 트윈 속 펜이 거기 남아,
      손으로 펜을 되돌려 놓아도 트윈은 옛 자리를 믿었다 (말로 만들기가 엉뚱해짐).
      -> [불러오기] 를 누르면 나오는 그 배치(화면에서 고른 것, 없으면 마지막으로
         불러온 것)의 물체 위치로 되돌린다. 설비·로봇 위치는 건드리지 않는다."""
    _pn=str(preset or '') or LAST_PRESET[0]
    _nb=_bricks_to_preset(_pn) if _pn else None
    q=[0.0]*NJOINT
    if 'GRIP_STATE' in globals():
        q[GRIP_IDX]=GRIP_STATE.get('rad',0.0)      # 그리퍼 상태는 유지
    ros_publish(q)
    JOY['tgt']=None                                 # 조이콘 목표점도 초기화
    if JOY.get('on'):
        p=tool_pos(q)
        JOY['tgt']=[float(p[0]),float(p[1]),float(p[2])]
    BTN('로봇 원위치','전 관절 0도')
    p=tool_pos(q)
    _bm=(f" · 물체 {_nb}개를 배치 '{_pn}' 위치로" if _nb is not None
         else (f" · 배치 '{_pn}' 를 찾지 못해 물체는 그대로" if _pn
               else " · 불러온 배치가 없어 물체는 그대로"))
    return (f"로봇 원위치 (URDF 기본 자세) — 툴 "
            f"({p[0]*1000:.0f},{p[1]*1000:.0f},{p[2]*1000:.0f})mm"+_bm)

def h_joy_start():
    """조이콘 조종 시작 — 지금 3D 자세의 툴 위치를 출발점으로 삼는다.

    단, base 처럼 팔을 위로 뻗은 자세는 '수평 유지' 해가 없다
    (물컵 규칙을 지키면서는 그 높이에 못 간다).
    그런 자리에서 시작하면 첫 이동부터 전부 막히므로,
    수평 유지가 가능한 높이까지 자동으로 내려서 시작한다."""
    q0=list(CURRENT_Q)
    p=tool_pos(q0)
    x,y,z=float(p[0]),float(p[1]),float(p[2])
    q=ik_tool((x,y,z),_seed_list(q0),q0[4],q0[GRIP_IDX])
    moved=0.0
    if q is None:
        z0=z
        while z>PLATE_TOP+0.05:
            z-=0.01
            q=ik_tool((x,y,z),_seed_list(q0),q0[4],q0[GRIP_IDX])
            if q is not None: break
        moved=z0-z
    if q is None:
        return "조종을 시작할 수 없습니다 — 이 자리에서는 수평 자세가 안 나옵니다"
    if LEVEL_ALWAYS: q[J5_IDX]=q0[J5_IDX]
    q[GRIP_IDX]=q0[GRIP_IDX]
    ros_publish(q)
    JOY['tgt']=[x,y,z]
    JOY['on']=True
    if not JOY_TABLE:
        # 표가 100MB 라 읽는 데 5초쯤 걸린다. 앞에서 기다리면 앱이 멈춘 것처럼
        # 보이므로 뒤에서 읽는다. 다 읽히기 전에는 그때그때 IK 로 움직인다.
        _joy_table_load_bg()
    BTN('조이콘 조종','시작')
    msg=(f"조이콘 조종 시작 — 출발점 ({x*1000:.0f},{y*1000:.0f},{z*1000:.0f})mm")
    if moved>0.001:
        msg+=(f"\n  * 물컵 규칙(수평)을 지킬 수 있는 높이까지 "
              f"{moved*1000:.0f}mm 내렸습니다")
    return msg

def h_joy_stop():
    JOY['on']=False
    BTN('조이콘 조종','정지')
    return "조이콘 조종 정지"

# ===== 조종용 IK 미리계산 표 =====
# 조종은 초당 20번 IK 를 푼다. 미리 격자로 풀어 저장해두면 그 지점들은
# 계산 없이 즉시 꺼내 쓸 수 있다 (SO-ARM 의 목심 도달표와 같은 발상).
#   키: (x,y,z) 를 GRID mm 로 반올림   값: 관절값
JOY_TABLE={}
JOY_GRID=0.02                    # 20mm 격자
# 표는 로봇마다 따로 (기구학이 다르므로 섞이면 안 된다).
# URDF 패키지 폴더 안에 두면 로봇을 바꿔도 각자 자기 표를 쓴다.
_JOY_TBL_FILE=os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(BASE_URDF)))
    if BASE_URDF else os.path.dirname(os.path.abspath(__file__)),
    'joy_table.json')

def _joy_key(p, world=True):
    """표의 키. **로봇 기준 좌표**로 만든다.

    월드 좌표로 키를 만들면 로봇을 옮기는 순간 표가 전부 어긋난다.
    로봇 기준으로 담아두면 로봇을 어디에 놓든 그대로 쓸 수 있다
    (목심 도달표를 상대좌표로 바꿨던 것과 같은 이유).
    world=True 면 월드 좌표를 받아 로봇 기준으로 바꿔서 키를 만든다."""
    q=w2l(np.asarray(p,float)) if world else np.asarray(p,float)
    g=JOY_GRID
    return (round(float(q[0])/g), round(float(q[1])/g), round(float(q[2])/g))

_JOY_LOADING=[False]
def _joy_table_load_bg():
    """조종표를 뒤에서 읽어들인다 (앱이 멈추지 않게).

    닭 머리 로봇에서는 읽지 않는다 — 옛 표는 world 모드라 지금 관문에
    전부 걸려 쓰이지 않는데, 읽는 데만 15초에 메모리도 크게 먹는다."""
    if NJOINT>=7 and HEAD_LOCK[0]: return
    if _JOY_LOADING[0] or JOY_TABLE: return
    _JOY_LOADING[0]=True
    def _w():
        try:
            t0=time.time(); n=joy_table_load()
            if n: print(f"조종표 {n:,}칸 불러옴 "
                        f"({time.time()-t0:.1f}초) — 계산 없이 즉시 이동")
        except Exception as e:
            print('조종표 불러오기 실패:',e)
        finally:
            _JOY_LOADING[0]=False
    threading.Thread(target=_w,daemon=True).start()

def joy_table_load():
    """저장된 표를 불러온다 (로봇이 바뀌면 무시)."""
    global JOY_GRID
    try:
        with open(_JOY_TBL_FILE) as f: d=json.load(f)
        if d.get('urdf')!=os.path.basename(BASE_URDF or ''): return 0
        # ★ 표를 만들 때 쓴 격자를 그대로 따라간다 ★
        #   빌더가 10mm 로 만들었는데 앱이 20mm 로 찾으면 한 칸도 못 맞춘다.
        g=d.get('grid')
        if g and abs(float(g)-JOY_GRID)>1e-9:
            JOY_GRID=float(g)
            print(f'  조종표 격자 {JOY_GRID*1000:.0f}mm 로 맞춤')
        # 표는 만들 때 이미 '평평한 칸'만 담겨 있다(build_joy_table.py).
        # 그래도 옛날 표가 섞일 수 있으니 조회할 때 한 번 더 확인한다.
        JOY_TABLE.clear()
        for k,v in d.get('tbl',{}).items():
            JOY_TABLE[tuple(int(x) for x in k.split(','))]=v
        return len(JOY_TABLE)
    except FileNotFoundError:
        return 0
    except Exception as e:
        print(f'조종표를 읽지 못했습니다 ({e}) — 표 없이 진행합니다')
        return 0

def joy_table_save():
    """조종표를 저장한다.
    임시 파일에 다 쓴 뒤 한 번에 바꿔치기한다(원자적 저장).
    이렇게 해야 저장 도중에 멈춰도 기존 표가 깨지지 않는다."""
    # 임시 파일 이름에 프로세스 번호를 넣는다.
    # (여러 곳에서 저장이 겹치면 서로의 .tmp 를 지워 저장이 통째로 실패한다
    #  — 실제로 "No such file or directory: ...json.tmp" 오류가 났다)
    tmp=f'{_JOY_TBL_FILE}.{os.getpid()}.tmp'
    try:
        with open(tmp,'w') as f:
            json.dump({'urdf':os.path.basename(BASE_URDF or ''),
                       'grid':JOY_GRID,
                       'tbl':{f"{k[0]},{k[1]},{k[2]}":v
                              for k,v in JOY_TABLE.items()}},f)
            f.flush(); os.fsync(f.fileno())
        os.replace(tmp,_JOY_TBL_FILE)
    except Exception as e:
        print('조종표 저장 실패:',e)
        try: os.remove(tmp)
        except Exception: pass

def h_joy_table_build(span_mm=400,step_mm=None):
    """지금 툴 위치 주변을 격자로 미리 풀어 표에 담는다.
    한 번 만들어두면 그 범위 안에서는 IK 계산 없이 즉시 움직인다."""
    step=(step_mm or JOY_GRID*1000)/1000.0
    half=float(span_mm)/2000.0
    q0=list(CURRENT_Q)
    c=tool_pos(q0)
    n=0; tried=0
    rng=np.arange(-half,half+1e-9,step)
    t0=time.time()
    BTN('조종표 만들기',f'{span_mm}mm 범위')
    for dx in rng:
        for dy in rng:
            for dz in rng:
                tgt=(float(c[0])+dx,float(c[1])+dy,float(c[2])+dz)
                if tgt[2]<PLATE_TOP+0.02: continue
                k=_joy_key(tgt)
                if k in JOY_TABLE: continue
                tried+=1
                q=joy_ik_fast(tgt,q0)
                if q is not None:
                    JOY_TABLE[k]=[round(float(v),5) for v in q]
                    q0=q                      # 이어서 풀면 더 빠르고 부드럽다
                    n+=1
    joy_table_save()
    dt=time.time()-t0
    return (f"조종표 {n}칸 생성 ({tried}칸 시도, {dt:.0f}초)\n"
            f"  격자 {JOY_GRID*1000:.0f}mm · 범위 {span_mm}mm\n"
            f"  이제 이 범위 안에서는 계산 없이 즉시 움직입니다")

# 조이콘 한 걸음(약 20mm)에 허용할 관절 변화. 이보다 크면 '팔이 튄 것'
# 으로 보고 정식 IK(최소 움직임 해 선택)로 다시 푼다.
JOY_MAX_JOINT_DEG=6.0
# 조종 중 느린 정식 IK 를 쓸지. 끄면 반응이 빠르고, 못 가는 방향은
# 그냥 '막힘' 으로 표시된다 (반대로 움직이면 된다).
JOY_SLOW_FALLBACK=[False]

def joy_ik_fast(tgt,q0,check_move=True):
    """조이콘 조종 전용 빠른 IK.

    일반 IK 는 여러 씨앗으로 여러 번 풀어 최선을 고르느라 1~2초가 걸린다.
    조종은 초당 20번 불리므로 그러면 조작이 밀린다.
    여기서는 **직전 자세 하나만 씨앗으로 한 번만** 푼다 —
    이동량이 작아서(수 mm) 그것으로 충분하고, 결과도 직전 자세에
    가까워 팔이 튀지 않는다.

    ★ 최소 움직임 검사 ★ 여유자유도 로봇(UR15 8축)은 같은 위치를 만드는
    자세가 무수히 많다. 씨앗을 줘도 IK 가 멀리 떨어진 해로 수렴할 수 있어
    20mm 움직였는데 관절이 7도씩 돌아간다. 그래서 '너무 많이 움직인 해'는
    버리고 호출측이 정식 IK 로 다시 풀게 한다.
    """
    try:
        n=len(CHAIN.links)
        init=[0.0]*n
        for k,v in enumerate(list(q0)[:NJOINT]):
            if k+1<n: init[k+1]=float(v)
        sol=CHAIN.inverse_kinematics(target_position=w2l(np.asarray(tgt,float)),
                                     initial_position=init)
        q=list(sol[1:1+NJOINT])
        got,_=fk(q)
        if np.linalg.norm(got-np.asarray(tgt,float))>0.006: return None
        # 이동 거리에 견줘 관절이 과하게 돌면(팔 모양이 통째로 바뀜) 버린다
        # check_move=False 면 이 검사를 건너뛴다 (넓은 탐색에서 쓴다 —
        # 거기서는 '멀어도 갈 수 있는 해' 를 일부러 찾는다)
        if check_move:
            moved=float(np.linalg.norm(np.asarray(tgt,float)-np.asarray(tool_pos(q0),float)))
            lim=math.radians(JOY_MAX_JOINT_DEG)*max(1.0,moved/0.02)
            if max(abs(a-b) for a,b in zip(q[:NJOINT],list(q0)[:NJOINT]))>lim:
                return None
        return q
    except Exception:
        return None

# 한 프레임에 옮길 수 있는 최대 관절 변화(도). 먼 해로 갈아탈 때
# 이 정도씩 나눠서 옮긴다 -> 팔이 순간이동하지 않는다.
JOY_WARP_DEG=4.0
# 넓은 탐색(다른 팔 모양으로 갈아타기)은 **여유자유도 로봇에서만** 쓴다.
# SO-ARM(6축)은 해가 몇 개 없고 물컵 규칙이 j5 고정이라, 이걸 켜면
# 갈아탈 자세를 못 찾고 '자세 바꾸는 중' 에 갇힌다 (20mm 대신 1.5mm씩 감).
# (판단은 h_joy_move 안에서 NJOINT>=7 로 직접 한다 — 로봇을 바꿔도 맞게)

# ===== 닭 머리 규칙 (2026-08-24, 사용자 정의) =====
# SO-ARM 이 쉬운 이유는 물컵 규칙(j5 고정)으로 롤·피치·요를 아예 안 건드려
# IK 가 '위치 3개만' 푸는 문제가 되기 때문이다. UR15 도 똑같이 만든다.
#   -> l7(끝단)의 방향을 **base 자세 그대로** 못박고, 위치만 푼다.
#      닭이 몸을 움직여도 머리 방향은 그대로인 것과 같다.
#   손목을 의도적으로 돌리는 건 나중에 IMU 로 따로 계산해 합친다.
HEAD_LOCK=[True]          # 끄면 예전(방향 자유) 방식
_HEAD_R=[None]            # base 자세에서의 l7 회전행렬 (고정 목표)

def head_ref_R():
    """base 자세(전 관절 0도)에서 l7 이 향하는 방향. 한 번만 계산해 둔다."""
    if _HEAD_R[0] is None:
        q=[0.0]*NJOINT
        T=CHAIN.forward_kinematics([0]+q[:len(CHAIN.links)-1],
                                   full_kinematics=True)
        _HEAD_R[0]=np.array(T[HEAD_FRAME[0]][:3,:3],dtype=float)
    return _HEAD_R[0]

# 화면에 보이는 마지막 '부품' 프레임 번호.
# CHAIN 의 끝(T[-1])은 end 좌표계라 눈에 안 보인다. 그 앞이 l7(실물 부품).
# end 만 고정하면 l7 이 48도까지 꺾여도 통과해버린다 (실측).
HEAD_FRAME=[-2]

_HEAD_CHAIN=[None]

def head_chain():
    """닭 머리 IK 전용 사슬 — 마지막 축(end)을 '안 움직이는' 축으로 둔다.

    end 는 l7 뒤에 붙은 가상 축이라, IK 가 이걸 돌려 방향을 맞추면
    정작 눈에 보이는 l7 이 그만큼 꺾인다 (실측 10.28도).
    -> 아예 IK 변수에서 빼서, l7 자체가 방향을 만들게 한다.
    """
    if _HEAD_CHAIN[0] is None:
        mask=list(CHAIN.active_links_mask)
        if len(mask)>=2: mask[-1]=False        # end 비활성
        _HEAD_CHAIN[0]=Chain(name='head',links=CHAIN.links,
                             active_links_mask=mask)
    return _HEAD_CHAIN[0]


def _head_R(q):
    """지금 자세에서 '고정 대상 부품'(l7)의 회전행렬."""
    T=CHAIN.forward_kinematics([0]+list(q)[:len(CHAIN.links)-1],
                               full_kinematics=True)
    return np.array(T[HEAD_FRAME[0]][:3,:3],dtype=float)

def _j1_sign():
    """j1(맨 아래 축)이 +z 로 도는지 -z 로 도는지. URDF 마다 다르다."""
    try:
        ax=np.asarray(BASE_DEF[JOINTS[0]]['axis'],float)
        return -1.0 if ax[2]<0 else 1.0
    except Exception:
        return 1.0

# 닭 머리 규칙에는 두 가지 뜻이 있다.
#   'world' : 노즐이 늘 정면(-Y)만 본다. 팔이 돌면 j5(손목)로 되돌린다.
#             -> 옛 표가 쓰던 방식. j1+j5=0 (실측). 손목을 90도씩 쓴다.
#   'body'  : 팔이 도는 만큼 노즐도 같이 돈다 (닭이 몸을 돌리듯).
#             손목은 안 쓴다. 뒤쪽 공간이 통째로 열린다.
# 사용자 정의: "팔목이 꺾이는 경우도 없어야 해" -> body 가 맞다.
# 'world' : 노즐이 늘 정면(-Y)만 본다. 팔이 돌면 j5(손목)로 되돌린다.
#           도색은 표면을 똑바로 마주봐야 고르게 발리므로 이쪽이 맞다.
#           대가로 손목을 크게 쓴다 (실측 평균 54도, 최대 177도).
# 'body'  : 팔이 도는 만큼 노즐도 같이 돈다. 손목은 안 쓰지만 벽을
#           비스듬히 보는 자리가 많다.
# 사용자 결정(2026-08-26): "손목 꺾기를 안하는 것을 취소한다" -> world
HEAD_MODE=['world']

def head_ref_R_at(q):
    """이 자세에서 l7 이 향해야 할 방향.

    ★ 닭 머리 규칙의 뜻 ★
      "l7 은 base 의 모습 그대로 '위상만' 움직이면 된다"
      = 팔이 통째로 도는 만큼(j1) 주둥이도 같이 돈다. 손목만 안 꺾이면 된다.

    예전에는 base 방향을 월드에 못박아 뒀다. 그러면 노즐이 늘 -Y(정면)만
    보게 되어, 팔을 뒤로 돌리면 오차 180도가 나고 표에서 잘렸다.
    실제로는 j1 만 돌리면 팔 모양이 완벽히 그대로다(실측 차이 0.000000).
    그래서 기준을 j1 만큼 같이 돌려준다 -> 뒤쪽 공간이 통째로 열린다."""
    if HEAD_MODE[0]=='world':
        return head_ref_R()            # 월드에 못박기 (옛 방식)
    a=_j1_sign()*float(q[0])
    ca,sa=math.cos(a),math.sin(a)
    Rz=np.array([[ca,-sa,0.0],[sa,ca,0.0],[0.0,0.0,1.0]])
    return Rz@head_ref_R()

def head_err_deg(q):
    """지금 자세의 l7 방향이 '있어야 할 방향' 에서 몇 도 벗어났나.
    있어야 할 방향 = base 방향을 j1 만큼 돌린 것 (위상만 움직인다)."""
    dR=head_ref_R_at(q).T@_head_R(q)
    c=(np.trace(dR)-1.0)/2.0
    return abs(math.degrees(math.acos(max(-1.0,min(1.0,c)))))

# 하단 관절 우선: j1 이 가장 싸고 뒤로 갈수록 비싸다.
# 같은 목표를 만드는 해가 여럿이면 '아래쪽 관절을 많이 쓴' 해를 고른다.
# (사용자: "가장 하단부터 조금씩 움직여서 해당 위치로 가자")
LOWER_FIRST=[True]

def _joint_cost(q,cur):
    """관절 변화에 가중치를 준다. 위쪽(손목) 관절일수록 비싸다.
    j1=1배, j2=2배, ... 이런 식이라 IK 가 아래쪽을 먼저 쓰게 된다."""
    c=0.0
    for k in range(min(NJOINT,len(q))):
        w=1.0+k*1.5 if LOWER_FIRST[0] else 1.0
        c+=abs(q[k]-cur[k])*w
    return c


_J1_REF=[None]

def _j1_for(pt):
    """이 목표를 향하려면 j1(맨 아래 축)을 얼마나 돌려야 하는가.

    팔 전체가 j1 위에 얹혀 있으므로, j1 은 '목표가 어느 방향에 있는가'
    로 정해진다. 다만 툴은 팔 중심축에서 옆으로 비켜 있어(실측 x=182mm),
    그 오프셋을 그대로 방위각에 넣으면 base 자세에서도 9도가 나온다.
    -> base 자세의 방위각을 기준으로 삼아 그 차이만 쓴다."""
    if _J1_REF[0] is None:
        b=w2l(np.asarray(tool_pos([0.0]*NJOINT),float))
        _J1_REF[0]=math.atan2(float(b[1]),float(b[0]))
    q=w2l(np.asarray(pt,float))
    a=math.atan2(float(q[1]),float(q[0]))-_J1_REF[0]
    a=(a+math.pi)%(2*math.pi)-math.pi          # -180~180 로 접기
    return a/_j1_sign()

def _head_tbl_q(pt,q0,max_jump_deg=25.0):
    """닭 머리 표에서 이 자리의 자세를 꺼낸다 (없으면 None).

    표는 GPU 로 푼 것이라 ikpy 보다 훨씬 잘 맞는다. 다만 표는 '그 칸의
    최선' 일 뿐 지금 자세와 이어진다는 보장이 없으므로, 지금 자세에서
    너무 멀면 쓰지 않는다(팔이 통째로 튀는 것을 막는다)."""
    if _HEAD_ARR['code'] is None: return None
    g=_HEAD_GRID[0]
    loc=w2l(np.asarray(pt,float))
    k=[int(round(float(loc[i])/g)) for i in range(3)]
    v=head_arr_get(k[0],k[1],k[2])
    if v is None: return None
    q=[float(x) for x in v][:NJOINT]
    if len(q)<NJOINT: return None
    lim=math.radians(max_jump_deg)
    if max(abs(a-b) for a,b in zip(q,list(q0)[:NJOINT]))>lim:
        return None
    for j in range(NJOINT):                    # 관절 한계 확인
        lo,hi=jr_limits(j)
        if not (lo-1e-6<=q[j]<=hi+1e-6): return None
    if head_err_deg(q)>4.0: return None        # 닭 머리 규칙
    if LEVEL_END and end_tilt_deg(q)>4.0: return None   # 변형 물컵 규칙
    return q

def ik_head(pt,seeds,tol_deg=1.5):
    """★ 닭 머리 IK ★ 손목은 안 꺾고, 팔은 통째로 돌려 목표를 향한다.

    "l7 은 base 의 모습 그대로 '위상만' 움직인다"
      -> 목표가 뒤에 있으면 j1 으로 팔 전체를 돌린다. 그때 주둥이도
         같이 돈다 (닭이 몸을 돌리는 것). 손목만 안 꺾이면 규칙은 지켜진다.
    예전에는 방향을 월드에 못박아 -Y 만 보게 했더니 뒤쪽이 통째로 막혔다.
    """
    tgt=np.asarray(pt,float)
    a0=_j1_for(tgt)
    n=len(CHAIN.links)
    cur=list(CURRENT_Q)[:NJOINT]
    # j1 후보. 방위각 그대로가 기본이지만, 목표가 팔 길이보다 가까우면
    # 팔을 접어야 하고 그때 j1 이 조금 달라진다 -> 언저리도 함께 시도한다.
    a_cands=[a0]
    for dd in (10,-10,20,-20,35,-35,55,-55,80,-80):
        a_cands.append(a0+math.radians(dd))
    if abs(_j1_for(np.asarray(tool_pos(cur),float))-a0)>1e-9:
        a_cands.insert(1,float(cur[0]))      # 지금 j1 도 후보 (덜 움직인다)
    # 후보를 여러 개 만들어야 '하단 우선' 이 의미가 있다.
    # 씨앗이 하나면 해도 하나라 고를 게 없다.
    # 위쪽 관절을 미리 조금 접어둔 씨앗을 섞어, IK 가 그 근처에서
    # 수렴하도록 유도한다 -> 아래쪽만 움직이는 해가 후보에 들어온다.
    best=None
    trials=[]
    for a1 in a_cands:
        # 이 j1 로 갔을 때 l7 이 있어야 할 방향 (base 방향을 j1 만큼 돌린 것)
        ca,sa=math.cos(_j1_sign()*a1),math.sin(_j1_sign()*a1)
        R1=np.array([[ca,-sa,0.0],[sa,ca,0.0],[0.0,0.0,1.0]])@head_ref_R()
        sd=list(cur); sd[0]=a1                 # 그 방향으로 돌려둔 자세에서 출발
        trials.append((a1,R1,sd))
        sd2=[0.0]*NJOINT; sd2[0]=a1
        trials.append((a1,R1,sd2))
    for sd0 in list(seeds)[:2]:                # 불러준 씨앗도 같이
        a1=_j1_for(tgt)
        ca,sa=math.cos(_j1_sign()*a1),math.sin(_j1_sign()*a1)
        R1=np.array([[ca,-sa,0.0],[sa,ca,0.0],[0.0,0.0,1.0]])@head_ref_R()
        trials.append((a1,R1,list(sd0)))
    for a1,R,sd in trials[:26]:
        init=[0.0]*n
        for k,v in enumerate(list(sd)[:NJOINT]):
            if k+1<n: init[k+1]=float(v)
        # CHAIN 의 끝은 툴 중심이 아니라 j7 프레임이다. 툴 중심을 tgt 에
        # 맞추려면 방향이 R 로 고정돼 있으므로 오프셋을 그대로 빼면 된다.
        # (안 빼면 TOOL_OFF 만큼 어긋난다 — 실측 y -50.9mm)
        aim=w2l(tgt)-(R@np.asarray(TOOL_OFF,float) if TOOL_HAS else 0.0)
        try:
            sol=head_chain().inverse_kinematics(
                target_position=aim, target_orientation=R,
                orientation_mode='all', initial_position=init)
        except Exception:
            continue
        q=list(sol[1:1+NJOINT])
        # ★ end 축은 l7 뒤에 붙은 '가상 축' 이다 ★
        #   IK 가 방향을 맞추려고 end 를 돌리면, 정작 눈에 보이는 부품(l7)이
        #   꺾인다 (실측: end 0.02도인데 l7 은 48.74도 꺾임).
        #   -> end 는 base 값(0)으로 못박아, l7 자체가 방향을 만들게 한다.
        if NJOINT>=8: q[NJOINT-1]=0.0
        # 목표는 '툴 중심' 이므로 tool_pos 로 재야 한다.
        # fk() 는 j7 위치라 TOOL_OFF(50.9mm) 만큼 항상 어긋나 전부 버려졌다.
        got=np.asarray(tool_pos(q),float)
        if np.linalg.norm(got-tgt)>0.006: continue     # 위치 오차 6mm 이내
        e=head_err_deg(q)
        if e>tol_deg: continue                          # 방향이 틀어지면 버림
        d=_joint_cost(q,cur)          # 하단 관절 우선 (가중치)
        if best is None or d<best[0]: best=(d,q)
    return best[1] if best else None


def _joy_wide_ik(tgt,q0,level_ok):
    """'지금 자세 근처' 로는 못 가는 목표를, **다른 팔 모양**으로 풀어본다.

    여유자유도(UR15 8축)에서는 같은 점을 팔꿈치를 위/아래·앞/뒤로 접어
    갈 수 있다. 씨앗을 여러 개 줘서 그런 해를 찾아낸다.
    찾은 것 중 '지금 자세에서 가장 덜 움직이는' 해를 고른다.
    """
    seeds=[list(q0)]
    # 등록 자세들 (사용자가 만들어 둔 다양한 팔 모양)
    for pp in S.get('poses',[])[:6]:
        a=list(pp.get('angles') or [])
        if len(a)>=NJOINT: seeds.append(a[:NJOINT])
    # 팔꿈치를 반대로 접은 모양들 (중간 관절 부호를 뒤집어 본다)
    for k in (1,2,3,4,5):
        if k>=NJOINT: break
        f=list(q0); f[k]=-f[k]; seeds.append(f)
    best=None
    for sd in seeds:
        q=joy_ik_fast(tgt,sd,check_move=False)
        if q is None or not level_ok(q): continue
        d=sum(abs(a-b) for a,b in zip(q[:NJOINT],list(q0)[:NJOINT]))
        if best is None or d<best[0]: best=(d,q)
    if best is not None: return best[1]
    # 그래도 못 찾으면 정식 IK 로 한 번 더 (여러 씨앗 + 최소 움직임 선택).
    # 느리지만(0.3~1초) 여기까지 왔다는 건 '이대로면 막힘' 이라는 뜻이라
    # 한 번 제대로 찾아보는 편이 낫다.
    try:
        q=ik_general(np.asarray(tgt,float),seeds)
        if q is not None and level_ok(q): return q
    except Exception:
        pass
    return None


def _joy_warp_step(q0,tgt_q):
    """먼 자세로 한 번에 뛰지 않고 한 걸음만 다가간다."""
    lim=math.radians(JOY_WARP_DEG)
    d=[b-a for a,b in zip(list(q0)[:NJOINT],tgt_q[:NJOINT])]
    mx=max(abs(x) for x in d) or 1e-9
    r=min(1.0,lim/mx)
    return [a+x*r for a,x in zip(list(q0)[:NJOINT],d)]


_HEAD_CACHE={}          # (칸) -> 자세 (닭 머리 IK 결과 캐시)

def _head_cache_file():
    return os.path.join(URDF_PKG_DIR,'head_table.json')

_HEAD_GRID=[0.01]      # 닭 머리 표가 만들어진 격자 (표 파일이 들고 다닌다)

# ===== 닭 머리 표: 이진 형식(빠른 로딩) =====
# JSON 은 373만칸에 16초 걸린다 (파싱 11.7 + 키변환 4.5).
# npz + 정렬된 정수키 배열이면 0.7초. 조회는 이진탐색 3us.
_HEAD_ARR={'code':None,'val':None}
_HK_OFF=2000; _HK_M=4000

def _hk(kx,ky,kz):
    return ((int(kx)+_HK_OFF)*_HK_M+(int(ky)+_HK_OFF))*_HK_M+(int(kz)+_HK_OFF)

def head_arr_get(kx,ky,kz):
    """배열에서 한 칸 찾기 (없으면 None)."""
    c=_HEAD_ARR['code']
    if c is None: return None
    i=int(np.searchsorted(c,_hk(kx,ky,kz)))
    if i<len(c) and c[i]==_hk(kx,ky,kz):
        return _HEAD_ARR['val'][i].tolist()
    return None

def _head_npz_file():
    return os.path.join(URDF_PKG_DIR,'head_table.npz')

def head_arr_build():
    """dict(_HEAD_CACHE) -> 정렬된 배열. 저장/로딩 모두 이걸 쓴다."""
    items=[(k,v) for k,v in _HEAD_CACHE.items() if v]
    if not items:
        _HEAD_ARR['code']=None; _HEAD_ARR['val']=None; return 0
    K=np.array([k for k,_ in items],dtype=np.int64)
    V=np.array([v for _,v in items],dtype=np.float32)
    code=((K[:,0]+_HK_OFF)*_HK_M+(K[:,1]+_HK_OFF))*_HK_M+(K[:,2]+_HK_OFF)
    o=np.argsort(code)
    _HEAD_ARR['code']=code[o]; _HEAD_ARR['val']=V[o]
    return len(items)

def head_npz_save():
    try:
        n=head_arr_build()
        if not n: return 0
        tmp=f'{_head_npz_file()}.{os.getpid()}.tmp.npz'
        np.savez(tmp,code=_HEAD_ARR['code'],val=_HEAD_ARR['val'],
                 grid=np.float64(_HEAD_GRID[0]),
                 urdf=np.array([os.path.basename(BASE_URDF or '')]))
        os.replace(tmp,_head_npz_file())
        return n
    except Exception as e:
        print('닭 머리 표(npz) 저장 실패:',e); return 0

def head_npz_merge():
    """조종 중 새로 푼 칸(_HEAD_CACHE)을 이진 표에 합쳐 저장한다."""
    try:
        new=[(k,v) for k,v in _HEAD_CACHE.items() if v]
        if not new: return 0
        K=np.array([k for k,_ in new],dtype=np.int64)
        V=np.array([v for _,v in new],dtype=np.float32)
        code=((K[:,0]+_HK_OFF)*_HK_M+(K[:,1]+_HK_OFF))*_HK_M+(K[:,2]+_HK_OFF)
        if _HEAD_ARR['code'] is not None:
            code=np.concatenate([_HEAD_ARR['code'],code])
            V=np.concatenate([_HEAD_ARR['val'],V])
        o=np.argsort(code,kind='stable')
        code=code[o]; V=V[o]
        keep=np.ones(len(code),bool)
        keep[:-1]=code[:-1]!=code[1:]           # 같은 칸은 뒤(새 값)만
        _HEAD_ARR['code']=code[keep]; _HEAD_ARR['val']=V[keep]
        _HEAD_CACHE.clear()
        tmp=f'{_head_npz_file()}.{os.getpid()}.tmp.npz'
        np.savez(tmp,code=_HEAD_ARR['code'],val=_HEAD_ARR['val'],
                 grid=np.float64(_HEAD_GRID[0]),
                 urdf=np.array([os.path.basename(BASE_URDF or '')]))
        os.replace(tmp,_head_npz_file())
        return len(_HEAD_ARR['code'])
    except Exception as e:
        print('닭 머리 표 합치기 실패:',e); return 0

def head_npz_load():
    """빠른 이진 표를 읽는다. dict 는 만들지 않는다(11초 절약)."""
    try:
        z=np.load(_head_npz_file(),allow_pickle=False)
        if str(z['urdf'][0])!=os.path.basename(BASE_URDF or ''): return 0
        _HEAD_GRID[0]=float(z['grid'])
        _HEAD_ARR['code']=z['code']; _HEAD_ARR['val']=z['val']
        return len(_HEAD_ARR['code'])
    except FileNotFoundError:
        return 0
    except Exception as e:
        print('닭 머리 표(npz) 읽기 실패:',e); return 0

def head_load_any():
    """빠른 npz 를 먼저 쓰고, 없으면 JSON 을 읽어 npz 로 바꿔 둔다."""
    n=head_npz_load()
    if n: return n
    n=head_cache_load()          # 옛 JSON
    if n:
        head_npz_save()          # 다음부터 0.7초에 뜬다
        _HEAD_CACHE.clear()      # dict 는 메모리만 먹으므로 비운다
        head_npz_load()
        return len(_HEAD_ARR['code']) if _HEAD_ARR['code'] is not None else n
    return 0

def head_cache_load():
    """디스크에 저장해 둔 닭 머리 표를 불러온다.

    격자는 표 파일에 적힌 값을 그대로 쓴다. JOY_GRID 와 비교하면 안 된다 —
    시작 직후엔 조종표(15초 로딩)가 아직이라 JOY_GRID 가 기본값(0.02)이고,
    그 순간 비교하면 멀쩡한 표를 거부한다 (실제 발생: 표가 안 실려서
    joy_chunk 가 실시간 IK 로 빠져 몇 분씩 멈췄다)."""
    try:
        with open(_head_cache_file()) as f: d=json.load(f)
        if d.get('urdf')!=os.path.basename(BASE_URDF or ''): return 0
        _HEAD_GRID[0]=float(d.get('grid',0.01)) or 0.01
        _HEAD_CACHE.clear()
        for k,v in d.get('tbl',{}).items():
            _HEAD_CACHE[tuple(int(x) for x in k.split(','))]=v
        return len(_HEAD_CACHE)
    except FileNotFoundError:
        return 0
    except Exception as e:
        print('닭 머리 표 읽기 실패:',e); return 0

def head_cache_save():
    tmp=f'{_head_cache_file()}.{os.getpid()}.tmp'
    try:
        with open(tmp,'w') as f:
            json.dump({'urdf':os.path.basename(BASE_URDF or ''),
                       'grid':_HEAD_GRID[0],
                       'tbl':{f'{k[0]},{k[1]},{k[2]}':v
                              for k,v in _HEAD_CACHE.items() if v}},f)
        os.replace(tmp,_head_cache_file())
        return len(_HEAD_CACHE)
    except Exception as e:
        print('닭 머리 표 저장 실패:',e)
        try: os.remove(tmp)
        except Exception: pass
        return 0

_HEAD_BUILD=[False]
_HEAD_FILL=[False]
# 표가 바뀐 횟수. 브라우저가 이 번호를 보고 조각을 다시 받는다.
HEAD_GEN=[0]

def _head_fill_bg(k,r):
    """조종 중 부족한 칸을 뒤에서 채운다 (조종은 안 멈춘다).
    가까운 칸부터 채워야 바로 쓸 수 있으므로 중심에서 바깥으로 넓혀간다."""
    if _HEAD_FILL[0]: return
    _HEAD_FILL[0]=True
    def _w():
        try:
            g=_HEAD_GRID[0]; seed=list(CURRENT_Q); n=0
            for rad in range(0,int(r)+1):          # 안쪽 껍질부터
                for dz in range(-rad,rad+1):
                    for dy in range(-rad,rad+1):
                        for dx in range(-rad,rad+1):
                            if max(abs(dx),abs(dy),abs(dz))!=rad: continue
                            kk=(k[0]+dx,k[1]+dy,k[2]+dz)
                            if kk in _HEAD_CACHE: continue
                            pt=(kk[0]*g,kk[1]*g,kk[2]*g)
                            if pt[2]<PLATE_TOP+0.02:
                                _HEAD_CACHE[kk]=None; continue
                            q=ik_head(pt,[seed])
                            if q is None:
                                _HEAD_CACHE[kk]=None; continue
                            seed=q; n+=1
                            _HEAD_CACHE[kk]=[round(float(x),4) for x in q]
            if n:
                # 이진 표에 새로 푼 칸을 합쳐 저장 (JSON 은 안 쓴다)
                if _HEAD_ARR['code'] is not None:
                    for i,c in enumerate(_HEAD_ARR['code']):
                        pass                    # 배열은 그대로 두고
                head_npz_merge(); HEAD_GEN[0]+=1
                print(f'  닭 머리 표 +{n:,}칸 (총 {len(_HEAD_CACHE):,})')
        except Exception as e:
            print('표 채우기 오류:',e)
        finally:
            _HEAD_FILL[0]=False
    threading.Thread(target=_w,daemon=True).start()


def h_head_reload():
    """디스크의 닭 머리 표를 다시 읽는다 (앱을 켠 채 표를 새로 만들었을 때).
    브라우저도 조각을 다시 받아가야 하므로 세대 번호를 올린다."""
    n=head_load_any()
    HEAD_GEN[0]+=1
    return {'n':n,'gen':HEAD_GEN[0]}, (f'닭 머리 표 {n:,}칸 다시 읽음'
                                       if n else '저장된 표가 없습니다')

def h_head_build(r=15):
    """닭 머리 표를 미리 만들어 저장한다 (뒤에서 돌린다).
    한 번 만들어두면 조종이 왕복 없이 즉시 반응한다."""
    if _HEAD_BUILD[0]: return {'busy':True},'이미 만드는 중입니다'
    def _w():
        _HEAD_BUILD[0]=True
        try:
            t0=time.time()
            g=_HEAD_GRID[0]
            p0=tool_pos(CURRENT_Q)
            k0=(int(round(float(p0[0])/g)),int(round(float(p0[1])/g)),
                int(round(float(p0[2])/g)))
            R=max(1,min(25,int(r)))
            seed=list(CURRENT_Q); n=0
            for dz in range(-R,R+1):
                for dy in range(-R,R+1):
                    for dx in range(-R,R+1):
                        kk=(k0[0]+dx,k0[1]+dy,k0[2]+dz)
                        if kk in _HEAD_CACHE: continue
                        pt=(kk[0]*g,kk[1]*g,kk[2]*g)
                        if pt[2]<PLATE_TOP+0.02:
                            _HEAD_CACHE[kk]=None; continue
                        q=ik_head(pt,[seed])
                        if q is None: _HEAD_CACHE[kk]=None; continue
                        seed=q; n+=1
                        _HEAD_CACHE[kk]=[round(float(x),4) for x in q]
                print(f'  닭 머리 표 {n:,}칸 ({time.time()-t0:.0f}초)',end='\r')
            head_cache_save()
            print(f'\n닭 머리 표 완료: {n:,}칸 ({time.time()-t0:.0f}초)')
        except Exception as e:
            print('닭 머리 표 오류:',e)
        finally:
            _HEAD_BUILD[0]=False
    threading.Thread(target=_w,daemon=True).start()
    return {'busy':True},'닭 머리 표 만드는 중… (로그에 진행 표시)'


def h_joy_chunk(cx=None,cy=None,cz=None,r=8):
    """조종표에서 '지금 있는 자리 주변' 만 잘라 브라우저에 준다.

    왜: 조이콘을 움직일 때마다 서버에 IK 를 물으면 왕복(40ms)이 곧 화면
    속도가 되어 끊긴다. 주변 표를 미리 받아두면 브라우저가 60Hz 로 즉시
    자세를 찾아 그릴 수 있다 (공 굴리기가 부드러웠던 것과 같은 구조).

    반경 30칸(±300mm) 이면 약 3만칸 = 1MB 라 한 번에 보낼 만하다.
    """
    # ★ 저장된 조종표는 못 쓴다 ★
    #   그 표는 'axis_end 를 바닥과 평행하게'(변형 물컵) 기준으로 만들었는데,
    #   지금 규칙은 '닭 머리'(l7 방향을 base 그대로) 라 서로 다르다.
    #   실측: 표 3000칸 전부가 손목 기준을 못 넘겼다.
    #   -> 여기서 닭 머리 IK 로 주변 칸을 직접 풀어 준다 (한 번만 계산해 두면
    #      브라우저가 그 뒤로는 왕복 없이 즉시 쓴다).
    g=_HEAD_GRID[0]
    if cx is None:
        p0=tool_pos(CURRENT_Q)
        k=(int(round(float(p0[0])/g)),int(round(float(p0[1])/g)),
           int(round(float(p0[2])/g)))
    else:
        k=(int(cx),int(cy),int(cz))
    r=max(1,min(25,int(r)))
    # ★ 여기서 IK 를 풀면 안 된다 ★
    #   빈 칸이 수천 개면 33ms x 수천 = 몇 분씩 멈춘다 (표 밖으로 나가는
    #   순간 조종이 얼어붙던 원인). 있는 것만 즉시 주고, 없는 칸은
    #   백그라운드로 채운다.
    out=[]; miss=0
    for dz in range(-r,r+1):
        for dy in range(-r,r+1):
            for dx in range(-r,r+1):
                kk=(k[0]+dx,k[1]+dy,k[2]+dz)
                hit=_HEAD_CACHE.get(kk)
                if hit is None and _HEAD_ARR['code'] is not None:
                    hit=head_arr_get(*kk)        # 이진 표에서 찾기
                if hit is not None:
                    if hit: out.append([kk[0],kk[1],kk[2]]+list(hit))
                    continue
                miss+=1          # 아직 없는 칸 -> 여기서 풀지 않는다
    if miss>0: _head_fill_bg(k,r)        # 부족한 만큼 뒤에서 채운다
    return ({'ready':True,'center':list(k),'r':r,'grid':JOY_GRID,
             'njoint':NJOINT,'cells':out,'n':len(out)},
            f'조종표 주변 {len(out):,}칸 전달')


def h_joy_apply(q,px=None,py=None,pz=None):
    """브라우저가 '주변 표'로 직접 찾은 자세를 그대로 반영한다.

    브라우저는 60Hz 로 즉시 그리고, 여기는 3D 상태 동기화만
    한다. 왕복을 기다리지 않으므로 화면이 끊기지 않는다.
    안전: 관절 수·한계를 확인하고, 손목(닭 머리)이 깨진 자세는 거른다.
    """
    q=[float(v) for v in (q or [])][:NJOINT]
    if len(q)<NJOINT: return {'ok':False},'자세 길이가 맞지 않습니다'
    for j in range(NJOINT):                     # 관절 한계 확인
        lo,hi=jr_limits(j)
        if not (lo-1e-6<=q[j]<=hi+1e-6):
            return {'ok':False},f'{JOINTS[j]} 가동범위를 벗어납니다'
    if NJOINT>=7 and HEAD_LOCK[0] and head_err_deg(q)>4.0:
        return {'ok':False},'손목 고정이 깨진 자세입니다'
    ros_publish(q)
    if px is not None:
        JOY['tgt']=[float(px),float(py),float(pz)]
    else:
        p=tool_pos(q); JOY['tgt']=[float(p[0]),float(p[1]),float(p[2])]
    p=JOY['tgt']
    return ({'ok':True,'pos':[round(v,4) for v in p]},
            f"조이콘: ({p[0]*1000:.0f},{p[1]*1000:.0f},{p[2]*1000:.0f})mm")


def h_joy_move(dx,dy,dz,grip=None):
    """조이콘이 보낸 이동량(m)만큼 툴 목표점을 옮기고 IK 로 자세를 만든다.

    좌표계는 **월드(대좌표계)** 다 — 3D 화면에 보이는 X/Y/Z 그대로.
    로봇을 회전(BASE_YAW)시켜도 조작 방향은 변하지 않는다
    (로봇 기준으로 하면 로봇을 돌릴 때마다 조작이 뒤집혀 헷갈린다).
    tool_pos / ik_tool 이 이미 월드 좌표로 동작하므로 그대로 더하면 된다.

    도달 불가한 축은 그 축만 되돌린다 (벽에 부딪힌 느낌 — 나머지는 계속 이동)."""
    if not JOY['on'] or JOY['tgt'] is None:
        return {'ok':False,'why':'조종이 시작되지 않았습니다'},'조종 정지 상태'
    q0=list(CURRENT_Q)
    cur=list(JOY['tgt'])
    # 목표점이 실제 팔 위치보다 너무 앞서면(막혀서 못 따라간 것) 되돌린다.
    # 안 그러면 이미 도달 불가한 지점을 계속 요청해 넓은 탐색도 실패한다.
    # (실측: 41mm 벌어져 있었다). 단, 매번 덮으면 조종이 끊기므로
    # '많이 벌어졌을 때만' 맞춘다.
    _real=tool_pos(q0)
    if max(abs(float(_real[i])-cur[i]) for i in range(3))>0.02:
        cur=[float(_real[0]),float(_real[1]),float(_real[2])]
        JOY['tgt']=list(cur)
    # 먼 자세로 갈아타는 중이면 계속 다가간다 (여러 프레임에 나눠 옮김).
    # 다 옮겨질 때까지는 목표점을 바꾸지 않는다 — 자세 전환과 이동을 동시에
    # 하면 팔이 어디로 갈지 알 수 없어진다.
    _w=JOY.get('warp')
    if _w is not None:
        if max(abs(a-b) for a,b in zip(_w[:NJOINT],q0[:NJOINT])) \
           <= math.radians(JOY_WARP_DEG)*1.2:
            JOY['warp']=None                   # 다 옮겼다
            q0=list(_w)
            ros_publish(_w)
        else:
            nq=_joy_warp_step(q0,_w)
            nq[GRIP_IDX]=q0[GRIP_IDX]
            ros_publish(nq)
            p2=tool_pos(nq)
            return ({'ok':True,'pos':[round(float(v),4) for v in p2],
                     'blocked':[],'warp':True},
                    '자세 바꾸는 중… (갈 수 있는 팔 모양으로 옮기는 중)')
    # 한 번에 너무 크게 움직이지 않게 제한 (통신이 밀렸다 몰아서 오면
    # 로봇이 순간이동하듯 튄다 — 조종 감각을 해치고 위험하다)
    STEP_MAX=0.03                      # 한 번에 최대 30mm
    dx=max(-STEP_MAX,min(STEP_MAX,float(dx)))
    dy=max(-STEP_MAX,min(STEP_MAX,float(dy)))
    dz=max(-STEP_MAX,min(STEP_MAX,float(dz)))
    want=[cur[0]+dx,cur[1]+dy,cur[2]+dz]
    # 바닥 아래로는 절대 안 감 (안전)
    want[2]=max(PLATE_TOP+0.02,want[2])
    # ① 미리 만든 조종표에 있으면 계산 없이 즉시 (가장 빠름)
    # 단, 변형 물컵 규칙을 쓰는 로봇에서는 '끝단이 평평한' 칸만 쓴다.
    # 표에는 기울어진 칸도 들어 있어서(그 자리의 최선이 45도인 곳도 있다)
    # 그냥 쓰면 규칙이 깨진 채로 움직인다.
    # 8도는 '허용 한계' 라 매 걸음 조금씩 기울면 금세 누적된다(0.6->1.1도...).
    # 조종 중에는 훨씬 엄격하게 봐서 axis_end 가 처음 모양을 유지하게 한다.
    LEVEL_TOL=4.0
    def _level_ok(qq):
        return (not LEVEL_END) or end_tilt_deg(qq)<=LEVEL_TOL
    def _level_ok_far(qq):
        """넓은 탐색(마지막 수단) 전용 — 규칙을 조금 느슨하게 본다.
        팔을 멀리 뻗으면 자세 여유가 없어 기울기가 커진다. 여기서까지
        4도를 고집하면 '갈 수 있는데 못 가는' 자리가 생긴다
        (실측: 해가 4.94도라 버려져 -868mm 에서 막혔다).
        규칙 한계(8도) 안이면 허용한다."""
        return (not LEVEL_END) or end_tilt_deg(qq)<=8.0
    q=None
    # ★ 닭 머리 규칙 ★ 8축 로봇은 방향을 base 그대로 못박고 위치만 푼다.
    #   그러면 SO-ARM 처럼 '해가 하나로 좁혀져' 손목이 꺾이지 않는다.
    #   조종표는 방향을 안 맞춘 칸이라 이 모드에서는 쓰지 않는다.
    if NJOINT>=7 and HEAD_LOCK[0]:
        # ① 먼저 '닭 머리 표' 를 본다.
        #    ik_head(ikpy)는 body 모드를 잘 못 푼다 — 표에 정답이 있는 칸을
        #    정답으로 씨앗을 줘도 20개 중 4개만 푼다(실측). 그래서 갈 수 있는
        #    자리인데도 "막힘" 이 떴다. 표는 GPU 로 푼 것이라 훨씬 정확하다.
        q=_head_tbl_q(want,q0)
        if q is None:
            q=ik_head(want,_seed_list(q0))
        if q is None:
            # 이 자리는 '방향을 유지한 채' 로는 못 간다 -> 축별로 살펴본다
            blocked=[]
            want=list(cur)                       # cur 은 건드리지 않는다
            for i,d in enumerate((dx,dy,dz)):
                if abs(d)<1e-9: continue
                t=list(want); t[i]+=d
                if i==2: t[2]=max(PLATE_TOP+0.02,t[2])
                qq=ik_head(t,_seed_list(q0))
                if qq is not None: want=t; q=qq
                else: blocked.append('xyz'[i])
            if q is None:
                return ({'ok':False,'blocked':blocked or ['x','y','z'],
                         'pos':[round(v,4) for v in cur]},
                        '그 방향은 손목을 고정한 채로는 갈 수 없습니다')
        # ★ UR15 의 GRIP_IDX(7)는 'end' = 손목 방향을 만드는 축이다 ★
        #   여기를 옛 값으로 덮어쓰면 IK 가 0.02도로 맞춰 준 방향이 깨진다
        #   (실측: ik_head 는 0.02도인데 적용 후 13.58도로 벌어졌다).
        #   그리퍼가 따로 없는 로봇이므로 IK 결과를 그대로 쓴다.
        if LEVEL_ALWAYS:                       # 6축(SO-ARM) 만 그리퍼 유지
            q[GRIP_IDX]=q0[GRIP_IDX]
        if grip is not None and LEVEL_ALWAYS:
            JOY['grip']=bool(grip); q[GRIP_IDX]=_j6_rad(bool(grip))
            ros_publish(q,with_grip=True)
        else:
            ros_publish(q)
        JOY['tgt']=want; JOY['warp']=None
        return ({'ok':True,'pos':[round(v,4) for v in want],
                 'blocked':locals().get('blocked') or [],
                 'head':round(head_err_deg(q),2)},
                f"조이콘: ({want[0]*1000:.0f},{want[1]*1000:.0f},"
                f"{want[2]*1000:.0f})mm · 손목 고정 "
                f"{head_err_deg(q):.1f}도")
    # 조종표는 '그 칸의 최선' 을 담고 있을 뿐, 지금 자세와 이어진다는 보장이
    # 없다. 그대로 쓰면 팔 모양이 통째로 바뀐다(=랜덤하게 움직이는 느낌).
    # -> 지금 자세에서 너무 멀면 쓰지 않고 IK 로 이어서 푼다.
    # ★ 옛 조종표(joy_table.json)는 world 모드로 만들어졌다 ★
    #   그 자세들은 j5(손목)를 90도씩 쓰므로, 지금의 body 모드 관문
    #   (head_err_deg > 4도면 거부)에 전부 걸린다 — 실측 최대 175도.
    #   닭 머리 규칙을 쓰는 로봇에서는 아예 보지 않는다.
    hit=None if (NJOINT>=7 and HEAD_LOCK[0]) else JOY_TABLE.get(_joy_key(want))
    if hit is not None and len(hit)==NJOINT and _level_ok(list(hit)):
        if max(abs(a-b) for a,b in zip(list(hit)[:NJOINT],list(q0)[:NJOINT])) \
           <= math.radians(JOY_MAX_JOINT_DEG):
            q=list(hit)
    # ② 빠른 IK. 한 번에 안 되면 '조금씩 나눠서' 다시 (여전히 빠르다).
    #    정식 IK(ik_tool)는 씨앗을 여러 개 돌려 1초 가까이 걸리므로
    #    조종(20Hz)에는 못 쓴다. 마지막 수단으로만 남긴다.
    if q is None:
        q=joy_ik_fast(want,q0)
        if q is not None and not _level_ok(q): q=None
    if q is None:
        # 목표까지 2~3 등분해 이어서 풀면 각 걸음이 작아 해가 튀지 않는다
        qs=list(q0); ok=True
        for k in (0.5,1.0):
            mid=[cur[i]+(want[i]-cur[i])*k for i in range(3)]
            qq=joy_ik_fast(mid,qs)
            if qq is None or not _level_ok(qq): ok=False; break
            qs=qq
        if ok: q=qs
    # ③ 여기까지 실패 = '지금 팔 모양 근처' 로는 못 간다는 뜻이다.
    #    하지만 팔꿈치를 반대로 접는 등 **다른 자세로는 갈 수 있는 경우가
    #    많다**. 가까운 해만 찾다가 "막힘"이라고 하면, 실제로는 갈 수 있는
    #    자리를 못 가게 된다 (사용자 지적: "더 뒤로 갈 수 있는데 안 간다").
    #    -> 이때만 넓게 탐색한다. 찾은 해가 멀면 한 번에 점프하지 않고
    #       JOY['warp'] 에 넣어 여러 프레임에 걸쳐 서서히 옮긴다.
    warp=None
    if q is None and NJOINT>=7:
        far=_joy_wide_ik(want,q0,_level_ok_far)
        if far is not None:
            d=max(abs(a-b) for a,b in zip(far[:NJOINT],list(q0)[:NJOINT]))
            if d<=math.radians(JOY_MAX_JOINT_DEG):
                q=far                      # 가까우면 바로 써도 안 튄다
            else:
                warp=far                   # 멀면 나눠서 옮긴다
                q=_joy_warp_step(q0,far)
    blocked=[]
    if q is None:
        # 축별로 하나씩 시도 — 갈 수 있는 방향만 살린다
        want=list(cur)
        for i,d in enumerate((dx,dy,dz)):
            if abs(d)<1e-9: continue
            t=list(want); t[i]+=d
            if i==2: t[2]=max(PLATE_TOP+0.02,t[2])
            qq=ik_tool(t,_seed_list(q0),q0[4],q0[GRIP_IDX])
            if qq is not None and not _level_ok(qq): qq=None
            if qq is None and NJOINT>=7:
                # 이 축도 '지금 팔 모양' 으로는 막혔다 -> 다른 팔 모양으로
                # 갈 수 있는지 넓게 본다 (한 축만 움직일 때도 필요하다)
                far1=_joy_wide_ik(t,q0,_level_ok_far)
                if far1 is not None:
                    dd=max(abs(a-b) for a,b in zip(far1[:NJOINT],list(q0)[:NJOINT]))
                    if dd<=math.radians(JOY_MAX_JOINT_DEG):
                        qq=far1
                    else:
                        warp=far1; qq=_joy_warp_step(q0,far1)
            if qq is not None:
                want=t; q=qq
            else:
                blocked.append('xyz'[i])
    if q is None:
        # 어느 축이 진짜 막혔는지 하나씩 확인한다.
        # (예전엔 무조건 x,y,z 전부 막힘이라고 보고해서 원인을 알 수 없었다)
        why=[]
        for i,d in enumerate((dx,dy,dz)):
            if abs(d)<1e-9: continue
            t=list(cur); t[i]+=d
            if i==2: t[2]=max(PLATE_TOP+0.02,t[2])
            qq=ik_tool(t,_seed_list(q0),q0[4],q0[GRIP_IDX])
            ok_here=qq is not None and _level_ok(qq)
            if not ok_here:
                # 그 축의 '반대 방향' 은 갈 수 있는지도 본다
                t2=list(cur); t2[i]-=d
                if i==2: t2[2]=max(PLATE_TOP+0.02,t2[2])
                q2=ik_tool(t2,_seed_list(q0),q0[4],q0[GRIP_IDX])
                back=(q2 is not None and _level_ok(q2))
                why.append('xyz'[i]+('(반대는 가능)' if back else ''))
        return ({'ok':False,'blocked':why or ['x','y','z'],
                 'pos':[round(v,4) for v in cur]},
                ('막힘: '+', '.join(why)+' — 반대 방향으로는 갈 수 있습니다'
                 if any('반대는 가능' in w for w in why)
                 else '팔 끝(도달 한계)입니다 — 반대로 움직여 돌아오세요'))
    if LEVEL_ALWAYS: q[J5_IDX]=q0[J5_IDX]      # 물컵 규칙 (j5 고정)
    if grip is not None:
        JOY['grip']=bool(grip)
        q[GRIP_IDX]=_j6_rad(bool(grip))
        ros_publish(q,with_grip=True)
    else:
        q[GRIP_IDX]=q0[GRIP_IDX]
        ros_publish(q)
    JOY['tgt']=want
    # 먼 자세로 갈아타는 중이면 다음 프레임에도 이어서 옮긴다
    JOY['warp']=warp
    tilt=end_tilt_deg(q) if LEVEL_END else 0.0
    return ({'ok':True,'pos':[round(v,4) for v in want],'blocked':blocked,
             'tilt':round(tilt,1)},
            f"조이콘: ({want[0]*1000:.0f},{want[1]*1000:.0f},"
            f"{want[2]*1000:.0f})mm"
            +(f" · axis_end 기울기 {tilt:.1f}도" if LEVEL_END else ""))

def h_seq_clear():
    """파이프라인 블록 전부 삭제 (Ctrl+Z 로 되돌릴 수 있음)."""
    n=len(S['seq'])
    if not n: return gr.update(),"파이프라인이 이미 비어 있습니다"
    BTN('블록 일괄 삭제',f'{n}개')
    _undo_snapshot('블록 일괄 삭제')
    with LOCK: S['seq']=[]
    _macro_save(); _clear_plan_cache()
    return (gr.update(choices=seq_labels(),value=None),
            f"블록 {n}개 전부 삭제 (Ctrl+Z 로 되돌리기)")

def _arm_of_block(idx):
    """파이프라인 idx 번 블록을 하는 팔 (그 앞 마지막 팔 블록, 없으면 A)."""
    L='A'
    for x in S['seq'][:int(idx)+1]:
        if is_arm_item(x): L=x[len(ARM_PREFIX):]
    return L

def _switch_to_block_arm(idx):
    """11-05: 블록 우클릭(자세 등록·이 자세로)은 '그 블록의 팔' 기준 — 지금 팔이 다르면 바꾼다. 바꿨으면 그 팔 이름."""
    L=_arm_of_block(idx)
    if L!=ACTIVE_ARM[0] and (L=='A' or L in arms_cfg()):
        arm_switch(L)
        return L
    return None

def h_block_to_pose(label):
    """파이프라인 블록(➜이동 / ◈잡기·놓기 / ▼하강 / ▲상승)이 가리키는 자리를
    '자세'로 등록한다. 등록되면 기존 미세조정 기능(+/- , 보정 저장)을 그
    자세에 그대로 쓸 수 있다 — 즉석 IK 블록은 보정할 방법이 없었던 문제 해결.

    등록 자세의 관절값 = 그 블록을 실행할 때 IK 가 만드는 값 (지금 기준)."""
    BTN('블록을 자세로',str(label))
    m2=re.match(r'^\s*[■◆◈➜▼▲]\s*(\d+)\.',str(label or ''))
    idx=int(m2.group(1)) if m2 else None
    if idx is None or idx>=len(S['seq']):
        return gr.update(),"블록을 먼저 선택하세요 (이동/잡기/놓기/하강/상승)"
    item=S['seq'][idx]
    _sw=_switch_to_block_arm(idx)              # 11-05: B팔 블록은 B팔로 (예전엔 지금 팔(A)로 풀어 '도달 불가' — 반응 없음처럼 보였다)
    # ★ 잡기·놓기는 자세로 등록하지 않는다 ★
    #   좌표를 가진 블록은 [이동] 하나뿐이다. 잡기/놓기를 자세로 등록하면
    #   그 자세가 좌표를 갖게 되어 이동과 두 개가 된다(팔이 두 자리 사이를
    #   오감 — 실측 j1 4.5도). 실수로도 안 되게 서버에서 막는다.
    if is_grasp_item(item) or is_release_item(item):
        return gr.update(),("잡기·놓기는 제자리 동작이라 자세로 등록하지 "
                            "않습니다 — 미세조정은 바로 앞 [이동] 블록에서 "
                            "하세요 (좌표는 이동 하나만 가집니다)")
    # 그 블록이 도달해야 하는 관절값 구하기
    q=None; tag=None
    if is_goto_item(item):
        oid=item[len(GOTO_PREFIX):]; q=goto_q(oid); tag=f"{disp_name(oid)}_이동"
    elif is_grasp_item(item):
        b=item[len(GRASP_PREFIX):]
        p=brick_grasp_point(b)
        if p is not None:
            q0=list(CURRENT_Q); q=ik_tool(p,_seed_list(q0),q0[4],q0[GRIP_IDX])
        tag=f"{disp_name(b)}_잡기"
    elif is_release_item(item):
        d=item[len(RELEASE_PREFIX):]
        t=dock_socket_world(d)
        if t is not None:
            z=float(t[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT
            q0=list(CURRENT_Q)
            q=ik_tool((float(t[0]),float(t[1]),z),_seed_list(q0),q0[4],q0[GRIP_IDX])
        tag=f"{disp_name(d)}_놓기"
    elif is_down_item(item) or is_up_item(item):
        # 그 자리 동작이라 '지금 로봇이 서 있는 자세'가 곧 그 블록의 자세다.
        # 우클릭 [이 자세로 이동] 으로 그 블록까지 재생한 뒤 등록하면 정확하다.
        q=list(CURRENT_Q)
        _tp=tool_pos(q)
        tag=('하강' if is_down_item(item) else '상승')+'자리'
        # 발밑에 뭐가 있으면 이름에 넣어 구분되게
        for _ob,_b in BRICKS.items():
            _bp=_b.get('pos')
            if _bp and math.hypot(_bp[0]-_tp[0],_bp[1]-_tp[1])<0.03:
                tag=f"{disp_name(_ob)}_{tag}"; break
        else:
            for _dk in docking_ids():
                _t=dock_socket_world(_dk)
                if _t and math.hypot(_t[0]-_tp[0],_t[1]-_tp[1])<0.03:
                    tag=f"{disp_name(_dk)}_{tag}"; break
    else:
        return gr.update(),f"'{item}' 은 자세로 등록할 수 없는 블록입니다"
    if q is None:
        return gr.update(),(f"그 블록의 자세를 만들 수 없습니다 ({arm_label(ACTIVE_ARM[0])} 자리에서 도달 불가) — "
                            "[로봇 자리 추천]을 눌러보세요")
    # URDF 자세 등록(_add_urdf_pose)과 완전히 같은 절차로 넣는다
    angles=[float(v) for v in q]
    clamped=[]
    for j,a in enumerate(angles[:5]):
        lo,hi=jr_limits(j)
        c=max(lo,min(hi,a))
        if abs(c-a)>0.002:
            clamped.append(f"{jname(j)} {math.degrees(a):+.1f}->{math.degrees(c):+.1f}도")
        angles[j]=c
    # 이름에 위치를 넣어 목록에서 어떤 자세인지 바로 알 수 있게 한다
    _tp0=tool_pos(angles)
    _slot=is_goto_item(item)
    if _slot:
        # ★ 이동 블록의 보정 = '로봇 기준 자리' 하나 (2026-11-04, 사용자) ★
        #   이름은 로봇 기준 좌표(mm)만. 로봇을 옮겨도 로봇 기준 같은 점이면 같은 이름·같은 보정이라
        #   새로 만들지 않고 그 보정을 연다 → 보정 자세 수는 팔이 닿는 점의 수를 넘지 않는다.
        _rel=w2l(np.asarray(_tp0,float))
        _old=_slot_find(ACTIVE_ARM[0],_rel)
        if _old is not None:
            FT['pose']=_old['name']
            return (gr.update(choices=pose_names(),value=_old['name']),
                    f"'{_old['name']}' — 로봇 기준 같은 자리의 보정이 이미 있어 그것을 엽니다 "
                    "(로봇을 옮겨도 같은 자리입니다)\n  ③연동 탭에서 [로봇을 이 자세로] -> +/- 로 "
                    "미세조정 -> [보정 저장]")
        name=_slot_name(ACTIVE_ARM[0],_rel)
    else:
        tag=f"{tag}({_tp0[0]*1000:.0f},{_tp0[1]*1000:.0f},{_tp0[2]*1000:.0f})"
        name=tag; k=2
        while name in pose_names(): name=f'{tag}_{k}'; k+=1
    off,src,dist=nearest_offset(angles)        # 가까운 보정 자동 적용
    if ACTIVE_ARM[0]!='A': off,src,dist=None,None,None   # 11-06: base 보정은 A팔 값 — B팔 자세에 복사하지 않는다
    rec={'name':name,'angles':angles,
         'offset':off if off else [0.0]*NJOINT,
         'offsets':{'sts':off if off else [0.0]*NJOINT}}
    if _slot:
        rec.update(rel=[round(float(v),5) for v in _rel],arm=ACTIVE_ARM[0])
    with LOCK:
        S['poses'].append(rec)
    _macro_save()
    FT['pose']=name                            # 미세조정 대상으로 바로 선택
    tp=tool_pos(angles)
    msg=(f"'{name}' 자세로 등록됨 — 툴 ({tp[0]*1000:.0f},{tp[1]*1000:.0f},"
         f"{tp[2]*1000:.0f})mm\n  ③연동 탭에서 [로봇을 이 자세로] -> +/- 로 "
         "미세조정 -> [보정 저장]")
    if is_grasp_item(item) or is_release_item(item):
        # 잡기/놓기는 '상승 전' 자리로 등록된다 (미세조정하려면 그 자리에
        # 머물러야 하므로). 그리퍼가 닫힌 채 가면 물체에 부딪히니 안내한다.
        msg+=("\n  * 상승 전 자리입니다 (잡는 높이). 이동 전에 그리퍼를 "
              "열어두세요 — 닫힌 채로 가면 물체에 부딪힙니다")
    if clamped: msg+="\n  관절 한계 조정: "+", ".join(clamped)
    if _sw: msg+=f"\n  지금 팔을 {arm_label(_sw)} 로 바꿨습니다 (이 블록의 팔 — 미세조정·보정 저장도 이 팔)"
    if off: msg+=f"\n  base 보정 자동 적용 (관절각 차이 {dist:.1f}도)"
    return gr.update(choices=pose_names(),value=name),msg

def calibrated_poses():
    """보정값이 실제로 들어간 자세 목록."""
    out=[]
    for p in S['poses']:
        off=p.get('offset',[0.0]*NJOINT)
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
    off=ref.get('offset',[0.0]*NJOINT)
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
    set_offset(name,[0.0]*NJOINT)
    try: _cal_send(get_angles(name),direct=True)
    except Exception: pass
    return f"'{name}' {ROBOT_LABEL.get(offset_robot())} 보정 초기화 (실물 = 정답 자세로)"
def h_ft_save(pose_name):
    """미세조정을 확정한다.

    ★ 구형(그라디오) 방식 ★
      보정값을 따로 들고 있다가 보낼 때마다 더하면, '어느 보정을 쓸지' 를
      매번 다시 고르게 되어 같은 자리인데 값이 둘이 된다
      (실측: j1 raw 859 <-> 808, 4.5도 차이로 팔이 좌우로 튐).
      구형에는 offset 개념이 아예 없었다 — 슬라이더로 맞춘 각도를 그대로
      저장하고, 그 각도를 그대로 보냈다. 값이 하나뿐이라 어긋날 수가 없다.
      -> 저장할 때 보정을 각도에 녹여 넣고 보정은 0 으로 되돌린다."""
    name=FT['pose'] or pose_name
    if not name: return "자세를 먼저 선택하세요"
    _ensure_pose_arm(name)                     # 11-05: 그 자세의 팔 보정으로 저장
    rec=_pose_rec(name)
    off=get_offset(name)
    merged=False
    _before=list((rec or {}).get('angles') or [])[:4]     # 11-06: 저장 전후 기록
    if rec is not None: rec['tuned']=True           # 보정 지도: [보정 저장] 해야 초록 (11-04)
    if rec is not None and any(abs(v)>1e-9 for v in off):
        try:
            a=list(rec.get('angles') or [])
            a=(a+[0.0]*NJOINT)[:NJOINT]
            o=_off_for_send(name,off)              # 11-06: 자리 보정 자세는 기울기까지 그대로 (예전: 합 0 으로 지움)
            # 보정을 각도에 더하고, 관절 한계 안으로 자른다
            for j in range(min(NJOINT,len(a))):
                lo,hi=jr_limits(j)
                a[j]=max(lo,min(hi,a[j]+float(o[j] if j<len(o) else 0.0)))
            rec['angles']=a
            set_offset(name,[0.0]*NJOINT)      # 녹였으니 보정은 비운다
            merged=True
        except Exception as e:
            print('보정 합치기 실패:',e)
    # ★ 같은 자리 보정은 하나만 (덮어쓰기, 2026-10-20) ★
    #   보정의 주인은 XY 자리라, 같은 자리를 다시 보정하면 예전 것은 지운다.
    #   등록할 때가 아니라 '저장할 때' 지운다 — 등록만 하고 저장 안 하면 예전 보정이 남는다.
    #   11-04: '같은 자리' 는 로봇 기준 자리다 (_pose_slot — 같은 팔의 보정끼리만, 지금 로봇 자리로 바꿔 비교)
    replaced=[]
    me=_pose_slot(rec) if rec is not None else None
    if me is not None:
        px,py=me[0],me[1]
        dup=[]
        for p in list(S['poses']):
            if p['name']==name: continue
            s2=_pose_slot(p)
            if s2 is not None and math.hypot(s2[0]-px,s2[1]-py)<=SLOT_TOL: dup.append(p)
        with LOCK:
            for p in dup:
                if p in S['poses']: S['poses'].remove(p); replaced.append(p['name'])
    with open(PROJECT_FILE,'w') as f: json.dump(S,f,indent=1)
    try:
        _after=list((rec or {}).get('angles') or [])[:4]
        LOG(f"[보정 저장] {name} (팔 {(rec or {}).get('arm') or 'A'}): "
            +' '.join(f"j{j+1} {math.degrees(a):+.2f}→{math.degrees(b):+.2f}°" for j,(a,b) in enumerate(zip(_before,_after)))
            +(f" · 같은 자리 예전 보정 지움: {', '.join(replaced)}" if replaced else ""))
    except Exception: pass
    rep=(f"\n  같은 자리의 예전 보정을 덮어썼습니다: {', '.join(replaced)}"
         if replaced else "")
    if replaced:
        print(f"[보정 저장] 같은 자리 예전 보정 덮어씀: {', '.join(replaced)} -> {name}")
    if merged:
        return (f"'{name}' 저장됨 — 미세조정을 자세 각도에 반영했습니다 "
                f"(보정값은 0 으로, 이제 값이 하나뿐입니다)"+rep)
    return f"'{name}' 보정 저장됨: "+_off_str(get_offset(name))+"  (그 로봇에만 적용)"+rep

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
        tgt[GRIP_IDX]=q0[GRIP_IDX]                         # 그리퍼는 현재 상태 유지
        # j5 는 그 자세의 값까지 부드럽게 (물컵 규칙 유지, ramp_j5 설명 참고)
        way,dt=plan_joint_seq(q0,tgt)
        for f in way: f[GRIP_IDX]=q0[GRIP_IDX]
        if LEVEL_ALWAYS: ramp_j5(way,q0,tgt)
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
    PAUSE.clear()                # 일시 정지 중이어도 바로 정지 (11-06)
    ESTOP['hold']=False          # 화면 [■ 정지] 는 비상정지 버튼으로 멈춘 뒤에도 토크를 끈다
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

def h_zero_diag():
    """진단(읽기 전용): 지금 실물 모터 raw 와 영점/한계/환산각을 나란히 본다.
    로봇을 움직이지 않는다 — base 저장이 왜 어긋나는지 볼 때 쓴다."""
    if not ROBOT_STS.connect(): return f"SO-ARM 연결 실패 ({PORT})"
    rows=[]
    for k,i in enumerate(J2ID[:6]):
        if i in STS_DEAD_IDS:
            rows.append(f"  고정 관절(URDF {k+1}번째): 모터 없음 — 0도 고정 (통신 안 함)"); continue
        got=[]
        for _ in range(9):
            v=ROBOT_STS._r2(i,ADDR_PRESENT)
            if v is not None: got.append(int(v))
            if len(got)>=5: break
            time.sleep(0.02)
        if not got:
            rows.append(f"  j{k+1}(ID{i}): 읽기 실패"); continue
        got.sort(); raw=got[len(got)//2]
        z=ZERO_RAW.get(i); lim=LIMIT_RAW.get(i)
        deg=DIR[i]*(raw-z)*180.0/2048.0 if z is not None else float('nan')
        inlim='' if (lim and lim[0]<=raw<=lim[1]) else '  ❌한계밖'
        rows.append(f"  j{k+1}(ID{i}): 지금 raw={raw:5d}  영점={z}  "
                    f"→ {deg:+7.2f}도   한계 {lim}{inlim}")
    off=get_offset('base')
    return ("[진단] 실물 현재값 vs 저장된 영점 (읽기 전용)\n"
            +"\n".join(rows)
            +f"\n  base 각도: {[round(math.degrees(a),2) for a in get_angles('base')[:5]]}"
            +f"\n  base 보정: {[round(math.degrees(a),2) for a in off[:5]]}"
            +"\n  * '지금 raw' 와 '영점' 이 같으면 = 지금 자세가 곧 base")

def h_cal_capture_zero(rlabel):
    """지금 손으로 잡아둔 자세를 그 로봇의 영점(base)으로 저장."""
    if ACTIVE_ARM[0]!=REAL_ARM:       # 두 팔 (10-30): 실물 A팔 모터를 읽어 저장하므로, B팔을 잡고 누르면 A 영점이 망가진다
        return (f"영점 저장 막음 — 3D 지금 팔이 {arm_label(ACTIVE_ARM[0])} 입니다. 영점은 실물 팔"
                f"({arm_label(REAL_ARM)})만 저장합니다. '3D 에서 옮길 팔' 을 {REAL_ARM}팔로 바꾸세요")
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
        ids=sorted(set(J2ID)-STS_DEAD_IDS); rd=lambda i: ROBOT_STS._r2(i,ADDR_PRESENT)
        path=os.path.join(here,'sts_zero.py'); title='SO-ARM'
    vals={}
    if rk!='dxl':                     # 선 없는 모터(j5 고정)는 읽지 않고 예전 영점을 그대로 둔다
        vals.update({i:ZERO_RAW[i] for i in STS_DEAD_IDS if i in ZERO_RAW})
    for i in ids:
        got=[]
        for _ in range(30):
            v=rd(i)
            if v is not None: got.append(int(v))
            if len(got)>=5: break
            time.sleep(0.04)
        if not got: return f"ID{i} 읽기 실패 - 배선/전원 확인"
        got.sort(); vals[i]=got[len(got)//2]        # 중앙값 (노이즈 제거)
    # !! 안전 검증: 읽은 값이 그 축의 물리적 한계 밖이면 영점으로 쓰면 안 된다.
    #    (STS 가 다회전/통신오류로 엉뚱한 raw 를 주는 경우가 있다. 실제로
    #     j4 가 3971 로 저장돼 base 0도가 가동범위 밖이 되는 사고가 났다.)
    if rk!='dxl':
        bad=[]; tight=[]
        grip_id=GRIP_ID
        for i,v in vals.items():
            if i in STS_DEAD_IDS: continue
            # 그리퍼(j6)는 '자세'가 아니라 여닫기다. 완전히 닫으면 한계 끝을
            # 살짝 벗어난 값이 읽히는데, 그것 때문에 영점 저장이 막히면 안 된다.
            if i==grip_id: continue
            lim=LIMIT_RAW.get(i)
            if not lim: continue
            if not (lim[0]<=v<=lim[1]):
                bad.append(f"ID{i}={v} (한계 {lim[0]}~{lim[1]})")
            else:
                # 양쪽 여유(도) — 한쪽이 10도 미만이면 IK 가 자주 실패한다
                room=min(abs(v-lim[0]),abs(lim[1]-v))*180.0/2048.0
                if room<10.0:
                    tight.append(f"ID{i} 여유 {room:.0f}도")
        if bad:
            _m=("영점 저장 취소 — 읽은 값이 관절 한계를 벗어났습니다:\n  "
                +"\n  ".join(bad))
            LOG(_m)                      # 로그에도 남겨 원인이 보이게
            return (_m
                    +"\n  * 모터가 한 바퀴 넘게 돌아가 있거나 통신 오류입니다."
                    "\n  * 팔을 가동범위 안으로 되돌린 뒤 다시 저장하세요."
                    "\n  (기존 영점은 그대로 두었습니다)")
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
        _keep={k:v for k,v in ZERO_RAW.items() if _ARM2 is not None and k in _ARM2.ZERO_RAW}   # B팔 영점(sts_arm2.py)은 지우지 않는다 (11-02)
        ZERO_RAW.clear(); ZERO_RAW.update(vals); ZERO_RAW.update(_keep)
        globals()['CHAIN']=build_chain(); globals()['_CHAIN_L']=None
        _AN['done']=False                            # 해석 파라미터 재산출
        for k in range(nj):
            if _dead_joint(k): continue
            lo,hi=jr_limits(k)
            rng.append(f"{jname(k)} {math.degrees(lo):+.0f}~{math.degrees(hi):+.0f}")
    # !! 영점을 새로 잡으면 '지금 이 자세 = 0도' 가 된다.
    #    이때 예전 보정값을 그대로 두면 base 로 보낼 때 그만큼 엉뚱한 곳으로
    #    간다 (손으로 맞춘 자세와 다른 곳 -> 실제 부상 사고 발생).
    #    그러므로 그 로봇의 보정값을 전부 0 으로 초기화한다.
    _cleared=0
    for _p in S['poses']:
        _d=_p.setdefault('offsets',{'sts':list(_p.get('offset',[0.0]*NJOINT))})
        if any(abs(v)>1e-9 for v in _d.get(rk,[0.0]*NJOINT)):
            _cleared+=1
        _d[rk]=[0.0]*NJOINT
        if rk=='sts': _p['offset']=[0.0]*NJOINT
    # base 자세 자체도 '전 축 0' 이어야 한다. 새 영점 = 지금 이 자세이므로,
    # base 에 옛 각도(예: j6 -1.14도)가 남아 있으면 손으로 잡은 자세와 달라진다.
    _bp=_pose_rec('base')
    if _bp is not None and any(abs(v)>1e-9 for v in _bp['angles']):
        LOG(f"  [영점] base 각도 초기화: "
            +' '.join(f"j{i+1}:{math.degrees(v):+.2f}"
                      for i,v in enumerate(_bp['angles']) if abs(v)>1e-9))
        _bp['angles']=[0.0]*NJOINT
    _macro_save()
    # 영점이 바뀌면 '관절각 -> 실제 위치' 관계 자체가 달라진다.
    # 그 위에서 계산해 둔 표/캐시는 전부 무효 — 안 지우면 옛 영점 기준으로
    # 움직이거나 도달 판정이 틀린다.
    _GOTO_Q.clear(); _clear_plan_cache()
    _MAXZ_R.clear(); _MAXZ_KEY[0]=None          # 도달상한 (디스크 캐시 포함)
    globals()['_ARM_REACH']=None                # 팔 도달범위
    if 'PEG_TABLE' in globals():                # 목심 도달표 (재계산 필요)
        PEG_TABLE['key']=None; PEG_TABLE['span']={}
    if '_PEG_CACHE' in globals(): _PEG_CACHE.clear()
    if 'PEG_VIEW' in globals(): PEG_VIEW['dirty']=True
    if '_OBS' in globals(): _OBS['grid']=None
    try:                                        # 옛 영점으로 만든 디스크 캐시 폐기
        if os.path.exists(_MAXZ_FILE): os.remove(_MAXZ_FILE)
    except Exception: pass
    ros_publish([0.0]*NJOINT)                    # 3D 도 새 base(전 축 0)로
    LOG(f"{title} base 저장됨: "+'  '.join(f"ID{i}:{v}" for i,v in vals.items()))
    return (f"{title} base 저장됨 ({os.path.basename(path)})\n  "
            +'  '.join(f"ID{i}:{v}" for i,v in vals.items())
            +"\n  가동범위: "+'  '.join(rng)
            +"\n  * 범위가 좌우 대칭에 가까우면 기계적 중립에 잘 앉은 것입니다"
            +(f"\n  * 새 영점 기준이므로 예전 보정값 {_cleared}개를 0 으로 "
              "초기화했습니다 (지금 이 자세가 곧 base)" if _cleared else "")
            +(f"\n  ⚠ 가동범위 끝에 가깝습니다: {', '.join(tight)}"
              "\n     그 축을 범위 가운데로 옮기고 다시 잡으면 IK 실패가 줄어듭니다"
              if (rk!='dxl' and tight) else ""))

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
    """자동 저장 (블록 편집 등에서 자주 불린다).

    ★ 여기가 위험하다 ★ 메모리(S)를 그대로 파일에 덮어쓴다. 그래서 어떤
    이유로든 S 에 항목이 비어 있으면 파일에서도 통째로 사라진다.
    (2026-08-24: 로봇 교체가 공장 항목을 안 읽어서 설비·이름이 날아갔다)
    -> 파일에는 있는데 메모리에 없는 항목은 지우지 않고 살려둔다.
    """
    _calib_backup()
    data=dict(S)
    try:
        with open(PROJECT_FILE) as f: old=json.load(f)
        for k,v in old.items():
            if k not in data or (not data.get(k) and v):
                data[k]=v            # 메모리에 없는 건 파일 값을 유지
    except Exception:
        pass
    tmp=f'{PROJECT_FILE}.{os.getpid()}.tmp'
    with open(tmp,'w') as f: json.dump(data,f,indent=1)
    os.replace(tmp,PROJECT_FILE)

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
        tgt[GRIP_IDX]=q0[GRIP_IDX]                           # 그리퍼는 이동 중 상태 불변
        # j5 는 기록된 자세 값까지 부드럽게 (물컵 규칙 유지)
        way,dt=plan_joint_seq(q0,tgt)
        for f in way: f[GRIP_IDX]=q0[GRIP_IDX]
        if LEVEL_ALWAYS: ramp_j5(way,q0,tgt)
        ros_publish(tgt)
        for q in way:
            if STOP.is_set(): break
            ROBOT.send(q,log=True,tag=f'M{slot}'); time.sleep(dt)
        return f"M{slot} ({m['name']}) 도착"
    except Exception as e:
        ROBOT.estop(); return f"오류: {e} - 토크 해제됨"
    finally:
        RUNNING.clear()

def _calib_backup():
    """저장 전에 지금 파일을 백업해 둔다 (하루 한 번).
    설비/이름/자세가 통째로 날아간 적이 있어(2026-08-24) 안전장치를 둔다."""
    try:
        if not os.path.exists(PROJECT_FILE): return
        import shutil, datetime
        day=datetime.date.today().strftime('%Y%m%d')
        bak=f'{PROJECT_FILE}.{day}.bak'
        if not os.path.exists(bak):
            shutil.copy2(PROJECT_FILE,bak)
    except Exception:
        pass

def h_save():
    BTN('저장')
    _calib_backup()
    # 원자적 저장: 임시 파일에 다 쓴 뒤 바꿔치기 (쓰다 멈춰도 안 깨진다)
    tmp=f'{PROJECT_FILE}.{os.getpid()}.tmp'
    with open(tmp,'w') as f: json.dump(S,f,indent=1)
    os.replace(tmp,PROJECT_FILE)
    return f"saved -> {PROJECT_FILE}"

def h_load():
    BTN('불러오기')
    global S
    try:
        with open(PROJECT_FILE) as f: data=json.load(f)
        with LOCK:
            S['poses']=data['poses']; S['seq']=data['seq']
            if not any(p['name']=='base' for p in S['poses']):
                S['poses'].insert(0,{'name':'base','angles':[0.0]*NJOINT})
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
    if _d.get('robot_yaw') is not None: S['robot_yaw']=_d['robot_yaw']
    if _d.get('env_off'): S['env_off']=_d['env_off']
    S['names']=_d.get('names',{})
    if _d.get('brick_pos'): S['brick_pos']=_d['brick_pos']   # 물체 배치 복원용
    if _d.get('env_clones'): S['env_clones']=_d['env_clones']   # 설비 복제본
    if _d.get('env_removed'): S['env_removed']=_d['env_removed'] # 삭제한 설비
    if _d.get('env_vis'): S['env_vis']=_d['env_vis']            # 표시 여부
    if _d.get('factory_presets'): S['factory_presets']=_d['factory_presets']
    if _d.get('plates_added'): S['plates_added']=_d['plates_added']  # 추가 플레이트
    if _d.get('calmap_ok'): S['calmap_ok']=_d['calmap_ok']            # ✅ 보정 성공 자리 (11-04)
    if _d.get('arm_pose'): S['arm_pose']=_d['arm_pose']               # 11-05: B팔 자리 (빠져 있어 B팔이 사라졌다)
    if _d.get('arms'): S['arms']=_d['arms']                           # 11-05: 팔 이름·색
    if _d.get('level_trim'): S['level_trim']=_d['level_trim']         # 11-05: 팔별 그리퍼 수평 보정 (j4 영점, 실물만)
    if not any(p['name']=='base' for p in S['poses']):
        S['poses'].insert(0,{'name':'base','angles':[0.0]*NJOINT,'offset':[0.0]*NJOINT})
    print(f"  자세 {len(S['poses'])}개, 녹화 {len(S['moves'])}건 불러옴")
except FileNotFoundError:
    print("  (새 URDF - 보정 데이터 없음, base 부터 시작)")

# ===== RViz 스타일 네이티브 웹 뷰어 (three.js, VNC보다 빠름) =====
# 메쉬는 브라우저가 1번만 로드, 이후 관절값만 폴링 -> 60fps 부드러움
import http.server, socketserver
VIEWER_PORT=7870
MESH_REG={}          # id -> (path, scale)
ENV_SHOW=[]          # [{'id','scale','color'}]
def _is_floor_plate(oid):
    """바닥 플레이트(고정 지형)인가? seed_plate 처럼 이름에 plate 가 들어가는
    '일반 설비'까지 걸리면 안 되므로 바닥 계열(plate_bottom/plate_tile)만 매칭."""
    s=str(oid).lower()
    return 'plate_bottom' in s or 'plate_tile' in s
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

# ★ 기본 바닥판 = 2×4 (사용자, 11-03) ★ 앱 처음 화면 기준 가로(x) 2장 × 세로(y) 4장, 흰·파 체크무늬.
#   로봇 판(칸 0,0)이 왼쪽 위(안쪽) = 흰, 앞쪽(-y, 팔이 향하는 쪽)으로 늘린다.  흰 파 / 파 흰 / 흰 파 / 파 흰
#   실행할 때마다 이 배치로 맞춘다 (부팅 끝 '기본 바닥판' 참고). 카메라도 판이 이 모양이라고 안다 (cam_grid/grid.py BI·BJ).
PLATE_LAYOUT=[(ix,iy) for iy in (0,-1,-2,-3) for ix in (0,1)]
PLATE_WHITE='#f2f2f2'; PLATE_BLUE='#2f6fd6'
def plate_color(t):
    """칸 (ix,iy) 의 판 색 — (0,0) 이 흰, 이웃은 번갈아."""
    return PLATE_WHITE if (int(t[0])+int(t[1]))%2==0 else PLATE_BLUE

# 기존 플레이트들의 타일 좌표(정수 격자) 파악
PLATES={}                 # (ix,iy) -> {'id','origin'(m), 'src'}
def _register_existing_plates():
    for e in list(ENV_SHOW):
        if 'plate_bottom' not in e['id'].lower(): continue
        b=_plate_bounds_m(MESH_REG[e['id']][0])
        ox,oy=b[0][0],b[0][1]                    # 좌하단(원점)
        ix,iy=tile_of(ox,oy)
        PLATES[(ix,iy)]={'id':e['id'],'origin':(ox,oy),'added':False}
        e['tile']=(ix,iy); e['color']=plate_color((ix,iy))
_register_existing_plates()

PLATE_EDIT={'on':False,'sel':None}    # 편집 모드 / 선택 타일

# ===== 로봇 위치 이동 — 2핀 배치 (회전 포함) =====
# 로봇 바닥에는 목심에 꽂는 핀 2개가 있다(로봇 목심). 배치 순서:
#   ① 로봇 핀 하나 클릭  ② 그 핀이 꽂힐 바닥 목심 클릭 (위치 고정, 회전만 남음)
#   ③ 남은 핀이 갈 수 있는 목심(=핀 간격 거리, 플레이트 위)만 색으로 표시
#   ④ 그중 하나 클릭 -> 위치+회전 확정
ROBOT_MOVE={'on':False,'phase':0,'pin':None,'g1':None,'cands':[]}

_RPINS=None
def robot_pins_local():
    """base_link 바닥의 목심 핀 2개 (로컬 좌표, m). 메쉬에서 자동 검출.
    지름 ~10mm 원기둥 핀. 실패 시 실측 폴백값."""
    global _RPINS
    if _RPINS is not None: return _RPINS
    # ① 사용자가 pick_robot_pins.py 로 고른 고정값이 있으면 무조건 그걸 쓴다
    #    (자동 감지 없음 = 실행할 때마다 결과가 바뀔 여지 자체가 없음)
    try:
        from robot_pins_fixed import ROBOT_PINS_MM as _PM
        _RPINS=[np.array([_PM[0][0]/1000.0,_PM[0][1]/1000.0,0.0]),
                np.array([_PM[1][0]/1000.0,_PM[1][1]/1000.0,0.0])]
        print(f"로봇 핀(사용자 지정 고정값): {_PM} mm  <- robot_pins_fixed.py")
        return _RPINS
    except ImportError:
        pass
    # ② 고정값이 없으면 기존 자동 감지 (폴백)
    try:
        import trimesh as _tm
        from scipy.cluster.hierarchy import fclusterdata as _fc
        m=_tm.load(os.path.join(MESH_DIR,'base_link.STL'))
        v=np.asarray(m.vertices)*1000.0
        n=m.face_normals; f=np.asarray(m.faces)
        zmin=v[:,2].min()
        w=v[np.unique(f[np.abs(n[:,2])<0.25])]
        w=w[np.abs(w[:,2]-zmin)<4]
        lab=_fc(w[:,:2],t=3.0,criterion='distance')
        cs=[]
        for k in set(lab):
            g=w[lab==k]
            if len(g)<6: continue
            c=g[:,:2].mean(0); r=np.linalg.norm(g[:,:2]-c,axis=1)
            if 4.0<np.median(r)<6.0:          # 지름 8~12mm = 목심 핀
                cs.append((float(c[0]),float(c[1])))
        # 20mm 격자 위에 있고 서로 20mm 떨어진 쌍을 찾는다
        grid=[c for c in cs if abs((c[0]+90)%20)<2.5 or abs((c[0]+90)%20)>17.5]
        best=None
        for i in range(len(grid)):
            for j in range(i+1,len(grid)):
                d=math.hypot(grid[i][0]-grid[j][0],grid[i][1]-grid[j][1])
                if abs(d-20.0)<2.0: best=(grid[i],grid[j]); break
            if best: break
        if best:
            _RPINS=[np.array([best[0][0]/1000,best[0][1]/1000,0.0]),
                    np.array([best[1][0]/1000,best[1][1]/1000,0.0])]
            print(f"로봇 핀 검출: {np.round(np.array(_RPINS)[:,:2]*1000,1).tolist()} mm")
            return _RPINS
    except Exception as e:
        print(f"로봇 핀 검출 실패({e}) - 실측 폴백")
    _RPINS=[np.array([0.210,-0.290,0.0]),np.array([0.210,-0.270,0.0])]
    return _RPINS

def robot_pins_world():
    return [l2w(p) for p in robot_pins_local()]

def _pegs_all_world():
    """바닥 전체 목심 (x,y) 목록."""
    out=[]
    for nm in peg_names():
        xy=peg_xy(nm)
        if xy: out.append((float(xy[0]),float(xy[1])))
    return out

def _on_plate(x,y,margin=0.0):
    for t in PLATES:
        ox,oy=tile_origin(t)
        if ox-margin<=x<=ox+PLATE_SIZE+margin and oy-margin<=y<=oy+PLATE_SIZE+margin:
            return True
    return False

def set_robot_pose(x,y,yaw,save=True):
    """로봇 base 를 월드 (x,y) + 회전 yaw 로. 캐시 전부 무효화."""
    BASE_OFF[0]=float(x); BASE_OFF[1]=float(y); BASE_YAW[0]=float(yaw)
    _MAXZ_R.clear(); _MAXZ_KEY[0]=None; _clear_plan_cache()
    _OBS['grid']=None
    # 도달표(PEG_TABLE)는 상대좌표라 로봇 이동/회전에도 그대로 유효.
    if '_PEG_CACHE' in globals(): _PEG_CACHE.clear()
    if 'PEG_VIEW' in globals(): PEG_VIEW['dirty']=True
    if save:
        with LOCK:
            if 'ACTIVE_ARM' in globals() and ACTIVE_ARM[0]!='A':     # B팔을 옮기는 중 (10-30)
                S.setdefault('arm_pose',{})[ACTIVE_ARM[0]]=[float(x),float(y),float(yaw)]
            else:
                S['robot_xy']=[float(x),float(y)]; S['robot_yaw']=float(yaw)
        _macro_save()

def _arm_block_real():
    """3D '지금 팔' 이 실물로 연결된 팔(REAL_ARM)이 아니면 실물 목표 쓰기를 막는다 (10-30).
    B팔을 3D 에서 고르고 실물 버튼을 누르면, B팔 자리·자세로 계산한 값이 A팔 모터로 가기 때문."""
    if ACTIVE_ARM[0] not in REAL_ARMS:
        if not _ARM_BLOCK_LOGGED[0]:
            LOG(f"  ⚠ 실물 전송 막음 — 3D 지금 팔이 {arm_label(ACTIVE_ARM[0])} 입니다 "
                f"(실물은 {', '.join(sorted(REAL_ARMS))}팔). 블록 팔레트에서 '3D 에서 옮길 팔' 을 바꾸세요")
            _ARM_BLOCK_LOGGED[0]=True
        return True
    _ARM_BLOCK_LOGGED[0]=False
    return False
_ARM_BLOCK_LOGGED=[False]

# ===== 두 팔 실물 (2026-11-02) — 팔마다 모터 번호·그리퍼 값이 다르다 (A 1~5, B 11~15) =====
_ARM_A={'j2id':None}            # A팔 관절→모터 번호 (팔을 바꾸기 직전 값). 그리퍼 번호·값은 _ARM_A_GRIP
def _arm_motors(L):
    """실물 모터 번호·그리퍼 값·IK 체인을 L 팔 것으로. SO-ARM(sts3215) 이고 sts_arm2.py 가 있을 때만.
    영점·범위·방향은 모터 번호로 나뉜 표에 두 팔 것이 다 들어 있어서, 번호만 바꾸면 따라 바뀐다."""
    global J2ID,GRIP_ID,GRIPPER_CLOSE,GRIPPER_OPEN,CHAIN,_CHAIN_L
    if _ARM2 is None or os.path.basename(os.path.dirname(URDF_PKG_DIR))!='sts3215': return
    if _ARM_A['j2id'] is None: _ARM_A['j2id']=list(J2ID)
    a=_ARM_A['j2id']; g,c,o=_ARM_A_GRIP
    if L=='B':
        k=_ARM2.ID_OFFSET
        J2ID=[i if i in STS_DEAD_IDS else i+k for i in a]; GRIP_ID=g+k
        GRIPPER_CLOSE,GRIPPER_OPEN=_ARM2.GRIPPER_CLOSE,_ARM2.GRIPPER_OPEN
    else:                                    # A (그 밖의 팔은 3D 만이라 A 번호 그대로 — 실물 전송은 _arm_block_real 이 막는다)
        J2ID=list(a); GRIP_ID,GRIPPER_CLOSE,GRIPPER_OPEN=g,c,o
    CHAIN=build_chain(); _CHAIN_L=None       # 관절 범위가 팔마다 달라서 IK 체인을 다시 만든다
    globals()['_ARM_REACH']=None; _AN['done']=False
    LOG(f"  [팔] 모터 번호: {', '.join(jname(k)+'='+str(i) for k,i in enumerate(J2ID) if i not in STS_DEAD_IDS)}")

# ===== 그리퍼 수평 맞추기 (11-05 사용자: 화면은 수평인데 실물 B팔이 기운다) =====
#   원인 = 그 팔의 영점(손으로 잡고 잰 기본 자세)이 정확히 수평이 아니었다 → 어느 자세에서나 같은 각도로 기운다.
#   미세조정은 기울기를 j4 로 되돌리게(level_offset) 돼 있어 이걸 못 고친다.
#   -> 팔마다 'j4 영점' 만 몇 도 옮긴다 (S['level_trim'] = {팔: 도}). 실물 각도↔모터값 변환이 모두 ZERO_RAW 를
#      쓰므로 보내기·읽기가 같이 맞는다. 3D(이론)는 그대로.
_J4_ZERO0={}                     # 모터 번호 -> 수평 보정 전 원래 영점
def _j4_motor(L):
    """그 팔의 j4 모터 번호 (SO-ARM 두 팔만). 없으면 None."""
    if _ARM2 is None or os.path.basename(os.path.dirname(URDF_PKG_DIR))!='sts3215': return None
    a=_ARM_A['j2id'] if _ARM_A['j2id'] is not None else (list(J2ID) if ACTIVE_ARM[0]=='A' else None)
    if a is None or len(a)<4: return None
    return a[3]+(_ARM2.ID_OFFSET if L=='B' else 0)
def _apply_level_trim():
    for L,deg in (S.get('level_trim') or {}).items():
        mid=_j4_motor(L)
        if mid is None or mid not in ZERO_RAW: continue
        z0=_J4_ZERO0.setdefault(mid,ZERO_RAW[mid])
        ZERO_RAW[mid]=z0+DIR[mid]*math.radians(float(deg))*2048.0/math.pi
def _level_trim_msg(L,extra=''):
    v=float((S.get('level_trim') or {}).get(L,0.0))
    return f"{arm_label(L)} 그리퍼 수평 보정 {v:+.1f}° (실물 j4 영점만 · 3D 는 그대로){extra}"
def h_level_trim(cmd='status'):
    """미세조정 칸의 [−0.5°]/[+0.5°]/[0으로]: 지금 팔의 j4 영점을 옮기고, 실물이 연결돼 있으면 지금 자세를
    새 영점으로 다시 보내 그리퍼가 바로 바뀌게 한다 (j4 만 그만큼 움직인다)."""
    L=ACTIVE_ARM[0]
    if _j4_motor(L) is None: return f"{arm_label(L)}: 그리퍼 수평 보정은 SO-ARM 두 팔에서만 됩니다"
    cmd=str(cmd if cmd is not None else 'status').strip()
    if cmd in ('','status'): return _level_trim_msg(L)
    if cmd!='reset':                         # 11-06 (사용자): 보정 자세의 모터값을 바꾸므로 막는다 (0 으로 되돌리기만 허용)
        return (f"{arm_label(L)}: 그리퍼 수평 보정은 쓰지 않습니다 — 보정 자세 전부의 j4 모터값이 같이 바뀌기 때문"
                " (보정은 [보정 저장] 으로만 바뀝니다)")
    d=S.setdefault('level_trim',{})
    cur=float(d.get(L,0.0))
    new=0.0 if cmd=='reset' else cur+float(cmd)
    new=max(-10.0,min(10.0,round(new,2)))
    if RUNNING.is_set(): return "실행 중에는 바꿀 수 없습니다"
    q_now=None                               # 옛 영점으로 읽은 지금 관절값
    if not ROBOT._sim():
        try:
            if ROBOT.connect(): q_now=ROBOT.read()
        except Exception: q_now=None
    with LOCK: d[L]=new
    _apply_level_trim()
    _macro_save()
    extra=''
    if q_now is not None:
        try:
            ROBOT.time_profile(True); ROBOT.torque(True)
            ROBOT.send(list(q_now))           # 같은 관절값 · 새 영점 -> j4 만 그만큼
            extra=' · 실물 j4 를 바로 움직였습니다'
        except Exception as e:
            extra=f' · 실물 전송 실패: {e}'
    else:
        extra=' · 실물이 연결되지 않아 값만 저장 (다음 실물 동작부터)'
    LOG(f"  [수평 보정] {arm_label(L)} {cur:+.1f}° -> {new:+.1f}°")
    return _level_trim_msg(L,extra)

def _real_arm_to_base(sp):
    """실물: 지금 팔을 기본 자세(base)로 (실행 시작 때와 같은 방법). 두 팔 실행에서 팔을 바꾼 직후에 쓴다 (11-02)."""
    q0=ROBOT.read(); bq=get_angles('base')
    if q0 is None or bq is None: return
    bqc=[a+o for a,o in zip(bq,level_offset(get_offset('base')))]   # B팔 보정은 따로 (없으면 0)
    bqc[GRIP_IDX]=q0[GRIP_IDX]
    if LEVEL_ALWAYS: bqc[J5_IDX]=q0[J5_IDX]
    LOG(f"  [시작] {arm_label(ACTIVE_ARM[0])} 기본 자세(base)로")
    w,dt=plan_joint_all(list(q0),bqc,steps=30)
    for q in w: q[GRIP_IDX]=q0[GRIP_IDX]
    if LEVEL_ALWAYS: lock_j5(w,q0)
    ros_publish(bq)                          # 3D 는 이론 자세
    for q in w:
        if STOP.is_set(): raise KeyboardInterrupt
        ROBOT.send(q); time.sleep(dt/sp)
    _settle_or_rest(bqc)

# ===== 두 팔 디지털 트윈 (2026-10-30) — B팔은 A팔 복제 (같은 URDF·같은 도달표) =====
#   '지금 팔'(ACTIVE_ARM) 의 자리·자세가 전역(BASE_OFF·CURRENT_Q)에 들어 있고, 나머지 팔은 ARM_STATE 에 쉰다.
#   팔을 바꾸면 전역을 갈아끼우므로 이동·잡기·놓기·도달 계산이 그대로 그 팔 기준이 된다.
#   A팔 자리 = S['robot_xy'/'robot_yaw'] (예전 그대로), 다른 팔 자리 = S['arm_pose'][L] = [x,y,yaw].
ACTIVE_ARM=['A']
ARM_STATE={}                     # 쉬는 팔: L -> {'q':관절, 'grip':그리퍼}
def arm_pose_of(L):
    """그 팔의 자리 (x,y,yaw)."""
    if L==ACTIVE_ARM[0]: return float(BASE_OFF[0]),float(BASE_OFF[1]),float(BASE_YAW[0])
    if L=='A':
        st=ARM_STATE.get('A',{})
        if st.get('pose'): return tuple(st['pose'])
        xy=S.get('robot_xy') or [0.0,0.0]
        return float(xy[0]),float(xy[1]),float(S.get('robot_yaw',0.0))
    p=S.setdefault('arm_pose',{}).get(L)
    if not p:                    # 처음 쓰는 팔: A팔 옆 30cm 에 둔다 (3D 에서 옮기면 된다)
        ax,ay,ayaw=arm_pose_of('A'); p=[ax,ay+0.30,ayaw]; S['arm_pose'][L]=p
    return float(p[0]),float(p[1]),float(p[2])
def arms_shown():
    """3D 에 그릴 팔: 설정된 팔 전부 (11-05 사용자: 기본이 두 팔 — 예전엔 자리를 정한 팔만 그렸다).
    자리를 아직 안 정한 팔은 arm_pose_of 가 A팔 옆 30cm 에 둔다."""
    out=['A']
    for L in arms_cfg():
        if L!='A': out.append(L)
    return out
def arm_switch(L):
    """지금 팔을 L 로 (3D·계산 기준). 같은 팔이면 아무것도 안 한다."""
    cur=ACTIVE_ARM[0]
    if L==cur: return
    if L not in arms_cfg() and L!='A': raise ValueError(f'설정에 없는 팔: {L}')
    ARM_STATE[cur]={'q':list(CURRENT_Q),'grip':GRIP_STATE['rad'],
                    'pose':(float(BASE_OFF[0]),float(BASE_OFF[1]),float(BASE_YAW[0]))}
    if cur=='A': _ARM_A['j2id']=list(J2ID)        # A팔 번호 (로봇을 바꾸면 달라질 수 있어 떠날 때마다 적어 둔다)
    x,y,yaw=arm_pose_of(L)
    ACTIVE_ARM[0]=L
    _arm_motors(L)                               # 실물 모터 번호·그리퍼 값도 그 팔 것으로 (11-02)
    set_robot_pose(x,y,yaw,save=False)
    st=ARM_STATE.get(L) or {'q':list(get_angles('base') or [0.0]*NJOINT),'grip':GRIP_STATE['rad']}
    GRIP_STATE['rad']=st['grip']; ros_publish(st['q'],with_grip=True)
    LOG(f"  [팔] 지금 팔: {arm_label(L)}")
def _frames_at(q,off,yaw):
    c,si=math.cos(yaw),math.sin(yaw); R=np.array([[c,-si,0],[si,c,0],[0,0,1]])
    fr=[]
    for f in CHAIN.forward_kinematics([0]+list(q),full_kinematics=True):
        g=f.copy(); g[:3,:3]=R@g[:3,:3]; g[:3,3]=R@g[:3,3]+np.asarray(off,float); fr.append(g)
    return fr
_ARM_TINT={'빨강':'#d9534f','주황':'#f0a030','노랑':'#e8d44d','초록':'#5cb85c','파랑':'#3d7fd9','보라':'#9b59b6'}
ARM_GRAY='#9097a1'               # 두 팔일 때 팔 몸 색 (그리퍼 턱만 팔 색) — 11-03
def arm_links():
    """3D 링크 목록 — 팔마다. A팔은 예전 이름 그대로(l1…), 다른 팔은 'B~l1'."""
    out=[]
    for L in arms_shown():
        if L==ACTIVE_ARM[0]:
            q=list(CURRENT_Q); off=BASE_OFF; yaw=BASE_YAW[0]
        else:
            st=ARM_STATE.get(L) or {}
            q=st.get('q') or list(get_angles('base') or [0.0]*NJOINT)
            x,y,yaw=arm_pose_of(L); off=np.array([x,y,0.0])
        fr=_frames_at(q,off,yaw)
        c,si=math.cos(yaw),math.sin(yaw)
        B=np.eye(4); B[:3,:3]=np.array([[c,-si,0],[si,c,0],[0,0,1]]); B[:3,3]=off
        for k,ln in enumerate(ROBOT_LINKS):
            T=B if k==0 else fr[k]
            # 두 팔 (11-03, 사용자): 몸은 전부 회색, 그리퍼의 움직이는 턱(마지막 링크)만 그 팔 색
            col=(_ARM_TINT.get(arms_cfg().get(L)) or ROBOT_COLOR_MAP[ln]) if k==len(ROBOT_LINKS)-1 else ARM_GRAY
            out.append({'id':ln if L=='A' else f'{L}~{ln}','color':col,
                        'm':[float(x) for x in T[:3,:].flatten()]+[0,0,0,1]})
    return out

def rmove_reset():
    ROBOT_MOVE.update(phase=0,pin=None,g1=None,cands=[])

def rmove_click_pin(idx):
    """① 로봇 핀 선택"""
    ROBOT_MOVE.update(phase=1,pin=int(idx),g1=None,cands=[])
    return True

def rmove_click_peg(x,y):
    """②/④ 바닥 목심 클릭"""
    if RUNNING.is_set(): return False
    pins=robot_pins_local(); a=ROBOT_MOVE.get('pin') or 0
    if ROBOT_MOVE['phase']==1:
        # 첫 핀 위치 확정 -> 남은 핀 후보 = 20mm 거리의 4방향 중 플레이트 위
        g1=np.array([x,y]); ROBOT_MOVE['g1']=[float(x),float(y)]
        pa,pb=pins[a],pins[1-a]
        gap=float(np.linalg.norm((pb-pa)[:2]))
        cands=[]
        for dx,dy in ((gap,0),(-gap,0),(0,gap),(0,-gap)):
            gx,gy=x+dx,y+dy
            if not _on_plate(gx,gy): continue
            # 그 방향으로 놓았을 때 로봇 발이 플레이트를 벗어나지 않는지는
            # 확정 시점에 다시 확인. 여기선 목심 존재만 본다
            cands.append([round(gx,4),round(gy,4)])
        if not cands:
            print("[로봇 배치] 그 목심 옆엔 남은 핀이 갈 자리가 없습니다 — 다른 목심을 고르세요")
            return False                     # 1단계 유지
        ROBOT_MOVE['g1']=[float(x),float(y)]
        ROBOT_MOVE.update(phase=2,cands=cands)
        return True
    if ROBOT_MOVE['phase']==2:
        # 후보 중 하나인지 확인
        hit=None
        for gx,gy in ROBOT_MOVE['cands']:
            if math.hypot(gx-x,gy-y)<0.005: hit=(gx,gy); break
        if hit is None: return False
        g1=np.array(ROBOT_MOVE['g1']); g2=np.array(hit)
        pa,pb=pins[a][:2],pins[1-a][:2]
        yaw=math.atan2(g2[1]-g1[1],g2[0]-g1[0]) - math.atan2(pb[1]-pa[1],pb[0]-pa[0])
        yaw=round(yaw/(math.pi/2))*(math.pi/2)      # 90도 스냅
        c,si=math.cos(yaw),math.sin(yaw)
        off=g1-np.array([c*pa[0]-si*pa[1], si*pa[0]+c*pa[1]])
        set_robot_pose(off[0],off[1],yaw)
        rmove_reset()
        ROBOT_MOVE['on']=False               # 배치 완료 -> 이동 모드 자동 종료
        print(f"[로봇 배치] {arm_label(ACTIVE_ARM[0])} 핀{a+1}->({g1[0]*1000:.0f},{g1[1]*1000:.0f}) "
              f"yaw={math.degrees(yaw):.0f}도 -> {_robot_pos_str()} (이동 모드 자동 꺼짐)")
        _arm_move_back()                     # 우클릭으로 옮긴 팔이 B 면 A팔로 돌아온다 (11-03)
        return True
    return False


# ===== 설비 2핀 배치 (로봇 이동과 같은 방식) =====
# 드래그 드롭 -> 다리(핀) 클릭 -> 바닥 목심 클릭 -> 하늘색 후보 클릭 = 자리+회전 확정
_EPINS={}
def env_pins_local(oid):
    """설비 STL 바닥의 목심 다리(핀) 중심들 (로컬 좌표 m). 지름 ~10mm 검출."""
    if oid in _EPINS: return _EPINS[oid]
    pins=[]
    try:
        from scipy.cluster.hierarchy import fclusterdata as _fc
        path,scale=MESH_REG[oid]
        m=trimesh.load(path)
        v=np.asarray(m.vertices)*scale*1000.0     # mm
        n=m.face_normals; f=np.asarray(m.faces)
        zmin=v[:,2].min()
        w=v[np.unique(f[np.abs(n[:,2])<0.25])]
        w=w[np.abs(w[:,2]-zmin)<4]
        if len(w):
            lab=_fc(w[:,:2],t=3.0,criterion='distance')
            for k in set(lab):
                g=w[lab==k]
                if len(g)<6: continue
                c=g[:,:2].mean(0); r=np.linalg.norm(g[:,:2]-c,axis=1)
                if 4.0<np.median(r)<6.0:          # 지름 8~12mm
                    pins.append(np.array([c[0]/1000.0,c[1]/1000.0,0.0]))
        if pins:
            print(f"설비 핀 검출 {disp_name(oid)}: "
                  f"{[list(np.round(q[:2]*1000,1)) for q in pins]} mm")
    except Exception as e:
        print(f"설비 핀 검출 실패({oid}): {e}")
    _EPINS[oid]=pins
    return pins

def _rz2(yaw):
    c,si=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-si],[si,c]])

def _env_ent(oid):
    for e in ENV_SHOW:
        if e['id']==oid: return e
    return None

def env_pins_world(oid):
    e=_env_ent(oid) or {}
    R=_rz2(float(e.get('yaw',0.0)))
    off=np.asarray(e.get('offset',(0,0,0)),float)
    return [np.array([*(R@q[:2]+off[:2]),0.0]) for q in env_pins_local(oid)]

def set_env_pose(oid,ox,oy,yaw):
    """설비를 오프셋(ox,oy)+회전 yaw 로. 캐시 무효화 + 영속화.
    도킹이면 '결합'된 물체(꽂힌 펜)도 같이 따라간다."""
    e=_env_ent(oid)
    if e is None: return False
    riders=[]
    if 'docking' in str(oid).lower():
        _t_old=dock_socket_world(oid)
        if _t_old:
            for _ob,_b2 in BRICKS.items():
                _bp=_b2.get('pos')
                if _bp and math.hypot(_bp[0]-_t_old[0],_bp[1]-_t_old[1])<0.025:
                    riders.append(_ob)
    e['offset']=(float(ox),float(oy),0.0); e['yaw']=float(yaw)
    _OBS['grid']=None; _clear_plan_cache()
    # 도달표는 순수 '팔이 닿는가' 라 설비 이동과 무관 - 무효화하지 않는다.
    with LOCK:
        S.setdefault('env_off',{})[oid]=[float(ox),float(oy),float(yaw)]
    _macro_save()
    for _ob in riders:                     # 결합된 물체가 새 소켓으로 따라감
        try:
            place_brick_on_docking(_ob,oid)
            print(f"[결합 이동] {_ob} 가 {disp_name(oid)} 를 따라감")
        except Exception as _e:
            print(f"[결합 이동 실패] {_ob}: {_e}")
    return True

DRAG_ENV={'id':None,'fresh_id':None,'dropped':False}

def h_env_drag(oid='',move=False):
    """썸네일 드래그 시작/끝.

    move=False (견본 카드): 복제본을 미리 만들어 그것이 커서를 따라온다.
    move=True  (배치됨 카드): 복제 없이 '그 인스턴스'를 그대로 옮긴다.
    끝(drop 없이 취소): 미확정 복제본을 정리한다.
    """
    oid=str(oid or '')
    if oid and move:                              # 배치된 것 이동 모드
        DRAG_ENV.update(id=oid,fresh_id=None,dropped=False,move=True)
        return ''
    if not oid:                                   # dragend
        # 주의: 브라우저는 drop 과 dragend 를 거의 동시에 보내는데, dragend 가
        # 먼저 도착하면 '취소' 로 오해해 복제본을 지워버린다 (물체가 띡 사라지는
        # 버그). 그래서 1.2초 기다렸다가, 그때도 drop 이 없었을 때만 정리한다.
        pend=DRAG_ENV.get('fresh_id')
        if pend and not DRAG_ENV.get('dropped'):
            def _cancel_later(p=pend):
                time.sleep(1.2)
                if not DRAG_ENV.get('dropped') and DRAG_ENV.get('fresh_id')==p:
                    env_delete(p)
                    DRAG_ENV.update(id=None,fresh_id=None,dropped=False)
                    print(f"[드래그 취소] {p} 정리")
            threading.Thread(target=_cancel_later,daemon=True).start()
        else:
            DRAG_ENV.update(id=None,fresh_id=None,dropped=False)
        return ''
    base=_clone_base(oid)
    DRAG_ENV['dropped']=False; DRAG_ENV['move']=False
    _undo_snapshot('드래그 배치')                  # Ctrl+Z 로 통째로 되돌리게
    if base in BRICKS:                            # 물체(ppap 등)
        b=BRICKS.get(oid) or BRICKS[base]
        if b.get('pos') is not None:              # 배치돼 있으면 복제본이 따라옴
            n=2
            while f"{base}#{n}" in BRICKS: n+=1
            new=f"{base}#{n}"
            BRICKS[new]=dict(BRICKS[base]); MESH_REG[new]=MESH_REG[base]
            BRICKS[new]['pos']=tuple(b['pos'])    # 원본 위에 겹쳐 시작
            with LOCK:
                S.setdefault('names',{})[new]={'name':'',
                                               'color':_next_auto_color(base)}
            DRAG_ENV.update(id=new,fresh_id=new)
        else:                                     # 미배치 원본은 그대로
            DRAG_ENV.update(id=oid,fresh_id=None)
        return ''
    if oid in MESH_REG and not _is_floor_plate(oid):   # 설비
        new=env_clone(oid)
        if new:
            e=_env_ent(_clone_base(oid)); ne=_env_ent(new)
            if e and ne:                          # 원본 위에 겹쳐 시작
                ne['offset']=tuple(e.get('offset',(0,0,0)))
                ne['yaw']=float(e.get('yaw',0.0))
            DRAG_ENV.update(id=new,fresh_id=new)
            return ''
    DRAG_ENV.update(id=oid,fresh_id=None)
    return ''

ENV_MOVE={'on':False,'id':None,'phase':0,'pin':None,'g1':None,'cands':[]}
def emove_reset():
    ENV_MOVE.update(on=False,id=None,phase=0,pin=None,g1=None,cands=[])

def emove_start(oid):
    """드롭 시 호출. 핀이 2개 이상이면 배치 모드 시작, 아니면 (False,None)=폴백."""
    if _is_floor_plate(oid): return False,'플레이트는 더블클릭 확장으로만'
    if _env_ent(oid) is None: return False,f'설비를 찾을 수 없습니다: {oid}'
    if len(env_pins_local(oid))<2: return False,None
    rmove_reset()                          # 로봇 배치와 동시 진행 금지
    ENV_MOVE.update(on=True,id=oid,phase=0,pin=None,g1=None,cands=[])
    print(f"[설비 배치] {disp_name(oid)} — ① 주황 다리(핀) 클릭 "
          f"② 꽂을 목심 클릭 ③ 남은 다리가 갈 목심(하늘색) 클릭")
    return True,None

def _emove_pair(oid,a):
    """핀 a 의 짝이 될 두 번째 기준 핀: 20mm 배수 거리 우선, 그 중 최단."""
    pins=env_pins_local(oid); best=None; bk=None
    for j,q in enumerate(pins):
        if j==a: continue
        d=float(np.linalg.norm((q-pins[a])[:2]))*1000.0
        key=(0 if abs(d/20.0-round(d/20.0))<0.15 else 1, d)
        if bk is None or key<bk: best,bk=j,key
    return best

def emove_click_pin(idx):
    ENV_MOVE.update(phase=1,pin=int(idx),g1=None,cands=[])
    return True

def emove_click_peg(x,y):
    oid=ENV_MOVE['id']; pins=env_pins_local(oid)
    a=int(ENV_MOVE.get('pin') or 0)
    b=_emove_pair(oid,a)
    if b is None: return False
    if ENV_MOVE['phase']==1:
        d=float(np.linalg.norm((pins[b]-pins[a])[:2]))
        cands=[]
        for dx,dy in ((d,0),(-d,0),(0,d),(0,-d)):
            gx,gy=x+dx,y+dy
            if _on_plate(gx,gy): cands.append([round(gx,4),round(gy,4)])
        if not cands:
            print("[설비 배치] 그 목심 옆엔 남은 다리가 갈 자리가 없습니다 — 다른 목심을 고르세요")
            return False
        ENV_MOVE['g1']=[float(x),float(y)]
        ENV_MOVE.update(phase=2,cands=cands)
        return True
    if ENV_MOVE['phase']==2:
        hit=None
        for gx,gy in ENV_MOVE['cands']:
            if math.hypot(gx-x,gy-y)<0.005: hit=(gx,gy); break
        if hit is None: return False
        g1=np.array(ENV_MOVE['g1']); g2=np.array(hit)
        pa,pb=pins[a][:2],pins[b][:2]
        yaw=math.atan2(g2[1]-g1[1],g2[0]-g1[0])-math.atan2(pb[1]-pa[1],pb[0]-pa[0])
        yaw=round(yaw/(math.pi/2))*(math.pi/2)
        off=g1-_rz2(yaw)@pa
        _undo_snapshot('설비 위치 확정')
        set_env_pose(oid,off[0],off[1],yaw)
        print(f"[설비 배치] {disp_name(oid)}: 다리{a+1}->"
              f"({g1[0]*1000:.0f},{g1[1]*1000:.0f})mm 회전={math.degrees(yaw):.0f}도")
        emove_reset()
        return True
    return False

def _robot_pos_str():
    return (f"x={BASE_OFF[0]*1000:+.0f}mm  y={BASE_OFF[1]*1000:+.0f}mm"
            f"  회전={math.degrees(BASE_YAW[0]):.0f}도")

def set_robot_xy(x,y,save=True):
    """(구버전 호환) 회전 유지한 채 위치만 이동."""
    set_robot_pose(x,y,BASE_YAW[0],save=save)

def robot_to_tile(t):
    """타일 중앙으로 로봇 이동. 플레이트가 있는 타일만 허용."""
    if t not in PLATES: return False
    if RUNNING.is_set(): return False      # 동작 중 이동 금지
    ox,oy=tile_origin(t)
    set_robot_xy(ox+PLATE_SIZE/2, oy+PLATE_SIZE/2)
    print(f"[로봇 위치] 타일 {t} 중앙으로 -> {_robot_pos_str()}")
    return True

# ===== 설비/물체 인스턴스: 복제(팔레트에서 끌 때마다 새로) · 삭제(우클릭) =====
def _clone_base(oid):
    return str(oid).split('#')[0]

# 배치 인스턴스 자동 색: 빨-파-노-초-주-보... 눈에 확 띄는 순서
# 표준 6색 (무지개, 사용자 확정 2026-08-15): 누구나 이름이 갈리지 않는 색만.
# "이게 분홍인지 자홍인지" 논쟁 방지 — 어떤 색을 지정해도 이 6색으로 정착된다.
_AUTO_COLORS=['#e6194b',   # 빨강
              '#f58231',   # 주황
              '#ffe119',   # 노랑
              '#3cb44b',   # 초록
              '#4363d8',   # 파랑
              '#911eb4']   # 보라

def _sync_name_colors(keep=()):
    """설비 색(3D/카드)과 이름표 색(알약/음성 호칭)을 하나로 맞춘다.
    기준 = 공장에 보이는 설비 색. 전부 표준 6색으로 정착.
    (예전엔 불러오기가 설비 색만 복원해 알약/음성과 어긋났다)"""
    nm=S.setdefault('names',{})
    # ★ 색 규칙 ★ '같은 종류끼리' 만 안 겹치면 된다.
    #   도킹스테이션 #2 와 #3 은 서로 달라야 하지만,
    #   도킹스테이션과 ppap 는 같은 색이어도 상관없다 (종류가 다르니 헷갈리지 않는다).
    #   -> 종류(원본 이름)별로 따로 색을 관리한다.
    taken={}                       # 원본이름 -> 그 종류가 쓰는 색들
    def _take(kind):
        return taken.setdefault(kind,set())
    # 고정 배정(빨강#8·주황#9·노랑#12)을 먼저 자리잡게 한 뒤 나머지를 채운다.
    #   순서가 뒤죽박죽이면 원본이 복제본의 색을 선점해 같은 색 두 개가 남는다
    #   ("파란색으로 옮겨" 가 어느 쪽인지 못 정하는 사고).
    _fix={n:c for c,n in DOCK_FIXED}
    keep=set(keep or ())            # 11-05: 저장된 색(사용자가 고른 색)이 있는 것 — 그 색을 먼저 지킨다
    def _prio(e):
        i=e['id']; n=i.split('#')[-1]
        if i in keep: return (0,0)
        return (1,int(n)) if ('#' in i and n.isdigit() and int(n) in _fix) else (2,0)
    for e in sorted([x for x in ENV_SHOW if not _is_floor_plate(x['id'])],key=_prio):
        k=_clone_base(e['id']); t=_take(k)
        n=e['id'].split('#')[-1]
        if e['id'] not in keep and 'docking' in k.lower() and '#' in e['id'] and n.isdigit() \
           and int(n) in _fix and _fix[int(n)] not in t:
            c=_fix[int(n)]                     # 번호가 지정한 색 우선 (저장된 색이 없을 때만)
        else:
            c=_snap_color(e.get('color') or _AUTO_COLORS[0])
            if c in t:
                free=[x for x in _AUTO_COLORS if x not in t]
                if free: c=free[0]
        t.add(c)
        e['color']=c
        nm.setdefault(e['id'],{})['color']=c
    # 물체(ppap 등)도 같은 규칙 — ppap 끼리만 안 겹치게
    for bid,b in BRICKS.items():
        if b.get('pos') is None: continue
        k=_clone_base(bid); t=_take(k)
        cur=(nm.get(bid) or {}).get('color')
        c=_snap_color(cur) if cur else None
        if (c is None) or (c in t):
            free=[x for x in _AUTO_COLORS if x not in t]
            c=free[0] if free else (c or _AUTO_COLORS[0])
        t.add(c)
        nm.setdefault(bid,{})['color']=c

def _snap_color(c):
    """아무 색(hex) -> 표준 6색 중 가장 가까운 색."""
    try:
        h=str(c).lstrip('#')
        r,g,b=int(h[0:2],16),int(h[2:4],16),int(h[4:6],16)
    except Exception:
        return _AUTO_COLORS[0]
    def d2(p):
        return ((int(p[1:3],16)-r)**2+(int(p[3:5],16)-g)**2
                +(int(p[5:7],16)-b)**2)
    return min(_AUTO_COLORS,key=d2)
def _live_ids():
    """지금 공장에 실제로 있는 것들의 id (설비 + 물체)."""
    ids={e['id'] for e in ENV_SHOW if not _is_floor_plate(e['id'])}
    ids|={bid for bid,b in BRICKS.items() if b.get('pos') is not None}
    return ids

def _next_auto_color(kind=None):
    """다음 자동색.

    ★ '같은 종류' 중 지금 공장에 있는 것의 색만 피한다 ★
      도킹스테이션끼리는 달라야 하지만, 도킹스테이션과 ppap 는 같아도 된다.
      kind 를 안 주면 예전처럼 전체를 본다.

    예전에는 S['names'] 전체를 봤다. names 에는 지운 것, 다른 프리셋에만
    있는 것까지 남아 있어서(실측: 12개 중 10개가 유령) 6색 팔레트가 금세
    꽉 찼고, 그러면 마지막 줄로 떨어져 이미 쓰는 색을 또 줬다.
    -> 드래그로 새로 놓은 설비가 옆의 것과 같은 색이 되는 버그."""
    nm=S.get('names') or {}
    live=_live_ids()
    if kind:
        kind=_clone_base(kind)
        live={i for i in live if _clone_base(i)==kind}
    used={(nm.get(i) or {}).get('color') for i in live}
    for c in _AUTO_COLORS:
        if c not in used: return c
    # 그 종류를 6개 넘게 놓았다면 어쩔 수 없이 재사용 — 가장 적게 쓰인 색부터
    cnt={c:0 for c in _AUTO_COLORS}
    for i in live:
        c=(nm.get(i) or {}).get('color')
        if c in cnt: cnt[c]+=1
    return min(_AUTO_COLORS,key=lambda c:cnt[c])

# 도킹은 색과 번호를 고정 매칭한다: 1번째=빨강#8, 2번째=주황#9, 3번째=노랑#12.
#   지우고 다시 깔아도 같은 색이면 같은 번호 — 배치를 다시 해도 라벨이
#   안 바뀌어야 수업·발표 자료와 화면이 어긋나지 않는다.
#   세 개를 넘게 깔면 그때부터는 빈 번호 + 자동색.
DOCK_FIXED=[('#e6194b',8),('#f58231',9),('#ffe119',12)]   # 빨강, 주황, 노랑

def env_clone(base):
    """설비 복제본 생성 (STL 은 하나, 라벨만 #2,#3.. 늘어남).
    새 인스턴스에는 구분 잘 되는 자동색을 배정한다."""
    base=_clone_base(base)
    if base not in MESH_REG: return None
    col=None; n=None
    if 'docking' in base.lower():
        # '지금 화면에 있는 것' 기준으로 판정 — MESH_REG 는 삭제해도 남아서
        # 그걸 보면 지웠다 다시 깔 때 번호가 영영 안 풀린다.
        live={e['id'] for e in ENV_SHOW if _clone_base(e['id'])==base}
        usedc={(e.get('color') or '') for e in ENV_SHOW
               if _clone_base(e['id'])==base}
        for c,num in DOCK_FIXED:
            if c not in usedc and f"{base}#{num}" not in live:
                col=c; n=num; break
    if n is None:
        n=2
        while f"{base}#{n}" in MESH_REG: n+=1
    new=f"{base}#{n}"
    MESH_REG[new]=MESH_REG[base]
    if col is None:
        col=_next_auto_color(base)  # 같은 설비끼리만 색이 안 겹치면 된다
    ENV_SHOW.append({'id':new,'color':col,'visible':True})
    with LOCK:
        S.setdefault('env_clones',{})[new]=base
        S.setdefault('names',{})[new]={'name':'','color':col}
    _macro_save()
    print(f"[설비 복제] {disp_name(base)} -> {new} (색 {col})")
    return new

def brick_drop(bid,x,y,move=False):
    """물체(브릭) 드롭 = 반드시 '도킹 스테이션 위' 거치 (사용자 규칙).

    규칙: 물체의 거치 위치는 항상 도킹 스테이션 - 중심축 일치, 바닥=도킹 윗면
    (place_brick_on_docking 의 원래 규칙). 드롭 지점에서 가장 가까운
    '빈' 도킹을 골라 그 위에 세운다. 빈 도킹이 없으면 거절."""
    base=_clone_base(bid)
    if base not in BRICKS: return False,f'물체 없음: {bid}'
    if bid in BRICKS and (move or bid!=base
                          or BRICKS[base].get('pos') is None):
        tgt=bid                                    # 지정 인스턴스 그대로(이동 포함)
        made=False
    else:                                          # 배치된 원본 -> 새 복제본
        n=2
        while f"{base}#{n}" in BRICKS: n+=1
        tgt=f"{base}#{n}"
        BRICKS[tgt]=dict(BRICKS[base]); BRICKS[tgt]['pos']=None
        MESH_REG[tgt]=MESH_REG[base]
        with LOCK:
            S.setdefault('names',{})[tgt]={'name':'',
                                           'color':_next_auto_color(base)}
        made=True
    # 드롭 지점에서 가장 가까운 '빈' 도킹 (이미 결합된 도킹은 제외)
    cands=[]
    for did in docking_ids():
        e2=_env_ent(did)
        if e2 is not None and not e2.get('visible',True): continue
        t=dock_socket_world(did)
        if t is None: continue
        if not dock_occupied(did,except_bid=tgt):
            cands.append((math.hypot(t[0]-x,t[1]-y),did))
    if not cands:
        if made: BRICKS.pop(tgt,None); MESH_REG.pop(tgt,None)
        return False,'빈 도킹 스테이션이 없습니다 — 물체는 도킹 위에만 거치됩니다'
    cands.sort()
    return place_brick_on_docking(tgt,cands[0][1])

def env_delete(oid):
    """우클릭 삭제. 물체=배치 해제(복제본은 완전 삭제), 설비=목록에서 영구 제거."""
    oid=str(oid)
    if _is_floor_plate(oid): return False,'바닥 플레이트는 삭제할 수 없습니다'
    if _clone_base(oid) in BRICKS or oid in BRICKS:     # 물체
        if '#' in oid: BRICKS.pop(oid,None); MESH_REG.pop(oid,None)
        elif oid in BRICKS: BRICKS[oid]['pos']=None
        with LOCK:
            (S.get('brick_pos') or {}).pop(oid,None)
            if '#' in oid: (S.get('names') or {}).pop(oid,None)  # 자동색 반납
        _macro_save()
        return True,f'물체 삭제: {oid}'
    if _env_ent(oid) is None: return False,f'없음: {oid}'
    # 도킹을 지우면 거기 '결합'된 물체도 같이 삭제
    if 'docking' in oid.lower():
        t=dock_socket_world(oid)
        if t:
            for ob in list(BRICKS):
                bp=BRICKS[ob].get('pos')
                if bp and math.hypot(bp[0]-t[0],bp[1]-t[1])<0.025:
                    if '#' in ob: BRICKS.pop(ob,None); MESH_REG.pop(ob,None)
                    else: BRICKS[ob]['pos']=None
                    with LOCK:
                        (S.get('brick_pos') or {}).pop(ob,None)
                    print(f"[삭제 연쇄] 결합된 물체 {ob} 도 함께 삭제")
    ENV_SHOW[:]=[e for e in ENV_SHOW if e['id']!=oid]
    with LOCK:
        (S.get('env_clones') or {}).pop(oid,None)
        (S.get('env_off') or {}).pop(oid,None)
        if '#' in oid: (S.get('names') or {}).pop(oid,None)  # 자동색 반납
        if '#' not in oid:                     # 원본은 재시작 후에도 제외되게
            S.setdefault('env_removed',[])
            if oid not in S['env_removed']: S['env_removed'].append(oid)
    if ENV_MOVE.get('id')==oid:
        emove_reset(); ENV_MOVE['on']=False
    _OBS['grid']=None; _clear_plan_cache()
    _macro_save()
    return True,f'설비 삭제: {disp_name(oid)}'

# ===== 실행취소 (Ctrl+Z) — 편집 전 스냅샷을 쌓아두고 통째로 되돌린다 =====
UNDO=[]
def _undo_snapshot(tag=''):
    """편집(배치/삭제/복제/블록)이 일어나기 직전 상태 저장."""
    try:
        with LOCK:
            snap={'tag':tag,
                  'seq':list(S['seq']),
                  'env':[dict(e) for e in ENV_SHOW],
                  'env_clones':dict(S.get('env_clones') or {}),
                  'env_removed':list(S.get('env_removed') or []),
                  'bricks':{bid:dict(b) for bid,b in BRICKS.items()}}
        UNDO.append(snap)
        if len(UNDO)>50: UNDO.pop(0)
    except Exception as e:
        print(f"[undo] 스냅샷 실패: {e}")

def undo_last():
    """마지막 편집을 통째로 되돌린다."""
    if not UNDO: return False,'되돌릴 것이 없습니다'
    s=UNDO.pop()
    with LOCK:
        S['seq'][:]=s['seq']
        S['env_clones']=dict(s['env_clones'])
        S['env_removed']=list(s['env_removed'])
    ENV_SHOW[:]=[]
    for e in s['env']:
        oid=e['id']; base=_clone_base(oid)
        if oid not in MESH_REG and base in MESH_REG:
            MESH_REG[oid]=MESH_REG[base]
        if oid in MESH_REG or oid.startswith('plate_tile'):
            ENV_SHOW.append(dict(e))
    cur=set(BRICKS)
    for bid,b in s['bricks'].items():
        if bid not in BRICKS:
            BRICKS[bid]=dict(b)
            base=_clone_base(bid)
            if base in MESH_REG: MESH_REG[bid]=MESH_REG[base]
        else:
            BRICKS[bid]['pos']=b.get('pos')
    for bid in (cur-set(s['bricks'])):
        BRICKS.pop(bid,None)
    with LOCK:
        S['brick_pos']={bid:list(b['pos']) for bid,b in BRICKS.items()
                        if b.get('pos') is not None}
        S['env_off']={e['id']:[e.get('offset',(0,0,0))[0],
                               e.get('offset',(0,0,0))[1],
                               float(e.get('yaw',0.0))] for e in ENV_SHOW}
    _OBS['grid']=None; _clear_plan_cache()
    if ENV_MOVE.get('on'): emove_reset(); ENV_MOVE['on']=False
    _macro_save()
    tag=s.get('tag') or '편집'
    print(f"[실행취소] {tag} 되돌림")
    return True,f'실행취소: {tag}'

# ===== 파이프라인 도달 판정 + 로봇 자리 추천 =====
# 도달표가 '로봇 상대좌표'라서 후보 자리 × 타깃 전부를 표 조회만으로
# 판정할 수 있다 (IK 0회, 밀리초).
def _reach_at(off2,yaw,p):
    """후보 로봇 자리(off2,yaw) 기준으로 월드 점 p 도달 여부. 표 없으면 None."""
    if not peg_table_ready(): return None
    o=np.asarray(off2,float)[:2]
    d=_rz2(-yaw)@(np.array([float(p[0]),float(p[1])])-o)
    gx=round((d[0]-0.01)/0.02)*0.02+0.01     # 표 격자(10+20i)mm 로 스냅
    gy=round((d[1]-0.01)/0.02)*0.02+0.01
    sp=PEG_TABLE['span'].get((int(round(gx*1000)),int(round(gy*1000))))
    if not sp: return False
    h=(float(p[2])-PLATE_TOP)*1000.0
    return any(a-0.5<=h<=b+0.5 for a,b in sp)

def _seq_targets():
    """파이프라인에서 '세상에 고정된' 타깃: [(블록 라벨, [x,y,z] or None)]
    11-05 (사용자): 물체는 파이프라인 순서대로 따라간다 — 앞 블록이 '잡기 → 놓기(도킹)' 로 옮겨 놓았으면
    그 뒤의 '이동: 물체'·'잡기' 는 지금 자리가 아니라 그 도킹 자리로 검사한다 (B팔이 놓은 펜을 A팔이 받는 경우).
    예전에는 늘 지금 자리로 봐서 '도달 범위 밖' 이 잘못 떴다."""
    out=[]
    moved={}                    # 물체 id -> 이 순서에서 놓인 도킹 id
    held=None                   # 지금 잡고 있는 물체 (잡기 ~ 놓기 사이)
    for lab in seq_labels():
        mm_=re.match(r'^[◈➜]\s*(\d+)\.',lab)
        if not mm_: continue
        i=int(mm_.group(1))
        if not 0<=i<len(S['seq']): continue
        item=S['seq'][i]
        if item.startswith(GOTO_PREFIX) and item[len(GOTO_PREFIX):] in moved:
            dk=moved[item[len(GOTO_PREFIX):]]       # 앞에서 옮겨 놓은 물체 → 그 도킹 위로
            p=goto_point(dk); t2=dock_socket_world(dk)
            if p is None or t2 is None:
                out.append((lab,None)); continue
            gz=float(t2[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT
            out.append((lab,[float(p[0]),float(p[1]),float(p[2])]))
            out.append((lab,[float(p[0]),float(p[1]),gz]))
            out.append((lab,[float(p[0]),float(p[1]),(float(p[2])+gz)/2.0]))
            continue
        if item.startswith(GOTO_PREFIX) and held:
            _gp=goto_point(item[len(GOTO_PREFIX):])   # 11-05: 펜을 들고 가는 이동 — 안전 높이(펜이 늘어진 만큼)로 그 위에 닿아야
            if _gp is not None:
                out.append((lab,[float(_gp[0]),float(_gp[1]),max(float(_gp[2]),_clear_z_holding(held,moved))]))
        if item.startswith(GOTO_PREFIX):
            oid=item[len(GOTO_PREFIX):]
            e2=_env_ent(oid)
            if e2 is not None and not e2.get('visible',True):
                out.append((lab,None)); continue    # 숨겨진 원본 = 미배치
            p=goto_point(oid)
            if p is None:
                out.append((lab,None)); continue
            out.append((lab,[float(p[0]),float(p[1]),float(p[2])]))
            # 그 자리에서 '하강'까지 할 수 있어야 하므로 잡는 높이도 함께 검사
            gz=None
            if _clone_base(oid) in BRICKS or oid in BRICKS:
                gp=brick_grasp_point(oid)
                if gp is not None: gz=float(gp[2])
            elif 'docking' in oid.lower():
                t2=dock_socket_world(oid)
                if t2 is not None:
                    m3=_dock_mate()
                    gz=float(t2[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT   # 115+10+37
            if gz is not None:
                out.append((lab,[float(p[0]),float(p[1]),gz]))
                out.append((lab,[float(p[0]),float(p[1]),
                                 (float(p[2])+gz)/2.0]))   # 하강 중간 높이
        elif item.startswith(GRASP_PREFIX):
            bid=item[len(GRASP_PREFIX):]
            held=bid
            if bid in moved:                         # 앞에서 옮겨 놓은 물체 → 그 도킹에서 잡는다
                t=dock_socket_world(moved[bid])
                p=None if t is None else [float(t[0]),float(t[1]),float(t[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT]
            else:
                p=brick_grasp_point(bid)
            out.append((lab,None if p is None else
                        [float(p[0]),float(p[1]),float(p[2])]))
            if p is not None:                        # 11-05: 잡은 뒤 상승 높이도 (실제 잡기와 같은 식)
                _top=(_pen_top_seated(bid,moved[bid]) if bid in moved else _pen_top_now(bid))
                _zu=max(float(p[2])+GRASP_LIFT,(_top+0.02) if _top is not None else 0.0)
                out.append((lab,[float(p[0]),float(p[1]),_zu]))
        elif item.startswith(RELEASE_PREFIX):
            did=item[len(RELEASE_PREFIX):]
            if held:                                 # 이 순서에서 그 물체는 이제 이 도킹에 있다
                moved[held]=did; moved['__last_rel__']=held; held=None
            else:
                moved.pop('__last_rel__',None)
            e3=_env_ent(did)
            if e3 is not None and not e3.get('visible',True):
                out.append((lab,None)); continue    # 숨겨진 도킹 = 미배치
            t=dock_socket_world(did)
            if t is None: out.append((lab,None)); continue
            _zg=float(t[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT
            out.append((lab,[float(t[0]),float(t[1]),_zg]))
            _rb=moved.get('__last_rel__')            # 방금 놓은 물체 (위에서 moved 에 적어 둠)
            _top=_pen_top_seated(_rb,did) if _rb else None
            _zu=max(_zg+GRASP_LIFT,(_top+0.02) if _top is not None else 0.0)   # 11-05: 놓기③ 상승 높이
            out.append((lab,[float(t[0]),float(t[1]),_zu]))
    return out

def _clear_z_holding(hb,moved=None):
    """_clear_z 와 같은 식 — 물체 hb 를 들고 있을 때 수평 이동 안전 고도 (파이프라인 검사용)."""
    hz=PLATE_TOP
    moved=moved or {}
    for b,v in BRICKS.items():
        if b==hb or v.get('pos') is None: continue
        top=_pen_top_seated(b,moved[b]) if b in moved else float(v['pos'][2])+float(v.get('h',0.0))
        if top is not None: hz=max(hz,top)
    for dk in docking_ids():
        e=_env_ent(dk)
        if e is not None and not e.get('visible',True): continue
        t=docking_top(dk)
        if t: hz=max(hz,float(t[2]))
    bb=BRICKS.get(hb) or BRICKS.get(_clone_base(hb))
    return hz+0.02+(float(bb.get('grasp_dz',GRASP_DZ_DEFAULT)) if bb else 0.0)

def _pen_top_now(bid):
    """물체의 지금 꼭대기 높이 (배치 안 됐으면 None)."""
    b=BRICKS.get(bid) or BRICKS.get(_clone_base(bid))
    if not b or b.get('pos') is None: return None
    return float(b['pos'][2])+float(b.get('h',0.0))

def _pen_top_seated(bid,did):
    """물체를 그 도킹에 꽂았을 때의 꼭대기 높이 (place_brick_on_docking 과 같은 규칙)."""
    b=BRICKS.get(bid) or BRICKS.get(_clone_base(bid))
    t=docking_top(did)
    if not b or t is None: return None
    m=_dock_mate()
    zb=float(t[2])-(m['seat_dz_mm']/1000.0 if (m and _clone_base(bid)==m.get('brick')) else 0.0)
    return zb+float(b.get('h',0.0))

def seq_reach():
    """현재 로봇 위치 기준 블록별 도달 판정: {라벨:{'ok','why'}}
    두 팔 (10-30): 블록마다 '그 블록을 하는 팔' 의 자리로 판정한다."""
    out={}
    arm_at={}; L='A'
    for i,x in enumerate(S['seq']):
        if is_arm_item(x): L=x[len(ARM_PREFIX):]
        arm_at[i]=L
    for lab,p in _seq_targets():
        _m=re.match(r'^[◈➜]\s*(\d+)\.',lab)
        _L=arm_at.get(int(_m.group(1)),'A') if _m else 'A'
        if _L!=ACTIVE_ARM[0]:
            _x,_y,_yw=arm_pose_of(_L); _off=np.array([_x,_y,0.0])
        else: _off,_yw=BASE_OFF,BASE_YAW[0]
        if p is None:
            out[lab]={'ok':False,
                      'why':'대상이 공장에 배치되어 있지 않습니다 — '
                            '①공장 꾸미기에서 끌어다 놓으세요'}
            continue
        r=_reach_at(_off,_yw,p)
        if r is None: continue                     # 도달표 없음 - 보류
        ok=bool(r)
        prev=out.get(lab)
        if prev is not None: ok=ok and prev['ok']  # 접근+잡는높이 둘 다 AND
        out[lab]={'ok':ok,
                  'why':'' if ok else
                  f'{arm_label(_L)} 자리에서 도달 범위 밖 — 로봇 자리 추천을 눌러보세요'}
    return out
SUGGEST={'pose':None}
def h_robot_suggest():
    """파이프라인 전체를 수행할 수 있는 로봇 자리 중 최소 이동인 곳 추천."""
    tg=[(lab,p) for lab,p in _seq_targets() if p is not None]
    if not tg: return '검사할 잡기/놓기 블록이 없습니다'
    if not peg_table_ready(): return '도달표가 없어 판정할 수 없습니다'
    bad=[lab for lab,p in tg
         if _reach_at(BASE_OFF,BASE_YAW[0],p) is False]
    pins=robot_pins_local()
    pa=np.asarray(pins[0][:2],float); pb=np.asarray(pins[1][:2],float)
    pegs=_pegs_all_world()
    pegset={(round(x,3),round(y,3)) for x,y in pegs}
    best=None; cands2=[]
    for g1 in pegs:
        g1=np.asarray(g1,float)
        for k in range(4):
            yaw=k*math.pi/2
            g2=g1+_rz2(yaw)@(pb-pa)                # 두 번째 발도 목심 위여야
            if (round(float(g2[0]),3),round(float(g2[1]),3)) not in pegset:
                continue
            off=g1-_rz2(yaw)@pa
            if not all(_reach_at(off,yaw,p) for _,p in tg): continue
            dyaw=abs(((yaw-BASE_YAW[0]+math.pi)%(2*math.pi))-math.pi)
            dist=math.hypot(off[0]-BASE_OFF[0],off[1]-BASE_OFF[1])+0.001*dyaw
            cands2.append((dist,float(off[0]),float(off[1]),float(yaw)))
    cands2.sort()
    # 도달표는 격자 근사라, 진짜 IK 로 최종 확인 (가까운 후보부터 20곳)
    def _ik_ok(off2,yaw2,p):
        rel=_rz2(-yaw2)@(np.array([p[0],p[1]])-np.asarray(off2,float)[:2])
        pw=l2w(np.array([rel[0],rel[1],0.0]))
        q0=list(CURRENT_Q)
        return ik_tool(np.array([pw[0],pw[1],p[2]]),
                       _seed_list(q0),q0[4],q0[GRIP_IDX]) is not None
    for c in cands2[:20]:
        if all(_ik_ok(c[1:3],c[3],p) for _,p in tg):
            best=c; break
    if best is None:
        SUGGEST['pose']=None
        return ('어떤 로봇 자리에서도 전부 도달할 수 없습니다 — '
                '설비들을 더 가깝게 모아보세요')
    if not bad and best[0]<0.005:
        SUGGEST['pose']=None
        return '지금 자리에서 파이프라인 전체 수행 가능합니다 ✓'
    SUGGEST['pose']=(best[1],best[2],best[3])
    mv=math.hypot(best[1]-BASE_OFF[0],best[2]-BASE_OFF[1])*1000
    head=(f'못 가는 블록 {len(bad)}개 → ' if bad else '지금도 가능하지만 더 가까운 ')
    return (head+f'추천 자리: x={best[1]*1000:+.0f}mm '
            f'y={best[2]*1000:+.0f}mm 회전={math.degrees(best[3]):.0f}도 '
            f'(이동 {mv:.0f}mm)')

def h_robot_apply_suggest():
    p=SUGGEST.get('pose')
    if not p: return '적용할 추천 자리가 없습니다 — 먼저 [로봇 자리 추천]'
    if RUNNING.is_set(): return '동작 중에는 옮길 수 없습니다'
    _undo_snapshot('로봇 자리 이동')
    set_robot_pose(p[0],p[1],p[2])
    SUGGEST['pose']=None
    return f'로봇 이동: {_robot_pos_str()}'

# ===== 공장 배치 프리셋 (저장/불러오기/새 공장) =====
def factory_snapshot():
    """현재 공장 배치(설비/물체/로봇)를 저장용 dict 로.
    11-05 (사용자): 색을 바꾼 내역도 함께 — 설비·물체의 이름표(이름·색)를 'names' 로 저장한다."""
    _ids={e['id'] for e in ENV_SHOW}|set(BRICKS)
    return {'env':[dict(e) for e in ENV_SHOW],
            'env_clones':dict(S.get('env_clones') or {}),
            'bricks':{bid:{'pos':(list(b['pos']) if b.get('pos') else None)}
                      for bid,b in BRICKS.items()},
            'robot':[float(BASE_OFF[0]),float(BASE_OFF[1]),float(BASE_YAW[0])],
            'names':{i:dict(d) for i,d in (S.get('names') or {}).items() if i in _ids and isinstance(d,dict)}}

def _factory_apply(snap):
    """스냅샷을 현재 상태로 적용 (설비/복제본/물체/로봇)."""
    with LOCK:
        S['env_clones']=dict(snap.get('env_clones') or {})
    ENV_SHOW[:]=[]
    for e in (snap.get('env') or []):
        oid=e['id']; base=_clone_base(oid)
        if oid.startswith('plate_tile') and tuple(e.get('tile') or ()) not in set(PLATE_LAYOUT):
            continue                                # 옛 프리셋의 배치 밖 판은 안 되살린다 (바닥판은 늘 2×4, 11-03)
        if oid not in MESH_REG and base in MESH_REG:
            MESH_REG[oid]=MESH_REG[base]
        if oid in MESH_REG or oid.startswith('plate_tile'):
            ENV_SHOW.append(dict(e))
    # ---- 플레이트 장부(PLATES) 동기화 — '유령 칸' 방지 ----
    # ① 화면(ENV)에 없어진 '추가 플레이트'는 장부에서도 지운다
    #    (안 지우면: 판은 안 보이는데 초록 확장 후보도 안 뜨는 중첩 상태가 됨)
    _envids={e['id'] for e in ENV_SHOW}
    for _t4,_inf4 in list(PLATES.items()):
        if _inf4.get('added') and _inf4['id'] not in _envids:
            PLATES.pop(_t4,None)
    # ② 프리셋에 들어있는 추가 플레이트는 장부/메쉬에 재등록 (재시작 후에도)
    for e in ENV_SHOW:
        if str(e.get('id','')).startswith('plate_tile') and e.get('tile'):
            _t4=tuple(e['tile'])
            if PLATE_MESH_PATH: MESH_REG.setdefault(e['id'],(PLATE_MESH_PATH,0.001))
            PLATES.setdefault(_t4,{'id':e['id'],
                                   'origin':tile_origin(_t4),'added':True})
    try:         # ★ 바닥판은 늘 기본 2×4 (11-03) ★ 프리셋에 없는 칸은 채운다
        for _t4 in PLATE_LAYOUT:
            if _t4 not in PLATES: add_plate(_t4)
    except NameError: pass          # 부팅 초기(add_plate 정의 전) — 부팅 끝 '기본 바닥판' 이 채운다
    with LOCK:   # 저장 목록도 현재 장부와 일치 (재시작 시 되살아남 방지)
        S['plates_added']=[list(t) for t,i in PLATES.items() if i.get('added')]
    try: _peg_layout_changed()
    except NameError: pass          # 부팅 초기(정의 전)에 불릴 수 있음
    # 11-05 (사용자): 저장된 색 그대로 — 이름표(이름·색)를 되살리고, 저장된 색이 있는 것은 정리할 때 그 색을 지킨다
    _keep=set()
    for _i,_d in (snap.get('names') or {}).items():
        if isinstance(_d,dict):
            with LOCK: S.setdefault('names',{})[_i]=dict(_d)
            if _d.get('color'): _keep.add(_i)
    for _e in ENV_SHOW:
        _c=((snap.get('names') or {}).get(_e['id']) or {}).get('color')
        if _c: _e['color']=_c
        if _e.get('color') and not _is_floor_plate(_e['id']): _keep.add(_e['id'])
    _sync_name_colors(_keep)        # 3D 색과 알약/음성 색 동기화
    keep=set()
    for bid,info in (snap.get('bricks') or {}).items():
        base=_clone_base(bid)
        if bid not in BRICKS and base in BRICKS:
            BRICKS[bid]=dict(BRICKS[base]); MESH_REG[bid]=MESH_REG[base]
        if bid in BRICKS:
            BRICKS[bid]['pos']=(tuple(info['pos']) if info.get('pos') else None)
            keep.add(bid)
    for bid in list(BRICKS):
        if bid not in keep:
            if '#' in bid: BRICKS.pop(bid,None); MESH_REG.pop(bid,None)
            else: BRICKS[bid]['pos']=None
    r=(snap.get('robot') or [0.0,0.0,0.0])
    set_robot_pose(float(r[0]),float(r[1]),float(r[2]),save=False)
    # 결합 정규화: 도킹 소켓 근처의 물체는 '현재 규칙(메이트 + 37mm)'으로
    # 다시 앉힌다 - 옛 프리셋에 저장된 낡은 높이(z)를 자동 치유
    for _bid2,_b3 in list(BRICKS.items()):
        _bp2=_b3.get('pos')
        if not _bp2: continue
        for _dk2 in docking_ids():
            _e4=_env_ent(_dk2)
            if _e4 is None or not _e4.get('visible',True): continue
            _t4=dock_socket_world(_dk2)
            if _t4 and math.hypot(_bp2[0]-_t4[0],_bp2[1]-_t4[1])<0.03:
                place_brick_on_docking(_bid2,_dk2)
                print(f"[결합 정규화] {_bid2} -> {disp_name(_dk2)} 재안착")
                break
    # 잘못 등록된 잡기/놓기 자세 지우기 (좌표가 둘이 되는 원인)
    try:
        _rm=[p['name'] for p in S['poses']
             if ('_잡기(' in p['name'] or '_놓기(' in p['name'])]
        if _rm:
            with LOCK:
                S['poses']=[p for p in S['poses'] if p['name'] not in _rm]
            print(f"[자세 정리] 잡기/놓기 자세 {len(_rm)}개를 지웠습니다 "
                  f"(좌표는 [이동] 블록만 가집니다): {', '.join(x[:36] for x in _rm)}")
    except Exception as _e:
        print('자세 정리 실패:',_e)
    # 옛 보정 합치기: offset 을 각도에 녹여 넣어 '값 하나' 로 만든다 (구형 방식)
    try:
        _m=0
        for _p in S['poses']:
            # ★ 11-06 (사용자: "왜 계속 보정값이 바뀌냐") ★ 자리 보정 자세(rel)는 절대 건드리지 않는다.
            #   예전엔 지금 팔(A, 불러오기는 A팔로 함)의 'sts' 보정을 B팔 자세에도 합쳐서, B팔 자세를 만들 때 복사돼
            #   들어간 A팔 base 보정(j2+1 j3-2)이 각도에 녹아 B팔 보정이 j2+1 j3-2 j4+1 바뀌었다 (X+430,Y-43 실측).
            #   자리 보정은 [보정 저장] 때 이미 각도에 합쳐지므로 여기서 할 일이 없다.
            if _p.get('rel') is not None or _p.get('arm') not in (None,'A'): continue
            _o=get_offset(_p['name'])
            if not any(abs(v)>1e-9 for v in _o): continue
            _a=(list(_p.get('angles') or [])+[0.0]*NJOINT)[:NJOINT]
            _lo=level_offset(_o) if LEVEL_ALWAYS else list(_o)
            for _j in range(NJOINT):
                _l,_h=jr_limits(_j)
                _a[_j]=max(_l,min(_h,_a[_j]+float(_lo[_j] if _j<len(_lo) else 0.0)))
            _p['angles']=_a
            set_offset(_p['name'],[0.0]*NJOINT)
            _m+=1
        if _m:
            print(f"[보정 정리] 자세 {_m}개의 미세조정을 각도에 반영했습니다 "
                  f"(구형 방식 — 값이 하나뿐이라 어긋나지 않습니다)")
    except Exception as _e:
        print('보정 정리 실패:',_e)
    # 옛 파이프라인 고치기: 잡기/놓기 앞에 [이동] 이 없으면 넣어 준다
    # ★ 11-04: 끔 ★ 잡기·놓기가 스스로 대상 위로 가므로(_goto_before) 더는 필요 없다.
    #   켜 두면 재시작할 때마다 사용자가 만든 파이프라인에 이동 블록이 몰래 끼어든다.
    try:
        _fx,_n=(None,0)
        if _n:
            with LOCK: S['seq']=_fx
            print(f"[파이프라인 정리] 잡기/놓기 앞에 [이동] {_n}개를 넣었습니다 "
                  f"(잡기·놓기는 이제 제자리 동작입니다)")
    except Exception as _e:
        print('파이프라인 정리 실패:',_e)
    with LOCK:
        S['brick_pos']={bid:list(b['pos']) for bid,b in BRICKS.items()
                        if b.get('pos')}
        S['env_off']={e['id']:[e.get('offset',(0,0,0))[0],
                               e.get('offset',(0,0,0))[1],
                               float(e.get('yaw',0.0))] for e in ENV_SHOW}
        S['env_vis']={e['id']:bool(e.get('visible',True)) for e in ENV_SHOW}
        S['robot_xy']=[float(r[0]),float(r[1])]; S['robot_yaw']=float(r[2])
    _OBS['grid']=None; _clear_plan_cache()

def factory_default():
    """깨끗한 기본 공장: 플레이트만 보임. 설비 원본은 숨김(팔레트 전용),
    복제본/물체 배치 없음, 로봇은 원점(사용자가 직접 배치)."""
    env=[]
    for e in ENV_SHOW:
        if '#' in e['id']: continue                 # 복제본 제외
        d=dict(e)
        if not _is_floor_plate(e['id']):
            d['visible']=False; d['offset']=(0.0,0.0,0.0); d['yaw']=0.0
        env.append(d)
    return {'env':env,'env_clones':{},
            'bricks':{bid:{'pos':None} for bid in BRICKS if '#' not in bid},
            'robot':[0.0,0.0,0.0]}

def h_factory_save(name):
    """배치 저장. 11-05 (사용자): 이름을 비우면 '지금 배치'(마지막으로 불러온·저장한 이름)에 덮어쓴다."""
    name=str(name or '').strip() or LAST_PRESET[0]
    if not name: return '배치 이름을 입력하세요 (아직 불러온 배치가 없습니다)'
    _prev=ACTIVE_ARM[0]
    if _prev!='A': arm_switch('A')          # 11-05: 공장 배치는 A팔 자리로 저장 (불러오기와 같게 — B팔 자리가 저장되던 문제)
    try:
        with LOCK:
            _old=name in (S.get('factory_presets') or {})
            S.setdefault('factory_presets',{})[name]=factory_snapshot()
    finally:
        if _prev!='A': arm_switch(_prev)
    LAST_PRESET[0]=name
    _macro_save()
    return f"공장 배치 {'덮어쓰기' if _old else '저장'}: {name}"

LAST_PRESET=['']        # 마지막으로 [불러오기] 한 배치 이름 (원위치 때 물체를 되돌릴 기준)

def h_factory_load(name):
    if ACTIVE_ARM[0]!='A': arm_switch('A')        # 공장 배치는 A팔 자리로 저장·복원한다 (10-30)
    return _h_factory_load(name)
def _h_factory_load(name):
    p=(S.get('factory_presets') or {}).get(str(name))
    if not p: return f'저장된 배치가 없습니다: {name}'
    _undo_snapshot('배치 불러오기')
    _factory_apply(p)
    # ★ 불러온 로봇 자리도 저장 (11-04) ★ _factory_apply 는 save=False 라 화면만 옮기고 S['robot_xy'] 는
    #   그대로였다 -> 다시 켜면 로봇이 옛 자리(0,0 등)로 복원되고, 그 자리로 '이전 배치' 까지 덮어썼다.
    with LOCK:
        S['robot_xy']=[float(BASE_OFF[0]),float(BASE_OFF[1])]; S['robot_yaw']=float(BASE_YAW[0])
    _macro_save()
    LAST_PRESET[0]=str(name)
    return f'공장 배치 불러옴: {name}'

def _bricks_to_preset(name):
    """물체(펜 등)만 배치 프리셋의 위치로 되돌린다. 설비·로봇 위치는 그대로.
    반환: 되돌린 물체 수, 프리셋이 없으면 None."""
    p=(S.get('factory_presets') or {}).get(str(name))
    if not p: return None
    old={b:v.get('pos') for b,v in BRICKS.items()}    # 도킹 없는 자리면 이걸로 되돌린다 (11-04)
    keep=set()
    for bid,info in (p.get('bricks') or {}).items():
        base=_clone_base(bid)
        if bid not in BRICKS and base in BRICKS:
            BRICKS[bid]=dict(BRICKS[base]); MESH_REG[bid]=MESH_REG[base]
        if bid in BRICKS:
            BRICKS[bid]['pos']=(tuple(info['pos']) if info.get('pos') else None)
            keep.add(bid)
    for bid in list(BRICKS):
        if bid not in keep:
            if '#' in bid: BRICKS.pop(bid,None); MESH_REG.pop(bid,None)
            else: BRICKS[bid]['pos']=None
    HELD['brick']=None
    # 프리셋 불러오기와 같은 규칙으로 도킹에 다시 꽂는다 (결합 정규화)
    # ★ 펜은 도킹 위에만 있다 (2026-11-04, 사용자) ★
    #   프리셋 자리에 지금 도킹이 없으면 (도킹을 옮겼거나 숨김) 펜이 판 위 허공에 서 버렸다.
    #   -> 그런 펜은 프리셋으로 옮기지 않고 원래 자리(도킹 위)에 그대로 둔다.
    for bid,b in list(BRICKS.items()):
        bp=b.get('pos')
        if not bp: continue
        for dk in docking_ids():
            e=_env_ent(dk)
            if e is None or not e.get('visible',True): continue
            t=dock_socket_world(dk)
            if t and math.hypot(bp[0]-t[0],bp[1]-t[1])<0.03:
                place_brick_on_docking(bid,dk); break
        else:
            if old.get(bid):
                b['pos']=old[bid]
                print(f"[원위치] {disp_name(bid)}: 프리셋 자리에 도킹이 없어 지금 자리(도킹 위)에 그대로 둡니다")
    with LOCK:
        S['brick_pos']={bid:list(b['pos']) for bid,b in BRICKS.items() if b.get('pos')}
    _macro_save()
    return len([b for b in keep if BRICKS.get(b,{}).get('pos')])

def h_factory_reset():
    _undo_snapshot('새 공장')
    _factory_apply(factory_default()); _macro_save()
    return '새 공장: 플레이트만 남김 (설비는 팔레트에서 끌어오세요)'

def h_factory_del(name):
    with LOCK:
        (S.get('factory_presets') or {}).pop(str(name),None)
    _macro_save()
    return f'배치 삭제: {name}'

# 시작 시: 설비 복제본 복원 + 삭제 목록 반영 (env_off 적용 전에)
for _cid,_cbase in (S.get('env_clones') or {}).items():
    if _cbase in MESH_REG and not any(e['id']==_cid for e in ENV_SHOW):
        MESH_REG[_cid]=MESH_REG[_cbase]
        _ccol=next((e['color'] for e in ENV_SHOW if e['id']==_cbase),'#b8865b')
        ENV_SHOW.append({'id':_cid,'color':_ccol,'visible':True})
        print(f"설비 복제 복원: {_cid}")
for _rid in (S.get('env_removed') or []):
    ENV_SHOW[:]=[e for e in ENV_SHOW if e['id']!=_rid]

# 시작 시 저장된 표시 여부(visible) 복원
for _e in ENV_SHOW:
    _v=(S.get('env_vis') or {}).get(_e['id'])
    if _v is not None: _e['visible']=bool(_v)

# 시작 시 저장된 로봇 위치 복원
for _oid,_o in (S.get('env_off') or {}).items():
    for _e in ENV_SHOW:
        if _e['id']==_oid:
            _e['offset']=(float(_o[0]),float(_o[1]),0.0)
            if len(_o)>2: _e['yaw']=float(_o[2])
if S.get('robot_xy'):
    set_robot_pose(S['robot_xy'][0],S['robot_xy'][1],
                   float(S.get('robot_yaw',0.0)),save=False)
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
    return nm if ik_tool((x,y,z),_seed_list(q0),q0[4],q0[GRIP_IDX]) is not None else None

# ===== 목심 도달 구간 표 (한 번 만들고 계속 조회) =====
# 각 목심의 도달 가능 높이가 '연속 구간' 임을 실측으로 확인했다(중간 끊김 0).
# 그래서 목심마다 [zmin,zmax] 두 값만 저장하면 어떤 높이든 즉시 판정된다.
#   -> 1mm 는 물론 소수점 높이까지 조회 가능(해상도 제한 없음)
# 표는 (로봇위치, URDF) 가 바뀌면 무효화된다.
PEG_H_MAX=250          # 표가 담는 최대 높이(mm). 슬라이더 상한과 같아야 한다
# 도달표는 "로봇 기준 상대좌표" 로 저장한다.
#   - 도달 가능성은 로봇에서 (dx,dy) 떨어진 지점의 성질이라 로봇을 어디로
#     옮기든(90도 회전 포함) 표는 그대로다. 재계산이 아니라 조회만 한다.
#   - 목심도 로봇 발도 전부 20mm 격자 + 90도 스냅이라, 상대좌표는 항상
#     (10+20i, 10+20j)mm 격자에 떨어진다 (검증: 핀 로컬 (270,±270) 의
#     4회전 모두 mod 20 = (10,10)).
#   - 실제 이동(목심 클릭 -> IK -> 애니메이션)은 표와 무관하게 지금 위치에서
#     라이브로 푼다. 표는 같은 ik_tool 을 로컬 변위로 돌려 만든 것이라
#     판정과 실제 이동이 어긋나지 않는다.
PEG_TABLE={'key':None,'span':{}}     # span: {(dx_mm,dy_mm): [(a,b),...]}
_PEG_SCAN={'busy':False,'msg':''}

def _rel_span_one(args):
    """로봇 로컬 변위 (dx,dy)m 지점의 도달 높이 구간들 [(a,b),...] (mm).

    주의: 도달 영역이 항상 하나의 연속 구간은 아니다. 관절 한계 때문에
    중간이 끊기는 지점이 실제로 존재한다 -> '구간 목록' 으로 저장.
    reach_ring(높이별 수평거리 한계)으로 IK 없이 거를 수 있는 높이는 걸러
    1mm 전수 검사 비용을 크게 줄인다.
    """
    dx,dy=args
    q0=list(CURRENT_Q); seeds=_seed_list(q0)
    o1=CHAIN.forward_kinematics([0]*7,full_kinematics=True)[1][:3,3][:2]
    r=math.hypot(dx-float(o1[0]),dy-float(o1[1]))
    hits=[]
    for h in range(0,PEG_H_MAX+1):
        z=peg_z(h)
        ring=reach_ring(z)
        if ring is not None and not (ring[0]-0.02<=r<=ring[1]+0.02):
            continue                     # 그 높이 수평거리 밖 - IK 생략
        if ik_tool(l2w((dx,dy,z)),seeds,q0[4],q0[GRIP_IDX]) is not None:
            hits.append(h)
    if not hits: return ((dx,dy),None)
    spans=[]; a=p=hits[0]
    for h in hits[1:]:
        if h==p+1: p=h; continue
        spans.append((a,p)); a=p=h
    spans.append((a,p))
    return ((dx,dy),spans)

def peg_table_key():
    # 기하(URDF)에만 의존. 로봇 위치/회전/플레이트/설비 배치와 무관.
    return (BASE_URDF,int(PEG_H_MAX))

def _rel_cells():
    """표를 만들 로컬 변위 격자: (10+20i, 10+20j)mm, 도달 한계 반경 안."""
    o1=CHAIN.forward_kinematics([0]*7,full_kinematics=True)[1][:3,3][:2]
    rmax=0.0
    for h in range(0,PEG_H_MAX+1,5):
        ring=reach_ring(peg_z(h))
        if ring is not None: rmax=max(rmax,ring[1])
    rmax+=0.04
    cells=[]
    i0=int(math.floor((float(o1[0])-rmax-0.01)/0.02))
    i1=int(math.ceil ((float(o1[0])+rmax+0.01)/0.02))
    j0=int(math.floor((float(o1[1])-rmax-0.01)/0.02))
    j1=int(math.ceil ((float(o1[1])+rmax+0.01)/0.02))
    for i in range(i0,i1+1):
        for j in range(j0,j1+1):
            dx,dy=0.01+i*0.02, 0.01+j*0.02
            if math.hypot(dx-float(o1[0]),dy-float(o1[1]))<=rmax:
                cells.append((round(dx,4),round(dy,4)))
    return cells

def build_peg_table():
    """로봇 기준 상대 도달표 (URDF 당 한 번만 계산하면 된다)."""
    key=peg_table_key()
    jobs=_rel_cells()
    t0=time.time()
    try:
        import multiprocessing as _mp
        ctx=_mp.get_context('fork')
        with ctx.Pool(_worker_count()) as pool:
            res=pool.map(_rel_span_one,jobs,chunksize=4)
    except Exception as e:
        print(f"[목심표] 병렬 실패({type(e).__name__}) - 순차")
        res=[_rel_span_one(j) for j in jobs]
    span={(int(round(d[0]*1000)),int(round(d[1]*1000))):v
          for d,v in res if v}
    PEG_TABLE['key']=key; PEG_TABLE['span']=span
    el=time.time()-t0
    print(f"[목심표] 상대격자 {len(span)}/{len(jobs)}칸 도달 가능, {el:.1f}초")
    try:                                   # 다음 실행에서 바로 쓰도록 저장
        with open(peg_table_file(),'w') as f:
            json.dump({'rel':True,'key':[key[0],key[1]],
                       'max_h':PEG_H_MAX,'pitch':PEG_PITCH,
                       'built':time.strftime('%Y-%m-%d %H:%M:%S'),
                       'span':{f"{k[0]},{k[1]}":[list(s) for s in sp]
                               for k,sp in span.items()}},f)
        print(f"[목심표] 저장: {peg_table_file()}")
    except Exception as e:
        print(f"[목심표] 저장 실패: {e}")
    return span,el

def peg_table_file():
    return os.path.join(URDF_PKG_DIR,'peg_table.json')

def load_peg_table():
    """미리 만들어 둔 상대 도달표를 불러온다. URDF 가 다르면 무시.
    (구버전 = 월드 목심 이름 형식 파일도 무시하고 새로 만든다)"""
    p=peg_table_file()
    if not os.path.exists(p): return False
    try:
        with open(p) as f: d=json.load(f)
        k=peg_table_key()
        if not d.get('rel'):
            print("  (peg_table.json 이 구형식(월드 좌표) — 상대표로 새로 만듭니다)")
            return False
        saved=d.get('key') or []
        if len(saved)<1 or saved[0]!=k[0]:
            print("  (peg_table.json 은 다른 URDF 의 것이라 무시)")
            return False
        span={}
        for ks,sp in d['span'].items():
            xs,ys=ks.split(',')
            span[(int(xs),int(ys))]=[tuple(s) for s in sp]
        PEG_TABLE['span']=span
        globals()['PEG_H_MAX']=int(d.get('max_h',PEG_H_MAX))
        PEG_TABLE['key']=peg_table_key()
        print(f"목심 도달표(상대) 불러옴: {len(span)}칸 "
              f"(생성 {d.get('built','?')}) — 로봇을 옮겨도 재계산 없음")
        return True
    except Exception as e:
        print(f"  (peg_table.json 읽기 실패: {e})")
        return False

def peg_table_ready():
    return PEG_TABLE['key']==peg_table_key() and PEG_TABLE['span']

def _peg_rel_key(x,y):
    """월드 (x,y) -> 로봇 로컬 변위 격자 키 (mm 정수)."""
    d=w2l(np.array([x,y,0.0]))
    return (int(round(d[0]*1000)),int(round(d[1]*1000)))

def peg_reachable_fast(height_mm):
    """상대표 조회 (IK 없음, 로봇 위치/회전 반영). 표가 없으면 None."""
    if not peg_table_ready(): return None
    h=int(round(float(height_mm)))
    memo=('fast',h,round(BASE_OFF[0],4),round(BASE_OFF[1],4),
          round(BASE_YAW[0],4),tuple(sorted(PLATES.keys())))
    if memo in _PEG_CACHE: return _PEG_CACHE[memo]
    out=[]
    for nm in peg_names():
        xy=peg_xy(nm)
        if not xy: continue
        sp=PEG_TABLE['span'].get(_peg_rel_key(xy[0],xy[1]))
        if sp and any(a<=h<=b for a,b in sp): out.append(nm)
    out.sort(); _PEG_CACHE[memo]=out
    return out

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
        jobs.append((nm,xy[0],xy[1],z,seeds,q0[4],q0[GRIP_IDX]))
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
    tgt=ik_tool((xy[0],xy[1],peg_z(height_mm)),_seed_list(q0),q0[4],q0[GRIP_IDX])
    if tgt is None: return
    tgt[GRIP_IDX]=q0[GRIP_IDX]
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
    # 구형처럼 '이동 중에는 모터를 읽지 않는다'.
    #   ROBOT.read() 는 250ms 나 걸려(모터 5개를 하나씩) 그 사이 팔이 멈춘다.
    #   3D(CURRENT_Q)가 직전 목표를 이미 알고 있으므로 그걸 쓴다.
    q0=list(CURRENT_Q)
    tgt=ik_tool((xy[0],xy[1],z),_seed_list(q0),q0[4],q0[GRIP_IDX])
    if tgt is None:
        return (f"{name} 의 {height_mm}mm 높이는 도달 불가 - "
                "높이를 올리거나 다른 목심을 고르세요")
    if not ROBOT.connect():
        return f"로봇 연결 실패 - [이동] 탭에서 '시뮬레이션' 을 고르면 3D로 볼 수 있습니다"
    STOP.clear(); RUNNING.set()
    try:
        ROBOT.time_profile(True); ROBOT.torque(True)
        tgt[GRIP_IDX]=q0[GRIP_IDX]                          # 그리퍼 상태 유지
        if LEVEL_ALWAYS: tgt[J5_IDX]=q0[J5_IDX]
        way,dt=plan_joint_seq(q0,tgt)
        for f in way: f[GRIP_IDX]=q0[GRIP_IDX]
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
    q=ik_tool((xy[0],xy[1],peg_z(height_mm)),_seed_list(q0),q0[4],q0[GRIP_IDX])
    if q is None:
        return gr.update(),gr.update(),f"{name} 의 {height_mm}mm 는 도달 불가"
    q[GRIP_IDX]=q0[GRIP_IDX]
    if LEVEL_ALWAYS: q[J5_IDX]=q0[J5_IDX]
    nm=(newname or '').strip() or f"{name}@{int(height_mm)}mm"
    base=nm; k=2
    while nm in pose_names(): nm=f"{base}_{k}"; k+=1
    with LOCK:
        S['poses'].append({'name':nm,'angles':list(q),'offset':[0.0]*NJOINT})
    _macro_save()
    ros_publish(q)                      # 등록한 자세를 바로 3D 로 보여줌
    return (gr.update(choices=pose_names(),value=nm),
            gr.update(choices=pose_names(),value=nm),
            f"자세 '{nm}' 등록 — [이동] 탭에서 파이프라인 블록으로 쓸 수 있습니다")

def arm_move_start(L):
    """3D 에서 팔을 우클릭 → '이 팔 위치 옮기기' (11-03, 사용자). 그 팔로 바꾸고 로봇 이동 모드를 켠다.
    다른 팔(B)을 옮겼으면, 다 옮기거나 끄면 A팔로 돌아온다 — 안 그러면 팔 블록 없는 프로그램을 실물로 돌릴 때 B팔이 움직인다."""
    if RUNNING.is_set(): return False,'동작 실행 중 - [정지] 후 다시'
    L=str(L).strip().upper()
    if L not in arms_cfg(): return False,f'설정에 없는 팔: {L}'
    if L!=ACTIVE_ARM[0]: arm_switch(L)
    _,msg=h_robot_move_mode(True)
    ROBOT_MOVE['back']='A' if L!='A' else None
    msg=f"{arm_label(L)} 위치 옮기기 — "+msg.split('— ',1)[-1]
    LOG(msg); return True,msg
def _arm_move_back():
    L=ROBOT_MOVE.pop('back',None)
    if L and ACTIVE_ARM[0]!=L: arm_switch(L)

def h_robot_move_mode(on):
    if not on: _arm_move_back()             # 우클릭으로 B팔을 옮기다 끄면 A팔로 (11-03)
    ROBOT_MOVE['on']=bool(on); rmove_reset(); emove_reset()
    if on: PLATE_EDIT['on']=False          # 두 클릭 모드 동시 금지
    msg=('로봇 이동 모드 ON — ① 로봇의 주황 핀 클릭 ② 꽂을 목심 클릭 '
         '③ 남은 핀이 갈 목심(하늘색) 클릭' if on else '로봇 이동 모드 OFF')
    return _robot_pos_str(), msg

def h_robot_home():
    if RUNNING.is_set(): return _robot_pos_str(),"동작 실행 중 - [정지] 후 다시"
    set_robot_pose(0.0,0.0,0.0)
    return _robot_pos_str(), "로봇 위치 원점 복귀 (회전 0)"
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
#   ① 물체 중심축(axis_ppap)과 툴 중심축(axis_7/cr_7)이 x-y 에서 일치
#   ② cr_7 과 cr_ppap 의 z 수직 거리 = 물체마다 정해진 값 (PPAP 는 36.37mm)
# !! cr_ppap 은 펜 '바닥'이 아니라 메쉬 원점(SolidWorks 기준 원)이다.
#    PPAP 은 원점이 바닥에서 48.5mm 위에 있으므로, 바닥 기준으로 환산하면
#    잡는 높이 = 48.5 + 37 = 85.5mm 다. (예전에 바닥+37 로 잘못 굳혀서
#    그리퍼가 47.9mm 나 더 내려가 도킹/펜을 파고드는 사고가 났었다)
# 원통이라 회전 방향은 무관하고, 수평 유지는 기존 IK 가 이미 보장한다.
BRICK_DIR='/home/drboom/2026_project/urdf/bricks'
# cr 실측은 36.37mm 지만 37mm 로 고정한다.
# 정확히 맞추면 오차가 아래로 날 때 그리퍼가 물체를 파고들어 이상하게 잡힌다
# -> 0.63mm 여유를 위로 두는 편이 안전 (사용자 결정)
GRASP_DZ_DEFAULT=0.037            # 잡는 면 -> cr_7 거리 (실측 36.67 -> 37 고정)
# ===== 하강/잡기 높이 (사용자 고정 치수 — 2026-08-14 확정) =====
#   원점(플레이트 최하단) -> 도킹 윗면        = 115mm (반드시)
#   도킹 윗면 -> 그리퍼가 닿는 펜 면          = +10mm (정확히 1cm)
#   그 면 -> cr_7(그리퍼 기준점)              = +37mm
#   => 도킹에 결합된 펜을 잡을 때 cr_7 높이 = 115+10+37 = 162mm  (고정)
GRASP_FACE_DZ=0.010               # 도킹 윗면 -> 그리퍼 닿는 면

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
                          'cx':float(v[:,0].mean()),   # 메쉬 XY 중심 (렌더용)
                          'cy':float(v[:,1].mean()),
                          # 잡는 높이(바닥 기준) = 기준 원(cr=메쉬 원점) 높이 + 37mm
                          'grasp_dz':float(-b[0][2])+GRASP_DZ_DEFAULT,
                          'pos':None}                  # 아직 배치 안 됨
            MESH_REG[name]=(cands[0],1.0)              # 3D 뷰/썸네일 제공용
            print(f"물체 등록: {name}  지름 {rad*2*1000:.1f}mm "
                  f"높이 {(b[1][2]-b[0][2])*1000:.1f}mm  "
                  f"잡는 높이 바닥+{(float(-b[0][2])+GRASP_DZ_DEFAULT)*1000:.2f}mm "
                  f"(기준 원 {-b[0][2]*1000:.1f} + 37)")
        except Exception as e:
            print(f"물체 로드 실패 {pkg}: {e}")
_load_bricks()
# 저장된 물체 배치 복원 (재시작해도 물체가 그 자리에 보이게)
try:
    for _bid,_bp in (S.get('brick_pos') or {}).items():
        if _bid not in BRICKS and '#' in _bid:         # 복제본 (ppap#2 등)
            _bbase=_bid.split('#')[0]
            if _bbase in BRICKS:
                BRICKS[_bid]=dict(BRICKS[_bbase])
                MESH_REG[_bid]=MESH_REG[_bbase]
        if _bid in BRICKS and _bp:
            BRICKS[_bid]['pos']=(float(_bp[0]),float(_bp[1]),float(_bp[2]))
            print(f"물체 배치 복원: {_bid} ({_bp[0]*1000:.0f},{_bp[1]*1000:.0f})mm")
except Exception as _e:
    print(f"물체 배치 복원 실패: {_e}")

# ===== 시작: 마지막 배치 = 시작 배치 하나 (11-06) =====
# 위에서 복원한 마지막 배치를 시작 배치 이름으로 저장해 두고, 화면은 잠깐 플레이트만으로 비운 뒤
# 모듈 끝에서 그 시작 배치를 불러온다 (불러오기 절차를 거쳐야 결합·이름·색이 모두 맞는다).
try:
    # '의미 있는 배치'가 있었을 때만 보존한다. (초기화 상태에도 env_off 에
    # 0 값들이 남아 있어서, 단순 존재 여부로 보면 매 부팅마다 빈 배치가
    # '이전 배치'를 덮어쓰는 버그가 있었다)
    _had=(bool(S.get('env_clones'))
          or bool(S.get('brick_pos'))
          or any(e.get('visible',True) and not _is_floor_plate(e['id'])
                 for e in ENV_SHOW)
          or abs(BASE_OFF[0])+abs(BASE_OFF[1])>1e-6)
    # ★ 배치는 하나만 (11-06 사용자: "배치 여러 개 안 하고 저거 하나 쓸 거다, '이전 배치' 같은 거 치워라") ★
    #   껐다 켜도 마지막 화면 그대로 — 직전 배치를 시작 배치(esw_contest_drboom)에 덮어쓰고, 모듈 끝에서 그걸 불러온다.
    #   '이전 배치' 프리셋은 더 만들지 않고, 남아 있던 것도 지운다.
    _one=os.environ.get('ARM_START_LAYOUT','esw_contest_drboom')
    _fp=S.setdefault('factory_presets',{})
    _fp.pop('이전 배치',None)
    if _had and _one:
        _fp[_one]=factory_snapshot()
    _factory_apply(factory_default())
    _macro_save()
    print("공장 초기화: 플레이트만 표시"
          +(f" (마지막 배치를 '{_one}' 에 그대로 저장 — 배치는 하나)" if (_had and _one) else "")
          +" — 시작 배치는 모듈 끝에서 불러온다")
except Exception as _e:
    print(f"공장 초기화 실패: {_e}")

def brick_grasp_point(bid):
    """그 물체를 잡을 때 axis_7(cr_7, 툴 중심)이 놓여야 할 월드 좌표.

    도킹에 결합된 물체는 사용자 고정 치수로 계산한다:
      cr_7 = 도킹 윗면(115) + 잡는 면(10) + cr 거리(37) = 162mm
    도킹 밖에 서 있는 물체만 바닥+grasp_dz 로 계산한다."""
    b=BRICKS.get(bid)
    if not b or b.get('pos') is None: return None
    x,y,z_bottom=b['pos']
    for _dk in docking_ids():
        t=dock_socket_world(_dk)
        if t and math.hypot(float(t[0])-x,float(t[1])-y)<0.025:
            dt=docking_top(_dk)
            if dt is not None:
                return np.array([x,y,float(dt[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT])
    return np.array([x,y,z_bottom+b['grasp_dz']])

def place_brick(bid,x,y,z_bottom):
    """물체를 (x,y) 에 세우고 바닥 높이를 z_bottom 으로 둔다.
    배치는 프로젝트에 저장 -> 재시작해도 그 자리에 그대로 (영구).
    단, 미리보기(FORCE_SIM) 중에는 화면상 이동만 하고 저장하지 않는다."""
    if bid not in BRICKS: return False
    BRICKS[bid]['pos']=(float(x),float(y),float(z_bottom))
    if FORCE_SIM[0]:
        return True
    with LOCK:
        S.setdefault('brick_pos',{})[bid]=[float(x),float(y),float(z_bottom)]
    try: _macro_save()
    except Exception: pass
    return True

def docking_ids():
    """환경 STL 중 도킹 스테이션 목록."""
    return [e['id'] for e in ENV_SHOW if 'docking' in e['id'].lower()]

_DTOP_CACHE={}
def docking_top(did):
    """도킹 스테이션의 (중심 x, 중심 y, 윗면 z). 물체를 세울 자리.
    메쉬 통계는 파일별 캐시 (폴링 중 매번 STL 을 읽지 않게)."""
    if did not in MESH_REG: return None
    path,scale=MESH_REG[did]
    st=_DTOP_CACHE.get((path,scale))
    if st is None:
        try: mm=trimesh.load(path)
        except Exception: return None
        v=np.asarray(mm.vertices)*scale
        st=(float(v[:,0].mean()),float(v[:,1].mean()),float(v[:,2].max()))
        _DTOP_CACHE[(path,scale)]=st
    off=np.zeros(3); yaw=0.0
    for e in ENV_SHOW:
        if e['id']==did:
            off=np.asarray(e.get('offset',(0,0,0)),float)
            yaw=float(e.get('yaw',0.0))
    cx,cy=_rz2(yaw)@np.array([st[0],st[1]])
    return (float(cx+off[0]), float(cy+off[1]), float(st[2]+off[2]))

_DOCK_MATE_CACHE={'v':-1}
def _dock_mate():
    """pick_dock_mate.py 로 정한 메이트(소켓 축 + 꽂힘 깊이). 없으면 None."""
    if _DOCK_MATE_CACHE['v']==-1:
        try:
            from dock_mate_fixed import DOCK_MATE as _dm
            _DOCK_MATE_CACHE['v']=dict(_dm)
            print(f"도킹 메이트 로드: 소켓 {_dm['socket_xy_mm']}mm, "
                  f"꽂힘 깊이 {_dm['seat_dz_mm']}mm <- dock_mate_fixed.py")
        except ImportError:
            _DOCK_MATE_CACHE['v']=None
    return _DOCK_MATE_CACHE['v']

def dock_socket_world(did):
    """도킹의 '소켓 축' 월드 좌표 (x,y,윗면z). 메이트가 있으면 소켓,
    없으면 근사(중심). 도킹의 위치/회전 반영."""
    t=docking_top(did)
    if t is None: return None
    m=_dock_mate()
    if m:
        e=_env_ent(did)
        _,scale=MESH_REG[did]
        off=np.asarray(e.get('offset',(0,0,0)),float) if e else np.zeros(3)
        yaw=float(e.get('yaw',0.0)) if e else 0.0
        s=_rz2(yaw)@(np.asarray(m['socket_xy_mm'],float)*scale)
        return (float(s[0]+off[0]), float(s[1]+off[1]), float(t[2]))
    return t

# ===== 사물 중심 이동 (@goto:) — 자세 등록 없이 즉석 IK =====
_GOTO_Q={}
def goto_point(oid):
    """@goto 대상의 접근점: 물체=잡는 점 위 여유높이, 도킹=소켓 위,
    일반 설비=윗면 중심 위. 반환 (x,y,z) 또는 None."""
    oid=str(oid)
    if _clone_base(oid) in BRICKS or oid in BRICKS:
        b=BRICKS.get(oid) or BRICKS.get(_clone_base(oid))
        p=brick_grasp_point(oid)
        if p is None or b is None: return None
        # 펜 위로 가는 접근점은 '펜 꼭대기 + 2cm' (몸통을 스치지 않게)
        top=float(b['pos'][2])+float(b.get('h',0.0))
        return (float(p[0]),float(p[1]),
                max(float(p[2])+GRASP_LIFT,top+0.02))
    if 'docking' in oid.lower():
        t=dock_socket_world(oid)
        if t is None: return None
        z=float(t[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT+GRASP_LIFT   # (115+10+37)+여유
        # 그 도킹에 물체가 꽂혀 있으면 그 꼭대기도 넘어야 한다
        for _ob,_b2 in BRICKS.items():
            _bp=_b2.get('pos')
            if _bp and math.hypot(_bp[0]-t[0],_bp[1]-t[1])<0.025:
                z=max(z,float(_bp[2])+float(_b2.get('h',0.0))+0.02)
        return (float(t[0]),float(t[1]),z)
    t=docking_top(oid)                    # 일반 설비: 윗면 중심
    if t is None: return None
    return (float(t[0]),float(t[1]),float(t[2])+GRASP_LIFT)

def goto_q(oid):
    """@goto 접근점의 관절해 (수평 유지 IK). 사물/로봇이 움직이면 자동 재계산."""
    p=goto_point(oid)
    if p is None: return None
    key=(tuple(np.round(p,4)),
         round(float(BASE_OFF[0]),4),round(float(BASE_OFF[1]),4),
         round(float(BASE_YAW[0]),4))
    hit=_GOTO_Q.get(oid)
    if hit and hit[0]==key: return list(hit[1])
    q0=list(CURRENT_Q)
    q=ik_tool(p,_seed_list(q0),q0[4],q0[GRIP_IDX])
    if q is None: return None
    _GOTO_Q[oid]=(key,list(q))
    return list(q)

def do_descend_here():
    """하강 블록: 현재 XY 를 유지한 채 '그 아래 잡을 수 있는 높이'까지
    수직으로만 내려간다 (물컵 규칙 유지, _descend 재사용).
    아래에 뭐가 있는지 순서대로 찾는다: 펜 축(25mm) > 도킹 소켓 > 플레이트."""
    # 구형처럼 '이동 중에는 모터를 읽지 않는다'.
    #   ROBOT.read() 는 250ms 나 걸려(모터 5개를 하나씩) 그 사이 팔이 멈춘다.
    #   3D(CURRENT_Q)가 직전 목표를 이미 알고 있으므로 그걸 쓴다.
    q0=list(CURRENT_Q)
    now=tool_pos(q0)
    x,y,z=float(now[0]),float(now[1]),float(now[2])
    tz=None; what='플레이트'
    for _ob,_b in BRICKS.items():
        _bp=_b.get('pos')
        if (_bp and HELD.get('brick')!=_ob
                and math.hypot(_bp[0]-x,_bp[1]-y)<0.025):
            tz=float(brick_grasp_point(_ob)[2])   # 도킹 결합이면 고정식 162mm
            what=f'{_ob} 잡는 높이'; break
    if tz is None:
        for _dk in docking_ids():
            _e=_env_ent(_dk)
            if _e is not None and not _e.get('visible',True): continue
            _t=dock_socket_world(_dk)
            if _t and math.hypot(_t[0]-x,_t[1]-y)<0.025:
                _dt=docking_top(_dk)
                tz=float(_dt[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT   # 115+10+37
                what=f'{disp_name(_dk)} 잡는 높이'; break
    if tz is None:
        tz=PLATE_TOP+GRASP_DZ_DEFAULT; what='플레이트 위 잡는 높이'
    if tz>=z-1e-4:
        return f"하강: 이미 목표 높이입니다 (지금 z={z*1000:.0f}mm)"
    e=_descend(x,y,z,tz,'하강')
    if e:
        return (f"{e} (목표: {what} z={tz*1000:.0f}mm) — 로봇 자리에서 팔이 "
                "그 깊이까지 안 닿습니다. [로봇 자리 추천]을 눌러보세요")
    return f"하강: {what} z={tz*1000:.0f}mm 까지 수직 완료"

def _clear_z():
    """지금 세계에서 수평 이동이 안전한 최소 고도 (cr_7 기준).
    = 모든 보이는 물체/도킹 꼭대기 + 2cm, 물체를 들고 있으면 그 늘어진
    길이(잡는 점~바닥)만큼 더 올린다."""
    hz=PLATE_TOP
    for b,v in BRICKS.items():
        if v.get('pos') is not None and HELD.get('brick')!=b:
            hz=max(hz,float(v['pos'][2])+float(v.get('h',0.0)))
    for dk in docking_ids():
        e=_env_ent(dk)
        if e is not None and not e.get('visible',True): continue
        t=docking_top(dk)
        if t: hz=max(hz,float(t[2]))
    hang=0.0
    hb=HELD.get('brick')
    if hb in BRICKS: hang=float(BRICKS[hb].get('grasp_dz',GRASP_DZ_DEFAULT))
    return hz+0.02+hang

SAME_SPOT_XY=0.003      # 이동 목표가 지금 자리와 이만큼 안이면 '같은 자리' — 움직이지 않는다 (11-04)
SAME_SPOT_Z=0.005
SLOT_TOL=0.003          # 같은 자리로 보는 XY 거리 (m)
#   11-04: 10mm -> 3mm. 도킹을 90° 씩 돌려 꽂으면 소켓이 다리 중심에서 7.5mm 떨어져 돌아서, 서로 다른 소켓 자리가
#   5mm 까지 가까워진다 (0° 와 한 칸 아래 180°). 같은 소켓이면 등록 좌표가 0.5mm 안으로 같으므로 3mm 로 충분.
SLOT_SAME_Z=0.005       # 높이가 이만큼 안이면 보정 각도를 그대로 쓴다 (m)
_POSE_XYZ=re.compile(r'_이동\((-?\d+),(-?\d+),(-?\d+)\)(?:_\d+)?$')

# ★ 보정 자리는 '로봇 기준' 좌표다 (2026-11-04, 사용자) ★
#   예전엔 이름 속 판 좌표(월드)로 자리를 찾아서, 로봇을 옮기면 (a) 로봇 기준 같은 자리의
#   도킹은 보정을 못 찾고 (b) 우연히 예전 판 자리에 있는 도킹이 엉뚱한 각도를 받았다.
#   보정의 실체(관절 각도)는 원래 로봇 몸에 붙은 값이므로, 자리 열쇠도 로봇 기준으로 둔다.
#     p['rel']  = 등록할 때의 이론 목표점을 로봇 기준으로 (m, [x,y,z])
#     p['arm']  = 그 보정을 잰 팔 ('A'/'B'). 팔마다 따로 쓴다.
#   지금 로봇 자리에서의 판 좌표는 l2w(rel) 로 매번 다시 구한다 → 로봇을 옮기거나 돌려도
#   '로봇 기준 X+2칸, Y+3칸' 의 보정은 그대로 따라간다. 이름 속 숫자는 등록 당시 판 좌표(표시용).
#   rel 이 없는 예전 자세: 지금 로봇 자리에서 각도와 이름이 맞으면(차이 ≤ SLOT_XY_MAX) 그 자리에서
#   등록한 것으로 보고 rel 을 붙여 준다(한 번). 안 맞으면 쓰지 않는다 (엉뚱한 각도 방지).
_SLOT_ADOPT={}                  # (자세이름, 로봇 자리) -> 판정 (예전 자세 rel 붙이기 시도 기록)

def _is_slot_pose(p):
    """자리 보정 자세인가 (로봇 기준 rel 이 있거나, 예전 이름 '…_이동(x,y,z)')."""
    return p.get('rel') is not None or bool(_POSE_XYZ.search(p['name']))

def _slot_name(arm,rel):
    """보정 자세 이름 = 팔 + 로봇 기준 좌표 (mm). 예: 'A팔 보정 (X+130, Y+2)'.
    로봇 몸에 붙은 좌표라 로봇을 어디로 옮기든 같은 점이면 같은 이름이다 (2026-11-04)."""
    return f"{arm}팔 보정 (X{float(rel[0])*1000:+.0f}, Y{float(rel[1])*1000:+.0f})"

def _slot_find(arm,rel,exclude=None):
    """그 팔의 로봇 기준 같은 자리(XY SLOT_TOL 안) 보정 자세. 여럿이면 가장 최근 것."""
    hit=None
    for p in S['poses']:
        r=p.get('rel')
        if r is None or p is exclude or p.get('arm','A')!=arm: continue
        if math.hypot(float(r[0])-float(rel[0]),float(r[1])-float(rel[1]))<=SLOT_TOL: hit=p
    return hit

def _rename_slot_poses():
    """rel 이 있는 보정 자세 이름을 '로봇 기준' 형식으로 맞추고, 같은 팔·같은 자리가 둘 이상이면
    가장 최근 것 하나만 남긴다 (시작할 때 한 번, 11-04)."""
    keep=[]; drop=[]
    for p in reversed(S['poses']):                # 뒤(최근)부터 — 같은 자리 먼저 본 것을 남긴다
        r=p.get('rel')
        if r is not None and any(q.get('rel') is not None and q.get('arm','A')==p.get('arm','A')
                                 and math.hypot(float(q['rel'][0])-float(r[0]),
                                                float(q['rel'][1])-float(r[1]))<=SLOT_TOL for q in keep):
            drop.append(p['name']); continue
        keep.append(p)
    keep.reverse()
    ren={}
    for p in keep:
        if p.get('rel') is None: continue
        new=_slot_name(p.get('arm','A'),p['rel'])
        if new!=p['name'] and not any(q['name']==new for q in keep if q is not p):
            ren[p['name']]=new; p['name']=new
    if not drop and not ren: return False
    if drop: print(f"[자세 정리] 같은 팔·같은 자리(3mm) 보정이 둘 이상이라 예전 것을 지움: {', '.join(drop)}")   # 11-06 기록
    S['poses']=keep
    def _r(n): return ren.get(n,n)
    S['seq']=[_r(x) if isinstance(x,str) else x for x in S.get('seq',[])]
    for k in ('moves','axorders','modes','rotmodes','simple_seg','ignore_obs','paths2d'):
        d=S.get(k)
        if isinstance(d,dict):
            S[k]={('=>'.join(_r(a) for a in kk.split('=>')) if isinstance(kk,str) else kk):v
                  for kk,v in d.items()}
    for a,b in ren.items(): print(f"[자리 보정] 이름 바꿈: {a} -> {b}")
    if drop: print(f"[자리 보정] 로봇 기준 같은 자리 중복 정리 (최근 것만 남김): {', '.join(drop)}")
    return True

def _pose_slot(p):
    """보정 자세 p 의 자리를 '지금 로봇 자리 기준 판 좌표'로 (px,py,pz). 지금 팔 것이 아니거나
    쓸 수 없으면 None."""
    L=ACTIVE_ARM[0] if 'ACTIVE_ARM' in globals() else 'A'
    rel=p.get('rel')
    if rel is not None:
        if p.get('arm','A')!=L: return None
        w=l2w(np.array([float(rel[0]),float(rel[1]),float(rel[2])]))
        return float(w[0]),float(w[1]),float(w[2])
    m=_POSE_XYZ.search(p['name'])
    if not m: return None
    if L!='A': return None                        # 예전 자세는 모두 A팔로 잰 것
    px,py,pz=(int(m.group(i))/1000.0 for i in (1,2,3))
    key=(p['name'],round(float(BASE_OFF[0]),4),round(float(BASE_OFF[1]),4),round(float(BASE_YAW[0]),4))
    ok=_SLOT_ADOPT.get(key)
    if ok is None:
        try:
            a=get_angles(p['name']); o=get_offset(p['name'])
            cal=[u+v for u,v in zip(list(a)[:NJOINT],(list(o)+[0.0]*NJOINT)[:NJOINT])]
            fk=tool_pos(cal)
            ok=math.hypot(float(fk[0])-px,float(fk[1])-py)<=SLOT_XY_MAX
        except Exception:
            ok=False
        _SLOT_ADOPT[key]=ok
        if ok:
            # 실물 전송 중에도 불리므로 LOCK·파일 저장은 여기서 하지 않는다 (다음 자동 저장 때 같이 저장된다.
            #   저장 전에 꺼져도 다음에 같은 규칙으로 다시 붙는다)
            r=w2l(np.array([px,py,pz]))
            p['rel']=[round(float(v),5) for v in r]; p['arm']='A'
            print(f"[자리 보정] 예전 자세 {p['name']} -> 로봇 기준 "
                  f"({r[0]*1000:+.0f},{r[1]*1000:+.0f},{r[2]*1000:.0f})mm 로 옮겨 적음")
        else:
            print(f"[자리 보정] 예전 자세 {p['name']}: 지금 로봇 자리에서 등록한 것이 아니라 안 씀 "
                  "(로봇을 등록 때 자리로 돌리면 로봇 기준으로 옮겨 적습니다)")
    return (px,py,pz) if ok else None

SLOT_ADOPT_TOL=0.030            # 예전 자세의 등록 당시 로봇 자리를 찾을 때 허용 오차 (= 보정 상한 SLOT_XY_MAX). 후보가 딱 하나여야 한다

def _adopt_legacy_slots():
    """rel 이 없는 예전 보정 자세에 '등록 당시 로봇 자리'를 찾아 로봇 기준 rel 을 붙인다 (시작할 때 한 번).
    후보 = 지금 자리 · 저장된 배치 프리셋들의 로봇 자리 · 원점. 보정 각도가 실제로 가리키는 점이
    이름의 판 좌표와 SLOT_ADOPT_TOL 안에 맞는 후보가 '딱 하나' 일 때만 붙인다 (애매하면 그대로 —
    그때는 로봇을 등록 때 자리에 두면 _pose_slot 이 붙인다)."""
    cands=[(float(BASE_OFF[0]),float(BASE_OFF[1]),float(BASE_YAW[0])),(0.0,0.0,0.0)]
    if S.get('robot_xy'):
        cands.append((float(S['robot_xy'][0]),float(S['robot_xy'][1]),float(S.get('robot_yaw') or 0.0)))
    for _pp in (S.get('factory_presets') or {}).values():
        r=(_pp or {}).get('robot')
        if r and len(r)>=3: cands.append((float(r[0]),float(r[1]),float(r[2])))
    uniq=[]
    for c in cands:
        if not any(abs(c[0]-u[0])<1e-4 and abs(c[1]-u[1])<1e-4
                   and abs(((c[2]-u[2]+math.pi)%(2*math.pi))-math.pi)<1e-3 for u in uniq):
            uniq.append(c)
    n=0
    for p in S['poses']:
        m=_POSE_XYZ.search(p['name'])
        if not m or p.get('rel') is not None: continue
        N=np.array([int(m.group(i))/1000.0 for i in (1,2,3)])
        try:
            a=get_angles(p['name']); o=get_offset(p['name'],'sts')
            cal=[u+v for u,v in zip(list(a)[:NJOINT],(list(o)+[0.0]*NJOINT)[:NJOINT])]
            loc=w2l(tool_pos(cal))                     # 보정 각도가 가리키는 점 (로봇 기준) — 로봇 자리와 무관
        except Exception:
            continue
        hits=[]; best=None
        for (bx,by,byaw) in uniq:
            c,si=math.cos(byaw),math.sin(byaw)
            wx=c*loc[0]-si*loc[1]+bx; wy=si*loc[0]+c*loc[1]+by
            r=math.hypot(wx-N[0],wy-N[1])
            if best is None or r<best[0]: best=(r,bx,by,byaw)
            if r<=SLOT_ADOPT_TOL: hits.append((bx,by,byaw))
        if len(hits)!=1:
            print(f"[자리 보정] 예전 자세 {p['name']}: 등록 당시 로봇 자리를 못 정함 (맞는 후보 {len(hits)}개, "
                  f"가장 가까운 후보 x={best[1]*1000:+.0f} y={best[2]*1000:+.0f} 회전={math.degrees(best[3]):.0f}도 "
                  f"오차 {best[0]*1000:.0f}mm) — 그대로 둠 (이 자세는 그 자리를 다시 보정해 주세요)")
            continue
        bx,by,byaw=hits[0]; c,si=math.cos(byaw),math.sin(byaw)
        dx,dy=N[0]-bx,N[1]-by
        p['rel']=[round(c*dx+si*dy,5),round(-si*dx+c*dy,5),round(float(N[2]),5)]; p['arm']='A'
        n+=1
        print(f"[자리 보정] 예전 자세 {p['name']} -> 로봇 기준 ({p['rel'][0]*1000:+.0f},{p['rel'][1]*1000:+.0f})mm "
              f"(등록 당시 로봇 x={bx*1000:+.0f} y={by*1000:+.0f} 회전={math.degrees(byaw):.0f}도)")
    return n

def _slot_pose(x,y):
    """판 위 XY 자리 (x,y) 의 '이동' 보정 자세. (이름, 관절값, 자리 좌표 m) 또는 None.
    자리 좌표는 로봇 기준 rel 을 지금 로봇 자리로 바꾼 값 (_pose_slot).
    같은 자리에 여럿이면 가장 최근에 등록한 것(목록 뒤쪽)."""
    hit=None
    for p in S['poses']:
        s=_pose_slot(p)
        if s is None: continue
        px,py,pz=s
        if math.hypot(px-x,py-y)>SLOT_TOL: continue
        a=get_angles(p['name'])
        if a is None: continue
        o=get_offset(p['name'])
        if any(abs(v)>1e-9 for v in o):
            o=level_offset(o) if LEVEL_ALWAYS else o
            a=[u+v for u,v in zip(list(a)[:NJOINT],list(o)[:NJOINT])]
        hit=(p['name'],list(a)[:NJOINT],(px,py,pz))
    return hit

try:                                            # 시작할 때 한 번: 예전 보정 자세 -> 로봇 기준 (11-04)
    _ch=_adopt_legacy_slots()
    # 보정량이 0 인 채 남은 자리 보정 자세 = 등록만 하고 미세조정 안 한 것 -> 보정 지도에서 아직(빨강)으로 보인다 (11-04, 표시만)
    for _p in S['poses']:
        if _p.get('rel') is None or 'calmap' in _p or _p.get('tuned'): continue
        try:
            _a=get_angles(_p['name']); _o=get_offset(_p['name'],'sts')
            _cal=[u+v for u,v in zip(list(_a)[:NJOINT],(list(_o)+[0.0]*NJOINT)[:NJOINT])]
            _fk=tool_pos(_cal); _w=l2w(np.asarray(_p['rel'],float))
            if math.hypot(float(_fk[0])-float(_w[0]),float(_fk[1])-float(_w[1]))<0.00005:
                _p['calmap']=True; _ch=True
                print(f"[자리 보정] {_p['name']}: 보정량 0 (미세조정 전) — 보정 지도에 '아직' 으로 표시 (저장하면 초록)")
        except Exception:
            pass
    if _rename_slot_poses() or _ch: _macro_save()
except Exception as _e:
    print('[자리 보정] 예전 자세 옮겨 적기 실패:',_e)

SLOT_FADE_XY=(0.005,0.010)   # 자리 중심에서 이 XY 거리(m) 사이에서 보정량이 1 -> 0
#   ★ 11-04: 30mm -> 10mm ★ 목심 간격이 20mm 라, 30mm 면 도킹을 한 칸 옮겼을 때 옆 칸의 보정이 40% 쯤
#   섞여 들어갔다 (보정한 적 없는 자리가 엉뚱하게 틀어짐). 이제 보정은 '그 자리' 에만, 안 한 칸은 이론값.
SLOT_FADE_Z=(0.020,0.080)    # 등록 높이보다 이만큼(m) 위로 갈수록 1 -> 0 (높이 이동 중엔 안 붙게)
SLOT_XY_MAX=0.030            # 이보다 큰 XY 보정은 보정이 아니라 잘못 저장된 것 -> 안 씀
_SLOT_XY={}                  # (자세이름, 각도) -> (dx,dy) 캐시
_SLOT_Q={}                   # (반올림 목표, j5, j6) -> 실물 관절값 캐시

def _slot_xy(p,px,py):
    """보정 자세가 실제로 가리키는 XY - 자리 XY = 그 자리의 XY 보정량 (m, 지금 판 좌표 방향)."""
    a=get_angles(p['name'])
    if a is None: return None
    o=get_offset(p['name'])
    cal=[u+v for u,v in zip(list(a)[:NJOINT],(list(o)+[0.0]*NJOINT)[:NJOINT])]
    key=(p['name'],tuple(round(v,6) for v in cal),        # 로봇을 돌리면 판 좌표 방향도 돈다 -> 자리도 열쇠에
         round(float(BASE_OFF[0]),4),round(float(BASE_OFF[1]),4),round(float(BASE_YAW[0]),4))
    hit=_SLOT_XY.get(key)
    if hit is None:
        fk=tool_pos(cal)
        hit=(float(fk[0])-px,float(fk[1])-py,float(fk[2]))
        _SLOT_XY[key]=hit
        print(f"[자리 보정] {p['name']}: 실물에만 XY {hit[0]*1000:+.1f},{hit[1]*1000:+.1f}mm "
              f"(높이는 쓰지 않음 — 저장 각도의 실제 높이 {hit[2]*1000:.1f}mm)")
    return hit

# ★ 물병 규칙 = '직전 보정 자세의 수평' (2026-11-06, 사용자) ★
#   손으로 보정해 저장한 자세의 그리퍼 기울기가 그 자리의 '수평' 이다. 그 다음 하강·잡기·놓기·상승은
#   그 기울기를 그대로 지키며 실물 기준으로 똑바로 움직여야 한다 (계산상 수평 j2+j3+j4=0 으로 되돌리면 안 된다).
#   B팔처럼 영점이 기운 팔은 계산상 수평 ≠ 실물 수평이라, 예전 방식(자리 XY 만 옮기고 계산상 수평으로 다시 풀기)은
#   보정 자리에서도 내려갈수록 기울고 수직이 아니었다.
#   -> 그 자리의 보정량 = '보정 자세 각도 − 그 자리의 이론 자세 각도' (관절 차이, 기울기 포함) 를 그대로 더한다.
#      실물 오차가 관절 영점 차이라면 이 값 하나로 모든 높이에서 정확히 수직·같은 기울기가 된다.
SLOT_JOINT_DELTA=True          # False = 예전 방식 (XY 만 옮기고 계산상 수평)
_SLOT_DQ={}                    # (자세이름, 보정각도, 로봇 자리) -> 관절 차이 (보정 − 이론)

def _off_for_send(name,off):
    """보정값을 실물로 보낼 때: 자리 보정 자세는 사용자가 맞춘 그대로 (j5 만 0 — 모터 없음), 그 밖(base 등)은
    예전처럼 계산상 수평(level_offset)."""
    rec=_pose_rec(name)
    if rec is not None and _is_slot_pose(rec) and LEVEL_ALWAYS:
        o=list(off); o[J5_IDX]=0.0
        return o
    return level_offset(off) if LEVEL_ALWAYS else list(off)

SLOT_DQ_MAX_DEG=25.0           # 11-06: 손으로 맞춘 보정이 이론 자세와 이만큼 넘게 다르면 다른 해로 보고 안 쓴다 (예전 15° — 말없이 버렸다)
_SLOT_LAST=[None]              # 마지막으로 기록에 알린 보정 자리 (같은 자리에서는 한 번만 알린다)

def _slot_dq(p,slot,q_hint=None):
    """보정 자세 p 의 '보정 − 이론' 관절 차이. slot = 그 자리의 이론 목표점 (월드).
    11-06: 이론 자세는 지금 트윈 자세(q_hint)에서 먼저 풀고(같은 해), 안 되면 보정 자세에서 푼다.
           못 쓰게 되면 말없이 버리지 않고 기록에 이유를 남긴다 (False 와 함께 이유를 캐시)."""
    a=get_angles(p['name'])
    if a is None: return None
    o=_off_for_send(p['name'],get_offset(p['name']))
    cal=[u+v for u,v in zip(list(a)[:NJOINT],(list(o)+[0.0]*NJOINT)[:NJOINT])]
    key=(p['name'],tuple(round(v,6) for v in cal),tuple(round(float(v),4) for v in slot),
         round(float(BASE_OFF[0]),4),round(float(BASE_OFF[1]),4),round(float(BASE_YAW[0]),4))
    hit=_SLOT_DQ.get(key)
    _newB=(p.get('arm','A')!='A')              # 11-06 (사용자: A팔에는 적용하지 마) 새 방식은 B팔만 — A팔은 예전 그대로
    if hit is None:
        why='그 자리의 이론 자세를 풀 수 없음'
        seeds=([list(q_hint)] if (_newB and q_hint is not None) else [])+[cal]
        lim=1e9 if _newB else 15.0                # 11-06 (사용자: 보정한 값을 그대로 써라) B팔은 크기와 상관없이 쓴다 — 큰 값은 아래에서 경고만
        for sd in seeds:
            th=ik_tool((float(slot[0]),float(slot[1]),float(slot[2])),_seed_list(sd),cal[4],cal[GRIP_IDX])
            if th is None: continue
            bo=_ik_off_base(list(th))                    # 실물 = 이론 + base 보정 + 이 값 -> 그 자리에서 정확히 보정 자세
            d=[u-v-w for u,v,w in zip(cal,list(th),list(bo)+[0.0]*NJOINT)]
            d[GRIP_IDX]=0.0
            d[J5_IDX]=0.0
            mx=max(abs(math.degrees(v)) for v in d[:4])
            if mx<=lim:
                hit=d; break
            why=f"이론 자세와 {mx:.1f}° 차이 (한도 {lim:.0f}°)"
        if not hit: hit=('x',why)
        if len(_SLOT_DQ)>2000: _SLOT_DQ.clear()
        _SLOT_DQ[key]=hit
        if isinstance(hit,list) and _newB and max(abs(math.degrees(v)) for v in hit[:4])>SLOT_DQ_MAX_DEG:
            LOG(f"  ⚠ [자리 보정] {p['name']}: 계산값과 {max(abs(math.degrees(v)) for v in hit[:4]):.1f}° 차이 — 손으로 맞춘 값이라 그대로 씁니다")
        if isinstance(hit,list):
            print(f"[자리 보정] {p['name']}: 관절 차이 "
                  +' '.join(f"j{j+1}:{math.degrees(hit[j]):+.2f}°" for j in range(4))
                  +f" (기울기 j2+j3+j4 {math.degrees(hit[1]+hit[2]+hit[3]):+.2f}° — 이 자리의 수평)")
        elif _newB:
            print(f"[자리 보정] {p['name']}: 못 씀 — {why}")
    return hit if isinstance(hit,list) else None

# ★ B팔 수직선 (11-06 사용자: "B팔 수직 하강 확인해봐, 수직이 아니라 밀지?") ★
#   보정 차이(관절 각도)를 높이와 상관없이 똑같이 더하면, 그 차이가 '관절 영점 오차' 가 아니라 자리 오차일 때
#   손끝이 내려가며 옆으로 밀린다 (실측 기록: 잡기 47mm 하강에 약 10mm, B팔 #2 j2 +13.5 j4 -20.5).
#   -> 보정 자세가 실제로 가리키는 점(팔 모형)을 지나는 수직선 위로, 그 자세의 기울기(j2+j3+j4)를 지키며 푼다.
#      보정한 높이에서는 손으로 맞춘 모터값 그대로 (보정 값은 절대 바꾸지 않는다), 위·아래는 그 점의 수직선.
#   풀이가 안 되면(한계 등) 예전처럼 보정 차이를 그대로 더한다.
SLOT_VERTICAL_B=True

def _slot_cal(p):
    """보정 자세의 실물 각도 (저장 각도 + 아직 저장 안 한 미세조정)."""
    a=get_angles(p['name'])
    if a is None: return None
    o=_off_for_send(p['name'],get_offset(p['name']))
    return [u+v for u,v in zip(list(a)[:NJOINT],(list(o)+[0.0]*NJOINT)[:NJOINT])]

def _tilt_ik(target,tilt,seed,j5,j6):
    """툴 중심이 target(월드)에 오고 j2+j3+j4=tilt 인 j1~j4 (뉴턴법, seed 근처 해). 안 되면 None."""
    tgt=np.asarray(target,float)
    def qf(x):
        q=[float(x[0]),float(x[1]),float(x[2]),float(tilt-x[1]-x[2])]+[0.0]*(NJOINT-4)
        q[J5_IDX]=j5; q[GRIP_IDX]=j6
        return q
    x=np.array([seed[0],seed[1],seed[2]],float)
    for _ in range(25):
        p0=np.asarray(tool_pos(qf(x)),float); r=p0-tgt
        if np.linalg.norm(r)<2e-6: break
        J=np.zeros((3,3))
        for i in range(3):
            d=np.zeros(3); d[i]=1e-5
            J[:,i]=(np.asarray(tool_pos(qf(x+d)),float)-p0)/1e-5
        try: x=x-np.linalg.solve(J,r)
        except Exception: return None
    q=qf(x)
    if float(np.linalg.norm(np.asarray(tool_pos(q),float)-tgt))>0.0005: return None
    for k in range(4):
        lo,hi=jr_limits(k)
        if not (lo-1e-6<=q[k]<=hi+1e-6): return None
    return q

_SLOT_VFAIL=[None]
GRASP_EXTRA_DEFAULT={'B':0.020}   # 11-06: 보정 자리에서 잡기·놓기 바닥을 3D 잡는 높이보다 더 내리는 양 (m). S['grasp_extra'] 로 바꿀 수 있다

def _slot_floor_z(sl):
    """보정 자리 아래 도킹의 잡는 높이 (소켓 + 10 + 37). 20mm 안에 보이는 도킹이 없으면 None."""
    best=None
    for d in docking_ids():
        e=_env_ent(d)
        if e is not None and not e.get('visible',True): continue
        t=dock_socket_world(d)
        if not t: continue
        dd=math.hypot(float(t[0])-float(sl[0]),float(t[1])-float(sl[1]))
        if dd<0.02 and (best is None or dd<best[0]):
            best=(dd,float(t[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT)
    return best[1] if best else None

def _slot_vertical_delta(p,sl,q,dq):
    """B팔: 보정 자세 점을 지나는 수직선 위 실물 자세 - (트윈 q + base 보정). 안 되면 None."""
    cal=_slot_cal(p)
    if cal is None: return None
    pc=np.asarray(tool_pos(cal),float)
    tilt=cal[1]+cal[2]+cal[3]
    z=float(np.asarray(tool_pos(list(q)),float)[2])
    # 11-06 (사용자: "끝까지 안 내려간다") 보정 자세는 위에서 XY 를 맞추다 높이도 몇 mm 바뀐다 (B #2: +8mm).
    #   그 높이 차이를 바닥까지 끌고 가면 잡는 높이(162)보다 8mm 위에서 멈춰 펜 위 허공을 잡았다.
    #   -> 보정 높이 이상: 보정 점 높이 차이 그대로 (보정 높이에선 저장 모터값과 정확히 같음)
    #      보정 높이 ~ 그 도킹의 잡는 높이(소켓+10+37): 높이 차이를 0 으로 줄여 바닥은 정확히 잡는 높이
    zs=float(sl[2]); dz=float(pc[2])-zs
    zf=_slot_floor_z(sl)
    if zf is not None and zs-zf>0.005:
        # 11-06 (사용자: "지금보다 20mm 더 내려가야 함") 팔마다 바닥을 더 내리는 양 — B팔 실물이 모형보다 약 20mm 높다
        ex=float((S.get('grasp_extra') or {}).get(p.get('arm','A'),GRASP_EXTRA_DEFAULT.get(p.get('arm','A'),0.0)))
        if z>=zs: zt=z+dz
        elif z>zf: zt=z+dz+(-ex-dz)*(zs-z)/(zs-zf)      # 보정 높이(높이차 dz) → 잡는 높이(-ex) 로 선형
        else: zt=z-ex
    else:
        zt=z+dz
    tgt=np.array([float(pc[0]),float(pc[1]),zt])
    bo=list(_ik_off_base(list(q)))+[0.0]*NJOINT
    seed=[a+b+c for a,b,c in zip(list(q),bo,list(dq)+[0.0]*NJOINT)]
    qr=_tilt_ik(tgt,tilt,seed,cal[J5_IDX],list(q)[GRIP_IDX])
    if qr is None:
        if _SLOT_VFAIL[0]!=p['name']:
            _SLOT_VFAIL[0]=p['name']
            LOG(f"  ⚠ [자리 보정] {p['name']}: 높이 {z*1000:.0f}mm 에서 수직선 자세를 못 풂 — 이 구간은 보정 차이를 그대로 더함")
        return None
    return [qr[j]-q[j]-bo[j] for j in range(4)]+[0.0]*(NJOINT-4)

def _slot_why(p,slot):
    """_slot_dq 가 못 쓴 이유 (캐시에서)."""
    for k,v in list(_SLOT_DQ.items()):
        if k[0]==p['name'] and isinstance(v,tuple): return v[1]
    return '알 수 없음'

def _slot_jdelta(q):
    """새 방식: 트윈 팔 끝이 보정 자리 위에 있으면 그 자리의 관절 차이를 (가까울수록 1, 5~10mm 에서 0 으로) 돌려준다.
    11-06: 보정을 쓸 때·못 쓸 때 모두 화면 기록에 한 번 알린다 (예전엔 말없이 빠져서 '저장이 안 됐나' 처럼 보였다)."""
    tp=tool_pos(list(q))
    best=None
    for p in S['poses']:
        sl=_pose_slot(p)
        if sl is None: continue
        d=math.hypot(float(tp[0])-sl[0],float(tp[1])-sl[1])
        if d<SLOT_FADE_XY[1] and (best is None or d<best[0]): best=(d,p,sl)
    if best is None:
        _SLOT_LAST[0]=None
        return None
    d,p,sl=best
    dq=_slot_dq(p,sl,q)
    _newB=(p.get('arm','A')!='A')              # 기록 알림도 B팔만 (A팔은 예전처럼 조용히)
    if dq is None:
        if _newB and _SLOT_LAST[0]!=('x',p['name']):
            _SLOT_LAST[0]=('x',p['name'])
            LOG(f"  ⚠ [자리 보정] {p['name']}: 저장된 보정을 못 씁니다 — {_slot_why(p,sl)}. "
                "이 자리는 보정 없이(계산값) 갑니다")
        return None
    a0,a1=SLOT_FADE_XY
    w=1.0 if d<=max(a0,SLOT_EXACT) else max(0.0,1.0-(d-a0)/(a1-a0))
    vd=None
    if _newB and SLOT_VERTICAL_B:                # 11-06: B팔은 보정 자세 점의 수직선 (밀림 없음)
        try:
            vd=_slot_vertical_delta(p,sl,q,dq)
        except Exception as e:
            print('[자리 보정] 수직선 계산 실패:',e)
    if _newB and w>=0.999 and _SLOT_LAST[0]!=p['name']:
        _SLOT_LAST[0]=p['name']
        LOG(f"  [자리 보정] {p['name']} 적용 — 손으로 맞춘 그대로 (그 점의 수직선으로 하강): "
            +' '.join(f"j{j+1} {math.degrees(dq[j]):+.1f}°" for j in range(4))
            +(f" · 그리퍼 수평 보정 {float((S.get('level_trim') or {}).get(ACTIVE_ARM[0],0.0)):+.1f}° 별도"
              if float((S.get('level_trim') or {}).get(ACTIVE_ARM[0],0.0)) else ''))
    return [w*v for v in (vd if vd is not None else dq)]

def _slot_delta(q):
    """실물에만 더할 '그 자리의 보정량' (관절 각도 차이). 해당 없으면 None.

    ★ 트윈은 이론, 실물은 이론+보정 (2026-10-20, 사용자) ★
      예전에는 보정 자세의 각도로 3D 까지 움직여서 트윈 속 펜이 도킹에서 어긋났다.
    ★ 보정은 XY 만 (높이는 팔이 알아서) ★
      보정 자세의 각도를 통째로 쓰면, 미세조정 중 높이까지 바뀐 채 저장된 경우
      (실측: #10 이 이름은 235mm 인데 실제 175.9mm) 펜 위로 가야 할 때 펜 속으로
      내려가 손목이 꺾였다. -> 그 자세에서 'XY 차이'만 뽑아,
      트윈의 팔 끝과 '같은 높이'에서 XY 만 옮긴 자리를 실물 목표로 푼다.
      트윈 팔 끝이 그 자리 위에 내려와 있을 때만 붙고, 멀어지면 0 으로 서서히 준다."""
    if ROBOT._sim(): return None
    if SLOT_JOINT_DELTA: return _slot_jdelta(q)      # 11-06: 보정 자세의 기울기를 지키는 새 방식
    # 11-04: 팔마다 자기 보정만 쓴다 (_pose_slot 이 p['arm'] 으로 거른다 — B팔은 B팔로 잰 보정만)
    tp=tool_pos(list(q))
    # ★ 높이로는 줄이지 않는다 (11-04) ★ 같은 XY 면 높이와 상관없이 같은 보정 -> 하강·상승은 늘 곧게.
    off=_slot_off_at(float(tp[0]),float(tp[1]))
    if off is None: return None
    tgt=(float(tp[0])+off[0],float(tp[1])+off[1],float(tp[2]))
    key=(tuple(round(v,4) for v in tgt),round(float(q[4]),4),round(float(q[GRIP_IDX]),4),
         round(float(BASE_OFF[0]),4),round(float(BASE_OFF[1]),4),round(float(BASE_YAW[0]),4))
    qr=_SLOT_Q.get(key)
    if qr is None:
        qr=ik_tool_fast(tgt,q[4],q[GRIP_IDX]) or ik_tool(tgt,_seed_list(list(q)),q[4],q[GRIP_IDX])
        qr=list(qr) if qr is not None else False
        if len(_SLOT_Q)>20000: _SLOT_Q.clear()
        _SLOT_Q[key]=qr
    if not qr: return None
    d=[u-v for u,v in zip(qr,list(q))]
    d[GRIP_IDX]=0.0
    return d

SLOT_INTERP_R=0.080     # 중간값에 쓰는 보정점 반경 (m)
SLOT_INTERP_K=8         # 가까운 보정점 몇 개까지
SLOT_EXACT=0.002        # 이 안이면 그 보정점 하나만 (손으로 맞춘 값 그대로)
SLOT_BBOX_TOL=0.015     # 보정점들이 둘러싼 범위 밖으로 이만큼까지 채운다 (판 가장자리 회전 소켓 7.5mm)
SLOT_ODD=0.008          # 주변 보정점 중앙값과 이만큼 넘게 다르면 '튀는 점' — 중간값 계산에서 뺀다
# ★ 11-05 (사용자): 보정 지도·보간 안 씀 — '등록한 자리만 손으로 맞춘 값 그대로' ★
#   False = 여러 보정점으로 중간값을 짐작하지 않는다 (아래 ② 건너뜀). 등록 안 한 자리는 이론값.
#   True 로 바꾸면 11-04 보간이 되살아난다 (데이터는 그대로 있다).
SLOT_INTERP_ON=False

def _slot_pts(display=False):
    """지금 팔·지금 로봇 자리 기준, 쓸 수 있는 보정점 [(px,py,ox,oy)] (o = 그 자리의 XY 보정량 m).
    display=True 는 보정 지도 표시용 — 찍기만 하고 [보정 저장] 안 한 자세는 빼서 초록으로 안 보이게 한다.
    (실제 움직임은 display=False — 11-04 사용자: 작동은 그대로, 지도만)"""
    out=[]
    for p in S['poses']:
        if display and p.get('calmap') and not p.get('tuned'): continue
        s=_pose_slot(p)
        if s is None: continue
        off=_slot_xy(p,s[0],s[1])
        if off is None or math.hypot(off[0],off[1])>SLOT_XY_MAX: continue
        out.append((s[0],s[1],float(off[0]),float(off[1])))
    return out

def _med(v):
    v=sorted(v); n=len(v)
    return v[n//2] if n%2 else 0.5*(v[n//2-1]+v[n//2])

def _slot_odd(pt,pts):
    """그 보정점이 주변(80mm 안, 자기 제외 3개 이상) 보정점들의 중앙값과 SLOT_ODD 넘게 다른가."""
    px,py,ox,oy=pt
    nb=[q for q in pts if q is not pt and 1e-9<math.hypot(q[0]-px,q[1]-py)<=SLOT_INTERP_R]
    if len(nb)<3: return False
    nb.sort(key=lambda q:math.hypot(q[0]-px,q[1]-py)); nb=nb[:SLOT_INTERP_K]
    return math.hypot(ox-_med([q[2] for q in nb]),oy-_med([q[3] for q in nb]))>SLOT_ODD

def _slot_off_at(x,y,pts=None):
    """판 위 (x,y) 에 줄 XY 보정량 (ox,oy) 또는 None.  (2026-11-04, 사용자: 막 보정할 테니 중간값을 잘 찾아라)
    보정량은 '팔이 판 위 어느 XY 로 가나' 의 함수라 도킹 회전과 상관없이 모든 보정점(0°·90°·180°·270° 어디서
    맞췄든)을 같이 쓴다.
      ① 보정점 바로 위(2mm)          -> 손으로 맞춘 값 그대로
      ② 가까운 보정점(80mm 안, 8개까지) 중 '튀는 점'(주변 중앙값과 8mm 넘게 다름)을 빼고 거리²반비례 평균.
         보정점들이 둘러싼 범위 밖 15mm 까지만 (그보다 바깥은 짐작하지 않음 -> 이론값)
      ③ 보정점 하나만 가까우면(10mm) -> 그 보정점 (SLOT_FADE_XY 로 서서히)"""
    pts=_slot_pts() if pts is None else pts
    near=[]
    for q in pts:
        d=math.hypot(x-q[0],y-q[1])
        if d<=SLOT_INTERP_R: near.append((d,q))
    if not near: return None
    near.sort(key=lambda n:n[0])
    d0,q0=near[0]
    if d0<=SLOT_EXACT: return (q0[2],q0[3])
    if not SLOT_INTERP_ON:                      # 11-05: 보간 끔 — 그 등록점 하나의 값만 (③, 가까울 때만 서서히)
        a0,a1=SLOT_FADE_XY
        if d0<a1:
            w=1.0 if d0<=a0 else 1.0-(d0-a0)/(a1-a0)
            return (q0[2]*w,q0[3]*w)
        return None
    cand=near[:SLOT_INTERP_K]
    if len(cand)>=3:
        mx=_med([n[1][2] for n in cand]); my=_med([n[1][3] for n in cand])
        keep=[n for n in cand if math.hypot(n[1][2]-mx,n[1][3]-my)<=SLOT_ODD]
    else:
        keep=cand
    if len(keep)>=2:
        xs=[n[1][0] for n in keep]; ys=[n[1][1] for n in keep]; tol=SLOT_BBOX_TOL
        if min(xs)-tol<=x<=max(xs)+tol and min(ys)-tol<=y<=max(ys)+tol:
            ws=[1.0/(n[0]**2) for n in keep]; W=sum(ws)
            return (sum(w*n[1][2] for w,n in zip(ws,keep))/W, sum(w*n[1][3] for w,n in zip(ws,keep))/W)
    a0,a1=SLOT_FADE_XY
    if d0<a1:
        w=1.0 if d0<=a0 else 1.0-(d0-a0)/(a1-a0)
        return (q0[2]*w,q0[3]*w)
    return None

# ===== 보정 지도 (2026-11-04, 사용자) =====
#   로봇 하나마다 보정할 영역 = 로봇이 선 판 기준  앞 판 · 대각 판 · 옆(X) 판 · 앞앞 판의 가까운 3줄.
#   그 안에서 도킹(회전 0°)을 꽂을 수 있는 자리마다 '소켓 점' 하나. 40mm(2칸) 간격 기준점만 손으로
#   보정하고 나머지는 _slot_off_at 보간. 3D 에 점으로 보인다:
#     초록 = 보정됨 · 빨강 = 보정할 기준점 · 노랑 = 보간으로 채워짐 · 회색 = 아직 못 채움
#   빨강/회색 점을 누르면 그 자리 보정 자세를 등록(또는 이미 있으면 염)한다 -> ③연동에서 미세조정·저장.
CALMAP={'on':False,'cut':0}
CALMAP_FAR_ROWS=6        # 앞앞 판(파랑)은 로봇 쪽 6줄까지 (11-04: 3 -> 6, 사용자)
CALMAP_NEAR_R=0.0       # 11-04 사용자: 안 되는 자리도 지우지 말고 보이기 -> 0 (전부 보라로 표시)
_CALMAP_REACH={}        # (로봇 자리, 팔, x, y) -> 닿는가

def _calmap_reach(x,y,ztop):
    """그 소켓 자리에 팔이 실제로 갈 수 있나 (물리적으로 되는 자리만 남긴다, 11-04).
    도킹 위 접근 높이와 펜을 꽂는 높이(162mm) 둘 다 수평 유지(물컵 규칙)로 풀려야 한다."""
    key=(round(float(BASE_OFF[0]),4),round(float(BASE_OFF[1]),4),round(float(BASE_YAW[0]),4),
         ACTIVE_ARM[0],round(x,4),round(y,4))
    hit=_CALMAP_REACH.get(key)
    if hit is not None: return hit
    # ★ 11-04: 도달표(08-12)로 미리 거르지 않는다 (사용자: 반경이 작다) ★ 도달표가 실제보다 좁아 닿는 자리까지
    #   뺐다. 지금 관절 한계로 '접근 높이 -> 펜 높이' 가 실제로 풀리는지(아래 뒤 검사)만으로 정한다.
    ok=True
    if ok:
        # ★ 도달표(08-12)는 그 뒤 바뀐 관절 한계를 모른다 (11-04 실측: (50,-477) 에서 j3 가 한계 2913 에 닿아
        #   잡기 하강이 z=178mm 에서 멈춤). 지금 한계로 '접근 높이 -> 펜 높이' 가 실제로 풀리는지 뒤에서 검사한다.
        #   검사 전(None)엔 보이고, 안 풀리면(False) 뺀다.
        r=_CALMAP_IK.get(key)
        if r is None:
            _CALMAP_IK_Q[key]=(x,y,ztop); _calmap_ik_kick()
        elif r is False:
            ok=False
    if len(_CALMAP_REACH)>50000: _CALMAP_REACH.clear()
    if ok is False or _CALMAP_IK.get(key) is not None: _CALMAP_REACH[key]=ok
    return ok

_CALMAP_IK={}           # key -> True/False (지금 관절 한계로 접근~펜 높이가 풀리나)
_CALMAP_IK_Q={}         # 검사 대기 key -> (x,y,ztop)
_CALMAP_IK_TH=[None]
def _calmap_ik_kick():
    th=_CALMAP_IK_TH[0]
    if th is not None and th.is_alive(): return
    def _work():
        while _CALMAP_IK_Q:
            if RUNNING.is_set():                  # 팔이 움직이는 중엔 쉰다 (전송 타이밍을 뺏지 않게)
                time.sleep(0.5); continue
            _it=list(_CALMAP_IK_Q.items())[:1]           # 화면 쪽이 동시에 넣어도 안전하게 (복사본에서 하나)
            if not _it: break
            key,(x,y,zt)=_it[0]
            ok=True
            try:
                q0=list(get_angles('base') or CURRENT_Q)
                za=zt+GRASP_FACE_DZ+GRASP_DZ_DEFAULT+GRASP_LIFT; zs=zt+GRASP_FACE_DZ+GRASP_DZ_DEFAULT
                qa=ik_tool((x,y,za),_seed_list(q0),q0[4],q0[GRIP_IDX])
                if qa is None: ok=False
                else:
                    qs=qa                         # 하강처럼 위에서부터 몇 걸음에 나눠 푼다 (같은 갈래 유지)
                    for k in range(1,5):
                        z=za+(zs-za)*k/4
                        qs=ik_tool((x,y,z),_seed_list(qs),qa[4],qa[GRIP_IDX])
                        if qs is None: ok=False; break
            except Exception:
                ok=True                           # 검사 실패는 빼지 않는다
            _CALMAP_IK[key]=ok
            _CALMAP_IK_Q.pop(key,None)
            _CALMAP_REACH.pop(key,None)
            time.sleep(0.02)                              # 화면·통신이 밀리지 않게 조금씩
    th=threading.Thread(target=_work,daemon=True); _CALMAP_IK_TH[0]=th; th.start()

def _calmap_dock_geom():
    """도킹(회전 0°) 다리 4개 로컬 좌표와 소켓 로컬 XY, 소켓 윗면 z. (m)"""
    for d in docking_ids():
        try:
            pins=env_pins_local(d)
            m=_dock_mate()
            if len(pins)>=4 and m:
                _,scale=MESH_REG[d]
                sock=np.asarray(m['socket_xy_mm'],float)*scale
                top=docking_top(d)
                return [np.asarray(q[:2],float) for q in pins], sock, (float(top[2]) if top else 0.115)
        except Exception:
            continue
    return None

_CALMAP_CACHE={'key':None,'pts':[]}
def calmap_points():
    """보정 지도 점 (바뀐 게 없으면 저장해 둔 결과)."""
    key=(round(float(BASE_OFF[0]),4),round(float(BASE_OFF[1]),4),round(float(BASE_YAW[0]),4),ACTIVE_ARM[0],
         len(_CALMAP_IK)//25, len(_CALMAP_IK_Q)==0, len(S.get('calmap_ok') or []),
         tuple(sorted(PLATES.keys())),
         tuple((p['name'],bool(p.get('tuned')),tuple(round(float(v),4) for v in (p.get('angles') or [])[:5]))
               for p in S['poses'] if p.get('rel') is not None))
    if _CALMAP_CACHE['key']!=key:
        _CALMAP_CACHE['pts']=_calmap_points_calc(); _CALMAP_CACHE['key']=key
    return _CALMAP_CACHE['pts']

def _calmap_points_calc():
    """보정 지도 점 [{x,y,z,kind,base}] (지금 팔 기준)."""
    g=_calmap_dock_geom()
    if g is None: return []
    pins,sock,ztop=g
    try:
        rp=[l2w(np.array([q[0],q[1],0.0])) for q in robot_pins_local()]
        cx=float(np.mean([r[0] for r in rp])); cy=float(np.mean([r[1] for r in rp]))
    except Exception:
        return []
    rt=(int(math.floor((cx-GRID_ORG[0])/PLATE_SIZE)),int(math.floor((cy-GRID_ORG[1])/PLATE_SIZE)))
    k=int(round(float(BASE_YAW[0])/(math.pi/2)))%4
    def rot(dx,dy):
        for _ in range(k): dx,dy=-dy,dx
        return dx,dy
    fdx,fdy=rot(0,1)                                # 로봇 앞 방향 (판 단위)
    tiles=[(rt[0]+a,rt[1]+b) for a,b in (rot(0,1),rot(1,1),rot(1,0))]
    far=(rt[0]+rot(0,2)[0],rt[1]+rot(0,2)[1])
    pegs=set()
    def tile_pegs(tl):
        ox,oy=tile_origin(tl)
        n=int(round(PLATE_SIZE/PEG_PITCH))
        return [(round(ox+PEG_PITCH/2+i*PEG_PITCH,4),round(oy+PEG_PITCH/2+j*PEG_PITCH,4))
                for i in range(n) for j in range(n)]
    for tl in tiles:
        if tl in PLATES: pegs.update(tile_pegs(tl))
    if far in PLATES:
        fp=tile_pegs(far)
        proj=lambda q:q[0]*fdx+q[1]*fdy
        lv=sorted({round(proj(q),4) for q in fp})[:CALMAP_FAR_ROWS]
        pegs.update(q for q in fp if round(proj(q),4) in lv)
    p0=min(pins,key=lambda v:(v[0],v[1]))
    # 다리 간격은 목심 칸 단위로 맞춘다 (STL 에서 읽은 다리 중심이 249.9 / 250.3mm 처럼 조금씩 어긋난다)
    rel=[(round(round(float(v[0]-p0[0])/PEG_PITCH)*PEG_PITCH,4),
          round(round(float(v[1]-p0[1])/PEG_PITCH)*PEG_PITCH,4)) for v in pins]
    soff=(float(sock[0]-p0[0]),float(sock[1]-p0[1]))
    cands=[]
    for (x,y) in pegs:
        if all((round(x+a,4),round(y+b,4)) in pegs for a,b in rel):
            cands.append((x,y))
    if not cands: return []
    # 기준점 격자는 영역 전체로 잡는다 (닿는 자리만으로 잡으면 격자가 들쭉날쭉해진다)
    xs=sorted({c[0] for c in cands}); ys=sorted({c[1] for c in cands})
    bx=set(xs[::2])|{xs[-1]}; by=set(ys[::2])|{ys[-1]}
    # ★ 회전 90°·180°·270° 도 (11-04, 사용자) ★ 다리 정사각형은 같은 목심 4개에 그대로 꽂히고,
    #   소켓만 다리 중심에서 7.5mm 떨어진 채 중심을 돈다 (0°=아래 · 90°=오른쪽 · 180°=위 · 270°=왼쪽).
    #   손으로 보정할 기준점은 0° 의 2칸 격자뿐 — 회전 자리는 그 사이에 들어가 보간으로 채워진다.
    cxr=max(a for a,b in rel)/2.0; cyr=max(b for a,b in rel)/2.0      # 다리 정사각형 중심 (p0 기준)
    vx,vy=soff[0]-cxr,soff[1]-cyr                                     # 중심 -> 소켓 (0°)
    pts=_slot_pts(display=True)
    good=_calmap_good_world()
    try:
        _l1=fk_points(list(get_angles('base') or CURRENT_Q))[1]; j1x,j1y=float(_l1[0]),float(_l1[1])
    except Exception:
        j1x,j1y=cx,cy
    out=[]; cut=0
    for rot in (0,90,180,270):
        c,s=math.cos(math.radians(rot)),math.sin(math.radians(rot))
        rx,ry=c*vx-s*vy,s*vx+c*vy
        for (x,y) in cands:
            sx,sy=x+cxr+rx,y+cyr+ry
            if not _calmap_reach(sx,sy,ztop):
                # 로봇 몸 가까이(팔꿈치 한계)는 실제로도 안 된다 -> 빼기. 먼 쪽은 사용자가 '닿는다' 고 해서 (11-04)
                #   보라 점으로 남긴다 (3D 모델 계산으로는 안 닿음 — 실물 확인 후 보정)
                if math.hypot(sx-j1x,sy-j1y)<CALMAP_NEAR_R: cut+=1; continue
                out.append({'x':round(sx,4),'y':round(sy,4),'z':round(ztop,4),'kind':'far','base':False,'rot':rot})
                continue
            base=(rot==0 and x in bx and y in by)
            done=any(math.hypot(sx-p[0],sy-p[1])<=SLOT_TOL for p in pts)
            if any(math.hypot(sx-g[0],sy-g[1])<=SLOT_TOL for g in good): kind='good'
            elif done:
                _me=min(pts,key=lambda q:math.hypot(sx-q[0],sy-q[1]))
                kind='odd' if _slot_odd(_me,pts) else 'ok'
            elif base: kind='need'
            else: kind='fill' if _slot_off_at(sx,sy,pts) is not None else 'none'
            out.append({'x':round(sx,4),'y':round(sy,4),'z':round(ztop,4),'kind':kind,'base':base,'rot':rot})
    CALMAP['cut']=cut
    return out

def _calmap_good_world():
    """'보정 성공' 으로 기록된 자리 (지금 팔·지금 로봇 자리 기준 판 좌표)."""
    out=[]
    for g in S.get('calmap_ok') or []:
        if g.get('arm','A')!=ACTIVE_ARM[0]: continue
        w=l2w(np.array([float(g['rel'][0]),float(g['rel'][1]),0.0]))
        out.append((float(w[0]),float(w[1])))
    return out

def h_calmap_ok():
    """✅ 보정 성공: 지금 3D 에 꽂혀 있는 도킹 자리들을 '실물로 해 보니 잘 됐다' 로 기록한다 (11-04, 사용자).
    로봇 기준 좌표로 남겨 로봇을 옮겨도 유지. 지도에 파란 점. 표시용 기록 — 움직임에는 쓰지 않는다."""
    lst=S.setdefault('calmap_ok',[])
    pts=_slot_pts()
    done=[]
    for d in docking_ids():
        e=_env_ent(d)
        if e is None or not e.get('visible',True): continue
        tt=dock_socket_world(d)
        if not tt: continue
        rel=w2l(np.array([float(tt[0]),float(tt[1]),0.0]))
        how=('보정점' if any(math.hypot(tt[0]-q[0],tt[1]-q[1])<=SLOT_TOL for q in pts)
             else ('자동 계산' if _slot_off_at(float(tt[0]),float(tt[1]),pts) is not None else '이론값'))
        old=[g for g in lst if g.get('arm','A')==ACTIVE_ARM[0]
             and math.hypot(float(g['rel'][0])-rel[0],float(g['rel'][1])-rel[1])<=SLOT_TOL]
        for g in old: lst.remove(g)
        lst.append({'arm':ACTIVE_ARM[0],'rel':[round(float(rel[0]),5),round(float(rel[1]),5)],
                    't':time.strftime('%Y-%m-%d %H:%M'),'how':how})
        done.append(f"{disp_name(d)} (X{rel[0]*1000:+.0f}, Y{rel[1]*1000:+.0f} · {how})")
    if not done: return '꽂혀 있는 도킹이 없습니다'
    _macro_save()
    _CALMAP_CACHE['key']=None
    print('[보정 성공] '+', '.join(done))
    return '✅ 보정 성공 기록: '+', '.join(done)+f" — 지금까지 성공 {sum(1 for g in lst if g.get('arm','A')==ACTIVE_ARM[0])}곳"

def h_calmap(on=None):
    """보정 지도 켜기/끄기 + 요약. on='status' 면 상태만."""
    if on!='status': CALMAP['on']=(not CALMAP['on']) if on in (None,'') else bool(on)
    if not CALMAP['on']: return '보정 지도 끔'
    pts=calmap_points()
    def cnt(rs):
        q=[p for p in pts if p['rot'] in rs]
        return {k:sum(1 for p in q if p['kind']==k) for k in ('ok','need','fill','none')},len(q)
    c0,n0=cnt((0,)); cr,nr=cnt((90,180,270))
    ng=sum(1 for p in pts if p['kind']=='good')
    nf=sum(1 for p in pts if p['kind']=='far')
    no=sum(1 for p in pts if p['kind']=='odd')
    nb=sum(1 for p in pts if p['base'])
    _pend=len(_CALMAP_IK_Q)
    return (f"보정 지도 켬 ({arm_label(ACTIVE_ARM[0])}) — 팔이 안 닿아 뺀 자리 {CALMAP['cut']}"
            +(f" (관절 한계 검사 중 {_pend}곳 남음 — 조금 뒤 [켜기/끄기] 두 번으로 갱신)" if _pend else "")+" | "
            f"✅ 성공 {ng}곳 · 🟠 주변과 튀는 보정 {no}곳(다시 확인) · 보라(모델상 안 닿음, 실물 확인) {nf}곳 · 기준점 {nb}곳 중 남은 빨강 {c0['need']} | "
            f"회전 0° {n0}자리: 초록 {c0['ok']} · 노랑 {c0['fill']} · 회색 {c0['none']} | "
            f"회전 90·180·270° {nr}자리: 초록 {cr['ok']} · 노랑 {cr['fill']} · 회색 {cr['none']}"
            " — 3D 에서 빨간 점을 누르면 그 자리 보정 자세가 등록됩니다")

def calmap_register(x,y):
    """보정 지도 점 클릭: 그 소켓 자리 위(도킹 접근 높이)로 보정 자세 등록. 이미 있으면 그것을 연다.
    반환 (이름 또는 None, 메시지)."""
    g=_calmap_dock_geom()
    ztop=g[2] if g else 0.115
    z=ztop+GRASP_FACE_DZ+GRASP_DZ_DEFAULT+GRASP_LIFT          # 이동 블록이 도킹 위로 가는 높이와 같다
    rel=w2l(np.array([float(x),float(y),z]))
    old=_slot_find(ACTIVE_ARM[0],rel)
    if old is not None:
        FT['pose']=old['name']
        return old['name'],f"'{old['name']}' — 이미 있는 보정 자세를 엽니다 (③연동에서 미세조정)"
    q0=list(CURRENT_Q)
    q=ik_tool((float(x),float(y),z),_seed_list(q0),q0[4],q0[GRIP_IDX])
    if q is None: return None,f"그 자리 ({x*1000:.0f},{y*1000:.0f})mm 는 팔이 닿지 않습니다"
    angles=[float(v) for v in q]
    name=_slot_name(ACTIVE_ARM[0],rel)
    with LOCK:
        S['poses'].append({'name':name,'angles':angles,'offset':[0.0]*NJOINT,
                           'offsets':{'sts':[0.0]*NJOINT},
                           'rel':[round(float(v),5) for v in rel],'arm':ACTIVE_ARM[0],
                           'calmap':True})               # [보정 저장] 전에는 보정 지도에서 초록으로 안 보인다 (표시만)
    _macro_save()
    FT['pose']=name
    print(f"[보정 지도] {name} 등록 (판 위 {x*1000:.0f},{y*1000:.0f}mm)")
    return name,(f"'{name}' 등록 — 실제 도킹을 그 자리에 꽂고 ③연동 [로봇을 이 자세로] → +/- 미세조정 → [보정 저장]")

def _goto_pose_q(oid):
    """그 대상 자리의 '이동' 보정 자세 (없으면 None,None).

    ★ 보정의 주인은 물건이 아니라 판 위 XY 자리다 (2026-10-20) ★
      예전에는 이름 "<대상이름>_이동(x,y,z)" 으로만 찾아서, 보라 도킹에서
      보정해 두고 같은 자리에 주황 도킹을 놓으면 보정 전 값으로 갔다
      (같은 자리의 ppap 보정도 '#2 로 이동' 에서는 무시됐다).
      -> 이동 목표 XY 에서 SLOT_TOL 안에 등록된 '이동' 자세를 이름과 상관없이 쓴다.
         높이가 같으면 보정 각도 그대로, 다르면 그 자리의 XY 보정량만 옮겨
         목표 높이에서 다시 푼다 (높이가 다른 도킹을 같은 자리에 놓은 경우)."""
    try:
        t=goto_point(oid)
        if t is None: return None,None
        hit=_slot_pose(float(t[0]),float(t[1]))
        if hit is None: return None,None
        name,a,(px,py,pz)=hit
        if abs(float(t[2])-pz)<=SLOT_SAME_Z: return name,a
        fp=tool_pos(a)                         # 보정된 자세가 실제로 가리키는 곳
        dx,dy=float(fp[0])-px,float(fp[1])-py  # 그 자리의 XY 보정량
        q=ik_tool((float(t[0])+dx,float(t[1])+dy,float(t[2])),
                  _seed_list(a),a[4],list(CURRENT_Q)[GRIP_IDX])
        if q is None: return None,None
        return f"{name} (XY 보정 {dx*1000:+.1f},{dy*1000:+.1f}mm)",list(q)[:NJOINT]
    except Exception as e:
        print('등록 자세 조회 실패:',e)
    return None,None

# ---- 📱 목표가 사라지면 대기 / 다시 나타나면 이어서 (2026-10-22) ----
#   휴대폰 카메라가 판을 보고 있고(자동 반영 켬, 판 맞춤 완료) 실물로 움직일 때만 작동한다.
#   목표 도킹이 숨겨지면(=카메라에 사라짐) 팔을 든 채(토크 유지) 기다리고, 다시 보이면 그 자리로
#   새로 계산해서 바로 이어 간다. 이동 중에 사라지거나 옮겨지면 그 자리에서 멈추고 다시 계산한다.
class _TargetLost(Exception): pass
GOTO_GUARD={'oid':None,'xy':None,'poid':None,'pxy':None}   # poid/pxy = 일시 정지 뒤 '옮겨졌나' 볼 목표 (11-06)

def _guard_target(oid):
    """일시 정지에서 풀릴 때 '옮겨졌나' 볼 목표를 정한다 (None = 안 봄)."""
    if oid is None:
        GOTO_GUARD.update(poid=None,pxy=None); return
    p=goto_point(oid)
    GOTO_GUARD.update(poid=str(oid),pxy=((float(p[0]),float(p[1])) if p is not None else None))
PATH_VIS={'pts':None}          # 지금 움직이는 블록의 예상 경로 (3D 에 반투명으로)

def _live_hold_on():
    lv=_LIVE[0]
    return (bool(lv) and LIVE_APPLY['on'] and LIVE_APPLY.get('reg') not in (None,'manual')
            and lv.live() and not ROBOT._sim())

def _target_ready(oid):
    oid=str(oid)
    if 'docking' in oid.lower():
        e=_env_ent(oid); return e is not None and bool(e.get('visible',True))
    b=BRICKS.get(oid) or BRICKS.get(_clone_base(oid))
    if b is None or not b.get('pos'): return False
    if HELD.get('brick')==oid: return True
    bp=b['pos']
    for d in docking_ids():                    # 꽂혀 있던 도킹이 사라졌으면 물체도 없는 것
        t=dock_socket_world(d)
        if t and math.hypot(bp[0]-t[0],bp[1]-t[1])<0.025:
            return bool((_env_ent(d) or {}).get('visible',True))
    return True

def _wait_target(oid,tag):
    if _target_ready(oid): return None
    LOG(f"  [{tag}] 목표 {disp_name(oid)} 가 안 보입니다 — 팔을 든 채 기다립니다 (다시 꽂으면 이어서)")
    while not _target_ready(oid):
        if STOP.is_set(): return f"{tag}: 정지됨"
        time.sleep(0.2)
    LOG(f"  [{tag}] 목표 {disp_name(oid)} 다시 보임 — 이어서 갑니다")
    return None

def goto_move(oid,tag='이동'):
    """이동 블록. 휴대폰 카메라가 지켜볼 때는 목표가 사라지면 기다렸다가 이어 간다."""
    while True:
        if _live_hold_on():
            w=_wait_target(oid,tag)
            if w: return w
        p=goto_point(oid)
        GOTO_GUARD.update(oid=(str(oid) if _live_hold_on() else None),
                          xy=((float(p[0]),float(p[1])) if p is not None else None))
        _guard_target(oid)                         # 11-06 일시 정지 뒤 옮겨졌으면 다시 계산
        try:
            return _goto_move_core(oid,tag)
        except _TargetLost as ex:
            LOG(f"  [{tag}] 목표 {disp_name(oid)} {ex} — 그 자리에서 멈추고 다시 계산합니다")
            continue
        finally:
            GOTO_GUARD.update(oid=None,xy=None,poid=None,pxy=None); PATH_VIS['pts']=None

def _goto_move_core(oid,tag='이동'):
    """이동 블록의 '실행 시점' 재계산 버전: 지금 물체가 있는 곳으로
    뽑기(상승) → 수평 → 하강. 미리 계산한 경로는 물체가 파이프라인 중간에
    옮겨지면 옛 위치로 가버리므로, 이동 블록은 항상 이걸 쓴다.

    ★ 단, 그 대상의 '이동' 등록 자세가 있으면 그 자세로 간다 ★
      사용자가 우클릭으로 등록하고 미세조정해 [보정 저장] 한 각도가 정답이다.
      예전에는 그걸 무시하고 소켓 좌표로 매번 새로 풀어서, 아무리 보정해도
      늘 같은 자리로 갔다.
"""
    # ① 보정해 둔 등록 자세가 있으면 그리로 (관절 보간으로 바로 간다)
    #    단 시뮬은 제외 — 등록 자세는 '실물의 조립오차를 손으로 메운 각도' 라
    #    실물에서만 정답이다. 3D 에는 그 오차가 없으므로, 시뮬이 이 값을 쓰면
    #    화면에서 엉뚱한 자리로 간다. 시뮬은 ② 계산 좌표로 간다.
    # ★ 2026-10-20: 등록(보정) 자세 각도로 트윈을 움직이지 않는다 ★
    #   트윈은 아래 ② 이론 경로로 가고, 그 자리 보정량은 _send_real -> _ik_off
    #   (_slot_delta) 가 실물에만 더한다. 아래 ① 은 쓰지 않는다.
    _pn,_pq=None,None
    if _pq is not None:
        LOG(f"  [{tag}] 등록 자세 사용: {_pn}")
        q0=list(CURRENT_Q)
        way,dt=plan_joint_seq(q0,_pq)
        for f in way: f[GRIP_IDX]=q0[GRIP_IDX]
        if LEVEL_ALWAYS: lock_j5(way,q0)
        prev=list(q0)
        for f in way:
            if STOP.is_set(): return f"{tag}: 정지됨"
            try: ROBOT_STS.sync_speed(prev,f,dt)
            except Exception: pass
            _send_real(f,log=True,tag=tag)
            prev=list(f)
            time.sleep(dt/SIM_SPEED[0])
        _settle_or_rest(_pq)
        return None
    # ② 등록 자세가 없으면 예전처럼 소켓 좌표로 푼다
    p=goto_point(oid)
    if p is None: return f"{tag}: {disp_name(oid)} 위 접근점을 만들 수 없습니다"
    # 구형처럼 '이동 중에는 모터를 읽지 않는다'.
    #   ROBOT.read() 는 250ms 나 걸려(모터 5개를 하나씩) 그 사이 팔이 멈춘다.
    #   3D(CURRENT_Q)가 직전 목표를 이미 알고 있으므로 그걸 쓴다.
    q0=list(CURRENT_Q)
    now=tool_pos(q0)
    if (math.hypot(float(p[0])-float(now[0]),float(p[1])-float(now[1]))<SAME_SPOT_XY
            and abs(float(p[2])-float(now[2]))<SAME_SPOT_Z):
        LOG(f"  [{tag}] 이미 {disp_name(oid)} 위입니다 (이동 없음)")      # 11-04: 같은 자리 이동은 제자리
        return None
    zmin=max(float(p[2]),_clear_z())
    ztr=_travel_z(now,p,zmin)
    if ztr is None:
        return (f"{tag}: {disp_name(oid)} 까지 안전 높이({zmin*1000:.0f}mm)로 "
                "갈 수 없습니다 — [로봇 자리 추천]을 눌러보세요")
    PATH_VIS['pts']=[[float(now[0]),float(now[1]),float(now[2])],[float(now[0]),float(now[1]),float(ztr)],
                     [float(p[0]),float(p[1]),float(ztr)],[float(p[0]),float(p[1]),float(p[2])]]
    # ① 뽑기: 지금 자리에서 수직 상승. 실물이 낮게 접힌 자세면 그 자리에서
    #    위로 못 올라가는 경우가 있다 -> base 자세를 경유해 다시 시도한다
    #    (예전엔 여기서 "z=68mm 도달 불가" 로 실물 실행이 첫걸음에 죽었다)
    def _lift(_now,_ztr):
        if _ztr<=float(_now[2])+1e-4: return None      # 이미 충분히 높다
        return _descend(float(_now[0]),float(_now[1]),float(_now[2]),_ztr,
                        f'{tag}① 뽑기')
    e=_lift(now,ztr)
    if e and '정지' not in e:
        bq=get_angles('base')
        if bq is not None:
            LOG(f"  [{tag}] 그 자리에서 상승 불가 -> 기본 자세 경유")
            way,dt=plan_joint_all(list(CURRENT_Q),bq,steps=30)
            for f in way:
                if STOP.is_set(): return f"{tag}: 정지됨"
                _send_real(f,tag=tag)
                time.sleep(dt/SIM_SPEED[0])
            now=tool_pos(list(CURRENT_Q))
            ztr=_travel_z(now,p,zmin)
            if ztr is None:
                return (f"{tag}: {disp_name(oid)} 까지 안전 높이"
                        f"({zmin*1000:.0f}mm)로 갈 수 없습니다 — "
                        "[로봇 자리 추천]을 눌러보세요")
            e=_lift(now,ztr)
    if e: return e
    e=_brick_move_to((float(p[0]),float(p[1]),ztr),f'{tag}② 수평')
    if e: return e
    e=_descend(float(p[0]),float(p[1]),ztr,float(p[2]),f'{tag}③ 하강')
    if e: return e
    return None

def do_ascend_here():
    """상승 블록: 하강의 반대 — 현재 XY 를 유지한 채 '접근 높이'까지
    수직으로만 올라간다 (물컵 규칙 유지, _descend 재사용 — 선형이라 위로도 됨).
    접근 높이 = 잡는 점 +60mm 와 아래 물체 꼭대기 +20mm 중 큰 값."""
    # 구형처럼 '이동 중에는 모터를 읽지 않는다'.
    #   ROBOT.read() 는 250ms 나 걸려(모터 5개를 하나씩) 그 사이 팔이 멈춘다.
    #   3D(CURRENT_Q)가 직전 목표를 이미 알고 있으므로 그걸 쓴다.
    q0=list(CURRENT_Q)
    now=tool_pos(q0)
    x,y,z=float(now[0]),float(now[1]),float(now[2])
    tz=None; what='접근 높이'
    for _ob,_b in BRICKS.items():
        _bp=_b.get('pos')
        if (_bp and HELD.get('brick')!=_ob
                and math.hypot(_bp[0]-x,_bp[1]-y)<0.025):
            gz=float(brick_grasp_point(_ob)[2])
            top=float(_bp[2])+float(_b.get('h',0.0))
            tz=max(gz+GRASP_LIFT,top+0.02)
            what=f'{_ob} 위 접근 높이'; break
    if tz is None:
        for _dk in docking_ids():
            _e=_env_ent(_dk)
            if _e is not None and not _e.get('visible',True): continue
            _t=dock_socket_world(_dk)
            if _t and math.hypot(_t[0]-x,_t[1]-y)<0.025:
                tz=float(_t[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT+GRASP_LIFT
                what=f'{disp_name(_dk)} 위 접근 높이'; break
    if tz is None:
        tz=z+GRASP_LIFT; what=f'지금 높이 +{GRASP_LIFT*1000:.0f}mm'
    if tz<=z+1e-4:
        return f"상승: 이미 접근 높이입니다 (지금 z={z*1000:.0f}mm)"
    e=_descend(x,y,z,tz,'상승')
    if e:
        return (f"{e} (목표: {what} z={tz*1000:.0f}mm) — 그 높이까지 팔이 "
                "안 올라갑니다. [로봇 자리 추천]을 눌러보세요")
    return f"상승: {what} z={tz*1000:.0f}mm 까지 수직 완료"

def dock_occupied(did,except_bid=None):
    """그 도킹 소켓에 이미 물체가 '결합'되어 있는가."""
    t=dock_socket_world(did)
    if t is None: return False
    for ob,b2 in BRICKS.items():
        if ob==except_bid: continue
        bp=b2.get('pos')
        if bp and math.hypot(bp[0]-t[0],bp[1]-t[1])<0.025:
            return True
    return False

def place_brick_on_docking(bid,did):
    """물체를 도킹 스테이션에 거치.

    메이트(dock_mate_fixed.py)가 있으면 솔리드웍스 구속처럼 '꽂는다':
      동심  = 펜 중심축 -> 도킹 소켓 축 (도킹의 위치/회전 반영)
      Z 일치 = 펜 기준 원이 도킹 맨 윗면 높이 (꽂힘 깊이만큼 내려감)
    없으면 예전 근사(중심축 일치, 바닥=도킹 윗면)로 세운다."""
    t=docking_top(did)
    if t is None: return False,f'도킹 스테이션을 찾을 수 없습니다: {did}'
    m=_dock_mate()
    if m and _clone_base(bid)==m.get('brick'):
        sw=dock_socket_world(did)
        x,y=float(sw[0]),float(sw[1])
        zb=float(t[2]-m['seat_dz_mm']/1000.0)
        if not place_brick(bid,x,y,zb):
            return False,f'물체를 찾을 수 없습니다: {bid}'
        p=brick_grasp_point(bid)
        return True,(f"{bid} 를 {did} 소켓에 꽂음 "
                     f"(축 {x*1000:.0f},{y*1000:.0f}mm · "
                     f"바닥 z={zb*1000:.0f}mm = 윗면-{m['seat_dz_mm']:.1f}mm)\n"
                     f"  잡는 점: z={p[2]*1000:.1f}mm "
                     f"(바닥 +{BRICKS[bid]['grasp_dz']*1000:.2f}mm)")
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
    q=ik_tool(p,_seed_list(q0),q0[4],q0[GRIP_IDX])
    if q is None:
        return False,(f'그 지점({p[0]*1000:.0f},{p[1]*1000:.0f},'
                      f'{p[2]*1000:.0f}mm)은 수평 유지로 도달 불가')
    return True,q

GRASP_LIFT=0.047       # 원하는 접근/상승 여유 높이 (m)
#   11-06 (사용자: 펜이 짧아졌으니 더 낮게 가도 된다) 60 -> 47mm. 접근·상승 높이 162+60=222 -> 162+47=209mm.
#   209 = 도킹 윗면 115 + 안전 여유 20 + 새 펜이 잡는 점 아래로 늘어진 길이 74 (82mm 펜: 원점 아래 37 + 37)
#   = 펜을 들고 옮길 때 _clear_z() 가 요구하는 높이와 같다 → 더 낮추면 _clear_z 가 다시 올린다.
#   (B팔은 #9 위에서 최고 225mm 라 222 는 한계 3mm 앞이었다 — 수치 IK 가 초기값 탓에 '못 감' 판정)
HELD={'brick':None}    # 지금 그리퍼가 들고 있는 물체

def _max_z_over(x,y,z_from,z_to=0.30):
    """(x,y) 축 위에서 도달 가능한 가장 높은 z. 없으면 None.
    11-06: 수치 IK 가 초기값 탓에 실패하면 해석 IK(ik_tool_fast — FK 검산·관절 한계 확인 포함)로 한 번 더,
           그리고 마지막에 z_from 자체도 본다 (예전엔 5mm 계단이라 z_from 바로 위만 되는 자리를 놓쳤다)."""
    q0=list(CURRENT_Q); seeds=_seed_list(q0)
    def _ok(z):
        return (ik_tool((x,y,z),seeds,q0[4],q0[GRIP_IDX]) is not None
                or ik_tool_fast((x,y,z),q0[4],q0[GRIP_IDX]) is not None)
    z=z_to
    while z>=z_from-1e-9:
        if _ok(z): return z
        z-=0.005
    if z_from<=z_to and _ok(z_from): return z_from
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
        if z is None:
            try:                                   # 11-06: 어디서 막혔는지 남긴다 (로봇 기준 mm)
                cl=w2l(c); al=w2l(a); bl=w2l(b)
                LOG(f"  [안전 높이] {arm_label(ACTIVE_ARM[0])}: 길 {k}/{n} 지점 (로봇 기준 X{cl[0]*1000:+.0f} Y{cl[1]*1000:+.0f})"
                    f" 에서 {z_min*1000:.0f}~{best*1000:.0f}mm 높이 자세를 못 찾음 — 출발 X{al[0]*1000:+.0f} Y{al[1]*1000:+.0f}"
                    f" Z{al[2]*1000:.0f} → 목표 X{bl[0]*1000:+.0f} Y{bl[1]*1000:+.0f} Z{bl[2]*1000:.0f}")
            except Exception: pass
            return None
        best=min(best,z)
    return best

FT_NEAR_DEG=25.0        # 이 각도 안에 등록 자세가 있으면 그 보정을 쓴다

def _ik_off_base(q=None):
    """IK 로 만든 자세를 실물로 보낼 때 더할 보정값.

    IK 결과는 '이론 자세'라 실물의 조립오차만큼 어긋난다. 이걸 빼먹으면
    실물이 이론값 그대로 움직여 사람이 다친다 (실제 사고 발생).

    보정 선택 규칙:
      ① q 와 관절각이 가장 가까운 '등록 자세'(FT_NEAR_DEG 안)의 보정
         -> 파이프라인 블록을 [자세로 등록]해서 미세조정한 값이 여기서 쓰인다
      ② 없으면 base 보정 (전역 기준)
    """
    try:
        if q is not None:
            best=None
            for p in S['poses']:
                if _is_slot_pose(p): continue   # 자리 보정은 _slot_delta 가 (두 번 더하지 않게)
                o=get_offset(p['name'])
                if not any(abs(v)>1e-9 for v in o): continue
                d=math.degrees(math.sqrt(sum((a-b)**2 for a,b
                               in zip(list(q)[:5],p['angles'][:5]))))
                if d<=FT_NEAR_DEG and (best is None or d<best[0]):
                    best=(d,o,p['name'])
            if best is not None:
                return level_offset(best[1])
        return level_offset(get_offset('base'))
    except Exception:
        return [0.0]*NJOINT

def _ik_off(q=None):
    """실물로 보낼 때 더할 보정값 = 기존 보정(_ik_off_base) + 그 자리의 보정량(_slot_delta).
    ★ 실물 전송 경로에서만 쓰인다 — 3D 트윈은 언제나 이론 자세 q 를 그린다 (2026-10-20) ★"""
    o=_ik_off_base(q)
    if SEAT_FIX[0] is not None:                  # 잡기 중: 놓을 때 잰 '소켓이 끌어준 양' (11-04)
        o=[a+b for a,b in zip(o,SEAT_FIX[0])]
    if q is None: return o
    try:
        d=_slot_delta(q)
        if d is not None: o=[a+b for a,b in zip(o,d)]
    except Exception as e:
        print('[자리 보정] 계산 실패:',e)
    return o

# 실물 모터가 있는 로봇(패밀리). 이 외의 로봇을 골라 두고 실물로 보내면
# 축 수·기구학이 달라 엉뚱한 각도가 모터에 들어간다 -> 사람이 다친다.
REAL_FAMILIES=('sts3215','dynamixel')

def cal_joint_labels(rk):
    """③연동 미세조정에 보일 관절 이름. SO-ARM 은 선 없는 모터(j5 고정)를 뺀다."""
    n=robot_njoints(rk)
    return [f'J{i+1}' for i in range(n)
            if not (rk=='sts' and i<len(J2ID) and J2ID[i] in STS_DEAD_IDS)]

def real_robot_ok():
    """지금 3D 에서 고른 로봇이 '실물이 있는 로봇' 인가?
    반환: (가능?, 사유). 예: UR15 를 골라 두면 실물 전송을 막는다."""
    fam=os.path.basename(os.path.dirname(URDF_PKG_DIR))
    if fam in REAL_FAMILIES: return True,''
    return False,(f"지금 3D 로봇은 '{os.path.basename(URDF_PKG_DIR)}' 입니다 — "
                  "실물 로봇이 아닙니다.\n"
                  "  실물을 움직이려면 ③로봇 탭에서 SO-ARM 을 고르세요.")

def _send_real(q,log=False,tag=''):
    """3D 는 이론 자세, 실물은 보정 적용 자세로 보낸다.

    안전장치: 고른 로봇과 실물이 다르면 아예 보내지 않는다.
    (UR15 8축 각도를 SO-ARM 6축 모터에 보내면 매우 위험하다)"""
    g=GOTO_GUARD
    if g['oid'] is not None and _live_hold_on():      # 📱 이동 중 목표가 사라지거나 옮겨지면 멈춘다
        if not _target_ready(g['oid']): raise _TargetLost('사라짐')
        _p=goto_point(g['oid'])
        if _p is not None and g['xy'] and math.hypot(_p[0]-g['xy'][0],_p[1]-g['xy'][1])>0.015:
            raise _TargetLost('옮겨짐')
    if ESTOP['held']: STOP.set(); return       # 11-06 비상정지 버튼이 눌려 있으면 멈춤 (잡기·놓기가 STOP 을 지워도)
    if _pause_point()=='stop': return          # 11-06 일시 정지 (정지로 끝나면 보내지 않는다)
    ros_publish(q)
    # 실물 전송 차단은 ROBOT.send(RobotMux) 안에서 최종 판단한다
    ROBOT.send([a+o for a,o in zip(q,_ik_off(q))],log=log,tag=tag)
# ★ 구형(arm_studio.py) 방식 ★
#   구형은 이동이 이게 전부였다:  send -> sleep 반복 -> 끝에 sleep(0.5)
#   settle(도착 확인 + 과보정)도, 이동 중 read() 도 아예 없었고
#   3개월간 진동·보정 문제가 없었다.
#   지금 코드는 걸음마다 모터를 읽고 오차를 재고 목표를 다시 밀어넣는다.
#   그게 진동의 근원이었다 -> 구형처럼 '보내고 쉬기' 만 한다.
#   문제가 생기면 SETTLE_ON 을 True 로 되돌리면 예전 동작으로 돌아간다.
SETTLE_ON=[False]          # False = 구형 방식 (settle 안 함)
SETTLE_PUSH=[True]         # settle 을 켤 때만 의미 있음

def _settle_or_rest(q=None):
    """구형 방식: 도착을 확인하지 않고 잠깐 쉬기만 한다.
    (구형은 한 구간 끝에 time.sleep(0.5) 하나뿐이었다)"""
    if SETTLE_ON[0] and q is not None:
        return ROBOT.settle(q)
    time.sleep(0.5/max(0.1,SIM_SPEED[0]))
    return True,0

_SENT_WARN=[False]

ONESHOT_VERT=[True]     # 수직 이동을 한 번에 부드럽게 (끄면 예전 방식)

def flow_send(way,dt,tag='',log=False):
    """★ 예전 Gradio 가 부드러웠던 방식 ★

    예전(Dynamixel)은 웨이포인트를 50ms 마다 보내면서, 모터에는
    '110ms 짜리 이동' 을 시켰다(시간 기반 프로파일). 그래서 이전 이동이
    끝나기 전에 다음 목표가 도착해 모터가 멈추지 않고 이어서 움직였다.
      -> 가다서다가 없다 = 부드럽다

    지금(STS3215)은 속도만 주면 '최대한 빨리' 가서 남는 시간 멈춰 있다가
    다음 목표에 급출발한다 = 덜덜.
    그래서 각 걸음을 '프레임 간격보다 넉넉한 시간' 에 맞춘 속도로 보낸다.
    """
    if not way: return None
    # 그라디오(Dynamixel)는 '110ms 안에 도착' 이라는 시간을 직접 줄 수 있어
    # 프레임(50ms)보다 길게 잡아 이동이 겹쳤다 = 멈추지 않는다.
    # STS 는 속도만 줄 수 있으므로, 프레임 시간에 '딱 맞는' 속도를 계산해
    # 다음 목표가 올 때까지 계속 움직이도록 한다.
    OVER=1.15                      # 프레임을 살짝 넘겨 이동이 이어지게
    prev=None
    for f in way:
        if STOP.is_set(): return f"{tag}: 정지됨"
        qc=[a+o for a,o in zip(f,_ik_off(f))]
        raws={}; vel={}
        for j in range(min(5,NJOINT)):
            i=J2ID[j]
            if i==GRIP_ID or i in STS_DEAD_IDS: continue
            raws[i]=rad_to_raw(j,qc[j])[0]
            if prev is not None:
                d=abs(raws[i]-prev.get(i,raws[i]))
                # 이 걸음을 dt*OVER 초에 걸쳐 가도록 (=천천히, 끊기지 않게)
                # 하한을 높게 두면 걸음을 순식간에 끝내고 멈춰 기다린다.
                # 실측: 속도값 ≈ 카운트/초 (50 근처는 급격히 느려지므로 55 하한)
                need=d/max(0.02,dt*OVER)
                vel[i]=int(max(55,min(1500,need)))
            else:
                vel[i]=SAFE_VEL*2
        if not ROBOT_STS._sync_goal_timed(raws,vel):
            ROBOT_STS._sync_goal(raws)
        prev=raws
        ros_publish(f)
        if log: LOG_TARGET(raws,tag)
        time.sleep(dt/SIM_SPEED[0])
    return None

_VPATH={}          # (x,y,z_hi,z_lo) -> 미리 계산해 둔 경로

def _vert_path(x,y,z_hi,z_lo,q0):
    """★ 그라디오 방식 ★ 수직 경로를 3mm 마다 미리 다 계산해 둔다.

    그라디오(plan_ee_line)가 부드러웠던 이유:
      · 경로를 '미리' 전부 계산 -> 실행 중엔 계산으로 멈추는 일이 없다
      · 3mm 마다 촘촘히 (지금은 15mm 라 뚝뚝 끊겼다)
      · te=t*t*(3-2*t) 감속곡선 -> 출발/도착에서 천천히
    같은 구간은 캐시해서 두 번 계산하지 않는다.
    """
    key=(round(x,4),round(y,4),round(z_hi,4),round(z_lo,4),
         tuple(round(v,4) for v in q0[:5]))
    hit=_VPATH.get(key)
    if hit is not None: return hit
    L=abs(z_hi-z_lo)
    n=max(6,int(L/0.003))           # 3mm 마다 (그라디오와 동일)
    way=[list(q0)]; q=list(q0)
    last_z=z_hi
    for k in range(1,n+1):
        t=k/n
        te=t*t*(3-2*t)               # 감속곡선 (그라디오와 동일)
        z=z_hi+(z_lo-z_hi)*te
        # 걸음이 1.5mm 보다 작으면 IK 가 수렴하지 못해 실패한다
        # (감속곡선은 시작/끝에서 걸음이 아주 작아진다) -> 건너뛴다
        if k<n and abs(z-last_z)<0.0015: continue
        last_z=z
        qn=ik_tool((x,y,z),_seed_list(q),q[4],q[GRIP_IDX])
        if qn is None: return None
        qn[GRIP_IDX]=q0[GRIP_IDX]
        if LEVEL_ALWAYS: qn[J5_IDX]=q0[J5_IDX]
        # 한 걸음이 너무 크면 포기 (그라디오와 동일한 안전장치)
        if max(abs(a-b) for a,b in zip(qn,q))>math.radians(15): return None
        way.append(qn); q=qn
    if len(_VPATH)>200: _VPATH.clear()
    _VPATH[key]=way
    return way


def _descend_smooth(x,y,z_hi,z_lo,tag,n):
    """수직 이동을 '한 번에' — 중간 높이는 도달 가능 검사만 한다.

    걸음마다 멈췄다 다시 출발하면 덜덜거린다. 중간 지점을 미리 다 풀어보고
    (경로가 옆으로 새지 않는지 확인), 실제 모터에는 끝점 하나만 준다.
    """
    # ROBOT.read() 는 250ms 나 걸린다(모터 5개를 하나씩 읽음).
    # 그 사이 로봇이 멈춰 있어 '뚝 끊긴' 느낌이 난다.
    # 3D(CURRENT_Q)가 직전 목표를 이미 알고 있으므로 그걸 쓴다.
    q0=list(CURRENT_Q)
    qs=q0; qend=None
    for k in range(1,n+1):
        if STOP.is_set(): return f"{tag}: 정지됨"
        z=z_hi+(z_lo-z_hi)*k/n
        q=ik_tool((x,y,z),_seed_list(qs),qs[4],qs[GRIP_IDX])
        if q is None: return (f"{tag}: ({x*1000:.0f},{y*1000:.0f}) "
                              f"z={z*1000:.0f}mm 도달 불가")
        q[GRIP_IDX]=q0[GRIP_IDX]
        if LEVEL_ALWAYS: q[J5_IDX]=q0[J5_IDX]
        qs=q; qend=q
    if qend is None: return None
    # 하강은 항상 조금 덜 내려간다 -> 그만큼 더 내려갈 목표로
    if z_lo<z_hi and DESCEND_BIAS_MM:
        qb2=ik_tool((x,y,z_lo-DESCEND_BIAS_MM/1000.0),
                    _seed_list(qend),qend[4],qend[GRIP_IDX])
        if qb2 is not None:
            qb2[GRIP_IDX]=q0[GRIP_IDX]
            if LEVEL_ALWAYS: qb2[J5_IDX]=q0[J5_IDX]
            qend=qb2
    qc=[a+o for a,o in zip(qend,_ik_off(qend))]
    q0c=[a+o for a,o in zip(q0,_ik_off(q0))]
    def _vis(t,_a=list(q0),_b=list(qend)):
        ros_publish([u+(v-u)*t for u,v in zip(_a,_b)])
    LOG_TARGET({J2ID[j]:rad_to_raw(j,qc[j])[0] for j in range(5)},tag)
    if not ROBOT_STS.glide(q0c,qc,watch=_vis):
        return f"{tag}: 정지됨"
    ros_publish(qend)
    if not FORCE_SIM[0] and ROBOT.connect():
        try: ROBOT_STS.hold_gain()
        except Exception: pass
        ROBOT.send(qc); _settle_or_rest(qc)
    return None


def _descend(x,y,z_hi,z_lo,tag,steps=None):
    """축을 맞춘 채 수직으로만 내려간다 (x,y 고정 = 두 축 일치 유지).
    중간 지점마다 IK 를 풀어 보내므로 경로가 옆으로 새지 않는다."""
    # 하강/상승은 수직선을 지키려고 높이마다 IK 를 푼다. 하지만 걸음마다
    # 가속->감속->정지를 반복하면 뚝뚝 끊겨 덜덜거린다(특히 중력이 걸리는
    # 하강). 그래서 중간 지점들은 '검사만' 하고, 실제 이동은 끝점까지
    # 한 번에 부드럽게 간다. (사용자: "하강이 진짜 덜덜거린다")
    n=steps or max(6,int(abs(z_hi-z_lo)/0.005))   # 8/13 값 (5mm)
    if FORCE_SIM[0] and SIM_SPEED[0]>1.5:         # 11-05 (사용자: 미리보기가 느리다) 3D 에서만 배속만큼 성기게 (최대 4배) — 실물은 그대로 5mm
        n=max(3,int(round(n/min(4.0,SIM_SPEED[0]/1.5))))
    if False:                                     # 8/13 방식으로 되돌림
        # ★ 그라디오 방식 ★ 3mm 마다 미리 계산해 둔 경로를 재생만 한다.
        q0=list(CURRENT_Q)
        way=_vert_path(x,y,z_hi,z_lo,q0)
        if way is not None:
            e=flow_send(way,0.08,tag,log=True)   # 그라디오 간격(80ms)
            if e: return e
            if not FORCE_SIM[0] and ROBOT.connect():
                qf=list(way[-1])
                # 하강은 늘 조금 덜 내려간다 -> 편향 보정
                if z_lo<z_hi and DESCEND_BIAS_MM:
                    qb2=ik_tool((x,y,z_lo-DESCEND_BIAS_MM/1000.0),
                                _seed_list(qf),qf[4],qf[GRIP_IDX])
                    if qb2 is not None:
                        qb2[GRIP_IDX]=q0[GRIP_IDX]
                        if LEVEL_ALWAYS: qb2[J5_IDX]=q0[J5_IDX]
                        qf=qb2
                qc=[a+o for a,o in zip(qf,_ik_off(qf))]
                try: ROBOT_STS.hold_gain()
                except Exception: pass
                ROBOT.send(qc); _settle_or_rest(qc)
            return None
        return _descend_smooth(x,y,z_hi,z_lo,tag,min(n,4))
    # 구형처럼 '이동 중에는 모터를 읽지 않는다'.
    #   ROBOT.read() 는 250ms 나 걸려(모터 5개를 하나씩) 그 사이 팔이 멈춘다.
    #   3D(CURRENT_Q)가 직전 목표를 이미 알고 있으므로 그걸 쓴다.
    q0=list(CURRENT_Q)
    for k in range(1,n+1):
        if STOP.is_set(): return f"{tag}: 정지됨"
        z=z_hi+(z_lo-z_hi)*k/n
        q=ik_tool((x,y,z),_seed_list(q0),q0[4],q0[GRIP_IDX])
        if q is None: return (f"{tag}: ({x*1000:.0f},{y*1000:.0f}) "
                              f"z={z*1000:.0f}mm 도달 불가")
        q[GRIP_IDX]=q0[GRIP_IDX]
        if LEVEL_ALWAYS: q[J5_IDX]=q0[J5_IDX]
        # 축마다 이동량이 다르면 같이 도착하도록 속도를 맞춘다(물컵 유지) — 실물 모터용 (3D 미리보기에서는 건너뜀)
        if not FORCE_SIM[0]:
            try: ROBOT_STS.sync_speed(q0,q,0.05)
            except Exception: pass
        _send_real(q,log=True,tag=tag)        # 8/13 방식: 그냥 보내고 쉰다
        q0=q
        time.sleep(0.05*(DXL_STEP_SLOW if EXEC.get('mode')=='dxl' else 1.0)
                   /SIM_SPEED[0])
    # 마지막 목표에 정확히 앉을 때까지 (실물만) — 8/13 과 동일
    if not FORCE_SIM[0] and ROBOT.connect():
        _settle_or_rest(q0)
    return None

def _brick_move_to(p,tag):
    """툴 중심을 월드 좌표 p 로 (수평 유지). 실패 시 사유 문자열 반환."""
    # 구형처럼 '이동 중에는 모터를 읽지 않는다'.
    #   ROBOT.read() 는 250ms 나 걸려(모터 5개를 하나씩) 그 사이 팔이 멈춘다.
    #   3D(CURRENT_Q)가 직전 목표를 이미 알고 있으므로 그걸 쓴다.
    q0=list(CURRENT_Q)
    q=ik_tool(np.asarray(p,float),_seed_list(q0),q0[4],q0[GRIP_IDX])
    if q is None:
        return f"{tag}: 그 지점에 도달할 수 없습니다"
    q[GRIP_IDX]=q0[GRIP_IDX]
    if LEVEL_ALWAYS: q[J5_IDX]=q0[J5_IDX]
    way,dt=plan_joint_seq(q0,q)
    for f in way: f[GRIP_IDX]=q0[GRIP_IDX]
    if LEVEL_ALWAYS: lock_j5(way,q0)
    # 8/13 방식: 웨이포인트를 일정 간격으로 그냥 보낸다
    # + 축마다 이동량이 다르면 같이 도착하도록 속도만 맞춘다(물컵 유지)
    ros_publish(q)
    prev=list(q0)
    for f in way:
        if STOP.is_set(): return f"{tag}: 정지됨"
        if not FORCE_SIM[0]:                     # 실물 모터용 속도 맞추기 (3D 미리보기에서는 건너뜀, 11-05)
            try: ROBOT_STS.sync_speed(prev,f,dt)
            except Exception: pass
        _send_real(f,log=True,tag=tag); time.sleep(dt/SIM_SPEED[0])
        prev=f
    return None

def _grip_vis(op):
    """그리퍼 열기/닫기를 3D 화면에도 반영."""
    gq=list(CURRENT_Q); gq[GRIP_IDX]=_j6_rad(op)
    ros_publish(gq,with_grip=True)

def grip_wait(op,timeout=8.0):
    """그리퍼가 실제로 다 열리거나 닫힐 때까지 기다린다.

    ★ 이걸 안 하면 그리퍼가 채 닫히기 전에 팔이 상승해 물체를 놓친다 ★
      (실측: 완전히 닫히는 데 3.2초. 예전엔 0.5초만 쉬고 바로 올라갔다)

    닫기 중 위치가 멈추면 '물체를 잡은 것' 으로 보고 거기서 멈춘다.
    (끝까지 조이면 과부하 보호가 걸린다)
    반환: (사유, 마지막 위치)
    """
    # ★ 시뮬에는 기다릴 실물이 없다 ★
    #   시뮬인데도 그리퍼 위치를 읽으려 해서, 모터가 없으니 매번 None →
    #   도착 판정을 못 하고 8초 타임아웃을 꽉 채웠다.
    #   (잡기 한 번에 16초 = 열기 8 + 닫기 8)
    #   실물은 절대 손대지 않는다 — 그리퍼가 덜 닫힌 채 팔이 올라가면 위험하다.
    if ROBOT._sim():
        return '시뮬',None

    tgt=GRIPPER_OPEN if op else GRIPPER_CLOSE
    t0=time.time(); p=None; lastp=None; p_start=None
    still=0; retried=False; why='타임아웃'
    while time.time()-t0<timeout:
        if STOP.is_set(): return '정지',p
        p=ROBOT._r2(GRIP_ID,ADDR_PRESENT)
        if p is not None:
            if p_start is None: p_start=p
            if abs(p-tgt)<25: why='도착'; break
            if op:
                if (not retried and time.time()-t0>1.2
                        and abs(p-p_start)<15):
                    LOG('  [그리퍼] 안 움직임 -> 보호 해제 + 토크 상향 재시도')
                    ROBOT.gripper(True,tq=500); retried=True
            else:
                if lastp is not None and abs(p-lastp)<8:
                    still+=1
                    if still>=4:
                        why='물체 잡음'
                        hold=max(GRIPPER_CLOSE,p-25)
                        ROBOT._w2(GRIP_ID,ADDR_GOAL,hold)
                        LOG(f'  [그리퍼] 잡음 감지 @{p} -> 유지 목표 {hold}')
                        break
                else: still=0
            lastp=p
        time.sleep(0.12)
    LOG(f"  [그리퍼] {'열기' if op else '닫기'} {why} "
        f"({time.time()-t0:.1f}초, 위치 {p})")
    return why,p


def grasp_z(bid):
    p=brick_grasp_point(bid); return float(p[2]) if p is not None else None
def release_z(did):
    t=dock_socket_world(did); return float(t[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT if t is not None else None

# ★ 놓은 자리 기억 → 다시 잡을 때 그 자리로 (2026-11-04, 사용자: 놓고 올라올 땐 괜찮은데 다시 내려가면 펜을 찍는다) ★
#   놓기와 잡기는 같은 목표(틱 단위까지 같음)로 내려가는데, 놓을 때는 펜이 소켓 홈을 타고 들어가며
#   팔을 몇 mm 끌어당겨 맞춰 준다. 펜을 놓으면 그 힘이 사라져 팔이 제자리로 돌아가고, 다시 내려가면
#   그 몇 mm 때문에 집게가 펜 머리를 찍는다.
#   -> 놓기 바닥에서 '꽂힌 채(닫힘)' A1 과 '놓은 뒤(열림)' A2 실제 각도를 읽어, A1-A2 (= 소켓이 끌어준 양)
#      를 그 도킹에 기억한다. 같은 로봇 자리·같은 도킹 축에서 그 펜을 잡을 때 이만큼 더해 내려간다
#      (실물만, j1~j4). 너무 크면 (관절 SEAT_MAX_DEG 또는 SEAT_MAX_MM 초과) 안 쓰고 기록만 한다.
SEAT_MEM={}              # 도킹 id -> {'delta':[rad..],'xy':(x,y),'pose':(bx,by,yaw,팔),'mm':(dx,dy),'t':시각}
SEAT_FIX=[None]          # 잡는 동안만 _ik_off 에 더해지는 값
SEAT_APPLY=[False]       # 11-04: 끔 — 측정만 (물컵 규칙을 깨서 그리퍼가 기울었다)
SEAT_MAX_DEG=4.0
SEAT_MAX_MM=10.0
SEAT_XY_TOL=0.003

def _seat_pose_key():
    return (round(float(BASE_OFF[0]),4),round(float(BASE_OFF[1]),4),round(float(BASE_YAW[0]),4),ACTIVE_ARM[0])

def _seat_real():
    return (not FORCE_SIM[0]) and (not ROBOT._sim()) and ROBOT.connect()

def _seat_read():
    try:
        time.sleep(0.25)                              # 멈춘 뒤 읽기
        return list(ROBOT.read())
    except Exception as e:
        LOG(f"  [꽂힘 자리] 읽기 실패: {e}"); return None

def _seat_dock_of(bid):
    """그 물체가 꽂혀 있는 (보이는) 도킹 id."""
    b=BRICKS.get(bid) or {}
    bp=b.get('pos')
    if not bp: return None
    for d in docking_ids():
        e=_env_ent(d)
        if e is not None and not e.get('visible',True): continue
        tt=dock_socket_world(d)
        if tt and math.hypot(bp[0]-tt[0],bp[1]-tt[1])<0.015: return d
    return None

GOTO_BEFORE_TOL=0.015    # 잡기·놓기 전, 대상 축에서 이만큼(m) 넘게 떨어져 있으면 먼저 그 위로 간다

def _goto_before(oid,tag):
    """★ 잡기·놓기는 '어디'를 가진다 (2026-11-04, 사용자) ★
    예전엔 제자리에서 수직으로만 움직여서, [놓기 #8] 을 #2 위에서 하면 실물은 #2 에
    내려놓고 트윈만 펜을 #8 로 옮겼다. -> 대상 위가 아니면 [이동] 과 똑같은 길(goto_move,
    자리 보정 포함)로 먼저 그 위로 간다. 바로 앞에 같은 대상 [이동] 이 있으면 이미 위라 그냥 지나간다."""
    if _live_hold_on():
        w=_wait_target(oid,tag)
        if w: return w
    _p=goto_point(oid); _n=tool_pos(list(CURRENT_Q))
    if _p is not None and math.hypot(_p[0]-_n[0],_p[1]-_n[1])>GOTO_BEFORE_TOL:
        LOG(f"  [{tag}] {disp_name(oid)} 위가 아니라 먼저 그 위로 이동합니다")
        return goto_move(oid,f'{tag} 전 이동')
    return None

def do_grasp(bid):
    """잡기 블록. 대상(펜) 위가 아니면 먼저 그 위로 간다 (_goto_before)."""
    if bid not in BRICKS or not BRICKS[bid].get('pos'):
        return f"{bid} 가 배치되지 않았습니다"
    if HELD.get('brick')!=bid:
        e=_goto_before(bid,'잡기')
        if e: return e
    try:
        _n=tool_pos(list(CURRENT_Q))
        _z=grasp_z(bid)
        if _z is not None:
            PATH_VIS['pts']=[[float(_n[0]),float(_n[1]),float(_n[2])],[float(_n[0]),float(_n[1]),_z]]
    except Exception:
        pass
    try:
        for _k in range(6):
            try:
                return _do_grasp_core(bid)
            except _TargetLost as ex:            # 11-06 일시 정지 중 펜이 옮겨짐 → 위로 올라가 새 자리로
                LOG(f"  [잡기] {disp_name(bid)} {ex} — 그 자리에서 올라가 새 자리로 다시 갑니다")
                e=goto_move(bid,'잡기 다시')
                if e: return e
        return f"잡기: {disp_name(bid)} 자리가 계속 바뀌어 멈춥니다"
    finally:
        PATH_VIS['pts']=None

def _do_grasp_core(bid):
    """잡기 (사용자 정의 2026-08-14) = 열기 → 하강 → 닫기 → 상승.

    이동은 포함하지 않는다 — 먼저 [이동→사물] 블록으로 축 위에 와 있어야
    한다 (축이 어긋나 있으면 여기서 정직하게 실패).
    하강 목표 = 잡는 점 (도킹 결합 시 고정식 115+10+37 = 162mm)."""
    BTN('잡기',bid)
    p=brick_grasp_point(bid)
    if p is None: return f"{bid} 가 배치되지 않았습니다"
    if RUNNING.is_set(): return "다른 동작 실행 중 - [정지] 후 다시"
    if not ROBOT.connect():
        return "로봇 연결 실패 — [이동] 탭에서 '시뮬레이션' 을 고르면 3D로 볼 수 있습니다"
    # ★ 잡기는 좌표를 갖지 않는다 ★
    #   좌표를 가진 블록은 [이동] 하나뿐이다. 잡기가 대상 XY 를 따로
    #   계산하면 이동과 두 개가 되어 서로 어긋난다(실측 23mm).
    #   -> 지금 서 있는 자리(XY)에서 수직으로만 움직인다. 높이만 대상에서 얻는다.
    now=tool_pos(list(CURRENT_Q))      # 구형: 이동 중 모터를 읽지 않는다
    x,y=float(now[0]),float(now[1])               # 제자리 — 이동이 데려다 준 곳
    zg=float(p[2])                                # 잡는 높이만 대상에서
    pen_top=float(BRICKS[bid]['pos'][2])+float(BRICKS[bid].get('h',0.0))
    z_up=max(zg+GRASP_LIFT,pen_top+0.02)          # 상승 목표 = 접근 높이
    _fix=None
    _dk=_seat_dock_of(bid)
    _sm=SEAT_MEM.get(_dk) if _dk else None
    if _sm is not None and _seat_real():
        if _sm['pose']!=_seat_pose_key() or math.hypot(_sm['xy'][0]-x,_sm['xy'][1]-y)>SEAT_XY_TOL:
            LOG(f"  [꽂힘 자리] {disp_name(_dk)}: 놓을 때와 로봇·도킹 자리가 달라 안 씀")
        elif (max(abs(math.degrees(v)) for v in _sm['delta'][:4])>SEAT_MAX_DEG
              or math.hypot(*_sm['mm'])>SEAT_MAX_MM):
            LOG(f"  [꽂힘 자리] {disp_name(_dk)}: 값이 너무 커서({math.hypot(*_sm['mm']):.1f}mm) 안 씀 — 보정을 확인하세요")
        else:
            # ★ 11-04 실측 후 끔 ★ 관절 차이를 그대로 더하면 물컵 규칙(j2+j3+j4=0)이 깨져 그리퍼가
            #   3.6° 기울었다. 게다가 옆으로 돌아간 양은 0.3~0.6mm 뿐(대부분 위아래)이라 쓸 이유도 없다.
            #   측정은 참고용 기록으로만 남긴다. 다시 켜려면 SEAT_APPLY[0]=True (물컵 규칙 맞춤 필요).
            if SEAT_APPLY[0]:
                _fix=list(_sm['delta'])
            LOG(f"  [꽂힘 자리] {disp_name(_dk)}: 놓을 때 옆으로 X{_sm['mm'][0]:+.1f} Y{_sm['mm'][1]:+.1f}mm "
                +("— 맞춰 내려갑니다" if _fix is not None else "(참고 기록 · 보정 안 함)"))
    STOP.clear(); RUNNING.set()
    try:
        ROBOT.time_profile(True); ROBOT.torque(True)
        if _fix is not None:
            SEAT_FIX[0]=_fix
            ROBOT.send([a+o for a,o in zip(list(CURRENT_Q),_ik_off(list(CURRENT_Q)))])   # 위에서 먼저 맞추고
            _settle_or_rest()
        _run_at(RUN_AT.get('blk',-1),'열기')
        ROBOT.gripper(True); _grip_vis(True)
        grip_wait(True)                   # ① 다 열릴 때까지
        _run_at(RUN_AT.get('blk',-1),'하강')
        _guard_target(bid)                       # 11-06: 일시 정지 중 펜(도킹)이 옮겨지면 다시
        try:
            e=_descend(x,y,float(now[2]),zg,'잡기② 하강')            # ② 수직 하강
        finally:
            _guard_target(None)
        if e: return e
        _run_at(RUN_AT.get('blk',-1),'닫기')
        ROBOT.gripper(False); _grip_vis(False)
        grip_wait(False)                  # ③ 다 닫힐 때까지 기다린다(약 3초)
        HELD['brick']=bid
        _run_at(RUN_AT.get('blk',-1),'상승')
        e=_descend(x,y,zg,z_up,'잡기④ 상승')                     # ④ 수직 상승
        if e: return e
        if _dk: SEAT_MEM.pop(_dk,None)            # 펜을 뽑았으니 그 도킹 기억은 끝
        return (f"{bid} 잡음 — 열기 → 하강 {zg*1000:.0f}mm → 닫기 → "
                f"상승 {z_up*1000:.0f}mm")
    except _TargetLost:
        raise                                    # 11-06: do_grasp 가 올라가서 새 자리로 다시
    except Exception as ex:
        ROBOT.estop(); return f"오류: {ex} - 토크 해제됨"
    finally:
        SEAT_FIX[0]=None
        RUNNING.clear()

def do_release(did):
    """놓기 블록. 대상 도킹 위가 아니면 먼저 그 위로 간다 (_goto_before)."""
    if HELD.get('brick') is None: return "들고 있는 물체가 없습니다 (먼저 [잡기])"
    if dock_socket_world(did) is None: return f"도킹을 찾을 수 없습니다: {did}"
    e=_goto_before(did,'놓기')
    if e: return e
    try:
        _n=tool_pos(list(CURRENT_Q))
        _z=release_z(did)
        if _z is not None:
            PATH_VIS['pts']=[[float(_n[0]),float(_n[1]),float(_n[2])],[float(_n[0]),float(_n[1]),_z]]
    except Exception:
        pass
    try:
        for _k in range(6):
            try:
                return _do_release_core(did)
            except _TargetLost as ex:            # 11-06 일시 정지 중 도킹이 옮겨짐 → 위로 올라가 새 자리로
                LOG(f"  [놓기] {disp_name(did)} {ex} — 그 자리에서 올라가 새 자리로 다시 갑니다")
                e=goto_move(did,'놓기 다시')
                if e: return e
        return f"놓기: {disp_name(did)} 자리가 계속 바뀌어 멈춥니다"
    finally:
        PATH_VIS['pts']=None

def _do_release_core(did):
    """놓기 (사용자 정의 2026-08-14) = 하강 → 열기 → 상승 → 닫기.

    이동은 포함하지 않는다 — 먼저 [이동→사물] 블록으로 소켓 축 위에 와
    있어야 한다. 하강 목표 = 놓는 점 (고정식 115+10+37 = 162mm)."""
    BTN('놓기',did)
    bid=HELD['brick']
    if bid is None: return "들고 있는 물체가 없습니다 (먼저 [잡기])"
    t=dock_socket_world(did)
    if t is None: return f"도킹을 찾을 수 없습니다: {did}"
    if dock_occupied(did,except_bid=bid):
        return f"{disp_name(did)} 에 이미 다른 물체가 있습니다"
    if RUNNING.is_set(): return "다른 동작 실행 중 - [정지] 후 다시"
    if not ROBOT.connect(): return "로봇 연결 실패"
    # ★ 놓기도 좌표를 갖지 않는다 ★ (잡기와 같은 이유)
    #   좌표는 [이동] 블록 하나뿐. 여기서는 지금 자리에서 수직으로만 움직인다.
    zg=float(t[2])+GRASP_FACE_DZ+GRASP_DZ_DEFAULT       # 놓는 점 = 162mm 고정식
    now=tool_pos(list(CURRENT_Q))      # 구형: 이동 중 모터를 읽지 않는다
    x,y=float(now[0]),float(now[1])                     # 제자리
    STOP.clear(); RUNNING.set()
    try:
        ROBOT.time_profile(True); ROBOT.torque(True)
        _run_at(RUN_AT.get('blk',-1),'하강')
        _guard_target(did)                       # 11-06: 일시 정지 중 도킹이 옮겨지면 다시
        try:
            e=_descend(x,y,float(now[2]),zg,'놓기① 하강')            # ① 펜이 소켓에
        finally:
            _guard_target(None)
        if e: return e
        _a1=_seat_read() if _seat_real() else None              # 꽂힌 채 (소켓이 팔을 끌어준 상태)
        _run_at(RUN_AT.get('blk',-1),'열기')
        ROBOT.gripper(True); _grip_vis(True)
        grip_wait(True)                   # ② 다 열릴 때까지 = 놓임
        if _a1 is not None:
            _a2=_seat_read()                                    # 놓은 뒤 (팔이 제자리로 돌아간 상태)
            if _a2 is not None:
                _d=[(_a1[j]-_a2[j]) if j<4 else 0.0 for j in range(NJOINT)]
                try:
                    _q=list(CURRENT_Q); _p0=tool_pos(_q); _p1=tool_pos([a+b for a,b in zip(_q,_d)])
                    _mm=((float(_p1[0])-float(_p0[0]))*1000,(float(_p1[1])-float(_p0[1]))*1000)
                except Exception:
                    _mm=(0.0,0.0)
                SEAT_MEM[did]={'delta':_d,'xy':(x,y),'pose':_seat_pose_key(),'mm':_mm,'t':time.time()}
                LOG(f"  [꽂힘 자리] {disp_name(did)}: 펜을 놓자 팔이 {math.hypot(*_mm):.1f}mm 돌아감 "
                    f"(X{_mm[0]:+.1f} Y{_mm[1]:+.1f}mm · "
                    +' '.join(f"j{j+1}:{math.degrees(_d[j]):+.2f}°" for j in range(4))
                    +") — 참고 기록")
        place_brick_on_docking(bid,did)                          # 소켓에 안착
        HELD['brick']=None
        pen_top=float(BRICKS[bid]['pos'][2])+float(BRICKS[bid].get('h',0.0))
        z_up=max(zg+GRASP_LIFT,pen_top+0.02)
        _run_at(RUN_AT.get('blk',-1),'상승')
        e=_descend(x,y,zg,z_up,'놓기③ 상승')                     # ③ 열린 채 상승
        if e: return e
        _run_at(RUN_AT.get('blk',-1),'닫기')
        ROBOT.gripper(False); _grip_vis(False)
        grip_wait(False)                  # ④ 닫기
        return (f"{bid} 를 {disp_name(did)} 소켓에 놓음 — 하강 {zg*1000:.0f}mm "
                f"→ 열기 → 상승 {z_up*1000:.0f}mm → 닫기")
    except _TargetLost:
        raise                                    # 11-06: do_release 가 올라가서 새 자리로 다시
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
    if color: color=_snap_color(color)       # 어떤 색이든 표준 6색으로 정착
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
    s=re.sub(r'Industry_gyu_arm_\d*_?','',s)   # 프로젝트 접두어
    s=re.sub(r'^\d{8}_?','',s)                 # 날짜 접두어
    s=s.replace('_-_','').lstrip('-_ ')
    return s or str(oid)

def brick_label(bid): return disp_name(bid)
def dock_label(did):  return disp_name(did)

# ===== 말로 만들기: 한국어 명령 -> 블록 =====
# 1단계(지금): 규칙 파서 — 마이크/텍스트로 즉시 동작.
# 2단계(다음): DGX Spark LLM 이 이 자리(h_say)에서 파싱만 대신한다.
_COLOR_WORDS={'#e6194b':['빨강','빨간','레드'],
    '#f58231':['주황','오렌지'],
    '#ffe119':['노랑','노란','옐로'],
    '#3cb44b':['초록','그린'],
    '#4363d8':['파랑','파란','블루'],
    '#911eb4':['보라','퍼플']}
def _color_word(c):
    """hex 색 -> 대표 한국어 이름 ('빨강' 등). 표준 6색 기준."""
    ws=_COLOR_WORDS.get(_snap_color(c) if c else '')
    return ws[0] if ws else ''

# 부팅 시: 예전에 저장된 색을 표준 6색으로 일괄 정착 (한 번만)
try:
    _fixed=0
    for _oid,_d in (S.get('names') or {}).items():
        _c=(_d or {}).get('color')
        if _c and _snap_color(_c)!=_c:
            _d['color']=_snap_color(_c)
            for _e in ENV_SHOW:
                if _e['id']==_oid: _e['color']=_d['color']
            _fixed+=1
    if _fixed:
        print(f'[색 표준화] 저장된 색 {_fixed}개를 무지개 6색으로 정착')
        _macro_save()
except Exception as _e:
    print('[색 표준화] 실패:',_e)

def _speech_vocab():
    """말로 지칭 가능한 사물: [(키워드(공백제거), id, 종류)]. 배치된 것만."""
    out=[]
    nm=_names()
    def add(k,oid,kind):
        k=str(k or '').strip().lower().replace(' ','')
        if k: out.append((k,oid,kind))
    pens=[b for b,v in BRICKS.items()
          if v.get('pos') is not None or HELD.get('brick')==b]
    for bid in pens:
        add(disp_name(bid),bid,'brick'); add(_clone_base(bid),bid,'brick')
        if len(pens)==1:      # '펜' 같은 두루뭉술한 말은 하나뿐일 때만
            add('펜',bid,'brick'); add('부품',bid,'brick'); add('물건',bid,'brick')
        for w in _COLOR_WORDS.get((nm.get(bid) or {}).get('color') or '',[]):
            add(w,bid,'brick'); add(w+'펜',bid,'brick')
    dks=[]
    for did in docking_ids():
        e=_env_ent(did)
        if e is not None and not e.get('visible',True): continue
        if dock_socket_world(did) is None: continue
        dks.append(did)
    for did in dks:
        add(disp_name(did),did,'dock')
        if len(dks)==1: add('도킹',did,'dock')   # '도킹' 단독은 하나뿐일 때만
        if '#' in did:
            n=did.split('#')[-1]
            for w in (f'{n}번도킹',f'도킹{n}번',f'도킹{n}',f'{n}번'):
                add(w,did,'dock')
        for w in _COLOR_WORDS.get((nm.get(did) or {}).get('color') or '',[]):
            add(w,did,'dock'); add(w+'도킹',did,'dock')
    return out

def _msg_of(out):
    """핸들러 반환(문자열 또는 튜플)에서 사람용 메시지만 뽑기."""
    if isinstance(out,str): return out
    if isinstance(out,(list,tuple)):
        for x in reversed(out):
            if isinstance(x,str): return x
    return '완료'

# Spark LLM (2단계): 주소가 설정돼 있으면 규칙 파서보다 먼저 시도한다.
# 예) export ARM_LLM_URL=http://172.21.160.1:11434/v1   (ollama OpenAI 호환)
# 실사용 기록 (파인튜닝용). 말한 것과 LLM 이 준 답을 한 줄씩 쌓는다.
LLM_LOG_FILE=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'llm_usage.jsonl')

# AI 모델 선택: auto(Spark 되면 Gemma, 아니면 규칙) / gemma / rachel(자작·내장) / rule
LLM_CFG={'engine':'auto',
         'url':os.environ.get('ARM_LLM_URL',''),
         'model':os.environ.get('ARM_LLM_MODEL','qwen3:32b')}

def _llm_autodetect():
    """ARM_LLM_URL 미설정이면 ollama 기본 포트(11434)를 로컬/윈도우 쪽에서
    잠깐 찾아본다 (VS Code 포트 전달로 열려 있으면 자동 연결)."""
    if LLM_CFG['url']:
        print(f"[LLM] 설정된 주소 사용: {LLM_CFG['url']} (모델 {LLM_CFG['model']})")
        return
    import urllib.request as _rq, json as _json
    # 우선순위: ① Spark 공용 두뇌(터널 18000) ② 노트북 ollama(11434)
    cands=[('http://localhost:18000','Spark 공용 LLM'),
           ('http://localhost:11434','노트북 ollama')]
    try:
        gw=os.popen("ip route show default 2>/dev/null").read().split()
        if 'via' in gw:
            cands.append((f"http://{gw[gw.index('via')+1]}:11434",'윈도우 쪽'))
    except Exception: pass
    for c,label in cands:
        try:
            j=_json.loads(_rq.urlopen(c+'/v1/models',timeout=0.7).read())
            ids=[x.get('id') or x.get('name','')
                 for x in (j.get('data') or j.get('models') or [])]
            if not ids: continue
            if LLM_CFG['model'] not in ids:
                LLM_CFG['model']=ids[0]          # 설정 모델이 없으면 있는 걸로
            LLM_CFG['url']=c+'/v1'
            print(f"[LLM] {label} 자동 발견: {c} "
                  f"(모델 {os.path.basename(LLM_CFG['model'])}) "
                  "— 말로 만들기가 LLM 으로 동작합니다")
            return
        except Exception: pass
    print('[LLM] Spark 연결 안 됨 — 말로 만들기는 더블 레이첼(내장)로, 그것도 안 되면 규칙 파서 (Spark 포트가 열리면 자동 전환)')
_llm_autodetect()

def _flat_blocks(x):
    """LLM 이 blocks 를 이중으로 포장해도 펴준다 ([{'blocks':[...]}] 등)."""
    out=[]
    if isinstance(x,dict): x=x.get('blocks') or []
    for b in (x or []):
        if isinstance(b,(list,dict)): out+=_flat_blocks(b)
        else: out.append(b)
    return out

def _resolve_item(it):
    """LLM 이 만든 블록을 실제 항목으로 보정 (안전장치 겸용).
    - 아는 원초 블록은 그대로
    - id 가 약식('#8', 'docking_station-5#8', 이름)이어도 유일하게 맞으면
      전체 id 로 바꿔서 반환. 못 맞추면 None (블록 폐기)."""
    if it in (GRIP_OPEN_ITEM,GRIP_CLOSE_ITEM,DOWN_ITEM,UP_ITEM): return it
    if not isinstance(it,str): return None
    if is_arm_item(it):                       # 두 팔 블록 (10-30): 설정에 있는 팔만
        return it if it[len(ARM_PREFIX):] in arms_cfg() else None
    if is_wait_item(it):
        if wait_arm(it) is not None:          # 'A팔 끝날 때까지' (11-03): 설정에 있는 팔만
            return it if wait_arm(it) in arms_cfg() else None
        try:
            w=float(it[len(WAIT_PREFIX):])
            return f"{WAIT_PREFIX}{w:g}" if 0<=w<=600 else None
        except ValueError: return None
    for pre,pool in ((GRASP_PREFIX,list(BRICKS)),
                     (RELEASE_PREFIX,docking_ids()),
                     (GOTO_PREFIX,list(BRICKS)+[e['id'] for e in ENV_SHOW])):
        if it.startswith(pre):
            oid=it[len(pre):].strip()
            if oid in pool: return pre+oid
            # LLM 이 목록에 보여준 설명까지 통째로 복사할 때가 있다.
            #   예: "@grasp:ppap (이름:ppap, 색:초록, ...#5 에 결합됨)"
            # 괄호 앞까지만 잘라 한 번 더 맞춰본다 (없으면 그대로).
            if oid not in pool and '(' in oid:
                oid=oid.split('(')[0].strip()
                if oid in pool: return pre+oid
            c=[x for x in pool
               if x.endswith(oid) or disp_name(x)==oid or oid in x]
            return pre+c[0] if len(c)==1 else None
    return None

_LLM_RETRY=[0.0]

def _llm_expand(js):
    """파인튜닝 모델이 쓰는 짧은 별명을 실제 id 로 되돌린다.

    실제 도킹 id 는 48자라 그대로 뱉게 하면 답이 3배 길어지고,
    작은 기기(라떼판다 등)에서 그만큼 느려진다. 그래서 "d11" 처럼
    짧게 배우게 하고 여기서 펼친다. 원래 형식(전체 id)도 그대로 통과한다."""
    if not isinstance(js,dict): return js
    try:
        alias={}
        for d in docking_ids():
            n=disp_name(d).split('#')[-1]
            # 'd7' '#7' 뿐 아니라 맨 숫자 '7' 도 받는다 — LLM 이 "@goto:7" 처럼
            # 번호만 쓰는 일이 잦은데, 그러면 부분일치로 여러 개에 걸려 폐기됐다.
            alias[f'd{n}']=d; alias[f'#{n}']=d; alias.setdefault(n,d)
        for b in list(BRICKS):
            alias[_clone_base(b)]=b; alias[b]=b
        out=[]
        # 중첩([{'blocks':[...]}])도 여기서 먼저 펴 준다 — 예전에는 문자열이
        # 아니면 그냥 버려서, LLM 이 한 번 포장하면 전부 사라졌다.
        for blk in _flat_blocks(js.get('blocks')):
            if not isinstance(blk,str): continue
            for pre in (GOTO_PREFIX,GRASP_PREFIX,RELEASE_PREFIX):
                if blk.startswith(pre):
                    tgt=blk[len(pre):]
                    blk=pre+alias.get(tgt,tgt)     # 모르면 그대로 둔다
                    break
            # ★ 잡기 직전은 반드시 '그 물체' 로 가는 이동이어야 한다 ★
            #   LLM 은 물체가 옮겨진 걸 추적 못해서, 2단계 이송이면 두 번째
            #   잡기 앞에 엉뚱한 도킹 @goto 를 넣는다 (펜은 딴 데 있는데).
            #   물체 @goto 는 실행 순간 물체 위치로 가므로 항상 맞다.
            #   -> 직전 블록이 그 물체 @goto 가 아니면 하나 끼워 넣는다.
            #   (프롬프트로 지시해도 안 지켜서, 계산으로 보장한다)
            if blk.startswith(GRASP_PREFIX):
                need=GOTO_PREFIX+blk[len(GRASP_PREFIX):]
                if out and out[-1].startswith(GOTO_PREFIX) and out[-1]!=need:
                    out[-1]=need       # 도킹으로 가던 이동을 물체 이동으로 교체
                elif not out or out[-1]!=need:
                    out.append(need)   # 이동이 아예 없으면 하나 끼워 넣는다
            out.append(blk)
        js['blocks']=out
    except Exception as e:
        print('별명 펼치기 실패:',e)
    return js

# ===== RACHEL — 자작 소형 LLM (nano_llm/, 내장 실행) =====
#   Gemma 와 달리 서버가 필요 없다. 38MB 체크포인트를 프로세스 안에서 돌린다.
#   색→도킹 조회는 신경망이 아니라 hints.build()(결정론 전처리)가 한다 —
#   같은 크기에서 조회를 모델에 맡기면 64%, 계산으로 빼면 99.6% (실측).
RACHEL_ST={'m':None,'stoi':None,'itos':None,'err':''}
_RACHEL_LOCK=threading.Lock()

def _rachel_load():
    """RACHEL 로드. int8 양자화본이 있으면 그쪽을 먼저 쓴다.
       실측(4코어): fp32 1,654ms·38MB / int8 1,265ms·19.5MB, 정확도 차이 0%.
       int8 은 Linear 층 구조가 달라 빈 모델을 먼저 양자화한 뒤 가중치를 넣는다."""
    if RACHEL_ST['m'] is not None: return True
    if RACHEL_ST['err']: return False
    try:
        import torch
        from nano_llm import model as _rm, tokenizer as _rt
        d=os.path.join(os.path.dirname(os.path.abspath(__file__)),'nano_llm')
        p8=os.path.join(d,'rachel_int8.pt'); p32=os.path.join(d,'rachel.pt')
        path=p8 if os.path.exists(p8) else p32
        ck=torch.load(path,map_location='cpu')
        itos=ck['vocab']; m=_rm.Rachel(len(itos))
        if ck.get('int8'):
            m=torch.quantization.quantize_dynamic(
                m,{torch.nn.Linear},dtype=torch.qint8)
        m.load_state_dict(ck['model']); m.eval()
        RACHEL_ST.update(m=m,stoi={c:i for i,c in enumerate(itos)},itos=itos)
        print(f"[RACHEL] 로드됨 (10M 파라미터, CPU"
              f"{', int8 양자화' if ck.get('int8') else ''})")
        return True
    except Exception as e:
        RACHEL_ST['err']=str(e); print('[RACHEL] 로드 실패:',e); return False

def _rachel_scene():
    """학습 형식 그대로 장면 문자열을 만든다: '빨강=d8 주황=d9 | 펜=초록@d8'."""
    cmap={}                                   # 별명(dN) -> 표준 색이름
    for d in docking_ids():
        e=_env_ent(d)
        if e is not None and not e.get('visible',True): continue
        n=disp_name(d).split('#')[-1]
        if not n.isdigit(): continue
        w=_color_word((_names().get(d) or {}).get('color'))
        if w: cmap[f'd{n}']=w
    if not cmap: return None
    bid=next((b for b,v in BRICKS.items() if v.get('pos') is not None),None)
    if bid is None: return None
    pcol=_color_word((_names().get(bid) or {}).get('color')) or '초록'
    home=None; p=BRICKS[bid].get('pos')
    for d in docking_ids():                   # 펜이 결합된 도킹 = 출발지
        t=dock_socket_world(d)
        if p is not None and t is not None and \
           math.hypot(p[0]-t[0],p[1]-t[1])<0.025:
            n=disp_name(d).split('#')[-1]
            if f'd{n}' in cmap: home=f'd{n}'
            break
    if home is None: home=next(iter(cmap))
    hdr=' '.join(f'{c}={d}' for d,c in cmap.items())
    return cmap,home,pcol,f'{hdr} | 펜={pcol}@{home}'

# ★ C++ 추론기 우선 ★
#   같은 모델·같은 출력인데 14배 빠르고(1,265ms -> 91ms) 의존성이 없다.
#   rachel_cpp/rachel(_static) 과 rachel.bin 이 있으면 그걸 쓰고,
#   없으면 torch 경로로 되돌아간다.
RACHEL_CPP=[None]     # (실행파일 경로, 가중치 경로) 또는 False

def _rachel_cpp_paths():
    if RACHEL_CPP[0] is not None: return RACHEL_CPP[0]
    d=os.path.join(os.path.dirname(os.path.abspath(__file__)),'rachel_cpp')
    w=os.path.join(d,'rachel.bin')
    # rachel(-march=native) 이 rachel_static 보다 100ms 빠르다 — 이걸 먼저 본다.
    # static 은 다른 기기로 옮길 때의 예비책 (의존성 없음).
    for exe in ('rachel','rachel_static'):
        p=os.path.join(d,exe)
        if os.path.exists(p) and os.access(p,os.X_OK) and os.path.exists(w):
            RACHEL_CPP[0]=(p,w); print(f'[RACHEL] C++ 추론기 사용: {exe}')
            return RACHEL_CPP[0]
    RACHEL_CPP[0]=False; return False

# 상주 프로세스 — 모델을 한 번만 읽고 계속 쓴다.
#   매번 새로 실행하면 프로세스 시작 + 9.6MB 로드가 붙어 100ms 를 손해본다.
RACHEL_SRV=[None]                      # subprocess.Popen 또는 None
_RACHEL_SRV_LOCK=threading.Lock()

def _rachel_srv():
    """상주 프로세스를 얻는다 (없으면 띄운다). 실패하면 None."""
    p=RACHEL_SRV[0]
    if p is not None and p.poll() is None: return p
    paths=_rachel_cpp_paths()
    if not paths: return None
    exe,w=paths
    try:
        import subprocess
        p=subprocess.Popen([exe,w,'--serve'],stdin=subprocess.PIPE,
                           stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
        if p.stdout.readline().strip()!=b'READY':
            p.kill(); return None
        RACHEL_SRV[0]=p
        print('[RACHEL] C++ 상주 프로세스 기동')
        return p
    except Exception as e:
        print('[RACHEL] 상주 기동 실패:',e); return None

def _rachel_cpp_run(prompt):
    """C++ 추론기 실행 (상주 우선, 실패 시 1회성 실행)."""
    paths=_rachel_cpp_paths()
    if not paths: return None
    exe,w=paths
    with _RACHEL_SRV_LOCK:
        p=_rachel_srv()
        if p is not None:
            try:
                p.stdin.write(prompt.replace('\n','\x01').encode()+b'\n')
                p.stdin.flush()
                out=p.stdout.readline().decode('utf-8','replace').strip()
                if out: return out
            except Exception as e:
                print('[RACHEL] 상주 통신 실패 -> 재기동:',e)
                try: p.kill()
                except Exception: pass
                RACHEL_SRV[0]=None
    try:                                    # 예비: 1회성 실행
        import subprocess
        r=subprocess.run([exe,w,prompt],capture_output=True,timeout=30)
        return r.stdout.decode('utf-8','replace').strip() or None
    except Exception as e:
        print('[RACHEL] C++ 실행 실패:',e); RACHEL_CPP[0]=False; return None

def _rachel_parse(t):
    """RACHEL 로 문장 해석. 실패하면 None (규칙 파서로)."""
    if not _rachel_cpp_paths() and not _rachel_load(): return None
    sc=_rachel_scene()
    if sc is None: return None
    import json as _json
    from nano_llm import hints as _rh
    try: import torch; from nano_llm import tokenizer as _rt
    except Exception: torch=None; _rt=None
    cmap,home,pcol,hdr=sc
    # ★ 문장 안 줄바꿈 제거 (2026-10-20) ★ 입력은 '장면/힌트/문장/>' 를 줄로 나누므로,
    #   음성·붙여넣기로 들어온 줄바꿈이 남으면 문장이 두 줄로 쪼개져 엉뚱한 답이 나온다
    #   (실측: "…주황색으로\n 옮기는 걸" -> 빨강을 건너뜀). 학습·검증 데이터는 한 줄이다.
    t=' '.join(str(t).split())
    h=_rh.build(cmap,home,pcol,t)             # 색·번호 조회는 계산이 한다
    prompt=(f'{hdr}\n힌트 {h}' if h else hdr)+f'\n{t}\n>'
    t0=time.time()
    use_cpp = LLM_CFG.get('engine') != 'rachel_py'   # 파이썬 강제 선택 시 건너뜀
    txt=_rachel_cpp_run(prompt) if use_cpp else None
    if txt is None:                       # C++ 이 없거나 실패 -> torch 경로
        if not _rachel_load(): return None
        with _RACHEL_LOCK:
            ids=torch.tensor([_rt.encode(RACHEL_ST['stoi'],prompt)])
            out=RACHEL_ST['m'].generate(ids,_rt.EOS)
        txt=_rt.decode(RACHEL_ST['itos'],out[0,ids.size(1):].tolist()).strip()
    try:
        js=_json.loads(txt[txt.find('{'):txt.rfind('}')+1])
    except Exception:
        # 글자 단위 생성이라 드물게 JSON 틀이 깨진다 (실측: ':[' 누락).
        # 내용물(@블록)은 정규식으로 완전 복구된다 — 형식 수리도 계산이 한다.
        import re as _re
        blks=_re.findall(r'@(?:goto|grasp|release):[A-Za-z0-9_#\-]+'
                         r'|@open|@close|@down|@up',txt)
        if not blks:
            print('[RACHEL] JSON 아님:',txt[:80]); return None
        js={'blocks':blks}
        for k in ('clear','stop'):
            if _re.search(rf'"{k}"\s*:?\s*true',txt): js[k]=True
        m=_re.search(r'"speed"\s*:?\s*"([0-9.]+x)"',txt)
        if m: js['speed']=m.group(1)
        print('[RACHEL] JSON 수리 ->',len(blks),'블록')
    js=_llm_expand(js)                        # dN -> 실제 id
    _way='C++' if (use_cpp and _rachel_cpp_paths()) else 'torch'
    LOG(f'  [RACHEL/{_way}] {(time.time()-t0)*1000:.0f}ms: {txt[:100]}')
    try:
        with open(LLM_LOG_FILE,'a',encoding='utf-8') as _f:
            _f.write(_json.dumps({'engine':'rachel','text':t,'answer':js},
                                 ensure_ascii=False)+'\n')
    except Exception: pass
    return js if isinstance(js,dict) else None

# ===== 더블 레이첼 (nano_double/, 2026-10-30) — 두 팔 · 지저분한 말 → 정리문 + 블록 =====
#   C++ 엔진은 v9 와 같은 rachel_cpp/rachel 을 쓰고 가중치만 nano_double/double.bin.
#   출력은 검사(nano_double/check.py)를 통과해야만 블록이 된다 — 못 믿으면 None (규칙 파서로).
RACHEL2=[None]
def _rachel2_parse(t):
    sc=_rachel_scene()
    if sc is None: return None
    cmap,home,pcol,_=sc
    here=os.path.dirname(os.path.abspath(__file__))
    w=os.path.join(here,'nano_double','double.bin')
    paths=_rachel_cpp_paths()
    if not paths or not os.path.exists(w): return None
    if os.path.join(here,'nano_double') not in sys.path: sys.path.insert(0,os.path.join(here,'nano_double'))
    try:
        from rachel2 import Rachel2
    except Exception as e:
        print('[더블 레이첼] 불러오기 실패:',e); return None
    docks={}
    for d,c in cmap.items(): docks.setdefault(c,d)      # 같은 색 도킹이 둘이면 앞의 것
    scn=dict(arms=dict(arms_cfg()),docks=docks,pen=pcol,pen_home=home)
    t0=time.time()
    with _RACHEL_SRV_LOCK:
        try:
            if RACHEL2[0] is None or RACHEL2[0].p.poll() is not None:
                RACHEL2[0]=Rachel2(paths[0],w)
            res=RACHEL2[0].ask(scn,' '.join(str(t).split()),fallback=6)   # 검사에 걸리면 6개 더 뽑아 다수결
        except Exception as e:
            print('[더블 레이첼] 실행 실패:',e); RACHEL2[0]=None; return None
    LOG(f"  [더블 레이첼] {(time.time()-t0)*1000:.0f}ms: {res['canon'][:120]}"
        +('' if res['ok'] else f"  ✗ 검사 걸림: {res['why']}"))
    try:
        import json as _json
        with open(LLM_LOG_FILE,'a',encoding='utf-8') as _f:
            _f.write(_json.dumps({'engine':'rachel2','text':t,'canon':res['canon'],'ok':res['ok'],
                                  'why':res['why'],'blocks':res['blocks']},ensure_ascii=False)+'\n')
    except Exception: pass
    if not res['ok']: return None
    app=res['app'] or []
    js={'blocks':[b for b in app if b not in ('@stop','@clear','@preview','@faster','@slower')]}
    if '@stop' in app: js['stop']=True
    if '@clear' in app: js['clear']=True
    if '@preview' in app: js['preview']=True
    if '@faster' in app: js['speed']='3.0x'
    if '@slower' in app: js['speed']='0.5x'
    if not app: js['say']='명령이 아닌 말로 알아들었습니다'
    return _llm_expand(js)

_LLM_ENGINES={'auto':'자동 (Spark 되면 Gemma, 아니면 더블 레이첼, 그것도 안 되면 규칙)',
              'rachel2':'더블 레이첼 (두 팔 · 자작 25.6M · 검사 통과만)',
              'gemma':'Gemma 4 (Spark 파인튜닝)',
              'rachel':'RACHEL C++ (자작 10M · 134ms · 9.6MB)',
              'rachel_py':'RACHEL 파이썬 (torch · 1,265ms · 비교용)',
              'rule':'규칙 파서 (AI 미사용)'}
def h_llm_engine(sel):
    """말로 만들기의 AI 모델 선택."""
    eng=str(sel or 'auto').strip()
    if eng not in _LLM_ENGINES: eng='auto'
    BTN(f'AI 모델 {eng}')
    LLM_CFG['engine']=eng
    if eng=='rachel':
        p=_rachel_cpp_paths()
        if not p:
            return ('RACHEL C++ 실행파일이 없습니다 — rachel_cpp/ 에서 빌드하세요:\n'
                    '  g++ -O3 -march=native -ffast-math -fopenmp -w -o rachel rachel.cpp')
        return f'AI 모델: {_LLM_ENGINES[eng]}  ({os.path.basename(p[0])})'
    if eng=='rachel2':
        _w=os.path.join(os.path.dirname(os.path.abspath(__file__)),'nano_double','double.bin')
        if not _rachel_cpp_paths() or not os.path.exists(_w):
            return '더블 레이첼 가중치(nano_double/double.bin) 또는 C++ 실행파일이 없습니다 — 규칙 파서로 동작합니다'
        if _rachel_scene() is None:
            return '더블 레이첼 선택됨 — 단, 번호 있는 도킹과 펜을 먼저 배치해야 동작합니다'
        return f"AI 모델: {_LLM_ENGINES[eng]} · 팔 {' · '.join(f'{k}={v}' for k,v in arms_cfg().items())}"
    if eng=='rachel_py':
        if not _rachel_load():
            return f"RACHEL 로드 실패: {RACHEL_ST['err'][:80]} — 규칙 파서로 동작합니다"
        if _rachel_scene() is None:
            return 'RACHEL 선택됨 — 단, 번호 있는 도킹과 펜을 먼저 배치해야 동작합니다'
    if eng=='gemma' and not LLM_CFG.get('url'):
        return 'Gemma 선택됨 — Spark 연결이 아직 없어 붙는 대로 사용됩니다'
    return f'AI 모델: {_LLM_ENGINES[eng]}'

def _llm_parse(t):
    """문장 해석. engine 설정에 따라 RACHEL(내장)/Gemma(Spark)/규칙으로 간다.
    Spark LLM 에게 문장 해석을 맡긴다. 미설정/실패면 None (규칙 파서로).

    ★ 끊겼다 살아나면 다시 잡는다 ★
      예전에는 앱이 켜질 때 딱 한 번만 찾았다. 그래서 그때 Spark 터널이
      죽어 있으면 그 뒤로 영영 규칙 파서만 썼다(터널을 되살려도 소용없음).
      -> 주소가 없거나 연결이 끊겼으면 30초마다 다시 찾아본다."""
    eng=LLM_CFG.get('engine','auto')
    if eng=='rule':   return None            # AI 끄기 — 규칙 파서만
    if eng in ('rachel','rachel_py'): return _rachel_parse(t)
    if eng=='rachel2': return _rachel2_parse(t)
    url=LLM_CFG.get('url')
    if not url:
        now=time.time()
        if now-_LLM_RETRY[0]>30.0:      # 너무 자주 찾지는 않는다
            _LLM_RETRY[0]=now
            try: _llm_autodetect()
            except Exception: pass
            url=LLM_CFG.get('url')
        if not url:
            # ★ Spark 가 없으면 '자동' 은 내장 더블 레이첼로 (11-04) ★ 라떼판다엔 Spark 가 없어 '자동' 이 늘 규칙 파서였다
            #   ('비팔로 … 옮겼다가 다시 …' 에서 B팔·둘째 목적지가 빠짐). 더블 레이첼도 못 쓰면 None -> 규칙 파서.
            return _rachel2_parse(t) if eng=='auto' else None
    import json as _json, urllib.request as _rq
    def _pen_where(b):
        p=BRICKS[b].get('pos')
        for d in docking_ids():
            t=dock_socket_world(d)
            if p and t and math.hypot(p[0]-t[0],p[1]-t[1])<0.025:
                return f'{disp_name(d)} 에 결합됨'
        return '단독 배치'
    def _cw(oid):
        w=_color_word((_names().get(oid) or {}).get('color'))
        return f'색:{w}, ' if w else ''
    pens=[(b,disp_name(b),_pen_where(b)) for b,v in BRICKS.items()
          if v.get('pos') is not None]
    dks=[(d,disp_name(d),'물체 있음' if dock_occupied(d) else '비어있음')
         for d in docking_ids()
         if (_env_ent(d) is None or _env_ent(d).get('visible',True))]
    sysmsg=(
        '너는 로봇팔 블록코딩 앱의 통역이다. 사용자 한국어 명령을 JSON 하나로만 답해라.\n'
        f'물체(잡기 대상): {[f"{i} (이름:{n}, {_cw(i)}{w})" for i,n,w in pens]}\n'
        f'도킹(놓기 대상): {[f"{i} (이름:{n}, {_cw(i)}{w})" for i,n,w in dks]}\n'
        '호칭 규칙: 물체는 흔히 "펜/부품/그거/저거" 라고 부른다 — 물체가 '
        '하나뿐이면 그 물체로 확정해라. 도킹 id 의 "#N" 은 "N번(자리/도킹)" 을 '
        '뜻한다 (예: "8번 자리" = #8 도킹). 색깔 이름으로 부르기도 한다.\n'
        '블록 형식: "@goto:<id>"(이동) "@grasp:<물체id>"(잡기=열기·하강·닫기·상승) '
        '"@release:<도킹id>"(놓기) "@open" "@close" "@down"(하강) "@up"(상승)\n'
        '잡기 전에는 반드시 "@goto:<그 물체id>" 를, 놓기 전에는 "@goto:<그 도킹id>" 를 '
        '먼저 넣어라. 물체 @goto 는 실행 순간 물체가 있는 곳으로 가므로, 물체가 '
        '중간에 옮겨져도 항상 맞다 — 잡기 앞에는 도킹이 아니라 물체 id 로 가라.\n'
        'id 는 목록 각 줄에서 맨 앞의 id 부분만 써라 — 괄호 안 설명(이름/색/상태)은 '
        '절대 붙이지 마라. 예: "@grasp:ppap" (O) / "@grasp:ppap (이름:ppap, 색:초록)" (X)\n'
        'blocks 는 문자열만 담은 평평한 배열이다. 배열 안에 객체나 배열을 넣지 마라.\n'
        'JSON 키: blocks(블록 배열, 없으면 []), clear(파이프라인 비우기 bool), '
        'preview(미리보기 실행 bool), stop(정지 bool), speed("0.5x"~"4.0x" 중 하나 또는 null), '
        'say(사용자에게 할 한국어 한 줄)\n'
        '모르는 사물/불가능한 요청이면 blocks=[] 로 두고 say 로 되물어라. '
        '실물 실행은 절대 하지 마라. JSON 외 다른 텍스트 금지.')
    # ★ 사고(thinking) 를 끈다 ★
    #   Gemma 4 는 답하기 전에 혼자 길게 생각하고, 그 내용을 content 가 아니라
    #   reasoning_content 에 담는다. 2단계 이송처럼 조금만 복잡해지면 생각하다
    #   토큰을 다 써 finish_reason='length' 로 잘리고 content 는 '' 가 된다
    #   -> JSON 파싱 실패 -> 규칙 파서로 넘어가 "말만 하고 블록은 안 만듦".
    #   (실측: 사고 켬 = 14초·블록 0개 / 사고 끔 = 4.3초·블록 8개)
    #   프롬프트로 "생각하지 마라" 라고 해도 안 통한다. 이 옵션만 듣는다.
    body={'model':LLM_CFG.get('model'),'temperature':0,'max_tokens':900,
          'chat_template_kwargs':{'enable_thinking':False},
          'messages':[{'role':'system','content':sysmsg},
                      {'role':'user','content':t}]}
    try:
        req=_rq.Request(url.rstrip('/')+'/chat/completions',
                        data=_json.dumps(body).encode(),
                        headers={'Content-Type':'application/json'})
        out=_json.loads(_rq.urlopen(req,timeout=90).read())
        txt=out['choices'][0]['message']['content']
        js=_json.loads(txt[txt.find('{'):txt.rfind('}')+1])
        js=_llm_expand(js)          # 짧은 별명(d11) -> 실제 id
        # 실사용 기록을 남긴다 — 나중에 이걸로 파인튜닝하면 사장님 말투를
        # 그대로 배운다 (자동 생성 데이터보다 훨씬 값지다).
        try:
            with open(LLM_LOG_FILE,'a',encoding='utf-8') as _f:
                _f.write(_json.dumps({'text':t,'answer':js},
                                     ensure_ascii=False)+'\n')
        except Exception: pass
        return js if isinstance(js,dict) else None
    except Exception as e:
        print(f'[LLM] 사용 불가 -> 규칙 파서로 대체: {e}')
        # 연결이 끊긴 것이면 주소를 비워, 다음에 다시 찾게 한다
        if 'refused' in str(e) or 'timed out' in str(e) or 'reset' in str(e):
            LLM_CFG['url']=''
        return None

def h_say(text):
    """말/텍스트 명령 -> 블록 추가 / 앱 조작.
    반환: ({'preview':bool,'added':n,'speed':...,'run':...}, 상태 메시지)"""
    t=str(text or '').strip()
    if not t: return {'preview':False,'added':0},'빈 명령입니다'
    low=t.lower().replace(' ','')          # 띄어쓰기 편차 흡수
    def has(*ws): return any(w in low for w in ws)

    # ⓪ Spark LLM 이 연결돼 있으면 먼저 해석을 맡긴다 (실패 시 아래 규칙으로)
    js=_llm_parse(t)
    if js is not None:
        r={'preview':bool(js.get('preview')),'added':0}
        msgs=[]
        if js.get('stop'): h_stop(); msgs.append('정지')
        if js.get('clear'):
            _undo_snapshot('말로 만들기: 비우기')
            with LOCK: S['seq']=[]
            _macro_save(); msgs.append('파이프라인 비움')
        blocks=[x for x in (_resolve_item(b) for b in _flat_blocks(js.get('blocks')))
                if x is not None]
        if blocks:
            _undo_snapshot('말로 만들기(LLM)')
            with LOCK: S['seq'].extend(blocks)
            _macro_save()
            r['added']=len(blocks); r['preview']=True
            msgs.append(f'블록 {len(blocks)}개 추가')
        if js.get('speed') in ('0.5x','1.0x','1.5x','2.0x','3.0x','4.0x'):
            r['speed']=js['speed']; msgs.append(f"속도 {js['speed']}")
        say=str(js.get('say') or '').strip()
        return r,(say if say else (' · '.join(msgs) or '알겠습니다'))

    # ① 파이프라인 비우기
    if has('지우','지워','삭제','비우','비워','초기화','처음부터','리셋'):
        _undo_snapshot('말로 만들기: 비우기')
        with LOCK: S['seq']=[]
        _macro_save()
        return {'preview':False,'added':0},'파이프라인을 비웠습니다 (Ctrl+Z 로 되돌리기)'

    # ①-2 앱 조작 명령 (프로그램 자체를 말로)
    if has('멈춰','정지','스톱'):
        h_stop()
        return {'preview':False,'added':0},'정지했습니다'
    if has('되돌려','되돌리','취소','언두'):
        return {'preview':False,'added':0},_msg_of(undo_last())
    if has('배속','속도','느리게','빠르게','천천히'):
        pick=None
        for s2 in ('0.5x','1.0x','1.5x','2.0x','3.0x','4.0x'):
            base=s2.replace('.0','').replace('x','')      # 0.5 / 1 / 1.5 ...
            if any(p in low for p in (s2,base+'x',base+'배속',base+'배')):
                pick=s2; break
        if pick is None:
            pick='0.5x' if has('느리게','천천히') else \
                 ('3.0x' if has('빠르게') else None)
        if pick is None:
            return {'preview':False,'added':0}, \
                '속도는 0.5 / 1 / 1.5 / 2 / 3 / 4 배속 중에서 말해주세요'
        return {'preview':False,'added':0,'speed':pick}, \
            f'재생 속도를 {pick} 로 바꿨습니다'
    pres=list((S.get('factory_presets') or {}).keys())
    pname=next((p for p in pres if p.lower().replace(' ','') in low),None)
    if pname and has('불러','열어','로드','배치해','공정으로'):
        return {'preview':False,'added':0},_msg_of(h_factory_load(pname))
    if has('불러') and has('공장','배치','공정','프리셋'):
        return {'preview':False,'added':0}, \
            f"어떤 배치요? 저장된 것: {', '.join(pres) if pres else '없음'}"
    if has('저장') and has('공장','배치','공정','프리셋'):
        if pname:
            return {'preview':False,'added':0},_msg_of(h_factory_save(pname))
        return {'preview':False,'added':0}, \
            '저장 이름까지 말해주세요 (예: "지금 배치를 1번공정으로 저장")'
    if has('그자리로','추천자리로','추천대로','자리적용','추천적용'):
        return {'preview':False,'added':0},_msg_of(h_robot_apply_suggest())
    if has('자리추천','자리를추천','추천해'):
        return {'preview':False,'added':0},_msg_of(h_robot_suggest())
    if has('실물') and has('실행','시작','돌려'):
        if has('확인'):
            return {'preview':False,'added':0,'run':True},'실물 실행을 시작합니다'
        return {'preview':False,'added':0}, \
            '실물 로봇이 움직입니다! 정말이면 "실물 실행 확인" 이라고 말해주세요'

    # 사물 언급 찾기 (긴 키워드 우선, 겹침 제거, 등장 순서 유지)
    ment=[]
    for kw,oid,kind in _speech_vocab():
        p=low.find(kw)
        while p>=0:
            ment.append((p,len(kw),oid,kind)); p=low.find(kw,p+len(kw))
    ment.sort(key=lambda m:(m[0],-m[1]))
    used=[False]*len(low); filt=[]
    for p,L,oid,kind in ment:
        if any(used[p:p+L]): continue
        for i in range(p,p+L): used[i]=True
        filt.append((p,oid,kind))
    filt.sort()
    seen=set(); order=[]
    for p,oid,kind in filt:
        if oid in seen: continue
        seen.add(oid); order.append((p,oid,kind))
    pens=[o for _,o,k in order if k=='brick']
    docks=[o for _,o,k in order if k=='dock']
    pen0=pens[0] if pens else next((b for b,v in BRICKS.items()
                                    if v.get('pos') is not None),None)
    dock0=docks[0] if docks else None
    # "N번" 을 말했는데 그 번호 도킹이 없으면 추측하지 말고 되묻는다
    # (도킹 관련 동사가 있을 때만 — "1번공정 불러와" 같은 말은 해당 없음)
    want_no=re.findall(r'(\d+)\s*번',t)
    if want_no and not docks and has('꽂','넣','옮','놓','도킹','잡','이동'):
        have=sorted(d.split('#')[-1] for d in docking_ids() if '#' in d
                    and (_env_ent(d) is None or _env_ent(d).get('visible',True)))
        return {'preview':False,'added':0}, \
            (f"{want_no[0]}번 도킹은 없습니다 (지금 있는 것: "
             f"{', '.join(have)}번)")

    # ② 미리보기만 (다른 동작 동사가 없을 때)
    act_verb=has('잡','옮','꽂','넣','놓','열','닫','내려','내리','하강',
                 '올려','올리','상승','이동','로가','으로가','가줘','위로')
    if has('미리보','재생','돌려','보여') and not act_verb:
        return {'preview':True,'added':0},'미리보기를 시작합니다'

    blocks=[]; notes=[]
    if has('옮','꽂','넣'):                # ③ 옮기기 = 이동+잡기+이동+놓기
        if pen0 is None: return {'preview':False,'added':0}, \
            '옮길 물체를 찾지 못했습니다 — 이름이나 색으로 불러주세요'
        if dock0 is None: return {'preview':False,'added':0}, \
            '꽂을 도킹을 찾지 못했습니다 — 예: "3번 도킹에 꽂아"'
        blocks=([] if HELD.get('brick')==pen0 else
                [GOTO_PREFIX+pen0,GRASP_PREFIX+pen0]) \
               +[GOTO_PREFIX+dock0,RELEASE_PREFIX+dock0]
        notes.append(f'{disp_name(pen0)} → {disp_name(dock0)}')
    elif has('잡'):                        # ④ 잡기 (= 이동+잡기)
        if pen0 is None: return {'preview':False,'added':0}, \
            '잡을 물체를 찾지 못했습니다'
        blocks=[GOTO_PREFIX+pen0,GRASP_PREFIX+pen0]
        notes.append(f'{disp_name(pen0)} 잡기')
    elif has('놓'):                        # ⑤ 놓기 (= 이동+놓기)
        if dock0 is None: return {'preview':False,'added':0}, \
            '놓을 도킹을 찾지 못했습니다 — 예: "3번 도킹에 놓아"'
        blocks=[GOTO_PREFIX+dock0,RELEASE_PREFIX+dock0]
        notes.append(f'{disp_name(dock0)} 에 놓기')
    else:                                  # ⑥ 원초 블록들 (말한 순서대로)
        prim=[]
        for ws,item in ((('열',),GRIP_OPEN_ITEM),(('닫',),GRIP_CLOSE_ITEM),
                        (('내려','내리','하강'),DOWN_ITEM),
                        (('올려','올리','올라','상승'),UP_ITEM)):
            for w in ws:
                p=low.find(w)
                if p>=0: prim.append((p,item)); break
        if has('이동','로가','으로가','에가','가줘','가라','위로','올라가'):
            for p,o,k in order:            # "3번 도킹으로 가" 같은 이동
                prim.append((p,GOTO_PREFIX+o))
        prim.sort()
        blocks=[it for _,it in prim]
        if blocks: notes.append('말한 순서대로')
    if not blocks:
        return {'preview':False,'added':0}, \
            ('무슨 동작인지 못 알아들었습니다. 예: "펜 잡아서 3번 도킹에 꽂아", '
             '"그리퍼 열어", "하강", "파이프라인 지워"')

    _undo_snapshot('말로 만들기')
    with LOCK: S['seq'].extend(blocks)
    _macro_save()
    return {'preview':True,'added':len(blocks)}, \
        (f"블록 {len(blocks)}개 추가 ({', '.join(notes)}) → 바로 미리보기")

def move_env(oid,x,y,snap=True):
    """설비(STL)를 월드 (x,y) 로 이동. snap=True 면 20mm 격자 스냅,
    False 면 마우스가 멈춘 정확히 그 자리 (드롭 직후 - 핀 선택이 정렬 담당).
    플레이트는 이동 금지."""
    if _is_floor_plate(oid): return False,'플레이트는 더블클릭 확장으로만'
    ent=None
    for e in ENV_SHOW:
        if e['id']==oid: ent=e; break
    if ent is None: return False,f'설비를 찾을 수 없습니다: {oid}'
    path,scale=MESH_REG[oid]
    try:
        mm=trimesh.load(path)
        # 뷰어 고스트와 같은 '바운딩박스 중심' 기준 (정점 평균을 쓰면
        # 삼각형 밀도에 따라 중심이 어긋나 놓는 순간 메쉬가 튄다)
        v=np.asarray(mm.vertices,float)
        c=(v.min(axis=0)+v.max(axis=0))/2.0*scale
    except Exception as e:
        return False,f'메쉬 읽기 실패: {e}'
    yaw=float(ent.get('yaw',0.0))
    cw=_rz2(yaw)@np.asarray(c[:2],float)           # 회전 반영한 중심
    off=np.array([x-cw[0],y-cw[1]])
    if snap:
        off=np.round(off/PEG_PITCH)*PEG_PITCH      # 20mm 스냅 = 목심 정렬 유지
    set_env_pose(oid,off[0],off[1],yaw)
    return True,(f"{disp_name(oid)} 이동 -> 오프셋 "
                 f"({off[0]*1000:+.0f},{off[1]*1000:+.0f})mm (목심 정렬)")

def _peg_layout_changed():
    """플레이트 배치가 바뀌면 목심 목록이 바뀐다. 도달표는 상대좌표라
    그대로 유효 - 표시 갱신만 필요하다."""
    _PEG_CACHE.clear()
    PEG_VIEW['dirty']=True

def add_plate(tile):
    """빈 타일에 플레이트 추가 (원본 STL을 오프셋만 다르게 복제)."""
    if tile in PLATES or PLATE_SRC is None: return False
    ix,iy=tile
    mid=f"plate_tile_{ix}_{iy}"
    k=2; base=mid
    _shown={e['id'] for e in ENV_SHOW}
    # 화면에 없는 옛 등록(프리셋을 불러와 빠진 판)은 같은 이름을 다시 쓴다 — 불러올 때마다 _2, _3 … 로 늘지 않게 (11-04)
    while mid in _shown or (mid in MESH_REG and MESH_REG[mid][0]!=PLATE_MESH_PATH): mid=f"{base}_{k}"; k+=1
    # 원본 플레이트 STL의 현재 원점 -> 목표 타일 원점으로 평행이동
    b=_plate_bounds_m(PLATE_MESH_PATH)
    tox,toy=tile_origin(tile)
    off=(tox-b[0][0], toy-b[0][1], 0.0)
    MESH_REG[mid]=(PLATE_MESH_PATH,0.001)
    ENV_SHOW.append({'id':mid,'color':plate_color(tile),'visible':True,
                     'tile':tile,'offset':off,'added':True})
    PLATES[tile]={'id':mid,'origin':(tox,toy),'added':True}
    with LOCK:                                  # 영구 저장 (재시작 유지)
        pl=S.setdefault('plates_added',[])
        if list(tile) not in pl: pl.append(list(tile))
    _macro_save()
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
    with LOCK:
        S['plates_added']=[t for t in S.get('plates_added',[])
                           if tuple(t)!=tuple(tile)]
    _macro_save()
    _peg_layout_changed()
    return True

# 부팅 시: 기본 바닥판 2×4 로 맞춘다 (사용자 11-03 — 실행하자마자 이 모양, 위 PLATE_LAYOUT)
#   원본 STL 판(로봇 둘레 4장)은 그대로 두고 빈 칸만 복제로 채운다. 배치 밖의 예전 추가분은 저장 목록에서 뺀다.
#   (예전: 저장해 둔 추가분을 그대로 되살렸다 → 좌우 4 × 앞뒤 2 회색 판이었다)
try:
    _pl_old=[tuple(t) for t in (S.get('plates_added') or [])]
    with LOCK: S['plates_added']=[]
    _pl_n=0
    for _t5 in PLATE_LAYOUT:
        if add_plate(tuple(_t5)): _pl_n+=1
    _pl_out=[t for t in PLATES if t not in set(PLATE_LAYOUT)]
    print(f"기본 바닥판 2×4: 원본 {len(PLATES)-_pl_n}장 + 추가 {_pl_n}장"
          +(f" (예전 추가분 {[t for t in _pl_old if t not in set(PLATE_LAYOUT)]} 뺌)" if set(_pl_old)-set(PLATE_LAYOUT) else '')
          +(f" ★ 배치 밖 원본 판 {_pl_out}" if _pl_out else ''))
except Exception as _e:
    print('기본 바닥판 만들기 실패:',_e)

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
/* ================= PSVR 입체 보기 (왜곡 보정) =================
   PSVR 렌즈는 화면을 확대하면서 가장자리를 바깥으로 휘게 만든다(핀쿠션).
   그래서 화면을 미리 안쪽으로 휘어서(배럴) 그려야 똑바로 보인다.
   보정이 없으면 초점이 안 맞고 흐리게 보인다.

   두 단계로 그린다:
     ① 장면을 텍스처에 그린다 (눈마다 한 번씩)
     ② 그 텍스처를 '휘어서' 화면에 그린다
   슬라이더로 눈 간격·왜곡·시야각을 직접 맞출 수 있다(사람마다 눈이 다르다). */
const VR={on:false, ipd:0.064, k1:0.22, k2:0.06, fov:100, scale:1.0};
try{ Object.assign(VR, JSON.parse(localStorage.getItem('vrCfg')||'{}')); }catch(e){}
VR.on=false;                       // 저장값과 무관하게 시작은 꺼진 상태

const _vrTgt=[null,null];          // 눈별 렌더 타겟
const _vrCam=new THREE.PerspectiveCamera(100,1,0.01,10);
const _vrScene=new THREE.Scene();
const _vrCam2=new THREE.OrthographicCamera(-1,1,1,-1,0,1);
const _vrMat=new THREE.ShaderMaterial({
  uniforms:{tex:{value:null},k1:{value:VR.k1},k2:{value:VR.k2},
            scale:{value:VR.scale}},
  vertexShader:`varying vec2 vUv; void main(){ vUv=uv;
    gl_Position=vec4(position.xy,0.0,1.0); }`,
  fragmentShader:`
    varying vec2 vUv; uniform sampler2D tex;
    uniform float k1; uniform float k2; uniform float scale;
    void main(){
      vec2 c=vUv*2.0-1.0;              // 화면 중앙이 (0,0)
      float r2=dot(c,c);
      // 배럴 왜곡: 중앙에서 멀수록 안쪽으로 당긴다
      vec2 d=c*(1.0+k1*r2+k2*r2*r2)/scale;
      vec2 uv=(d+1.0)*0.5;
      if(uv.x<0.0||uv.x>1.0||uv.y<0.0||uv.y>1.0){
        gl_FragColor=vec4(0.0,0.0,0.0,1.0); return; }   // 밖은 검정
      gl_FragColor=texture2D(tex,uv);
    }`});
_vrScene.add(new THREE.Mesh(new THREE.PlaneGeometry(2,2),_vrMat));

function vrSize(){
  const w=Math.max(2,innerWidth), h=Math.max(2,innerHeight);   // 화면 전체
  for(let i=0;i<1;i++){
    if(!_vrTgt[i]||_vrTgt[i].width!==w||_vrTgt[i].height!==h){
      if(_vrTgt[i])_vrTgt[i].dispose();
      _vrTgt[i]=new THREE.WebGLRenderTarget(w,h,
        {minFilter:THREE.LinearFilter,magFilter:THREE.LinearFilter});
    }
  }
  return [w,h];
}
function vrRender(){
  /* ★ 화면을 쪼개지 않는다 ★
     PSVR 을 '모니터 복제' 로 쓰면 헤드셋이 알아서 입체로 보여준다.
     여기서 또 좌우로 나누면 이중이 되어 오히려 이상해진다.
     -> 화면 하나를 그대로 두고, 렌즈 왜곡만 반대로 휘어서 또렷하게 만든다. */
  const [w,h]=vrSize();
  _vrCam.fov=VR.fov; _vrCam.aspect=w/h; _vrCam.updateProjectionMatrix();
  _vrMat.uniforms.k1.value=VR.k1;
  _vrMat.uniforms.k2.value=VR.k2;
  _vrMat.uniforms.scale.value=VR.scale;
  _vrCam.position.copy(cam.position);
  _vrCam.quaternion.copy(cam.quaternion);
  ren.setRenderTarget(_vrTgt[0]);
  ren.render(scene,_vrCam);
  ren.setRenderTarget(null);
  _vrMat.uniforms.tex.value=_vrTgt[0].texture;
  ren.render(_vrScene,_vrCam2);
}
function vrSave(){ try{ localStorage.setItem('vrCfg',JSON.stringify(VR)); }catch(e){} }
addEventListener('message',ev=>{
  const d=ev.data;
  if(!d||d.type!=='vr')return;
  if(d.on!==undefined)VR.on=!!d.on;
  for(const k of ['ipd','k1','k2','fov','scale'])
    if(d[k]!==undefined)VR[k]=+d[k];
  vrSave();
});

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
// 월드 원점 축 (X 빨강 / Y 초록 / Z 파랑).
// 조이콘 조종처럼 '어디가 앞뒤인지' 알아야 하는 기능이 생겨서 다시 켰다.
// 로봇 크기에 맞춰 길이를 정한다 (UR15 처럼 큰 로봇이면 더 길게).
world.add(makeAxes(0.40,3));       // 0.4m — 로봇 크기와 무관하게 잘 보이는 길이

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
// ---- 카메라 동기화: 녹화 영상의 카메라 위치(주황) + 판 테두리(빨강) + 판 가운데→카메라 선(하늘) ----
//   1인칭이면 이 장면을 카메라 눈으로 그리고, 실제 영상을 배경으로 깐다.
//   앱의 로봇·도킹이 영상 속 실물 위에 겹치면 디지털 트윈이 맞는 것.
const camSync={st:null,fp:false,k:-1,tex:null,loading:false,saved:null,bg0:null};
camSync.vscene=new THREE.Scene();      // '영상만' 보기용 빈 장면 (배경 = 영상)
camSync.zoom=1;                        // 1인칭 '트윈만' 보기의 확대 배율 (휠). 영상이 켜진 보기에선 늘 1
ren.domElement.addEventListener('wheel',ev=>{
  if(!camSync.fp||!camSync.st||camSync.st.view!=='twin')return;   // 영상이 보이면 확대 금지 (영상과 3D 가 어긋난다)
  ev.preventDefault(); ev.stopPropagation();
  camSync.zoom=Math.min(6,Math.max(0.25,camSync.zoom*(ev.deltaY<0?1.1:1/1.1)));
},{passive:false,capture:true});
camSync.grp=new THREE.Group(); camSync.grp.visible=false; world.add(camSync.grp);
camSync.edgeGrp=new THREE.Group(); camSync.edgeGrp.visible=false; world.add(camSync.edgeGrp);
camSync.dot=new THREE.Mesh(new THREE.SphereGeometry(0.012,16,12),
  new THREE.MeshBasicMaterial({color:0xff8800}));
camSync.grp.add(camSync.dot);
function _csLines(color,grp){
  const g=new THREE.BufferGeometry();
  g.setAttribute('position',new THREE.Float32BufferAttribute(new Float32Array(3*32),3));
  const l=new THREE.LineSegments(g,new THREE.LineBasicMaterial({color:color}));
  l.frustumCulled=false; grp.add(l); return l;
}
camSync.frus=_csLines(0xff8800,camSync.grp);
camSync.ray=_csLines(0x33ccff,camSync.grp);
camSync.edge=_csLines(0xff3333,camSync.edgeGrp);
// 카메라가 읽은 물건: 바닥에 고리 (로봇 = 자홍 큰 고리, 도킹 = 그 색 고리)
camSync.rings=[];
for(let i=0;i<12;i++){
  const pts=[];for(let a=0;a<=32;a++)pts.push(new THREE.Vector3(Math.cos(a/32*Math.PI*2),Math.sin(a/32*Math.PI*2),0));
  const r=new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),new THREE.LineBasicMaterial({color:0xffffff,depthTest:false}));
  r.visible=false; r.renderOrder=5; camSync.edgeGrp.add(r); camSync.rings.push(r);
}
function _csSet(l,pts){
  const a=l.geometry.attributes.position;
  for(let i=0;i<pts.length&&i<a.count;i++){a.array[i*3]=pts[i][0];a.array[i*3+1]=pts[i][1];a.array[i*3+2]=pts[i][2];}
  a.needsUpdate=true; l.geometry.setDrawRange(0,Math.min(pts.length,a.count));
}
const _csLoader=new THREE.TextureLoader();
function camSyncUpdate(cs){
  camSync.st=cs||null;
  if(!cs||!cs.on){camSync.grp.visible=false;camSync.edgeGrp.visible=false;camSyncFp(false);return;}
  camSync.edgeGrp.visible=true;
  const objs=cs.objs||[];
  camSync.rings.forEach((r,i)=>{const o=objs[i]; r.visible=!!o; if(!o)return;
    const rad=o.kind==='robot'?0.07:0.045;
    r.scale.set(rad,rad,1); r.position.set(o.xy[0],o.xy[1],(cs.center[2]||0.02)+0.003);
    r.material.color.set(o.kind==='robot'?0xff00ff:(o.hex||'#ffffff'));});
  const c=cs.corners; _csSet(camSync.edge,[c[0],c[1],c[1],c[2],c[2],c[3],c[3],c[0]]);
  if(cs.pos){
    const P=cs.pos, R=cs.R, ctr=cs.center;
    camSync.dot.position.set(P[0],P[1],P[2]);
    // 보는 범위 사각뿔 (카메라 축: x 오른쪽, y 아래, z 앞)
    const tv=Math.tan(cs.fovy*Math.PI/360), th=tv*cs.aspect, d=0.08, q=[];
    for(const [u,v] of [[-th,-tv],[th,-tv],[th,tv],[-th,tv]]){
      q.push([P[0]+d*(R[0]*u+R[1]*v+R[2]),P[1]+d*(R[3]*u+R[4]*v+R[5]),P[2]+d*(R[6]*u+R[7]*v+R[8])]);}
    _csSet(camSync.frus,[P,q[0],P,q[1],P,q[2],P,q[3],q[0],q[1],q[1],q[2],q[2],q[3],q[3],q[0]]);
    const e=[ctr[0]+(P[0]-ctr[0])*1.25,ctr[1]+(P[1]-ctr[1])*1.25,ctr[2]+(P[2]-ctr[2])*1.25];
    _csSet(camSync.ray,[ctr,e]);
  }
  camSync.grp.visible=!!cs.pos&&!cs.fp;            // 1인칭에선 자기 몸(카메라 표시)은 숨긴다
  if(!!cs.fp!==camSync.fp)camSyncFp(!!cs.fp);
  if(camSync.fp&&cs.view==='twin'&&scene.background!==camSync.bg0)scene.background=camSync.bg0;
  if(camSync.fp&&cs.view!=='twin'&&camSync.tex&&scene.background!==camSync.tex)scene.background=camSync.tex;
  if(camSync.fp&&cs.k!==camSync.k&&!camSync.loading){   // 배경 = 지금 프레임
    camSync.loading=true; const k=cs.k;
    _csLoader.load(`/camframe/${k}?t=${Date.now()}`,t=>{
      t.colorSpace=THREE.SRGBColorSpace;
      if(camSync.fp){ if(camSync.tex)camSync.tex.dispose(); camSync.tex=t;
        scene.background=(camSync.st&&camSync.st.view==='twin')?camSync.bg0:t; camSync.vscene.background=t; }
      else t.dispose();
      camSync.k=k; camSync.loading=false;
    },undefined,()=>{camSync.loading=false;});
  }
}
function camSyncFp(on){
  if(on&&!camSync.fp){
    camSync.saved={p:cam.position.clone(),t:ctl.target.clone(),fov:cam.fov};
    camSync.bg0=scene.background; ctl.enabled=false; camSync.fp=true; camSync.k=-1;
  }else if(!on&&camSync.fp){
    cam.position.copy(camSync.saved.p); ctl.target.copy(camSync.saved.t);
    cam.fov=camSync.saved.fov; cam.aspect=innerWidth/innerHeight; cam.updateProjectionMatrix();
    scene.background=camSync.bg0; if(camSync.tex){camSync.tex.dispose();camSync.tex=null;}
    ctl.enabled=true; camSync.fp=false; camSync.zoom=1;
    ren.setScissorTest(false); ren.setViewport(0,0,innerWidth,innerHeight);
  }
}
function camSyncRender(){
  const cs=camSync.st;
  if(cs&&cs.pos){
    // 카메라(OpenCV 축, z-up 월드) -> three.js 카메라 (y-up, -z 를 본다). world 그룹은 x 축 -90도 회전.
    const R=cs.R, M=v=>new THREE.Vector3(v[0],v[2],-v[1]);
    const X=M([R[0],R[3],R[6]]), Y=M([R[1],R[4],R[7]]).negate(), Z=M([R[2],R[5],R[8]]).negate();
    cam.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(X,Y,Z));
    cam.position.copy(M(cs.pos));
    // 트윈만 보기: 카메라 자리에서 시야각만 좁히고 넓힌다 (망원처럼). 영상이 보이면 배율 1 = 영상과 같은 시야
    const z=(cs.view==='twin')?camSync.zoom:1;
    cam.fov=2*Math.atan(Math.tan(cs.fovy*Math.PI/360)/z)*180/Math.PI;
    cam.aspect=cs.aspect; cam.updateProjectionMatrix();
  }
  // 영상 비율 그대로 가운데에 (남는 곳은 검정) — 비율이 다르면 배경과 3D 가 어긋난다
  const W=innerWidth,H=innerHeight,a=(cs&&cs.aspect)||16/9;
  let w=W,h=W/a; if(h>H){h=H;w=H*a;}
  const x=(W-w)/2,y=(H-h)/2;
  ren.setScissorTest(false); ren.setViewport(0,0,W,H);
  const bg=scene.background; scene.background=null; ren.setClearColor(0x000000,1); ren.clear(); scene.background=bg;
  ren.setScissorTest(true); ren.setScissor(x,y,w,h); ren.setViewport(x,y,w,h);
  ren.render((cs&&cs.view==='video')?camSync.vscene:scene,cam);   // '영상만' 이면 3D 없이 영상만
  ren.setScissorTest(false); ren.setViewport(0,0,W,H);
}
const loader=new STLLoader(); const objs={}; window._objsDbg=objs;
const robotIds=new Set();   // 로봇 링크 메쉬 id (우클릭에서 팔을 알아본다, 11-03)
let ROBGEN=null;                 // 로봇 교체 세대. 바뀌면 STL 을 새로 받는다
function dropRobotMeshes(ids){   // 예전 로봇 부품을 화면에서 치운다
  for(const id of ids){
    const d=objs[id]; if(!d)continue;
    if(d.mesh){ world.remove(d.mesh);
      if(d.mesh.geometry)d.mesh.geometry.dispose();
      if(d.mesh.material)d.mesh.material.dispose(); }
    delete objs[id];
  }
}
function ensure(id,color,scale,gen){
  if(objs[id])return;
  objs[id]={mesh:null,tp:new THREE.Vector3(),tq:new THREE.Quaternion(),
            cp:new THREE.Vector3(),cq:new THREE.Quaternion(),s:scale,init:false};
  loader.load('/mesh/'+id+(gen!=null?('?g='+gen):''),g=>{
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
// ---- 보정 지도 (11-04) — 초록 보정됨 / 빨강 보정할 기준점 / 노랑 보간됨 / 회색 아직 ----
const calGroup=new THREE.Group(); world.add(calGroup);
let calMeshes=[];
const CAL_COL={good:0x2299ff,ok:0x22cc55,odd:0xff8800,need:0xff3344,fill:0xffcc22,none:0x8a8f99,far:0xb066ff};
function buildCalpts(list){
  const key=JSON.stringify(list);
  if(calGroup.userData.key===key) return;
  calGroup.userData.key=key;
  calGroup.clear(); calMeshes=[];
  if(!list) return;
  for(const c of list){
    const r=(c.kind==='good'||c.kind==='odd')?0.0075:(c.base?0.0065:(c.rot?0.0026:0.0042));   // 성공·튀는점 > 기준점 > 0° 자리 > 회전 자리
    const geo=new THREE.CylinderGeometry(r,r,0.004,20);
    const m=new THREE.Mesh(geo,new THREE.MeshBasicMaterial({color:CAL_COL[c.kind]||0xffffff,
      transparent:true,opacity:c.kind==='none'?0.55:0.95}));
    m.rotation.x=Math.PI/2;                          // world 는 z 가 위
    m.position.set(c.x,c.y,c.z+0.004);
    m.userData.cal=c;
    calGroup.add(m); calMeshes.push(m);
  }
}
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
// ---- 로봇 2핀 배치 레이어 ----
// 주황 원기둥 = 로봇 핀(클릭=선택), 하늘색 디스크 = 꽂을 수 있는 목심
const rmGroup=new THREE.Group(); world.add(rmGroup);
const RM_PIN =new THREE.MeshBasicMaterial({color:0xe8720c});
const RM_PINS=new THREE.MeshBasicMaterial({color:0xffb066});
const RM_PEG =new THREE.MeshBasicMaterial({color:0x33ccff,transparent:true,opacity:0.9});
let rmPins=[],rmPegs=[];
function buildRMove(rm){
  const key=rm?JSON.stringify([rm.phase,rm.pin,rm.pins,rm.pegs.length,
                               rm.pegs[0]||0]):'off';
  if(rmGroup.userData.key===key)return;
  rmGroup.userData.key=key;
  rmGroup.clear();rmPins=[];rmPegs=[];
  if(!rm)return;
  const pg=new THREE.CylinderGeometry(0.007,0.007,0.03,14);pg.rotateX(Math.PI/2);
  rm.pins.forEach((p,i)=>{
    const m=new THREE.Mesh(pg,(rm.pin===i)?RM_PINS:RM_PIN);
    m.position.set(p[0],p[1],rm.ptop+0.02);
    m.userData.pin=i;rmGroup.add(m);rmPins.push(m);
  });
  const dg=new THREE.CylinderGeometry(0.009,0.009,0.004,16);dg.rotateX(Math.PI/2);
  rm.pegs.forEach(g=>{
    const m=new THREE.Mesh(dg,RM_PEG);
    m.position.set(g[0],g[1],rm.ptop+0.003);
    m.userData.gxy=g;rmGroup.add(m);rmPegs.push(m);
  });
  window._rmDbg={pins:rmPins.map(m=>[m.position.x,m.position.y,m.position.z]),
                 pegs:rmPegs.length};   // 모듈 밖 점검용
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
// 플레이트 더블클릭 = 확장 모드 (고스트 표시). 고스트 클릭하면 서버가 알아서 닫는다
ren.domElement.addEventListener('dblclick',async ev=>{
  const r=ren.domElement.getBoundingClientRect();
  mouse.x=((ev.clientX-r.left)/r.width)*2-1;
  mouse.y=-((ev.clientY-r.top)/r.height)*2+1;
  ray.setFromCamera(mouse,cam);
  const plates=Object.values(objs).filter(o=>o.plate&&o.mesh).map(o=>o.mesh);
  if(ray.intersectObjects(plates,false).length){
    await fetch('/pedit/1'); ghostGroup.userData.key=null;
  }
});
ren.domElement.addEventListener('click',async ev=>{
  {// 로봇 2핀 배치가 우선
    const r=ren.domElement.getBoundingClientRect();
    mouse.x=((ev.clientX-r.left)/r.width)*2-1;
    mouse.y=-((ev.clientY-r.top)/r.height)*2+1;
    ray.setFromCamera(mouse,cam);
    const hp=ray.intersectObjects(rmPins,false);
    if(hp.length){await fetch('/rpin/'+hp[0].object.userData.pin);
      rmGroup.userData.key=null;return;}
    const hg=ray.intersectObjects(rmPegs,false);
    if(hg.length){const g=hg[0].object.userData.gxy;
      await fetch('/rpeg/'+g[0]+','+g[1]);
      rmGroup.userData.key=null;return;}
    /* ★ 빈 곳을 눌렀다고 배치를 끄지 않는다 ★
       예전에는 핀·목심이 아닌 곳을 한 번만 잘못 눌러도 주황 핀이 사라져,
       처음부터 다시 끌어다 놓아야 했다.
       배치는 '끝났을 때' 스스로 꺼지고, 그 전에는 ESC 나 우클릭으로만 끈다. */
    if(rmPins.length)return;
    if(calMeshes.length){                       // 보정 지도 점 (11-04)
      const hc=ray.intersectObjects(calMeshes,false);
      if(hc.length){
        const c=hc[0].object.userData.cal;
        const j=await (await fetch('/calpt/'+c.x+','+c.y)).json();
        if(window.parent!==window)
          window.parent.postMessage({calpose:j.name||null,armmsg:j.msg||''},'*');
        calGroup.userData.key=null; return;
      }
    }
  }
  if(pegHover){                            // 목심 선택 (도달 가능한 것만)
    if(!pegHover.userData.ok) return;      // 빨강은 클릭 무시
    const prev=pegMeshes.find(m=>m.userData.name===pegSelName);
    pegSelName=pegHover.userData.name;
    pegPaint(prev); pegPaint(pegHover);
    await fetch('/peg/'+encodeURIComponent(pegSelName));
    return;
  }
  if(!hover){
    // 설비/물체 메쉬 클릭 = 선택 하이라이트 + 오른쪽 탭 이름·색 자동 선택
    if(!ghostMeshes.length){
      const picks=[],ids=[];
      for(const id in objs){const o=objs[id];
        if(o.env&&!o.plate&&o.mesh&&o.mesh.visible){picks.push(o.mesh);ids.push(id);}}
      const he=ray.intersectObjects(picks,false);
      if(he.length){
        const id=ids[picks.indexOf(he[0].object)];
        selEnvSet(id);                              // 파랗게 '선택된 느낌'
        if(window.parent!==window)
          window.parent.postMessage({armsel:id},'*');
        return;
      }
      selEnvSet(null);                              // 빈 곳 클릭 = 선택 해제
    }
    if(ghostMeshes.length){await fetch('/pedit/0');ghostGroup.userData.key=null;}
    return;
  }
  const t=hover.userData.tile;
  await fetch('/click/'+t[0]+','+t[1]);
  ghostGroup.userData.key=null;      // 다음 폴링에서 갱신
});

// 클릭 선택 하이라이트 (선택된 설비/물체가 파랗게 빛남)
let selEnvId=null;
function selEnvSet(id){
  const off=o=>{if(o&&o.mesh&&o.mesh.material.emissive)
    o.mesh.material.emissive.setHex(0x000000);};
  const on=o=>{if(o&&o.mesh&&o.mesh.material.emissive)
    o.mesh.material.emissive.setHex(0x2255aa);};
  if(selEnvId&&objs[selEnvId])off(objs[selEnvId]);
  selEnvId=id;
  if(id&&objs[id])on(objs[id]);
}
// Ctrl+Z = 실행취소 (배치/삭제/복제/블록 편집)
addEventListener('keydown',e=>{
  if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'){
    e.preventDefault();fetch('/undo');
  }
  // ESC = 배치 그만두기 (빈 곳 클릭으로는 안 꺼지므로 이 길을 열어둔다)
  if(e.key==='Escape'&&rmPins.length){ fetch('/emoff'); rmGroup.userData.key=null; }
});

// 우클릭 = 설비/물체 컨텍스트 메뉴 (이동 / 삭제)
const ctxMenu=document.createElement('div');
ctxMenu.style.cssText='position:fixed;display:none;z-index:99;background:#1d222d;'
  +'border:1px solid #3a4152;border-radius:8px;padding:4px;'
  +'font:13px sans-serif;color:#eee;box-shadow:0 6px 20px rgba(0,0,0,.5)';
document.body.appendChild(ctxMenu);
function hideCtx(){ctxMenu.style.display='none';}
addEventListener('click',hideCtx);
ren.domElement.addEventListener('contextmenu',ev=>{
  const r=ren.domElement.getBoundingClientRect();
  mouse.x=((ev.clientX-r.left)/r.width)*2-1;
  mouse.y=-((ev.clientY-r.top)/r.height)*2+1;
  ray.setFromCamera(mouse,cam);
  const picks=[],ids=[];
  for(const id in objs){const o=objs[id];
    if(((o.env&&!o.plate)||robotIds.has(id))&&o.mesh&&o.mesh.visible){picks.push(o.mesh);ids.push(id);}}
  const h=ray.intersectObjects(picks,false);
  if(!h.length)return;                 // 빈 곳 우클릭은 브라우저 기본 메뉴
  ev.preventDefault();
  const id=ids[picks.indexOf(h[0].object)];
  ctxMenu.innerHTML='';
  const mk=(txt,fn)=>{const b=document.createElement('div');
    b.textContent=txt;
    b.style.cssText='padding:6px 14px;cursor:pointer;border-radius:6px;white-space:nowrap';
    b.onmouseenter=()=>b.style.background='#2f6ae0';
    b.onmouseleave=()=>b.style.background='';
    b.onclick=e=>{e.stopPropagation();hideCtx();fn();};
    ctxMenu.appendChild(b);};
  if(robotIds.has(id)){                // 로봇 팔 우클릭 (11-03): 그 팔 위치 옮기기 ('B~l3' = B팔, 'l3' = A팔)
    const L=id.includes('~')?id.split('~')[0]:'A';
    mk(`📍 ${L}팔 위치 옮기기 (핀 배치)`,async()=>{
      const j=await (await fetch('/armmv/'+encodeURIComponent(L))).json();
      if(window.parent!==window)window.parent.postMessage({armmsg:j.msg},'*');
      rmGroup.userData.key=null;});
    ctxMenu.style.left=ev.clientX+'px';ctxMenu.style.top=ev.clientY+'px';
    ctxMenu.style.display='block';
    return;
  }
  selEnvSet(id);                       // 우클릭한 대상도 '선택된 느낌'
  if(!objs[id].brick)                 // 물체는 핀이 없어 이동 메뉴 제외
    mk('📍 이동 (핀 배치)',()=>fetch('/envmv/'+encodeURIComponent(id)));
  mk('🗑 삭제',()=>fetch('/envdel/'+encodeURIComponent(id)));
  ctxMenu.style.left=ev.clientX+'px';ctxMenu.style.top=ev.clientY+'px';
  ctxMenu.style.display='block';
});

// 우측 목록에서 설비 썸네일을 끌어다 놓으면 그 자리(20mm 스냅)로 이동
// 드래그 중: 실제 STL 메쉬가 커서 밑 바닥 위치를 따라다닌다
const dragEnv={on:false,x:0,y:0,id:null,t:0}; window._dragDbg=dragEnv;
// 메쉬는 전부 -90도 회전된 world 그룹(로봇 Z-up 좌표) 안에 있으므로,
// 커서 광선을 world 로컬로 변환한 뒤 로봇 바닥(z=zp) 평면과 교차시켜야
// 정확하다. (안 그러면 카메라 각도에 따라 X/Y 가 뒤틀려 보인다)
const _fInv=new THREE.Matrix4(), _fro=new THREE.Vector3(), _frd=new THREE.Vector3();
function rayFloor(zp){
  // mouse/ray 는 호출 전에 setFromCamera 되어 있어야 한다
  _fInv.copy(world.matrixWorld).invert();
  _fro.copy(ray.ray.origin).applyMatrix4(_fInv);
  _frd.copy(ray.ray.direction).transformDirection(_fInv);
  const t=(zp-_fro.z)/_frd.z;
  return [_fro.x+_frd.x*t, _fro.y+_frd.y*t];
}
function floorXY(ev,zp=0){
  const r=ren.domElement.getBoundingClientRect();
  mouse.x=((ev.clientX-r.left)/r.width)*2-1;
  mouse.y=-((ev.clientY-r.top)/r.height)*2+1;
  ray.setFromCamera(mouse,cam);
  return rayFloor(zp);
}
// ★ 드래그 이벤트는 '문서 전체' 에서 받는다 ★
//   캔버스에만 걸면 iframe 안에서 캔버스가 못 덮는 여백으로 커서가 지나갈 때
//   dragover 를 놓쳐 주황 미리보기가 안 뜬다.
//   (3D 는 다른 포트의 iframe 이라 dataTransfer 가 안 넘어오므로,
//    어떤 설비인지는 서버가 주는 st.dragid 로 받는다)
document.addEventListener('dragenter',ev=>{
  ev.preventDefault();
  try{ ev.dataTransfer.dropEffect='copy'; }catch(e){}
});
document.addEventListener('dragover',ev=>{
  ev.preventDefault();
  // 받겠다는 표시. 이게 없으면 브라우저가 drop 을 안 띄운다.
  try{ ev.dataTransfer.dropEffect='copy'; }catch(e){}
  const p=floorXY(ev); dragEnv.on=true; dragEnv.x=p[0]; dragEnv.y=p[1];
  dragEnv.mx=mouse.x; dragEnv.my=mouse.y;   // NDC 저장 (매 프레임 재투영용)
  dragEnv.t=performance.now();
  // 어떤 설비를 끌고 있는지 = dataTransfer '타입 이름'에 hex 로 실려온다
  // (drop 전에는 값을 못 읽는 브라우저 제약 우회 -> 서버 왕복 없이 즉시 추적)
  for(const tp of ev.dataTransfer.types){
    if(tp.startsWith('envid-')){
      const h=tp.slice(6), b=[];
      for(let i=0;i+1<h.length;i+=2)b.push(parseInt(h.substr(i,2),16));
      try{const nid=new TextDecoder().decode(new Uint8Array(b));
          if(nid)dragEnv.id=nid;}catch(e){}
      break;
    }
  }
});
// dragleave 는 자식 요소를 지날 때도 뜨므로 '창 밖으로 나갈 때' 만 끈다
document.addEventListener('dragleave',ev=>{
  if(ev.relatedTarget===null)dragEnv.on=false;
});
document.addEventListener('drop',async ev=>{
  ev.preventDefault();
  const gid=dragEnv.id; dragEnv.on=false; dragEnv.id=null;
  // 다른 포트의 iframe 이라 dataTransfer 는 비어 있다 -> 서버가 알려준 id 를 쓴다
  let id='';
  try{ id=ev.dataTransfer.getData('text/plain')||''; }catch(e){}
  if(!id)id=gid||'';
  if(!id)return;
  let p=floorXY(ev);
  // 끌던 메쉬가 '보이던 자리' 그대로 놓이게 - 고스트의 현재 중심을 그대로 전달
  const o=gid?objs[gid]:null;
  if(o&&o.mesh&&o.c0){
    const yw=o.dragYaw||0, cy=Math.cos(yw), sy=Math.sin(yw);
    p=[o.cp.x+(cy*o.c0[0]-sy*o.c0[1]), o.cp.y+(sy*o.c0[0]+cy*o.c0[1])];
  }
  await fetch('/envdrag/'+encodeURIComponent(id)+'/'+p[0].toFixed(4)+','+p[1].toFixed(4));
});
/* ===== 조이콘 즉시 경로 (공 굴리기와 같은 구조) =====
   조이콘 페이지(7900)가 표에서 찾은 자세를 postMessage 로 바로 보내면,
   여기서 자체 FK 로 링크 행렬을 만들어 즉시 그린다. 서버 폴링(25ms+지터)을
   기다리지 않으므로 끊기지 않는다. */
let KIN=null, joyLive=0;
fetch('/kin').then(r=>r.json()).then(k=>{KIN=k;}).catch(()=>{});
function fkLocal(q){
  if(!KIN)return null;
  const cy=Math.cos(KIN.yaw), sy=Math.sin(KIN.yaw);
  const Y=new THREE.Matrix4().set(cy,-sy,0,KIN.off[0], sy,cy,0,KIN.off[1],
                                  0,0,1,KIN.off[2], 0,0,0,1);
  let T=new THREE.Matrix4();          // 단위
  const A=new THREE.Matrix4(), R=new THREE.Matrix4();
  const E=new THREE.Euler(), AX=new THREE.Vector3();
  const out=[Y.clone()];              // base_link
  for(let k=0;k<KIN.joints.length;k++){
    const j=KIN.joints[k];
    E.set(j.rpy[0],j.rpy[1],j.rpy[2],'ZYX');   // URDF rpy = Rz*Ry*Rx
    A.makeRotationFromEuler(E);
    A.setPosition(j.xyz[0],j.xyz[1],j.xyz[2]);
    T.multiply(A);
    AX.set(j.axis[0],j.axis[1],j.axis[2]).normalize();
    R.makeRotationAxis(AX,q[k]||0);
    T.multiply(R);
    out.push(new THREE.Matrix4().multiplyMatrices(Y,T));
  }
  return out;
}
addEventListener('message',ev=>{
  const d=ev.data;
  /* ★ 크롬 우회: 드래그를 부모 화면(7900)에서 받는다 ★
     크롬은 '다른 출처 iframe' 으로의 드롭을 막는다 (파이어폭스는 허용).
     그래서 앱 화면이 화면좌표를 보내주면, 여기서 바닥좌표로 바꿔
     원래 하던 일을 그대로 한다. */
  if(d&&d.type==='dragmove'){
    const r=ren.domElement.getBoundingClientRect();
    mouse.x=((d.x-r.left)/r.width)*2-1;
    mouse.y=-((d.y-r.top)/r.height)*2+1;
    ray.setFromCamera(mouse,cam);
    const p=rayFloor(0);
    dragEnv.on=true; dragEnv.x=p[0]; dragEnv.y=p[1];
    dragEnv.mx=mouse.x; dragEnv.my=mouse.y; dragEnv.t=performance.now();
    if(d.id)dragEnv.id=d.id;
    return;
  }
  if(d&&d.type==='dragoff'){ dragEnv.on=false; return; }
  if(d&&d.type==='dragdrop'){
    const gid=d.id||dragEnv.id; dragEnv.on=false; dragEnv.id=null;
    if(!gid)return;
    const r=ren.domElement.getBoundingClientRect();
    mouse.x=((d.x-r.left)/r.width)*2-1;
    mouse.y=-((d.y-r.top)/r.height)*2+1;
    ray.setFromCamera(mouse,cam);
    let p=rayFloor(0);
    const o=objs[gid];
    if(o&&o.mesh&&o.c0){          // 끌던 메쉬가 보이던 그 자리로
      const yw=o.dragYaw||0, cy=Math.cos(yw), sy=Math.sin(yw);
      p=[o.cp.x+(cy*o.c0[0]-sy*o.c0[1]), o.cp.y+(sy*o.c0[0]+cy*o.c0[1])];
    }
    fetch('/envdrag/'+encodeURIComponent(gid)+'/'
          +p[0].toFixed(4)+','+p[1].toFixed(4));
    return;
  }
  if(!d||d.type!=='joyq'||!KIN)return;
  const F=fkLocal(d.q);
  if(!F)return;
  joyLive=performance.now();          // 잠시 서버 links 갱신을 무시한다
  const n=Math.min(KIN.links.length,F.length);
  for(let k=1;k<n;k++){               // base_link 는 고정
    const M=F[k].elements;            // three.js 는 column-major
    setTarget(KIN.links[k],[M[0],M[4],M[8],M[12],
                            M[1],M[5],M[9],M[13],
                            M[2],M[6],M[10],M[14], 0,0,0,1]);
  }
});

/* 📱 지금 움직이는 블록의 예상 경로 — 반투명 관 (서버 st.path, 블록이 끝나면 비워진다) */
let pathObj=null, pathKey='';
function drawPath(pts){
  const k=JSON.stringify(pts||null); if(k===pathKey)return; pathKey=k;
  if(pathObj){ world.remove(pathObj); pathObj.geometry.dispose(); pathObj.material.dispose(); pathObj=null; }
  if(!pts||pts.length<2)return;
  const v=pts.map(p=>new THREE.Vector3(p[0],p[1],p[2])), cp=new THREE.CurvePath();
  for(let i=0;i+1<v.length;i++) if(v[i].distanceTo(v[i+1])>1e-4) cp.add(new THREE.LineCurve3(v[i],v[i+1]));
  if(!cp.curves.length)return;
  const g=new THREE.TubeGeometry(cp, 24*cp.curves.length, 0.006, 10, false);
  const m=new THREE.MeshBasicMaterial({color:0x22d3ee, transparent:true, opacity:0.35, depthWrite:false});
  pathObj=new THREE.Mesh(g,m); pathObj.renderOrder=5; world.add(pathObj);
}
async function poll(){
  try{
    const st=await (await fetch('/state')).json();
    if(st.gen!==undefined && ROBGEN!==null && st.gen!==ROBGEN){
      dropRobotMeshes(Object.keys(objs).filter(k=>!(st.env||[]).some(e=>e.id===k)));
    }
    if(st.gen!==undefined)ROBGEN=st.gen;
    const skipLinks=(performance.now()-joyLive)<400;   // 즉시 경로 우선
    for(const l of st.links){ ensure(l.id,l.color,1.0,st.gen); robotIds.add(l.id);
      const _lo=objs[l.id];                      // 두 팔 모드: 팔 색이 바뀌면 다시 칠한다 (10-30)
      if(_lo&&_lo.mesh&&_lo.col!==l.color){_lo.col=l.color;_lo.mesh.material.color.set(l.color);}
      if(!skipLinks)setTarget(l.id,l.m); }
    const alive=new Set();
    for(const e of st.env){ ensure(e.id,e.color,e.scale); alive.add(e.id);
      // 이름 붙이기에서 색을 바꾸면 반영 (ensure 는 최초 1회만 색을 칠하므로)
      const _o=objs[e.id];
      if(_o){_o.plate=!!e.plate;
        if(_o.mesh&&_o.col!==e.color){_o.col=e.color;_o.mesh.material.color.set(e.color);}}
      const o=e.off||[0,0,0], yw=e.yaw||0;
      const cy=Math.cos(yw), sy=Math.sin(yw);
      if(dragEnv.on&&!dragEnv.id&&st.dragid)dragEnv.id=st.dragid; // 예비 경로
      if(dragEnv.on&&dragEnv.id===e.id){
        // 드래그 중인 설비는 anim 루프가 매 프레임 커서를 따라 움직인다
        // (여기서 setTarget 하면 폴링 주기마다 서버 위치로 되돌아가서 뚝뚝 끊김)
        const _d=objs[e.id];
        if(_d){_d.dragYaw=yw;_d.dragZ=o[2];_d.pc=e.pc||null;}
      }else{
        setTarget(e.id,[cy,-sy,0,o[0], sy,cy,0,o[1], 0,0,1,o[2], 0,0,0,1]);
      }
      objs[e.id].vis=e.v; objs[e.id].env=true;
      objs[e.id].brick=!!e.brick; }
    // 삭제된 환경/물체 오브젝트 정리 (우클릭 삭제 등)
    for(const id in objs){
      if(objs[id].env&&!alive.has(id)&&objs[id].mesh){
        world.remove(objs[id].mesh); delete objs[id];
      }
    }
    window._snapPts=st.snaps||[];        // 물체 드래그용 자석 지점 (빈 도킹)
    buildGhosts(st.ghosts||[],st.psize||0.2);
    buildCalpts(st.calpts||null);
    buildPegs(st.pegs||[],st.ptop||0.02);
    buildRMove(st.rmove||null);
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
    camSyncUpdate(st.camsync);
    try{ drawPath(st.path); }catch(e){}
  }catch(err){}
  /* ★ 화면 그리는 박자에 맞춰 다음 폴링을 잡는다 ★
     예전에는 setTimeout(25ms) 로 화면(16.7ms)과 따로 돌았다. 두 주기가
     어긋나면 어떤 프레임엔 새 자세가 두 번 들어오고 어떤 프레임엔 안 들어와
     '뚝뚝' 끊겨 보인다. 크롬은 이 어긋남이 파이어폭스보다 크게 드러난다.
     requestAnimationFrame 으로 잡으면 한 프레임에 정확히 한 번만 갱신된다.
     (탭이 안 보일 땐 rAF 가 멈추므로 setTimeout 으로 되돌아간다) */
  if(document.hidden)setTimeout(poll,200);
  else requestAnimationFrame(()=>poll());
}
poll();
addEventListener('resize',()=>{cam.aspect=innerWidth/innerHeight;
  cam.updateProjectionMatrix();ren.setSize(innerWidth,innerHeight);});
let last=performance.now();
const _sm=new THREE.Matrix4(), _ss=new THREE.Vector3(1,1,1);
const _rD=new THREE.Raycaster(), _mD=new THREE.Vector2();  // 드래그 전용
const _dInv=new THREE.Matrix4(), _dro=new THREE.Vector3(), _drd=new THREE.Vector3();
(function anim(now){
  requestAnimationFrame(anim);
  const dt=Math.min((now-last)/1000||0.016,0.1); last=now;
  // 목표(조이콘)가 38ms 마다 갱신되는데 55ms 시정수로 따라가면 항상
  // 45% 쯤에서 다음 목표로 점프해 '뚝뚝' 끊겨 보인다 -> 목표 주기보다 빠르게.
  const a=1-Math.exp(-dt*40);          // 약 25ms 시정수
  // 드래그가 끊긴 채 방치되면 (ESC 취소 등) 자동 해제
  // 크롬은 다른 출처 iframe 위에서 dragover 를 띄엄띄엄 준다.
  // 800ms 는 너무 짧아 드래그 중에도 해제돼 유령이 깜빡였다.
  if(dragEnv.on&&now-dragEnv.t>2500){dragEnv.on=false;dragEnv.id=null;}
  // 드래그 중인 설비: 폴링을 기다리지 않고 매 프레임 커서 아래로 목표 갱신
  if(dragEnv.on&&dragEnv.id){
    const d=objs[dragEnv.id];
    if(d&&d.mesh){
      if(!d.c0){d.mesh.geometry.computeBoundingBox();
        const bb=d.mesh.geometry.boundingBox;
        d.c0=[(bb.min.x+bb.max.x)/2*d.s,(bb.min.y+bb.max.y)/2*d.s];
        d.z0=bb.min.z*d.s;}
      // 시차 보정: 커서 광선을 '다리 바닥' 평면에 쏘고, '다리(핀) 중심'이
      // 커서 바로 밑에 오게 한다 -> 카메라를 어느 각도로 돌려도
      // 주황 다리가 항상 마우스 정중앙 (몸통 높이 기준으로 하면 뒤틀려 보임)
      const zc=(d.dragZ||0)+d.z0;
      _mD.set(dragEnv.mx||0,dragEnv.my||0);
      _rD.setFromCamera(_mD,cam);
      // world 그룹(-90도 회전) 로컬로 광선 변환 후 다리 바닥 평면과 교차
      _dInv.copy(world.matrixWorld).invert();
      _dro.copy(_rD.ray.origin).applyMatrix4(_dInv);
      _drd.copy(_rD.ray.direction).transformDirection(_dInv);
      const tt=(zc-_dro.z)/_drd.z;
      let px=_dro.x+_drd.x*tt;
      let py=_dro.y+_drd.y*tt;
      // 물체(펜)는 빈 도킹 소켓 40mm 안에 오면 착 달라붙는다 (자석).
      // 이미 결합된 도킹은 스냅 목록에 없어서 끌어가도 안 붙는 느낌.
      if(d.brick&&window._snapPts&&window._snapPts.length){
        let best=null,bd=1e9;
        for(const sp of window._snapPts){
          const dd=Math.hypot(sp[0]-px,sp[1]-py);
          if(dd<bd){bd=dd;best=sp;}
        }
        if(best&&bd<0.04){px=best[0];py=best[1];}
      }
      const tp=d.pc||d.c0;              // 다리 중심 (없으면 몸통 중심)
      const yw=d.dragYaw||0, cy=Math.cos(yw), sy=Math.sin(yw);
      const ox=px-(cy*tp[0]-sy*tp[1]);
      const oy=py-(sy*tp[0]+cy*tp[1]);
      window._dragCalc={px,py,zc,tp,pc:d.pc,c0:d.c0,z0:d.z0,
                        dragZ:d.dragZ,yaw:yw,ox,oy};   // 점검용
      setTarget(dragEnv.id,[cy,-sy,0,ox, sy,cy,0,oy, 0,0,1,(d.dragZ||0), 0,0,0,1]);
    }
  }
  const ad=1-Math.exp(-dt*30);         // 드래그 추종은 더 빠릿하게 (~33ms)
  for(const id in objs){
    const o=objs[id];
    if(!o.mesh||!o.init)continue;
    if(o.env){
      if(dragEnv.on&&id===dragEnv.id){ o.cp.lerp(o.tp,ad); o.cq.slerp(o.tq,ad); }
      else { o.cp.copy(o.tp); o.cq.copy(o.tq); }     // 환경: 즉시
    }
    else { o.cp.lerp(o.tp,a); o.cq.slerp(o.tq,a); }  // 로봇: 부드럽게
    _ss.set(o.s,o.s,o.s);
    o.mesh.matrix.compose(o.cp,o.cq,_ss);
    o.mesh.visible=(o.vis!==false);
  }
  if(!camSync.fp) ctl.update();
  if(camSync.fp) camSyncRender(); else if(VR.on) vrRender(); else ren.render(scene,cam);
})(performance.now());
</script></body></html>"""

ROBOT_COLOR_MAP=dict(zip(ROBOT_LINKS,ROBOT_COLORS))
# 로봇을 바꾼 횟수. 브라우저가 예전 로봇의 STL 을 계속 쓰지 않도록
# /mesh/<링크>?g=<세대> 로 주소를 바꿔 새로 받게 한다.
ROBOT_GEN=[0]
_PINC={}
def _pin_centroid(oid):
    """드래그 추적용: 설비 다리(핀)들의 XY 중심 (로컬 m). 핀 없으면 None."""
    if oid in _PINC: return _PINC[oid]
    try:
        ps=env_pins_local(oid)
        _PINC[oid]=([float(np.mean([q[0] for q in ps])),
                     float(np.mean([q[1] for q in ps]))] if len(ps) else None)
    except Exception:
        _PINC[oid]=None
    return _PINC[oid]
# ===== 카메라 동기화 (cam_grid/camsync.py) =====
#   녹화 영상에서 구한 카메라 위치를 3D 뷰어에 띄우고, 1인칭(카메라 눈) 시점을 준다.
#   영상 속 판 가운데를 앱 바닥판들의 가운데에 맞춘다. 판 무늬가 대칭이라 회전은
#   사용자가 1인칭에서 로봇·도킹이 실물과 겹치게 고른다 (h_cam_ctl('rot')).
_CAMSYNC=[None]
# ★ 카메라 동기화 끔 (2026-10-20, 사용자: '아직은 안 쓸 거야') ★
#   True 로 바꾸면 그대로 다시 켜진다. 끄면 모듈을 안 불러오고, 자동 반영 스레드도 안 뜨고,
#   /state 의 camsync 는 None 이라 뷰어도 아무것도 안 그린다.
CAM_ENABLED=False
def _camsync():
    if not CAM_ENABLED: return None
    if _CAMSYNC[0] is None:
        try:
            _d=os.path.join(os.path.dirname(os.path.abspath(__file__)),'cam_grid')
            if _d not in sys.path: sys.path.insert(0,_d)
            import camsync as _cs
            _CAMSYNC[0]=_cs.SYNC
        except Exception as _e:
            print('[카메라 동기화] 모듈을 못 불러왔습니다:',_e); _CAMSYNC[0]=False
    return _CAMSYNC[0] or None
def _plates_t0():
    """앱 바닥판 중 왼쪽 아래 타일."""
    ts=list(PLATES.keys())
    return (min(t[0] for t in ts),min(t[1] for t in ts)) if ts else (0,0)
def _plates_origin():
    """영상 판의 첫 목심을 맞출 자리 = 왼쪽 아래 바닥판의 첫 목심 (타일 안 10mm)."""
    ox,oy=tile_origin(_plates_t0())
    return (ox+PEG_PITCH/2,oy+PEG_PITCH/2)
# ---- 📱 휴대폰 실시간 카메라 (2026-10-22) ----
#   녹화 영상용 카메라 동기화(CAM_ENABLED) 와 따로 돈다. 휴대폰이 프레임을 보내는 동안만 켜진다.
#   지금 단계: 카메라 위치·시야를 트윈에 보인다 (배치는 아직 건드리지 않는다).
_LIVE=[None]
def _livecam():
    if _LIVE[0] is None:
        try:
            _d=os.path.join(os.path.dirname(os.path.abspath(__file__)),'cam_grid')
            if _d not in sys.path: sys.path.insert(0,_d)
            import livecam as _lc
            _LIVE[0]=_lc.LIVE
            if LIVE_APPLY['thread'] is None:
                LIVE_APPLY['thread']=threading.Thread(target=_live_apply_loop,daemon=True)
                LIVE_APPLY['thread'].start()
        except Exception as _e:
            print('[휴대폰 카메라] 모듈을 못 불러왔습니다:',_e); _LIVE[0]=False
    return _LIVE[0] or None
def h_live_ctl(cmd):
    lv=_livecam()
    if not lv: return '휴대폰 카메라 모듈이 없습니다'
    cmd=str(cmd)
    if cmd=='apply':                            # 자동 반영 켜기/끄기
        LIVE_APPLY['on']=not LIVE_APPLY['on']
        return f"자동 반영 {'켬' if LIVE_APPLY['on'] else '끔'} | "+h_live_status()
    if cmd in ('reg','reset'):                  # 판 맞춤 다시
        LIVE_APPLY['reg']=None; LIVE_TRACKS.clear()
        if cmd=='reg': lv.reg=None; return '판을 로봇 받침으로 다시 맞춥니다 | '+h_live_status()
    if cmd in ('rot','x+','x-','y+','y-'):      # 손으로 맞추면 자동 맞춤을 끈다
        LIVE_APPLY['reg']='manual'
    return lv.ctl(cmd)
def h_live_status():
    lv=_livecam()
    if not lv: return '휴대폰 카메라 모듈이 없습니다'
    s=lv.summary()
    if lv.live():
        r=LIVE_APPLY.get('reg')
        s+=(' | 판 맞춤: '+('손으로' if r=='manual' else ('로봇 받침 기준' if r else '로봇 받침 찾는 중')))
        s+=f" | 자동 반영 {'켬' if LIVE_APPLY['on'] else '끔'}"
        if LIVE_APPLY.get('last'): s+=' | '+LIVE_APPLY['last']
    return s

# ---- 📱 휴대폰 카메라 -> 앱 반영 (2026-10-22) ----
#   ★ 바닥판은 안 바뀐다 (기준) · 로봇·도킹은 움직일 수 있다 (사용자, 10-26) ★ 로봇은 꾸준히 다른 자리에서 보일 때만,
#     실행 중이 아닐 때만 옮긴다 (set_robot_pose = 매크로 상태 저장, calib 는 안 건드림).
#   ★ 도킹은 지우지 않고 숨긴다 ★ 블록이 '#10 도킹' 처럼 이름으로 가리키므로, 지웠다 다시 만들면
#     이름이 바뀌어 블록이 끊긴다. 다시 보이면 같은 도킹을 다시 보이게 해서 옮긴다.
#   판 맞춤: 카메라가 찾은 로봇 받침 자리 = 앱의 로봇 받침 자리, 방향(90°×k) 은 앱 바닥판이
#     영상 판 안에 가장 많이 들어가는 쪽, 마지막에 타일 경계(200mm)에 끌어당긴다.
LIVE_APPLY={'on':True,'reg':None,'last':'','thread':None}
LIVE_TRACKS=[]                 # [{'xy','votes','oid','gone'}]
LIVE_FREE_CNT={}               # 도킹 id -> 연속 '비어 보임' 횟수
LIVE_ROBOT={'hits':[]}         # 앱과 다른 자리에서 본 로봇 받침 (연속 3번 같은 자리면 옮긴다)
LIVE_ROBOT_SECS=2.0            # 그 다른 자리가 이만큼(초) 넘게 이어져야
LIVE_ROBOT_TOL=0.09            # 이보다 멀어야 '옮겨짐' (실측: 안 옮겼을 때 받침 인식 차이 50~67mm 라 그보다 크게)
LIVE_DOCK_MIN_CELLS=4          # 다리 4개가 다 보여야 도킹 (10-22: 바닥색 차이 판정은 늘 4 를 보고한다 — 7 이면 하나도 안 들어갔다)

def _rz3(k):
    a=k*math.pi/2; c,s=round(math.cos(a)),round(math.sin(a))
    return np.array([[c,-s,0],[s,c,0],[0,0,1.0]])

def _live_register(lay):
    """판(창 좌표, m) -> 앱 월드 (R, t).
    ★ 바닥판 = 고정 2×4 (PLATE_LAYOUT, 사용자 11-03) ★ 카메라 판 창(목심 20×40) = 앱 바닥판 8장 전체.
      2×4 체크무늬는 180° 돌려도 똑같아 보이므로, 두 방향 중 '앱에 지금 있는 로봇·도킹 자리' 와 더 맞는 쪽 (= 이전 자리 유지).
      판 8장의 양 끝 줄이 다 보여야(창 = 판) 하고, 방향을 가를 로봇·도킹이 하나도 안 보이면 기다린다.
      (예전 3×3 판: 4방향 × 타일 이동을 따지고 모자란 앱 판을 더했다 — 이제 판은 늘 그 8장)
      계산은 cam_grid/plate_fit.py (앱 없이 시험할 수 있게)."""
    if not lay: return None
    import plate_fit as _pf                      # cam_grid 는 _livecam() 이 이미 경로에 넣었다
    fw=l2w(np.array([*_cam_robot_foot_local(),0.0]))
    tq=tool_pos([0.0]*NJOINT)                  # 로봇 팔이 뻗는 방향 (기본 자세의 툴 - 받침)
    fwd=np.array([tq[0]-fw[0],tq[1]-fw[1]]); fwd=fwd/(np.linalg.norm(fwd)+1e-9)
    app_docks=[xy for xy in (_cam_dock_xy(o) for o in docking_ids() if '#' in o
                             and (_env_ent(o) or {}).get('visible',True)) if xy]
    r=_pf.fit(lay,PLATE_LAYOUT,tile_origin,PEG_PITCH,PLATE_SIZE,PLATE_TOP,(fw[0],fw[1]),fwd,app_docks)
    if r: r['cover']=len(PLATES)
    return r

def _live_board_to_world(reg,ab):
    return reg['R']@np.array([ab[0]*PEG_PITCH,ab[1]*PEG_PITCH,0.0])+reg['t']

def _live_free(lay,reg,xy,r=1):
    """앱 월드 (x,y) 자리가 카메라에 '또렷이 비어' 보이는가."""
    fr=lay.get('free')
    if fr is None: return False
    b=reg['R'].T@(np.array([xy[0],xy[1],reg['t'][2]])-reg['t'])
    a0,b0=int(round(b[0]/PEG_PITCH)),int(round(b[1]/PEG_PITCH))
    ni,nj=np.asarray(fr).shape[:2]              # 판 칸 수 (2×4 판 = 20×40)
    cells=[(a0+i,b0+j) for i in range(-r,r+1) for j in range(-r,r+1) if 0<=a0+i<ni and 0<=b0+j<nj]
    return bool(cells) and sum(bool(fr[c]) for c in cells)>=0.6*len(cells)

def _live_set_visible(oid,v):
    e=_env_ent(oid)
    if e is None or bool(e.get('visible',True))==bool(v): return False
    e['visible']=bool(v)
    with LOCK: S.setdefault('env_vis',{})[oid]=bool(v)
    _OBS['grid']=None; _clear_plan_cache(); _macro_save()
    return True

def _live_apply_once(lv):
    lay=lv.layout()
    if not lay: return
    msgs=[]
    reg=LIVE_APPLY.get('reg')
    if reg=='manual': return                  # 손으로 맞추는 중엔 배치를 건드리지 않는다
    # ★ 바닥판은 안 바뀐다 — 한 번 맞추면 유지 (사용자, 10-26) ★
    #   같은 판(Board) 에서 창만 옮겨졌으면 맞춤을 정확히 그만큼 옮긴다 (칸 번호는 이어진다).
    #   판을 처음부터 다시 잡았거나 거울상이 바뀐 때만 다시 맞춘다 — 그때도 앱의 이전 자리와 가장 맞는 방향으로.
    if reg is not None and (reg.get('board_id')!=lay.get('board_id') or reg.get('mirror')!=lay.get('mirror')
                            or reg.get('cont')!=lay.get('cont')):
        reg=None; LIVE_APPLY['reg']=None; lv.reg=None
    if reg is not None and reg.get('win')!=lay.get('win') and lay.get('win') and reg.get('win'):
        di=lay['win'][0]-reg['win'][0]; dj=lay['win'][1]-reg['win'][1]
        d=np.array([di,-dj if lay.get('mirror') else dj,0.0])*PEG_PITCH
        reg['t']=reg['t']+reg['R']@d; reg['win']=tuple(lay['win']); lv.reg=(reg['R'],reg['t'])
    if reg is None:
        reg=_live_register(lay)
        if reg is None:
            LIVE_APPLY['last']='판을 맞추는 중 — 판 8장 전체와 로봇(또는 도킹)이 함께 보이게 비춰 주세요 (그동안 이전 자리 유지)'; return
        reg.update(win=tuple(lay['win']) if lay.get('win') else None,board_id=lay.get('board_id'),
                   mirror=lay.get('mirror'),cont=lay.get('cont'))
        LIVE_APPLY['reg']=reg; lv.reg=(reg['R'],reg['t'])
        msgs.append(f"판 맞춤: 앱 바닥판 2×4 기준 · {reg['k']*90}° · 이전 자리와 차이 {reg['cons']*1000:.0f}mm")
    # 판 맞춤 증거: 도킹을 옮기기 '전' 앱 도킹 자리와 카메라 도킹이 4cm 안으로 맞는 게 하나라도 있나 (로봇 옮기기 조건)
    app_dk=[xy for xy in (_cam_dock_xy(o) for o in docking_ids() if '#' in o
                          and (_env_ent(o) or {}).get('visible',True)) if xy]
    anchor=any(min((math.dist(xy,(w[0],w[1])) for xy in app_dk),default=1.0)<0.04
               for w in (_live_board_to_world(reg,d['center']) for d in lay['dockings']))
    # 바닥판은 더하지 않는다 — 늘 기본 2×4 (PLATE_LAYOUT, 11-03). 예전 3×3 판 시절엔 본 타일만큼 앱 판을 더했다.
    # 도킹: 색이 같은 앱 도킹을 데려온다 (숨긴 것 포함). 없으면 숨긴 도킹 색을 바꿔 쓰고, 그것도 없으면 복제.
    seen=set()
    for d in lay['dockings']:
        if d['n']<LIVE_DOCK_MIN_CELLS: continue
        w=_live_board_to_world(reg,d['center']); xy=(float(w[0]),float(w[1]))
        def _near(tr):
            dd=math.dist(tr['xy'],xy); same=(max(tr['votes'],key=tr['votes'].get)==d['hex']) if tr['votes'] else False
            return dd<=(0.16 if same else 0.06)
        tr=min((t for t in LIVE_TRACKS if _near(t)),key=lambda t:math.dist(t['xy'],xy),default=None)
        if tr is None:
            tr={'xy':list(xy),'votes':{},'oid':None,'gone':0}; LIVE_TRACKS.append(tr)
        tr['xy']=[0.7*tr['xy'][0]+0.3*xy[0],0.7*tr['xy'][1]+0.3*xy[1]]
        tr['votes'][d['hex']]=tr['votes'].get(d['hex'],0)+1; tr['gone']=0
        # 방향: 윗판 U 홈이 열린 쪽 (판 좌표 단위벡터) -> 앱 월드 yaw. 도킹 STL 의 홈은 로컬 -y 쪽이라
        #   Rz(yaw)@(0,-1) = 홈 방향 -> yaw = atan2(dx,-dy). 90° 단위로 다수결.
        if d.get('yaw'):
            dw=reg['R'][:2,:2]@np.array(d['yaw']['dir'],float)
            qk=int(round(math.degrees(math.atan2(dw[0],-dw[1]))/90.0))%4
            yv=tr.setdefault('yawv',{})
            for _k in yv: yv[_k]*=0.8                # 옛 판정은 서서히 잊는다 (도킹을 돌려 꽂으면 따라가게)
            yv[qk]=yv.get(qk,0)+1
        seen.add(id(tr))
        col=max(tr['votes'],key=tr['votes'].get)
        if tr['oid'] is None or _env_ent(tr['oid']) is None:
            taken={t['oid'] for t in LIVE_TRACKS if t['oid']}
            pool=[o for o in docking_ids() if '#' in o and o not in taken]
            same=[o for o in pool if _snap_color((_names().get(o) or {}).get('color') or '')==col]
            hidden=[o for o in pool if not (_env_ent(o) or {}).get('visible',True)]
            if same:   tr['oid']=min(same,key=lambda o:math.dist(_cam_dock_xy(o) or (9,9),tr['xy']))
            elif hidden: tr['oid']=hidden[0]
            else:
                # 복제할 원본: 화면에 없어도(공장 초기화 직후) 도킹 메쉬는 등록돼 있다
                base=_cam_dock_base() or next((m for m in MESH_REG
                                               if 'docking' in m.lower() and '#' not in m),None)
                if base is None: continue
                tr['oid']=env_clone(base)
                if tr['oid'] is None: continue
                msgs.append(f'도킹 추가 ({col})')
        oid=tr['oid']
        if _live_set_visible(oid,True): msgs.append(f'{disp_name(oid)} 다시 보임')
        if _snap_color((_names().get(oid) or {}).get('color') or '')!=col:
            set_name(oid,color=col); msgs.append(f'{disp_name(oid)} 색 -> {col}')
        cur=_cam_dock_xy(oid)
        turn=False
        if tr.get('yawv'):
            qk=max(tr['yawv'],key=tr['yawv'].get); want=qk*math.pi/2
            e=_env_ent(oid)
            if e is not None and abs((float(e.get('yaw',0.0))-want+math.pi)%(2*math.pi)-math.pi)>math.radians(5):
                e['yaw']=want; turn=True             # move_env 가 이 yaw 로 가운데를 맞춰 저장한다
        if turn or cur is None or math.dist(cur,tr['xy'])>0.03:
            ok,_m=move_env(oid,tr['xy'][0],tr['xy'][1],snap=True)
            if ok: msgs.append(f'{disp_name(oid)} '+(f'방향 {qk*90}°' if turn else '이동'))
    # 로봇: 카메라가 본 받침 자리가 앱과 꾸준히 다르면(옮겨짐) 앱 로봇을 옮긴다 (사용자, 10-26: 로봇은 움직일 수 있다).
    #   방향은 지금 값 그대로, 로봇 핀이 목심에 오게 20mm 격자로 맞춘다. 실행 중에는 절대 안 옮긴다.
    #   안 보이면 아무것도 안 한다 = 이전 자리 유지.
    #   ★ 판 맞춤이 맞다는 증거가 있어야 옮긴다 (10-26: 엉뚱한 받침 인식으로 로봇이 잘못 옮겨졌다) ★
    #     도킹 하나 이상이 앱 자리와 4cm 안으로 맞고, 2초 넘게 같은 다른 자리에서 보여야 한다.
    rb=lay.get('robot')
    if rb is not None and anchor and LIVE_APPLY.get('robot',True) and not RUNNING.is_set():
        w=_live_board_to_world(reg,rb)
        fw=l2w(np.array([*_cam_robot_foot_local(),0.0]))
        hits=LIVE_ROBOT['hits']
        if math.hypot(w[0]-fw[0],w[1]-fw[1])>LIVE_ROBOT_TOL:
            hits.append((float(w[0]),float(w[1]),time.time())); del hits[:-8]
            same=[q for q in hits if math.dist(hits[-1][:2],q[:2])<0.03]
            if len(same)>=3 and same[-1][2]-same[0][2]>=LIVE_ROBOT_SECS:
                x,y=np.mean([q[:2] for q in same],0)
                yaw=BASE_YAW[0]; R2=_rz2(yaw)
                off=np.array([x,y])-R2@np.array(_cam_robot_foot_local())
                pin=R2@np.asarray(robot_pins_local()[0][:2],float)+off
                g=np.array([GRID_ORG[0]+PEG_PITCH/2,GRID_ORG[1]+PEG_PITCH/2])
                off=off+(g+np.round((pin-g)/PEG_PITCH)*PEG_PITCH-pin)
                set_robot_pose(off[0],off[1],yaw)
                msgs.append(f"로봇 옮겨짐 -> 받침 ({x*1000:.0f},{y*1000:.0f})mm 로 이동")
                hits.clear()
        else:
            hits.clear()
    tracked={t['oid'] for t in LIVE_TRACKS if t['oid'] and id(t) in seen}
    for o in [d for d in docking_ids() if '#' in d and d not in tracked]:
        e=_env_ent(o); xy=_cam_dock_xy(o)
        if not e or not e.get('visible',True) or not xy: continue
        if _live_free(lay,reg,xy):
            LIVE_FREE_CNT[o]=LIVE_FREE_CNT.get(o,0)+1
            if LIVE_FREE_CNT[o]>=2 and _live_set_visible(o,False):
                msgs.append(f'{disp_name(o)} 자리가 비어 보임 (숨김)')
        else: LIVE_FREE_CNT[o]=0
    for tr in LIVE_TRACKS:
        if id(tr) in seen or not tr['oid']: continue
        if _live_free(lay,reg,tr['xy']):
            tr['gone']+=1
            if tr['gone']>=2 and _live_set_visible(tr['oid'],False):
                msgs.append(f"{disp_name(tr['oid'])} 사라짐 (숨김)")
        else: tr['gone']=0
    if msgs:
        LIVE_APPLY['last']=time.strftime('%H:%M:%S ')+' · '.join(msgs)
        print('[휴대폰 배치]',LIVE_APPLY['last'])

def _live_apply_loop():
    while True:
        time.sleep(0.5)
        lv=_LIVE[0]
        if not lv or not LIVE_APPLY['on'] or not lv.live(): continue
        try: _live_apply_once(lv)
        except Exception as _e: print('[휴대폰 배치] 반영 실패:',type(_e).__name__,_e)

def _camsync_state():
    lv=_livecam()
    if lv is not None and lv.live():            # 휴대폰이 보내는 중이면 그쪽이 우선
        try: return lv.state(_plates_origin(),PLATE_TOP)
        except Exception as _e: return {'on':False,'status':f'휴대폰 카메라 오류: {_e}'}
    cs=_camsync()
    if cs is None: return None
    try: return cs.state(_plates_origin(),PLATE_TOP)
    except Exception as _e: return {'on':False,'status':f'오류: {_e}'}

# ---- 카메라가 읽은 배치를 앱에 자동 반영 ----
#   ★ 손 배치 기능은 고치지 않는다 ★ 아래는 그 기능들이 쓰는 함수(set_robot_pose,
#   env_clone, set_name, move_env, env_delete)를 '부르기만' 한다.
#   도킹은 '추적 대상' 으로 관리한다: 실제 도킹 하나 = 추적 하나. 색은 그동안 읽힌 색의 다수결.
#     - 새로 보이면: 같은 색의 앱 도킹을 데려오고, 없으면 복제해서 만든다
#     - 옮겨지면: 5cm 넘게 벗어날 때만 옮긴다 (흔들림 무시)
#     - 안 보이면: 그 자리가 '또렷이 비어 있음' 이 두 번 연속일 때만 지운다
#       (아직 덜 봐서 판정이 없는 것과 뽑아서 없어진 것을 구별)
CAM_TRACKS=[]            # [{'xy':[x,y],'votes':{hex:n},'oid':id,'gone':n}]
CAM_ROBOT={}             # {'xy':(x,y)} 카메라가 놓은 로봇 받침 자리
_CAM_FOOT=[None]
def _cam_robot_foot_local():
    """로봇 받침 바닥 가운데 (로봇 로컬, m). base_link 메쉬의 맨 아래 3mm 면 가운데.
    메쉬를 못 읽으면 바닥 핀 두 개의 가운데."""
    if _CAM_FOOT[0] is None:
        try:
            import trimesh as _tm
            v=np.asarray(_tm.load(os.path.join(MESH_DIR,'base_link.STL')).vertices,float)
            lo=v[v[:,2]<v[:,2].min()+0.003]
            _CAM_FOOT[0]=(float(lo[:,0].mean()),float(lo[:,1].mean()))
        except Exception:
            ps=robot_pins_local()
            _CAM_FOOT[0]=(float((ps[0][0]+ps[1][0])/2),float((ps[0][1]+ps[1][1])/2))
    return _CAM_FOOT[0]
CAM_APPLY={'thread':None,'last':''}
def _cam_dock_xy(did):
    t=docking_top(did); return (float(t[0]),float(t[1])) if t else None
def _cam_visible_docks():
    return [d for d in docking_ids() if '#' in d
            and (_env_ent(d) is None or _env_ent(d).get('visible',True))]
def _cam_dock_base():
    return next((d for d in docking_ids() if '#' not in d),None)
def _cam_apply_once(cs):
    lay=cs.layout(_plates_origin(),PLATE_TOP)
    if not lay: return
    msgs=[]
    # 로봇: 위치만 (방향은 지금 값 그대로).
    #   ★ 한 번 놓으면 원래 자리가 '또렷이 비었다' 는 증거가 있을 때만 옮긴다 ★
    #   팔이 판 위로 뻗으면 팔 밑이 크게 가려져 받침처럼 보일 때가 있다 (실측: 30cm 튐).
    #   진짜 받침 자리는 로봇이 서 있는 한 절대 비어 보이지 않으므로 그 자리에 고정된다.
    #   ★ 카메라가 읽는 건 '받침 바닥 가운데' 인데, 앱의 로봇 위치(BASE_OFF)는 로봇 기준점(원점)이다.
    #     받침 바닥 가운데는 기준점에서 (241,-258)mm 떨어져 있다 (base_link 메쉬 실측)
    #     → 받침 바닥 가운데가 읽은 자리에 오도록 기준점을 거꾸로 계산한다.
    # ★ 받침 판의 목심 구멍 줄로 위치 + 방향까지 찾았으면 그것을 쓴다 (영상 하나에 한 번) ★
    rp=lay.get('robot_pose')
    if rp is not None:
        if not CAM_ROBOT.get('pose_done'):
            set_robot_pose(rp['base_off'][0],rp['base_off'][1],rp['yaw'])
            CAM_ROBOT['pose_done']=True
            CAM_ROBOT['xy']=tuple(l2w(np.array([*_cam_robot_foot_local(),0.0]))[:2])
            msgs.append(f"로봇 받침 구멍으로 배치: 방향 {math.degrees(rp['yaw'])%360:.0f}°")
    elif lay['robot'] is not None:
        x,y=lay['robot']; old=CAM_ROBOT.get('xy')
        if old is None or (math.dist(old,(x,y))>0.03 and cs.is_free_world(lay,old,r_cells=2)):
            fl=np.array([*_cam_robot_foot_local(),0.0])
            cur=l2w(fl)                                   # 지금 받침 바닥 가운데 (월드)
            if math.hypot(x-cur[0],y-cur[1])>0.03:
                b=np.array([x,y,0.0])-_yawR()@fl
                set_robot_pose(float(b[0]),float(b[1]),BASE_YAW[0])
                msgs.append(f'로봇 받침 -> ({x*1000:.0f},{y*1000:.0f})mm')
            CAM_ROBOT['xy']=(x,y)
    seen=set()
    for d in lay['dockings']:
        xy=d['xy']
        # 같은 색이면 16cm 안까지 같은 도킹 (덩어리 중심이 흔들려도 지웠다 다시 만들지 않게),
        # 색이 달라도 6cm 안이면 같은 도킹 (같은 의자가 잠깐 다른 색으로 읽힌 것)
        def _near(t):
            dd=math.dist(t['xy'],xy); same=max(t['votes'],key=t['votes'].get)==d['hex'] if t['votes'] else False
            return dd<=(0.16 if same else 0.06)
        tr=min((t for t in CAM_TRACKS if _near(t)),key=lambda t:math.dist(t['xy'],xy),default=None)
        if tr is None:
            tr={'xy':list(xy),'votes':{},'oid':None,'gone':0}; CAM_TRACKS.append(tr)
        tr['xy']=[0.7*tr['xy'][0]+0.3*xy[0],0.7*tr['xy'][1]+0.3*xy[1]]
        tr['votes'][d['hex']]=tr['votes'].get(d['hex'],0)+1; tr['gone']=0
        seen.add(id(tr))
        col=max(tr['votes'],key=tr['votes'].get)
        if tr['oid'] is None or _env_ent(tr['oid']) is None:
            taken={t['oid'] for t in CAM_TRACKS if t['oid']}
            cands=[o for o in _cam_visible_docks() if o not in taken
                   and _snap_color((_names().get(o) or {}).get('color') or '')==col]
            if cands:
                tr['oid']=min(cands,key=lambda o:math.dist(_cam_dock_xy(o) or (9,9),tr['xy']))
            else:
                base=_cam_dock_base()
                if base is None: continue
                tr['oid']=env_clone(base)
                if tr['oid'] is None: continue
                set_name(tr['oid'],color=col); msgs.append(f'도킹 추가 ({col})')
        oid=tr['oid']
        if _snap_color((_names().get(oid) or {}).get('color') or '')!=col:
            set_name(oid,color=col); msgs.append(f'{disp_name(oid)} 색 -> {col}')
        cur=_cam_dock_xy(oid)
        if cur is None or math.dist(cur,tr['xy'])>0.05:
            ok,_m=move_env(oid,tr['xy'][0],tr['xy'][1],snap=True)
            if ok: msgs.append(f'{disp_name(oid)} 이동')
    for tr in list(CAM_TRACKS):
        if id(tr) in seen: continue
        if cs.is_free_world(lay,tr['xy']):
            tr['gone']+=1
            if tr['gone']>=2:
                if tr['oid'] and _env_ent(tr['oid']) is not None:
                    _ok,_m=env_delete(tr['oid']); msgs.append(f'도킹 제거 ({_m})')
                CAM_TRACKS.remove(tr)
        else: tr['gone']=0
    if msgs:
        CAM_APPLY['last']=time.strftime('%H:%M:%S ')+' · '.join(msgs)
        print('[카메라 배치]',CAM_APPLY['last'])
def _cam_apply_loop():
    while True:
        time.sleep(0.5)
        cs=_camsync()
        if not cs or not cs.ready or not cs.auto: continue
        try: _cam_apply_once(cs)
        except Exception as _e: print('[카메라 배치] 반영 실패:',_e)
def h_cam_load(path):
    cs=_camsync()
    if not cs: return '카메라 동기화 모듈이 없습니다'
    CAM_TRACKS.clear(); CAM_ROBOT.clear()
    cs.robot_mesh=os.path.join(MESH_DIR,'base_link.STL')    # 로봇 받침 자세(방향) 찾기에 쓴다
    if CAM_APPLY['thread'] is None:
        CAM_APPLY['thread']=threading.Thread(target=_cam_apply_loop,daemon=True); CAM_APPLY['thread'].start()
    return cs.load(path)
def h_cam_ctl(cmd):
    cs=_camsync(); return cs.ctl(cmd) if cs else '카메라 동기화 모듈이 없습니다'
def h_cam_status():
    cs=_camsync()
    if not cs: return '카메라 동기화 모듈이 없습니다'
    s=cs.summary()
    if cs.ready:
        need=len(cs.tiles_needed())
        if need and need!=len(PLATES):
            s+=f" | ★ 영상 판 {need}타일 · 앱 바닥판 {len(PLATES)}장 → [바닥판 맞추기] 권장"
    if CAM_APPLY['last']: s+=' | 마지막 반영: '+CAM_APPLY['last']
    return s
def h_cam_fit_plates():
    """영상에서 읽은 판 크기(타일)대로 앱 바닥판을 더한다 (add_plate 를 부르기만). 빼지는 않는다."""
    cs=_camsync()
    if not cs or not cs.ready: return '먼저 영상을 불러오세요'
    t0=_plates_t0(); added=[]
    for u,v in cs.tiles_needed():
        t=(t0[0]+u,t0[1]+v)
        if t not in PLATES and add_plate(t): added.append(t)
    if added: _peg_layout_changed()
    return (f'바닥판 {len(added)}장 추가 → 지금 {len(PLATES)}장' if added
            else f'추가할 바닥판이 없습니다 (지금 {len(PLATES)}장)')

class _ViewerHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self,*a): pass
    def _send(self,code,ctype,body):
        self.send_response(code)
        self.send_header('Content-Type',ctype)
        self.send_header('Content-Length',str(len(body)))
        # 앱(7900)과 뷰어(7870)는 포트가 달라, 이 허락이 없으면 브라우저가
        # 조종표 조각 요청을 통째로 막는다. 그러면 표가 안 실려
        # 모든 움직임이 서버 IK 로 떨어지고 전 구간이 끊긴다.
        self.send_header('Access-Control-Allow-Origin','*')
        self.end_headers(); self.wfile.write(body)
    def do_GET(self):
        try:
            if self.path=='/' or self.path.startswith('/index'):
                self._send(200,'text/html',VIEWER_HTML.encode())
            elif self.path=='/state':
                if len(arms_shown())>1:            # 두 팔 (10-30)
                    links=arm_links()
                else:
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
                    # 색의 원본은 names (영구) - 재시작해도 지정색 유지
                    # 단, 바닥 플레이트는 제외 (옛날 색 기록이 바닥을 칠하지 않게)
                    env.append({'id':e['id'],
                                'color':((plate_color(e['tile']) if e.get('tile') else e['color'])
                                         if _is_floor_plate(e['id'])   # 바닥판 = 흰·파 체크무늬 (11-03, 프리셋을 불러도)
                                         else ((S.get('names',{}).get(e['id'])
                                                or {}).get('color'))
                                              or e['color']),
                                'scale':MESH_REG[e['id']][1],
                                'v':e.get('visible',True),
                                'off':list(e.get('offset',(0,0,0))),
                                'yaw':float(e.get('yaw',0.0)),
                                'tile':list(e['tile']) if e.get('tile') else None,
                                'plate':_is_floor_plate(e['id']),
                                # 드래그 중인 설비만: 다리(핀) 중심 (로컬 m)
                                # -> 뷰어가 '다리가 커서 밑' 기준으로 추적
                                'pc':(_pin_centroid(e['id'])
                                      if DRAG_ENV['id']==e['id'] else None)})
                # 배치된 물체(브릭)도 3D 에 그린다.
                # 잡혀 있는 물체는 '그리퍼에 붙어서' 따라다닌다 (축=툴 중심,
                # 바닥 = 툴 z - 잡는높이). 놓으면 다시 놓인 자리에 그려진다.
                for _bid,_b in BRICKS.items():
                    _held=(HELD.get('brick')==_bid)
                    if _b.get('pos') is None and not _held: continue
                    if _held:
                        _bx=float(_tp[0]); _by=float(_tp[1])
                        _bz=float(_tp[2])-float(_b.get('grasp_dz',0.037))
                    else:
                        _bx,_by,_bz=_b['pos']
                    env.append({'id':_bid,
                                'color':((S.get('names',{}).get(_bid) or {})
                                         .get('color')) or '#e0c020',
                                'scale':1.0,'v':True,'brick':True,
                                'off':[_bx-_b.get('cx',0.0),
                                       _by-_b.get('cy',0.0),
                                       _bz-_b['z_bottom']],
                                'yaw':0.0,'tile':None,'plate':False,'pc':None})
                if PLATE_EDIT['on']:
                    ghosts=[{'tile':list(t),
                             'x':tile_origin(t)[0],'y':tile_origin(t)[1]}
                            for t in ghost_tiles()]
                else:
                    ghosts=[]   # 로봇 이동은 핀/목심 클릭 (타일 강조 없음)
                rmv=None
                if ENV_MOVE['on']:               # 설비 2핀 배치 (뷰어 표시 공유)
                    rmv={'phase':ENV_MOVE['phase'],
                         'pin':ENV_MOVE.get('pin'),
                         'pins':[[float(q[0]),float(q[1])]
                                 for q in env_pins_world(ENV_MOVE['id'])],
                         'pegs':([] if ENV_MOVE['phase']==0 else
                                 (ENV_MOVE['cands'] if ENV_MOVE['phase']==2
                                  else [[x,y] for x,y in _pegs_all_world()])),
                         'ptop':PLATE_TOP}
                elif ROBOT_MOVE['on']:
                    rmv={'phase':ROBOT_MOVE['phase'],
                         'pin':ROBOT_MOVE.get('pin'),
                         'pins':[[float(p[0]),float(p[1])] for p in robot_pins_world()],
                         'pegs':([] if ROBOT_MOVE['phase']==0 else
                                 (ROBOT_MOVE['cands'] if ROBOT_MOVE['phase']==2
                                  else [[x,y] for x,y in _pegs_all_world()])),
                         'ptop':PLATE_TOP}
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
                # 물체 드래그 중: 빈 도킹 소켓 = 자석 스냅 지점 (결합된 곳 제외)
                snaps=[]
                _did_=DRAG_ENV.get('id')
                if _did_ and _clone_base(_did_) in BRICKS:
                    for _dk in docking_ids():
                        _e2=_env_ent(_dk)
                        if _e2 is not None and not _e2.get('visible',True):
                            continue
                        if dock_occupied(_dk,except_bid=_did_): continue
                        _sw=dock_socket_world(_dk)
                        if _sw: snaps.append([round(_sw[0],4),round(_sw[1],4)])
                body=json.dumps({'pegs':pegs,'ptop':PLATE_TOP,'rmove':rmv,
                    'snaps':snaps,
                    'pegsel':PEG_VIEW.get('sel'),
                    'dragid':DRAG_ENV['id'],
                    'gen':ROBOT_GEN[0],
                    'headgen':HEAD_GEN[0],
                    'links':links,'env':env,'ghosts':ghosts,
                    'psize':PLATE_SIZE,'sel':PLATE_EDIT.get('sel'),
                    'ee':f"x={p[0]*1000:.0f} y={p[1]*1000:.0f} z={p[2]*1000:.0f}mm",
                    'eexyz':[float(v) for v in p],
                    'toolxyz':([float(v) for v in _tp] if TOOL_HAS else None),
                    'tool':(f"x={_tp[0]*1000:.0f} y={_tp[1]*1000:.0f} "
                            f"z={_tp[2]*1000:.0f}mm" if TOOL_HAS else None),
                    'camsync':_camsync_state(),
                    'calpts':(calmap_points() if CALMAP['on'] else None),
                    'path':PATH_VIS['pts']
                    }).encode()
                self._send(200,'application/json',body)
            elif self.path.startswith('/camframe/'):     # 카메라 동기화: k 번째 프레임 JPEG
                _cs=_camsync(); _b=None
                _lv=_livecam()
                try:
                    if _lv is not None and _lv.live(): _b=_lv.jpeg()     # 휴대폰: 가장 최근 프레임
                    else: _b=_cs.jpeg(int(self.path[len('/camframe/'):].split('?')[0])) if _cs else None
                except Exception: _b=None
                if _b: self._send(200,'image/jpeg',_b)
                else: self._send(404,'text/plain',b'no frame')
            elif self.path=='/emoff':
                if ENV_MOVE['on']:
                    print(f"[설비 배치] 취소: {disp_name(ENV_MOVE['id'])}")
                    emove_reset()
                elif ROBOT_MOVE['on']:           # 11-03: 로봇 팔 옮기기도 ESC 로 취소 (공장 꾸미기 체크박스를 없앴다)
                    print(f"[로봇 배치] 취소: {arm_label(ACTIVE_ARM[0])}")
                    h_robot_move_mode(False)
                self._send(200,'application/json',b'{"ok":true}')
            elif self.path.startswith('/envdrag/'):
                try:
                    from urllib.parse import unquote
                    rest=self.path[len('/envdrag/'):]
                    oid,xy=rest.rsplit('/',1)
                    xs,ys=xy.split(','); oid=unquote(oid)
                    # 드래그 시작 때 미리 만든 복제본이 있으면 그걸 확정
                    _fresh=DRAG_ENV.get('fresh_id')
                    if _fresh and _clone_base(_fresh)!=_clone_base(oid):
                        _fresh=None
                    DRAG_ENV['dropped']=True
                    _mv=bool(DRAG_ENV.get('move'))
                    if _clone_base(oid) in BRICKS:     # 물체(ppap 등) 드롭
                        ok,msg=brick_drop(_fresh or oid,float(xs),float(ys),
                                          move=_mv)
                        print(f"[물체 배치] {msg}")
                        # 실패(빈 도킹 없음)면 dragend 때 복제본이 정리되게
                        DRAG_ENV['dropped']=bool(ok)
                    else:
                        # 복제본(드래그 시작 때 생성)을 '마우스 멈춘 그 자리'에
                        # 그대로 두고 (스냅 없음, 판 밖이어도 OK),
                        # 항상 주황 핀 선택 단계로 넘어간다
                        new=oid if _mv else (_fresh or env_clone(oid) or oid)
                        ok,msg=move_env(new,float(xs),float(ys),snap=False)
                        if ok:
                            ok2,msg2=emove_start(new)  # 무조건 핀 선택
                            if msg2: print(f"[설비 배치] {msg2}")
                            if not ok2 and msg2 is None:
                                print(f"[설비 배치] {disp_name(new)}: 핀이 "
                                      f"2개 미만이라 그 자리에 둡니다")
                        elif msg: print(f"[설비 이동] {msg}")
                    DRAG_ENV['fresh_id']=None          # drop 이 소비했음
                    self._send(200,'application/json',
                               json.dumps({'ok':bool(ok)}).encode())
                except Exception as e:
                    self._send(400,'text/plain',str(e).encode())
            elif self.path.startswith('/envmove/'):
                try:
                    from urllib.parse import unquote
                    rest=self.path[len('/envmove/'):]
                    oid,xy=rest.rsplit('/',1)
                    xs,ys=xy.split(',')
                    ok,msg=move_env(unquote(oid),float(xs),float(ys))
                    print(f"[설비 이동] {msg}")
                    self._send(200,'application/json',
                               json.dumps({'ok':ok}).encode())
                except Exception as e:
                    self._send(400,'text/plain',str(e).encode())
            elif self.path.startswith('/envdel/'):      # 우클릭 삭제
                from urllib.parse import unquote
                _undo_snapshot('삭제')
                _ok,_m=env_delete(unquote(self.path[len('/envdel/'):]))
                print(f"[우클릭 삭제] {_m}")
                self._send(200,'application/json',
                           json.dumps({'ok':_ok,'msg':_m}).encode())
            elif self.path=='/undo':                    # Ctrl+Z
                _ok,_m=undo_last()
                self._send(200,'application/json',
                           json.dumps({'ok':_ok,'msg':_m}).encode())
            elif self.path.startswith('/armmv/'):       # 팔 우클릭 → 그 팔 위치 옮기기 (11-03)
                from urllib.parse import unquote
                _ok,_m=arm_move_start(unquote(self.path[len('/armmv/'):]))
                self._send(200,'application/json',
                           json.dumps({'ok':bool(_ok),'msg':_m}).encode())
            elif self.path.startswith('/envmv/'):       # 우클릭 이동(핀 재배치)
                from urllib.parse import unquote
                _ok,_m=emove_start(unquote(self.path[len('/envmv/'):]))
                if _m: print(f"[우클릭 이동] {_m}")
                self._send(200,'application/json',
                           json.dumps({'ok':bool(_ok)}).encode())
            elif self.path.startswith('/rpin/'):
                _i=int(self.path.rsplit('/',1)[1])
                (emove_click_pin if ENV_MOVE['on'] else rmove_click_pin)(_i)
                self._send(200,'application/json',b'{"ok":true}')
            elif self.path.startswith('/rpeg/'):
                try:
                    xs,ys=self.path.rsplit('/',1)[1].split(',')
                    ok=(emove_click_peg if ENV_MOVE['on']
                        else rmove_click_peg)(float(xs),float(ys))
                    self._send(200,'application/json',
                               json.dumps({'ok':bool(ok)}).encode())
                except Exception:
                    self._send(400,'text/plain',b'bad')
            elif self.path.startswith('/pedit/'):
                # 플레이트 더블클릭 -> 확장 모드 on/off
                PLATE_EDIT['on']=self.path.endswith('/1')
                if PLATE_EDIT['on']: ROBOT_MOVE['on']=False
                self._send(200,'application/json',b'{"ok":true}')
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
            elif self.path.startswith('/calpt/'):
                # 보정 지도 점 클릭 -> 그 자리 보정 자세 등록/열기 (11-04)
                try:
                    _x,_y=self.path[len('/calpt/'):].split(',')
                    _nm,_msg=calmap_register(float(_x),float(_y))
                    self._send(200,'application/json',
                               json.dumps({'ok':_nm is not None,'name':_nm,'msg':_msg},ensure_ascii=False).encode())
                except Exception as _e:
                    self._send(200,'application/json',json.dumps({'ok':False,'msg':str(_e)},ensure_ascii=False).encode())
            elif self.path.startswith('/click/'):
                # 뷰어에서 고스트 클릭 -> 플레이트 추가
                try:
                    ix,iy=self.path[len('/click/'):].split(',')
                    t=(int(ix),int(iy))
                    ok=add_plate(t)
                    if ok: PLATE_EDIT['on']=False       # 추가되면 알아서 닫힘
                    self._send(200,'application/json',
                               json.dumps({'ok':ok}).encode())
                except Exception:
                    self._send(400,'text/plain',b'bad')
            elif self.path.startswith('/chunk'):
                # 조종표 조각을 '이진' 으로 준다.
                # JSON 4.1MB 는 전송에 2.5초씩 걸려 조종이 버벅인다.
                # int16 키 + float32 값 = 같은 내용이 1/4 크기, 파싱도 즉시.
                from urllib.parse import urlparse, parse_qs
                qs=parse_qs(urlparse(self.path).query)
                def _i(nm,dv):
                    try: return int(qs.get(nm,[dv])[0])
                    except Exception: return dv
                g=_HEAD_GRID[0]
                if 'cx' in qs:
                    k=(_i('cx',0),_i('cy',0),_i('cz',0))
                else:
                    p0=tool_pos(CURRENT_Q)
                    k=tuple(int(round(float(p0[i])/g)) for i in range(3))
                r=max(1,min(25,_i('r',15)))
                code=_HEAD_ARR['code']
                if code is None:
                    self._send(200,'application/octet-stream',b''); return
                # 요청 상자 안의 칸들을 한 번에 골라낸다
                xs=np.arange(k[0]-r,k[0]+r+1,dtype=np.int64)
                ys=np.arange(k[1]-r,k[1]+r+1,dtype=np.int64)
                zs=np.arange(k[2]-r,k[2]+r+1,dtype=np.int64)
                KX,KY,KZ=np.meshgrid(xs,ys,zs,indexing='ij')
                want=(((KX.ravel()+_HK_OFF)*_HK_M+(KY.ravel()+_HK_OFF))*_HK_M
                      +(KZ.ravel()+_HK_OFF))
                idx=np.searchsorted(code,want)
                ok=(idx<len(code))
                idx2=np.where(ok,idx,0)
                ok&=(code[idx2]==want)
                sel=idx2[ok]
                keys=np.stack([KX.ravel()[ok],KY.ravel()[ok],KZ.ravel()[ok]],1)
                vals=_HEAD_ARR['val'][sel]
                head=np.array([len(sel),NJOINT,k[0],k[1],k[2],r],dtype=np.int32)
                buf=(head.tobytes()+np.float64(g).tobytes()
                     +keys.astype(np.int16).tobytes()+vals.astype(np.float32).tobytes())
                self._send(200,'application/octet-stream',buf)
            elif self.path=='/kin':
                # 뷰어가 자체 FK 로 즉시 그릴 수 있게 관절 정의를 준다
                kin={'joints':[{'xyz':list(map(float,BASE_DEF[jn]['xyz'])),
                                'rpy':list(map(float,BASE_DEF[jn]['rpy'])),
                                'axis':list(map(float,BASE_DEF[jn]['axis']))}
                               for jn in JOINTS],
                     'links':list(ROBOT_LINKS),
                     'yaw':float(BASE_YAW[0]),
                     'off':[float(v) for v in BASE_OFF]}
                self._send(200,'application/json',json.dumps(kin).encode())
            elif self.path.startswith('/mesh/'):
                from urllib.parse import unquote
                mid=unquote(self.path[len('/mesh/'):].split('?')[0])   # ?g=세대 떼기 (중계가 인코딩해 보낸다)
                if mid not in MESH_REG and '~' in mid: mid=mid.split('~',1)[1]   # 다른 팔 = 같은 메쉬
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

# ===== 조종표 미리 읽기 =====
# UR15 표는 263MB 라 읽는 데 15~26초 걸린다. 조이콘을 잡은 뒤에 읽으면
# 그동안 반응이 없다 -> 앱이 뜨는 순간부터 뒤에서 읽어둔다.
# (웹 화면은 기다리지 않고 바로 뜬다 — 별도 스레드)
try: _joy_table_load_bg()
except Exception: pass
# 닭 머리 표(있으면) 도 미리 불러둔다 — 조종이 즉시 반응한다
try:
    _n=head_load_any()
    if _n: print(f'닭 머리 표 {_n:,}칸 불러옴 — 조종 즉시 반응')
except Exception: pass

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
                                value=0.0,step=0.1,label=jname(j),
                                visible=not _dead_joint(j)))
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
                # 이 그림은 화면에 안 나온다. 로봇에 따라 그리다 실패해도
                # 앱 전체가 못 뜨는 일이 없도록 빈 그림으로 넘어간다.
                try:
                    _fig0=robot_fig([0.0]*NJOINT)
                except Exception as _e:
                    print(f'  (숨김 plotly 그림 생략: {_e})')
                    _fig0=None
                plot=gr.Plot(value=_fig0,label='plotly 3D')

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

try: _apply_level_trim()                     # 11-05: 팔별 그리퍼 수평 보정 (calib.json level_trim)
except Exception as _e: print('수평 보정 적용 실패:',_e)
if S.get('level_trim'): print('그리퍼 수평 보정:',', '.join(f"{k}팔 {float(v):+.1f}°" for k,v in S['level_trim'].items()))

# ===== 시작 배치 (11-05 사용자: 'esw_contest_drboom' 이 기본) =====
#   모든 함수가 정의된 뒤라야 불러오기(결합 정규화 · 자세 정리 등)가 끝까지 돈다.
#   ARM_START_LAYOUT 환경변수로 바꿀 수 있다 (빈 값 = 플레이트만).
try:
    _start=os.environ.get('ARM_START_LAYOUT','esw_contest_drboom')
    _sp=(S.get('factory_presets') or {}).get(_start) if _start else None
    if _sp:
        _factory_apply(_sp)
        with LOCK:
            S['robot_xy']=[float(BASE_OFF[0]),float(BASE_OFF[1])]; S['robot_yaw']=float(BASE_YAW[0])
        _macro_save()
        LAST_PRESET[0]=_start
        print(f"시작 배치: {_start}")
    elif _start:
        print(f"시작 배치 '{_start}' 없음 — 플레이트만")
except Exception as _e:
    print(f"시작 배치 불러오기 실패: {_e}")

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
