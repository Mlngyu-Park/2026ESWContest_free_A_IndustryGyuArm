#!/usr/bin/env python3
"""
터미널 키보드로 각 조인트를 미세 조정하는 조그(jog) 컨트롤.
- RViz와 실물이 같이 움직임 (/joint_states 발행 + 실물 모터 전송)
- j4<->j5 실제 배선 교차 매핑 반영
- 's' 로 현재 자세를 base(영점)로 저장

조작:
  Up/Down    : 조인트 선택 이동 (편집모드가 아닐 때)
  Space      : 선택 조인트 편집모드 ON/OFF
  Up/Down    : (편집모드) 각도 증가/감소
  Left/Right : 스텝 크기 변경 (미세<->거칠게)
  s          : 현재 자세를 base(영점)로 저장
  0          : 선택 조인트를 0(rad)으로
  q / Ctrl+C : 종료 (토크 OFF)

안전: 안전속도 + 가감속, 측정 한계 클램프(각도로 환산), Ctrl+C = 즉시 토크 OFF.
"""
import sys, math, signal, termios, tty, select, threading
from dynamixel_sdk import PortHandler, PacketHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112; ADDR_GOAL=116; ADDR_PRESENT=132
SAFE_VEL=25; SAFE_ACCEL=8
SAVE_PATH='/home/drboom/2026_project/Industry_gyu_arm/base_captured.py'

JOINTS=['j1','j2','j3','j4','j5','j6']
# 저장된 자세 (관절각 rad, 조인트 순서 j1..j6)
POSES={
    "base":[0.0,    0.0,    0.0,    0.0, 0.0,    0.0],
    # 1st_position 역산 후 jog로 교정한 1번 자세
    "1":   [-0.3161,0.3773,-0.3053,0.0,-0.6826, 0.0],
}
POSE_SAVE_PATH='/home/drboom/2026_project/Industry_gyu_arm/poses_captured.py'
# 모터 ID 매핑. j4<->j5 실제 배선 교차: RViz j4 -> 실물 ID4, RViz j5 -> 실물 ID3
NAME2ID={'j1':0,'j2':1,'j3':2,'j4':4,'j5':3,'j6':5}
ZERO_RAW={0:1517,1:2483,2:1562,3:2244,4:2545,5:2018}
DIR={0:-1,1:1,2:1,3:1,4:1,5:1}   # j1 반전 (브리지 확인)
# 2026-07-24 재측정 (j1~j4). j5,j6는 미측정이라 기존값 유지.
LIMIT_RAW={0:(380,2571),1:(1479,3555),2:(1334,3382),3:(1271,3153),4:(1599,3524),5:(1970,3525)}

# 각도 스텝(rad) 후보: 좌우로 전환
STEPS=[math.radians(x) for x in (0.5, 1, 2, 5, 10)]
step_idx=1   # 기본 1도

# 현재 목표 관절각(rad)
angles={j:0.0 for j in JOINTS}
sel=0            # 선택된 조인트 index
editing=False    # 편집모드

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def rad_limit(name, rad):
    """각도(rad)를 raw 한계 내로 클램프한 rad 반환."""
    i=NAME2ID[name]
    raw=int(ZERO_RAW[i]+DIR[i]*rad*2048.0/math.pi)
    lo,hi=LIMIT_RAW[i]
    raw=max(lo,min(hi,raw))
    return DIR[i]*(raw-ZERO_RAW[i])*math.pi/2048.0
def rad_to_raw(name,rad):
    i=NAME2ID[name]
    raw=int(ZERO_RAW[i]+DIR[i]*rad*2048.0/math.pi)
    lo,hi=LIMIT_RAW[i]
    return max(lo,min(hi,raw))
def get(i):
    p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT); return s32(p) if r==0 else None

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)

_orig_term=None
def restore_term():
    if _orig_term is not None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, _orig_term)

_stop=False
def emergency_off(*_):
    global _stop
    _stop=True
    restore_term()
    print("\n[긴급정지] 토크 OFF")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for i in set(NAME2ID.values()): packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
    port.closePort()
    try: rclpy.shutdown()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT, emergency_off)

# 속도/가감속/토크
for i in set(NAME2ID.values()):
    packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,SAFE_ACCEL)
    packet.write4ByteTxRx(port,i,ADDR_P_VEL,SAFE_VEL)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)

def send_all():
    for name in JOINTS:
        packet.write4ByteTxRx(port,NAME2ID[name],ADDR_GOAL, rad_to_raw(name,angles[name]) & 0xFFFFFFFF)

def save_base():
    cur={}
    for name in JOINTS:
        i=NAME2ID[name]
        p=get(i); cur[i]= p if p is not None else ZERO_RAW[i]
    lines=["# jog_terminal.py 로 저장한 base 자세 (ZERO_RAW)","ZERO_RAW={"]
    for i in sorted(cur): lines.append(f"    {i}: {cur[i]},")
    lines.append("}")
    text="\n".join(lines)
    try:
        with open(SAVE_PATH,'w') as f: f.write(text+"\n")
    except Exception as e:
        return f"파일 저장 실패: {e}\n"+text
    return "[저장됨] base -> "+SAVE_PATH+"\n"+text

def save_pose(name):
    """현재 목표 관절각(angles, rad)을 POSES[name]에 저장 + 파일 기록."""
    POSES[name]=[angles[j] for j in JOINTS]
    lines=["# jog_terminal.py 로 저장한 자세 (관절각 rad, j1..j6)","POSES_SAVED={"]
    for nm in ("base","1-1","1"):
        if nm in POSES:
            vals=", ".join(f"{v:.4f}" for v in POSES[nm])
            lines.append(f'    "{nm}": [{vals}],')
    lines.append("}")
    text="\n".join(lines)
    try:
        with open(POSE_SAVE_PATH,'w') as f: f.write(text+"\n")
    except Exception as e:
        return f"파일 저장 실패: {e}"
    degs=", ".join(f"{math.degrees(v):+.1f}" for v in POSES[name])
    return f"[저장됨] '{name}' 자세 (deg: {degs}) -> {POSE_SAVE_PATH}"

msg_line=""
def render():
    # 화면 갱신 (덮어쓰기)
    lines=[]
    lines.append(f"스텝: {math.degrees(STEPS[step_idx]):.1f}deg   (Left/Right로 변경)")
    for k,name in enumerate(JOINTS):
        mark='>' if k==sel else ' '
        edit='[편집]' if (k==sel and editing) else '     '
        deg=math.degrees(angles[name])
        lines.append(f"{mark} {name} {edit} : {deg:+7.2f} deg")
    lines.append("Space=편집토글  Up/Down=선택 or 각도  Left/Right=스텝")
    lines.append("이동: b=base  1=1자세     저장: ]=1  s=base영점")
    lines.append("0=선택조인트영점  q=종료")
    if msg_line: lines.append(msg_line)
    # 커서 위로 올려 덮어쓰기
    sys.stdout.write("\033[2J\033[H")  # clear + home
    sys.stdout.write("\n".join(lines)+"\n")
    sys.stdout.flush()

# --- ROS publish ---
rclpy.init(); node=Node("jog_terminal")
pub=node.create_publisher(JointState, "joint_states", 10)
def publish():
    m=JointState()
    m.header.stamp=node.get_clock().now().to_msg()
    m.name=list(JOINTS)
    m.position=[angles[n] for n in JOINTS]
    pub.publish(m)

def ease(t): return t*t*(3-2*t)
def goto_pose(name):
    """저장된 자세로 부드럽게 이동 (실물+RViz). 이동 중 Ctrl+C 정지 가능."""
    global msg_line
    target=POSES[name]
    start=[angles[j] for j in JOINTS]
    tgt=[rad_limit(JOINTS[k], target[k]) for k in range(len(JOINTS))]
    N=40
    import time as _t
    for s in range(N+1):
        if _stop: return
        f=ease(s/N)
        for k,j in enumerate(JOINTS):
            angles[j]=start[k]+(tgt[k]-start[k])*f
        send_all(); publish()
        _t.sleep(0.04)
    msg_line=f"{name} 자세 도착"

def read_key():
    """방향키/문자 1개 읽기. 방향키는 'UP''DOWN''LEFT''RIGHT', 그 외 문자."""
    ch=sys.stdin.read(1)
    if ch=='\x1b':
        seq=sys.stdin.read(2)
        return {'[A':'UP','[B':'DOWN','[C':'RIGHT','[D':'LEFT'}.get(seq,'ESC')
    return ch

def key_loop():
    global sel, editing, step_idx, msg_line, _stop
    while not _stop:
        r,_,_=select.select([sys.stdin],[],[],0.1)
        if not r: continue
        k=read_key()
        if k=='q': emergency_off(); return
        elif k==' ':
            editing=not editing
            msg_line="편집모드 "+("ON" if editing else "OFF")
        elif k=='UP':
            if editing:
                name=JOINTS[sel]
                angles[name]=rad_limit(name, angles[name]+STEPS[step_idx])
                send_all(); publish()
            else:
                sel=(sel-1)%len(JOINTS)
        elif k=='DOWN':
            if editing:
                name=JOINTS[sel]
                angles[name]=rad_limit(name, angles[name]-STEPS[step_idx])
                send_all(); publish()
            else:
                sel=(sel+1)%len(JOINTS)
        elif k=='LEFT':
            step_idx=(step_idx-1)%len(STEPS)
        elif k=='RIGHT':
            step_idx=(step_idx+1)%len(STEPS)
        elif k=='0':
            name=JOINTS[sel]; angles[name]=rad_limit(name,0.0)
            send_all(); publish(); msg_line=f"{name} 0으로"
        elif k=='1':
            msg_line="-> 1 자세로 이동중..."; render(); goto_pose("1")
        elif k=='b':
            msg_line="-> base 자세로 이동중..."; render(); goto_pose("base")
        elif k=='[':
            msg_line=save_pose("1-1")
        elif k==']':
            msg_line=save_pose("1")
        elif k=='s':
            msg_line=save_base()
        render()

# 시작하면 먼저 base 자세로 이동 (angles가 0이므로 base=0으로 정렬됨)
print("먼저 base 자세로 이동합니다... (Ctrl+C = 즉시 정지)")
for k,j in enumerate(JOINTS): angles[j]=0.0
goto_pose("base")

_orig_term=termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)
render()

t=threading.Thread(target=key_loop, daemon=True)
t.start()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    emergency_off()
finally:
    restore_term()
