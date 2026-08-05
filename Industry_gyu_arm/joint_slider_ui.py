#!/usr/bin/env python3
"""
각 조인트 각도 슬라이더 GUI (실제 로봇 제어 + RViz 발행).
- j1~j6 슬라이더로 각도(rad) 직접 조절 → 실제 모터 이동
- RViz용 /joint_states 도 발행 (RViz 켜두면 같이 움직임)
- 시작 시 현재 자세로 슬라이더 초기화
- [기본자세] 버튼, [토크 ON/OFF] 버튼
필요: tkinter (기본), rclpy
"""
import sys, math, threading, time
import tkinter as tk
from dynamixel_sdk import PortHandler, PacketHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112
ADDR_GOAL=116; ADDR_PRESENT=132
PROFILE_VEL=40; PROFILE_ACCEL=15

MOTORS={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5',5:'j6'}
ZERO_RAW={0:2370,1:1320,2:998,3:1948,4:2575,5:538}   # j1~5 최신 영점 + j6 그리퍼
# 관절 rad 한계 (슬라이더 범위) = 측정한 실제 가동범위 (최신 영점 기준).
# 안전 여유로 약간 안쪽으로 좁힘.
MARGIN=0.05
_RAW_LIMITS={0:(1300,3606),1:(421,2806),2:(692,3104),3:(1215,3398),4:(1522,3583),5:(375,3768)}
LIMITS_RAD={}
for _i,(_mn,_mx) in _RAW_LIMITS.items():
    _lo=(_mn-ZERO_RAW[_i])*math.pi/2048 + MARGIN
    _hi=(_mx-ZERO_RAW[_i])*math.pi/2048 - MARGIN
    LIMITS_RAD[_i]=(_lo,_hi)

# 속도 제한 범위 (Profile Velocity 단위 0.229rpm). 슬라이더로 조절.
VEL_MIN=5; VEL_MAX=80; VEL_DEFAULT=40
# RViz용 조인트 이름 (팔 5축만; 그리퍼는 RViz prismatic이라 생략)
RVIZ_JOINTS=["joint_1","joint_2","joint_3","joint_4","joint_5"]

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def raw_to_rad(i,raw): return (raw-ZERO_RAW[i])*(math.pi/2048.0)
def rad_to_raw(i,rad): return int(ZERO_RAW[i]+rad*2048.0/math.pi)

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort():
    print("포트 실패"); sys.exit(1)
port.setBaudRate(BAUD)

rclpy.init(); node=Node("joint_slider_ui")
pub=node.create_publisher(JointState,"joint_states",10)

def get_pos(i):
    p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT); return s32(p) if r==0 else None

# 초기화: 속도/가감속/토크
for i in MOTORS:
    packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,PROFILE_ACCEL)
    packet.write4ByteTxRx(port,i,ADDR_P_VEL,PROFILE_VEL)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)

torque_on=[True]

# ---- 녹화/재생 상태 ----
import json
REC_HZ=20
recording=[False]      # 녹화 중 플래그
rec_frames=[]          # 녹화 버퍼
playing=[False]        # 재생 중 플래그
MOTION_FILE="motion_ui.json"

# ---- GUI ----
root=tk.Tk(); root.title("Joint Angle Control")
sliders={}
val_labels={}

programmatic=[False]   # True면 슬라이더 변경이 프로그램에 의한 것 → 모터 전송 안 함

def on_slide(i, v):
    rad=float(v)
    if not programmatic[0]:
        packet.write4ByteTxRx(port,i,ADDR_GOAL, rad_to_raw(i,rad)&0xFFFFFFFF)
    val_labels[i].config(text=f"{rad:+.2f} rad ({math.degrees(rad):+.0f}deg)")

for i,name in MOTORS.items():
    frame=tk.Frame(root); frame.pack(fill="x", padx=8, pady=3)
    tk.Label(frame, text=name, width=4, font=("",11,"bold")).pack(side="left")
    lo,hi=LIMITS_RAD[i]
    cur=get_pos(i)
    cur_rad=raw_to_rad(i,cur) if cur is not None else 0.0
    cur_rad=max(lo,min(hi,cur_rad))
    s=tk.Scale(frame, from_=lo, to=hi, resolution=0.01, orient="horizontal",
               length=320, showvalue=False,
               command=lambda v,i=i: on_slide(i,v))
    s.set(cur_rad); s.pack(side="left", padx=6)
    sliders[i]=s
    lbl=tk.Label(frame, text="", width=16); lbl.pack(side="left")
    val_labels[i]=lbl
    val_labels[i].config(text=f"{cur_rad:+.2f} rad ({math.degrees(cur_rad):+.0f}deg)")

def go_home():
    for i in MOTORS:
        sliders[i].set(0.0)      # command 콜백이 모터로 전송

def toggle_torque():
    torque_on[0]=not torque_on[0]
    for i in MOTORS:
        packet.write1ByteTxRx(port,i,ADDR_TORQUE,1 if torque_on[0] else 0)
    if torque_on[0]:
        tbtn.config(text="Torque ON (slider)", bg="#cfc")
        # 켜는 순간 실제 위치로 슬라이더 맞춤 (손으로 옮긴 위치 이어받기, 홱 튐 방지)
        sync_sliders_to_robot()
    else:
        tbtn.config(text="Torque OFF (hand)", bg="#fcc")

def sync_sliders_to_robot():
    """실제 모터 위치를 읽어 슬라이더에 반영 (모터 전송 없이)."""
    programmatic[0]=True   # 이 동안 on_slide는 모터로 안 보냄
    try:
        for i in MOTORS:
            p=get_pos(i)
            if p is not None:
                lo,hi=LIMITS_RAD[i]
                rad=max(lo,min(hi,raw_to_rad(i,p)))
                sliders[i].set(rad)
                val_labels[i].config(text=f"{rad:+.2f} rad ({math.degrees(rad):+.0f}deg)")
    finally:
        programmatic[0]=False

# ---- 속도 제한 슬라이더 ----
velf=tk.Frame(root); velf.pack(fill="x", padx=8, pady=6)
tk.Label(velf, text="Speed", width=4, font=("",11,"bold")).pack(side="left")
vel_label=tk.Label(velf, text="", width=16)

def on_vel(v):
    spd=int(float(v))
    for i in MOTORS:
        packet.write4ByteTxRx(port,i,ADDR_P_VEL,spd)
    vel_label.config(text=f"{spd}  ({spd*0.229:.1f}rpm)")

vel_slider=tk.Scale(velf, from_=VEL_MIN, to=VEL_MAX, resolution=1, orient="horizontal",
                    length=320, showvalue=False, command=on_vel)
vel_slider.set(VEL_DEFAULT); vel_slider.pack(side="left", padx=6)
vel_label.pack(side="left")
vel_label.config(text=f"{VEL_DEFAULT}  ({VEL_DEFAULT*0.229:.1f}rpm)")

btnf=tk.Frame(root); btnf.pack(pady=6)
tk.Button(btnf, text="Home (0)", command=go_home).pack(side="left", padx=5)
tbtn=tk.Button(btnf, text="Torque ON (slider)", bg="#cfc", command=toggle_torque)
tbtn.pack(side="left", padx=5)

# ---- 녹화 / 재생 ----
def _record_loop():
    t0=time.time()
    while recording[0]:
        pos=[get_pos(i) for i in MOTORS]
        if all(p is not None for p in pos):
            rec_frames.append({"t":round(time.time()-t0,3),"pos":pos})
        time.sleep(1.0/REC_HZ)

def toggle_record():
    if playing[0]:
        return
    if not recording[0]:
        # 녹화 시작: 토크는 계속 ON. 슬라이더로 움직이는 걸 기록. 버퍼 비움
        rec_frames.clear()
        recording[0]=True
        recbtn.config(text="[] Stop Rec", bg="#f88")
        status.config(text="Recording... move sliders")
        threading.Thread(target=_record_loop, daemon=True).start()
    else:
        # 녹화 종료 + 저장
        recording[0]=False
        recbtn.config(text="O Record", bg="#d9d9d9")   # 리눅스 기본 버튼색
        with open(MOTION_FILE,"w") as f:
            json.dump({"motors":list(MOTORS),"frames":rec_frames}, f)
        dur=rec_frames[-1]["t"] if rec_frames else 0
        status.config(text=f"Recorded: {len(rec_frames)} frames, {dur:.1f}s -> {MOTION_FILE}")

def _play_loop():
    try:
        with open(MOTION_FILE) as f:
            frames=json.load(f)["frames"]
    except FileNotFoundError:
        status.config(text="No recording file. Record first."); playing[0]=False; return
    if not frames:
        status.config(text="Empty motion"); playing[0]=False; return

    # 토크 ON + 속도/가감속
    for i in MOTORS:
        packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,PROFILE_ACCEL)
        packet.write4ByteTxRx(port,i,ADDR_P_VEL,int(vel_slider.get()))
        packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)

    # ★ 현재 자세와 무관하게, 녹화 시작(첫 프레임=기본자세)으로 먼저 이동
    status.config(text="Moving to recorded start pose...")
    first=frames[0]["pos"]
    for idx,i in enumerate(MOTORS):
        packet.write4ByteTxRx(port,i,ADDR_GOAL, first[idx]&0xFFFFFFFF)
    time.sleep(2.5)   # 도착 대기

    status.config(text="Playing...")
    t0=time.time()
    for fr in frames:
        if not playing[0]: break
        dt=fr["t"]-(time.time()-t0)
        if dt>0: time.sleep(dt)
        for idx,i in enumerate(MOTORS):
            packet.write4ByteTxRx(port,i,ADDR_GOAL, fr["pos"][idx]&0xFFFFFFFF)
    playing[0]=False
    playbtn.config(text="> Play")
    status.config(text="Play done")

def toggle_play():
    if recording[0]:
        return
    if not playing[0]:
        playing[0]=True
        playbtn.config(text="[] Stop")
        threading.Thread(target=_play_loop, daemon=True).start()
    else:
        playing[0]=False
        playbtn.config(text="> Play")

rpf=tk.Frame(root); rpf.pack(pady=6)
recbtn=tk.Button(rpf, text="O Record", width=10, command=toggle_record); recbtn.pack(side="left", padx=5)
playbtn=tk.Button(rpf, text="> Play", width=10, command=toggle_play); playbtn.pack(side="left", padx=5)

status=tk.Label(root, text="Ready", fg="blue"); status.pack(pady=3)

# RViz 발행 + 토크 OFF 시 슬라이더 자동 추종 (주기적)
def ros_publish():
    # 토크 OFF면 손으로 움직인 실제 위치를 슬라이더에 반영 (녹화/RViz에 손동작 기록)
    if not torque_on[0]:
        sync_sliders_to_robot()
    msg=JointState(); msg.header.stamp=node.get_clock().now().to_msg()
    msg.name=list(RVIZ_JOINTS)
    msg.position=[float(sliders[i].get()) for i in range(5)]
    pub.publish(msg)
    root.after(100, ros_publish)   # 10Hz
ros_publish()

def on_close():
    root.destroy()
try:
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
except KeyboardInterrupt:
    pass
finally:
    # 종료 정리 (중복 호출/에러 무시)
    for _fn in (node.destroy_node,
                lambda: rclpy.shutdown() if rclpy.ok() else None,
                port.closePort):
        try:
            _fn()
        except Exception:
            pass
