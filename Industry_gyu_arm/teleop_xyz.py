#!/usr/bin/env python3
"""
키보드로 엔드이펙터 XYZ 이동 (5자유도 IK).
  a/d = X축 -/+
  w/s = Y축 +/-
  방향키 위/아래 = Z축 +/-
  space = 현재 위치 정지(재동기화)
  q = 종료 (토크 유지)

매 입력마다 목표 XYZ를 STEP 만큼 이동 → IK → 관절각 → 모터 전송.
안전 속도 + 가감속 적용. Ctrl+C 로도 종료.
"""
import sys, tty, termios, time, math, select
import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler
import ik_lib as K

# ROS2 joint_states 발행 (RViz에서 실물 자세 보기)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# URDF 조인트 이름 (팔 5축). RViz robot_description 과 일치해야 함.
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112
ADDR_GOAL=116; ADDR_PRESENT=132
PROFILE_VEL=30; PROFILE_ACCEL=10
STEP=0.015   # 한 번에 이동할 거리 (m) = 15mm (기존 5mm의 3배)

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v

def read_arm_q():
    q=[]
    for i in sorted(K.MOTORS):
        p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT)
        q.append(K.raw_to_rad(i, s32(p)) if r==0 else 0.0)
    return q

def send_arm_q(q5):
    for idx,i in enumerate(sorted(K.MOTORS)):
        lo,hi=K.LIMITS_RAD[i]
        val=max(lo,min(hi,q5[idx]))          # 한계 클램프
        packet.write4ByteTxRx(port,i,ADDR_GOAL, K.rad_to_raw(i,val)&0xFFFFFFFF)

# ROS2 초기화 (joint_states 발행)
rclpy.init()
ros_node = Node("teleop_joint_state_pub")
js_pub = ros_node.create_publisher(JointState, "joint_states", 10)

def publish_joint_states(q5):
    msg = JointState()
    msg.header.stamp = ros_node.get_clock().now().to_msg()
    msg.name = list(JOINT_NAMES)
    msg.position = [float(x) for x in q5]
    js_pub.publish(msg)

# 초기화: 속도/가감속/토크
for i in K.MOTORS:
    packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,PROFILE_ACCEL)
    packet.write4ByteTxRx(port,i,ADDR_P_VEL,PROFILE_VEL)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)

# 조작 전에 먼저 기본자세(영점=모든 관절 0 rad)로 이동.
# 이러면 RViz 0자세 = 실물 자세로 정렬된 상태에서 시작.
print("기본자세(영점)로 이동 중...")
for i in sorted(K.MOTORS):
    goal = K.rad_to_raw(i, 0.0)                      # 0 rad = ZERO_RAW
    packet.write4ByteTxRx(port,i,ADDR_GOAL, goal & 0xFFFFFFFF)
t0=time.time()
while time.time()-t0 < 6.0:                          # 도착 대기(최대 6초) + RViz 발행
    publish_joint_states(read_arm_q())
    rclpy.spin_once(ros_node, timeout_sec=0)
    done=all(abs(K.rad_to_raw(i,0.0)-(s32(packet.read4ByteTxRx(port,i,ADDR_PRESENT)[0]))) < 15
             for i in sorted(K.MOTORS))
    if done: break
    time.sleep(0.1)
print("기본자세 도착.")

q=read_arm_q()
target=K.fk(q).copy()
print(f"시작 엔드이펙터 XYZ = ({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})")
print("조작: a/d=X  w/s=Y  9/0=Z  space=정지  q=종료")

def getch(timeout=0.03):
    """짧은 타임아웃. 꾹 누르면 OS 키리피트로 같은 문자가 연타로 들어와 연속 이동됨.
    쌓인 입력은 마지막 것만 남기고 비워서 지연 없이 반응."""
    fd=sys.stdin.fileno(); old=termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r,_,_=select.select([sys.stdin],[],[],timeout)
        if not r: return None
        c=sys.stdin.read(1)
        # ESC 시퀀스(방향키 등)는 무시하고 버림
        if c=='\x1b':
            while select.select([sys.stdin],[],[],0.005)[0]:
                sys.stdin.read(1)
            return None
        # 버퍼에 쌓인 같은 반복입력 흡수 (최신 것만 반영, 지연 방지)
        last=c
        while select.select([sys.stdin],[],[],0)[0]:
            nc=sys.stdin.read(1)
            if nc!='\x1b': last=nc
        return last
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

idle_ticks = 0
try:
    while True:
        # 1) 키 입력 먼저 (통신에 안 막히게)
        k=getch()

        if k is None:
            # 키 없을 때만 가끔 RViz 발행 (명령각 q 기준, 모터 안 읽어 빠름)
            idle_ticks += 1
            if idle_ticks >= 5:
                publish_joint_states(q)
                rclpy.spin_once(ros_node, timeout_sec=0)
                idle_ticks = 0
            continue

        moved=True
        # a/d 는 RViz 방향과 맞게 부호 반대로 (a=+X, d=-X)
        if   k=='a': target[0]+=STEP
        elif k=='d': target[0]-=STEP
        elif k=='w': target[1]+=STEP
        elif k=='s': target[1]-=STEP
        elif k=='9': target[2]+=STEP   # 9키 = Z +
        elif k=='0': target[2]-=STEP   # 0키 = Z -
        elif k==' ':
            q=read_arm_q(); target=K.fk(q).copy(); moved=False
            print(f"\r[재동기] XYZ=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})    ")
        elif k in ('q','\x03'):
            print("\n종료"); break
        else:
            moved=False
        if moved:
            sol=K.ik(target, q)            # IK
            reached=K.fk(sol)
            err=np.linalg.norm(reached-target)*1000
            if err<30:                     # 30mm 이내면 도달 가능으로 보고 전송
                send_arm_q(sol); q=sol
                publish_joint_states(sol)  # RViz에 명령각 즉시 반영 (모터 안 읽음)
                rclpy.spin_once(ros_node, timeout_sec=0)
                print(f"\rXYZ target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) err={err:.0f}mm    ", end="")
            else:
                # 도달 불가 → 목표 되돌림 (한계 밖)
                if   k=='a': target[0]-=STEP
                elif k=='d': target[0]+=STEP
                elif k=='w': target[1]-=STEP
                elif k=='s': target[1]+=STEP
                elif k=='9': target[2]-=STEP
                elif k=='0': target[2]+=STEP
                print(f"\r[도달불가] 그 방향 한계. err={err:.0f}mm            ", end="")
except KeyboardInterrupt:
    print("\n중단")
finally:
    # 종료 시 토크 해제. 해제 도중 Ctrl+C 무시로 확실히 완료.
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    print("토크 해제 중...")
    for i in sorted(K.MOTORS, reverse=True):
        for _ in range(3):                       # 통신 실패 대비 재시도
            res, _ = packet.write1ByteTxRx(port, i, ADDR_TORQUE, 0)  # write는 (result, error) 2개
            if res == 0:
                break
    print("토크 해제 완료")
    port.closePort()
    try:
        ros_node.destroy_node(); rclpy.shutdown()
    except Exception:
        pass
