#!/usr/bin/env python3
"""
안전 속도로 기본자세(영점)로 이동 + RViz로 자세 발행.
- 실제 로봇을 HOME_RAW(영점)로 이동
- 동시에 /joint_states 발행 → RViz가 같은 자세 표시
- 이동 후 실제 관절각을 계속 발행하며 대기 (RViz 비교용)
목적: RViz 형상 vs 실제 로봇 형상 비교 (영점에서 같아야 정답).
q 또는 Ctrl+C 종료 (토크 유지).
"""
import sys, time, signal, math
from dynamixel_sdk import PortHandler, PacketHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112
ADDR_GOAL=116; ADDR_PRESENT=132
PROFILE_VEL=30; PROFILE_ACCEL=10
POS_TOL=15; TIMEOUT=20.0

MOTORS={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5'}     # 팔 5축 (RViz용)
ZERO_RAW={0:2370,1:1320,2:998,3:1948,4:2575}  # 2026-07-20 기본자세 재설정
HOME_RAW=dict(ZERO_RAW)                           # 기본자세=영점
JOINT_NAMES=["joint_1","joint_2","joint_3","joint_4","joint_5"]

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def raw_to_rad(i,raw): return (raw-ZERO_RAW[i])*(math.pi/2048.0)

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)

rclpy.init(); node=Node("go_home_rviz"); pub=node.create_publisher(JointState,"joint_states",10)

def get_pos(i):
    p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT); return s32(p) if r==0 else None

def publish():
    msg=JointState(); msg.header.stamp=node.get_clock().now().to_msg()
    msg.name=list(JOINT_NAMES); pos=[]
    for i in sorted(MOTORS):
        p=get_pos(i); pos.append(raw_to_rad(i,p) if p is not None else 0.0)
    msg.position=[float(x) for x in pos]; pub.publish(msg)

# 속도/가감속/토크
for i in MOTORS:
    packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,PROFILE_ACCEL)
    packet.write4ByteTxRx(port,i,ADDR_P_VEL,PROFILE_VEL)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)
time.sleep(0.3)

print("기본자세로 이동 (j1→j5)...")
for i in MOTORS:
    goal=HOME_RAW[i]; packet.write4ByteTxRx(port,i,ADDR_GOAL,goal & 0xFFFFFFFF)
    start=time.time()
    while True:
        publish()
        p=get_pos(i)
        if p is not None and abs(p-goal)<=POS_TOL: break
        if time.time()-start>TIMEOUT: print(f"  {MOTORS[i]} 타임아웃"); break
        time.sleep(0.05)
    print(f"  {MOTORS[i]} 도착")

print("\n기본자세 완료. RViz와 비교하세요. (Ctrl+C 종료)")
try:
    while True:
        publish(); time.sleep(0.1)
except KeyboardInterrupt:
    print("\n종료")
    node.destroy_node(); rclpy.shutdown(); port.closePort()
