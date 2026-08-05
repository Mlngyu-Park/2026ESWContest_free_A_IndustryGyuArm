#!/usr/bin/env python3
"""
현재 실물 로봇 자세를 읽어 RViz로 표시 (읽기만, 안 움직임).
- 임시 영점(현재 raw)을 기준으로 joint_states 발행
- 실행하면 RViz 로봇 = 현재 실물 자세
동시에 현재 raw값을 출력 (영점 후보).
Ctrl+C 종료.
"""
import sys, time, math
from dynamixel_sdk import PortHandler, PacketHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_PRESENT=132
MOTORS={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5',5:'j6'}
JOINT_NAMES=["j1","j2","j3","j4","j5","j6"]

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)

# 시작 시 현재 raw = 임시 영점
zero={}
for i in MOTORS:
    p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT)
    zero[i]=s32(p) if r==0 else 0
print("현재 raw (영점 후보):")
for i in MOTORS: print(f"  {MOTORS[i]}(ID{i}): {zero[i]}")
print("\nRViz에 현재 자세 발행 중. Ctrl+C 종료.")

rclpy.init(); node=Node("show_current"); pub=node.create_publisher(JointState,"joint_states",10)
try:
    while True:
        msg=JointState(); msg.header.stamp=node.get_clock().now().to_msg()
        msg.name=list(JOINT_NAMES); pos=[]
        for i in MOTORS:
            p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT)
            raw=s32(p) if r==0 else zero[i]
            pos.append((raw-zero[i])*(math.pi/2048.0))
        msg.position=[float(x) for x in pos]; pub.publish(msg)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n종료"); node.destroy_node(); rclpy.shutdown(); port.closePort()
