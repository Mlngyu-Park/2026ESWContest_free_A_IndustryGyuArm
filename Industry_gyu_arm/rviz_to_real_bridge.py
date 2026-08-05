#!/usr/bin/env python3
"""
RViz 슬라이더 -> 실제 로봇 브리지 (방향 확인용).
- /joint_states 구독 (RViz joint_state_publisher_gui 슬라이더 값)
- 그 관절각(rad)을 실물 모터로 전송 -> RViz와 실물이 같이 움직임
- 슬라이더 움직였을 때 실물이 반대로 가면 그 관절 DIR을 뒤집으면 됨

안전: 안전속도 + 가감속, 측정 한계 클램프, Ctrl+C = 즉시 토크 OFF.
"""
import sys, math, signal
from dynamixel_sdk import PortHandler, PacketHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112; ADDR_GOAL=116
SAFE_VEL=25; SAFE_ACCEL=8

MOTORS={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5',5:'j6'}
# j4<->j5 실제 배선 교차: RViz j4 -> 실물 ID4, RViz j5 -> 실물 ID3
NAME2ID={'j1':0,'j2':1,'j3':2,'j4':4,'j5':3,'j6':5}
ZERO_RAW={0:1517,1:2483,2:1562,3:2244,4:2545,5:2018}
# 방향부호: 실물이 RViz와 반대면 -1 로. 확인 후 여기 수정.
DIR={0:-1,1:1,2:1,3:1,4:1,5:1}   # j1 반전, j2 정방향
LIMIT_RAW={0:(380,2571),1:(1479,3555),2:(1334,3382),3:(1271,3153),4:(1599,3524),5:(1970,3525)}

def rad_to_raw(i,rad):
    raw=int(ZERO_RAW[i]+DIR[i]*rad*2048.0/math.pi)
    lo,hi=LIMIT_RAW[i]
    return max(lo,min(hi,raw))

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)

def emergency_off(*_):
    print("\n[긴급정지] 토크 OFF")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for i in MOTORS: packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
    port.closePort()
    try: rclpy.shutdown()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT, emergency_off)

# 속도/가감속/토크
for i in MOTORS:
    packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,SAFE_ACCEL)
    packet.write4ByteTxRx(port,i,ADDR_P_VEL,SAFE_VEL)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)
print("브리지 시작: RViz 슬라이더 -> 실물 모터. Ctrl+C = 토크 OFF")
print("슬라이더를 움직여 실물과 RViz 방향 비교. 반대면 그 관절 알려주세요.")

rclpy.init(); node=Node("rviz_to_real")

def cb(msg):
    for name, pos in zip(msg.name, msg.position):
        if name in NAME2ID:
            i=NAME2ID[name]
            packet.write4ByteTxRx(port,i,ADDR_GOAL, rad_to_raw(i,pos) & 0xFFFFFFFF)

node.create_subscription(JointState,"joint_states",cb,10)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    emergency_off()
