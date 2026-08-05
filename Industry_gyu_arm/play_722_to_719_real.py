#!/usr/bin/env python3
"""
722 시작자세 → 719 목표자세 이동을 실제 로봇 + RViz 동시 실행.
- 팔 5축(j1~j5): rad(719영점 기준) → raw → 모터
- 그리퍼(모터 ID5): 열림/닫힘 raw 사이 보간
- 부드러운 가감속 보간 이동
- RViz용 joint_states 도 발행 (팔 5축 + 그리퍼 prismatic 근사)

실행: python play_722_to_719_real.py
  기본은 1회 이동(시작→목표). --loop 주면 왕복 반복.
Ctrl+C: 정지 후 토크 해제.
"""
import sys, time, math, signal
from dynamixel_sdk import PortHandler, PacketHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
LOOP = "--loop" in sys.argv

PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112; ADDR_GOAL=116; ADDR_PRESENT=132
PROFILE_VEL=30; PROFILE_ACCEL=10

# 팔 모터 (719 영점 기준)
ARM={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5'}
ZERO_RAW={0:2370,1:1320,2:998,3:1948,4:2575}
# 그리퍼 모터 ID5: 열림/닫힘 raw (measure 범위 375~3768 내). 방향 반대라 값 교체.
GRIP_ID=5
GRIP_OPEN_RAW=2500     # 열림
GRIP_CLOSE_RAW=3580    # 닫힘

# 자세 (팔 rad). 722 시작자세를 719 관절각으로 변환한 값.
ARM_START=[0.241, 0.197, -0.558, 0.182, 0.067]
ARM_GOAL =[0.0,   0.0,   0.0,    0.0,   0.0]

# RViz용
JOINT_NAMES=["joint_1","joint_2","joint_3","joint_4","joint_5","joint_left","joint_right"]
STEPS=60; STEP_DT=0.05    # 3초 이동

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
# 관절별 회전 방향 부호 (실물 모터 방향이 URDF와 반대면 -1)
DIR={0:1, 1:1, 2:-1, 3:1, 4:1}   # j3(ID2) 반대라 -1
def rad_to_raw(i,rad): return int(ZERO_RAW[i]+DIR[i]*rad*2048.0/math.pi)

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
rclpy.init(); node=Node("play_real"); pub=node.create_publisher(JointState,"joint_states",10)

def cleanup(*_):
    print("\n정지 + 토크 해제")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for i in list(ARM)+[GRIP_ID]:
        packet.write1ByteTxRx(port,i,ADDR_TORQUE,0)
    try: node.destroy_node(); rclpy.shutdown()
    except Exception: pass
    port.closePort(); sys.exit(0)
signal.signal(signal.SIGINT, cleanup)

# 속도/가감속/토크
for i in list(ARM)+[GRIP_ID]:
    packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,PROFILE_ACCEL)
    packet.write4ByteTxRx(port,i,ADDR_P_VEL,PROFILE_VEL)
    packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)

def ease(t): return t*t*(3-2*t)

def move(arm_a, grip_a, arm_b, grip_b):
    """arm_a(rad리스트)+grip_a(raw) -> arm_b+grip_b 로 보간 이동."""
    for k in range(STEPS+1):
        t=ease(k/STEPS)
        arm=[a+(b-a)*t for a,b in zip(arm_a,arm_b)]
        grip=int(grip_a+(grip_b-grip_a)*t)
        # 모터 전송
        for idx,i in enumerate(ARM):
            packet.write4ByteTxRx(port,i,ADDR_GOAL, rad_to_raw(i,arm[idx])&0xFFFFFFFF)
        packet.write4ByteTxRx(port,GRIP_ID,ADDR_GOAL, grip&0xFFFFFFFF)
        # RViz 발행 (그리퍼는 prismatic 근사: 0~0.027)
        gp=(grip-GRIP_CLOSE_RAW)/(GRIP_OPEN_RAW-GRIP_CLOSE_RAW)*0.027
        msg=JointState(); msg.header.stamp=node.get_clock().now().to_msg()
        msg.name=list(JOINT_NAMES); msg.position=[float(x) for x in arm]+[gp,-gp]
        pub.publish(msg)
        time.sleep(STEP_DT)

print("먼저 기본자세(719 영점)로 이동 후, 시작↔목표 반복")
# 1) 현재 위치에서 기본자세(719 영점, 그리퍼 닫힘)로 먼저 이동
#    현재 관절각을 읽어 거기서부터 보간
cur_arm=[]
for i in ARM:
    p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT)
    # raw->rad 도 같은 방향부호 적용 (rad_to_raw 역변환)
    cur_arm.append(DIR[i]*(s32(p)-ZERO_RAW[i])*math.pi/2048.0 if r==0 else 0.0)
pg,r,_=packet.read4ByteTxRx(port,GRIP_ID,ADDR_PRESENT)
cur_grip=s32(pg) if r==0 else GRIP_CLOSE_RAW
move(cur_arm, cur_grip, ARM_GOAL, GRIP_CLOSE_RAW)   # → 기본자세
time.sleep(0.8)

# 2) 시작(722)↔목표(719) 무한 반복
while True:
    print("  → 시작(722) + 그리퍼 열기")
    move(ARM_GOAL, GRIP_CLOSE_RAW, ARM_START, GRIP_OPEN_RAW)
    time.sleep(0.8)
    print("  → 목표(719) + 그리퍼 닫기")
    move(ARM_START, GRIP_OPEN_RAW, ARM_GOAL, GRIP_CLOSE_RAW)
    time.sleep(0.8)
