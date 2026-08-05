#!/usr/bin/env python3
"""
722 시작자세 → 719 목표자세(0) 이동을 RViz에 애니메이션으로 발행.
- 719 로봇(현재 URDF) 하나로, 관절각을 시작값 -> 0 으로 서서히 보간
- /joint_states 발행 → RViz가 부드럽게 이동
- 반복 재생 (시작→목표→시작...)
실물 제어와 동일한 관절각 기준. Ctrl+C 종료.
"""
import sys, time, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

# 팔 5축 + 그리퍼 2축(joint_left/right)
JOINT_NAMES=["joint_1","joint_2","joint_3","joint_4","joint_5","joint_left","joint_right"]
# 719 URDF 그리퍼 한계: joint_left 0~0.027(열림), joint_right 0~-0.027(대칭)
GRIP_OPEN =[ 0.027, -0.027]   # 열림
GRIP_CLOSE=[ 0.0,    0.0]     # 닫힘
# 시작자세: 팔은 722자세 + 그리퍼 열림
START=[0.241, 0.197, -0.558, 0.182, 0.067] + GRIP_OPEN
# 목표자세: 719 영점(팔 0) + 그리퍼 닫힘 (이동하며 집는 동작)
GOAL =[0.0,   0.0,   0.0,    0.0,   0.0]   + GRIP_CLOSE

DURATION=3.0     # 편도 이동 시간(초)
HZ=30

rclpy.init(); node=Node("animate_722_to_719")
pub=node.create_publisher(JointState,"joint_states",10)

def publish(q):
    m=JointState(); m.header.stamp=node.get_clock().now().to_msg()
    m.name=list(JOINT_NAMES); m.position=[float(x) for x in q]
    pub.publish(m)

def lerp(a,b,t):
    # 부드러운 보간 (ease in-out)
    t=t*t*(3-2*t)
    return [ai+(bi-ai)*t for ai,bi in zip(a,b)]

print("애니메이션: 722 시작자세 → 719 목표자세(0) 반복. Ctrl+C 종료.")
try:
    while True:
        # 시작 → 목표
        print("  → 목표(719)로 이동")
        n=int(DURATION*HZ)
        for k in range(n+1):
            publish(lerp(START,GOAL,k/n)); time.sleep(1.0/HZ)
        time.sleep(1.0)
        # 목표 → 시작 (반복용)
        print("  → 시작(722)으로 복귀")
        for k in range(n+1):
            publish(lerp(GOAL,START,k/n)); time.sleep(1.0/HZ)
        time.sleep(1.0)
except KeyboardInterrupt:
    print("\n종료")
    node.destroy_node(); rclpy.shutdown()
