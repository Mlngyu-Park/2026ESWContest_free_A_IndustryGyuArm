#!/usr/bin/env python3
"""
RViz에 URDF 0자세(모든 관절 = 0 rad) 표시.
- /joint_states 로 joint_1~5 = 0 을 계속 발행
- 모터 포트를 쓰지 않음 (ROS만) → teleop/다른 것과 포트 충돌 없음
목적: URDF 기본자세가 어떤 모양인지 RViz로 확인 → 실물을 그 모양에 맞추기.
Ctrl+C 종료.
"""
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

rclpy.init()
node = Node("zero_pose_pub")
pub = node.create_publisher(JointState, "joint_states", 10)
print("URDF 0자세 발행 중 (모든 관절 0 rad). RViz에서 기본자세 모양 확인. Ctrl+C 종료.")
try:
    while True:
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(msg)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n종료")
    node.destroy_node(); rclpy.shutdown()
