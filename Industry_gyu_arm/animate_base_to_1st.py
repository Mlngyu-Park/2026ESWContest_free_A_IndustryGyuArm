#!/usr/bin/env python3
"""
base -> 1st position 이동 애니메이션 (RViz).
- 20260724 로봇으로 joint_states 발행
- base(모두 0) <-> 1st 왕복 반복
Ctrl+C 종료.
"""
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
JOINT_NAMES=["j1","j2","j3","j4","j5","j6"]

BASE=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
POS1=[-0.2599, 0.3778, -0.3043, 0.0, -0.682, -0.2599]

DUR=2.5; HZ=30

rclpy.init(); node=Node("animate_base_1st")
pub=node.create_publisher(JointState,"joint_states",10)

def publish(q):
    m=JointState(); m.header.stamp=node.get_clock().now().to_msg()
    m.name=list(JOINT_NAMES); m.position=[float(x) for x in q]; pub.publish(m)

def ease(t): return t*t*(3-2*t)
def lerp(a,b,t): return [ai+(bi-ai)*ease(t) for ai,bi in zip(a,b)]

print("base <-> 1st 왕복 반복. Ctrl+C 종료.")
try:
    while True:
        print("  -> 1st")
        n=int(DUR*HZ)
        for k in range(n+1): publish(lerp(BASE,POS1,k/n)); time.sleep(1.0/HZ)
        time.sleep(0.8)
        print("  -> base")
        for k in range(n+1): publish(lerp(POS1,BASE,k/n)); time.sleep(1.0/HZ)
        time.sleep(0.8)
except KeyboardInterrupt:
    print("\n종료"); node.destroy_node(); rclpy.shutdown()
