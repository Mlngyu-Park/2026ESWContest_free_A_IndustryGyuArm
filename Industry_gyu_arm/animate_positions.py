#!/usr/bin/env python3
"""
base -> 1 -> base -> 2 -> base -> 3 -> (반복) 이동 애니메이션 (RViz).
- base_position 로봇으로 joint_states 발행
- 각 position 관절각은 IK로 구한 값 (오차 0mm)
Ctrl+C 종료.
"""
import sys, time, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
JOINT_NAMES=["j1","j2","j3","j4","j5","j6"]

# IK로 구한 각 자세 관절각 (rad)
POSES={
    "base":[0.0,   0.0,    0.0,    0.0, 0.0,    0.0],
    "1st": [-0.26, -1.2827,-1.8599,0.0,-0.5773,-0.26],
    "2nd": [-0.0375,-1.5374,-2.0274,0.0,-0.49,  -0.0375],
    "3rd": [0.2612,-1.2827,-1.8599,0.0,-0.5773, 0.2612],
}
# 이동 순서
SEQ=["base","1st","base","2nd","base","3rd"]
DUR=2.5; HZ=30

rclpy.init(); node=Node("animate_positions")
pub=node.create_publisher(JointState,"joint_states",10)

def publish(q):
    m=JointState(); m.header.stamp=node.get_clock().now().to_msg()
    m.name=list(JOINT_NAMES); m.position=[float(x) for x in q]; pub.publish(m)

def ease(t): return t*t*(3-2*t)
def lerp(a,b,t): return [ai+(bi-ai)*ease(t) for ai,bi in zip(a,b)]

print("애니메이션: base->1->base->2->base->3 반복. Ctrl+C 종료.")
cur=POSES["base"]
try:
    while True:
        for name in SEQ:
            tgt=POSES[name]
            print(f"  → {name}")
            n=int(DUR*HZ)
            for k in range(n+1):
                publish(lerp(cur,tgt,k/n)); time.sleep(1.0/HZ)
            cur=tgt
            time.sleep(0.6)
except KeyboardInterrupt:
    print("\n종료"); node.destroy_node(); rclpy.shutdown()
