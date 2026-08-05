#!/usr/bin/env python3
"""
base -> 1-1 -> 1 -> 1-1 -> base 시퀀스 애니메이션 (RViz). 반복.
Ctrl+C 종료.
"""
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
JOINT_NAMES=["j1","j2","j3","j4","j5","j6"]

POSES={
    "base":[0.0,   0.0,    0.0,    0.0, 0.0,    0.0],
    "1-1": [-0.26, 0.5198, 1.4182, 0.0, 0.8984, -0.26],
    "1":   [-0.2599,0.3778,-0.3043,0.0,-0.682, -0.2599],
}
SEQ=["base","1-1","1","1-1","base"]
DUR=2.0; HZ=30

rclpy.init(); node=Node("animate_seq_1")
pub=node.create_publisher(JointState,"joint_states",10)

def publish(q):
    m=JointState(); m.header.stamp=node.get_clock().now().to_msg()
    m.name=list(JOINT_NAMES); m.position=[float(x) for x in q]; pub.publish(m)
def ease(t): return t*t*(3-2*t)
def lerp(a,b,t): return [ai+(bi-ai)*ease(t) for ai,bi in zip(a,b)]

print("base -> 1-1 -> 1 -> 1-1 -> base 반복. Ctrl+C 종료.")
cur=POSES["base"]
try:
    while True:
        for name in SEQ:
            tgt=POSES[name]; print(f"  -> {name}")
            n=int(DUR*HZ)
            for k in range(n+1): publish(lerp(cur,tgt,k/n)); time.sleep(1.0/HZ)
            cur=tgt; time.sleep(0.6)
except KeyboardInterrupt:
    print("\n종료"); node.destroy_node(); rclpy.shutdown()
