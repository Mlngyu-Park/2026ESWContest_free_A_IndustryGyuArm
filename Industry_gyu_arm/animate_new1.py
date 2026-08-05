#!/usr/bin/env python3
"""
base -> 1(new 1st_position URDF에서 역산한 자세) -> base 반복 애니메이션 (RViz).
자세는 1st_position URDF 축/위치/메쉬 역산으로 구함 (오차 0 검증 완료).
Ctrl+C 종료.
"""
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)
JOINT_NAMES=["j1","j2","j3","j4","j5","j6"]

POSES={
    "base":[0.0,    0.0,    0.0,    0.0, 0.0,    0.0],
    # 1st_position 역산 후 jog로 교정한 1번 자세
    "1":   [-0.3161,0.3773,-0.3053,0.0,-0.6826, 0.0],
}
SEQ=["1","base"]
DUR=2.0; HZ=30

rclpy.init(); node=Node("animate_new1")
pub=node.create_publisher(JointState,"joint_states",10)

def publish(q):
    m=JointState(); m.header.stamp=node.get_clock().now().to_msg()
    m.name=list(JOINT_NAMES); m.position=[float(x) for x in q]; pub.publish(m)
def ease(t): return t*t*(3-2*t)
def lerp(a,b,t): return [ai+(bi-ai)*ease(t) for ai,bi in zip(a,b)]

def move_seq(cur,tgt):
    """조인트 순서대로(j1->j6) 하나씩 이동."""
    q=list(cur)
    for j in range(6):
        if abs(tgt[j]-q[j])<0.005:
            q[j]=tgt[j]; continue
        start=q[j]; n=int(DUR*HZ)
        for k in range(n+1):
            q[j]=start+(tgt[j]-start)*ease(k/n)
            publish(q); time.sleep(1.0/HZ)
        time.sleep(0.2)
    return q

print("base -> 1 -> base 반복 (조인트 순서대로). Ctrl+C 종료.")
cur=POSES["base"]; publish(cur); time.sleep(0.5)
try:
    while True:
        for name in SEQ:
            print(f"  -> {name}")
            cur=move_seq(cur,POSES[name]); time.sleep(0.6)
except KeyboardInterrupt:
    print("\n종료"); node.destroy_node(); rclpy.shutdown()
