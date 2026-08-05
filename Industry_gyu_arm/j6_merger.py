#!/usr/bin/env python3
"""
j6 를 joint_states 에 합쳐주는 노드.

joint_state_publisher_gui 는 j1~j5 만 발행(j6 는 limit 0~0 이라 슬라이더 없음).
이 노드가 /gripper_joint (Float64) 를 받아 j6 각도를 채워
/joint_states 로 다시 발행 -> RViz 의 그리퍼도 열고 닫힌다.

원본 GUI 토픽: /joint_states_raw  (launch 에서 remap)
출력:          /joint_states

실행(단독): python j6_merger.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

J6_NAME='j6'

class Merger(Node):
    def __init__(self):
        super().__init__('j6_merger')
        self.j6=0.0
        self.pub=self.create_publisher(JointState,'joint_states',10)
        self.create_subscription(JointState,'joint_states_raw',self.on_js,10)
        self.create_subscription(Float64,'gripper_joint',self.on_g,10)
    def on_g(self,msg):
        self.j6=float(msg.data)
    def on_js(self,msg):
        names=list(msg.name); pos=list(msg.position)
        if J6_NAME in names:
            pos[names.index(J6_NAME)]=self.j6
        else:
            names.append(J6_NAME); pos.append(self.j6)
        out=JointState()
        out.header=msg.header
        out.name=names; out.position=pos
        self.pub.publish(out)

def main():
    rclpy.init()
    n=Merger()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        try: rclpy.shutdown()
        except Exception: pass

if __name__=='__main__':
    main()
