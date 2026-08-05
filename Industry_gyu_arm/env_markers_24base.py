#!/usr/bin/env python3
"""20260724 환경 STL 마커 (base_link 프레임, 회전/오프셋 없이 원본좌표). Ctrl+C 종료."""
import sys, time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

sys.stdout.reconfigure(line_buffering=True)
MESH="package://Industry_gyu_arm_20260724_base_position/meshes"
OBJECTS=[
    ("plate1",  f"{MESH}/env_plate_1.STL",   (0.6,0.6,0.6)),
    ("plate2",  f"{MESH}/env_plate_2.STL",   (0.6,0.6,0.6)),
    ("plate3",  f"{MESH}/env_plate_3.STL",   (0.6,0.6,0.6)),
    ("dock1",   f"{MESH}/env_docking_1.STL", (0.4,0.6,0.9)),
    ("dock2",   f"{MESH}/env_docking_2.STL", (0.4,0.6,0.9)),
    ("dock3",   f"{MESH}/env_docking_3.STL", (0.4,0.6,0.9)),
]
rclpy.init(); node=Node("env_markers_24")
pub=node.create_publisher(MarkerArray,"environment_markers",10)

def make():
    arr=MarkerArray()
    for i,(name,mesh,rgb) in enumerate(OBJECTS):
        m=Marker()
        m.header.frame_id="base_link"
        m.header.stamp=node.get_clock().now().to_msg()
        m.ns=name; m.id=i
        m.type=Marker.MESH_RESOURCE
        m.mesh_resource=mesh
        m.action=Marker.ADD
        m.pose.orientation.w=1.0
        m.scale.x=m.scale.y=m.scale.z=0.001   # mm -> m
        m.color.r,m.color.g,m.color.b=rgb; m.color.a=1.0
        arr.markers.append(m)
    return arr

print("환경 마커 발행 (24일자). Ctrl+C 종료.")
try:
    while True:
        pub.publish(make()); time.sleep(0.5)
except KeyboardInterrupt:
    print("\n종료"); node.destroy_node(); rclpy.shutdown()
