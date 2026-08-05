#!/usr/bin/env python3
"""
환경 물체(PPAP, docking_station) STL을 RViz Marker로 표시.
- SolidWorks 원점 배치 그대로 (base_link 프레임, 위치 0,0,0)
Ctrl+C 종료.
"""
import sys, time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

sys.stdout.reconfigure(line_buffering=True)

PKG_MESH="package://Industry_gyu_arm_20260719/meshes"
OBJECTS=[
    ("PPAP", f"{PKG_MESH}/PPAP.STL", (0.7,0.7,0.7)),
    ("docking_station", f"{PKG_MESH}/docking_station.STL", (0.4,0.6,0.9)),
]

rclpy.init(); node=Node("env_markers")
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
        m.pose.orientation.w=1.0     # 원점 배치 그대로
        m.scale.x=m.scale.y=m.scale.z=1.0
        m.color.r,m.color.g,m.color.b=rgb; m.color.a=1.0
        arr.markers.append(m)
    return arr

print("환경 마커 발행 중 (PPAP, docking_station). Ctrl+C 종료.")
try:
    while True:
        pub.publish(make()); time.sleep(0.5)
except KeyboardInterrupt:
    print("\n종료"); node.destroy_node(); rclpy.shutdown()
