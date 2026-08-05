#!/usr/bin/env python3
"""
20260723 환경 물체(docking_station x3, plate_bottom x3, PPAP) STL을 RViz Marker로 표시.
- base_link 프레임에 원점 배치 (로봇과 같은 프레임 → static transform으로 같이 세워짐)
Ctrl+C 종료.
"""
import sys, time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

sys.stdout.reconfigure(line_buffering=True)
MESH="package://Industry_gyu_arm_20260723_base_position/meshes"
OBJECTS=[
    ("plate1",  f"{MESH}/env_plate_1.STL",   (0.6,0.6,0.6)),
    ("plate2",  f"{MESH}/env_plate_2.STL",   (0.6,0.6,0.6)),
    ("plate3",  f"{MESH}/env_plate_3.STL",   (0.6,0.6,0.6)),
    ("dock1",   f"{MESH}/env_docking_1.STL", (0.4,0.6,0.9)),
    ("dock2",   f"{MESH}/env_docking_2.STL", (0.4,0.6,0.9)),
    ("dock3",   f"{MESH}/env_docking_3.STL", (0.4,0.6,0.9)),
    ("ppap",    f"{MESH}/env_ppap.STL",      (0.9,0.7,0.3)),
]

rclpy.init(); node=Node("env_markers_723")
pub=node.create_publisher(MarkerArray,"environment_markers",10)

def make():
    arr=MarkerArray()
    for i,(name,mesh,rgb) in enumerate(OBJECTS):
        m=Marker()
        m.header.frame_id="world"    # 로봇과 같은 프레임 (같이 세워짐)
        m.header.stamp=node.get_clock().now().to_msg()
        m.ns=name; m.id=i
        m.type=Marker.MESH_RESOURCE
        m.mesh_resource=mesh
        m.action=Marker.ADD
        m.pose.orientation.w=1.0
        # 환경 STL은 mm 단위(raw) → m로 축소 (0.001배). 로봇 메쉬는 m라 맞춤.
        m.scale.x=m.scale.y=m.scale.z=0.001
        m.color.r,m.color.g,m.color.b=rgb; m.color.a=1.0
        arr.markers.append(m)
    return arr

print("환경 마커 발행 (docking x3, plate x3, PPAP). Ctrl+C 종료.")
try:
    while True:
        pub.publish(make()); time.sleep(0.5)
except KeyboardInterrupt:
    print("\n종료"); node.destroy_node(); rclpy.shutdown()
