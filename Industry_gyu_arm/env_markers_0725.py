#!/usr/bin/env python3
"""/home/drboom/2026_project/stl 의 환경 STL 마커 (base_link, mm->m 0.001). Ctrl+C 종료."""
import sys, os, glob, time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

sys.stdout.reconfigure(line_buffering=True)
STL_DIR='/home/drboom/2026_project/stl'

def color_for(name):
    n=name.lower()
    if 'docking' in n: return (0.4,0.6,0.9)
    if 'seed' in n:    return (0.5,0.8,0.5)
    return (0.6,0.6,0.6)   # plate 등

files=sorted(glob.glob(os.path.join(STL_DIR,'*.STL'))+glob.glob(os.path.join(STL_DIR,'*.stl')))

# RViz 파일로더는 경로에 공백이 있으면 실패 -> 공백 없는 심볼릭 링크 자동 생성
CLEAN_DIR=os.path.join(STL_DIR,'nospace')
os.makedirs(CLEAN_DIR,exist_ok=True)
clean_files=[]
for p in files:
    name=os.path.basename(p).replace('Industry_gyu_arm - ','').replace(' ','_')
    link=os.path.join(CLEAN_DIR,name)
    try:
        if os.path.islink(link) or os.path.exists(link): os.remove(link)
        os.symlink(p,link)
    except OSError: pass
    clean_files.append(link)
files=clean_files
print(f"{len(files)}개 STL 마커 발행:", [os.path.basename(f) for f in files])

rclpy.init(); node=Node("env_markers_0725")
pub=node.create_publisher(MarkerArray,"environment_markers",10)

def publish():
    arr=MarkerArray()
    for i,path in enumerate(files):
        m=Marker()
        m.header.frame_id="base_link"
        m.header.stamp=node.get_clock().now().to_msg()
        m.ns="env0725"; m.id=i
        m.type=Marker.MESH_RESOURCE
        m.mesh_resource="file://"+path
        m.action=Marker.ADD
        m.pose.orientation.w=1.0
        m.scale.x=m.scale.y=m.scale.z=0.001   # mm -> m
        r,g,b=color_for(os.path.basename(path))
        m.color.r,m.color.g,m.color.b=r,g,b; m.color.a=1.0
        arr.markers.append(m)
    pub.publish(arr)

try:
    while True:
        publish(); time.sleep(1.0)
except KeyboardInterrupt:
    node.destroy_node(); rclpy.shutdown()
