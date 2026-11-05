#!/usr/bin/env bash
# 영상 속 카메라 위치를 RViz 에 띄운다:  ./cam_rviz.sh shots/영상.mp4
cd "$(dirname "$0")"
[ -n "$1" ] || { echo "사용: ./cam_rviz.sh shots/영상.mp4"; exit 1; }
source /opt/ros/humble/setup.bash
rviz2 -d cam.rviz >/dev/null 2>&1 &
RV=$!
trap 'kill $RV 2>/dev/null' EXIT
conda run --no-capture-output -n gyu_arm python -u cam_rviz.py "$1"
