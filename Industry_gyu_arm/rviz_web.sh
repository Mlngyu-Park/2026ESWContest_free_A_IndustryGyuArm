#!/bin/bash
# 진짜 RViz를 웹(noVNC)으로 띄우는 스크립트.
# 사용: bash rviz_web.sh   (arm_studio_web.py가 iframe으로 접속)
# 종료: Ctrl+C (자식 프로세스 전부 정리)
set -e
DISP=:99
GEOM=1280x800
VNC_PORT=5900
WEB_PORT=6080

cleanup(){ kill $(jobs -p) 2>/dev/null; exit 0; }
trap cleanup INT TERM

# 1) 가상 디스플레이
Xvfb $DISP -screen 0 ${GEOM}x24 &
sleep 1

# 2) RViz (+ robot_state_publisher) - jog.launch.py 사용 (/joint_states는 웹UI가 발행)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
DISPLAY=$DISP ros2 launch Industry_gyu_arm_20260724_base_position jog.launch.py &
sleep 3

# 3) 환경 마커 (플레이트/도킹 STL)
DISPLAY=$DISP conda run --no-capture-output -n gyu_arm python \
  /home/drboom/2026_project/Industry_gyu_arm/env_markers_24base.py &

# 4) VNC 서버 + noVNC 웹 게이트웨이
x11vnc -display $DISP -nopw -forever -shared -rfbport $VNC_PORT -quiet &
websockify --web=/usr/share/novnc $WEB_PORT localhost:$VNC_PORT &

echo "=============================================="
echo " RViz web ready: http://localhost:$WEB_PORT/vnc.html?autoconnect=true&resize=scale"
echo " (arm_studio_web.py 페이지 안 iframe으로 표시됨)"
echo " Ctrl+C = 전부 종료"
echo "=============================================="
wait
