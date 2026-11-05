#!/bin/bash
# 조종표 만들기 — 백그라운드 (창 닫아도 계속)
#   ./run_table_bg.sh            UR15 표 (기본)
#   ./run_table_bg.sh soarm      SO-ARM 표
#   ./run_table_bg.sh --fresh    처음부터 새로
#
# ★ 왜 로봇을 여기서 정하나: 예전엔 '앱에서 마지막으로 고른 로봇'을
#   따라갔다. 앱에서 SO-ARM 을 골라두고 표를 돌리면 UR15 가 아니라
#   SO-ARM 표가 만들어지는 사고가 났다 (2026-08-24 실제 발생).
cd "$(dirname "$0")"
ROBOT_URDF="$HOME/2026_project/urdf/ur15_20260823/urdf/ur15_20260823.urdf"
ARGS=""
for a in "$@"; do
  case "$a" in
    soarm) ROBOT_URDF="$HOME/2026_project/urdf/sts3215/Industry_gyu_arm_20260728_base/urdf/Industry_gyu_arm_20260728.urdf";;
    ur15)  ;;
    *) ARGS="$ARGS $a";;
  esac
done
export ARM_BASE_URDF="$ROBOT_URDF"
setsid nohup ./build_joy_table.sh $ARGS > /tmp/joytbl.log 2>&1 < /dev/null &
echo "조종표 생성 시작: $(basename $ROBOT_URDF)"
echo "  진행 상황:  tail -f /tmp/joytbl.log"
echo "  중단:       pkill -f build_joy_table   (5분마다 저장돼 있음)"
