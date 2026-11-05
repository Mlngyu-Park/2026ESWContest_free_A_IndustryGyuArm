#!/bin/bash
# 로봇 기준 목심 2개 고르기 (1회용 도구)
#   ./pick_pins.sh            -> 브라우저에서 주황 원 2개 클릭 -> 저장
# 저장되면 robot_pins_fixed.py 가 생기고, ARM STUDIO 재시작 시 그 값만 쓴다.
cd "$(dirname "$0")"
exec conda run --no-capture-output -n gyu_arm python pick_robot_pins.py "$@"
