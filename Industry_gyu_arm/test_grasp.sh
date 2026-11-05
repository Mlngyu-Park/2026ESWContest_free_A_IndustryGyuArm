#!/bin/bash
# 펜 잡기 단독 검증 (본체와 분리)
#   ./test_grasp.sh            시뮬레이션
#   ./test_grasp.sh --real     실물 로봇
#   ./test_grasp.sh --release  잡은 뒤 놓기까지
cd "$(dirname "$0")"
exec conda run --no-capture-output -n gyu_arm python test_grasp.py "$@"
