#!/bin/bash
# 물컵 규칙(그리퍼 수평) 미세조정 — 터미널 조그
#   ./level_jog.sh          기본: base 자세
#   ./level_jog.sh 픽업위치  특정 자세
cd "$(dirname "$0")"
exec conda run --no-capture-output -n gyu_arm python level_jog.py "$@"
