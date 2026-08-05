#!/bin/bash
# ARM STUDIO 실행 (이 파일만 실행하면 됨)
#   bash start.sh   또는   ./start.sh
# 종료: Ctrl+C
cd "$(dirname "$0")"
echo "ARM STUDIO 시작... 브라우저에서 http://localhost:7860 여세요"
exec conda run --no-capture-output -n gyu_arm python arm_studio_web.py
