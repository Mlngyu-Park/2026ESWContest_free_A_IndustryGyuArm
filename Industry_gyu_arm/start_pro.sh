#!/bin/bash
# ARM STUDIO PRO (새 디자인 UI) 실행
#   bash start_pro.sh   또는   ./start_pro.sh
# 종료: Ctrl+C   ·   기존 UI 는 start.sh 그대로 사용 가능
cd "$(dirname "$0")"
echo "ARM STUDIO PRO 시작... 브라우저에서 http://localhost:7900 여세요"
exec conda run --no-capture-output -n gyu_arm python arm_studio_pro.py
