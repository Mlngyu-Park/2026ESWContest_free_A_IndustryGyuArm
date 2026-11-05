#!/bin/bash
# ARM STUDIO 실행 (이 파일만 실행하면 됨)
#   bash start.sh   또는   ./start.sh
# 종료: Ctrl+C
cd "$(dirname "$0")"

# ── 이전에 떠 있던 서버 싹 정리 (포트 충돌 에러 방지) ──
pkill -f 'python.*arm_studio_web\.py' 2>/dev/null && echo "이전 서버 정리 중..."
for i in $(seq 1 10); do
    ss -tln 2>/dev/null | grep -qE ':7860\s' || break
    sleep 0.5
done

echo "ARM STUDIO 시작... 브라우저에서 http://localhost:7860 여세요"
exec conda run --no-capture-output -n gyu_arm python arm_studio_web.py
