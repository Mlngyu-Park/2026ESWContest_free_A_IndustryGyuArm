#!/bin/bash
# ARM STUDIO PRO (새 디자인 UI) 실행
#   bash start_pro.sh   또는   ./start_pro.sh
# 종료: Ctrl+C   ·   기존 UI 는 start.sh 그대로 사용 가능
cd "$(dirname "$0")"

# ── 이전에 떠 있던 서버 싹 정리 (포트 충돌 에러 방지) ──
pkill -f 'python.*arm_studio_pro\.py' 2>/dev/null && echo "이전 서버 정리 중..."
# 포트(7900 웹 / 7870 뷰어)가 완전히 풀릴 때까지 대기 (최대 5초)
for i in $(seq 1 10); do
    ss -tln 2>/dev/null | grep -qE ':(7900|7870)\s' || break
    sleep 0.5
done

bash spark_link.sh || true      # DGX Spark 두뇌 연결 (안 되면 자동 대체)

# ── 로그를 파일에도 남긴다 (터미널 출력은 그대로) ──
#    화면에 보이는 것과 똑같은 내용이 여기 쌓인다 -> 나중에 원인 추적 가능
LOG=/tmp/arm_studio.log
: > "$LOG"                       # 새로 시작할 때마다 비움
ln -sf "$LOG" ./arm_studio.log 2>/dev/null || true
echo "로그 파일: $LOG  (tail -f $LOG 로 실시간 보기)"

echo "ARM STUDIO PRO 시작... 브라우저에서 http://localhost:7900 여세요"
# stdbuf: 파이프로 보내도 즉시 기록되게 (버퍼링 끄기)
exec stdbuf -oL -eL conda run --no-capture-output -n gyu_arm \
     python -u arm_studio_pro.py 2>&1 | tee -a "$LOG"
