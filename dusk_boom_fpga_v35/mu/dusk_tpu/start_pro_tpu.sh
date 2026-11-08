#!/bin/bash
# ARM STUDIO PRO + FPGA TPU 화면 (뮤 전용, 2026-11-05) — 원본 start_pro.sh 와 같고, 실행만 dusk_tpu_launch.py 를 거친다
APP="$HOME/2026_project/Industry_gyu_arm"
HERE="$(dirname "$(readlink -f "$0")")"
cd "$APP" || exit 1
pkill -f 'python.*arm_studio_pro\.py' 2>/dev/null && echo "이전 서버 정리 중..."
for i in $(seq 1 10); do
    ss -tln 2>/dev/null | grep -qE ':(7900|7870)\s' || break
    sleep 0.5
done
bash spark_link.sh || true
bash "$HERE/start_spot.sh" || true      # 11-06: ④ Spot 제어 서버 (:7910) 같이 켜기
LOG=/tmp/arm_studio.log
: > "$LOG"
ln -sf "$LOG" ./arm_studio.log 2>/dev/null || true
echo "ARM STUDIO PRO (TPU 화면) 시작... http://localhost:7900"
exec stdbuf -oL -eL conda run --no-capture-output -n gyu_arm \
     python -u "$HERE/dusk_tpu_launch.py" arm_studio_pro.py 2>&1 | tee -a "$LOG"
