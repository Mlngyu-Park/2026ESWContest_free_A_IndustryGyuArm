#!/bin/sh
# ARM STUDIO PRO server on the board (inside the Debian tree).  Log: /var/log/armstudio.log
export HOME=/root LANG=C.UTF-8 LC_ALL=C.UTF-8 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
# 음성 인식: whisper.cpp 서버를 스파크와 같은 자리(127.0.0.1:18090)에 띄우면 원본 /api/stt 가 그대로 쓴다
if [ -x /opt/dusk/stt/whisper-server ] && ! pgrep -f whisper-server >/dev/null; then
    M=$(ls /opt/dusk/stt/ggml-*.bin 2>/dev/null | head -1)
    [ -n "$M" ] && /opt/dusk/stt/whisper-server -m "$M" -l ko -t 2 --host 127.0.0.1 --port 18090 \
        >> /var/log/whisper.log 2>&1 &
fi
cd /home/drboom/2026_project/Industry_gyu_arm || exit 1
echo "==== $(date '+%F %T') ARM STUDIO start" >> /var/log/armstudio.log
exec python3 -u /opt/dusk/run_armstudio.py >> /var/log/armstudio.log 2>&1
