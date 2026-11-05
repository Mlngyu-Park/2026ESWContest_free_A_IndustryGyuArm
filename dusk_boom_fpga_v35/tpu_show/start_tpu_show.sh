#!/bin/sh
# 보드 HDMI = TPU 추론 화면 (tpu_show.py). ARM STUDIO 는 뮤(라떼판다)가 맡는다 (2026-11-05)
export HOME=/root LANG=C.UTF-8 LC_ALL=C.UTF-8 PYTHONUNBUFFERED=1
touch /run/tpu_cmd && chmod 666 /run/tpu_cmd
exec nice -n 5 python3 -u /opt/dusk/tpu_show/tpu_show.py >> /var/log/tpu_show.log 2>&1
