#!/bin/sh
# 보드 화면: 기본 = 브라우저 없는 ARM STUDIO 화면 (/opt/dusk/screen/armstudio_screen.py)
#           /boot/dusk.conf 에 SCREEN=browser 이면 예전 방식 (Xorg + Firefox, xsession.sh)
export HOME=/root LANG=C.UTF-8 LC_ALL=C.UTF-8 PYTHONUNBUFFERED=1
SCREEN=native
[ -f /boot/dusk.conf ] && eval "$(tr -d '\r' < /boot/dusk.conf | grep '^SCREEN=')"
[ "$SCREEN" = browser ] && exec /opt/dusk/kiosk/xsession.sh
export PYTHONPATH=/opt/dusk/py
STT_HOSTS=""; [ -f /boot/dusk.conf ] && STT_HOSTS="$(tr -d "\r" < /boot/dusk.conf | sed -n "s/^STT_HOSTS=//p")"
export STT_HOSTS
# 커서는 C 프로그램이 따로 (화면을 다시 그리는 동안에도 마우스가 끊기지 않게, 높은 우선순위)
if [ -x /opt/dusk/screen/dusk_cursor ] && ! pgrep -x dusk_cursor >/dev/null; then
    nice -n -10 /opt/dusk/screen/dusk_cursor >> /var/log/dusk_cursor.log 2>&1 &
fi
exec nice -n 5 python3 -u /opt/dusk/screen/armstudio_screen.py >> /var/log/armstudio_screen.log 2>&1
