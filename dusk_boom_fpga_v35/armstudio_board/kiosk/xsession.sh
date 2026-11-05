#!/bin/sh
# Xorg on HDMI (simpledrm, software rendering) + browser.sh.  Log: /var/log/armstudio_screen.log
export HOME=/root LANG=C.UTF-8 LC_ALL=C.UTF-8
exec xinit /opt/dusk/kiosk/browser.sh -- /usr/lib/xorg/Xorg :0 vt1 -nolisten tcp -s 0 -dpms \
     -logfile /var/log/Xorg.0.log > /var/log/armstudio_screen.log 2>&1
