#!/bin/sh
# ARM STUDIO PRO server (python, Debian tree on SD partition 3) - restarted by init when it stops
[ "$(cat /run/dusk.mode 2>/dev/null)" = armstudio ] || exec sleep 2147483647
sleep 1
exec chroot /mnt/debian /opt/dusk/kiosk/server.sh
