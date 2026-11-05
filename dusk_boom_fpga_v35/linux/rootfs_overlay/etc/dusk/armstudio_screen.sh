#!/bin/sh
# ARM STUDIO screen on HDMI (Debian tree, start_screen.sh) - restarted by init when it stops
[ "$(cat /run/dusk.mode 2>/dev/null)" = armstudio ] || exec sleep 2147483647
sleep 2
exec chroot /mnt/debian /opt/dusk/kiosk/start_screen.sh
