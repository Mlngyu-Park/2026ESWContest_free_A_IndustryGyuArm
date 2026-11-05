#!/bin/sh
# board_switch.sh — 보드를 'TPU 화면' 모드로 (보드 셸에서 실행, 되돌리기: *.armstudio 를 제자리로)
K=/mnt/debian/opt/dusk/kiosk
[ -f $K/start_screen.sh.armstudio ] || cp -p $K/start_screen.sh $K/start_screen.sh.armstudio
[ -f $K/server.sh.armstudio ] || cp -p $K/server.sh $K/server.sh.armstudio
printf '#!/bin/sh\n# 보드 HDMI = TPU 추론 화면 (ARM STUDIO 화면은 start_screen.sh.armstudio)\nexec /opt/dusk/kiosk/start_tpu_show.sh\n' > $K/start_screen.sh
printf '#!/bin/sh\n# ARM STUDIO 서버는 뮤(라떼판다)로 옮김 - 보드에서는 쉰다 (원래 것: server.sh.armstudio)\nexec sleep 2147483647\n' > $K/server.sh
chmod 755 $K/start_screen.sh $K/server.sh $K/start_tpu_show.sh
for p in $(pgrep -f run_armstudio.py) $(pgrep -f armstudio_screen.py) $(pgrep -x dusk_cursor); do kill $p; done
echo SWITCHED
