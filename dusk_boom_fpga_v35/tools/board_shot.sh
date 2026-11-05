#!/bin/bash
# board_shot.sh OUT.png - grab the HDMI screen (/dev/fb0) of the board over LAN
B=${BOARD_IP:-192.168.0.47}
printf '(nc -l -p 5556 < /dev/fb0 &)\r\n' > /mnt/c/work/fpga_v35/tools/shot_cmd.txt
timeout 30 powershell.exe -NoProfile -ExecutionPolicy Bypass -File 'C:\work\fpga_v35\tools\com_cmds.ps1' -File 'C:\work\fpga_v35\tools\shot_cmd.txt' -Wait 1200 -Tail 200 > /dev/null 2>&1
timeout 30 nc -w 3 $B 5556 > "$1.raw"
python3 -c "
from PIL import Image
d=open('$1.raw','rb').read()[:1280*720*4]
Image.frombytes('RGBX',(1280,720),d,'raw','BGRX').convert('RGB').save('$1')"
rm -f "$1.raw"; echo "$1"
