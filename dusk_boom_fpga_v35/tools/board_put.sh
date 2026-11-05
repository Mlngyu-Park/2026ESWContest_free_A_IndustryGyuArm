#!/bin/bash
# board_put.sh LOCAL_FILE DEBIAN_PATH [after-command]  - copy one file to the board over LAN
#   DEBIAN_PATH is inside the Debian tree (/ = SD partition 3, e.g. /opt/dusk/screen/x.py);
#   the board receives with /opt/dusk/tools/rx_tcp.py, the sha256 is compared
set -e
B=${BOARD_IP:-192.168.0.47}; PORT=${PORT:-5557}
T=/mnt/c/work/fpga_v35/tools
src=$1; dst=$2; after=${3:-true}
sum=$(sha256sum "$src" | cut -c1-16)
printf '(chroot /mnt/debian python3 /opt/dusk/tools/rx_tcp.py %s %s < /dev/null > /tmp/put.log 2>&1 && %s) &\r\n' $PORT "$dst" "$after" > $T/put_cmd.txt
timeout 30 powershell.exe -NoProfile -ExecutionPolicy Bypass -File 'C:\work\fpga_v35\tools\com_cmds.ps1' -File 'C:\work\fpga_v35\tools\put_cmd.txt' -Wait 2500 -Tail 200 > /dev/null 2>&1
timeout 300 nc -q 1 $B $PORT < "$src"
printf 'sleep 1; cat /tmp/put.log\r\n' > $T/put_chk.txt
got=$(timeout 30 powershell.exe -NoProfile -ExecutionPolicy Bypass -File 'C:\work\fpga_v35\tools\com_cmds.ps1' -File 'C:\work\fpga_v35\tools\put_chk.txt' -Wait 2500 -Tail 300 2>&1 | tr -d '\r' | grep -o "PUT_OK [0-9a-f]*" | tail -1)
echo "$src -> $dst : local $sum, board ${got#PUT_OK }"
[ "${got#PUT_OK }" = "$sum" ]
