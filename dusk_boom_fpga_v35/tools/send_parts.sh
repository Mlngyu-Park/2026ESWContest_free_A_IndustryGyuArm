#!/bin/bash
# send_parts.sh - upload xfer/deb/*.lines one by one (skips parts already OK), log to send_parts.log
cd /mnt/c/work/fpga_v35/tools
LOG=/home/drboom/2026_project/dusk_boom_fpga_v35/debian/send_parts.log
for f in xfer/deb/debootstrap.lines xfer/deb/part*.lines; do
  n=$(basename $f .lines)
  grep -q "^OK $n$" $LOG 2>/dev/null && continue
  for try in 1 2 3; do
    r=$(timeout 1800 powershell.exe -NoProfile -ExecutionPolicy Bypass -File 'C:\work\fpga_v35\tools\send.ps1' -Baud 921600 -Lines "C:\\work\\fpga_v35\\tools\\$(echo $f | tr / '\\')" 2>&1 | tr -d '\r' | tail -1)
    echo "$(date +%T) $n try$try: $r" >> $LOG
    if echo "$r" | grep -q RX_OK; then echo "OK $n" >> $LOG; break; fi
  done
done
echo "ALL_DONE" >> $LOG
