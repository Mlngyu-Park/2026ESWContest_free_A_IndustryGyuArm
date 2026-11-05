#!/bin/bash
# poll the board install log over serial every 60 s until INSTALL_DONE / INSTALL_FAIL (max 90 min)
for i in $(seq 1 90); do
  out=$(timeout 40 powershell.exe -NoProfile -ExecutionPolicy Bypass -File 'C:\work\fpga_v35\tools\com_cmds.ps1' -File 'C:\work\fpga_v35\tools\c42.txt' -Wait 3000 -Tail 500 2>&1 | tr -d '\r' | grep -v "^grep\|^root@" | tail -6)
  echo "== $(date +%T)"; echo "$out"
  echo "$out" | grep -q "INSTALL_DONE\|INSTALL_FAIL" && exit 0
  sleep 60
done
