#!/bin/bash
cd /home/drboom/2026_project/Industry_gyu_arm/sol
printf "%s\n" "60 37 1" "66 41 1" "72 45 1" "80 50 1" "84 62 1" "54 33 2" "60 37 2" "66 41 2" "72 45 2" "54 33 3" "60 37 3" "54 33 4" \
 | xargs -P 6 -L 1 bash -c 'PYTHONUNBUFFERED=1 python3 space.py --pes 1 --board $0 $1 --layers $2 > .space/log_$0x$1_L$2.txt 2>&1'
echo "SWEEP DONE" >> .space/results.txt
