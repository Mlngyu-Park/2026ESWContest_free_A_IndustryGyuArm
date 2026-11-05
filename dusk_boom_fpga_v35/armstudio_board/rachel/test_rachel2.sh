#!/bin/sh
# test_rachel2.sh — 서버와 같은 길 (nano_double/rachel2.py -> rachel_cpp/rachel -> TPU) 로 한 문장
cd /home/drboom/2026_project/Industry_gyu_arm/nano_double || exit 1
S="$(head -1 /opt/dusk/rachel/prompt1.txt)"
T="${1:-보라색 도킹에 있는 펜을 B팔로 잡아서 초록색에다 갖다 놔 줄래}"
python3 rachel2.py double.bin "$S" "$T" 2>&1 | tail -12
