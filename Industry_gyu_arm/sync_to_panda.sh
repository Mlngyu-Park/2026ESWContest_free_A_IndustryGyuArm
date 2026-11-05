#!/bin/bash
# 노트북 -> 라떼판다 Mu (192.168.0.46) 로 ARM STUDIO 다시 보내기 (2026-11-04)
#   ./sync_to_panda.sh          보낼 것만 보여 준다 (아무것도 안 바꿈)
#   ./sync_to_panda.sh go       실제로 보낸다 → 라떼판다에서 ./start_pro.sh 다시 실행
# 안 보내는 것 (라떼판다 것을 지킨다):
#   · rachel_cpp/rachel  — 노트북용(AVX-512)은 라떼판다 CPU(i3-N305)에서 멈춘다. 대신 rachel_avx2 를 rachel 로 보낸다
#   · calib.json         — 라떼판다에서 실물 모터로 맞춘 보정값을 덮어쓰지 않게
#   · 백업(*.bak*, backup_*), __pycache__, certs/(라떼판다 주소로 새로 만든다)
# 접속 열쇠: ~/.ssh/latte_key  (라떼판다 사용자 drboom)
PANDA=${PANDA:-drboom@192.168.0.46}
KEY=$HOME/.ssh/latte_key
cd "$(dirname "$0")/.." || exit 1
DRY=--dry-run; [ "$1" = go ] && DRY=
rsync -a $DRY --itemize-changes --partial \
  --exclude='backup_*' --exclude='*.bak*' --exclude='__pycache__' --exclude='*.mine*' \
  --exclude='Industry_gyu_arm/certs/' --exclude='.git' \
  --exclude='Industry_gyu_arm/rachel_cpp/rachel' --exclude='calib.json' \
  -e "ssh -i $KEY -o BatchMode=yes" --relative ./Industry_gyu_arm ./urdf ./stl "$PANDA:/home/drboom/2026_project/" \
  | grep -v '^\.d' 
if [ -z "$DRY" ]; then
  scp -i "$KEY" -o BatchMode=yes -q Industry_gyu_arm/rachel_cpp/rachel_avx2 \
      "$PANDA:/home/drboom/2026_project/Industry_gyu_arm/rachel_cpp/rachel" && echo "엔진: rachel_avx2 -> 라떼판다 rachel"
  echo "보냄 — 라떼판다에서: cd ~/2026_project/Industry_gyu_arm && ./start_pro.sh"
else
  echo "(미리보기만 했습니다 — 실제로 보내려면: ./sync_to_panda.sh go)"
fi
