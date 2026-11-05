#!/bin/bash
# 조종용 IK 표 미리 만들기 — 이거 하나면 끝
#   ./build_joy_table.sh          이어서 만들기 (Ctrl+C 로 멈춰도 저장됨)
#   ./build_joy_table.sh --fresh  처음부터 새로
cd "$(dirname "$0")"

# 가상환경(gyu_arm) 을 확실히 켠다 — GPU(CUDA) 는 여기에만 설치돼 있음
if [ -z "$CONDA_EXE" ]; then
  for c in "$HOME/anaconda3" "$HOME/miniconda3" /opt/conda; do
    [ -f "$c/etc/profile.d/conda.sh" ] && . "$c/etc/profile.d/conda.sh" && break
  done
else
  . "$(dirname "$(dirname "$CONDA_EXE")")/etc/profile.d/conda.sh"
fi
conda activate gyu_arm || { echo "가상환경 gyu_arm 을 못 켰습니다"; exit 1; }
echo "가상환경: $CONDA_DEFAULT_ENV"

exec python -u build_joy_table.py "$@"
