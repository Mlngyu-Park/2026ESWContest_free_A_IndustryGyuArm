#!/bin/bash
# 닭 머리 표 만들기 (GPU) — CPU 판보다 약 60배 빠름
#   ./build_head_gpu.sh           ±300mm
#   ./build_head_gpu.sh 1200      전 범위 (약 2시간)
cd "$(dirname "$0")"
if [ -z "$CONDA_EXE" ]; then
  for c in "$HOME/anaconda3" "$HOME/miniconda3" /opt/conda; do
    [ -f "$c/etc/profile.d/conda.sh" ] && . "$c/etc/profile.d/conda.sh" && break
  done
else
  . "$(dirname "$(dirname "$CONDA_EXE")")/etc/profile.d/conda.sh"
fi
conda activate gyu_arm || { echo "가상환경 gyu_arm 을 못 켰습니다"; exit 1; }
export ARM_BASE_URDF="$HOME/2026_project/urdf/ur15_20260823/urdf/ur15_20260823.urdf"
echo "가상환경: $CONDA_DEFAULT_ENV · 로봇: UR15 · GPU"
exec python -u build_head_gpu.py "$@"
