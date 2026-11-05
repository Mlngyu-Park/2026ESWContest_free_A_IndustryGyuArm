#!/usr/bin/env bash
# 닭 머리 표의 구멍 메우기 (조종 중 뚝 끊기는 칸들)
cd "$(dirname "$0")"
source ~/anaconda3/etc/profile.d/conda.sh
conda activate gyu_arm
python fill_head_holes.py "$@"
