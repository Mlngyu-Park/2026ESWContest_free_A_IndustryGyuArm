#!/bin/bash
# 예전 Gradio 화면으로 실행 (지금 앱과 같은 코드, 화면만 옛것)
#   비교용: 움직임이 부드러운지 확인할 때
cd "$(dirname "$0")"
if [ -z "$CONDA_EXE" ]; then
  for c in "$HOME/anaconda3" "$HOME/miniconda3" /opt/conda; do
    [ -f "$c/etc/profile.d/conda.sh" ] && . "$c/etc/profile.d/conda.sh" && break
  done
else
  . "$(dirname "$(dirname "$CONDA_EXE")")/etc/profile.d/conda.sh"
fi
conda activate gyu_arm || { echo "가상환경 gyu_arm 을 못 켰습니다"; exit 1; }
echo "Gradio 화면: http://localhost:7860 (7860 이 차 있으면 7861...)"
exec python -u arm_studio_web.py
