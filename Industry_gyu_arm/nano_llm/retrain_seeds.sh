#!/usr/bin/env bash
# 표 1 재학습 — 힌트 유/무 × 시드 3. DGX Spark 에서 돌린다 (노트북 금지).
#   같은 검증셋(val_fixed.jsonl)으로 여섯 판을 전부 채점한다.
#   결과: results_seeds.txt  (판 · 시드 · 전체 · 색지칭)
set -e
cd "$(dirname "$0")"
python gen_data.py 100000 > /dev/null                     # 힌트 O 데이터 (기본)
cp val.jsonl val_fixed.jsonl                              # ★ 여섯 판 공통 검증셋 — 다시 만들지 않는다
mkdir -p seeds
for H in hint nohint; do
  for S in 1 2 3; do
    if [ $H = nohint ]; then NOHINT=1 python gen_data.py 100000 > /dev/null; else python gen_data.py 100000 > /dev/null; fi
    cp val_fixed.jsonl val.jsonl                          # 검증은 항상 같은 셋
    SEED=$S RACHEL_OUT=seeds/rachel_${H}_s${S}.pt python train.py
    python analyze.py seeds/rachel_${H}_s${S}.pt > seeds/analyze_${H}_s${S}.txt   # analyze.py 가 인자를 받게 고쳐야 함(아래)
  done
done
echo "끝. seeds/analyze_*.txt 의 '색지칭' 줄과 6종류 합산이 표 1 이다."
