#!/bin/bash
# 10차 (11-05): 9차 모델이 "주황색에 걸쳐서 빨간색으로" 의 경유(주황)를 빠뜨림 — 생성기에 경유 말투가 없었다.
#   ① 지금 앱 모델(9차 1000) 경유 시험지 기준  ② 데이터 6×60000 (CPU 0~9)  ③ 9차 1000 에서 이어 1에폭 1e-4, 1000걸음마다 저장
#   ④ 저장 걸음 전부: 기존 시험지(실제·격자1~3·팔·비) + 경유 시험지 → evals_v10_gpu.log
cd ~/arm_eval/nano_double
PY=~/miniconda3/envs/kanon_ft/bin/python
L=evals_v10_gpu.log
via() {
  m=$1
  { echo "  [경유 시험지]"; EVAL_OUT=gpu10_via_$m.json taskset -c 0-9 $PY eval_gpu.py grid $m.pt grid_eval_via.json > /dev/null 2>&1; \
    python3 via_sum.py gpu10_via_$m.json | sed -n "1,40p"; } >> $L
}
evalall() {
  m=$1
  { echo "######## $(date +%H:%M) $m"; \
    EVAL_OUT=gpu10_real_$m.json taskset -c 0-9 $PY eval_gpu.py real $m.pt 2>&1 | grep "검사+보충"; \
    for g in 1 2 3; do f=grid_eval$([ $g = 1 ] || echo $g).json; echo "  [격자 세트$g]"; \
      EVAL_OUT=gpu10_grid${g}_$m.json taskset -c 0-9 $PY eval_gpu.py grid $m.pt $f 2>&1 | sed -n "2,2p"; done; \
    python3 g3sum.py gpu10_grid3_$m.json | sed -n "2,3p"; \
    echo "  [팔 이름 시험지]"; EVAL_OUT=gpu10_arm_$m.json taskset -c 0-9 $PY eval_gpu.py grid $m.pt grid_eval_arm.json > /dev/null 2>&1; \
    python3 arm_sum.py gpu10_arm_$m.json | sed -n "2,12p"; \
    echo "  [비 낱말 시험지]"; EVAL_OUT=gpu10_bi_$m.json taskset -c 0-9 $PY eval_gpu.py grid $m.pt grid_eval_bi.json > /dev/null 2>&1; \
    python3 arm_sum.py gpu10_bi_$m.json | sed -n "2,6p"; } >> $L
  via $m
}
echo "######## $(date +%H:%M) 10차 시작 (경유 말투)" > $L
cp gen_double.py gen_double_v9_keep.py; cp gen_double_v10.py gen_double.py
mkdir -p v10; rm -f v10/part*.jsonl
for s in 1 2 3 4 5 6; do DOUBLE_SEED=$((s*1000+101)) taskset -c 0-9 nice -n 10 python3 gen_double.py 60000 > v10/part$s.jsonl 2> v10/gen$s.log & done
{ echo "######## $(date +%H:%M) ck_v9_1000 (지금 앱 모델) — 경유 시험지 기준"; } >> $L; via ck_v9_1000
wait
cat v10/part*.jsonl > v10/double_data_v10.jsonl && rm v10/part*.jsonl
echo "$(date +%H:%M) 데이터 $(wc -l < v10/double_data_v10.jsonl)줄 · 경유 문장 $(grep -c '걸쳐\|거쳐\|들렀\|들러\|찍고\|경유\|놨다가\|두었다가\|갔다가' v10/double_data_v10.jsonl) · 학습 시작 (init = 9차 1000)" >> $L
cp ck_v9_1000.pt init_v10.pt
DBL_INIT=init_v10.pt DBL_DATA=v10/double_data_v10.jsonl DBL_OUT=double_v10.pt DBL_DIM=448 DBL_LAYER=10 DBL_CTX=768 DBL_BATCH=128 DBL_EPOCH=1 DBL_LR=1e-4 DBL_SAVE_EVERY=1000 \
  taskset -c 0-9 $PY train_double_v2.py > train_v10.log 2>&1 &
TP=$!
seen=""; Q=""
while true; do
  st=$(grep "==" train_v10.log | tail -1 | grep -o "^== [0-9]*" | grep -o "[0-9]*")
  if [ -n "$st" ] && [ "$st" != "$seen" ]; then
    sleep 20; cp double_v10_last.pt ck_v10_$st.pt
    CUDA_VISIBLE_DEVICES= taskset -c 0-9 $PY export_double.py ck_v10_$st.pt ck_v10_$st.bin > /dev/null 2>&1
    Q="$Q $st"; seen=$st; echo "$(date +%H:%M) 저장 $st" >> $L
  fi
  if ! kill -0 $TP 2>/dev/null; then
    st=$(grep "==" train_v10.log | tail -1 | grep -o "^== [0-9]*" | grep -o "[0-9]*")
    [ "$st" = "$seen" ] && break
  fi
  sleep 15
done
echo "$(date +%H:%M) 학습 끝 · $(grep "^끝" train_v10.log)" >> $L
for st in $Q; do evalall ck_v10_$st; done
echo "######## 채점 끝 $(date +%H:%M)" >> $L
