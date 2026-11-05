#!/usr/bin/env bash
# ARM STUDIO 전용 LLM 파인튜닝 (DGX Spark, 밤새 돌리기)
#
#   ./finetune_spark.sh
#
# 하는 일:
#   ① 학습 데이터 만들기 (자동 생성 + 실사용 기록)
#   ② Spark 로 데이터 보내기
#   ③ Spark 에서 LoRA 학습 (컨테이너 안에서)
#   ④ 끝나면 GGUF 로 바꿔 지금 앱이 바로 쓰게
#
# 중간에 끊겨도 다시 실행하면 이어서 됩니다.
set -u
cd "$(dirname "$0")"

REMOTE=drboom@127.0.0.1
KEY=$HOME/.ssh/spark_key
RDIR=/home/drboom/project/2026_project/finetune
# 이미 Spark 에 있는 이미지를 쓴다 (torch 2.11+cu130, transformers 5.6 포함).
# 새 컨테이너를 받으면 수십 GB 라 오래 걸린다 — 확인해보니 이걸로 충분했다.
IMG=vllm/vllm-openai:cu130-nightly
LOG=/tmp/finetune.log

# SSH 포트는 devtunnel 이 바뀌므로 살아 있는 것을 찾는다
port() {
  pgrep -af "ssh.*-L 18000" 2>/dev/null \
    | grep -oE '\-p [0-9]+' | head -1 | cut -d' ' -f2
}
P=$(port)
[ -n "$P" ] && ! ssh -i "$KEY" -p "$P" -o ConnectTimeout=5 \
    -o StrictHostKeyChecking=no "$REMOTE" true 2>/dev/null && P=""
if [ -z "$P" ]; then
  echo "[1/5] Spark 터널 연결..."
  bash spark_link.sh >/dev/null 2>&1
  P=$(port)
fi
[ -z "$P" ] && { echo "★ Spark 에 연결할 수 없습니다. spark_link.sh 를 먼저 확인하세요."; exit 1; }
SSH="ssh -i $KEY -p $P -o StrictHostKeyChecking=no $REMOTE"
echo "[Spark] SSH 포트 $P"

echo "[2/5] 학습 데이터 만들기..."
python make_llm_data.py 2000 >/dev/null || exit 1
# 실사용 기록이 있으면 합친다 (사장님이 실제로 말한 것 — 가장 값지다)
python - <<'PY'
import json, os
out = []
with open('llm_train.jsonl', encoding='utf-8') as f:
    out = [l.rstrip('\n') for l in f if l.strip()]
n0 = len(out)
SYS = json.loads(out[0])['messages'][0]['content']
if os.path.exists('llm_usage.jsonl'):
    seen = set()
    for l in open('llm_usage.jsonl', encoding='utf-8'):
        try:
            d = json.loads(l)
            t = (d.get('text') or '').strip()
            a = d.get('answer')
            if not t or not isinstance(a, dict) or t in seen: continue
            seen.add(t)
            # 실사용은 3번 넣어 비중을 높인다
            row = json.dumps({"messages": [
                {"role": "system", "content": SYS},
                {"role": "user", "content": t},
                {"role": "assistant", "content": json.dumps(a, ensure_ascii=False)}
            ]}, ensure_ascii=False)
            out += [row] * 3
        except Exception: pass
with open('llm_train.jsonl', 'w', encoding='utf-8') as f:
    f.write('\n'.join(out) + '\n')
print(f"   자동생성 {n0:,} + 실사용 {(len(out)-n0)//3:,}개(x3) = 총 {len(out):,} 줄")
PY

echo "[3/5] Spark 로 보내기..."
$SSH "mkdir -p $RDIR" || exit 1
scp -i "$KEY" -P "$P" -o StrictHostKeyChecking=no \
    llm_train.jsonl train_lora.py "$REMOTE:$RDIR/" >/dev/null || exit 1

echo "[4/5] 학습 시작 (밤새 돌아갑니다)"
echo "      진행 상황:  ssh -p $P $REMOTE 'tail -f $RDIR/train.log'"
$SSH "cd $RDIR && nohup docker run --rm --gpus all \
      -v $RDIR:/w -w /w --shm-size=8g \
      --entrypoint bash $IMG -lc '
        pip install -q --no-cache-dir peft trl datasets accelerate 2>&1 | tail -3
        python3 train_lora.py
      ' > $RDIR/train.log 2>&1 &" 

sleep 5
echo
echo "[5/5] 시작했습니다."
echo "   확인:  ssh -i $KEY -p $P $REMOTE 'tail -20 $RDIR/train.log'"
echo "   끝나면 $RDIR/out/ 에 결과가 생깁니다."
