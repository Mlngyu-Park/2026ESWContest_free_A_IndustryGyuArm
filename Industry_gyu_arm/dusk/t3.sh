#!/bin/bash
# 문장 3개로 빠르게 확인 — 시프트 변환이 답을 바꾸는가
cd "$(dirname "$0")"
W=../rachel_cpp/rachel.bin
ok=0
for i in 0 1 2; do
  P=$(python3 -c "
import json
r=[json.loads(l) for l in open('../nano_llm/val.jsonl',encoding='utf-8')][:3]
d=r[$i]; print(f\"{d['scene']}\n{d['text']}\n>\", end='')")
  E=$(python3 -c "
import json
r=[json.loads(l) for l in open('../nano_llm/val.jsonl',encoding='utf-8')][:3]
print(r[$i]['answer'].strip())")
  G=$(timeout 300 ./dusk --weights $W --ask "$P" 2>/dev/null | grep '^출력' | sed 's/^출력: //')
  if [ "$G" = "$E" ]; then ok=$((ok+1)); else
    echo "  [$i] 틀림"
    echo "      정답: ${E:0:60}"
    echo "      dusk: ${G:0:60}"
  fi
done
echo "  => $ok/3"
