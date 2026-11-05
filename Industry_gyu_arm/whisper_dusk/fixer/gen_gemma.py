#!/usr/bin/env python3
"""Gemma(DGX Spark)로 '진짜 같은' 오염 데이터를 만든다.

왜 필요한가
  gen_data.py 의 규칙 기반 오염은 너무 정형화돼 있다. 모델이 그 규칙만
  외워서, 조금만 다른 오염이 오면 손도 못 댄다 (실측: 새 형태 5개 중
  0~1개만 교정).

  Gemma 는 실제 음성인식이 어떻게 틀리는지 안다. 같은 정답 문장에
  대해 매번 다른 방식으로 틀려 준다 — 그게 우리가 필요한 다양성이다.

쓰는 법
  python gen_gemma.py 3000 > gemma_data.jsonl

  한 번에 여러 문장을 묶어 물어본다 (왕복이 느리므로).
  결과는 gen_data.py 것과 섞어서 학습한다.
"""
import json
import random
import sys
import urllib.request

LLM = "http://localhost:18000/v1/chat/completions"

# 정답 문장은 우리가 만든다 — Gemma 는 '틀리게 하는' 일만 시킨다.
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from gen_data import clean_sentence  # noqa: E402

PROMPT = """한국어 음성인식(STT)이 잘못 받아쓴 결과를 만들어라.

작은 음성인식 모델(27MB)이 실제로 내는 오류의 특징:
- 비슷한 소리로 잘못 들음 (도킹->토킹/카페, 옮겨->옴겨, 펜->팬)
- 조사가 틀림 (도킹으로->도킹이로)
- 같은 구간을 두 번 받아씀 (소리를 겹쳐 나눠 처리해서)
- 어미가 잘림 (옮겨줘->옮겨주)
- 잔말이 낌 (어, 음, 그)
- 같은 말이 여러 번 되풀이됨

★ 중요: 색깔 이름과 동작(잡아/놓아/옮겨)의 '순서'는 절대 바꾸지 마라.
  소리만 비슷하게 흐트러뜨려라.

아래 문장들을 각각 한 번씩 '잘못 들은 판'으로 바꿔라.
JSON 배열로만 답하라. 설명 금지.
[{"dst":"정답 그대로","src":"잘못 들은 판"}, ...]

문장:
%s"""


def ask(sentences, timeout=240):
    body = PROMPT % "\n".join(f"{i+1}. {s}" for i, s in enumerate(sentences))
    req = urllib.request.Request(
        LLM,
        data=json.dumps({
            "model": "gemma",
            "messages": [{"role": "user", "content": body}],
            "max_tokens": 2000,
            "temperature": 1.0,
        }).encode(),
        headers={"Content-Type": "application/json"})
    txt = json.loads(urllib.request.urlopen(req, timeout=timeout).read())
    txt = txt["choices"][0]["message"]["content"]
    # ```json ... ``` 을 벗긴다
    if "```" in txt:
        txt = txt.split("```")[1]
        if txt.startswith("json"):
            txt = txt[4:]
    return json.loads(txt)


def main():
    want = int(sys.argv[1]) if len(sys.argv) > 1 else 3000
    batch = 12
    random.seed()
    made = 0
    fails = 0
    while made < want and fails < 20:
        sents = [clean_sentence() for _ in range(batch)]
        try:
            rows = ask(sents)
        except Exception as e:
            fails += 1
            print(f"# 실패 {fails}: {type(e).__name__}", file=sys.stderr)
            continue
        fails = 0
        for r in rows:
            src = (r.get("src") or "").strip()
            dst = (r.get("dst") or "").strip()
            if not src or not dst or src == dst:
                continue
            # 너무 동떨어지면 버린다 (Gemma 가 딴소리한 경우)
            if len(src) > len(dst) * 2.2 or len(src) < len(dst) * 0.4:
                continue
            print(json.dumps({"src": src, "dst": dst}, ensure_ascii=False),
                  flush=True)
            made += 1
        print(f"# {made}/{want}", file=sys.stderr)


if __name__ == "__main__":
    main()
