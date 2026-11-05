#!/usr/bin/env python3
"""RACHEL 추론 + 지연 측정.
  python infer.py "d8=빨강 d9=주황 d12=노랑 | ppap=초록@d8" "빨간 도킹의 초록 펜을 노란 데로 옮겨"
  python infer.py --bench     # val 100개 정확도·평균 지연
"""
import sys, json, time, torch
from model import Rachel
import tokenizer as tk

def _mk(ck, itos):
    """체크포인트에 저장된 구조로 모델을 만든다.
    문맥·차원을 하드코딩하면 판이 바뀔 때마다 깨진다 (실제로 깨졌다)."""
    return Rachel(len(itos),
                  d=ck.get("dim", 320), layers=ck.get("layers", 8),
                  heads=ck.get("heads", 8), ctx=ck.get("ctx", 256))


DEV = "cuda" if torch.cuda.is_available() else "cpu"
ck = torch.load("rachel.pt", map_location=DEV)
itos = ck["vocab"]; stoi = {c: i for i, c in enumerate(itos)}
model = _mk(ck, itos).to(DEV); model.load_state_dict(ck["model"]); model.eval()

def ask(scene, text):
    ids = torch.tensor([tk.encode(stoi, f"{scene}\n{text}\n>")], device=DEV)
    t0 = time.time()
    out = model.generate(ids, tk.EOS)
    dt = time.time() - t0
    return tk.decode(itos, out[0, ids.size(1):].tolist()).strip(), dt

if sys.argv[1:2] == ["--bench"]:
    ok = 0; tot = 0.0; n = 100
    rows = [json.loads(l) for l in open("val.jsonl", encoding="utf-8")][:n]
    for d in rows:
        got, dt = ask(d["scene"], d["text"]); tot += dt
        ok += (got == d["answer"].strip())
    print(f"정확도 {ok}/{n}  평균 {tot/n*1000:.0f}ms  ({DEV})")
else:
    got, dt = ask(sys.argv[1], sys.argv[2])
    print(got); print(f"({dt*1000:.0f}ms, {DEV})")
