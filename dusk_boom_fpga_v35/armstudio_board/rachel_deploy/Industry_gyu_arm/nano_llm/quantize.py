#!/usr/bin/env python3
"""RACHEL int8 동적 양자화 — 크기·속도·정확도 비교.

동적 양자화(dynamic quantization): Linear 층의 가중치를 int8 로 저장하고,
활성값은 실행 중에 스케일을 정해 int8 로 곱한 뒤 되돌린다.
학습 없이 바로 적용되고 CPU 에서 효과가 있다 (GPU 는 대상 아님).

  python quantize.py            # 비교 후 rachel_int8.pt 저장
"""
import json, os, time, torch
from model import Rachel
import tokenizer as tk, hints

def _mk(ck, itos):
    """체크포인트에 저장된 구조로 모델을 만든다.
    문맥·차원을 하드코딩하면 판이 바뀔 때마다 깨진다 (실제로 깨졌다)."""
    return Rachel(len(itos),
                  d=ck.get("dim", 320), layers=ck.get("layers", 8),
                  heads=ck.get("heads", 8), ctx=ck.get("ctx", 256))


CASES = [json.loads(l) for l in open("val.jsonl", encoding="utf-8")][:60]

def load_fp32():
    ck = torch.load("rachel.pt", map_location="cpu")
    itos = ck["vocab"]; m = _mk(ck, itos)
    m.load_state_dict(ck["model"]); m.eval()
    return m, itos, {c: i for i, c in enumerate(itos)}

def bench(m, stoi, itos, n=60):
    ok = 0; t_all = 0.0
    for d in CASES[:n]:
        p = f"{d['scene']}\n{d['text']}\n>"
        ids = torch.tensor([tk.encode(stoi, p)])
        t0 = time.time()
        out = m.generate(ids, tk.EOS)
        t_all += time.time() - t0
        got = tk.decode(itos, out[0, ids.size(1):].tolist()).strip()
        ok += (got == d["answer"].strip())
    return ok / n, t_all / n * 1000

def size_mb(path): return os.path.getsize(path) / 1048576

torch.set_num_threads(4)          # 라떼판다급(4코어) 조건으로 맞춘다
m, itos, stoi = load_fp32()
a1, t1 = bench(m, stoi, itos)
print(f"fp32   정확도 {a1:6.1%}  {t1:7.1f}ms  {size_mb('rachel.pt'):.1f}MB")

q = torch.quantization.quantize_dynamic(m, {torch.nn.Linear}, dtype=torch.qint8)
torch.save({"model": q.state_dict(), "vocab": itos, "int8": True}, "rachel_int8.pt")
a2, t2 = bench(q, stoi, itos)
print(f"int8   정확도 {a2:6.1%}  {t2:7.1f}ms  {size_mb('rachel_int8.pt'):.1f}MB")
print(f"\n크기 {size_mb('rachel.pt')/size_mb('rachel_int8.pt'):.2f}배 감소 · "
      f"속도 {t1/t2:.2f}배 · 정확도 {a2-a1:+.1%}")
