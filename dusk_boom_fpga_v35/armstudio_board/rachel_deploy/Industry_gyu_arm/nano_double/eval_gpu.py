#!/usr/bin/env python3
"""GPU 채점 — eval_cpp.py · eval_grid.py 와 같은 판정(검사+보충)을, C++ 엔진 대신 GPU(학습 모델 .pt)로 뽑아서 한다 (11-01).

  python eval_gpu.py real  모델.pt              → 실제 말 98 · 손 세트 (eval_cpp 와 같은 출력)
  python eval_gpu.py grid  모델.pt 격자.json    → 말씨 격자 (eval_grid 와 같은 출력)

C++ 엔진(앱)은 int8 이라 답이 아주 조금 다를 수 있다 → GPU 로 고르고, 고른 것만 C++ 로 다시 확인할 것.
한 문장마다 한 번 뽑기(온도 0) 1개 + 다수결용(온도 0.7) 6개를 한 묶음(7줄)으로 뽑는다.
"""
import os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "nano_norm"))
import torch
import train_double as T
import tok_norm as tk
from model import Rachel
import eval_cpp as E
import eval_grid as EG

M = {}

def load(path):
    ck = torch.load(path, map_location="cpu")
    itos = ck["vocab"]; T.CTX = ck["ctx"]
    m = Rachel(len(itos), d=ck["dim"], layers=ck["layers"], heads=ck["heads"], ctx=ck["ctx"])
    m.load_state_dict(ck["model"]); m.eval().to(T.DEV)
    M.update(m=m, itos=itos, stoi={c: i for i, c in enumerate(itos)})

@torch.no_grad()
def gen7(scene, text, n=6, temp=0.7):
    """0번 줄 = 가장 확률 높은 글자, 1~n번 줄 = 온도 뽑기."""
    m, stoi, itos = M["m"], M["stoi"], M["itos"]
    ids = T.encode_prompt(stoi, scene, text)
    x = torch.tensor([ids] * (n + 1), device=T.DEV)
    out = [[] for _ in range(n + 1)]; done = [False] * (n + 1)
    for _ in range(T.ANS_ROOM):
        with torch.autocast(device_type="cuda", dtype=torch.bfloat16, enabled=(T.DEV == "cuda")):
            lg = m(x[:, -T.CTX:])[:, -1].float()
        t = torch.multinomial(torch.softmax(lg / temp, -1), 1)[:, 0]
        t[0] = lg[0].argmax()
        for i, v in enumerate(t.tolist()):
            if done[i]: continue
            if v == tk.EOS: done[i] = True
            else: out[i].append(v)
        if all(done): break
        x = torch.cat([x, t[:, None]], 1)
    o = [tk.decode(itos, r) for r in out]
    return o[0], o[1:]

def work(args):
    wid, items = args
    res = []
    for it in items:
        t0 = time.time()
        g, many = gen7(E.scene_str(it["sc"]), it["text"], E.N, E.TEMP)
        res.append(dict(it, greedy=g, many=many, t=time.time() - t0))
    return res

class Serial:                         # Pool 대신 — 모델 하나를 GPU 에서 차례로
    def __init__(self, n): pass
    def __enter__(self): return self
    def __exit__(self, *a): pass
    def map(self, f, chunks): return [f(c) for c in chunks]

if __name__ == "__main__":
    mode, path = sys.argv[1], sys.argv[2]
    load(path)
    E.work = work; E.Pool = Serial; EG.Pool = Serial
    if mode == "real":
        sys.argv = ["eval_cpp.py", path, "6", "0.7", "1"]; E.main()
    else:
        sys.argv = ["eval_grid.py", path, sys.argv[3], "1"]; EG.main()
