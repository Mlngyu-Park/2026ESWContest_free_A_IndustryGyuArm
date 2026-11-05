#!/usr/bin/env python3
"""FIXER 학습 — 지저분한 말을 깨끗한 말로.

시퀀스 형식:  <오염된 말>\n><깨끗한 말><eos>
손실은 '>' 뒤(고친 말)에만 건다 — 입력을 외우는 데 용량을 쓰지 않게.
(RACHEL 의 train.py 와 같은 방식이다. 검증된 것을 그대로 쓴다)

  python train.py               # fix_data.jsonl 을 읽는다
"""
import json
import os
import random
import time

import torch
import torch.nn.functional as F

import tokenizer as tk
from model import Rachel

DEV   = "cuda" if torch.cuda.is_available() else "cpu"
DIM   = int(os.environ.get("FIX_DIM",   256))
LAYER = int(os.environ.get("FIX_LAYER",   6))
HEAD  = int(os.environ.get("FIX_HEAD",    8))
CTX   = int(os.environ.get("FIX_CTX",   384))
OUT   = os.environ.get("FIX_OUT", "fixer.pt")
DATA  = os.environ.get("FIX_DATA", "fix_data.jsonl")
BATCH = int(os.environ.get("FIX_BATCH", 96))
EPOCH = int(os.environ.get("FIX_EPOCH", 6))
LR    = float(os.environ.get("FIX_LR", 3e-4))
VAL_N = 2000


def load(name):
    out = []
    for line in open(name, encoding="utf-8"):
        d = json.loads(line)
        out.append((f"{d['src']}\n>", d["dst"]))
    return out


def pack(stoi, prompt, answer):
    ids = tk.encode(stoi, prompt)
    ans = tk.encode(stoi, answer) + [tk.EOS]
    x = (ids + ans)[:CTX]
    loss_from = min(len(ids), CTX)
    return x, loss_from


def batches(data, stoi, bs, shuffle=True):
    idx = list(range(len(data)))
    if shuffle:
        random.shuffle(idx)
    for i in range(0, len(idx), bs):
        chunk = [pack(stoi, *data[j]) for j in idx[i:i + bs]]
        L = max(len(x) for x, _ in chunk)
        X = torch.full((len(chunk), L), tk.PAD, dtype=torch.long)
        M = torch.zeros((len(chunk), L), dtype=torch.bool)
        for r, (x, lf) in enumerate(chunk):
            X[r, :len(x)] = torch.tensor(x)
            M[r, lf:len(x)] = True          # 답 부분에만 손실
        yield X.to(DEV), M.to(DEV)


def run_loss(model, X, M):
    out = model(X[:, :-1])
    tgt = X[:, 1:]
    msk = M[:, 1:]
    if msk.sum() == 0:
        return None
    return F.cross_entropy(out[msk], tgt[msk])


def main():
    data = load(DATA)
    random.seed(7)
    random.shuffle(data)
    val, train = data[:VAL_N], data[VAL_N:]

    texts = [p + a for p, a in data]
    stoi = tk.build(texts, "vocab.json")
    itos = json.load(open("vocab.json", encoding="utf-8"))

    model = Rachel(len(itos), d=DIM, layers=LAYER, heads=HEAD, ctx=CTX).to(DEV)
    mb = model.n_params() * 4 / 1e6
    print(f"장치 {DEV} · 어휘 {len(itos)} · d{DIM} L{LAYER} ctx{CTX}")
    print(f"파라미터 {model.n_params()/1e6:.2f}M · fp32 {mb:.1f}MB "
          f"· int8 {mb/4:.1f}MB")
    print(f"학습 {len(train):,} / 검증 {len(val):,}")

    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=0.01)
    steps = EPOCH * ((len(train) + BATCH - 1) // BATCH)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, LR, total_steps=steps,
                                                pct_start=0.1)
    best = 1e9
    t0 = time.time()
    for ep in range(1, EPOCH + 1):
        model.train()
        tot = n = 0
        for X, M in batches(train, stoi, BATCH):
            loss = run_loss(model, X, M)
            if loss is None:
                continue
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            sched.step()
            tot += loss.item()
            n += 1
        tr = tot / max(n, 1)

        model.eval()
        tot = n = 0
        with torch.no_grad():
            for X, M in batches(val, stoi, BATCH, shuffle=False):
                loss = run_loss(model, X, M)
                if loss is not None:
                    tot += loss.item()
                    n += 1
        vl = tot / max(n, 1)
        el = time.time() - t0
        print(f"  {ep}/{EPOCH}  학습 {tr:.4f}  검증 {vl:.4f}  ({el/60:.1f}분)")
        if vl < best:
            best = vl
            torch.save({"model": model.state_dict(), "vocab": itos,
                        "dim": DIM, "layers": LAYER, "heads": HEAD,
                        "ctx": CTX}, OUT)
            print(f"      저장: {OUT}")
    print(f"끝. 최고 검증 손실 {best:.4f}")


if __name__ == "__main__":
    main()
