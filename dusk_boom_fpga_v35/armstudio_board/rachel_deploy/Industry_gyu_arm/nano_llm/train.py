#!/usr/bin/env python3
"""RACHEL 학습.

시퀀스 형식:  <장면>\n<명령>\n><답 JSON><eos>
손실은 '>' 뒤(답 부분)에만 건다 — 장면·명령을 외우는 데 용량을 쓰지 않게.

  python train.py                # train.jsonl / val.jsonl
"""
import json, math, time, random, torch
import torch.nn.functional as F
from model import Rachel
import tokenizer as tk

import os
DEV   = "cuda" if torch.cuda.is_available() else "cpu"
# Z80 어레이용 축소판을 같은 코드로 만든다 (환경변수로 크기 지정)
DIM   = int(os.environ.get("RACHEL_DIM",   320))
LAYER = int(os.environ.get("RACHEL_LAYER",   8))
HEAD  = int(os.environ.get("RACHEL_HEAD",    8))
OUT   = os.environ.get("RACHEL_OUT", "rachel.pt")
SEED  = os.environ.get("SEED")                 # 재학습 대조용. 없으면 지금과 동일(시드 미고정)
if SEED is not None:
    random.seed(int(SEED)); torch.manual_seed(int(SEED))
    if torch.cuda.is_available(): torch.cuda.manual_seed_all(int(SEED))
CTX   = int(os.environ.get("RACHEL_CTX",   256))
BATCH = 192
EPOCH = 8
LR    = 3e-4

def load(name):
    out = []
    for line in open(name, encoding="utf-8"):
        d = json.loads(line)
        prompt = f"{d['scene']}\n{d['text']}\n>"
        out.append((prompt, d["answer"]))
    return out

def pack(stoi, prompt, answer):
    ids = tk.encode(stoi, prompt)
    ans = tk.encode(stoi, answer) + [tk.EOS]
    x = (ids + ans)[:CTX]
    loss_from = min(len(ids), CTX)
    return x, loss_from

def batches(data, stoi, bs, shuffle=True):
    idx = list(range(len(data)))
    if shuffle: random.shuffle(idx)
    for i in range(0, len(idx), bs):
        chunk = [pack(stoi, *data[j]) for j in idx[i:i+bs]]
        L = max(len(x) for x, _ in chunk)
        X = torch.full((len(chunk), L), tk.PAD, dtype=torch.long)
        M = torch.zeros((len(chunk), L), dtype=torch.bool)
        for r, (x, lf) in enumerate(chunk):
            X[r, :len(x)] = torch.tensor(x)
            M[r, lf:len(x)] = True              # 답 부분만 손실
        yield X.to(DEV), M.to(DEV)

def loss_of(model, X, M):
    logits = model(X[:, :-1])
    tgt = X[:, 1:].clone()
    tgt[~M[:, 1:]] = -100
    return F.cross_entropy(logits.reshape(-1, logits.size(-1)),
                           tgt.reshape(-1), ignore_index=-100)

@torch.no_grad()
def exact_match(model, stoi, itos, data, n=200):
    ok = 0
    for prompt, answer in data[:n]:
        ids = torch.tensor([tk.encode(stoi, prompt)], device=DEV)
        out = model.generate(ids, tk.EOS)
        got = tk.decode(itos, out[0, ids.size(1):].tolist())
        ok += (got.strip() == answer.strip())
    return ok / n

def main():
    train, val = load("train.jsonl"), load("val.jsonl")
    stoi = tk.build([p + a for p, a in train])
    _, itos = tk.load()
    model = Rachel(len(itos),d=DIM,layers=LAYER,heads=HEAD,ctx=CTX).to(DEV)
    print(f"장치 {DEV} · 어휘 {len(itos)} · d{DIM} L{LAYER} ctx{CTX} · "
          f"파라미터 {model.n_params()/1e6:.2f}M -> {OUT}")
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=0.05)
    steps = math.ceil(len(train)/BATCH) * EPOCH
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, steps)
    best = 0.0; step = 0; t0 = time.time()
    for ep in range(1, EPOCH+1):
        model.train()
        for X, M in batches(train, stoi, BATCH):
            with torch.autocast(DEV, dtype=torch.bfloat16, enabled=DEV=="cuda"):
                loss = loss_of(model, X, M)
            opt.zero_grad(set_to_none=True); loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step(); sched.step(); step += 1
            if step % 100 == 0:
                print(f"ep{ep} step{step}/{steps} loss {loss.item():.4f} "
                      f"({time.time()-t0:.0f}s)", flush=True)
        model.eval()
        with torch.no_grad():
            vl = sum(loss_of(model, X, M).item()
                     for X, M in batches(val, stoi, BATCH, False))
            vl /= math.ceil(len(val)/BATCH)
        em = exact_match(model, stoi, itos, val)
        print(f"== ep{ep} val_loss {vl:.4f} exact_match {em:.1%} ==", flush=True)
        if em >= best:
            best = em
            torch.save({"model": model.state_dict(), "vocab": itos,
                        "dim": DIM, "layers": LAYER, "heads": HEAD, "ctx": CTX}, OUT)
            print(f"   저장: {OUT} (best {best:.1%})", flush=True)
    print(f"끝. 최고 정확도 {best:.1%} · 총 {time.time()-t0:.0f}초")

if __name__ == "__main__":
    main()
