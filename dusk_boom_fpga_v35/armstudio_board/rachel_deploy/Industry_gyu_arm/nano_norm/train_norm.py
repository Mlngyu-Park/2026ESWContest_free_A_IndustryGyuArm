#!/usr/bin/env python3
"""말 정리기 학습 — 지저분한 말 → 정해진 꼴.

시퀀스:  <지저분한 말>\n><정해진 꼴><eos>   (손실은 '>' 뒤에만 — FIXER·RACHEL 과 같은 방식)
FIXER 의 train.py 를 바탕으로:
  · 글자 사전을 넓게 (tok_norm: 한글 2,350자 + 낱자모 풀기)
  · GPU 에서 bf16 (빠르게)
  · 중간 저장 (NORM_SAVE_EVERY 걸음마다) + 검증에서 '정답과 글자까지 같은 비율' 을 잰다

  NORM_DIM=384 NORM_LAYER=10 NORM_CTX=512 python train_norm.py
"""
import json, math, os, random, time
import torch
import torch.nn.functional as F
import tok_norm as tk
from model import Rachel

DEV   = "cuda" if torch.cuda.is_available() else "cpu"
DIM   = int(os.environ.get("NORM_DIM", 384))
LAYER = int(os.environ.get("NORM_LAYER", 10))
HEAD  = int(os.environ.get("NORM_HEAD", 8))
CTX   = int(os.environ.get("NORM_CTX", 512))
DATA  = os.environ.get("NORM_DATA", "norm_data.jsonl")
OUT   = os.environ.get("NORM_OUT", "norm.pt")
BATCH = int(os.environ.get("NORM_BATCH", 192))
EPOCH = int(os.environ.get("NORM_EPOCH", 2))
LR    = float(os.environ.get("NORM_LR", 4e-4))
SAVE_EVERY = int(os.environ.get("NORM_SAVE_EVERY", 2000))
VAL_N = 3000
EXACT_N = int(os.environ.get("NORM_EXACT_N", 300))   # 검증 중 실제로 생성해서 글자까지 맞는지 볼 개수

def load(name):
    out = []
    for line in open(name, encoding="utf-8"):
        d = json.loads(line)
        out.append((f"{d['src']}\n>", d["dst"]))
    return out

def pack(stoi, prompt, answer):
    ids = tk.encode(stoi, prompt)
    if len(ids) > CTX - 96: ids = ids[-(CTX - 96):]          # 아주 긴 입력은 앞을 자른다 (답 자리 확보)
    ans = tk.encode(stoi, answer) + [tk.EOS]
    x = (ids + ans)[:CTX]
    return x, len(ids)

def batches(data, stoi, bs, shuffle=True):
    idx = list(range(len(data)))
    if shuffle: random.shuffle(idx)
    for i in range(0, len(idx), bs):
        chunk = [pack(stoi, *data[j]) for j in idx[i:i + bs]]
        L = max(len(x) for x, _ in chunk)
        X = torch.full((len(chunk), L), tk.PAD, dtype=torch.long)
        M = torch.zeros((len(chunk), L), dtype=torch.bool)
        for r, (x, lf) in enumerate(chunk):
            X[r, :len(x)] = torch.tensor(x); M[r, lf:len(x)] = True
        yield X.to(DEV, non_blocking=True), M.to(DEV, non_blocking=True)

def run_loss(model, X, M):
    with torch.autocast(device_type="cuda", dtype=torch.bfloat16, enabled=(DEV == "cuda")):
        out = model(X[:, :-1])
    tgt, msk = X[:, 1:], M[:, 1:]
    if msk.sum() == 0: return None
    return F.cross_entropy(out[msk].float(), tgt[msk])

@torch.no_grad()
def greedy(model, stoi, itos, prompt, limit=200):
    ids = tk.encode(stoi, prompt)[-(CTX - 1):]
    x = torch.tensor([ids], device=DEV); out = []
    for _ in range(limit):
        with torch.autocast(device_type="cuda", dtype=torch.bfloat16, enabled=(DEV == "cuda")):
            lg = model(x[:, -CTX:])[0, -1]
        t = int(lg.argmax())
        if t == tk.EOS: break
        out.append(t); x = torch.cat([x, torch.tensor([[t]], device=DEV)], 1)
    return tk.decode(itos, out)

def evaluate(model, val, stoi, itos):
    model.eval(); tot = n = 0
    for X, M in batches(val, stoi, BATCH, shuffle=False):
        l = run_loss(model, X, M)
        if l is not None: tot += l.item(); n += 1
    ok = sum(greedy(model, stoi, itos, p) == a for p, a in val[:EXACT_N])
    model.train()
    return tot / max(n, 1), ok / EXACT_N

def main():
    data = load(DATA)
    random.seed(7); random.shuffle(data)
    val, train = data[:VAL_N], data[VAL_N:]
    stoi = tk.build([p + a for p, a in data[:200000]], "vocab.json")
    itos = json.load(open("vocab.json", encoding="utf-8"))
    model = Rachel(len(itos), d=DIM, layers=LAYER, heads=HEAD, ctx=CTX).to(DEV)
    print(f"장치 {DEV} · 어휘 {len(itos)} · d{DIM} L{LAYER} H{HEAD} ctx{CTX}")
    print(f"파라미터 {model.n_params()/1e6:.2f}M · int8 약 {model.n_params()/1e6:.1f}MB")
    print(f"학습 {len(train):,} / 검증 {len(val):,} · 배치 {BATCH} · {EPOCH}에폭", flush=True)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=0.01)
    steps = EPOCH * math.ceil(len(train) / BATCH)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, LR, total_steps=steps, pct_start=0.05)
    best = -1; step = 0; t0 = time.time(); run = 0.0
    def save(tag, vl, ex):
        torch.save({"model": model.state_dict(), "vocab": itos, "dim": DIM, "layers": LAYER,
                    "heads": HEAD, "ctx": CTX, "step": step, "val_loss": vl, "exact": ex}, OUT if tag == "best" else OUT.replace(".pt", "_last.pt"))
    for ep in range(1, EPOCH + 1):
        for X, M in batches(train, stoi, BATCH):
            loss = run_loss(model, X, M)
            if loss is None: continue
            opt.zero_grad(set_to_none=True); loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step(); sched.step(); step += 1
            run = 0.98 * run + 0.02 * loss.item() if step > 1 else loss.item()
            if step % 200 == 0:
                el = time.time() - t0
                print(f"ep{ep} {step}/{steps} 손실 {run:.4f} ({el/60:.1f}분 · 남은 약 {el/step*(steps-step)/60:.0f}분)", flush=True)
            if step % SAVE_EVERY == 0 or step == steps:
                vl, ex = evaluate(model, val, stoi, itos)
                save("last", vl, ex)
                mark = ""
                if ex > best or (ex == best and vl < 1e9):
                    best = ex; save("best", vl, ex); mark = f"  ★ 저장 {OUT}"
                print(f"== {step}걸음 검증 손실 {vl:.4f} · 글자까지 같음 {ex*100:.1f}%{mark}", flush=True)
    print(f"끝. 최고 글자까지 같음 {best*100:.1f}% · 총 {(time.time()-t0)/60:.0f}분")

if __name__ == "__main__":
    main()
