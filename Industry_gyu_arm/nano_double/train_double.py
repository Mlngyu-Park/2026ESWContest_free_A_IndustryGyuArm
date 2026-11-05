#!/usr/bin/env python3
"""더블 레이첼 학습 — 장면 + 지저분한 말 → 정리문 + 블록.

시퀀스:  <장면>\n<말>\n><정리문>\n=<블록><eos>   (손실은 '>' 뒤에만)
말 정리기(nano_norm/train_norm.py)를 바탕으로:
  · 말이 길면 '말의 앞부분' 을 자른다 — 장면 줄은 절대 자르지 않는다
  · 검증: 블록까지 글자 그대로 같은 비율

  DBL_DIM=448 DBL_LAYER=10 DBL_CTX=768 python train_double.py
"""
import json, math, os, random, sys, time
import torch
import torch.nn.functional as F
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "nano_norm"))
import tok_norm as tk
from model import Rachel

DEV   = "cuda" if torch.cuda.is_available() else "cpu"
DIM   = int(os.environ.get("DBL_DIM", 448))
LAYER = int(os.environ.get("DBL_LAYER", 10))
HEAD  = int(os.environ.get("DBL_HEAD", 8))
CTX   = int(os.environ.get("DBL_CTX", 768))
DATA  = os.environ.get("DBL_DATA", "double_data.jsonl")
OUT   = os.environ.get("DBL_OUT", "double.pt")
BATCH = int(os.environ.get("DBL_BATCH", 128))
EPOCH = int(os.environ.get("DBL_EPOCH", 2))
LR    = float(os.environ.get("DBL_LR", 4e-4))
SAVE_EVERY = int(os.environ.get("DBL_SAVE_EVERY", 2000))
VAL_N = 3000
EXACT_N = int(os.environ.get("DBL_EXACT_N", 150))
ANS_ROOM = 420                                     # 답 자리 (정리문+블록 최대 391자)

def prompt_of(scene, text): return f"{scene}\n{text}\n>"
def answer_of(canon, blocks): return f"{canon}\n={blocks}"

def load(name):
    out = []
    for line in open(name, encoding="utf-8"):
        d = json.loads(line)
        out.append((d["scene"], d["text"], answer_of(d["canon"], d["blocks"])))
    return out

def encode_prompt(stoi, scene, text):
    s = tk.encode(stoi, scene + "\n"); t = tk.encode(stoi, text + "\n>")
    room = CTX - ANS_ROOM - len(s)
    if len(t) > room: t = t[-room:]                # 말의 앞부분을 자른다 (장면은 그대로)
    return s + t

def pack(stoi, scene, text, answer):
    ids = encode_prompt(stoi, scene, text)
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
def generate(model, stoi, itos, scene, text, limit=ANS_ROOM, temp=0.0, n=1):
    """n 개를 한 번에 (다수결용). temp=0 이면 가장 확률 높은 글자."""
    ids = encode_prompt(stoi, scene, text)
    x = torch.tensor([ids] * n, device=DEV)
    out = [[] for _ in range(n)]; done = [False] * n
    for _ in range(limit):
        with torch.autocast(device_type="cuda", dtype=torch.bfloat16, enabled=(DEV == "cuda")):
            lg = model(x[:, -CTX:])[:, -1].float()
        if temp > 0: t = torch.multinomial(torch.softmax(lg / temp, -1), 1)[:, 0]
        else: t = lg.argmax(-1)
        for i, v in enumerate(t.tolist()):
            if done[i]: continue
            if v == tk.EOS: done[i] = True
            else: out[i].append(v)
        if all(done): break
        x = torch.cat([x, t[:, None]], 1)
    return [tk.decode(itos, o) for o in out]

def evaluate(model, val, stoi, itos):
    model.eval(); tot = n = 0
    for X, M in batches(val, stoi, BATCH, shuffle=False):
        l = run_loss(model, X, M)
        if l is not None: tot += l.item(); n += 1
    ok = sum(generate(model, stoi, itos, s, t)[0] == a for s, t, a in val[:EXACT_N])
    model.train()
    return tot / max(n, 1), ok / EXACT_N

def main():
    data = load(DATA)
    random.seed(7); random.shuffle(data)
    val, train = data[:VAL_N], data[VAL_N:]
    INIT = os.environ.get("DBL_INIT")              # 이전 가중치에서 이어 학습 (글자 사전도 그대로)
    if INIT:
        ck0 = torch.load(INIT, map_location="cpu"); itos = ck0["vocab"]
        stoi = {c: i for i, c in enumerate(itos)}
        json.dump(itos, open("vocab.json", "w", encoding="utf-8"), ensure_ascii=False)
    else:
        stoi = tk.build([s + t + a for s, t, a in data[:200000]], "vocab.json")
        itos = json.load(open("vocab.json", encoding="utf-8"))
    model = Rachel(len(itos), d=DIM, layers=LAYER, heads=HEAD, ctx=CTX).to(DEV)
    if INIT:
        model.load_state_dict(ck0["model"]); print(f"이어 학습: {INIT} ({ck0.get('step')}걸음)")
    print(f"장치 {DEV} · 어휘 {len(itos)} · d{DIM} L{LAYER} H{HEAD} ctx{CTX}")
    print(f"파라미터 {model.n_params()/1e6:.2f}M · int8 약 {model.n_params()/1e6:.1f}MB")
    print(f"학습 {len(train):,} / 검증 {len(val):,} · 배치 {BATCH} · {EPOCH}에폭", flush=True)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=0.01)
    steps = EPOCH * math.ceil(len(train) / BATCH)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, LR, total_steps=steps, pct_start=0.05)
    best = -1; step = 0; t0 = time.time(); run = 0.0
    def save(tag, vl, ex):
        torch.save({"model": model.state_dict(), "vocab": itos, "dim": DIM, "layers": LAYER,
                    "heads": HEAD, "ctx": CTX, "step": step, "val_loss": vl, "exact": ex},
                   OUT if tag == "best" else OUT.replace(".pt", "_last.pt"))
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
                if ex >= best:
                    best = ex; save("best", vl, ex); mark = f"  ★ 저장 {OUT}"
                print(f"== {step}걸음 검증 손실 {vl:.4f} · 블록까지 같음 {ex*100:.1f}%{mark}", flush=True)
    print(f"끝. 최고 {best*100:.1f}% · 총 {(time.time()-t0)/60:.0f}분")

if __name__ == "__main__":
    main()
