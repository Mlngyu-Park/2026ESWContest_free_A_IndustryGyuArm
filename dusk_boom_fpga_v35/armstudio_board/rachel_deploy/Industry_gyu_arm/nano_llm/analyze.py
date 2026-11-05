#!/usr/bin/env python3
"""오답 분석 — 어떤 종류의 명령에서 틀리는가."""
import json, sys, torch
from model import Rachel
import tokenizer as tk

def _mk(ck, itos):
    """체크포인트에 저장된 구조로 모델을 만든다.
    문맥·차원을 하드코딩하면 판이 바뀔 때마다 깨진다 (실제로 깨졌다)."""
    return Rachel(len(itos),
                  d=ck.get("dim", 320), layers=ck.get("layers", 8),
                  heads=ck.get("heads", 8), ctx=ck.get("ctx", 256))


DEV = "cuda" if torch.cuda.is_available() else "cpu"
ck = torch.load(sys.argv[1] if len(sys.argv) > 1 else "rachel.pt", map_location=DEV)   # 인자로 판 지정 가능
itos = ck["vocab"]; stoi = {c: i for i, c in enumerate(itos)}
m = _mk(ck, itos).to(DEV); m.load_state_dict(ck["model"]); m.eval()

COLORS = ["빨간","빨강","주황","노란","노랑","초록","파란","파랑","보라"]
def jpart(a):
    """답안지(&...) 줄을 떼고 JSON 만."""
    return a[a.find("{"):]

def cat(d):
    nb = json.loads(jpart(d["answer"])).get("blocks", [])
    n = len(nb)
    if n == 7: return "2단계 이송"
    if n == 4: return "잡아서 옮기기"
    if n == 2: return "잡기/놓기"
    if n == 1: return "제자리 동작"
    a = json.loads(jpart(d["answer"]))
    if len(a) > 1: return "조작(정지·속도 등)"
    return "거절(빈 블록)"

def ask(scene, text):
    ids = torch.tensor([tk.encode(stoi, f"{scene}\n{text}\n>")], device=DEV)
    out = m.generate(ids, tk.EOS)
    return tk.decode(itos, out[0, ids.size(1):].tolist()).strip()

rows = [json.loads(l) for l in open("val.jsonl", encoding="utf-8")][:400]
stat = {}; wrong = []
for d in rows:
    got = ask(d["scene"], d["text"])
    ok = got == d["answer"].strip()
    c = cat(d)
    col = "색지칭" if any(w in d["text"] for w in COLORS) else "번호지칭"
    for key in (c, col):
        s = stat.setdefault(key, [0, 0]); s[1] += 1; s[0] += ok
    if not ok and len(wrong) < 12:
        wrong.append((c, d["text"], d["answer"], got))

print("== 종류별 정확도 ==")
for k, (o, t) in sorted(stat.items(), key=lambda x: x[1][0]/x[1][1]):
    print(f"  {o:3d}/{t:3d} ({o/t:5.1%})  {k}")
print("\n== 오답 예시 ==")
for c, t, a, g in wrong:
    print(f"[{c}] {t}\n  정답: {a}\n  생성: {g}")
