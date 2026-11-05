#!/usr/bin/env python3
"""다수결 곡선 — 뽑는 개수(1·2·4·8·16·32)에 따라 정확도가 어떻게 바뀌나.

  python vote_curve.py double.bin [최대=32] [온도=0.7] [일꾼=16]

문장마다 '가장 큰 값' 1개 + 온도 뽑기 최대 N개를 한 번에 만들어 두고, 앞에서 n개만 써서 다수결한다
(같은 뽑기를 다시 쓰므로 n 끼리 공정하게 비교된다). 검사(check.py)를 통과한 답만 표를 얻는다.
"""
import json, os, sys, time
from collections import Counter
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import eval_cpp as E
from check import check
from eval_double import real_items, hand_items

def vote(r, n):
    votes = Counter()
    for o in [r["greedy"]] + r["many"][:n]:
        ok, _, b = check(r["text"], o, r["sc"])
        if ok: votes[" ".join(b)] += 1
    if not votes: return "거부"
    best = votes.most_common(1)[0][0]
    return "맞음" if (best.split() if best else []) == r["gold"] else "틀린 채 실행"

def main():
    E.W = sys.argv[1]; E.N = int(sys.argv[2]) if len(sys.argv) > 2 else 32
    E.TEMP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.7
    nw = int(sys.argv[4]) if len(sys.argv) > 4 else 16
    items = [dict(i, set="실제 말") for i in real_items()] + [dict(i, set="두 팔(손)") for i in hand_items()]
    t0 = time.time()
    with Pool(nw) as p: res = [r for part in p.map(E.work, [(k, items[k::nw]) for k in range(nw)]) for r in part]
    print(f"{E.W} · 최대 {E.N}개 · 온도 {E.TEMP} · {len(res)}문장 · {time.time()-t0:.0f}초")
    groups = {"실제 말 이송": [r for r in res if r["set"] == "실제 말" and r["kind"] == "이송"],
              "실제 말 명령아님": [r for r in res if r["set"] == "실제 말" and r["kind"] != "이송"],
              "두 팔 (손 60)": [r for r in res if r["set"] != "실제 말"]}
    ns = [n for n in (0, 1, 2, 4, 8, 16, 32) if n <= E.N]
    print(f"  {'':16s} " + " ".join(f"{('한번' if n == 0 else f'{n+1}개'):>12s}" for n in ns))
    out = {}
    for g, rs in groups.items():
        row = []
        for n in ns:
            c = Counter(vote(r, n) for r in rs)
            row.append(f"{c['맞음']:3d}/{len(rs)} 틀{c['틀린 채 실행']:2d}"); out.setdefault(g, {})[n + 1] = dict(c)
        print(f"  {g:16s} " + " ".join(f"{x:>12s}" for x in row))
    print("  (n개 = 가장 큰 값 1 + 온도 뽑기 n-1 · '틀' = 틀린 채 실행)")
    json.dump(out, open(os.path.join(HERE, "vote_curve_out.json"), "w"), ensure_ascii=False, indent=1)

if __name__ == "__main__":
    main()
