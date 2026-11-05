#!/usr/bin/env python3
"""더블 레이첼 채점 — C++ 엔진(CPU)으로. 앱이 실제로 쓰는 길 그대로 잰다.

  python eval_cpp.py double.bin [뽑는수=8] [온도=0.7] [일꾼=16]

eval_double.py 와 같은 문장·같은 세 방식 (한번 / 한번+검사 / 다수결).
일꾼마다 엔진 둘(가장 큰 값 하나, 온도 뽑기 하나)을 띄워 문장을 나눠 맡는다.
"""
import json, os, sys, time
from collections import Counter
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); APP = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from canon import scene_str, resolve, squash
from check import check, split_out
from rachel2 import Rachel2
from eval_double import real_items, hand_items

ENGINE = os.path.join(APP, "rachel_cpp", "rachel")
# 채점은 엔진 여러 개를 동시에 돌린다 → 엔진마다 스레드 1개 (엔진 스레드 나눔, 10-31)
os.environ.setdefault("RACHEL_THREADS", "1")
W = N = TEMP = None

def work(args):
    wid, items = args
    g = Rachel2(ENGINE, W); s = Rachel2(ENGINE, W, temp=TEMP, seed=wid + 1) if N else None
    out = []
    for it in items:
        sc = scene_str(it["sc"]); t0 = time.time()
        gr = g.raw(sc, it["text"]); tg = time.time() - t0
        many = [s.raw(sc, it["text"]) for _ in range(N)] if N else []
        out.append(dict(it, greedy=gr, many=many, t=tg))
    g.close()
    if s: s.close()
    return out

def judge(r):
    gold = r["gold"]
    v = lambda b: "맞음" if b is not None and squash(b) == squash(gold) else "틀린 채 실행"   # 동작이 같으면 맞음
    _, gb = split_out(r["greedy"])
    try: gb = resolve(gb, r["sc"]) if gb is not None else None     # 이름 블록 → 번호 (계산)
    except Exception: gb = None
    a = v(gb)
    ok, why, cb = check(r["text"], r["greedy"], r["sc"])
    b = v(cb) if ok else "거부"
    votes = Counter()
    for o in [r["greedy"]] + r["many"]:
        ok2, _, b2 = check(r["text"], o, r["sc"])
        if ok2: votes[" ".join(b2)] += 1
    if votes:
        best = votes.most_common(1)[0][0]; c = v(best.split() if best else [])
    else: c = "거부"
    # 정리문만: 모델의 블록 줄은 보지 않고, 정리문을 계산으로 블록으로 바꾼다 (검사 ①③④⑤만)
    c0, _ = split_out(r["greedy"])
    ok3, why3, cb3 = check(r["text"], c0, r["sc"])
    d = v(cb3) if ok3 else "거부"
    e = b if b != "거부" else c                 # 검사+보충: 통과하면 한번 답, 걸리면 다수결 (앱이 쓰는 방식)
    return a, b, c, why, votes, d, e

def main():
    global W, N, TEMP
    W = sys.argv[1]; N = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    TEMP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.7
    nw = int(sys.argv[4]) if len(sys.argv) > 4 else 16
    items = [dict(i, set="실제 말") for i in real_items()] + [dict(i, set="두 팔(손)") for i in hand_items()]
    only = os.environ.get("EVAL_ONLY")                    # 예: EVAL_ONLY=세트3 → 그 세트만
    if only: items = [i for i in items if i["kind"].startswith(only)]
    t0 = time.time()
    chunks = [(k, items[k::nw]) for k in range(nw)]
    with Pool(nw) as p: res = [r for part in p.map(work, chunks) for r in part]
    print(f"{W} · 뽑기 {N}개 온도 {TEMP} · {len(res)}문장 · {time.time()-t0:.0f}초 · "
          f"한 문장 평균 {sum(r['t'] for r in res)/len(res)*1000:.0f}ms (한번)")
    groups = {}
    for r in res: groups.setdefault((r["set"], r["kind"]), []).append(r)
    for (st, k), rs in sorted(groups.items()):
        J = [judge(r) for r in rs]
        for mi, name in ((0, "한번"), (1, "한번+검사"), (6, "검사+보충"), (2, "다수결")):
            c = Counter(j[mi] for j in J); n = len(rs)
            print(f"  {st}·{k:8s} {name:8s} 맞음 {c['맞음']:3d}/{n} ({c['맞음']/n*100:4.0f}%) · "
                  f"틀린 채 실행 {c['틀린 채 실행']:3d} · 거부 {c['거부']:3d}")
    print("\n한번에 틀린 것:")
    rows = []
    for r in res:
        a, b, c, why, votes, d, e = judge(r)
        canon, _ = split_out(r["greedy"])
        rows.append(dict(set=r["set"], kind=r["kind"], text=r["text"], greedy=r["greedy"], once=a, check=b, vote=c,
                         why=why, gold=" ".join(r["gold"]), canon_gold=r.get("canon")))
        if a != "맞음":
            print(f"  [{r['set']}·{r['kind']}] 다수결:{c} 검사:{b} | {r['text'][:70]}\n      → {canon[:120]}  {why[:50]}")
    json.dump(rows, open(os.environ.get("EVAL_OUT") or os.path.join(HERE, "eval_cpp_out.json"), "w"), ensure_ascii=False, indent=1)

if __name__ == "__main__":
    main()
