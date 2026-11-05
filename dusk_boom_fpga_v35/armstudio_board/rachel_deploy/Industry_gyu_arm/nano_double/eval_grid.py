#!/usr/bin/env python3
"""말씨 격자 채점 — grid_eval.json 을 C++ 엔진(CPU)으로, 앱과 같은 '검사+보충' 방식으로.

  python eval_grid.py 모델.bin [격자.json] [일꾼=16]  → 말씨별·팔 꼴별 맞음 / 틀린 채 실행 / 거부
"""
import json, os, sys
from collections import Counter, defaultdict
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import eval_cpp as E
from canon import parse_scene, to_blocks

def main():
    E.W = sys.argv[1]; E.N = 6; E.TEMP = 0.7
    gp = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "grid_eval.json")
    nw = int(sys.argv[3]) if len(sys.argv) > 3 else 16
    items = []
    for i in json.load(open(gp, encoding="utf-8"))["items"]:
        sc = parse_scene(i["scene"])
        items.append(dict(i, sc=sc, gold=to_blocks(i["canon"], sc), set="격자", kind=i["mode"]))
    with Pool(nw) as p: res = [r for part in p.map(E.work, [(k, items[k::nw]) for k in range(nw)]) for r in part]
    by = defaultdict(Counter); rows = []
    for r in res:
        a, b, c, why, votes, d, e = E.judge(r)
        for key in (("팔 꼴", r["mode"]), (r["factor"], r["form"]), ("전체", "")):
            by[key][e] += 1; by[key]["n"] += 1
        rows.append(dict(text=r["text"], canon=r["canon"], out=r["greedy"], once=a, final=e, why=why,
                         factor=r["factor"], form=r["form"], mode=r["mode"]))
    name = {"G": "잡기 말씨", "M": "옮기기 동사", "C": "연결 어미", "E": "맺음 어미"}
    for f in ("전체", "팔 꼴", "G", "M", "C", "E"):
        print(f"── {name.get(f, f)}")
        for (ff, form), c in sorted(by.items(), key=lambda kv: kv[1]["맞음"] / kv[1]["n"]):
            if ff != f: continue
            print(f"  {form:14s} 맞음 {c['맞음']}/{c['n']}  틀린 채 실행 {c['틀린 채 실행']}  거부 {c['거부']}")
    out = os.environ.get("EVAL_OUT") or os.path.join(HERE, "grid_out.json")
    json.dump(rows, open(out, "w"), ensure_ascii=False, indent=1)

if __name__ == "__main__":
    main()
