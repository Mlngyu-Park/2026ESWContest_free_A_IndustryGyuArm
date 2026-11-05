#!/usr/bin/env python3
"""경유 시험지 채점 요약 (10차). python via_sum.py 결과.json [-v]"""
import json, sys
from collections import Counter, defaultdict
rows = json.load(open(sys.argv[1], encoding="utf-8"))
by = defaultdict(Counter)
for r in rows:
    for k in ("전체", r["factor"], r["form"]):
        by[k][r["final"]] += 1; by[k]["n"] += 1
def line(c): return f"맞음 {c['맞음']}/{c['n']}  틀린 채 실행 {c['틀린 채 실행']}  거부 {c['거부']}"
for k in ["전체", "경유", "그대로"] + sorted(x for x in by if x not in ("전체", "경유", "그대로")):
    if k in by: print(f"  {k:28s} {line(by[k])}")
if "-v" in sys.argv:
    for r in rows:
        if r["final"] != "맞음": print(f"[{r['final']}] {r['form']} · {r['text']}\n    정답 {r['canon']}\n    출력 {r['out'][:150]!r}  {r['why']}")
