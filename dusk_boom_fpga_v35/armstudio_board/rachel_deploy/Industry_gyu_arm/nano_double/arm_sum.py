#!/usr/bin/env python3
"""팔 이름 시험지(grid_eval_arm.json) 채점 결과 요약 (11-03).

  python arm_sum.py gpu_arm_모델.json   → 꼴별 · 학습함/8차/처음 별 맞음 · 틀린 채 실행 · 거부
"""
import json, sys
from collections import Counter, defaultdict

rows = json.load(open(sys.argv[1], encoding="utf-8"))
by = defaultdict(Counter)
for r in rows:
    grp = r["form"].split("(")[-1].rstrip(")") if "(" in r["form"] else r["factor"]
    for k in (("전체", ""), ("묶음", grp), ("팔 꼴", r["mode"]), (r["factor"], r["form"])):
        by[k][r["final"]] += 1; by[k]["n"] += 1
def line(c): return f"맞음 {c['맞음']}/{c['n']}  틀린 채 실행 {c['틀린 채 실행']}  거부 {c['거부']}"
for f in ("전체", "묶음", "팔 꼴", "팔꼴", "조사", "반례", "실제", "비낱말"):
    print(f"── {f}")
    for (ff, form), c in sorted(by.items(), key=lambda kv: kv[1]["맞음"] / kv[1]["n"]):
        if ff == f: print(f"  {form:20s} {line(c)}")
if "-v" in sys.argv:
    for r in rows:
        if r["final"] != "맞음": print(f"[{r['final']}] {r['form']} · {r['text']}\n    정답 {r['canon']}\n    출력 {r['out'][:150]!r}  {r['why']}")
