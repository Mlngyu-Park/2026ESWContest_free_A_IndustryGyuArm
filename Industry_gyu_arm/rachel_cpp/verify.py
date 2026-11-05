#!/usr/bin/env python3
"""C++ 추론기 정확도 검증 — 파이썬 결과와 글자 단위 대조."""
import json, subprocess, sys, time
N = int(sys.argv[1]) if len(sys.argv) > 1 else 200
rows = [json.loads(l) for l in open("../nano_llm/val.jsonl", encoding="utf-8")][:N]
ok = mismatch = 0; t0 = time.time()
bad = []
for d in rows:
    prompt = f"{d['scene']}\n{d['text']}\n>"
    r = subprocess.run(["./rachel", "rachel.bin", prompt],
                       capture_output=True, timeout=60)
    got = r.stdout.decode("utf-8", "replace").strip()
    exp = d["answer"].strip()
    if got == exp: ok += 1
    else:
        mismatch += 1
        if len(bad) < 5: bad.append((d["text"][:40], exp[:80], got[:80]))
el = time.time() - t0
print(f"C++ 정확도 {ok}/{len(rows)} = {ok/len(rows):.1%}  (평균 {el/len(rows)*1000:.0f}ms)")
for t, e, g in bad:
    print(f"\n  틀림: {t}\n    정답: {e}\n    C++ : {g}")
