#!/usr/bin/env python3
"""dusk 정수판 정확도 검증 — C++ 정답과 대조.
   사용: python3 verify_dusk.py [개수]"""
import json, subprocess, sys, time
N = int(sys.argv[1]) if len(sys.argv) > 1 else 20
W = sys.argv[2] if len(sys.argv) > 2 else "/home/drboom/2026_project/Industry_gyu_arm/rachel_cpp/rachel_v9.bin"  # 기본 v9 (3단계 100%)
rows = [json.loads(l) for l in open("/home/drboom/2026_project/Industry_gyu_arm/nano_llm/val.jsonl", encoding="utf-8")][:N]
ok = 0; t0 = time.time(); bad = []
for i, d in enumerate(rows):
    prompt = f"{d['scene']}\n{d['text']}\n>"
    r = subprocess.run(["./dusk", "--weights", W,
                        "--ask", prompt], capture_output=True, timeout=300)
    out = r.stdout.decode("utf-8", "replace")
    got = ""
    for line in out.splitlines():
        if line.startswith("출력: "):
            got = line[4:].strip(); break
    exp = d["answer"].strip()
    if got == exp: ok += 1
    elif len(bad) < 3: bad.append((d["text"][:30], exp[:60], got[:60]))
    if (i+1) % 5 == 0:
        print(f"  {i+1}/{len(rows)} · 맞음 {ok}", flush=True)
el = time.time() - t0
print(f"\ndusk 정수판 정확도 {ok}/{len(rows)} = {ok/len(rows):.1%}  ({el/len(rows):.1f}초/문장)")
for t, e, g in bad:
    print(f"\n  틀림: {t}\n    정답: {e}\n    dusk: {g}")
