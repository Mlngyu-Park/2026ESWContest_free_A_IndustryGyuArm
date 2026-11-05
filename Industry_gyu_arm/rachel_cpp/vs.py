#!/usr/bin/env python3
"""상주 모드 검증 — 프로세스를 한 번만 띄우고 문장만 주고받는다.
   매번 새로 실행하면 가중치 9.6MB 로딩이 문장마다 붙어 780ms 손해다.
   사용: python3 vs.py <개수> <실행파일> [ENV=1 ...]"""
import json, subprocess, sys, os, time

N   = int(sys.argv[1])
EXE = sys.argv[2]
env = dict(os.environ)
for kv in sys.argv[3:]:
    k, v = kv.split('='); env[k] = v

BIN = "/home/drboom/2026_project/Industry_gyu_arm/rachel_cpp/rachel.bin"
VAL = "/home/drboom/2026_project/Industry_gyu_arm/nano_llm/val.jsonl"
rows = [json.loads(l) for l in open(VAL, encoding="utf-8")][:N]

p = subprocess.Popen([EXE, BIN, "--serve"], stdin=subprocess.PIPE,
                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, env=env)
assert p.stdout.readline().strip() == b"READY", "상주 모드 시작 실패"

ok = 0; bad = []; t0 = time.time()
for d in rows:
    # 줄바꿈은 \x01 로 보낸다 (한 줄에 프롬프트 전체)
    prompt = f"{d['scene']}\n{d['text']}\n>".replace("\n", "\x01")
    p.stdin.write(prompt.encode() + b"\n"); p.stdin.flush()
    got = p.stdout.readline().decode("utf-8", "replace").strip()
    exp = d["answer"].strip()
    if got == exp: ok += 1
    elif len(bad) < 3: bad.append((d["text"][:30], exp[:55], got[:55]))
p.stdin.write(b"QUIT\n"); p.stdin.flush(); p.wait()

el = time.time() - t0
print(f"{ok}/{len(rows)} = {ok/len(rows):.1%}   ({el:.1f}초 · {el/len(rows)*1000:.0f}ms/문장)")
for t, e, g in bad:
    print(f"\n  틀림: {t}\n    정답: {e}\n    결과: {g}")
