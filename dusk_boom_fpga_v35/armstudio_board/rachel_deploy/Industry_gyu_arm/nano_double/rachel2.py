#!/usr/bin/env python3
"""더블 레이첼 — 앱에서 부르는 곳 (C++ 엔진 상주 + 검사 + 블록 변환).

  r = Rachel2("rachel_cpp/rachel", "nano_double/double.bin")
  res = r.ask(scene, "빨간 팔이 보라색에 있는 펜을 노랑에 두고 10초 뒤에 파란 팔이 초록으로 옮겨")
  res = {"ok": True, "canon": "...", "blocks": ["arm:A", "move_to:d2", ...], "app": ["@arm:A", "@goto:d2", ...]}
  검사(check.py)에 걸리면 ok=False — 실행하지 않는다.

  python rachel2.py double.bin "장면" "말"      # 한 번 시험
"""
import os, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "nano_norm"))
import tok_norm as tk
from canon import parse_scene, scene_str, to_app, squash
from check import check, split_out

class Rachel2:
    def __init__(self, engine, weights, vocab_path=None, temp=0.0, seed=0):
        """temp>0 이면 매번 조금씩 다르게 뽑는다 (다수결용). seed 로 엔진마다 다르게."""
        env = {**os.environ, "RACHEL_MAXGEN": "440", "RACHEL_TEMP": str(temp), "RACHEL_SEED": str(seed)}
        self.p = subprocess.Popen([engine, weights, "--serve"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, env=env)
        assert self.p.stdout.readline().strip() == b"READY", "엔진 시작 실패"
        self.engine, self.weights = engine, weights
        import json
        vp = vocab_path or os.path.join(os.path.dirname(os.path.abspath(weights)), "vocab.json")
        self.vocab = set(json.load(open(vp, encoding="utf-8")))

    def _fit(self, t):
        """사전에 없는 한글 음절은 낱자모로 (학습 때 tok_norm 과 같게)."""
        out = []
        for ch in t:
            if ch in self.vocab: out.append(ch); continue
            dj = tk._decompose(ch)
            out.append(dj if dj else ch)
        return "".join(out)

    def raw(self, scene, text):
        pr = f"{scene}\n{self._fit(' '.join(text.split()))}\n>"
        self.p.stdin.write(pr.replace("\n", "\x01").encode() + b"\n"); self.p.stdin.flush()
        return self.p.stdout.readline().decode("utf-8", "replace").rstrip("\n").replace("\x01", "\n")

    def ask(self, scene, text, fallback=0):
        """한 번 뽑아 검사. fallback>0 이면 검사에 걸렸을 때만 온도 뽑기 fallback 개를 더 해서
        검사를 통과한 것 중 가장 많이 나온 블록을 쓴다 (다수결 보충, 10-30)."""
        sc = parse_scene(scene) if isinstance(scene, str) else scene
        out = self.raw(scene_str(sc), text)
        ok, why, blocks = check(text, out, sc)
        canon, _ = split_out(out)
        voted = 0
        if not ok and fallback > 0:
            if getattr(self, "_t", None) is None:
                self._t = Rachel2(self.engine, self.weights, temp=0.7, seed=7)
            from collections import Counter
            votes = Counter(); best_out = {}
            for _ in range(fallback):
                o = self._t.raw(scene_str(sc), text)
                ok2, _, b2 = check(text, o, sc)
                if ok2: k = " ".join(b2); votes[k] += 1; best_out.setdefault(k, o)
            if votes:
                k, voted = votes.most_common(1)[0]
                out = best_out[k]; canon, _ = split_out(out); blocks = k.split() if k else []
                ok, why = True, f"다수결 보충 ({voted}/{fallback})"
        if ok: blocks = squash(blocks)               # 같은 팔 다시 고르기 · 방금 놓은 자리로 또 이동 → 뺀다
        return dict(ok=ok, why=why, canon=canon, blocks=blocks, app=to_app(blocks) if ok else None, raw=out,
                    voted=voted)

    def close(self):
        try: self.p.stdin.write(b"QUIT\n"); self.p.stdin.flush(); self.p.wait(timeout=3)
        except Exception: self.p.kill()
        if getattr(self, "_t", None) is not None: self._t.close()

if __name__ == "__main__":
    w, scene, text = sys.argv[1:4]
    r = Rachel2(os.path.join(HERE, "..", "rachel_cpp", "rachel"), w)
    import json, time
    t0 = time.time(); res = r.ask(scene, text)
    print(json.dumps(res, ensure_ascii=False, indent=1)); print(f"{(time.time()-t0)*1000:.0f}ms")
    r.close()
