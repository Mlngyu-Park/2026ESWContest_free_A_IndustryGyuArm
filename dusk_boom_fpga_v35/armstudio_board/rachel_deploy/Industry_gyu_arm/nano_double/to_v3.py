#!/usr/bin/env python3
"""2차 데이터 → 3차: 말·정리문은 그대로, 블록 줄만 이름 블록으로 (python to_v3.py 입력 출력)."""
import json, sys
from canon import to_named
with open(sys.argv[2], "w", encoding="utf-8") as fo:
    for line in open(sys.argv[1], encoding="utf-8"):
        d = json.loads(line); d["blocks"] = " ".join(to_named(d["canon"]))
        fo.write(json.dumps(d, ensure_ascii=False) + "\n")
