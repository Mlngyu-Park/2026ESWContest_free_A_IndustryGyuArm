#!/usr/bin/env python3
"""더블 레이첼 체크포인트 → C++ 엔진(rachel_cpp)이 읽는 RCHL 바이너리.

nano_norm/model.py 는 어텐션을 qkv/proj 로 직접 짰고, C++ 엔진은 옛 이름
(attn.in_proj_weight / attn.out_proj) 을 읽는다. 행 순서(q,k,v)는 같으므로 이름만 바꾼다.

  python export_double.py double.pt double.bin
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rachel_cpp"))
import torch
import export_weights as E

REN = {".qkv.weight": ".attn.in_proj_weight", ".qkv.bias": ".attn.in_proj_bias",
       ".proj.weight": ".attn.out_proj.weight", ".proj.bias": ".attn.out_proj.bias"}

def main(src, dst):
    ck = torch.load(src, map_location="cpu")
    sd = {}
    for k, v in ck["model"].items():
        if k == "head.weight": continue                     # tok.weight 와 공유
        for a, b in REN.items():
            if k.endswith(a): k = k[: -len(a)] + b
        sd[k] = v
    tmp = dst + ".tmp.pt"
    torch.save({**ck, "model": sd}, tmp)
    E.main(tmp, dst); os.remove(tmp)

if __name__ == "__main__":
    main(*sys.argv[1:3])
