#!/usr/bin/env python3
"""RACHEL 체크포인트 -> C++/Z80 이 읽는 단순 바이너리.

형식 (little-endian):
  magic "RCHL"  u32
  version       u32 = 1
  dim, layers, heads, ctx, vocab   u32 × 5
  scale (fp32)  각 텐서마다 1개 — int8 = round(w / scale)
  텐서들        이름길이 u16 + 이름 + 원소수 u32 + int8 데이터

파이썬·torch 없이 읽을 수 있게 일부러 단순하게 만들었다.
Z80 은 여기서 뽑은 int8 을 그대로 ROM 에 굽는다.
"""
import struct, sys, torch

def q8(t):
    """대칭 int8 양자화: scale = max|w| / 127"""
    s = float(t.abs().max()) / 127.0 or 1.0
    return (t / s).round().clamp(-127, 127).to(torch.int8), s

def main(src="rachel.pt", dst="rachel.bin"):
    ck = torch.load(src, map_location="cpu")
    sd = ck["model"]; itos = ck["vocab"]
    d   = ck.get("dim", 320);   L = ck.get("layers", 8)
    h   = ck.get("heads", 8);   c = ck.get("ctx", 256)
    out = open(dst, "wb")
    out.write(b"RCHL" + struct.pack("<6I", 1, d, L, h, c, len(itos)))
    # 어휘 (UTF-8)
    # 줄바꿈으로 잇는데 어휘 안에 '\n' 자체가 있어 파싱이 한 칸 밀렸다.
    # -> 길이 접두어 방식으로 바꾼다 (글자마다 u8 길이 + UTF-8 바이트)
    vb = b"".join(bytes([len(c.encode())]) + c.encode() for c in itos)
    out.write(struct.pack("<I", len(vb))); out.write(vb)
    n_t = 0; n_b = 0
    body = bytearray()
    for k, v in sd.items():
        if v.dtype not in (torch.float32, torch.float16, torch.bfloat16):
            continue                        # int8 체크포인트는 원본에서 뽑을 것
        qt, s = q8(v.float())
        nm = k.encode()
        body += struct.pack("<H", len(nm)) + nm
        body += struct.pack("<If", qt.numel(), s)
        body += qt.numpy().tobytes()
        n_t += 1; n_b += qt.numel()
    out.write(struct.pack("<I", n_t)); out.write(body); out.close()
    print(f"{src} -> {dst}")
    print(f"  d{d} L{L} H{h} ctx{c} vocab{len(itos)}")
    print(f"  텐서 {n_t}개 · 가중치 {n_b:,} 바이트 ({n_b/1024:.0f} KB)")

if __name__ == "__main__":
    main(*sys.argv[1:3] if len(sys.argv) > 2 else ["rachel.pt", "rachel.bin"])
