#!/usr/bin/env python3
"""
8x8 부호있는 곱셈 LUT 생성 — SST39SF010 (128KB) 용

주소  A0~A7  = x (int8, 2의 보수)
      A8~A15 = w (int8, 2의 보수)
      A16    = 0 하위바이트 / 1 상위바이트
출력  D0~D7  = 곱의 해당 바이트

곱의 범위: -128*127 = -16,256  ~  -128*-128 = 16,384
→ 16비트 2의 보수로 담긴다
"""
import sys

def build():
    out = bytearray(131072)          # 128KB
    for w in range(256):
        ws = w - 256 if w >= 128 else w
        for x in range(256):
            xs = x - 256 if x >= 128 else x
            p16 = (ws * xs) & 0xFFFF
            addr = (w << 8) | x
            out[addr]           = p16 & 0xFF          # A16=0 하위
            out[0x10000 + addr] = (p16 >> 8) & 0xFF   # A16=1 상위
    return out

def verify(out):
    def lut(w, x):
        a = ((w & 0xFF) << 8) | (x & 0xFF)
        v = (out[0x10000 + a] << 8) | out[a]
        return v - 65536 if v >= 32768 else v

    bad = 0
    for w in range(-128, 128):
        for x in range(-128, 128):
            if lut(w, x) != w * x:
                if bad < 5:
                    print(f"  불일치: {w} x {x} = {lut(w,x)} (정답 {w*x})")
                bad += 1
    return bad

if __name__ == '__main__':
    out = build()
    open('mul_lut.bin', 'wb').write(out)
    print(f"생성: mul_lut.bin  {len(out):,} 바이트 (128KB)")
    bad = verify(out)
    if bad == 0:
        print("전수 검증 65,536 조합: 전부 일치 ✓")
    else:
        print(f"전수 검증: {bad}개 불일치 ✗")
        sys.exit(1)
