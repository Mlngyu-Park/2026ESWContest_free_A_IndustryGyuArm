#!/usr/bin/env python3
"""
8x8 부호있는 곱셈 LUT — 플래시 2개로 쪼개기 (새 PE 모양)

왜 2개인가:
  곱셈 결과는 16비트인데 플래시 출력은 8비트다.
  1개를 쓰면 하위/상위를 2번 읽어야 해서 PE 클럭이 절반이 된다.
  2개를 나란히 두고 같은 주소를 주면 16비트가 한 번에 나온다.  → 2배 빠름

  LO 칩: 곱의 하위 8비트
  HI 칩: 곱의 상위 8비트

주소 (양쪽 칩 공통, A16~A18 은 GND):
  A0~A7  = x (int8, 2의 보수)
  A8~A15 = w (int8, 2의 보수)

★ 옛 방식과 달리 A16 을 위상 선택에 쓰지 않는다.
  SST39SF040(512KB)은 A0~A18 이 있으므로 A16·A17·A18 을 전부 GND 로 내린다.

출력:
  LO 칩 DQ0~DQ7 = 곱의 비트 0~7
  HI 칩 DQ0~DQ7 = 곱의 비트 8~15

쓰는 법:
    python3 gen_lut2.py          # mul_lo.bin, mul_hi.bin 생성 + 전수 검증
"""
import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIZE = 65536          # 주소선 16개(A0~A15)만 쓴다


def build():
    lo = bytearray(SIZE)
    hi = bytearray(SIZE)
    for w in range(256):
        ws = w - 256 if w >= 128 else w
        for x in range(256):
            xs = x - 256 if x >= 128 else x
            p16 = (ws * xs) & 0xFFFF      # 16비트 2의 보수
            addr = (w << 8) | x
            lo[addr] = p16 & 0xFF
            hi[addr] = (p16 >> 8) & 0xFF
    return lo, hi


def verify(lo, hi):
    """65,536 조합 전수 검사"""
    bad = 0
    for w in range(-128, 128):
        for x in range(-128, 128):
            a = ((w & 0xFF) << 8) | (x & 0xFF)
            v = (hi[a] << 8) | lo[a]
            if v >= 32768:
                v -= 65536
            if v != w * x:
                if bad < 5:
                    print(f"  불일치: {w} x {x} = {v} (정답 {w*x})")
                bad += 1
    return bad


def main():
    lo, hi = build()
    print(f"생성: LO {len(lo):,}바이트, HI {len(hi):,}바이트")

    bad = verify(lo, hi)
    if bad:
        print(f"!!! {bad}개 불일치 !!!")
        return 1
    print("전수 검증 통과: 65,536 조합 전부 실제 곱과 일치")

    # 곱의 범위 확인
    vals = []
    for w in (-128, -1, 0, 1, 127):
        for x in (-128, -1, 0, 1, 127):
            vals.append(w * x)
    print(f"곱 범위: {min(vals)} ~ {max(vals)}  (16비트 안에 들어감)")

    for name, buf in (('mul_lo.bin', lo), ('mul_hi.bin', hi)):
        p = os.path.join(HERE, name)
        open(p, 'wb').write(buf)
        print(f"  {p}")

    print("\n굽는 법: MEGA 로 mul_lo.bin 을 LO 칩에, mul_hi.bin 을 HI 칩에")
    print("512KB 플래시의 앞 64KB 만 쓴다. A16·A17·A18 은 GND.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
