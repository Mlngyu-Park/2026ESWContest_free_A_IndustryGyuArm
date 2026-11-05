#!/usr/bin/env python3
"""Z80 어레이 성능 시뮬레이터.

mac_z80.asm 의 실제 명령을 세어 얻은 사이클(560T)로,
모델 크기·코어 수를 바꿔가며 문장 하나의 처리 시간을 계산한다.
보드를 사기 전에 "정말 몇 분 걸리는가" 를 확정하기 위한 도구.

  python simulate.py                # 표 출력
  python simulate.py --model 96 4   # 특정 구성만
"""
import sys

# ── 실측 상수 (mac_z80.asm 명령 카운트) ──────────────────────
T_MAC   = 560          # int8 MAC 1회 (부호처리+시프트곱셈+24bit 누산+CALL)
T_BANK  = 49           # 뱅크 스위칭 1회 (OUT + 재설정)
CLK     = 4_000_000    # Z80 4 MHz
BUS_BPS = 100_000      # 마스터↔워커 8비트 병렬 핸드셰이크 실효 (bytes/s)

def model_macs(d, layers, vocab, ctx, out_len):
    """한 문장 생성에 필요한 MAC 총량."""
    # 층당: QKV 3d² + 출력투영 d² + MLP 8d² = 12d², 어텐션 2·ctx·d
    per_tok = layers*(12*d*d + 2*ctx*d) + vocab*d
    return per_tok * out_len

def params(d, layers, vocab, ctx):
    return vocab*d + ctx*d + layers*12*d*d

def sim(d, layers, cores, vocab=160, ctx=128, out_len=58, bank_kb=32):
    macs = model_macs(d, layers, vocab, ctx, out_len)
    p    = params(d, layers, vocab, ctx)
    kb   = p/1024                                    # int8 = 1바이트/파라미터

    # 코어당 담당량
    macs_c = macs / cores
    t_calc = macs_c * T_MAC / CLK

    # 뱅크 스위칭: 코어 메모리(48KB 가용)를 넘으면 갈아끼워야 한다
    per_core_kb = kb / cores
    banks = max(1, int(per_core_kb / bank_kb + 0.999))
    t_bank = (banks-1) * out_len * T_BANK / CLK * (macs_c/1000)  # 근사

    # 통신: 글자마다 입력벡터 뿌리고 결과 취합
    bytes_io = out_len * (d*2 + cores*3)             # 브로드캐스트 + 수집
    t_io = bytes_io / BUS_BPS

    return dict(params=p, kb=kb, banks=banks,
                calc=t_calc, bank=t_bank, io=t_io,
                total=t_calc+t_bank+t_io)

def fmt(s):
    if s >= 3600: return f"{s/3600:.1f}시간"
    if s >= 60:   return f"{s/60:.1f}분"
    return f"{s:.1f}초"

if __name__ == "__main__":
    if "--model" in sys.argv:
        i = sys.argv.index("--model")
        d, L = int(sys.argv[i+1]), int(sys.argv[i+2])
        for c in (1, 10, 20, 40):
            r = sim(d, L, c)
            print(f"{c:3d}코어  계산 {fmt(r['calc']):>8}  통신 {fmt(r['io']):>7}  "
                  f"합계 {fmt(r['total']):>8}")
        sys.exit()

    print(f"MAC 1회 = {T_MAC}T (mac_z80.asm 명령 카운트) · Z80 {CLK/1e6:.0f}MHz")
    print(f"{'구성':<18}{'파라미터':>9}{'KB':>7}{'뱅크':>5}"
          f"{'10코어':>9}{'20코어':>9}{'40코어':>9}")
    print("-"*70)
    for name, d, L in [("RACHEL d320 L8", 320, 8),
                       ("nano-A d128 L4", 128, 4),
                       ("nano-B d96  L4",  96, 4),
                       ("nano-C d64  L4",  64, 4),
                       ("nano-D d64  L3",  64, 3),
                       ("nano-E d48  L3",  48, 3)]:
        ctx = 256 if d == 320 else 128
        r10 = sim(d, L, 10, ctx=ctx); r20 = sim(d, L, 20, ctx=ctx)
        r40 = sim(d, L, 40, ctx=ctx)
        print(f"{name:<18}{r20['params']/1000:>8.0f}K{r20['kb']:>7.0f}"
              f"{r20['banks']:>5}{fmt(r10['total']):>9}"
              f"{fmt(r20['total']):>9}{fmt(r40['total']):>9}")
    print()
    print("* 코어당 48KB 가용 가정 (64KB - ROM/스택/코드)")
    print("* 통신 8비트 병렬 100KB/s 가정 — 실제로는 이보다 느릴 수 있다")
