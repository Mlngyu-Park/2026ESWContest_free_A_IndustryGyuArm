#!/usr/bin/env python3
"""
TPU 전용 분석 — 보유 부품으로 가능한 배열, 임계 경로, 무엇을 바꾸면 얼마나 빨라지나

    python3 tpu.py                 (pe2.txt + inventory.txt + chips.txt 읽음)

원칙: 여기 숫자는 전부 chips.txt(데이터시트) 와 sol.py 타이밍 분석에서 나온다.
      "빠를 것 같다"는 판단은 없다. 계산과 대안 나열만 한다.
"""
import io, os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sol import parse_design, verify, parse_chips

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_MAC_PER_TOKEN = 10_126_061     # RACHEL v9 (rachel_v9.bin 크기 = int8 가중치 수)
TOKENS_PER_SENT = 130

# PE 하나의 부품 (새 모양)
PE_BOM = {'SST39SF040':2, '74HC574':4, '74HC283':4}
PE_BOM_SAFE = {'SST39SF040':2, '74HC574':4, '74HC283':4, 'RES4k7':16}   # pe2.txt: HC283 + DQ 풀업 16개/PE
COMMON = {'74HC74':1, '74HC244':1, '74HC04':1, 'OSC_FULL':1}

def read_inventory(path):
    inv = {}
    for raw in io.open(path, encoding='utf-8'):
        line = raw.split('#',1)[0].strip()
        if line.startswith('have'):
            _, chip, n = line.split()[:3]; inv[chip] = int(n)
    return inv

def max_pe_bom(inv, bom):
    lim = {}
    for chip, per in bom.items():
        lim[chip] = inv.get(chip, 0) // per
    n = min(lim.values())
    return n, lim
def max_pe(inv): return max_pe_bom(inv, PE_BOM)

def crit_path_ns(chips, adder='74HC283', flash='SST39SF040', carry_stages=2, latch='74HC574'):
    """PE 안의 레지스터→레지스터 최장 경로 (새 모양).
       x래치 Q → 플래시 A→DQ → 가산기 (캐리 단수) → psum 래치 D(setup)"""
    L = chips[latch]; F = chips[flash]; A = chips[adder]
    def path(c, i, o):
        import fnmatch
        best = 0
        for gi, go, d in c.paths:
            if fnmatch.fnmatch(i, gi) and fnmatch.fnmatch(o, go): best = max(best, d)
        return best
    t = L.attr['t_clk2q'] + path(F, 'A0', 'DQ0')
    # 가산기: 첫 칩 A→COUT, 마지막 칩 CIN→S4. 중간 칩은 CIN→COUT
    if carry_stages == 1:
        t += path(A, 'A1', 'S4')
    else:
        t += path(A, 'A1', 'COUT') + (carry_stages-2)*path(A,'CIN','COUT') + path(A, 'CIN', 'S4')
    t += L.attr['t_setup']
    return t*1e9

def sent_time(n_pe, pe_clk_mhz):
    rate = n_pe * pe_clk_mhz * 1e6
    tok = MODEL_MAC_PER_TOKEN / rate
    return tok, tok*TOKENS_PER_SENT

def main():
    chips = parse_chips(os.path.join(HERE, 'chips.txt'))
    inv = read_inventory(os.path.join(HERE, 'inventory.txt'))
    n, lim = max_pe(inv)
    ns_, lim_s = max_pe_bom(inv, PE_BOM_SAFE)

    print("=== 보유 부품으로 가능한 PE 수 ===")
    for chip, k in lim.items():
        print(f"  {chip:<12} {inv.get(chip,0):>3}개 ÷ {PE_BOM[chip]} = PE {k}개")
    print(f"  → PE {n}개  (제약: {[c for c,k in lim.items() if k==n]})")
    print(f"  ⚠ 플래시 VOH 2.4V < 74HC VIH 3.15V (sol.py R11) → pe2.txt 는 DQ 선마다 4.7kΩ 풀업 (PE당 16개).")
    print(f"    보유 RES4k7 {inv.get('RES4k7',0)}개 → 그 기준 PE {ns_}개. 저항은 갖고 있는 것으로 채울 것 (10kΩ 도 됨).")
    print()

    print("=== 임계 경로 (데이터시트 max, VCC=4.5V 25°C) ===")
    variants = [
        ("pe2.txt 그대로: 74HC283 캐리 2단 + DQ 풀업",   '74HC283',  'SST39SF040', 2),
        ("74HCT283 로 바꾸면 (풀업 불필요, 3ns 느림)",      '74HCT283', 'SST39SF040', 2),
        ("옛 모양: 74HC283 캐리 4단",                    '74HC283',  'SST39SF040', 4),
    ]
    rows = []
    for name, adder, flash, stages in variants:
        ns = crit_path_ns(chips, adder, flash, stages)
        rows.append((name, ns))
        print(f"  {name:<44}{ns:6.0f}ns → {1000/ns:5.1f}MHz")
    base_ns = rows[0][1]
    print()

    print("=== 이 경로에서 시간을 먹는 것 ===")
    L=chips['74HC574']; F=chips['SST39SF040']; A=chips['74HC283']
    items = [("74HC574 clk→Q", L.attr['t_clk2q']), ("플래시 A→DQ", F.attr['t_addr2dq']),
             ("74HC283 A→COUT", 39e-9), ("74HC283 CIN→S4", 46e-9), ("74HC574 setup", L.attr['t_setup'])]
    tot = sum(v for _,v in items)
    for k,v in items:
        print(f"  {k:<18}{v*1e9:5.0f}ns  {v/tot*100:4.0f}%")
    print(f"  {'합계':<18}{tot*1e9:5.0f}ns")
    print()

    pe_clk = 1000/base_ns
    print(f"=== RACHEL 시간 (PE 클럭 {pe_clk:.1f}MHz, 문장 {TOKENS_PER_SENT}토큰) ===")
    print(f"  {'PE':>4}{'토큰1개':>10}{'문장':>10}")
    for k in (1,2,4,8,16):
        tok, sent = sent_time(k, pe_clk)
        mark = " ← 보유" if k==n else ""
        print(f"  {k:>4}{tok:>9.2f}초{sent:>9.0f}초{mark}")
    print()

    print("=== 무엇을 바꾸면 얼마나 (부품 교체만, 회로 그대로) ===")
    alts = [
        ("플래시 -55 등급 (SST39SF040-55)",  dict(flash_ns=55)),
        ("SRAM 55ns 로 교체 (AS6C1008-55)",   dict(flash_ns=55)),
        ("SRAM 45ns 로 교체",                 dict(flash_ns=45)),
        ("74AC574 (clk→Q 12ns, setup 3ns)",  dict(clk2q=12, setup=3)),
        ("74AC283 (DIP 구하기 어려움)",        dict(adder_scale=0.4)),
    ]
    for name, ch in alts:
        clk2q = ch.get('clk2q', L.attr['t_clk2q']*1e9)
        setup = ch.get('setup', L.attr['t_setup']*1e9)
        fl = ch.get('flash_ns', F.attr['t_addr2dq']*1e9)
        ad = (39+46)*ch.get('adder_scale',1.0)
        ns = clk2q+fl+ad+setup
        tok, sent = sent_time(n, 1000/ns)
        print(f"  {name:<36}{ns:5.0f}ns  {1000/ns:5.1f}MHz  PE{n} 문장 {sent:4.0f}초  ({base_ns/ns:.2f}배)")
    print()

    print("=== 회로 구조를 바꾸면 (부품 추가 없음 또는 소량) ===")
    struct = [
        ("파이프라인: 플래시 뒤에 래치 (574 +2/PE)",
         max(L.attr['t_clk2q']*1e9 + F.attr['t_addr2dq']*1e9 + L.attr['t_setup']*1e9,
             L.attr['t_clk2q']*1e9 + 39+46 + L.attr['t_setup']*1e9)),
        ("파이프라인 + 캐리 1단씩 래치 (574 +4/PE)",
         max(L.attr['t_clk2q']*1e9 + F.attr['t_addr2dq']*1e9 + L.attr['t_setup']*1e9,
             L.attr['t_clk2q']*1e9 + 46 + L.attr['t_setup']*1e9)),
    ]
    for name, ns in struct:
        tok, sent = sent_time(n, 1000/ns)
        print(f"  {name:<40}{ns:5.0f}ns  {1000/ns:5.1f}MHz  PE{n} 문장 {sent:4.0f}초  ({base_ns/ns:.2f}배)")
    print()

    print("=== 5초/문장에 필요한 PE 수 ===")
    for name, ns in [("지금", base_ns)] + [(s[0], s[1]) for s in struct]:
        need = MODEL_MAC_PER_TOKEN*TOKENS_PER_SENT/5.0/(1000/ns*1e6)
        print(f"  {name:<40}{int(need)+1:>5}개")
    print()

    print("=== 가중치 대역폭 (2차원 배열이면 k배 재사용) ===")
    for k in (1,2,3,4):
        bw = 1000/base_ns * k   # MB/s (PE 클럭 × 행 수)
        print(f"  {k}×{k} ({k*k}PE): {bw:5.1f} MB/s   SRAM 55ns(18MB/s) {'OK' if bw<=18 else '부족'}")
    print()
    print("=== 검증기와 대조 ===")
    d = parse_design(os.path.join(HERE, 'pe2.txt'))
    R, ni, tim = verify(d)
    if tim['best']:
        print(f"  sol.py 최장 경로 {tim['best'][0]*1e9:.0f}ns  vs  tpu.py 계산 {base_ns:.0f}ns  → "
              f"{'일치' if abs(tim['best'][0]*1e9-base_ns)<1 else '⚠ 다름 — 둘 중 하나가 틀림'}")
    return 0

if __name__ == '__main__':
    sys.exit(main())
