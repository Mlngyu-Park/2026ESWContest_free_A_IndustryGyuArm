#!/usr/bin/env python3
"""
simulate.py 자동 시험 — 칩 동작(chips.txt func/reg)과 실제 회로 동작을 확인한다.
    python3 test_sim.py
AI 가 답을 아는 게 아니라, KYLE 과 chips.txt 만으로 계산한 결과를 기대값과 대조한다.
"""
import io, os, sys, random
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sol, simulate

fails = []
def check(c, m):
    print(("  ok   " if c else "  FAIL ") + m)
    if not c: fails.append(m)

# 1) 칩 진리표 (데이터시트 대조) — simulate.selftest 와 같은 검사
check(simulate.selftest() == 0, "칩 자체 점검 (인버터·가산기 512경우·버퍼 OE#)")

# 2) 클럭 분주기: 74HC74 되먹임(1nQ→1D)이 2분주로 도는가
DESIGN = os.path.join(HERE, 'pe1_54x33_2layer.txt')
if os.path.exists(DESIGN):
    d = sol.parse_design(DESIGN)
    s = simulate.Sim(d); s.add_jumper('JP_CLK', 'B-C')
    seq = []
    s.set_net('103', 0); s.settle(); s.clock_edge()
    for _ in range(6):
        s.set_net('103', 1); s.settle(); s.clock_edge(); s.settle(); seq.append(s.net.get('107'))
        s.set_net('103', 0); s.settle(); s.clock_edge(); s.settle()
    check(seq == [1, 0, 1, 0, 1, 0] or seq == [0, 1, 0, 1, 0, 1], f"클럭 2분주 토글 {seq}")

    # 3) 회로 전체: w×x 가 플래시(곱셈표) 를 거쳐 나오는가
    LO = bytearray(65536); HI = bytearray(65536)
    for w in range(256):
        for x in range(256):
            p = w * x; LO[(w << 8) | x] = p & 0xFF; HI[(w << 8) | x] = (p >> 8) & 0xFF
    LO = bytes(LO); HI = bytes(HI)

    def run(w, x):
        s = simulate.Sim(d); s.rom['PE0_U1L'] = LO; s.rom['PE0_U1H'] = HI
        s.add_jumper('JP_CLK', 'B-C')
        s.set_bus('MEGA', 'D', w); s.set_net('118', 0); s.settle(); s.clock_edge()
        s.set_net('118', 1); s.settle(); s.clock_edge(); s.settle(); s.set_net('118', 0)
        s.set_bus('MEGA', 'D', x); s.settle()
        for _ in range(4):
            s.set_net('103', 1); s.settle(); s.clock_edge(); s.settle()
            s.set_net('103', 0); s.settle(); s.clock_edge(); s.settle()
        lo, hi = s.bus('PE0_U1L', 'DQ'), s.bus('PE0_U1H', 'DQ')
        return None if (lo is None or hi is None) else (hi << 8) | lo

    random.seed(7)
    cases = [(0, 0), (1, 1), (7, 6), (255, 255), (12, 21), (200, 7)]
    cases += [(random.randrange(256), random.randrange(256)) for _ in range(10)]
    bad = [(w, x, run(w, x), w * x) for w, x in cases]
    bad = [t for t in bad if t[2] != t[3]]
    check(not bad, f"회로가 곱셈 {len(cases)}경우 계산 (틀림 {len(bad)}: {bad[:2]})")
    # 4) KYLE 안의 sim … end 시나리오를 sol.py R30 이 돌려 전부 ✔ 인가 (파일에 시험이 적혀 있고, 뷰어 검증에도 뜬다)
    if d.sims:
        R, _, _ = sol.verify(d)
        r30 = [it for it in R.items if (it[0] if isinstance(it,(tuple,list)) else it['rule']) == 'R30']
        lv = [(it[1] if isinstance(it,(tuple,list)) else it['level']) for it in r30]
        check(r30 and lv.count('오류') == 0 and lv.count('경고') == 0 and lv.count('정보') >= 2,
              f"sim 블록 {len(d.sims)}개 → R30 정보 {lv.count('정보')} 오류 {lv.count('오류')} 경고 {lv.count('경고')}")
    else:
        check(False, "설계 파일에 sim 블록이 없음")
else:
    print("  (설계 파일이 없어 회로 시험 건너뜀)")

print("\n결과:", "전부 통과" if not fails else f"실패 {len(fails)}건: " + "; ".join(fails))
sys.exit(0 if not fails else 1)
