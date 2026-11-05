#!/usr/bin/env python3
"""
place.py 자동 시험 — KYLE 을 넣어 KYLE 이 나오고, 그 KYLE 이 검증기를 통과하며, 배치가 규칙(겹침·채널·near·버스)을 지키는지.
    python3 test_place.py            (빠름: 배선 없이)
    python3 test_place.py --route    (실제 배선 채점까지)
"""
import io, os, sys, subprocess, re
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sol, place
fails = []
def check(c, m):
    print(("  ok   " if c else "  FAIL ") + m)
    if not c: fails.append(m)

# 시험은 고정 설계(2PE 84x62)를 쓴다 — pe2.txt 는 사용자가 기판·PE 수를 바꾸는 작업 파일이라 기준이 될 수 없다
SRC = os.path.join(HERE, 'fixture_2pe.txt'); OUT = os.path.join(HERE, '.test_placed.txt')
info = place.run(SRC, OUT, seed=3, iters=4000)
d0 = sol.parse_design(SRC); d1 = sol.parse_design(OUT)
check(not d1.errors, f"출력 KYLE 파싱 오류 없음 {d1.errors[:2]}")
check(set(d0.order) - set(d0.stacks) == set(d1.order) - set(d1.stacks), "부품 목록 보존")
check(d0.pinnet == {k: v for k, v in d1.pinnet.items() if k[0] not in d1.stacks}, "pin 연결(포인터) 보존")
check(d0.expects == d1.expects and d0.groups == d1.groups, "expect / group 보존")
def _inside(p, W, H, chips):
    q = dict(p); q['_pkg'] = chips[p['chip']].pkg
    b = place.bbox(q)
    return b[0] >= 1 and b[1] >= 1 and b[2] <= W - 2 and b[3] <= H - 2
# fixed 는 기판 안에 있을 때만 고정이다 (밖이면 배치기가 끌어들인다 — 더 큰 기판 기준 파일을 작은 기판에 올릴 때)
check(all(d1.parts[r]['col'] == d0.parts[r]['col'] and d1.parts[r]['row'] == d0.parts[r]['row']
          for r in d0.order if d0.parts[r].get('fixed') and _inside(d0.parts[r], d0.board[0], d0.board[1], d0.chips)),
      "기판 안의 fixed 부품은 안 움직임")
R, ni, tim = sol.verify(d1)
lv = {}
for it in R.items:
    rule, level = (it[0], it[1]) if isinstance(it, (tuple, list)) else (it['rule'], it['level'])
    lv[level] = lv.get(level, 0) + 1
check(lv.get('치명', 0) == 0 and lv.get('오류', 0) == 0, f"검증 치명 {lv.get('치명',0)} 오류 {lv.get('오류',0)} (R23 겹침·기판 밖 포함)")
r17 = [it for it in R.items if (it[0] if isinstance(it,(tuple,list)) else it['rule']) == 'R17' and (it[1] if isinstance(it,(tuple,list)) else it['level']) == '경고']
check(not r17, f"디커플링 캡이 칩 옆 (R17 경고 {len(r17)})")
check(len(d1.buses.get('GND', [])) <= 10 and len(d1.buses.get('VCC', [])) <= 10, f"버스 행 수 (칩이 62행에 퍼지고 574 는 1행 안이어야 해서 6~10줄) GND {d1.buses.get('GND')} VCC {d1.buses.get('VCC')}")
# 채널: 칩 상자끼리 2구멍 이상
pl = place.Placer(d1, d1.board[0], d1.board[1], d1.layers)
bad = 0
for i, a in enumerate(pl.refs):
    for b in pl.refs[i+1:]:
        if pl.P[a]['layer'] != pl.P[b]['layer']: continue
        if pl.P[a].get('fixed') and pl.P[b].get('fixed'): continue
        if a in pl.attached_caps or b in pl.attached_caps: continue
        if place.boxes_clash(pl.box[a], pl.box[b], place.gap_needed(pl.P[a], pl.P[b], 2)): bad += 1
check(bad == 0, f"칩 사이 채널 ≥ 2구멍, 작은 부품 1구멍 (위반 {bad})")
# 결정성
OUT2 = OUT + '.2'; place.run(SRC, OUT2, seed=3, iters=4000)
check(io.open(OUT, encoding='utf-8').read() == io.open(OUT2, encoding='utf-8').read(), "같은 씨앗 → 같은 결과")
# 2층: 라이저가 서고, 층을 넘는 네트가 stack 을 지난다 (R26 0)
OUT3 = OUT + '.l2'; place.run(SRC, OUT3, layers=2, seed=3, iters=4000)
d3 = sol.parse_design(OUT3); R3, _, _ = sol.verify(d3)
r26 = sum(1 for it in R3.items if (it[0] if isinstance(it,(tuple,list)) else it['rule']) in ('R26',) )
check(d3.layers == 2 and len(d3.stacks) >= 1, f"2층: 라이저 {len(d3.stacks)}개")
check(r26 == 0, f"2층: 층을 넘는 네트가 전부 라이저를 지남 (R26 {r26})")
lv3 = {}
for it in R3.items:
    level = it[1] if isinstance(it,(tuple,list)) else it['level']; lv3[level] = lv3.get(level, 0) + 1
check(lv3.get('치명', 0) == 0 and lv3.get('오류', 0) == 0, f"2층 검증 치명 {lv3.get('치명',0)} 오류 {lv3.get('오류',0)}")
# 좁은 2층 기판: 라이저 자리가 모자라도 "층 걸침 + 라이저 없음"(R26) 은 하나도 남으면 안 된다
for sd in (90, 91, 93):
    OUT4 = OUT + f'.s{sd}'
    place.run(SRC, OUT4, W=44, H=62, layers=2, channel=1, seed=sd, iters=3000)
    d4 = sol.parse_design(OUT4); R4, _, _ = sol.verify(d4)
    e4 = sum(1 for it in R4.items if (it[1] if isinstance(it, (tuple, list)) else it['level']) == '오류')
    carried = {n for (r, pn), n in d4.pinnet.items() if r in d4.stacks}
    lay = {r: d4.parts[r].get('layer', 1) for r in d4.order}
    cross = [n for n, pl_ in d4.nets.items()
             if n not in d4.supplies and len({lay[r] for r, k in pl_ if r in lay}) > 1 and n not in carried]
    check(e4 == 0 and not cross, f"44x62 2층 씨앗 {sd}: 검증 오류 {e4}, 라이저 없이 층 걸친 네트 {len(cross)}")
    os.remove(OUT4)

if '--route' in sys.argv:
    import route_eval
    r = route_eval.evaluate(OUT); check(r.get('r28_viol') == 0, "실제 배선 → R28 위반 0: " + route_eval.fmt(r))
for f in (OUT, OUT2, OUT3):
    if os.path.exists(f): os.remove(f)
print("\n결과:", "전부 통과" if not fails else f"실패 {len(fails)}건: " + "; ".join(fails))
sys.exit(0 if not fails else 1)
