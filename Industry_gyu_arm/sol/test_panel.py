#!/usr/bin/env python3
"""
여러 장(panel) 자동 시험 — panelize 로 나눈 파일이 검증을 통과하고, 장별로 배선되고, 뷰어에서 한 장씩 열어 저장해도
다른 장이 안 깨지는지 (KYLE 이 진실: 여러 장 구성이 파일에 남고 다시 열린다).
    python3 test_panel.py
"""
import io, os, sys, subprocess, time, re, asyncio
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sol, panelize, route_eval
fails = []
def check(c, m):
    print(("  ok   " if c else "  FAIL ") + m)
    if not c: fails.append(m)
def errs(d):
    R, _, _ = sol.verify(d); return sum(1 for it in R.items if (it[1] if isinstance(it,(tuple,list)) else it['level']) in ('치명','오류'))

SRC = os.path.join(HERE, 'fixture_2pe.txt'); OUT = os.path.join(HERE, '.test_panel.txt')
if os.path.exists(OUT): os.remove(OUT)
info = panelize.panelize(SRC, OUT, n=2, by='prefix', board=(54, 33), layers=2, channel=2, seed=3, iters=4000)
d = sol.parse_design(OUT)
check(not d.errors, f"여러 장 KYLE 파싱 오류 없음 {d.errors[:2]}")
check(list(d.panels) == ['A', 'B'], f"panel 2장 선언 {list(d.panels)}")
check(errs(d) == 0, f"검증 치명/오류 0 (R23 장별·R29 장 넘는 네트) — 실제 {errs(d)}")
pan = {r: d.parts[r].get('panel') for r in d.order if not d.parts[r].get('link') and r not in d.stacks}
check(all(pan[r] == 'A' for r in pan if r.startswith('PE0_')) and all(pan[r] == 'B' for r in pan if r.startswith('PE1_')), "PE0→A, PE1→B")
caps_ok = all(pan.get(r) == pan.get(d.parts[r]['near']) for r in pan if d.parts[r].get('near') in pan)
check(caps_ok, "디커플링 캡이 제 칩과 같은 장")
check(d.links and all((l['ref'] in d.parts and l['ref']+'_B' in d.parts) for l in d.links), f"link 헤더가 양쪽 장에 부품으로 {len(d.links)}개")
check(d.pbuses and set(d.pbuses) == {'A', 'B'}, f"장별 버스 {list(d.pbuses)}")
# 장별로 떼어낸 단일 기판이 각각 검증 통과
for nm in ('A', 'B'):
    txt = panelize.panel_subdesign(OUT, nm)
    sp = os.path.join(HERE, f'.test_panel_{nm}.txt'); io.open(sp, 'w', encoding='utf-8').write(txt)
    ds = sol.parse_design(sp)
    check(not ds.errors and errs(ds) == 0, f"{nm} 장 단독 검증 통과 (부품 {len(ds.order)})")
    os.remove(sp)
# 실제 배선(장별 합산) — R28 위반 0
r = route_eval.evaluate(OUT)
check(r.get('fail') is not None and r.get('r28_viol') == 0, "장별 실제 배선 → R28 위반 0: " + route_eval.fmt(r))

# 뷰어: B 장만 열어 저장하면 A 장은 그대로, B 장 좌표만 바뀐다
before = io.open(OUT, encoding='utf-8').read()
a_lines = sorted(l for l in before.split('\n') if l.startswith('part') and l.rstrip().endswith('panel A'))
srv = subprocess.Popen([sys.executable, os.path.join(HERE, 'serve.py'), OUT, '--panel', 'B'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=dict(os.environ, BROWSER='true'))
time.sleep(1.5)
try:
    from playwright.async_api import async_playwright
    async def run():
        async with async_playwright() as p:
            b = await p.chromium.launch(); pg = await b.new_page(viewport={'width': 1400, 'height': 900})
            jerr = []; pg.on('pageerror', lambda e: jerr.append(str(e)))
            await pg.goto('http://127.0.0.1:8765/'); await pg.wait_for_timeout(1500)
            sel = await pg.evaluate("(()=>{ const s=document.getElementById('selPanel'); return s? [s.value, s.options.length] : null; })()")
            check(sel and sel[0] == 'B' and sel[1] == 2, f"뷰어에 장 선택 (현재 {sel})")
            nparts = await pg.evaluate("__sol.D.parts.length")
            check(all(not p.startswith('PE0_') for p in await pg.evaluate("__sol.D.parts.map(p=>p.ref)")), f"B 장만 보임 (부품 {nparts})")
            # 부품 하나 옮기고 저장
            ref = await pg.evaluate("__sol.D.parts.find(p=>p.ref.startsWith('PE1_U'))?.ref")
            await pg.evaluate(f"(()=>{{ const p=__sol.partMap['{ref}']; p.col+=1; __sol.recomputePins(p); }})()")
            await pg.click('#bSave'); await pg.wait_for_timeout(1500)
            hint = await pg.evaluate("document.getElementById('srvHint').textContent")
            check('저장' in hint, "B 장 저장: " + hint[:60])
            check(not jerr, f"JS 오류 없음 {jerr[:1]}")
            await b.close()
            return ref
    moved = asyncio.run(run())
finally:
    srv.terminate()
after = io.open(OUT, encoding='utf-8').read()
a_after = sorted(l for l in after.split('\n') if l.startswith('part') and l.rstrip().endswith('panel A'))
check(a_lines == a_after, "저장 뒤 A 장 부품 줄은 그대로")
d2 = sol.parse_design(OUT)
check(not d2.errors and list(d2.panels) == ['A', 'B'] and len(d2.links) == len(d.links), "저장 뒤에도 여러 장 구성이 그대로 (panel/link)")
if moved: check(d2.parts[moved]['col'] == d.parts[moved]['col'] + 1, f"{moved} 의 새 좌표가 파일에 들어감")
for f in (OUT, OUT + '.bak'):
    if os.path.exists(f): os.remove(f)
print("\n결과:", "전부 통과" if not fails else f"실패 {len(fails)}건: " + "; ".join(fails))
sys.exit(0 if not fails else 1)
