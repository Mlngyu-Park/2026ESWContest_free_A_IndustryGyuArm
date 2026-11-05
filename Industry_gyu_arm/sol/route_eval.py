#!/usr/bin/env python3
"""
route_eval — KYLE 설계 하나를 '실제로' 채점한다 (배치기의 최종 심판).

    python3 route_eval.py 설계.txt            → 검증(치명/오류) + 뷰어 자동배선(미배선 수) + R28(독립 검사) 한 줄
    python3 route_eval.py 설계.txt --write    → 배선 결과(route 문)를 그 파일에 써 넣는다 (여러 장이면 장마다 되돌려 넣음). 파일이 진실.
    python3 route_eval.py 설계.txt --seed 3   → 라우터 씨앗

흐름: sol.py 검증 → JSON → viewer.html 빌드 → 헤드리스 브라우저에서 computeAllRoutes → toSOL 로 내보낸 배선을 sol.py R28 로 재검사.
뷰어(라우터)와 sol.py(검사기)는 서로 다른 구현이므로, 둘이 일치할 때만 '배선됨' 으로 믿는다.
"""
import io, os, sys, json, asyncio, subprocess, tempfile, shutil
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sol

def _rule_counts(R):
    out = {}
    for it in R.items:
        rule, level = (it[0], it[1]) if isinstance(it, (tuple, list)) else (it.get('rule'), it.get('level'))
        out[(rule, level)] = out.get((rule, level), 0) + 1
    return out

def evaluate(path, keep=False, seed=0, write=None):
    """→ dict(crit, err, warn, ok, fail, hop, bends, len, r28_viol, r28_segs, ms). 여러 장(panel)이면 장마다 채점해 합산."""
    d0 = sol.parse_design(os.path.abspath(path))
    if d0.panels:
        import panelize, tempfile
        tot = dict(crit=0, err=0, warn=0, ok=0, fail=0, hop=0, bends=0, len=0, r28_viol=0, r28_warn=0, r28_segs=0, ms=0, panels={})
        # 전체 파일 검증(R29 등)의 치명/오류도 센다
        R0, _, _ = sol.verify(d0); c0 = _rule_counts(R0)
        tot['crit'] += sum(v for (r, l), v in c0.items() if l == '치명'); tot['err'] += sum(v for (r, l), v in c0.items() if l == '오류')
        work = tempfile.mkdtemp(prefix='sol_panels_')
        try:
            for f in ('chips.txt', 'physics.txt'): os.symlink(os.path.join(HERE, f), os.path.join(work, f))
            for nm in d0.panel_order:
                sp = os.path.join(work, f'{nm}.txt'); io.open(sp, 'w', encoding='utf-8').write(panelize.panel_subdesign(path, nm))
                r = _evaluate_one(sp, keep=False, seed=seed, fresh=bool(write)); tot['panels'][nm] = r
                if write and r.get('sol_text'):
                    if not os.path.exists(write) or os.path.abspath(write) != os.path.abspath(path) and not tot.get('_copied'):
                        shutil.copy(path, write); tot['_copied'] = True
                    panelize.merge_panel(write, nm, r['sol_text'])
                for k in ('warn', 'ok', 'fail', 'hop', 'bends', 'len', 'r28_viol', 'r28_warn', 'r28_segs', 'ms'):
                    if r.get(k) is not None: tot[k] += r[k]
                if r.get('fail') is None: tot['fail'] = None; tot['ok'] = None; tot['note'] = f"{nm} 장 검증 실패"; break
        finally:
            shutil.rmtree(work, ignore_errors=True)
        return tot
    r = _evaluate_one(path, keep=keep, seed=seed, fresh=bool(write))
    if write and r.get('sol_text'):
        io.open(write, 'w', encoding='utf-8').write(r['sol_text']); r['written'] = write
    return r

def _strip_auto_routes(path, work):
    """--write 는 매번 처음부터 다시 깐다: 이전 자동배선(route … auto) 줄을 빼고 평가한다 (손으로 그린 route 는 남긴다).
    안 그러면 옛 배선을 먼저 차지한 채 나머지만 깔아 결과가 누적 의존이 된다."""
    lines = io.open(path, encoding='utf-8').read().split('\n')
    keep = [ln for ln in lines if not (ln.startswith('route') and ' auto ' in ln)]
    if len(keep) == len(lines): return path
    sp = os.path.join(work, 'noauto.txt'); io.open(sp, 'w', encoding='utf-8').write('\n'.join(keep)); return sp

def _evaluate_one(path, keep=False, seed=0, fresh=False):
    """→ dict(crit, err, warn, ok, fail, hop, bends, len, r28_viol, r28_segs, ms). fresh=True 면 옛 자동배선을 버리고 새로 깐다."""
    path = os.path.abspath(path)
    if fresh:
        work0 = tempfile.mkdtemp(prefix='sol_fresh_')
        for f in ('chips.txt', 'physics.txt'): os.symlink(os.path.join(HERE, f), os.path.join(work0, f))
        path = _strip_auto_routes(path, work0)
    d = sol.parse_design(path)
    R, ni, tim = sol.verify(d)
    cnt = _rule_counts(R)
    crit = sum(v for (r, l), v in cnt.items() if l == '치명'); err = sum(v for (r, l), v in cnt.items() if l == '오류'); warn = sum(v for (r, l), v in cnt.items() if l == '경고')
    res = dict(crit=crit, err=err, warn=warn, ok=None, fail=None)
    if d.errors or crit or err:
        res['note'] = '검증 실패 — 배선 생략'; return res
    work = tempfile.mkdtemp(prefix='sol_eval_')
    try:
        jpath = os.path.join(work, 'd.json'); hpath = os.path.join(work, 'v.html')
        sol.export_json(d, R, ni, tim, jpath)
        subprocess.run([sys.executable, os.path.join(HERE, 'build_viewer.py'), jpath, hpath], check=True, capture_output=True)
        r = asyncio.run(_route(hpath, seed))
        res.update(r)
        # 내보낸 배선을 독립 검사기(R28)로
        rt = os.path.join(work, 'rt.txt'); res['sol_text'] = r.pop('sol_text'); io.open(rt, 'w', encoding='utf-8').write(res['sol_text'])
        for f in ('chips.txt', 'physics.txt'):
            os.symlink(os.path.join(HERE, f), os.path.join(work, f))
        d2 = sol.parse_design(rt); R2, _, _ = sol.verify(d2); c2 = _rule_counts(R2)
        res['r28_viol'] = sum(v for (rr, l), v in c2.items() if rr == 'R28' and l == '오류')
        res['r28_warn'] = sum(v for (rr, l), v in c2.items() if rr == 'R28' and l == '경고')
        res['r28_segs'] = sum(len(v) for v in d2.routes.values())
        if keep: res['kept'] = shutil.copytree(work, path + '.eval', dirs_exist_ok=True)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return res

async def _route(html, seed):
    from playwright.async_api import async_playwright
    async with async_playwright() as p:
        b = await p.chromium.launch(); pg = await b.new_page(viewport={'width': 1200, 'height': 800})
        errs = []; pg.on('pageerror', lambda e: errs.append(str(e)))
        await pg.goto('file://' + html); await pg.wait_for_timeout(1200)
        r = await pg.evaluate("""(seed)=>{ __sol.S.routeSeed=seed|0; __sol.bumpGeom(); const t0=performance.now(); const res=__sol.computeAllRoutes(); const sc=__sol.routeScore(res);
          let hop=0; const failed=[]; for(const n in res){ res[n].forEach(s=>{ if(s.hop) hop++; }); if(!__sol.D.nets[n].supply && res[n].some(s=>s.fail)) failed.push(n); } sc.hop=hop; sc.failed=failed; sc.ms=Math.round(performance.now()-t0);
          sc.sol_text=__sol.toSOL(); return sc; }""", seed)
        await b.close()
        if errs: r['js_errors'] = errs[:3]
        return r

def fmt(r):
    if r.get('fail') is None: return f"치명 {r['crit']} 오류 {r['err']} — {r.get('note','')}"
    if r.get('panels'):
        per = ' · '.join(f"{nm}:{v.get('fail')}" for nm, v in r['panels'].items())
        return (f"치명 {r['crit']} 오류 {r['err']} 경고 {r['warn']} | 장별 미배선 {per} (합 {r['fail']}) 층넘기 {r['hop']} "
                f"| R28 위반 {r['r28_viol']} 경고 {r['r28_warn']} ({r['r28_segs']}구간) | {r['ms']}ms")
    return (f"치명 {r['crit']} 오류 {r['err']} 경고 {r['warn']} | 배선 {r['ok']} 미배선 {r['fail']} 층넘기 {r.get('hop',0)} 꺾임 {r['bends']} 길이 {r['len']} "
            f"| R28 위반 {r.get('r28_viol')} 경고 {r.get('r28_warn')} ({r.get('r28_segs')}구간) | {r.get('ms')}ms" + (f" | JS오류 {r['js_errors']}" if r.get('js_errors') else ''))

if __name__ == '__main__':
    if len(sys.argv) < 2: print(__doc__); sys.exit(1)
    a = sys.argv; seed = int(a[a.index('--seed') + 1]) if '--seed' in a else 0
    write = None
    if '--write' in a:
        i = a.index('--write'); write = a[i + 1] if len(a) > i + 1 and not a[i + 1].startswith('--') else a[1]
    r = evaluate(a[1], keep='--keep' in a, seed=seed, write=write)
    print(fmt(r) + (f"  → 썼음: {write}" if write else ''))
