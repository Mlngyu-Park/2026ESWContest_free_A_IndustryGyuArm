#!/usr/bin/env python3
"""
space — "PE 하나(또는 n개)를 만능기판에 올리려면 구멍이 몇 개 필요한가" 를 실측한다.

    python3 space.py --pes 1 --board 60 37 --layers 1            # 한 구성: 생성 → 배치(씨앗 탐색, 실제 배선 채점) → 다듬기 → 배선 확정
    python3 space.py --table                                     # 지금까지의 결과 표 (.space/results.txt)

한 줄 결과: 기판 W×H, 층, 미배선(0 이면 됨), 부품 면적 비율, 파일 경로.
숫자는 전부 실측(배치기 + 뷰어 라우터 + R28 독립 검사)이고 추정이 없다.
"""
import io, os, sys, argparse, subprocess, shutil, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sol, place, route_eval
DIR = os.path.join(HERE, '.space'); RES = os.path.join(DIR, 'results.txt')

def part_area(d):
    """부품이 차지하는 구멍 수 (핀 상자 기준, 소켓 폭 포함) / 기판 구멍 수"""
    tot = 0
    for ref, p in d.parts.items():
        if ref in d.stacks: continue
        c = d.chips[p['chip']]; p['_pkg'] = c.pkg
        n = sol.pkg_geom(c.pkg)[0]
        hs = [sol.pin_hole(p, k) for k in range(1, n+1)]
        cs = [h[0] for h in hs]; rs = [h[1] for h in hs]
        tot += (max(cs)-min(cs)+1) * (max(rs)-min(rs)+1)
    return tot / (d.board[0]*d.board[1]*max(1, d.layers))

def one(pes, W, H, L, search, refine, seed, channel):
    os.makedirs(DIR, exist_ok=True)
    for f in ('chips.txt', 'physics.txt'):                       # KYLE 의 include chips.txt 는 파일 옆에서 찾는다
        lk = os.path.join(DIR, f)
        if not os.path.exists(lk): os.symlink(os.path.join(HERE, f), lk)
    tag = f"pe{pes}_{W}x{H}_L{L}"; gen = os.path.join(DIR, tag + '.gen.txt'); out = os.path.join(DIR, tag + '.txt')
    env = dict(os.environ, PES=str(pes), BOARD_W=str(W), BOARD_H=str(H), LAYERS=str(L), NOPLACE='1', OUT=gen)
    subprocess.run([sys.executable, os.path.join(HERE, 'gen_pe2.py')], env=env, check=True, capture_output=True)
    t0 = time.time()
    key = lambda r: (r.get('crit', 9) + r.get('err', 9), r['fail'] if r.get('fail') is not None else 10**6, r.get('bends', 0))
    best = None
    for s in range(seed, seed + search):
        cand = out + f'.s{s}.txt'
        try:
            info = place.run(gen, cand, W=W, H=H, layers=L, seed=s, iters=25000, channel=channel)
        except SystemExit as e:
            print(f"  씨앗 {s}: 배치 실패 {e}"); continue
        if info.get('fail_place'): print(f"  씨앗 {s}: 자리 못 찾은 부품 {len(info['fail_place'])}"); continue
        r = route_eval.evaluate(cand); print(f"  씨앗 {s}: {route_eval.fmt(r)[:100]}", flush=True)
        if best is None or key(r) < best[0]: best = (key(r), cand, r)
    if best is None:
        line = f"PE {pes} | {W}x{H} | {L}층 | 배치 불가 (부품이 안 들어감)"
        io.open(RES, 'a', encoding='utf-8').write(line + '\n'); print(line); return
    if refine > 0 and best[2].get('fail'):
        place.refine(best[1], out, evals=refine, seed=seed, verbose=True)
    else:
        shutil.copy(best[1], out)
    r = route_eval.evaluate(out, write=out)
    d = sol.parse_design(out)
    fail = r.get('fail'); pa = part_area(d)
    line = (f"PE {pes} | {W}x{H} ({W*2.54:.0f}x{H*2.54:.0f}mm) | {L}층 | 미배선 {fail} | 부품 면적 {pa:.0%} | 검증 치명 {r['crit']} 오류 {r['err']} | R28 위반 {r.get('r28_viol')} "
            f"| {int(time.time()-t0)}s | {os.path.relpath(out, HERE)}")
    io.open(RES, 'a', encoding='utf-8').write(line + '\n'); print(line)
    for s in range(seed, seed + search):
        f = out + f'.s{s}.txt'
        if os.path.exists(f): os.remove(f)
    os.remove(gen)

def table():
    if not os.path.exists(RES): print("(결과 없음)"); return
    rows = [l.rstrip('\n') for l in io.open(RES, encoding='utf-8') if l.strip()]
    def k(l):
        t = [x.strip() for x in l.split('|')]
        try: W, H = t[1].split()[0].split('x'); return (int(t[2].rstrip('층')), int(W)*int(H))
        except Exception: return (99, 0)
    for l in sorted(rows, key=k): print(l)

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pes', type=int, default=1); ap.add_argument('--board', nargs=2, type=int); ap.add_argument('--layers', type=int, default=1)
    ap.add_argument('--search', type=int, default=4); ap.add_argument('--refine', type=int, default=30); ap.add_argument('--seed', type=int, default=300)
    ap.add_argument('--channel', type=int, default=2); ap.add_argument('--table', action='store_true')
    a = ap.parse_args()
    if a.table: table(); return
    if not a.board: ap.error('--board W H 가 필요합니다')
    one(a.pes, a.board[0], a.board[1], a.layers, a.search, a.refine, a.seed, a.channel)

if __name__ == '__main__':
    main()
