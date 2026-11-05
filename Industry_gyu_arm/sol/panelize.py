#!/usr/bin/env python3
"""
panelize — 설계를 여러 '장'(기판)으로 나눠 한 KYLE 파일에 담는다.

    python3 panelize.py pe2.txt --out two_boards.txt --by prefix        # PE0_* → A 장, PE1_* → B 장
    python3 panelize.py pe2.txt --out two_boards.txt --by graph --panels 2
    python3 panelize.py pe2.txt --out x.txt --by prefix --board 54 33 --layers 2

왜 필요한가: 기판 한 장에 안 들어가면 여러 장으로 나눠 핀헤더로 잇는다. 그 구성이 **파일에 남아야**
나중에 다시 열어 이어서 작업할 수 있다 (KYLE 이 진실이라는 원칙).

나오는 KYLE 문장:
    panel A perf 54 33 layers 2      ← 장 하나 (여러 개 쓸 수 있다)
    part PE0_U2 74HC574 at 12 2 panel A
    link L1 HDR8 at A 52 4 to B      ← A 장 가장자리 헤더가 B 장으로 (장을 넘는 네트를 나른다)
    pin  L1[1:101]

장을 나누는 방법(--by):
    prefix : 부품 이름 앞 토큰(PE0_/PE1_)으로. 공통 부품(OSC·MEGA·클럭)은 첫 장에.
    graph  : KYLE 의 pin 연결만 보고 나눈다 (place.py 의 층 분할과 같은 방식). 이름 규칙이 없을 때.
"""
import io, os, re, sys, argparse
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sol
import place


def split_prefix(d, refs, n):
    """이름의 '반복 단위'(PE0_/PE1_ …)로 나눈다. 번호가 없는 공통 부품(OSC·U_CLK·MEGA…)은 전부 첫 장에 모은다.
    (예전엔 접두어를 알파벳순으로 돌려가며 배정해서 U_* 같은 공통 부품이 두 번째 장으로 밀려났다)"""
    groups = {}
    for r in refs:
        m = re.match(r'^([A-Za-z]+)(\d+)_', r)          # PE0_U2 → 단위 'PE', 번호 0
        key = (m.group(1), int(m.group(2))) if m else None
        groups.setdefault(key, []).append(r)
    units = sorted(k for k in groups if k is not None)
    out = {}
    for i, k in enumerate(units):
        for r in groups[k]: out[r] = i % n
    for r in groups.get(None, []): out[r] = 0          # 공통 부품은 첫 장
    return out


def split_graph(d, refs, n):
    """KYLE 연결만 보고 나눈다 (연결 강한 것끼리 뭉친다)."""
    pl = place.Placer(d, d.board[0], d.board[1], 1)
    Wg = pl._net_graph()
    adj = {r: {} for r in refs}
    for (a, b), w in Wg.items():
        if a in adj and b in adj: adj[a][b] = w; adj[b][a] = w
    area = {r: (pl.box[r][2]-pl.box[r][0]+1) * (pl.box[r][3]-pl.box[r][1]+1) for r in refs}
    target = sum(area.values()) / n
    out = {}; left = set(refs); load = [0.0] * n
    for i in range(n):
        if not left: break
        seed = max(sorted(left), key=lambda r: area[r])
        out[seed] = i; load[i] += area[seed]; left.discard(seed)
        while left and load[i] < target:
            cand = max(sorted(left), key=lambda r: (sum(w for o, w in adj[r].items() if out.get(o) == i), -area[r]))
            out[cand] = i; load[i] += area[cand]; left.discard(cand)
    for r in sorted(left, key=lambda x: -area[x]):
        i = min(range(n), key=lambda j: load[j]); out[r] = i; load[i] += area[r]
    return out


def panelize(src, out_path, n=2, by='prefix', board=None, layers=None, hdr='HDR20',
             place_each=True, channel=2, seed=0, iters=20000, search=1, refine=0):
    d = sol.parse_design(src)
    if d.errors: raise SystemExit('입력 KYLE 오류: ' + '; '.join(d.errors[:3]))
    refs = [r for r in d.order if r not in d.stacks]
    assign = (split_graph if by == 'graph' else split_prefix)(d, refs, n)
    for r in refs:                                       # near 캡은 칩과 같은 장 (이름에 PE 번호가 없어 첫 장으로 밀렸었다)
        nr = d.parts[r].get('near')
        if nr and nr in assign: assign[r] = assign[nr]
    names = [chr(ord('A') + i) for i in range(n)]
    W, H = board if board else d.board
    L = layers if layers else d.layers

    # 장을 넘는 네트 → link 헤더가 날라야 한다
    cross = []
    for net in sorted(d.nets, key=str):
        if net in d.supplies: continue
        pans = {assign.get(r) for r, k in d.nets[net] if r in assign}
        pans.discard(None)
        if len(pans) > 1: cross.append((net, sorted(pans)))
    # 장 쌍마다 헤더. 전원은 모든 장에 있어야 하니 각 쌍에 VCC/GND 를 먼저 싣는다
    pairs = {}
    for net, pans in cross:
        for a, b in zip(pans, pans[1:]):
            pairs.setdefault((a, b), []).append(net)
    for i in range(n - 1):
        pairs.setdefault((i, i + 1), [])
    links = []; li = 0
    hdr = 'HDR8'; cap = 8                                   # 33행 기판엔 HDR20 두 개가 한 열에 안 들어간다 → 8핀으로 쪼갠다
    per_col = max(1, (H - 2) // (cap + 1))                  # 한 열에 들어가는 헤더 수 (사이 1칸)
    slot = {}                                               # 장 -> 다음 슬롯 번호
    def edge_spot(pname, right):
        k = slot.get(pname, 0); slot[pname] = k + 1
        colk, rowk = k // per_col, k % per_col
        col = (W - 2 - 2 * colk) if right else (1 + 2 * colk)
        return col, 1 + rowk * (cap + 1)
    for (a, b), nets in sorted(pairs.items()):
        take_all = ['VCC', 'GND'] + nets
        while take_all:
            chunk = take_all[:cap]; take_all = take_all[cap:]
            li += 1
            c1, r1 = edge_spot(names[a], right=True)        # A 장: 오른쪽 가장자리
            c2, r2 = edge_spot(names[b], right=False)       # B 장: 왼쪽 가장자리 (케이블 반대편)
            links.append(dict(ref=f'L{li}', kind=hdr, panel=names[a], col=c1, row=r1,
                              to=names[b], col2=c2, row2=r2, nets=chunk))

    # KYLE 쓰기: 원본을 그대로 두고 board→panel, part 에 panel 토큰, link/pin 추가
    lines = io.open(src, encoding='utf-8').read().split('\n')
    out = []; done_board = False; last_part = None
    for ln in lines:
        t = ln.split()
        if t and t[0] == 'board':
            if not done_board:
                for nm in names: out.append(f"panel   {nm} perf {W} {H}" + (f" layers {L}" if L > 1 else ''))
                done_board = True
            continue
        if t and t[0] == 'panel': continue
        if t and t[0] == 'part' and len(t) > 1 and t[1] in assign:
            out.append(ln.rstrip() + f"  panel {names[assign[t[1]]]}"); last_part = len(out) - 1; continue
        if t and t[0] == 'link': continue
        if t and t[0] == 'bus': continue                       # 버스는 장마다 배치기가 새로 정한다
        if t and t[0] == 'pin' and re.match(r'^L\d+\[', t[1] if len(t) > 1 else ''): continue
        out.append(ln)
    ins = []
    for lk in links:
        ins.append(f"link    {lk['ref']} {lk['kind']} at {lk['panel']} {lk['col']} {lk['row']} to {lk['to']} {lk['col2']} {lk['row2']}")
    for lk in links:
        for k, net in enumerate(lk['nets'], 1): ins.append(f"pin     {lk['ref']}[{k}:{net}]")
    if last_part is None: out += [''] + ins
    else: out[last_part + 1:last_part + 1] = [''] + ins
    txt = re.sub(r'\n{3,}', '\n\n', '\n'.join(out))
    io.open(out_path, 'w', encoding='utf-8').write(txt if txt.endswith('\n') else txt + '\n')
    if place_each:
        _place_panels(out_path, names, W, H, L, channel=channel, seed=seed, iters=iters, search=search, refine=refine)
    counts = {nm: sum(1 for r in refs if names[assign[r]] == nm) for nm in names}
    return dict(panels=counts, links=len(links), cross=len(cross))


def panel_subdesign(path, nm, W=None, H=None, L=None):
    """여러 장 KYLE 에서 장 nm 만 떼어 단일 기판 KYLE 텍스트로 (link 헤더는 그 장 쪽만 fixed 부품으로, 라이저는 그 장 것만)."""
    d = sol.parse_design(path)
    pv = d.panels.get(nm, {})
    W = W or pv.get('w') or d.board[0]; H = H or pv.get('h') or d.board[1]; L = L or pv.get('layers') or d.layers
    first = d.panel_order[0] if d.panel_order else nm
    pan = {r: (d.parts[r].get('panel') or (d.stacks[r].get('panel') if r in d.stacks else None) or first) for r in d.order}
    mine = {r for r in d.order if pan.get(r) == nm}
    link_refs = {l['ref'] for l in d.links}
    sub = []
    for ln in io.open(path, encoding='utf-8').read().split('\n'):
        t = ln.split()
        if not t: sub.append(ln); continue
        if t[0] == 'panel': continue
        if t[0] == 'board': continue
        if t[0] == 'link':
            lk = next((l for l in d.links if l['ref'] == t[1]), None)
            if not lk: continue
            if lk['panel'] == nm: sub.append(f"part    {lk['ref']:<9}{lk['kind']:<12}at {lk['col']:>2} {lk['row']:>2}  fixed")
            elif lk['to'] == nm: sub.append(f"part    {lk['ref']:<9}{lk['kind']:<12}at {lk['col2']:>2} {lk['row2']:>2}  fixed")
            continue
        if t[0] == 'part':
            if t[1] not in mine or t[1].endswith('_B'): continue
            sub.append(re.sub(r'\s+panel\s+\S+', '', ln)); continue
        if t[0] == 'stack':
            if t[1] not in mine: continue
            sub.append(re.sub(r'\s+panel\s+\S+', '', ln)); continue
        if t[0] == 'bus':
            if 'panel' in t and t[t.index('panel')+1] != nm: continue
            sub.append(re.sub(r'\s+panel\s+\S+', '', ln)); continue
        if t[0] == 'pin':
            m = re.match(r'^(\w+)\[', t[1]) if len(t) > 1 else None
            if m and m.group(1) in link_refs:
                lk = next((l for l in d.links if l['ref'] == m.group(1)), None)
                if not lk or nm not in (lk['panel'], lk['to']): continue
            elif m and m.group(1) not in mine: continue
        if t[0] == 'route':
            # 이 장의 경로만 (panel 토큰). 토큰 없는 옛 route 는 어느 장 것인지 몰라 버린다 (뷰어가 새로 깐다)
            if 'panel' not in t or t[t.index('panel')+1] != nm: continue
            sub.append(re.sub(r'\s+panel\s+\S+', '', ln)); continue
        sub.append(ln)
    sub.insert(1, f"board   perf {W} {H}" + (f" layers {L}" if L > 1 else ''))
    return '\n'.join(sub) + '\n'


def merge_panel(master_path, nm, subtext):
    """뷰어가 편집한 '한 장'(subtext, 단일 기판 KYLE)을 여러 장 파일(master)에 되돌려 넣는다.
    그 장에 속한 part 좌표·회전·층, stack, bus, route(auto), 그 장 부품의 pin 연결을 바꾸고 다른 장은 건드리지 않는다."""
    d = sol.parse_design(master_path)
    first = d.panel_order[0] if d.panel_order else nm
    pan = {r: (d.parts[r].get('panel') or (d.stacks[r].get('panel') if r in d.stacks else None) or first) for r in d.order}
    mine = {r for r in d.order if pan.get(r) == nm and not d.parts[r].get('link')}
    link_refs = {l['ref'] for l in d.links}
    sub_lines = subtext.split('\n')
    newpart, newstack, newbus, newpins, newroutes = {}, [], [], [], []
    for ln in sub_lines:
        t = ln.split()
        if not t: continue
        if t[0] == 'part' and len(t) > 1 and t[1] in mine: newpart[t[1]] = ln.rstrip()
        elif t[0] == 'stack': newstack.append(ln.rstrip())
        elif t[0] == 'bus': newbus.append(ln.rstrip())
        elif t[0] == 'pin':
            m = re.match(r'^(\w+)\[', t[1]) if len(t) > 1 else None
            if m and (m.group(1) in mine or m.group(1) in link_refs or re.match(r'^J\d+', m.group(1))): newpins.append((m.group(1), ln.rstrip()))
        elif t[0] == 'route': newroutes.append(ln.rstrip())
    sub_stack_refs = {l.split()[1] for l in newstack}
    # 주의: 장별 라이저 이름이 겹치지 않게 장 이름을 붙인다 (J1 → J1A)
    def stk_ref(r): return r if r.endswith(nm) else r + nm
    out = []; master = io.open(master_path, encoding='utf-8').read().split('\n')
    route_nets_mine = set()
    for ln in newroutes:
        t = ln.split()
        if len(t) > 1: route_nets_mine.add(t[1])
    last_part = None
    for ln in master:
        t = ln.split()
        if t and t[0] == 'part' and len(t) > 1 and t[1] in newpart:
            out.append(newpart[t[1]] + f"  panel {nm}"); last_part = len(out) - 1; continue
        if t and t[0] == 'stack' and len(t) > 1 and (t[1].endswith(nm) and ('panel' in t and t[t.index('panel')+1] == nm)): continue
        if t and t[0] == 'bus' and 'panel' in t and t[t.index('panel')+1] == nm: continue
        if t and t[0] == 'pin' and len(t) > 1:
            m = re.match(r'^(\w+)\[', t[1])
            if m and (m.group(1) in mine or (m.group(1).endswith(nm) and re.match(r'^J\d+', m.group(1)))): continue
        if t and t[0] == 'route' and len(t) > 1:
            if 'panel' in t:
                if t[t.index('panel')+1] == nm: continue          # 이 장의 옛 경로는 새 것으로
            elif t[1] in route_nets_mine: continue                  # 장 표시 없는 옛 경로
        if t and t[0] == 'part': last_part = len(out) - 1 if last_part is None else max(last_part, len(out) - 1)
        out.append(ln)
    ins = []
    for ln in newstack:
        t = ln.split(); ins.append(re.sub(r'^stack\s+\S+', f"stack   {stk_ref(t[1])}", ln) + f" panel {nm}")
    for ln in newbus: ins.append(ln + f" panel {nm}")
    for ref, ln in newpins:
        if ref in link_refs: continue                      # 케이블 핀 연결은 master 것이 원본
        if re.match(r'^J\d+', ref): ln = re.sub(r'^pin\s+\S+?\[', f"pin     {stk_ref(ref)}[", ln)
        ins.append(ln)
    for ln in newroutes:                                             # 경로에 장 이름을 붙여 넣는다 (R28 이 장별로 검사)
        t = ln.split()
        if 'panel' in t: ins.append(ln); continue
        m = re.match(r'^(route\s+\S+\s+(?:auto\s+)?(?:front|back)(?:\s+layer\s+\d+)?)(\s.*)$', ln)
        ins.append((m.group(1) + f" panel {nm}" + m.group(2)) if m else ln)
    if last_part is None: out += [''] + ins
    else: out[last_part + 1:last_part + 1] = [''] + ins
    txt = re.sub(r'\n{3,}', '\n\n', '\n'.join(out))
    io.open(master_path, 'w', encoding='utf-8').write(txt if txt.endswith('\n') else txt + '\n')
    return dict(parts=len(newpart), stacks=len(newstack), buses=len(newbus), routes=len(newroutes))


def _place_one(sp, op, W, H, L, channel, seed, iters, search, refine, nm):
    """한 장 배치: search>1 이면 씨앗 여러 개를 실제 배선으로 채점해 최고를 남기고, refine>0 이면 그 배치를 배선 심판으로 다듬는다."""
    if search <= 1 and refine <= 0:
        place.run(sp, op, W=W, H=H, layers=L, seed=seed, iters=iters, channel=channel); return
    import route_eval
    key = lambda r: (r.get('crit', 9) + r.get('err', 9), r['fail'] if r.get('fail') is not None else 10**6, r.get('bends', 0))
    best = None
    for s in range(seed, seed + max(1, search)):
        cand = op + f'.s{s}.txt'
        place.run(sp, cand, W=W, H=H, layers=L, seed=s, iters=iters, channel=channel)
        r = route_eval.evaluate(cand)
        print(f"  [{nm}] 씨앗 {s}: {route_eval.fmt(r)}", flush=True)
        if best is None or key(r) < best[0]: best = (key(r), cand)
    if refine > 0:
        place.refine(best[1], op, evals=refine, seed=seed, verbose=True)
    else:
        import shutil; shutil.copy(best[1], op)
    # 채점에 쓴 배선을 그 파일에 써 넣는다 — 라우터는 핀 순서에 민감해서, 합친 파일에서 새로 깔면 다른(대개 더 나쁜) 결과가 나온다.
    # 파일에 route 가 있으면 뷰어·route_eval 이 그 경로를 먼저 차지하고 나머지만 깐다 (파일이 진실).
    r = route_eval.evaluate(op, write=op)
    print(f"  [{nm}] 확정: {route_eval.fmt(r)}", flush=True)

def _place_panels(path, names, W, H, L, channel=2, seed=0, iters=20000, search=1, refine=0):
    """장마다 그 장의 부품만 떼어 place.py 로 배치하고, 좌표(at/rot/layer)만 원래 파일에 되돌려 쓴다.
    (장이 다르면 서로 겹쳐도 되므로 한꺼번에 배치하면 안 된다)"""
    import tempfile, shutil
    d = sol.parse_design(path)
    first = d.panel_order[0] if d.panel_order else names[0]
    pan = {r: (d.parts[r].get('panel') or first) for r in d.order}
    src_lines = io.open(path, encoding='utf-8').read().split('\n')
    link_refs = {l['ref'] for l in d.links}
    new_stacks = []; new_stackpins = []; new_buses = []; new_routes = []
    newpos = {}
    work = tempfile.mkdtemp(prefix='sol_panel_')
    try:
        for f in ('chips.txt', 'physics.txt'):
            fp = os.path.join(HERE, f)
            if os.path.exists(fp): os.symlink(fp, os.path.join(work, f))
        for nm in names:
            mine = {r for r in d.order if pan.get(r) == nm}
            if not mine: continue
            sub = []
            for ln in src_lines:
                t = ln.split()
                if t and t[0] == 'panel': continue
                if t and t[0] == 'link':
                    # 이 장에 있는 쪽 헤더만 fixed 부품으로 (케이블 반대편은 다른 장)
                    lk = next((l for l in d.links if l['ref'] == t[1]), None)
                    if not lk: continue
                    if lk['panel'] == nm: sub.append(f"part    {lk['ref']:<9}{lk['kind']:<12}at {lk['col']:>2} {lk['row']:>2}  fixed")
                    elif lk['to'] == nm: sub.append(f"part    {lk['ref']:<9}{lk['kind']:<12}at {lk['col2']:>2} {lk['row2']:>2}  fixed")
                    continue
                if t and t[0] == 'part':
                    if t[1] not in mine: continue
                    sub.append(re.sub(r'\s+panel\s+\S+', '', ln)); continue
                if t and t[0] == 'pin':
                    m = re.match(r'^(\w+)\[', t[1]) if len(t) > 1 else None
                    if m and m.group(1) in link_refs:
                        lk = next((l for l in d.links if l['ref'] == m.group(1)), None)
                        if not lk or nm not in (lk['panel'], lk['to']): continue     # 이 장에 없는 케이블의 핀
                    elif m and m.group(1) not in mine: continue
                if t and t[0] == 'route': continue
                sub.append(ln)
            sub.insert(1, f"board   perf {W} {H}" + (f" layers {L}" if L > 1 else ''))
            sp = os.path.join(work, f'{nm}.txt'); io.open(sp, 'w', encoding='utf-8').write('\n'.join(sub) + '\n')
            op = os.path.join(work, f'{nm}_placed.txt')
            _place_one(sp, op, W, H, L, channel, seed, iters, search, refine, nm)
            dp = sol.parse_design(op)
            for r in dp.order:
                if r in mine and r not in link_refs and not r.endswith('_B'): newpos[r] = dp.parts[r]
            for net_, rows_ in dp.buses.items(): new_buses.append((nm, net_, rows_))
            for ln_ in io.open(op, encoding='utf-8').read().split('\n'):        # 채점에 쓴 배선(route) 도 장 표시를 붙여 가져온다
                if ln_.startswith('route'):
                    mm_ = re.match(r'^(route\s+\S+\s+(?:auto\s+)?(?:front|back)(?:\s+layer\s+\d+)?)(\s.*)$', ln_.rstrip())
                    if mm_: new_routes.append(mm_.group(1) + f" panel {nm}" + mm_.group(2))
            for sref, st in dp.stacks.items():                 # 이 장이 필요로 하는 라이저 (이름에 장을 붙여 A/B 가 안 겹치게)
                new_stacks.append((nm, f'{sref}{nm}', st))
                for (rr, pn), nn in dp.pinnet.items():
                    if rr == sref: new_stackpins.append((f'{sref}{nm}', pn, nn))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    old_stack = set(re.findall(r'^\s*stack\s+(\S+)', '\n'.join(src_lines), re.M))
    out = []
    for ln in src_lines:
        t = ln.split()
        if t and t[0] == 'stack': continue                       # 옛 라이저는 버린다 (장별로 새로 세웠다)
        if t and t[0] == 'bus': continue                         # 버스도 장별로 새로
        if t and t[0] == 'route': continue                       # 배선도 장별로 새로 (panel 표시를 붙여 아래에 넣는다)
        if t and t[0] == 'pin' and len(t) > 1:
            mm = re.match(r'^(\w+)\[', t[1])
            if mm and mm.group(1) in old_stack: continue
        if t and t[0] == 'part' and len(t) > 1 and t[1] in newpos:
            p = newpos[t[1]]
            keep = re.search(r'(\s+panel\s+\S+)', ln); pt = keep.group(1) if keep else ''
            extra = ''
            if p.get('near'): extra += f"  near {p['near']}"
            if p.get('fixed'): extra += '  fixed'
            out.append(f"part    {t[1]:<9}{p['chip']:<12}at {p['col']:>2} {p['row']:>2}"
                       + (f"  rot {p['rot']}" if p['rot'] else '')
                       + (f"  layer {p['layer']}" if p.get('layer', 1) != 1 else '') + extra + pt)
            continue
        out.append(ln)
    ins = []
    for nm, sref, st in new_stacks:
        ins.append(f"stack   {sref} {st['kind']} at {st['col']} {st['row']} layers {st['lo']}-{st['hi']} panel {nm}")
    for nm_, net_, rows_ in new_buses:
        ins.append(f"bus     {net_} rows {','.join(map(str, rows_))} panel {nm_}")
    ins += new_routes
    for sref, pn, nn in new_stackpins:
        ins.append(f"pin     {sref}[{pn}:{nn}]")
    if ins: out += [''] + ins
    io.open(path, 'w', encoding='utf-8').write('\n'.join(out))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src'); ap.add_argument('--out', required=True)
    ap.add_argument('--panels', type=int, default=2); ap.add_argument('--by', default='prefix', choices=['prefix', 'graph'])
    ap.add_argument('--board', nargs=2, type=int); ap.add_argument('--layers', type=int)
    ap.add_argument('--hdr', default='HDR20', choices=['HDR8', 'HDR20', 'HDR40'])
    ap.add_argument('--channel', type=int, default=2); ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--iters', type=int, default=20000); ap.add_argument('--no-place', action='store_true')
    ap.add_argument('--search', type=int, default=1, help='장마다 씨앗 N개를 실제 배선으로 채점해 최고를 남긴다 (느림)')
    ap.add_argument('--refine', type=int, default=0, help='장마다 배선을 심판으로 N번 국소 탐색 (느림)')
    a = ap.parse_args()
    info = panelize(a.src, a.out, n=a.panels, by=a.by, board=a.board, layers=a.layers, hdr=a.hdr,
                    place_each=not a.no_place, channel=a.channel, seed=a.seed, iters=a.iters, search=a.search, refine=a.refine)
    print(f"장별 부품 {info['panels']}  ·  장을 넘는 네트 {info['cross']}개  ·  연결 헤더 {info['links']}개  → {a.out}")


if __name__ == '__main__':
    main()
