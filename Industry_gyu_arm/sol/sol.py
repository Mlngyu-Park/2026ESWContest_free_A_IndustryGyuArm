#!/usr/bin/env python3
"""
KYLE — 납땜 설계 언어: 파서 + 검증기 + 타이밍 + 내보내기

    python3 sol.py pe2.txt            검증 리포트
    python3 sol.py pe2.txt --json     pe2.json  (viewer.html 용)
    python3 sol.py pe2.txt --kicad    pe2.kicad.net (넷리스트) + pe2.kicad_pcb (배치 포함 기판)
    python3 sol.py pe2.txt --list     납땜 체크리스트 pe2_list.txt
    python3 sol.py pe2.txt --volt     네트별 전압·전류 표 pe2_volt.txt
    python3 sol.py pe2.txt --all      전부

원칙: 이 프로그램은 설계자의 의도를 모른다.
      chips.txt 의 데이터시트 숫자와 아래 규칙 R1~R12 만으로 판단한다.
"""
import io, os, re, sys, json, fnmatch, math
from collections import defaultdict

# ═════════════════════════════════════════════════════════════
# 1. 부품 라이브러리 파서
# ═════════════════════════════════════════════════════════════
UNIT = {'ns':1e-9,'us':1e-6,'ms':1e-3,'s':1,'pf':1e-12,'nf':1e-9,'uf':1e-6,
        'ua':1e-6,'ma':1e-3,'a':1,'v':1,'mhz':1e6,'khz':1e3,'hz':1}

def strip_comment(raw):
    """줄 시작 '#' 또는 공백 뒤 '#' 만 주석. 'OE#' 처럼 이름에 붙은 # 은 유지.
    color 줄의 '#rrggbb' 는 예외로 살린다."""
    if raw.lstrip().startswith('#'): return ''
    if raw.lstrip().startswith('color'):
        m = re.match(r'^(\s*color\s+\S+\s+#[0-9a-fA-F]{6})', raw)
        return m.group(1).strip() if m else raw.split(' #',1)[0].strip()
    return re.split(r'\s#', raw, 1)[0].strip()

def num(tok):
    """'42ns' → 42e-9,  '3.15' → 3.15"""
    m = re.match(r'^([-+]?\d*\.?\d+)\s*([a-zA-Zµ]*)$', tok.replace('µ','u'))
    if not m: raise ValueError(f"숫자 아님: {tok}")
    v, u = float(m.group(1)), m.group(2).lower()
    return v * UNIT.get(u, 1)

class Chip:
    def __init__(self, name, pkg):
        self.name, self.pkg = name, pkg
        self.pins = {}          # num -> (name, kind, cls)
        self.byname = {}        # name -> num
        self.paths = []         # (in_glob, out_glob, delay_s)
        self.regs = []
        self.funcs = {}          # 출력핀 -> 논리식 (func 문). 시뮬레이터가 쓴다          # (out_globs, in_globs, clk_name)
        self.attr = {}          # vih, vil, voh, vol, cin, icc, level, src...
        self.flags = set()      # 'flash', 'osc', 'decoupling'
        self.flash_we = None
    def npins(self):
        m = re.search(r'(\d+)$', self.pkg)
        return int(m.group(1)) if m else max(self.pins)
    def pin(self, n):
        return self.pins.get(n)

def parse_chips(path):
    chips, cur = {}, None
    for raw in io.open(path, encoding='utf-8'):
        line = strip_comment(raw)
        if not line: continue
        tok = line.split()
        if tok[0] == 'chip':
            cur = Chip(tok[1], tok[2]); chips[cur.name] = cur; continue
        if cur is None: continue
        k = tok[0]
        if k == 'pin':
            n = int(tok[1]); name = tok[2]; kind = tok[3] if len(tok) > 3 else 'pas'
            cls = tok[4] if len(tok) > 4 else ''
            cur.pins[n] = (name, kind, cls); cur.byname[name] = n
        elif k == 'alias':
            base = chips[tok[1]]
            cur.pins = dict(base.pins); cur.byname = dict(base.byname)
            cur.regs = list(base.regs); cur.flags = set(base.flags); cur.flash_we = base.flash_we
        elif k == 'src':
            cur.attr['src'] = line.split(None,1)[1].strip('"')
        elif k == 'level':
            cur.attr['level'] = tok[1]
        elif k in ('vih','vil','voh','vol','cin','cio','icc','iol','res'):
            # 한 줄에 여러 개: vih 3.15 vil 1.35
            i = 0
            while i < len(tok):
                if tok[i] in ('vih','vil','voh','vol','cin','cio','icc','iol','res'):
                    cur.attr[tok[i]] = num(tok[i+1]); i += 2
                else: i += 1
        elif k == 't':
            cur.attr['t_'+tok[1]] = num(tok[2])
        elif k == 'path':
            cur.paths.append((tok[1], tok[2], num(tok[3])))
        elif k == 'reg':
            cur.regs.append((tok[1].split(','), tok[2].split(','), tok[3]))
        elif k == 'func':
            # func <출력핀이름> = <식>   ← 칩의 논리 동작. 시뮬레이터(simulate.py)가 이걸 읽어 계산한다.
            #   식에서 쓸 수 있는 것: 다른 핀 이름, & | ^ ~, 괄호, 0/1, 그리고 add(a,b,cin) 같은 내장 함수
            #   예) func Y0 = ~A0        / func S0 = A0 ^ B0 ^ CIN
            rest = ' '.join(tok[1:])
            if '=' in rest:
                lhs, rhs = rest.split('=', 1)
                cur.funcs[lhs.strip()] = rhs.strip()
        elif k == 'flash':
            cur.flags.add('flash'); cur.flash_we = tok[1]
        elif k in ('osc','decoupling','stackable'):
            cur.flags.add(k)
        elif k == 'sso':
            cur.attr['sso'] = tok[1].split(',')
    return chips

# ═════════════════════════════════════════════════════════════
# 2. 설계 파서 (.txt KYLE)
# ═════════════════════════════════════════════════════════════
class Design:
    def __init__(self):
        self.title = ''; self.board = (60, 50)
        self.chips = {}
        self.supplies = {}        # net -> (volt, imax)
        self.parts = {}           # ref -> dict(chip, col, row, rot)
        self.order = []           # ref 순서
        self.pinnet = {}          # (ref, pin) -> net(str)
        self.nets = defaultdict(list)   # net -> [(ref, pin)]
        self.groups = {}          # net -> group
        self.colors = {}          # group -> color
        self.routes = defaultdict(list)  # net -> [dict(side, pts, layer, auto)]  .5 좌표 = 구멍 사이 통로
        self.settings = dict(wire=0.5, stitch=6, share=1)   # wire 굵기(mm) / 바느질 간격(칸) / 점당 선 수
        self.expects = {}         # net -> dict(hi, lo, i)   AI 가 적은 예상값 (툴은 이걸 검산만 한다)
        self.buses = {}           # net -> [row,...]  전원 버스 선이 가로로 지나는 행 (없으면 기판 위/아래 레일)
        self.pbuses = {}          # panel -> {net: [row,...]}  여러 장일 때 장별 버스
        self.layers = 1           # 기판 층수 (아파트 구조). 1 이면 단층
        self.stacks = {}          # ref -> dict(kind, col, row, lo, hi)  층간 연결 핀헤더
        self.panels = {}          # name -> dict(w, h, layers)  ★ 기판 여러 장 (한 장이면 비어 있다)
        self.panel_order = []     # 선언 순서
        self.links = []           # dict(ref, panel, col, row, kind, to)  장과 장을 잇는 가장자리 핀헤더
        self.sims = []            # ★ 회로 시험 시나리오: dict(name, steps=[[tok...]])  — simulate.py 가 실제로 돌리고 R30 이 대조
        self.errors = []

HERE_DIR = os.path.dirname(os.path.abspath(__file__))
def parse_design(path, text=None):
    d = Design()
    base = os.path.dirname(os.path.abspath(path)); d.base = base
    lines = (text if text is not None else io.open(path, encoding='utf-8').read()).split('\n')
    if not lines or not (lines[0].startswith('#KYLE') or lines[0].startswith('#SOL')):   # #SOL 은 옛 이름 (호환)
        d.errors.append("첫 줄이 #SOL 이 아님")
    cur_sim = None
    SIM_STEPS = ('rom', 'jumper', 'in', 'pulse', 'clock', 'out', 'show')
    for ln, raw in enumerate(lines, 1):
        line = strip_comment(raw)
        if not line: continue
        tok = line.split()
        k = tok[0]
        try:
            if k == 'sim':
                # sim 이름 … end  ← 회로 시험 시나리오. 입력을 넣고 클럭을 쳐서 나온 값을 기대값과 대조한다 (simulate.py, R30)
                #   rom REF 파일 / jumper REF A-B / in REF.핀 값 / in REF.접두어 값 / in 네트 값 / pulse REF.핀 / clock REF.핀 N / out REF.접두어 값 / show REF.접두어
                cur_sim = dict(name=tok[1] if len(tok) > 1 else f'sim{len(d.sims)+1}', steps=[], line=ln); d.sims.append(cur_sim)
            elif cur_sim is not None and k == 'end': cur_sim = None
            elif cur_sim is not None and k in SIM_STEPS:
                need = dict(rom=3, jumper=3, **{'in':3}, pulse=2, clock=3, out=3, show=2)[k]
                if len(tok) < need: raise ValueError(f"sim {k} 문에 값이 모자람")
                cur_sim['steps'].append(tok[:need])
            elif k == 'title':   d.title = line.split(None,1)[1]
            elif k == 'board':
                d.board = (int(tok[2]), int(tok[3]))
                if 'layers' in tok: d.layers = max(1, int(tok[tok.index('layers')+1]))
            elif k == 'panel':
                # panel A perf 54 33 layers 2   ← 기판 한 장. 여러 장을 한 파일에 담는다
                name = tok[1]; w, h = int(tok[3]), int(tok[4])
                lay = max(1, int(tok[tok.index('layers')+1])) if 'layers' in tok else 1
                d.panels[name] = dict(w=w, h=h, layers=lay)
                if name not in d.panel_order: d.panel_order.append(name)
                d.board = (w, h); d.layers = max(d.layers, lay)
            elif k == 'link':
                # link L1 HDR8 at A 52 4 to B    ← A 장의 (52,4) 가장자리 핀헤더가 B 장으로 간다
                i = tok.index('at'); j = tok.index('to')
                c2 = int(tok[j+2]) if len(tok) > j+3 and tok[j+2].lstrip('-').isdigit() else 1
                r2 = int(tok[j+3]) if len(tok) > j+3 and tok[j+3].lstrip('-').isdigit() else int(tok[i+3])
                d.links.append(dict(ref=tok[1], kind=tok[2], panel=tok[i+1],
                                    col=int(tok[i+2]), row=int(tok[i+3]), to=tok[j+1], col2=c2, row2=r2))
                # 케이블 양 끝의 헤더 두 개를 실제 부품으로 등록한다: L1 (A 장) 과 L1_B (B 장). 핀 연결(pin L1[k:net])은 둘 다에 적용.
                if tok[2] in d.chips:
                    d.parts[tok[1]] = dict(chip=tok[2], col=int(tok[i+2]), row=int(tok[i+3]), rot=0,
                                           layer=1, panel=tok[i+1], link=(tok[i+1], tok[j+1]), fixed=True)
                    d.order.append(tok[1])
                    d.parts[tok[1]+'_B'] = dict(chip=tok[2], col=c2, row=r2, rot=0,
                                           layer=1, panel=tok[j+1], link=(tok[i+1], tok[j+1]), fixed=True, twin=tok[1])
                    d.order.append(tok[1]+'_B')
            elif k == 'include':
                d.chips.update(parse_chips(os.path.join(base, tok[1])))
            elif k == 'supply':
                v = num(tok[2]); imax = num(tok[4]) if len(tok) > 4 else None
                d.supplies[tok[1]] = (v, imax)
            elif k == 'part':
                ref, chip = tok[1], tok[2]
                if chip not in d.chips: d.errors.append(f"{ln}행: 모르는 부품 {chip}"); continue
                i = tok.index('at'); col, row = int(tok[i+1]), int(tok[i+2])
                rot = int(tok[tok.index('rot')+1]) if 'rot' in tok else 0
                lay = int(tok[tok.index('layer')+1]) if 'layer' in tok else 1
                near = tok[tok.index('near')+1] if 'near' in tok else None     # 배치 힌트: 이 부품 옆에 (디커플링 캡 등)
                panel = tok[tok.index('panel')+1] if 'panel' in tok else None  # 이 부품이 올라가는 기판 장
                fixed = 'fixed' in tok                                          # 배치 힌트: 배치기가 옮기지 않는다 (가장자리 커넥터 등)
                d.parts[ref] = dict(chip=chip, col=col, row=row, rot=rot, layer=lay, near=near, fixed=fixed, panel=panel); d.order.append(ref)
            elif k == 'pin':
                m = re.match(r'^(\w+)\[(\d+):([\w#]+)\]$', tok[1])
                if not m: d.errors.append(f"{ln}행: 포인터 형식 아님 {tok[1]}"); continue
                ref, pin, net = m.group(1), int(m.group(2)), m.group(3)
                if ref not in d.parts: d.errors.append(f"{ln}행: 배치 안 된 부품 {ref}"); continue
                key = (ref, pin)
                if key in d.pinnet and d.pinnet[key] != net:
                    d.errors.append(f"{ln}행: {ref}[{pin}] 이 네트 {d.pinnet[key]} 와 {net} 둘 다")
                d.pinnet[key] = net
                if net != 'NC': d.nets[net].append(key)
                # 장간 케이블: L1 의 핀은 반대편 헤더 L1_B 에도 같은 네트
                if m and (m.group(1) + '_B') in d.parts and m.group(3) != 'NC':
                    tw = m.group(1) + '_B'; pn_ = int(m.group(2)); nt_ = m.group(3)
                    if (tw, pn_) not in d.pinnet: d.pinnet[(tw, pn_)] = nt_; d.nets[nt_].append((tw, pn_))
            elif k == 'stack':
                # stack J1 HDR20 at 2 10 layers 1-2   ← 1층과 2층을 잇는 20핀 헤더
                ref, kind = tok[1], tok[2]
                i = tok.index('at'); col, row = int(tok[i+1]), int(tok[i+2])
                lo, hi = 1, 2
                if 'layers' in tok:
                    m = re.match(r'(\d+)\s*-\s*(\d+)', tok[tok.index('layers')+1])
                    if m: lo, hi = int(m.group(1)), int(m.group(2))
                pnl = tok[tok.index('panel')+1] if 'panel' in tok else None      # 여러 장이면 어느 장의 라이저인지
                d.stacks[ref] = dict(kind=kind, col=col, row=row, lo=lo, hi=hi, panel=pnl)
                if kind in d.chips:
                    d.parts[ref] = dict(chip=kind, col=col, row=row, rot=0, layer=lo, stack=(lo,hi), panel=pnl)
                    if ref not in d.order: d.order.append(ref)      # part 줄로도 적힌 옛 파일: 두 번 세면 R23 이 '자기 자신과 겹침' 을 낸다
            elif k == 'bus':
                # bus GND rows 12,18,34   ← 이 행들에 GND 버스 선을 가로로 깐다. 각 GND 핀은 가장 가까운 버스로.
                rows_ = [int(x) for x in tok[3].split(',')]
                if 'panel' in tok:                                   # bus GND rows 3,9 panel A  ← 장별 버스
                    d.pbuses.setdefault(tok[tok.index('panel')+1], {})[tok[1]] = rows_
                    d.buses.setdefault(tok[1], rows_)                # 단일 기판 코드 경로용 대표값
                else:
                    d.buses[tok[1]] = rows_
            elif k == 'expect':
                # expect <net> <HIGH V> <LOW V> [<LOW 때 흡수전류 mA>]      예: expect 105 4.4 0.1
                #        expect VCC 5.0            (전원은 한 값)
                net = tok[1]; hi = num(tok[2]); lo = num(tok[3]) if len(tok) > 3 else hi
                cur = num(tok[4]) if len(tok) > 4 else None
                d.expects[net] = dict(hi=hi, lo=lo, i=cur)
            elif k == 'group': d.groups[tok[1]] = tok[2]
            elif k == 'color': d.colors[tok[1]] = tok[2]
            elif k == 'route':
                # route <net> [auto] <front|back> [layer N] (c,r) (c,r) | (c,r) ...
                #   '|' = 그 점의 구멍을 관통해 반대면으로 (바느질). 좌표의 .5 = 구멍 사이 통로. auto = 자동배선이 쓴 것
                i = 1; net = tok[i]; i += 1
                auto = False
                if tok[i] == 'auto': auto = True; i += 1
                side = tok[i]; i += 1
                if side not in ('front','back'): raise ValueError(f"route 면은 front/back 이어야 함: {side}")
                layer = 1; rpanel = None
                while i < len(tok) and tok[i] in ('layer', 'panel'):
                    if tok[i] == 'layer': layer = int(tok[i+1])
                    else: rpanel = tok[i+1]                      # 여러 장: 이 경로가 놓인 장 (route … panel A)
                    i += 2
                rest = ' '.join(tok[i:])
                toks = re.findall(r'\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)|(\w+\.\d+)|(\|)', rest)
                seg, cur = [], side
                for a, b, c, bar in toks:
                    if bar:
                        if seg: d.routes[net].append(dict(side=cur, pts=seg, layer=layer, auto=auto, panel=rpanel)); last = seg[-1]; seg = [last]
                        cur = 'front' if cur == 'back' else 'back'
                    else:
                        seg.append((float(a), float(b)) if a else c)
                if len(seg) >= 2: d.routes[net].append(dict(side=cur, pts=seg, layer=layer, auto=auto, panel=rpanel))
            elif k == 'wire':   d.settings['wire'] = float(re.sub(r'[^0-9.]','', tok[1]))
            elif k == 'stitch': d.settings['stitch'] = int(tok[1])
            elif k == 'share':  d.settings['share'] = int(tok[1])
        except Exception as e:
            d.errors.append(f"{ln}행: {e}  ← {line}")
    return d

# ═════════════════════════════════════════════════════════════
# 3. 기하 — DIP 핀 → 기판 구멍 (실물 치수)
# ═════════════════════════════════════════════════════════════
PITCH = 2.54
def pkg_geom(pkg):
    """패키지 → (핀수, 열폭(구멍수), 몸통폭 mm, 몸통높이 mm)"""
    m = re.match(r'([A-Z]+)(\d+)', pkg)
    fam, n = m.group(1), int(m.group(2))
    if fam == 'DIP':
        w = 6 if n == 32 else 3      # 600mil=6칸, 300mil=3칸
        return n, w, w*PITCH, 4.0
    if fam == 'CAP':  return 2, 1, 2.54, 5.0
    if fam == 'RES':  return 2, 0, 2.54, 3.0     # 세로 4칸 (핀1 위, 핀2 아래 +3)
    if fam == 'SIP':  return n, 0, 2.54, 11.0      # 1열 핀헤더 (층간 연결 시 11mm 높이)
    if fam == 'CONN': return n, 1, 5.08, 8.5
    return n, 3, 7.62, 4.0

def pin_hole(part, pin):
    """(ref 배치, 핀번호) → (col,row) 구멍.  DIP: 1번 좌상단, 반시계."""
    pkg = part['_pkg']; n, w, *_ = pkg_geom(pkg)
    col, row, rot = part['col'], part['row'], part['rot']
    fam = pkg[:3]
    if fam == 'DIP':
        half = n // 2
        dx, dy = (0, pin-1) if pin <= half else (w, 2*half - pin)
    elif fam == 'CAP':
        dx, dy = (0, pin-1)
    elif fam == 'RES':
        dx, dy = (0, 0 if pin == 1 else 3)
    elif fam in ('SIP','CON'):
        # CONN30: 2열 15행 (2.54)
        if fam == 'CON':
            dx, dy = ((pin-1) % 2, (pin-1)//2)
        else:
            dx, dy = (0, pin-1)
    else:
        dx, dy = (0, pin-1)
    # 회전 (1번 핀 기준)
    for _ in range(rot // 90):
        dx, dy = -dy, dx
    return col + dx, row + dy

# ═════════════════════════════════════════════════════════════
# 4. 검증 규칙 R1 ~ R12
# ═════════════════════════════════════════════════════════════
class Report:
    def __init__(self): self.items = []
    def add(self, rule, level, msg):
        self.items.append((rule, level, msg))
    def count(self, level): return sum(1 for r in self.items if r[1] == level)

def pin_info(d, ref, pin):
    c = d.chips[d.parts[ref]['chip']]
    p = c.pin(pin)
    return c, (p if p else (f'?{pin}', 'unknown', ''))

def verify(d):
    R = Report()
    for e in d.errors: R.add('R0', '오류', e)
    sup = set(d.supplies)
    vcc_nets = {n for n,(v,_) in d.supplies.items() if v > 0}
    gnd_nets = {n for n,(v,_) in d.supplies.items() if v == 0}

    # R1 핀 범위 / 정의
    for (ref, pin) in d.pinnet:
        c = d.chips[d.parts[ref]['chip']]
        if pin < 1 or pin > c.npins():
            R.add('R1','오류',f"{ref}[{pin}] — {c.pkg} 는 {c.npins()}핀")
        elif pin not in c.pins:
            R.add('R1','오류',f"{ref}[{pin}] — chips.txt 에 정의 안 된 핀")

    # 네트별 드라이버/수신 분류
    netinfo = {}
    for net, plist in d.nets.items():
        drivers, ins, clks, vccp, gndp, pas, bis = [], [], [], [], [], [], []
        for ref, pin in plist:
            c, (nm, kind, cls) = pin_info(d, ref, pin)
            tag = f"{ref}.{nm}({pin})"
            {'out':drivers,'in':ins,'clk':clks,'vcc':vccp,'gnd':gndp,'pas':pas,'bi':bis,'nc':pas,'unknown':pas}[kind].append((ref,pin,tag))
        netinfo[net] = dict(drivers=drivers, ins=ins, clks=clks, vccp=vccp, gndp=gndp, pas=pas, bis=bis)

    # R2 VCC-GND 쇼트 / 전원 네트에 출력
    for net, ni in netinfo.items():
        if net in vcc_nets and (ni['gndp']): R.add('R2','치명',f"네트 {net}(VCC) 에 GND 핀: {[t for *_,t in ni['gndp']]}")
        if net in gnd_nets and (ni['vccp']): R.add('R2','치명',f"네트 {net}(GND) 에 VCC 핀: {[t for *_,t in ni['vccp']]}")
        if net in sup and ni['drivers']:
            R.add('R2','치명',f"전원 네트 {net} 에 출력 핀이 붙음: {[t for *_,t in ni['drivers']]}")
        if net not in sup and (ni['vccp'] or ni['gndp']):
            R.add('R5','오류',f"전원 핀이 전원 아닌 네트 {net} 에: {[t for *_,t in ni['vccp']+ni['gndp']]}")

    # R3 출력 충돌  (bi 는 플래시 DQ — 읽기 전용 회로에서는 출력으로 본다)
    for net, ni in netinfo.items():
        outs = ni['drivers'] + ni['bis']
        if len(outs) > 1:
            R.add('R3','치명',f"네트 {net} 에 출력 {len(outs)}개: {[t for *_,t in outs]}")

    # R4 플로팅 입력 (배치된 부품의 in/clk 핀 중 pinnet 에 없는 것)
    for ref in d.order:
        c = d.chips[d.parts[ref]['chip']]
        for pin, (nm, kind, cls) in c.pins.items():
            if kind in ('in','clk') and (ref,pin) not in d.pinnet:
                R.add('R4','경고',f"{ref}.{nm}({pin}) 입력이 아무 데도 안 붙음 (플로팅)")
        # 전원핀 누락
        for pin, (nm, kind, cls) in c.pins.items():
            if kind in ('vcc','gnd') and (ref,pin) not in d.pinnet:
                R.add('R5','오류',f"{ref}.{nm}({pin}) 전원 핀 미연결")

    # R10 드라이버 없는 네트 (입력만 모임) / 수신 없는 출력
    for net, ni in netinfo.items():
        if net in sup: continue
        outs = ni['drivers'] + ni['bis']
        if not outs and not ni['pas'] and (ni['ins'] or ni['clks']):
            R.add('R10','경고',f"네트 {net}: 드라이버 없음. 입력만 {len(ni['ins']+ni['clks'])}개")
        if outs and not (ni['ins'] or ni['clks'] or ni['pas'] or len(outs) > 1):
            R.add('R10','정보',f"네트 {net}: 출력 {outs[0][2]} 가 아무 데도 안 감 (미사용 출력이면 OK)")

    # 풀업 찾기: 네트에 붙은 저항의 반대쪽 핀이 VCC 네트면 풀업
    def pullups(net):
        out = []
        for ref, pin, tag in netinfo[net]['pas']:
            c = d.chips[d.parts[ref]['chip']]
            if 'res' not in c.attr: continue
            other = 2 if pin == 1 else 1
            onet = d.pinnet.get((ref, other))
            if onet in vcc_nets: out.append((ref, c.attr['res'], d.supplies[onet][0]))
        return out
    pull_total = 0.0
    # R6 용량 부하  &  R11 레벨 불일치  &  R13 풀업 전류  &  R14 풀업 RC
    for net, ni in netinfo.items():
        if net in sup: continue
        outs = ni['drivers'] + ni['bis']
        if len(outs) != 1: continue
        dref, dpin, dtag = outs[0]
        dc = d.chips[d.parts[dref]['chip']]
        pus = pullups(net)
        load = 0.0
        for ref, pin, tag in ni['ins'] + ni['clks']:
            c = d.chips[d.parts[ref]['chip']]
            load += c.attr.get('cin', 5e-12)
            # R11
            voh = dc.attr.get('voh'); vih = c.attr.get('vih')
            if voh is not None and vih is not None and voh < vih:
                if pus:
                    vsup = pus[0][2]
                    R.add('R11','정보',f"레벨: {dtag} VOH(min) {voh}V < {tag} VIH {vih}V 이지만 풀업 {pus[0][1]/1e3:.1f}kΩ→{vsup}V 로 확보")
                else:
                    R.add('R11','경고',f"레벨: {dtag} VOH(min) {voh}V < {tag} VIH(min) {vih}V — 규격상 HIGH 보장 안 됨 (풀업 저항으로 해결 가능)")
        if pus:
            # R13: LOW 일 때 드라이버가 풀업 전류를 삼켜야 한다
            isink = sum(v/r for _, r, v in pus); pull_total += isink
            iol = dc.attr.get('iol')
            if iol is not None:
                lvl = '경고' if isink > iol else '정보'
                R.add('R13', lvl, f"풀업 전류: {dtag} 가 LOW 때 {isink*1e3:.2f}mA 삼킴 / 허용 IOL {iol*1e3:.1f}mA ({'초과' if isink>iol else 'OK'})")
            # R14: 풀업만으로 올릴 때의 RC (드라이버가 능동 구동하면 이보다 빠름)
            rc = (1/sum(1/r for _, r, _ in pus)) * (load + 2e-12*max(1,len(ni['ins'])+len(ni['clks'])))
            R.add('R14','정보',f"풀업 RC {rc*1e9:.0f}ns (네트 {net}) — 능동 구동 시 참고값")
        wire = 2e-12 * max(1, len(ni['ins'])+len(ni['clks']))   # 배선 약 2pF/수신
        if load + wire > 50e-12:
            R.add('R6','경고',f"네트 {net}: {dtag} 부하 {((load+wire)*1e12):.0f}pF > 50pF (데이터시트 지연 규격 밖)")

    # R9 플래시 WE#
    for ref in d.order:
        c = d.chips[d.parts[ref]['chip']]
        if 'flash' in c.flags:
            we = c.byname[c.flash_we]
            net = d.pinnet.get((ref, we))
            if net not in vcc_nets:
                R.add('R9','경고',f"{ref}.{c.flash_we} 가 VCC 아님 ({net}) — 오작동 시 지워질 수 있음")

    # R12 디커플링
    nchip = sum(1 for r in d.order if d.chips[d.parts[r]['chip']].pkg.startswith('DIP') and 'osc' not in d.chips[d.parts[r]['chip']].flags)
    ncap  = sum(1 for r in d.order if 'decoupling' in d.chips[d.parts[r]['chip']].flags)
    if ncap < nchip:
        R.add('R12','경고',f"0.1µF {ncap}개 < 칩 {nchip}개 — 칩마다 1개")
    else:
        R.add('R12','정보',f"디커플링 0.1µF {ncap}개 / 칩 {nchip}개  OK")

    # R8 전류
    for net,(v,imax) in d.supplies.items():
        if v <= 0 or imax is None: continue
        tot = 0.0
        for ref in d.order:
            c = d.chips[d.parts[ref]['chip']]
            tot += c.attr.get('icc', 0)
        tot += pull_total
        lvl = '경고' if tot > imax else '정보'
        R.add('R8', lvl, f"{net} 정지전류 합 {tot*1e3:.1f}mA (풀업 최악 {pull_total*1e3:.0f}mA 포함) / 한계 {imax:.1f}A  ({'초과' if tot>imax else 'OK'}; 동적 전류 별도)")

    # 전압·전류 표 + R15 (AI 예상 대조)
    volts = compute_volts(d, netinfo)
    for net, ex in d.expects.items():
        v = volts.get(net)
        if not v:
            R.add('R15','경고',f"expect {net}: 그런 네트 없음"); continue
        bad = []
        if v['kind'] == 'supply':
            if abs(ex['hi'] - v['v']) > 0.05: bad.append(f"전원 {v['v']}V 인데 예상 {ex['hi']}V")
        elif v['kind'] == 'signal':
            TOL = 0.1   # 예상값은 보장 범위 안이면 타당. ±0.1V 는 측정 오차 수준
            if ex['hi'] > v['vmax'] + TOL: bad.append(f"HIGH 예상 {ex['hi']}V 가 전원 {v['vmax']:.2f}V 보다 높음")
            elif ex['hi'] < v['voh'] - TOL: bad.append(f"HIGH 예상 {ex['hi']}V 가 보장 최소 {v['voh']:.2f}V 보다 낮음 (AI 가 회로를 비관적으로 봄)")
            if ex['lo'] < -TOL: bad.append(f"LOW 예상 {ex['lo']}V 가 0 아래")
            elif ex['lo'] > v['vol'] + TOL: bad.append(f"LOW 예상 {ex['lo']}V 가 보장 최대 {v['vol']:.2f}V 보다 높음")
            if ex['hi'] < v['vih'] - 1e-9: bad.append(f"AI 가 예상한 HIGH {ex['hi']}V 로는 수신 VIH {v['vih']:.2f}V 를 못 넘음")
            if ex['i'] is not None and v['i_low'] is not None:
                if abs(ex['i'] - v['i_low']) > max(0.25*v['i_low'], 0.05e-3): bad.append(f"LOW 흡수전류 예상 {ex['i']*1e3:.2f}mA vs 계산 {v['i_low']*1e3:.2f}mA")
        else:
            bad.append("드라이버 없는 네트라 전압을 정할 수 없음")
        if bad: R.add('R15','경고',f"AI 예상 ≠ 계산 (네트 {net}): " + '; '.join(bad))
    n_ok = sum(1 for net in d.expects if net in volts and not any(r for r in R.items if r[0]=='R15' and f'네트 {net})' in r[2]))
    if d.expects: R.add('R15','정보',f"AI 전압 예상 {len(d.expects)}개 중 {n_ok}개 타당, {len(d.expects)-n_ok}개 불일치")
    # R30 회로 시험 (sim … end): KYLE 의 pin + chips.txt 의 func/reg 만으로 실제로 돌려 out 기대값과 대조 — AI 는 관여하지 않는다
    if d.sims:
        try:
            import simulate
            for sc in d.sims:
                res = simulate.run_scenario(d, sc, getattr(d, 'base', '.'))
                for r in res:
                    lvl = '오류' if r.get('ok') is False else ('경고' if r.get('warn') else '정보')
                    R.add('R30', lvl, f"sim {sc['name']}: {r['msg']}")
        except Exception as e:
            R.add('R30', '경고', f"sim 실행 실패: {e}")
    # R23 부품 겹침 — 같은 구멍을 두 부품이 쓰거나, 몸통(풋프린트)이 겹침
    occ = {}
    for ref in d.order:
        p = d.parts[ref]; c = d.chips[p['chip']]; p['_pkg'] = c.pkg
        Lz = p.get('layer', 1)
        n = pkg_geom(c.pkg)[0]
        cells = set()
        holes = [pin_hole(p, k) for k in range(1, n+1)]
        cs = [h[0] for h in holes]; rs = [h[1] for h in holes]
        # DIP/CONN 은 두 열 사이도 몸통이 차지한다
        if c.pkg.startswith(('DIP','CONN')):
            for x in range(min(cs), max(cs)+1):
                for y in range(min(rs), max(rs)+1): cells.add((x,y))
        else:
            cells.update(holes)
        # 장(panel)이 다르면 다른 기판이라 같은 자리를 써도 된다. 층이 달라도 된다 (아파트).
        pn = p.get('panel') or (d.stacks[ref].get('panel') if ref in d.stacks else None) or (d.panel_order[0] if d.panel_order else None)
        for cell in cells:
            key = (pn, Lz) + cell
            if key in occ:
                R.add('R23','오류',f"부품 겹침({Lz}층{(', '+pn+' 장') if pn else ''}): {occ[key]} 와 {ref} 가 구멍 {cell} 을 같이 씀")
                break
            occ[key] = ref
        # 기판 밖: 여러 장이면 R29 가 장별 크기로 본다
        if not d.panels and any(x<0 or y<0 or x>=d.board[0] or y>=d.board[1] for x,y in cells):
            R.add('R23','오류',f"{ref} 가 기판 밖으로 나감 (기판 {d.board[0]}x{d.board[1]})")
    # R28 배선 규칙 — 뷰어 라우터와 '독립적으로' 다시 구현한 검사 (교차검증). 반칸 격자: 좌표×2, 짝수=구멍, 홀수=통로.
    #   ① 한 면(층×앞뒤)의 한 칸에 한 선  ② 남의 핀 구멍 금지  ③ 앞면은 부품 아래 금지  ④ 핀 접촉은 뒷면만
    #   ⑤ 관통은 빈 구멍에서만  ⑥ 버스 줄의 구멍은 버스 것(통로는 직각 횡단 OK)  ⑦ 전원 스텁 칸 금지  ⑧ 비스듬한 선 금지
    if d.routes:
      # 여러 장이면 장마다 따로 검사한다 (장이 다르면 같은 좌표라도 다른 기판이다). route 의 panel 토큰이 장을 정한다.
      _first = d.panel_order[0] if d.panel_order else None
      for PN in (d.panel_order if d.panels else [None]):
            _pan = lambda r: (d.parts[r].get('panel') or (d.stacks[r].get('panel') if r in d.stacks else None) or _first)
            order_ = [r for r in d.order if PN is None or _pan(r) == PN]
            oset_ = set(order_)
            routes_ = {n: [rt for rt in rs if PN is None or (rt.get('panel') or _first) == PN] for n, rs in d.routes.items()}
            routes_ = {n: rs for n, rs in routes_.items() if rs}
            buses_ = (d.pbuses.get(PN, {}) if PN else d.buses)
            board_ = ((d.panels[PN]['w'], d.panels[PN]['h']) if PN else d.board)
            if not routes_: continue
            tag_ = f"[{PN} 장] " if PN else ''
            NL = max(1, d.layers)
            foot, pinhole = set(), {}          # (층,x2,y2)
            for ref in order_:
                p = d.parts[ref]; c = d.chips[p['chip']]; p['_pkg'] = c.pkg; L = p.get('layer',1)
                holes = [pin_hole(p,k) for k in range(1, pkg_geom(c.pkg)[0]+1)]
                lays = range(d.stacks[ref]['lo'], d.stacks[ref]['hi']+1) if ref in d.stacks else [L]
                for Lz in lays:
                    for k,h in enumerate(holes,1):
                        info = c.pin(k)
                        if info is None or info[1] == 'undef': continue   # 정의 안 된/다리 없는 자리(OSC 풀캔 등) — 구멍은 비어 있다
                        pinhole[(Lz, h[0]*2, h[1]*2)] = (ref,k,d.pinnet.get((ref,k)))
                    if c.pkg.startswith(('DIP','CONN')):
                        cs=[h[0]*2 for h in holes]; rs=[h[1]*2 for h in holes]
                        for x in range(min(cs)-1, max(cs)+2):
                            for y in range(min(rs)-1, max(rs)+2): foot.add((Lz,x,y))
                    else:
                        for h in holes: foot.add((Lz,h[0]*2,h[1]*2))
            busrow = {}                        # (층,y2) -> net
            for net, rows in buses_.items():
                for r in rows:
                    for Lz in range(1, NL+1): busrow[(Lz, r*2)] = net
            # 전원 스텁은 이제 route 문장으로 명시된다 (뷰어가 신호와 같은 규칙으로 배선) → 별도 추정 없이 ①~⑥ 으로 검사
            occ = {}; msgs = []; nseg = 0; overlap = {}
            def bad(m): msgs.append(m)
            def resolve_pt(pt):
                if isinstance(pt, str):
                    ref, pin = pt.rsplit('.',1); p = d.parts[ref]; p['_pkg'] = d.chips[p['chip']].pkg; return pin_hole(p, int(pin))
                return pt
            for net, rs in routes_.items():
                mypins = set()
                for r, pn in d.nets.get(net, []):
                    if r not in oset_: continue
                    h = pin_hole(dict(d.parts[r], _pkg=d.chips[d.parts[r]['chip']].pkg), pn)
                    lays_ = range(d.stacks[r]['lo'], d.stacks[r]['hi']+1) if r in d.stacks else [d.parts[r].get('layer',1)]   # 라이저 핀은 모든 층에 있다
                    for Lz in lays_: mypins.add((Lz, h[0]*2, h[1]*2))
                ends = []
                for si, rt in enumerate(rs):
                    side, L = rt['side'], rt.get('layer',1); pts = [resolve_pt(x) for x in rt['pts']]; nseg += 1
                    for i in range(1, len(pts)):
                        (x0,y0),(x1,y1) = pts[i-1], pts[i]
                        if x0 != x1 and y0 != y1: bad(f"네트 {net} 비스듬한 선 ({x0},{y0})→({x1},{y1}) — 가로/세로만"); continue
                        X0,Y0,X1,Y1 = (int(round(v*2)) for v in (x0,y0,x1,y1))
                        dx, dy = (X1>X0)-(X1<X0), (Y1>Y0)-(Y1<Y0); n = abs(X1-X0)+abs(Y1-Y0)
                        for k in range(n+1):
                            x2, y2 = X0+dx*k, Y0+dy*k
                            if not (0 <= x2 < board_[0]*2 and 0 <= y2 < board_[1]*2): bad(f"네트 {net} 선이 기판 밖 ({x2/2},{y2/2})"); break
                            key = (L, side, x2, y2); hole = (x2%2==0 and y2%2==0)
                            share = max(1, int(d.settings.get('share', 1)))
                            holders = occ.setdefault(key, set())
                            if net not in holders and holders and not (hole and len(holders) < share):
                                o = sorted(holders)[0]; pk = (net, o, L, side)
                                if pk not in overlap: overlap[pk] = (x2/2, y2/2, 0)
                                overlap[pk] = (overlap[pk][0], overlap[pk][1], overlap[pk][2]+1)
                            holders.add(net)
                            ph = pinhole.get((L,x2,y2)) if hole else None
                            if ph and (L,x2,y2) not in mypins: bad(f"네트 {net} 선이 {ph[0]}.{ph[1]}(네트 {ph[2]}) 핀 구멍 ({x2//2},{y2//2}) 위를 지남 — 쇼트")
                            if side == 'front' and (L,x2,y2) in foot: bad(f"네트 {net} 앞면 선이 부품 아래 ({x2/2},{y2/2}) 를 지남 — 소켓·몸통 밑은 납땜 불가")
                            bn = busrow.get((L,y2))
                            if side == 'back' and hole and bn and bn != net and (L,x2,y2) not in mypins: bad(f"네트 {net} 뒷면 선이 {bn} 버스 줄의 납땜 구멍 ({x2//2},{y2//2}) 위를 지남 — 통로로만 직각 횡단")
                    # 끝점: 자기 핀이면 뒷면이어야 한다
                    for pt in (pts[0], pts[-1]):
                        k3 = (L, int(round(pt[0]*2)), int(round(pt[1]*2))); ends.append((k3, side))
                        if k3[1] % 2 or k3[2] % 2: bad(f"네트 {net} 선 끝 ({pt[0]},{pt[1]}) 이 구멍이 아님 — 선은 구멍(납땜점)에서만 시작·끝·갈라진다")
                        if k3 in mypins and side != 'back': bad(f"네트 {net} 선이 앞면에서 핀 ({pt[0]},{pt[1]}) 에 닿음 — 핀 접촉은 뒷면(납땜면)만")
                    # 관통: 다음 구간이 같은 점에서 반대면으로 이어지면 그 구멍 검사
                    if si+1 < len(rs):
                        nx = rs[si+1]; npts = [resolve_pt(x) for x in nx['pts']]
                        if npts and npts[0] == pts[-1] and nx['side'] != side and nx.get('layer',1) == L:
                            x2,y2 = int(round(pts[-1][0]*2)), int(round(pts[-1][1]*2))
                            if x2%2 or y2%2: bad(f"네트 {net} 관통점 ({pts[-1][0]},{pts[-1][1]}) 이 구멍이 아님")
                            elif (L,x2,y2) in pinhole: bad(f"네트 {net} 이 핀 구멍 ({x2//2},{y2//2}) 에서 관통 — 빈 구멍에서만")
                            elif (L,x2,y2) in foot: bad(f"네트 {net} 이 부품 아래 구멍 ({x2//2},{y2//2}) 에서 관통 — 앞면이 막혀 있음")
                # 끊긴 끝: 자기 핀도 아니고, 같은 네트 다른 구간과 만나지도 않는 끝점
                from collections import Counter
                cnt = Counter(k for k,_ in ends)
                # 완결성: 이 네트의 모든 핀에 경로가 닿아야 한다 (일부 핀만 이어진 경로는 '미배선이 감춰진 것')
                touched = {k for k,_ in ends}
                # 전원 핀이 버스 줄 위에 있으면 스텁이 없다 (그 자체로 연결) → 빠진 핀으로 세지 않는다
                missing = [f"({k[1]//2},{k[2]//2})" for k in mypins if k not in touched and not (net in d.supplies and busrow.get((k[0],k[2])) == net)]
                if missing: R.add('R28','경고', f"네트 {net} 경로가 핀 {len(missing)}개에 닿지 않음 {' '.join(missing[:4])} — 미배선")
                onbus = lambda k3: (net in d.supplies and busrow.get((k3[0], k3[2])) == net and k3[1] % 2 == 0)   # 전원 스텁은 버스 줄 구멍에서 끝나도 된다
                for k3,_ in ends:
                    if k3 not in mypins and cnt[k3] < 2 and not onbus(k3):
                        R.add('R28','경고',f"네트 {net} 경로 끝 ({k3[1]/2},{k3[2]/2}) 이 핀도 아니고 다른 구간과 이어지지도 않음 — 끊긴 선"); break
            for (n1,n2,L,side),(x,y,c) in overlap.items(): msgs.append(f"네트 {n1} 와 {n2} 가 {L}층 {side} ({x},{y}) 부터 {c}칸을 같이 씀 — 선 겹침")
            for m in msgs[:12]: R.add('R28','오류', tag_ + m)
            if len(msgs) > 12: R.add('R28','오류', f"… 배선 규칙 위반 {len(msgs)-12}건 더")
            R.add('R28','정보', f"{tag_}배선 검사: {len(routes_)}네트 {nseg}구간, 규칙 위반 {len(msgs)}건 (뷰어와 독립 구현)")
    # R29 여러 장(panel) 규칙: 장마다 부품이 기판 안에 있고, 장을 넘는 네트는 link 헤더를 지나야 한다
    if d.panels:
        R.add('R29','정보', f"기판 {len(d.panels)}장: " + ', '.join(f"{n}({v['w']}x{v['h']}, {v['layers']}층)" for n, v in d.panels.items()))
        first = d.panel_order[0] if d.panel_order else None
        pan = {r: (d.parts[r].get('panel') or (d.stacks[r].get('panel') if r in d.stacks else None) or first) for r in d.order}
        for r in d.order:
            pn = pan[r]
            if pn not in d.panels:
                R.add('R29','오류', f"{r} 의 panel {pn} 이 선언되지 않았다"); continue
            pv = d.panels[pn]; p = d.parts[r]; p['_pkg'] = d.chips[p['chip']].pkg
            holes = [pin_hole(p, k) for k in range(1, pkg_geom(p['_pkg'])[0]+1)]
            if any(x < 0 or y < 0 or x >= pv['w'] or y >= pv['h'] for x, y in holes):
                R.add('R29','오류', f"{r} 가 {pn} 장({pv['w']}x{pv['h']}) 밖으로 나감")
            if p.get('layer', 1) > pv['layers']:
                R.add('R29','오류', f"{r} 의 층 {p['layer']} 이 {pn} 장의 층수 {pv['layers']} 를 넘는다")
        # 장이 다른 부품끼리는 겹쳐도 된다 (다른 기판이니까) → R23 을 장별로 다시 보지 않는다. 대신 장을 넘는 네트를 본다
        carried = set()
        for lk in d.links:
            for (rr, pnum), nn in d.pinnet.items():
                if rr == lk['ref']: carried.add(nn)
        for net, plist in d.nets.items():
            if net in d.supplies: continue
            pans = {pan.get(r) for r, k in plist if r in pan}
            pans.discard(None)
            if len(pans) > 1 and net not in carried:
                R.add('R29','오류', f"네트 {net} 이 장 {sorted(pans)} 에 걸쳐 있는데 link 헤더를 안 지남 — 물리적으로 연결 안 됨")
        for lk in d.links:
            if lk['panel'] not in d.panels or lk['to'] not in d.panels:
                R.add('R29','오류', f"link {lk['ref']}: 모르는 장 {lk['panel']}→{lk['to']}")
        n_link = sum(1 for _ in d.links)
        if n_link: R.add('R29','정보', f"장간 연결 헤더 {n_link}개")

    # R25~R27 층(아파트) 규칙
    if d.layers > 1 or d.stacks:
        for ref in d.order:
            L = d.parts[ref].get('layer', 1)
            if L < 1 or L > d.layers:
                R.add('R25','오류',f"{ref} 의 층 {L} 이 기판 층수(1~{d.layers}) 밖")
        # 같은 네트가 서로 다른 층에 있는데 그 층들을 잇는 stack 이 그 네트를 안 물고 있으면 연결 불가
        for net, plist in d.nets.items():
            lays = {d.parts[r].get('layer',1) for r,p in plist}
            if len(lays) <= 1: continue
            carried = any(r in d.stacks for r,p in plist)
            if not carried:
                R.add('R26','오류',f"네트 {net} 이 층 {sorted(lays)} 에 걸쳐 있는데 층간 핀헤더(stack)를 안 지남 — 물리적으로 연결 안 됨")
        # stack 은 자기가 잇는 두 층 모두에 같은 자리여야 한다 (핀헤더가 수직으로 꽂히므로)
        for sref, st in d.stacks.items():
            if st['hi'] - st['lo'] != 1:
                R.add('R27','경고',f"{sref} 가 {st['lo']}층과 {st['hi']}층을 잇는다 — 이웃한 층끼리만 꽂을 수 있다")
        R.add('R25','정보',f"기판 {d.layers}층 · 층간 핀헤더 {len(d.stacks)}개 ({', '.join(f'{k}:{v[chr(39)+chr(39)] if False else v}' for k,v in list(d.stacks.items())[:0]) or '목록 생략'})")
        for L in range(1, d.layers+1):
            n = sum(1 for r in d.order if d.parts[r].get('layer',1)==L)
            R.add('R25','정보',f"  {L}층: 부품 {n}개")
    # R7 타이밍
    tim = timing(d, netinfo, R)
    tim['volts'] = volts
    tim['trans'] = transients(d, netinfo, volts, R, getattr(d, 'base', HERE_DIR))
    return R, netinfo, tim

# ═════════════════════════════════════════════════════════════
# 4b. 전압·전류 — 네트마다 데이터시트 값으로
#     HIGH: 드라이버 VOH(min) ~ VCC   (풀업 있으면 VCC 쪽으로 보장)
#     LOW : 0 ~ 드라이버 VOL(max)
#     수신 여유: HIGH 여유 = VOH - max(VIH),  LOW 여유 = min(VIL) - VOL
#     전류: LOW 때 풀업 흡수 = ΣV/R,  HIGH 때 수신 누설 ≈ 1µA/입력
# ═════════════════════════════════════════════════════════════
def compute_volts(d, netinfo):
    out = {}
    vcc_nets = {n for n,(v,_) in d.supplies.items() if v > 0}
    vcc = max([v for v,_ in d.supplies.values()] + [5.0])
    for net, ni in netinfo.items():
        if net in d.supplies:
            v = d.supplies[net][0]
            icc = sum(d.chips[d.parts[r]['chip']].attr.get('icc',0) for r,p,t in ni['vccp'])
            out[net] = dict(kind='supply', v=v, i=icc, pins=len(d.nets[net])); continue
        outs = ni['drivers'] + ni['bis']
        recv = ni['ins'] + ni['clks']
        # 풀업
        pus = []
        for ref, pin, tag in ni['pas']:
            c = d.chips[d.parts[ref]['chip']]
            if 'res' in c.attr:
                onet = d.pinnet.get((ref, 2 if pin == 1 else 1))
                if onet in vcc_nets: pus.append((c.attr['res'], d.supplies[onet][0]))
        if not outs:
            out[net] = dict(kind='floating' if recv else 'passive', pins=len(d.nets[net]), pullup=[r for r,_ in pus]); continue
        dref, dpin, dtag = outs[0]; dc = d.chips[d.parts[dref]['chip']]
        voh = dc.attr.get('voh', 0.0); vol = dc.attr.get('vol', vcc)
        vmax = vcc
        if pus:   # 풀업이 있으면 HIGH 는 VCC - 누설×R 로 보장. 누설 = 수신 1µA/입력 + 드라이버 3상태 누설 10µA (74HC IOZ max)
            leak = 1e-6 * max(1, len(recv)) + 10e-6; rpar = 1/sum(1/r for r,_ in pus)
            voh = max(voh, pus[0][1] - leak*rpar)
        vih = max([d.chips[d.parts[r]['chip']].attr.get('vih', 0) for r,p,t in recv] + [0])
        vil = min([d.chips[d.parts[r]['chip']].attr.get('vil', vcc) for r,p,t in recv] + [vcc])
        i_low = sum(v/r for r,v in pus) if pus else 1e-6*len(recv)
        i_high = 1e-6*len(recv)
        out[net] = dict(kind='signal', driver=dtag, voh=round(voh,3), vol=round(vol,3), vmax=vmax,
                        vih=vih, vil=vil, margin_hi=round(voh-vih,3), margin_lo=round(vil-vol,3),
                        i_low=i_low, i_high=i_high, pullup=[r for r,_ in pus], receivers=len(recv),
                        iol=dc.attr.get('iol'))
    return out

def export_volts(d, volts, path):
    L = [f"{d.title}", "="*78, "네트별 전압·전류 (데이터시트 값으로 계산. AI 예상은 expect 줄, 대조는 R15)", ""]
    L.append(f"{'네트':<8}{'종류':<8}{'HIGH(min)':>10}{'LOW(max)':>9}{'수신VIH':>8}{'수신VIL':>8}{'여유H':>7}{'여유L':>7}{'I_low':>9}{'풀업':>8}  드라이버 / AI예상")
    L.append("-"*78)
    for net in sorted(volts, key=lambda n:(volts[n]['kind']!='supply', str(n))):
        v = volts[net]; ex = d.expects.get(net)
        exs = f"AI {ex['hi']}/{ex['lo']}" if ex else ''
        if v['kind']=='supply': L.append(f"{net:<8}{'전원':<8}{v['v']:>10.2f}{v['v']:>9.2f}{'':>8}{'':>8}{'':>7}{'':>7}{v['i']*1e3:>7.1f}mA{'':>8}  {exs}")
        elif v['kind']=='signal': L.append(f"{net:<8}{'신호':<8}{v['voh']:>10.2f}{v['vol']:>9.2f}{v['vih']:>8.2f}{v['vil']:>8.2f}{v['margin_hi']:>7.2f}{v['margin_lo']:>7.2f}{v['i_low']*1e3:>7.2f}mA{(str(int(v['pullup'][0]))+'Ω') if v['pullup'] else '-':>8}  {v['driver']}  {exs}")
        else: L.append(f"{net:<8}{v['kind']:<8}{'?':>10}{'?':>9}  {exs}")
    io.open(path,'w',encoding='utf-8').write('\n'.join(L)+'\n')

# ═════════════════════════════════════════════════════════════
# 4c. 과도현상 — 그라운드 바운스 · 디커플링 · 크로스토크 · 반사 · 클럭 스큐 · IR 강하 · 발열
#     전부 교과서 공식 + physics.txt 경험 상수. SPICE 가 아니다. 크기(order) 를 보는 용도.
# ═════════════════════════════════════════════════════════════
def load_physics(base):
    ph = {}
    path = os.path.join(base, 'physics.txt')
    if not os.path.exists(path): path = os.path.join(HERE_DIR, 'physics.txt')
    if os.path.exists(path):
        for raw in io.open(path, encoding='utf-8'):
            line = strip_comment(raw)
            if line: k, v = line.split()[:2]; ph[k] = float(v)
    return ph

def transients(d, netinfo, volts, R, base):
    PH = load_physics(base)
    if not PH: R.add('R16','정보',"physics.txt 없음 — 과도현상 계산 생략"); return {}
    vcc = max([v for v,_ in d.supplies.values()] + [5.0])
    gnd_rows = d.buses.get('GND', [d.board[1]])
    def hole(ref, pin):
        p = d.parts[ref]; p['_pkg'] = d.chips[p['chip']].pkg; return pin_hole(p, pin)
    def chain(pins):
        order=[pins[0]]; rest=list(pins[1:])
        while rest:
            a=order[-1]; rest.sort(key=lambda b: abs(a[0]-b[0])+abs(a[1]-b[1])); order.append(rest.pop(0))
        return order
    def net_route_len_mm(net):
        pins = [hole(r,p) for r,p in d.nets.get(net,[])]
        if len(pins) < 2: return 0.0
        if d.routes.get(net):
            tot = 0.0
            for rt in d.routes[net]:
                q = [x for x in rt['pts'] if not isinstance(x,str)]
                tot += sum(abs(q[i][0]-q[i-1][0])+abs(q[i][1]-q[i-1][1]) for i in range(1,len(q)))*PITCH
            return tot
        o = chain(pins)
        return sum(abs(o[i][0]-o[i-1][0])+abs(o[i][1]-o[i-1][1]) for i in range(1,len(o)))*PITCH
    def net_cload(net):
        c = 0.0
        for r,p in d.nets.get(net,[]):
            k = d.chips[d.parts[r]['chip']].pin(p)
            if k and k[1] in ('in','clk'): c += d.chips[d.parts[r]['chip']].attr.get('cin',5e-12)
        return c + net_route_len_mm(net)*PH['wire_pF_per_mm']*1e-12
    out = {}
    caps = [r for r in d.order if 'decoupling' in d.chips[d.parts[r]['chip']].flags]
    bounce_list = []
    for ref in d.order:
        c = d.chips[d.parts[ref]['chip']]
        if 'sso' not in c.attr or 't_tt_min' not in c.attr: continue
        if c.pkg.startswith('CONN') or c.attr.get('icc',1) == 0:   # 외부 보드(MEGA): 자기 전원·디커플링을 가짐
            R.add('R16','정보',f"{ref}: 외부 보드 — 바운스/디커플링은 그쪽 기판 몫. 다만 이 기판으로 들어오는 접지선은 굵고 짧게"); continue
        outs = [pin for pin,(nm,kind,cls) in c.pins.items() if kind in ('out','bi') and any(fnmatch.fnmatch(nm,g) for g in c.attr['sso']) and (ref,pin) in d.pinnet and d.pinnet[(ref,pin)] not in d.supplies and d.pinnet[(ref,pin)] != 'NC']
        if not outs: continue
        ctot = sum(net_cload(d.pinnet[(ref,pin)]) for pin in outs)
        tr = c.attr['t_tt_min']
        ipk = ctot*vcc/tr
        didt = ipk/(tr/2)
        gpin = next((pin for pin,(nm,kind,cls) in c.pins.items() if kind=='gnd'), None)
        lead = PH['lead_nH_dip32'] if c.pkg=='DIP32' else PH['lead_nH_dip']
        gwire_mm = 0.0
        if gpin is not None:
            col,row = hole(ref,gpin)
            pn_ = d.parts[ref].get('panel') or (d.panel_order[0] if d.panel_order else None)
            rows_ = (d.pbuses.get(pn_, {}).get('GND') if d.pbuses else None) or gnd_rows
            gwire_mm = min(abs(r-row) for r in rows_)*PITCH if rows_ else gwire_mm
        L = (lead + gwire_mm*PH['wire_nH_per_mm'])*1e-9
        vb = min(vcc, L*didt)                                   # 물리적으로 VCC 를 넘을 수 없다 → 상한
        # 대책 두 가지를 숫자로: (a) 접지선 10mm, (b) + 출력 직렬 33Ω (에지가 RC 로 느려져 di/dt 감소)
        L10 = (lead + 10*PH['wire_nH_per_mm'])*1e-9
        vb10 = min(vcc, L10*didt)
        tr_rc = max(tr, 2.2*33*(ctot/len(outs)))                # 출력 하나당 RC
        didt_r = (ctot*vcc/tr_rc)/(tr_rc/2)
        vb10r = min(vcc, L10*didt_r)
        margins = [volts[d.pinnet[(ref,pin)]]['margin_lo'] for pin in outs if volts.get(d.pinnet[(ref,pin)],{}).get('kind')=='signal']
        mlo = min(margins) if margins else 0.5
        vpin = next((pin for pin,(nm,kind,cls) in c.pins.items() if kind=='vcc'), None)
        cdist = None
        if vpin is not None and caps:
            vc = hole(ref,vpin)
            cdist = min(math.hypot((hole(cr,1)[0]-vc[0])*PITCH,(hole(cr,1)[1]-vc[1])*PITCH) for cr in caps)
        Ldec = (PH['cap_esl_nH'] + (2*(cdist or 0))*PH['wire_nH_per_mm'])*1e-9
        vdec = min(vcc, Ldec*didt + (ctot*vcc)/100e-9)
        pwr = vcc*c.attr.get('icc',0); dT = pwr*PH['theta_ja_dip']
        out[ref] = dict(sso=len(outs), cload_pF=ctot*1e12, ipk_mA=ipk*1e3, didt=didt, gnd_wire_mm=gwire_mm, L_nH=L*1e9, bounce_V=vb, margin_lo=mlo, cap_mm=cdist, vdec_V=vdec, pwr_mW=pwr*1e3, dT=dT)
        # 이 출력들을 받는 쪽이 클럭 입력이면 바운스가 곧 오동작. 조합 입력(주소·데이터)이면 정착 뒤 샘플링되어 타이밍 여유로 흡수된다
        recv_kinds = set()
        for pin in outs:
            for r2,p2 in d.nets.get(d.pinnet[(ref,pin)],[]):
                k2 = d.chips[d.parts[r2]['chip']].pin(p2); recv_kinds.add(k2[1] if k2 else '')
        clocked = 'clk' in recv_kinds
        lvl = '경고' if (clocked and vb > 0.5*mlo) or vb > mlo else '정보'
        note = " [클럭 입력을 몰음 — 위험]" if clocked else (" [조합 입력: 정착 후 샘플링, 타이밍 여유로 흡수]" if vb > 0.5*mlo else "")
        R.add('R16', lvl, f"{ref}({c.name}) 그라운드 바운스 상한 ≈ {vb:.2f}V{note} (출력 {len(outs)}개 동시, 부하 {ctot*1e12:.0f}pF, di/dt {didt/1e6:.0f}mA/ns, L {L*1e9:.0f}nH = 리드 {lead:.0f}nH + 접지선 {gwire_mm:.0f}mm) vs LOW 여유 {mlo:.2f}V" + (f" → 접지선 10mm 면 {vb10:.2f}V, +출력 33Ω 직렬이면 {vb10r:.2f}V" if vb>0.5*mlo else ""))
        bounce_list.append((vb, ref, gwire_mm, vb10r))
        if cdist is not None:
            R.add('R17', '경고' if cdist>10 else '정보', f"{ref} 디커플링: 가장 가까운 0.1µF 까지 {cdist:.0f}mm, 스위칭 순간 VCC 처짐 상한 ≈ {vdec:.2f}V" + (" — 커패시터 리드를 VCC/GND 핀에 바로 (5mm 안)" if cdist>10 else ""))
        if dT > 40: R.add('R22','경고',f"{ref} 발열 {pwr*1e3:.0f}mW → 약 +{dT:.0f}°C (θJA {PH['theta_ja_dip']:.0f}°C/W)")
    # 크로스토크
    segs = []
    for net in d.nets:
        if net in d.supplies: continue
        pins = [hole(r,p) for r,p in d.nets[net]]
        if len(pins) < 2: continue
        o = chain(pins)
        for i in range(1,len(o)):
            a,b=o[i-1],o[i]
            segs.append((net,'v',a[0],min(a[1],b[1]),max(a[1],b[1])))
            segs.append((net,'h',b[1],min(a[0],b[0]),max(a[0],b[0])))
    worst = None
    didt1 = (50e-12*vcc/7e-9)/(3.5e-9)
    for i in range(len(segs)):
        n1,o1,c1,s1,e1 = segs[i]
        if e1-s1 < 3: continue
        for j in range(i+1,len(segs)):
            n2,o2,c2,s2,e2 = segs[j]
            if n1==n2 or o1!=o2 or not (0 < abs(c1-c2) <= 2): continue
            ov = min(e1,e2)-max(s1,s2)
            if ov < 3: continue
            l = ov*PITCH*1e-3; sp = abs(c1-c2)*PITCH*1e-3
            M = 2e-7*l*(math.log(2*l/sp)-1)
            v = M*didt1
            if worst is None or v > worst[0]: worst = (v, n1, n2, ov*PITCH, abs(c1-c2)*PITCH)
    if worst:
        v,n1,n2,l,sp = worst
        def has_clk(net): return any((d.chips[d.parts[r]['chip']].pin(p) or ('', '', ''))[1]=='clk' for r,p in d.nets.get(net,[]))
        clkv = has_clk(n1) or has_clk(n2)
        R.add('R18', '경고' if (v>0.3 and clkv) else '정보', f"크로스토크 상한 ≈ {v*1e3:.0f}mV: 네트 {n1}↔{n2}, 평행 {l:.0f}mm 간격 {sp:.1f}mm (개방 루프 상호인덕턴스, 접지면 없음)" + (" [클럭선 피해 — 위험] 사이에 GND 선을 깔거나 경로를 갈라라" if clkv and v>0.3 else (" [조합 입력: 정착 후 샘플링 → 타이밍 여유로 흡수. 클럭선은 이 다발과 떨어뜨릴 것]" if v>0.3 else "")))
    tr_min = min([c.attr['t_tt_min'] for c in d.chips.values() if 't_tt_min' in c.attr] + [7e-9])
    crit_mm = tr_min/(2*PH['prop_ps_per_mm']*1e-12)
    longest = max(((net_route_len_mm(n),n) for n in d.nets if n not in d.supplies), default=(0,None))
    R.add('R19', '경고' if longest[0]>crit_mm else '정보', f"반사: 최장 배선 {longest[0]:.0f}mm (네트 {longest[1]}) vs 임계 길이 {crit_mm:.0f}mm (에지 {tr_min*1e9:.0f}ns) — {'종단 필요' if longest[0]>crit_mm else '종단 불필요'}")
    for net, ni in netinfo.items():
        if not ni['clks'] or net in d.supplies: continue
        outs = ni['drivers']
        if not outs: continue
        src = hole(outs[0][0], outs[0][1])
        dists = [abs(hole(r,p)[0]-src[0])+abs(hole(r,p)[1]-src[1]) for r,p,t in ni['clks']]
        skew = (max(dists)-min(dists))*PITCH*PH['prop_ps_per_mm']*1e-12
        holdmin = min([d.chips[d.parts[r]['chip']].attr.get('t_hold',1) for r,p,t in ni['clks']] + [1])
        R.add('R20', '경고' if skew>holdmin else '정보', f"클럭 네트 {net}: 수신 {len(ni['clks'])}개, 배선 길이차 {(max(dists)-min(dists))*PITCH:.0f}mm → 스큐 {skew*1e12:.0f}ps (hold {holdmin*1e9:.0f}ns)")
    itot = sum(d.chips[d.parts[r]['chip']].attr.get('icc',0) for r in d.order)
    rail_m = d.board[0]*PITCH*1e-3
    vdrop = itot*PH['awg30_ohm_per_m']*rail_m
    R.add('R21','정보',f"전원 레일 IR 강하 ≈ {vdrop*1e3:.1f}mV (정지전류 {itot*1e3:.0f}mA × AWG30 {rail_m*100:.0f}cm)")
    if bounce_list:
        bad = [w for w in bounce_list if w[2] > 10]      # 접지선 10mm 넘는 칩
        if bad:
            R.add('R16','경고',f"요약: 칩 {len(bad)}개가 접지선이 길다 (최장 {max(w[2] for w in bad):.0f}mm). `bus GND rows ...` 로 칩 열마다 GND 버스를 깔고 각 GND 핀을 10mm 안에서 붙일 것. 그러면 최악 {max(w[3] for w in bad):.2f}V")
        else:
            R.add('R16','정보',f"접지 배선 OK: GND 버스 {len(gnd_rows)}줄, 최장 접지선 {max(w[2] for w in bounce_list):.0f}mm, 바운스 최악 {max(w[0] for w in bounce_list):.2f}V")
    R.add('R16','정보',"과도현상 상수는 physics.txt 의 경험값(⚠가정) — 실측하면 갱신할 것")
    return out

# ═════════════════════════════════════════════════════════════
# 5. 타이밍 — 레지스터→레지스터 최장 경로
# ═════════════════════════════════════════════════════════════
def timing(d, netinfo, R):
    """
    출발: 레지스터 출력핀 (clk2q)
    통과: 조합 부품의 path(in→out) 를 따라 네트를 건넌다
    도착: 레지스터 입력핀 (setup)
    """
    chips = d.chips
    # 핀 → (kind) 빠른 조회
    def kindof(ref, pin):
        return chips[d.parts[ref]['chip']].pin(pin)
    # 레지스터 출력 시작점
    starts = []
    for ref in d.order:
        c = chips[d.parts[ref]['chip']]
        for outs, ins, clk in c.regs:
            for pin,(nm,kind,cls) in c.pins.items():
                if any(fnmatch.fnmatch(nm, g) for g in outs) and (ref,pin) in d.pinnet:
                    starts.append((ref, pin, c.attr.get('t_clk2q', 0)))
    def is_reg_input(ref, pin):
        c = chips[d.parts[ref]['chip']]
        nm = c.pin(pin)[0]
        for outs, ins, clk in c.regs:
            if any(fnmatch.fnmatch(nm, g) for g in ins): return True
        return False
    best = None; allpaths = []
    for ref, pin, t0 in starts:
        # DFS
        stack = [(ref, pin, t0, [f"{ref}.{chips[d.parts[ref]['chip']].pin(pin)[0]}"])]
        seen = set()
        while stack:
            r, p, t, trail = stack.pop()
            net = d.pinnet.get((r,p))
            if net is None or net in d.supplies: continue
            for (r2, p2) in d.nets[net]:
                if (r2,p2) == (r,p): continue
                c2 = chips[d.parts[r2]['chip']]
                nm2, kind2, _ = c2.pin(p2)
                if kind2 not in ('in','clk','bi'): continue
                if is_reg_input(r2, p2):
                    tt = t + c2.attr.get('t_setup', 0)
                    path = trail + [f"{r2}.{nm2}(setup)"]
                    allpaths.append((tt, path))
                    if best is None or tt > best[0]: best = (tt, path)
                    continue
                # 조합 통과
                for gi, go, dly in c2.paths:
                    if fnmatch.fnmatch(nm2, gi):
                        for p3,(nm3,k3,_) in c2.pins.items():
                            if fnmatch.fnmatch(nm3, go) and (r2,p3) in d.pinnet:
                                key = (r2,p3,len(trail))
                                if len(trail) > 12: continue
                                stack.append((r2, p3, t+dly, trail + [f"{r2}.{nm2}→{nm3}({dly*1e9:.0f})"]))
    if best:
        tns = best[0]*1e9
        R.add('R7','정보',f"최장 경로 {tns:.0f}ns → 최대 클럭 {1e3/tns:.1f}MHz  : " + ' → '.join(best[1]))
    # 분주기 fmax
    for ref in d.order:
        c = chips[d.parts[ref]['chip']]
        if 't_fmax' in c.attr:
            R.add('R7','정보',f"{ref} ({c.name}) fmax {c.attr['t_fmax']/1e6:.0f}MHz — 오실레이터가 이보다 빠르면 분주기가 못 따라감")
    allpaths.sort(key=lambda x:-x[0])
    return dict(best=best, top=allpaths[:8])

# ═════════════════════════════════════════════════════════════
# 6. 내보내기
# ═════════════════════════════════════════════════════════════
KICAD_FP = {'DIP14':'Package_DIP:DIP-14_W7.62mm','DIP16':'Package_DIP:DIP-16_W7.62mm',
            'DIP20':'Package_DIP:DIP-20_W7.62mm','DIP32':'Package_DIP:DIP-32_W15.24mm',
            'CAP2':'Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm','RES2':'Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal','SIP3':'Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical',
            'CONN30':'Connector_PinHeader_2.54mm:PinHeader_2x15_P2.54mm_Vertical',
            'SIP3':'Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical',
            'SIP8':'Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical',
            'SIP20':'Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical',
            'SIP40':'Connector_PinHeader_2.54mm:PinHeader_1x40_P2.54mm_Vertical'}

def export_kicad(d, path):
    """KiCad 넷리스트 (레거시 S-expression, version D). Pcbnew → File → Import Netlist"""
    o = ['(export (version D)', '  (design', f'    (source "{d.title}")', '    (tool "KYLE sol.py"))', '  (components']
    for ref in d.order:
        p = d.parts[ref]; c = d.chips[p['chip']]
        fp = KICAD_FP.get(c.pkg, 'Package_DIP:DIP-16_W7.62mm')
        o.append(f'    (comp (ref {ref}) (value {c.name}) (footprint {fp}))')
    o.append('  )'); o.append('  (nets')
    code = 1
    for net in sorted(d.nets, key=lambda n: (n not in d.supplies, str(n))):
        o.append(f'    (net (code {code}) (name "{net}")')
        for ref, pin in d.nets[net]:
            o.append(f'      (node (ref {ref}) (pin {pin}))')
        o.append('    )'); code += 1
    o.append('  )'); o.append(')')
    txt = '\n'.join(o)+'\n'
    # 자체 점검: 괄호 균형, 모든 node 의 ref 가 components 에 있는지
    assert txt.count('(') == txt.count(')'), "KiCad 넷리스트 괄호 불균형"
    refs = set(d.order)
    for ref, pin in ((r,p) for plist in d.nets.values() for r,p in plist):
        assert ref in refs, f"KiCad: 부품 없는 노드 {ref}"
    io.open(path,'w',encoding='utf-8').write(txt)

def export_kicad_pcb(d, path):
    """KiCad 기판 파일(.kicad_pcb) — 넷리스트가 아니라 '배치까지' 그대로 옮긴다.

    KYLE 은 부품마다 구멍 좌표(col,row)·회전·층을 이미 갖고 있으므로,
    KiCad 에서 다시 늘어놓을 필요가 없다. 한 층이 KiCad 한 장이 되고,
    여러 층이면 층마다 X 로 (기판폭+10mm) 씩 떨어뜨려 나란히 놓는다.
    열고 나서: Pcbnew 로 열기 → 필요하면 Edit ▸ Move 로 층별 블록을 분리.
    """
    W, H = d.board
    bw, bh = (W-1)*PITCH, (H-1)*PITCH
    nlay = max(1, d.layers)
    o = ['(kicad_pcb (version 20211014) (generator "KYLE sol.py")',
         '  (general (thickness 1.6))', '  (paper "A3")',
         '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user) (37 "F.SilkS" user))']
    # 층마다 외곽선
    for Lz in range(1, nlay+1):
        ox = (Lz-1) * (bw + 10.0)
        for (x1,y1,x2,y2) in ((0,0,bw,0),(bw,0,bw,bh),(bw,bh,0,bh),(0,bh,0,0)):
            o.append(f'  (gr_line (start {ox+x1:.2f} {y1:.2f}) (end {ox+x2:.2f} {y2:.2f}) (layer "Edge.Cuts") (width 0.1))')
        o.append(f'  (gr_text "{Lz}층" (at {ox+bw/2:.2f} -3.0) (layer "F.SilkS") (effects (font (size 2 2) (thickness 0.3))))')
    # 부품: KYLE 좌표 그대로
    for ref in d.order:
        p = d.parts[ref]; c = d.chips[p['chip']]
        fp = KICAD_FP.get(c.pkg)
        if fp is None:
            raise SystemExit(f"KiCad: 패키지 {c.pkg} 의 풋프린트를 모른다 (KICAD_FP 에 추가할 것)")
        Lz = p.get('layer', 1); ox = (Lz-1) * (bw + 10.0)
        x, y = ox + p['col']*PITCH, p['row']*PITCH
        o.append(f'  (footprint "{fp}" (layer "F.Cu") (at {x:.2f} {y:.2f} {p["rot"]})')
        o.append(f'    (fp_text reference "{ref}" (at 0 -2) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))')
        o.append(f'    (fp_text value "{c.name}" (at 0 2) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))')
        n, _w, _bw, _bh = pkg_geom(c.pkg)
        for pin in range(1, n+1):
            col, row = pin_hole(p, pin)
            # 풋프린트 기준 상대좌표 (회전은 KiCad 가 at 의 각도로 처리하므로 여기선 무회전 좌표를 쓴다)
            base = dict(d.parts[ref]); base['rot'] = 0
            c0, r0 = pin_hole(base, pin)
            rx, ry = (c0 - base['col'])*PITCH, (r0 - base['row'])*PITCH
            net = None
            for nname, plist in d.nets.items():
                if (ref, pin) in plist: net = nname; break
            netpart = f' (net {_net_code(d, net)} "{net}")' if net is not None else ''
            o.append(f'    (pad "{pin}" thru_hole {"rect" if pin==1 else "circle"} (at {rx:.2f} {ry:.2f}) '
                     f'(size 1.7 1.7) (drill 0.9) (layers "*.Cu" "*.Mask"){netpart})')
        o.append('  )')
    # 넷 목록
    nets = ['  (net 0 "")'] + [f'  (net {_net_code(d, n)} "{n}")' for n in _net_order(d)]
    o[4:4] = nets
    txt = '\n'.join(o) + '\n)\n'
    assert txt.count('(') == txt.count(')'), "kicad_pcb 괄호 불균형"
    io.open(path, 'w', encoding='utf-8').write(txt)

def _net_order(d):
    return sorted(d.nets, key=lambda n: (n not in d.supplies, str(n)))

def _net_code(d, net):
    if net is None: return 0
    return _net_order(d).index(net) + 1

def export_json(d, R, netinfo, tim, path):
    parts = []
    for ref in d.order:
        p = d.parts[ref]; c = d.chips[p['chip']]
        p['_pkg'] = c.pkg
        n, w, bw, bh = pkg_geom(c.pkg)
        pins = []
        for pin in range(1, n+1):
            col, row = pin_hole(p, pin)
            info = c.pin(pin)
            raw = d.pinnet.get((ref,pin))
            pins.append(dict(n=pin, name=info[0] if info else '', kind=info[1] if info else 'undef',
                             col=col, row=row, net=(None if raw in (None,'NC') else raw), nc=(raw=='NC')))
        parts.append(dict(ref=ref, chip=c.name, pkg=c.pkg, col=p['col'], row=p['row'], rot=p['rot'], layer=p.get('layer',1), stack=p.get('stack'), near=p.get('near'), fixed=bool(p.get('fixed')),
                          width=w, bodyW=bw, bodyH=bh, pins=pins, src=c.attr.get('src','')))
    def resolve(pt):
        # 'REF.PIN' → 그 핀의 구멍 좌표. 이미 좌표면 그대로
        if isinstance(pt, str):
            ref, pin = pt.rsplit('.', 1)
            if ref in d.parts:
                p = d.parts[ref]; p['_pkg'] = d.chips[p['chip']].pkg
                return list(pin_hole(p, int(pin)))
            return None
        return list(pt)
    nets = {}
    for net, plist in d.nets.items():
        rts = []
        for rt in d.routes.get(net, []):
            rp = [q for q in (resolve(x) for x in rt['pts']) if q]
            if len(rp) >= 2: rts.append(dict(side=rt['side'], pts=rp, layer=rt.get('layer',1), auto=rt.get('auto',False)))
        nets[str(net)] = dict(pins=[[r,p] for r,p in plist], group=d.groups.get(net, ''),
                              supply=(net in d.supplies), routes=rts,
                              volt=tim.get('volts',{}).get(net), expect=d.expects.get(net))
    out = dict(title=d.title, board=list(d.board), layers=d.layers, stacks=d.stacks, pitch=PITCH, settings=d.settings, parts=parts, nets=nets, buses=d.buses,
               sims=[dict(name=sc['name'], steps=sc['steps']) for sc in d.sims],
               colors=d.colors, supplies={k:v[0] for k,v in d.supplies.items()},
               report=[dict(rule=r, level=l, msg=m) for r,l,m in R.items],
               trans=tim.get('trans',{}),
               timing=dict(best_ns=(tim['best'][0]*1e9 if tim['best'] else None),
                           best_path=(tim['best'][1] if tim['best'] else []),
                           top=[dict(ns=t*1e9, path=p) for t,p in tim['top']]))
    io.open(path,'w',encoding='utf-8').write(json.dumps(out, ensure_ascii=False, indent=1))

def export_list(d, path):
    L = [f"{d.title}", "="*60, "납땜 체크리스트 — 한 줄 하면 [ ] 에 표시", ""]
    L.append("[ 전원 ]")
    for net in d.supplies:
        for ref, pin in d.nets.get(net, []):
            c = d.chips[d.parts[ref]['chip']]; nm = c.pin(pin)[0] if c.pin(pin) else ''
            L.append(f"[ ] {ref:<9}{pin:>3}번 {nm:<5} → {net}")
    bygroup = defaultdict(list)
    for net, plist in d.nets.items():
        if net in d.supplies: continue
        bygroup[d.groups.get(net,'기타')].append((net, plist))
    for g in sorted(bygroup):
        L.append(f"\n[ {g} ]")
        for net, plist in bygroup[g]:
            names = [f"{r}.{pin}" for r,pin in plist]
            L.append(f"[ ] 네트 {net}: " + " — ".join(names))
    io.open(path,'w',encoding='utf-8').write('\n'.join(L)+'\n')

def export_sol(d, path):
    """Design → KYLE 텍스트 (뷰어에서 온 배치·연결을 정식 파일로)"""
    L = ['#KYLE 1', f'title   {d.title}', f'board   perf {d.board[0]} {d.board[1]}' + (f' layers {d.layers}' if d.layers>1 else ''), 'include chips.txt',
         f"wire    {d.settings['wire']}mm", f"stitch  {d.settings['stitch']}", f"share   {d.settings['share']}", '']
    for n,(v,imax) in d.supplies.items():
        L.append(f'supply  {n} {v}V' + (f' max {imax}A' if imax else ''))
    L.append('')
    for ref in d.order:
        p = d.parts[ref]
        if ref in d.stacks: continue
        L.append(f"part    {ref:<9}{p['chip']:<12}at {p['col']:>2} {p['row']:>2}" + (f"  rot {p['rot']}" if p['rot'] else '') + (f"  layer {p['layer']}" if p.get('layer',1)!=1 else '') + (f"  near {p['near']}" if p.get('near') else '') + ('  fixed' if p.get('fixed') else ''))
    L.append('')
    for g,c in d.colors.items(): L.append(f'color   {g:<12}{c}')
    L.append('')
    cur=None
    for (ref,pin),net in sorted(d.pinnet.items(), key=lambda kv:(d.order.index(kv[0][0]), kv[0][1])):
        if ref!=cur: L.append(f'# {ref}'); cur=ref
        L.append(f'pin     {ref}[{pin}:{net}]')
    L.append('')
    for net,g in d.groups.items(): L.append(f'group   {net:<6}{g}')
    for net,ex in d.expects.items(): L.append(f"expect  {net:<6}{ex['hi']} {ex['lo']}" + (f" {ex['i']*1e3:.2f}mA" if ex['i'] is not None else ''))
    if d.pbuses:
        for pn_, bb in d.pbuses.items():
            for net,rows in bb.items(): L.append(f"bus     {net} rows " + ','.join(map(str,rows)) + f" panel {pn_}")
    else:
        for net,rows in d.buses.items(): L.append(f"bus     {net} rows " + ','.join(map(str,rows)))
    for sref,st in d.stacks.items(): L.append(f"stack   {sref} {st['kind']} at {st['col']} {st['row']} layers {st['lo']}-{st['hi']}" + (f" panel {st['panel']}" if st.get('panel') else ''))
    for sc in d.sims:
        L.append(''); L.append(f"sim     {sc['name']}")
        for st in sc['steps']: L.append(f"  {st[0]:<7} " + ' '.join(st[1:]))
        L.append('end')
    fmt = lambda v: (str(int(v)) if float(v).is_integer() else f"{v:.1f}")
    for net,rs in d.routes.items():
        i = 0
        while i < len(rs):
            rt = rs[i]; side, Lz, auto = rt['side'], rt.get('layer',1), rt.get('auto',False); out = list(rt['pts']); j = i+1
            while j < len(rs) and rs[j]['pts'] and rs[j]['pts'][0] == out[-1] and rs[j]['side'] != rs[j-1]['side'] and rs[j].get('layer',1) == Lz and rs[j].get('auto',False) == auto and rs[j].get('panel') == rt.get('panel'):
                out.append('|'); out.extend(rs[j]['pts'][1:]); j += 1
            L.append(f"route   {net} {'auto ' if auto else ''}{side}{(' layer '+str(Lz)) if Lz != 1 else ''}{(' panel '+rt['panel']) if rt.get('panel') else ''} " + ' '.join(x if isinstance(x,str) else f'({fmt(x[0])},{fmt(x[1])})' for x in out))
            i = j
    io.open(path,'w',encoding='utf-8').write('\n'.join(L)+'\n')

# ═════════════════════════════════════════════════════════════
def main():
    if len(sys.argv) < 2: print(__doc__); return 1
    src = sys.argv[1]; flags = set(sys.argv[2:])
    d = parse_design(src)
    R, netinfo, tim = verify(d)
    stem = os.path.splitext(src)[0]

    print(f"\n{d.title}")
    print(f"부품 {len(d.order)}  네트 {len(d.nets)}  핀연결 {len(d.pinnet)}")
    print("-"*64)
    order = {'치명':0,'오류':1,'경고':2,'정보':3}
    for rule, lvl, msg in sorted(R.items, key=lambda x:(order[x[1]], x[0])):
        print(f"[{lvl}] {rule:<4} {msg}")
    print("-"*64)
    print(f"치명 {R.count('치명')}  오류 {R.count('오류')}  경고 {R.count('경고')}  정보 {R.count('정보')}")
    if tim['top']:
        print("\n최장 경로 상위:")
        for t, p in tim['top'][:5]:
            print(f"  {t*1e9:6.0f}ns  " + ' → '.join(p))

    if flags & {'--json','--all'}:
        export_json(d, R, netinfo, tim, stem+'.json'); print(f"\n→ {stem}.json")
    if flags & {'--kicad','--all'}:
        export_kicad(d, stem+'.kicad.net'); print(f"→ {stem}.kicad.net")
        export_kicad_pcb(d, stem+'.kicad_pcb'); print(f"→ {stem}.kicad_pcb  (배치까지 들어간 기판 파일)")
    if flags & {'--list','--all'}:
        export_list(d, stem+'_list.txt'); print(f"→ {stem}_list.txt")
    if flags & {'--volt','--all'}:
        export_volts(d, tim.get('volts',{}), stem+'_volt.txt'); print(f"→ {stem}_volt.txt")
    return 0 if R.count('치명')==0 and R.count('오류')==0 else 1

if __name__ == '__main__':
    sys.exit(main())
