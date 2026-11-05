#!/usr/bin/env python3
"""
simulate — KYLE 설계를 '실제로 돌린다'. 입력을 넣으면 회로가 계산한 출력이 나온다.

    python3 simulate.py pe2.txt --set A=5 --set B=3 --clock 4 --show PE0_U8
    python3 simulate.py pe2.txt --script run.txt        # 한 줄에 한 명령 (set/clock/show/assert)
    python3 simulate.py pe2.txt --selftest              # chips.txt 의 func 가 데이터시트대로인지 자체 점검

원칙 — **AI 는 관여하지 않는다.**
    회로는 KYLE(`pin`)이 정의하고, 칩의 동작은 chips.txt 의 `func`/`reg` 가 정의한다.
    이 프로그램은 그 둘만 읽어서 계산한다. 여기에 "이 회로는 곱셈기다" 같은 지식은 한 줄도 없다.

동작 방식
    네트 = 전선 한 가닥의 논리값 (0/1/None=미정).
    조합 논리: 값이 안 변할 때까지 칩의 func 를 반복 평가 (이벤트 없는 단순 수렴, 최대 200회).
    레지스터: chips.txt 의 `reg 출력 입력 클럭` 을 읽어, 클럭 네트가 0→1 이 될 때 입력을 출력으로 옮긴다.
    플래시(ROM): `flash` 플래그가 있는 칩은 주소핀(addr)으로 내용을 읽어 데이터핀(data)에 낸다.
                내용은 --rom REF=파일.bin 으로 준다. 없으면 0 을 낸다.
    3-state: 식이 None 을 내면 그 핀은 드라이브하지 않는다 (풀업이 있으면 1).
"""
import io, os, re, sys, argparse
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sol


class Sim:
    def __init__(self, d, verbose=False):
        self.d = d; self.verbose = verbose
        self.net = {}            # net -> 0/1/None
        self.ff = {}             # (ref, 출력핀이름) -> 0/1   레지스터가 기억하는 값
        self.rom = {}            # ref -> bytes
        self.prev_clk = {}       # (ref, 클럭네트) -> 이전 값
        self.driven = set()      # 바깥에서 고정한 네트 (입력 핀)
        self.shorts = []         # [(netA, netB)] 점퍼로 이은 네트 쌍 (실물 shunt)
        for n in d.nets: self.net[n] = None
        for n, (v, _i) in d.supplies.items():
            self.net[n] = 1 if v and v > 0.5 else 0
        # 풀업 저항: 한쪽이 VCC 인 저항의 반대쪽 네트는 기본 1
        self.pullup = set()
        for r in d.order:
            c = d.chips[d.parts[r]['chip']]
            if not c.pkg.startswith('RES'): continue
            pins = [d.pinnet.get((r, 1)), d.pinnet.get((r, 2))]
            if 'VCC' in pins:
                other = pins[0] if pins[1] == 'VCC' else pins[1]
                if other: self.pullup.add(other)
        # 레지스터 초기값: 실물 플립플롭도 전원을 넣으면 0 이나 1 중 하나로 시작한다.
        # 안 정하면 분주기(1nQ→1D 되먹임)가 영원히 '미정' 이라 클럭이 아예 안 돈다.
        self.reset()

    def reset(self, val=0):
        """모든 레지스터를 val 로 (반전 출력은 그 반대로)"""
        self.ff.clear(); self.prev_clk.clear()
        for ref in self.d.order:
            c = self.d.chips[self.d.parts[ref]['chip']]
            for outs, ins, clk in c.regs:
                if any('*' in x for x in outs + ins):
                    ob = outs[0].rstrip('*')
                    for k, (nm, kind, cls) in c.pins.items():
                        if re.match(rf'^{re.escape(ob)}\d+$', nm): self.ff[(ref, nm)] = val   # \\d 였음: raw f-string 에서 '역슬래시+d' 를 찾아 Q0..Q7 초기화가 안 됐다 (2026-09-18 DGX 쪽이 잡음)
                else:
                    base = outs[0]
                    for o in outs:
                        inv = o != base and o.replace('n', '', 1) == base
                        self.ff[(ref, o)] = (1 - val) if inv else val

    # ── 핀 ↔ 네트 ──
    def pin_net(self, ref, name):
        c = self.d.chips[self.d.parts[ref]['chip']]
        for k, (nm, kind, cls) in c.pins.items():
            if nm == name:
                n = self.d.pinnet.get((ref, k))
                return None if n == 'NC' else n          # NC 는 전선이 아니다 — 네트로 보면 출력 여럿이 'NC' 를 두고 값을 바꿔 영원히 '발진'
        return None

    def read(self, ref, name):
        n = self.pin_net(ref, name)
        if n is None: return None
        v = self.net.get(n)
        if v is None and n in self.pullup: return 1
        return v

    # ── 식 계산 ──
    # 핀 이름은 숫자로 시작할 수 있다 (1A, 1OE#, 2Y0) → 식별자를 먼저, 숫자 상수는 '숫자만' 일 때만.
    _TOK = re.compile(r'\s*(\(|\)|&|\||\^|~|[A-Za-z_0-9]*[A-Za-z_][A-Za-z_0-9#]*|\d+)')

    def eval_expr(self, ref, expr, local):
        """칩 ref 안에서 식을 계산. local 은 같은 칩의 이미 계산된 출력(내부 신호)."""
        toks = self._TOK.findall(expr)
        self._t = toks; self._i = 0
        v = self._or(ref, local)
        return v

    def _peek(self): return self._t[self._i] if self._i < len(self._t) else None
    def _next(self): t = self._peek(); self._i += 1; return t

    def _or(self, ref, local):
        v = self._xor(ref, local)
        while self._peek() == '|':
            self._next(); w = self._xor(ref, local)
            v = None if (v is None or w is None) else (v | w)
        return v

    def _xor(self, ref, local):
        v = self._and(ref, local)
        while self._peek() == '^':
            self._next(); w = self._and(ref, local)
            v = None if (v is None or w is None) else (v ^ w)
        return v

    def _and(self, ref, local):
        v = self._unary(ref, local)
        while self._peek() == '&':
            self._next(); w = self._unary(ref, local)
            v = None if (v is None or w is None) else (v & w)
        return v

    def _unary(self, ref, local):
        if self._peek() == '~':
            self._next(); v = self._unary(ref, local)
            return None if v is None else (1 - v)
        if self._peek() == '(':
            self._next(); v = self._or(ref, local)
            if self._peek() == ')': self._next()
            return v
        t = self._next()
        if t is None: return None
        if t.isdigit(): return int(t) & 1
        if t in local: return local[t]                     # 같은 칩 안의 중간 신호 (자리올림 등)
        return self.read(ref, t)                           # 핀 이름

    # ── 한 번 평가 ──
    def eval_comb(self):
        """조합 논리를 한 바퀴 돌린다. 값이 바뀌면 True."""
        changed = False
        for ref in self.d.order:
            p = self.d.parts[ref]; c = self.d.chips[p['chip']]
            if 'flash' in c.flags:
                changed |= self._eval_flash(ref, c)
                continue
            if not c.funcs: continue
            local = {}
            for out, expr in c.funcs.items():               # 선언 순서대로 (자리올림 사슬이 순서를 탄다)
                v = self.eval_expr(ref, expr, local)
                local[out] = v
                n = self.pin_net(ref, out)
                if n is not None and n not in self.driven and self.net.get(n) != v:
                    self.net[n] = v; changed = True
        # 레지스터 출력은 기억한 값을 계속 낸다
        for (ref, out), v in self.ff.items():
            n = self.pin_net(ref, out)
            if n is not None and n not in self.driven and self.net.get(n) != v:
                self.net[n] = v; changed = True
        return changed

    def _eval_flash(self, ref, c):
        """주소핀(addr)을 모아 ROM 내용을 데이터핀(data)에 낸다."""
        addr = {}; data = {}
        for k, (nm, kind, cls) in c.pins.items():
            m = re.match(r'^A(\d+)$', nm)
            if m and cls == 'addr': addr[int(m.group(1))] = nm
            m = re.match(r'^DQ(\d+)$', nm)
            if m: data[int(m.group(1))] = nm
        if not data: return False
        a = 0
        for bit, nm in addr.items():
            v = self.read(ref, nm)
            if v is None: return False                      # 주소가 아직 안 정해짐
            a |= (v & 1) << bit
        blob = self.rom.get(ref)
        byte = blob[a] if (blob and a < len(blob)) else 0
        changed = False
        for bit, nm in data.items():
            v = (byte >> bit) & 1
            n = self.pin_net(ref, nm)
            if n is not None and n not in self.driven and self.net.get(n) != v:
                self.net[n] = v; changed = True
        return changed

    def add_jumper(self, ref, spec):
        """점퍼(핀헤더 shunt): REF 의 두 핀 이름을 이어 같은 네트처럼 만든다. 예) JP_CLK=B-C"""
        p1, p2 = spec.split('-')
        n1, n2 = self.pin_net(ref, p1.strip()), self.pin_net(ref, p2.strip())
        if n1 and n2: self.shorts.append((n1, n2)); return True
        return False

    def _apply_shorts(self):
        """이은 네트끼리 값을 맞춘다 (한쪽이 정해져 있으면 다른 쪽도 그 값)"""
        ch = False
        for a, b in self.shorts:
            va, vb = self.net.get(a), self.net.get(b)
            v = va if va is not None else vb
            if v is None: continue
            if va != v and a not in self.driven: self.net[a] = v; ch = True
            if vb != v and b not in self.driven: self.net[b] = v; ch = True
        return ch

    def settle(self, limit=200):
        """조합 논리가 안정될 때까지 (수렴 안 하면 발진 — 그대로 알린다)"""
        for i in range(limit):
            ch = self.eval_comb()
            ch |= self._apply_shorts()
            if not ch: return i
        return None                                          # 수렴 실패

    def clock_edge(self):
        """모든 레지스터: 클럭 네트가 0→1 이면 D 를 Q 로.
        reg 문법 두 가지를 다 받는다:
          reg Q* D* CP        → Q0..Qn 이 D0..Dn 을 받는다 (* = 번호 전체)
          reg 1Q,1nQ 1D 1CP   → 1Q 는 1D, 1nQ 는 그 반대 (이름이 n 으로 시작하는 쪽이 반전 출력)
        먼저 모든 레지스터의 새 값을 '읽어서' 모아 둔 뒤 한꺼번에 적용한다 —
        안 그러면 같은 에지에서 앞 레지스터의 새 값이 뒤 레지스터에 새어 들어간다 (분주기가 특히 그렇다)."""
        pend = []
        for ref in self.d.order:
            c = self.d.chips[self.d.parts[ref]['chip']]
            for outs, ins, clk in c.regs:
                cn = self.pin_net(ref, clk)
                if cn is None: continue
                cur = self.net.get(cn); key = (ref, cn)
                prev = self.prev_clk.get(key); self.prev_clk[key] = cur
                if not (prev == 0 and cur == 1): continue          # 상승 에지만
                if any('*' in x for x in outs + ins):               # Q* D* : 번호별로 짝
                    ob = outs[0].rstrip('*'); ib = ins[0].rstrip('*')
                    for k, (nm, kind, cls) in c.pins.items():
                        m = re.match(rf'^{re.escape(ob)}(\d+)$', nm)
                        if not m: continue
                        v = self.read(ref, f'{ib}{m.group(1)}')
                        if v is not None: pend.append((ref, nm, v))
                else:                                               # 1Q,1nQ 1D 1CP
                    v = self.read(ref, ins[0])
                    if v is None: continue
                    base = outs[0]
                    for o in outs:
                        inv = ('n' in o[len(o) - len(o.lstrip('0123456789')):]) and o != base and o.replace('n', '', 1) == base
                        pend.append((ref, o, (1 - v) if inv else v))
        for ref, o, v in pend: self.ff[(ref, o)] = v
        return len(pend)

    def set_net(self, net, val, hold=True):
        """바깥(MEGA·측정 장비)에서 네트를 몬다. hold=True 면 조합 평가가 못 덮어쓴다
        (안 그러면 settle 한 번에 입력이 지워져, 넣은 값이 회로에 안 들어간다)."""
        self.net[net] = None if val is None else (val & 1)
        if val is None: self.driven.discard(net)
        elif hold: self.driven.add(net)

    def bus(self, ref, prefix):
        """REF 의 prefix0..N 핀을 모아 정수로 (예: bus('PE0_U8','Q') → Q0..Q7)"""
        c = self.d.chips[self.d.parts[ref]['chip']]
        bits = {}
        for k, (nm, kind, cls) in c.pins.items():
            m = re.match(rf'^{re.escape(prefix)}(\d+)$', nm)
            if m: bits[int(m.group(1))] = self.read(ref, nm)
        if not bits or any(v is None for v in bits.values()): return None
        return sum((bits[b] & 1) << b for b in bits)

    def set_bus(self, ref, prefix, value):
        c = self.d.chips[self.d.parts[ref]['chip']]
        n = 0
        for k, (nm, kind, cls) in c.pins.items():
            m = re.match(rf'^{re.escape(prefix)}(\d+)$', nm)
            if not m: continue
            net = self.d.pinnet.get((ref, k))
            if net is None: continue
            self.set_net(net, (value >> int(m.group(1))) & 1); n += 1
        return n


def selftest():
    """chips.txt 의 func 가 데이터시트 진리표대로인지 — 회로와 무관하게 칩 하나만 놓고 확인"""
    d = sol.Design(); d.chips = sol.parse_chips(os.path.join(HERE, 'chips.txt'))
    fails = []

    def mk(chip, ref='X'):
        dd = sol.Design(); dd.chips = d.chips
        dd.parts[ref] = dict(chip=chip, col=0, row=0, rot=0, layer=1); dd.order.append(ref)
        c = d.chips[chip]
        for k, (nm, kind, cls) in c.pins.items():
            if kind in ('vcc', 'gnd', 'undef'): continue
            net = f'{nm}'; dd.pinnet[(ref, k)] = net; dd.nets[net].append((ref, k))
        return Sim(dd), ref

    # 74HC04: 인버터
    s, r = mk('74HC04')
    for v in (0, 1):
        s.set_net('1A', v); s.settle()
        got = s.read(r, '1Y')
        if got != 1 - v: fails.append(f"74HC04 1A={v} → 1Y={got} (기대 {1-v})")

    # 74HC283: 4비트 가산기 — 전수 검사 (A 0..15, B 0..15, CIN 0/1)
    s, r = mk('74HC283')
    bad = 0
    for A in range(16):
        for B in range(16):
            for ci in (0, 1):
                for b in range(4):                                  # 핀은 A1..A4 (1부터)
                    s.set_net(f'A{b+1}', (A >> b) & 1); s.set_net(f'B{b+1}', (B >> b) & 1)
                s.set_net('CIN', ci); s.settle()
                got = sum((s.read(r, f'S{b+1}') or 0) << b for b in range(4)) + ((s.read(r, 'COUT') or 0) << 4)
                if got != A + B + ci: bad += 1
    if bad: fails.append(f"74HC283 가산기 {bad}/512 경우 틀림")

    # 74HC244: OE# 로 통과/차단
    s, r = mk('74HC244')
    s.set_net('1OE#', 0); s.set_net('1A0', 1); s.settle()
    if s.read(r, '1Y0') != 1: fails.append("74HC244 OE#=0 인데 안 통과")
    s.set_net('1OE#', 1); s.settle()
    if s.read(r, '1Y0') not in (0, None): fails.append("74HC244 OE#=1 인데 차단 안 됨")

    for f in fails: print("  FAIL " + f)
    print(f"\n자체 점검: {'전부 통과' if not fails else f'실패 {len(fails)}건'}")
    return 0 if not fails else 1


# ═══════════════ KYLE 의 sim … end 시나리오 실행 (sol.py R30 이 부른다) ═══════════════
def _resolve(s, spec):
    """'MEGA.D' → 버스(D0..D7) / 'MEGA.WCP0' → 핀 하나 / '118' → 네트"""
    if '.' not in spec: return ('net', spec, None)
    ref, nm = spec.split('.', 1)
    if ref not in s.d.parts: raise ValueError(f"부품 {ref} 없음")
    c = s.d.chips[s.d.parts[ref]['chip']]
    for k, (pn, kind, cls) in c.pins.items():
        if pn == nm: return ('pin', ref, nm)
    if any(re.match(rf'^{re.escape(nm)}\d+$', pn) for pn, kind, cls in c.pins.values()): return ('bus', ref, nm)
    raise ValueError(f"{ref} 에 핀 {nm} 도 버스 {nm}0.. 도 없음")

def _set(s, spec, v):
    kind, a, b = _resolve(s, spec)
    if kind == 'net': s.set_net(a, v)
    elif kind == 'pin':
        n = s.pin_net(a, b)
        if n is None: raise ValueError(f"{spec} 이 어느 네트에도 안 붙음")
        s.set_net(n, v & 1)
    else: s.set_bus(a, b, v)

def _get(s, spec):
    kind, a, b = _resolve(s, spec)
    if kind == 'net': return s.net.get(a)
    if kind == 'pin': return s.read(a, b)
    return s.bus(a, b)

def _cycle(s, spec):
    """클럭 한 번: 0 → 1 → (레지스터 갱신) → 안정"""
    _set(s, spec, 0); s.settle(); s.clock_edge()
    _set(s, spec, 1); s.settle(); s.clock_edge()
    return s.settle()

def run_scenario(d, sc, base='.'):
    """KYLE 의 sim 블록 하나를 돌린다. 결과 = [dict(msg, ok, warn)] — ok=False 면 기대값과 다름 (R30 오류)."""
    s = Sim(d); out = []; rom_missing = False
    ins = []
    for st in sc['steps']:
        k = st[0]
        try:
            if k == 'rom':
                path = st[2] if os.path.isabs(st[2]) else os.path.join(base, st[2])
                if not os.path.exists(path):
                    out.append(dict(msg=f"rom {st[1]} 파일 없음: {st[2]} (이 시나리오의 out 은 건너뜀)", ok=None, warn=True)); rom_missing = True
                else: s.rom[st[1]] = io.open(path, 'rb').read()
            elif k == 'jumper':
                if not s.add_jumper(st[1], st[2]): out.append(dict(msg=f"jumper {st[1]} {st[2]}: 핀을 못 찾음", ok=None, warn=True))
            elif k == 'in':
                v = int(st[2], 0); _set(s, st[1], v); ins.append(f"{st[1]}={v}")
                if s.settle() is None: out.append(dict(msg=f"in {st[1]} 뒤 조합 논리가 수렴 안 함 (발진)", ok=None, warn=True))
            elif k == 'pulse':
                if _cycle(s, st[1]) is None: out.append(dict(msg=f"pulse {st[1]} 뒤 수렴 안 함", ok=None, warn=True))
                _set(s, st[1], 0); s.settle(); s.clock_edge(); s.settle()
            elif k == 'clock':
                for i in range(int(st[2])):
                    if _cycle(s, st[1]) is None: out.append(dict(msg=f"clock {st[1]} {i+1}번째 뒤 수렴 안 함", ok=None, warn=True)); break
            elif k in ('out', 'show'):
                v = _get(s, st[1]); vs = '미정' if v is None else f"{v} (0x{v:X})"
                if k == 'show': out.append(dict(msg=f"{st[1]} = {vs}", ok=None)); continue
                if rom_missing: continue
                exp = int(st[2], 0)
                ok = (v == exp)
                out.append(dict(msg=f"{st[1]} = {vs}  기대 {exp} (0x{exp:X})  {'✔' if ok else '✘ 다름'}" + (f"  [입력 {', '.join(ins)}]" if not ok and ins else ''), ok=ok))
        except Exception as e:
            out.append(dict(msg=f"{' '.join(st)}: {e}", ok=False))
    if not any(r.get('ok') is not None for r in out) and not rom_missing:
        out.append(dict(msg="out 문이 없어 대조할 값이 없음 (show 만 있음)", ok=None, warn=True))
    return out

def make_mul_rom(dirpath):
    """곱셈표 ROM 두 개(하위/상위 바이트) — 주소 = (w<<8)|x. 플래시에 구울 데이터 파일이지 회로 지식이 아니다."""
    LO = bytearray(65536); HI = bytearray(65536)
    for w in range(256):
        for x in range(256):
            p = w * x; LO[(w << 8) | x] = p & 0xFF; HI[(w << 8) | x] = (p >> 8) & 0xFF
    a = os.path.join(dirpath, 'mul_lo.bin'); b = os.path.join(dirpath, 'mul_hi.bin')
    io.open(a, 'wb').write(bytes(LO)); io.open(b, 'wb').write(bytes(HI))
    return a, b


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src', nargs='?'); ap.add_argument('--set', action='append', default=[], metavar='REF.PREFIX=값')
    ap.add_argument('--net', action='append', default=[], metavar='네트=값')
    ap.add_argument('--rom', action='append', default=[], metavar='REF=파일')
    ap.add_argument('--jumper', action='append', default=[], metavar='REF=A-C', help='점퍼 꽂기 (실물 shunt)')
    ap.add_argument('--clock', type=int, default=0, help='클럭을 몇 번 칠지 (상승 에지)')
    ap.add_argument('--clknet', default=None, help='클럭 네트 이름 (기본: OSC 출력이 붙은 네트)')
    ap.add_argument('--show', action='append', default=[], metavar='REF.PREFIX')
    ap.add_argument('--selftest', action='store_true'); ap.add_argument('-v', action='store_true')
    ap.add_argument('--run', nargs='?', const='*', default=None, metavar='이름', help='KYLE 의 sim … end 시나리오를 돌린다 (이름 없으면 전부)')
    ap.add_argument('--mulrom', default=None, metavar='폴더', help='곱셈표 ROM(mul_lo.bin/mul_hi.bin) 을 만든다')
    a = ap.parse_args()
    if a.selftest: sys.exit(selftest())
    if a.mulrom:
        for f in make_mul_rom(a.mulrom): print(f); return
    if not a.src: ap.error('설계 파일이 필요합니다')
    if a.run is not None:
        d = sol.parse_design(a.src)
        if d.errors: raise SystemExit('KYLE 오류: ' + '; '.join(d.errors[:3]))
        bad = 0; n = 0
        for sc in d.sims:
            if a.run != '*' and sc['name'] != a.run: continue
            n += 1; print(f"sim {sc['name']}")
            for r in run_scenario(d, sc, d.base):
                print(("  ✘ " if r.get('ok') is False else "  ! " if r.get('warn') else "  · ") + r['msg'])
                if r.get('ok') is False: bad += 1
        if n == 0: print("  (sim 블록이 없음)")
        print("결과:", "전부 일치" if bad == 0 else f"{bad}개 다름"); sys.exit(1 if bad else 0)

    d = sol.parse_design(a.src)
    if d.errors: raise SystemExit('KYLE 오류: ' + '; '.join(d.errors[:3]))
    s = Sim(d, verbose=a.v)
    for spec in a.rom:
        ref, path = spec.split('=', 1); s.rom[ref] = io.open(path, 'rb').read()
    for spec in a.jumper:
        ref, pp = spec.split('=', 1)
        if not s.add_jumper(ref.strip(), pp.strip()): print(f'  ! 점퍼 {spec} 핀을 못 찾음')
    for spec in a.net:
        n, v = spec.split('='); s.set_net(n.strip(), int(v))
    for spec in a.set:
        lhs, v = spec.split('='); ref, pre = lhs.split('.')
        cnt = s.set_bus(ref.strip(), pre.strip(), int(v, 0))
        print(f"  입력 {ref}.{pre}* = {int(v,0)} ({cnt}비트)")
    # 클럭 네트 찾기
    clknet = a.clknet
    if clknet is None:
        for ref in d.order:
            c = d.chips[d.parts[ref]['chip']]
            if 'osc' in c.flags or c.name == 'OSC_FULL':
                clknet = s.pin_net(ref, 'OUT'); break
    it = s.settle()
    if it is None: print("  ! 조합 논리가 수렴하지 않았다 (발진 또는 되먹임 고리)")
    for i in range(a.clock):
        if clknet is None: print("  ! 클럭 네트를 못 찾았다 — --clknet 으로 지정하세요"); break
        s.set_net(clknet, 0); s.settle(); s.clock_edge()
        s.set_net(clknet, 1); s.settle(); n = s.clock_edge()
        if s.settle() is None: print(f"  ! {i+1}번째 클럭 뒤 수렴 실패")
        if a.v: print(f"  클럭 {i+1}: 레지스터 {n}개 갱신")
    for spec in a.show:
        ref, pre = spec.split('.')
        v = s.bus(ref.strip(), pre.strip())
        print(f"  {spec} = {v if v is not None else '미정'}" + (f"  (0x{v:X})" if v is not None else ''))


if __name__ == '__main__':
    main()
