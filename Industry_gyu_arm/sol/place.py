#!/usr/bin/env python3
"""
place — KYLE 자동 배치기.  KYLE 을 읽어 부품 자리(part … at … rot … layer)·층간 헤더(stack)·전원 버스(bus)를 새로 정해 KYLE 로 쓴다.

    python3 place.py pe2.txt --out pe2_placed.txt [--board 84 62] [--layers 2] [--seed 3] [--iters 40000] [--channel 2]
    python3 place.py pe2.txt --out best.txt --search 6 --route        ← 씨앗 6개 돌려 '실제 배선' 으로 채점, 최고를 남김

원칙: 입력도 출력도 SOL. 연결(pin)·기대값(expect)·색·설정은 손대지 않는다. 배치 힌트는 KYLE 토큰으로 읽는다:
    part C1 CAP100n at 15 2 near PE0_U2    ← 이 부품(디커플링 캡)은 PE0_U2 의 VCC 핀 옆에
    part MEGA MEGA at 2 47 fixed           ← 옮기지 않는다 (가장자리 커넥터)

알고리즘 (담금질, simulated annealing):
    비용 = Σ 네트 HPWL(반둘레 배선길이, 구멍 단위)
         + 층을 넘는 네트마다 CROSS (라이저를 타야 한다)
         + 칩 GND/VCC 핀 행이 흩어진 정도 (같은 행에 모이면 버스 한 줄로 끝난다)
         + near 힌트 거리 × NEAR_W
    합법성: 칩(DIP/CONN) 상자 사이 채널 ≥ channel 구멍, 작은 부품(RES/CAP) 은 1구멍, 기판 안, 층별로.
    이동: 옮기기 / 자리 바꾸기 / 회전 / 층 바꾸기.  온도는 기하급수로 식는다.
    배치 뒤: GND/VCC 버스 행 = 칩 GND/VCC 핀 행의 군집 → bus 문장.  층>1 이면 층을 넘는 네트를 열(col) 기준으로 묶어
    두 층 모두 빈 20행짜리 열에 HDR20 라이저를 세우고 pin J*[k:net] 을 쓴다.

채점: --route 를 주면 route_eval 로 실제 자동배선을 돌려 미배선 수(+R28 독립 검사)로 고른다. 담금질의 비용은 근사고, 배선이 심판이다.
"""
import io, os, re, sys, math, random, argparse, copy
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sol
from sol import pkg_geom, pin_hole

# ───────────────────────── 기하 ─────────────────────────
def is_big(pkg): return pkg.startswith(('DIP', 'CONN'))
def part_pins(p):
    n = pkg_geom(p['_pkg'])[0]
    return [pin_hole(p, k) for k in range(1, n+1)]
_FP = {}
def _fp(pkg, rot):
    """(pkg,rot) → (dx0,dy0,dx1,dy1) 상자 상대 오프셋 (1번 핀 = 원점)"""
    k = (pkg, rot)
    if k not in _FP:
        q = dict(_pkg=pkg, col=0, row=0, rot=rot); ps = part_pins(q)
        cs = [c for c, r in ps]; rs = [r for c, r in ps]; _FP[k] = (min(cs), min(rs), max(cs), max(rs))
    return _FP[k]
def bbox(p):
    d = _fp(p['_pkg'], p['rot']); return (p['col'] + d[0], p['row'] + d[1], p['col'] + d[2], p['row'] + d[3])
def gap_needed(pa, pb, channel):
    a, b = is_big(pa['_pkg']), is_big(pb['_pkg'])
    if a and b: return channel
    if a or b: return 1
    return 0
def boxes_clash(A, B, g):
    # A 를 g 만큼 넓혀 B 와 겹치면 충돌
    return not (A[2] + g < B[0] or B[2] + g < A[0] or A[3] + g < B[1] or B[3] + g < A[1])

class Placer:
    def __init__(self, d, W, H, layers, channel=2, seed=0, margin=1):
        self.d = d; self.W = W; self.H = H; self.L = max(1, layers); self.channel = channel; self.margin = margin
        self.rng = random.Random(seed)
        self.refs = [r for r in d.order if r not in d.stacks]
        self.P = {}
        for r in self.refs:
            p = dict(d.parts[r]); p['_pkg'] = d.chips[p['chip']].pkg; p['layer'] = min(self.L, p.get('layer', 1) or 1); self.P[r] = p
        # 네트 → 핀 목록 (스택 헤더 핀 제외)
        self.netpins = {n: [(r, k) for r, k in pl if r in self.P] for n, pl in d.nets.items()}
        self.netpins = {n: self.netpins[n] for n in sorted(self.netpins) if len(self.netpins[n]) >= 2 and n not in d.supplies}
        self.partnets = {r: set() for r in self.refs}
        for n, pl in self.netpins.items():
            for r, k in pl: self.partnets[r].add(n)
        self.gnd_pin = {}; self.vcc_pin = {}
        for r in self.refs:
            c = d.chips[self.P[r]['chip']]
            if not is_big(c.pkg): continue
            for k, (nm, kind, cls) in c.pins.items():
                if kind == 'gnd': self.gnd_pin[r] = k
                if kind == 'vcc': self.vcc_pin[r] = k
        # 네트 가중치: 동시 스위칭 출력이 많은 칩(574·244, sso ≥ 8)이 모는 네트는 2배 — 그 출력선이 길어지면 부하 pF 가 늘어
        # 그라운드 바운스(R16)가 LOW 여유를 넘긴다. 짧게 모아야 한다.
        self.netw = {}
        for n, pl_ in self.netpins.items():
            w = 1.0
            for r, k in pl_:
                c = d.chips[self.P[r]['chip']]; info = c.pin(k)
                if info and info[1] == 'out':
                    sso = c.attr.get('sso', 0); sso = len(sso) if isinstance(sso, (list, tuple, set)) else sso
                    if sso >= 8: w = 2.0
            self.netw[n] = w
        self.pos = {}   # ref -> (col,row,rot,layer)
        for r in self.refs: p = self.P[r]; self.pos[r] = (p['col'], p['row'], p['rot'], p['layer'])
        self.box = {r: bbox(self.P[r]) for r in self.refs}
        # near 힌트가 있는 작은 부품(디커플링 캡)은 그 칩에 '붙어' 다닌다: 칩 VCC 핀 쪽 바깥 2열을 캡 자리로 예약하고, 배치 뒤 거기 놓는다
        self.attached = {}   # chip -> [cap refs]
        for r in self.refs:
            near = self.P[r].get('near')
            if near and near in self.P and not is_big(self.P[r]['_pkg']): self.attached.setdefault(near, []).append(r)
        self.attached_caps = {c for caps in self.attached.values() for c in caps}
        self.unplaced = set()
        self.occ_big = {}; self.occ_small = {}     # layer -> {cell: count}
        self.cells_of = {}                          # ref -> (layer, big, frozenset(cells)) 현재 등록된 칸
        for r in self.refs: self._occ_add(r)
        self.CROSS = 14.0; self.ALIGN_W = 60.0; self.NEAR_W = 3.0; self.CONG_W = 4.0
        self.netcost = {n: self._hpwl(n) for n in self.netpins}
        self._global_cache = self._global()
        self.total = sum(self.netcost.values()) + self._global_cache + self._congestion()

    # ── 비용 ──
    def _pinpos(self, r, k):
        return pin_hole(self.P[r], k)
    def _hpwl(self, n):
        cs, rs, lays = [], [], set()
        for r, k in self.netpins[n]:
            c, rr = self._pinpos(r, k); cs.append(c); rs.append(rr); lays.add(self.P[r]['layer'])
        return self.netw.get(n, 1.0) * ((max(cs) - min(cs)) + (max(rs) - min(rs))) + (self.CROSS * (len(lays) - 1))
    def _global(self):
        # 전원 핀 행 정렬 + near 힌트
        g = 0.0
        for pinmap in (self.gnd_pin, self.vcc_pin):
            rows = {}
            for r, k in pinmap.items():
                _, rr = self._pinpos(r, k); rows.setdefault(self.P[r]['layer'], set()).add(rr)
            g += self.ALIGN_W * sum(max(0, len(s) - 1) for s in rows.values())
        for r in self.refs:
            near = self.P[r].get('near')
            if near and near in self.P:
                vk = self.vcc_pin.get(near)
                if vk is None: continue
                vc, vr = self._pinpos(near, vk); b = self.box[r]
                dist = abs((b[0] + b[2]) / 2 - vc) + abs((b[1] + b[3]) / 2 - vr) + (0 if self.P[r]['layer'] == self.P[near]['layer'] else 20)
                g += self.NEAR_W * max(0, dist - 2)
        return g

    def _net_box(self, n):
        cs, rs = [], []
        for r, k in self.netpins[n]:
            c, rr = self._pinpos(r, k); cs.append(c); rs.append(rr)
        return (min(cs), min(rs), max(cs), max(rs))
    def _rudy_add(self, n, sign):
        x0, y0, x1, y1 = self._net_box(n) if sign > 0 else self.nbox[n]
        area = max(1, (x1 - x0 + 1) * (y1 - y0 + 1)); w = sign * ((x1 - x0) + (y1 - y0)) / area
        dens, over = self.dens, 0.0
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                cell = (x, y); v0 = dens.get(cell, 0.0); v1 = v0 + w; dens[cell] = v1
                cap = 1.0 if cell in self.under else 2.0
                over += (max(0.0, v1 - cap) ** 2) - (max(0.0, v0 - cap) ** 2)
        if sign > 0: self.nbox[n] = (x0, y0, x1, y1)
        return over
    def _under_cells(self):
        under = set()
        for r in self.refs:
            if not is_big(self.P[r]['_pkg']): continue
            b = self.box[r]
            for x in range(b[0], b[2] + 1):
                for y in range(b[1], b[3] + 1): under.add((x, y))
        return under
    def _congestion(self):
        # RUDY 근사: 네트마다 bbox 위에 (HPWL / 넓이) 를 얹는다. 용량: 칩 상자 아래(앞면 막힘) 1, 그 밖 2. 초과분의 제곱 합.
        self.under = self._under_cells(); self.dens = {}; self.nbox = {}; self.over = 0.0
        for n in self.netpins: self.over += self._rudy_add(n, +1)
        return self.CONG_W * self.over
    def _cap_cells(self, cells, entering):
        """칩 상자 칸의 용량이 바뀔 때(2↔1) 그 칸들의 초과분만 다시 센다"""
        d = 0.0
        for cell in cells:
            v = self.dens.get(cell, 0.0)
            if entering:  # 2 → 1
                if cell in self.under: continue
                self.under.add(cell); d += max(0.0, v - 1.0) ** 2 - max(0.0, v - 2.0) ** 2
            else:         # 1 → 2
                if cell not in self.under: continue
                self.under.discard(cell); d += max(0.0, v - 2.0) ** 2 - max(0.0, v - 1.0) ** 2
        return d
    def _cong_delta(self, affected, moves_boxes):
        """이동한 부품의 네트만 갱신 + 칩이 움직였으면 그 상자 칸들의 용량 변화만 반영 (전체 재계산 없음)"""
        d = 0.0
        for old_box, new_box, big in moves_boxes:
            if not big or old_box == new_box: continue
            cells_old = {(x, y) for x in range(old_box[0], old_box[2] + 1) for y in range(old_box[1], old_box[3] + 1)}
            cells_new = {(x, y) for x in range(new_box[0], new_box[2] + 1) for y in range(new_box[1], new_box[3] + 1)}
            d += self._cap_cells(cells_old - cells_new, False)
            d += self._cap_cells(cells_new - cells_old, True)
        for n in affected: d += self._rudy_add(n, -1)
        for n in affected: d += self._rudy_add(n, +1)
        self.over += d
        return self.CONG_W * d
    # ── 격자 점유 ──
    def _box_cells(self, r, box):
        b = self._ext_box(r, box) if r in self.attached else box
        return {(x, y) for x in range(b[0], b[2] + 1) for y in range(b[1], b[3] + 1)}
    def _occ_add(self, r):
        if r in self.attached_caps: return
        L = self.P[r]['layer']; big = is_big(self.P[r]['_pkg']); cells = self._box_cells(r, self.box[r])
        g = self.occ_big.setdefault(L, {}) if big else self.occ_small.setdefault(L, {})
        for c in cells: g[c] = g.get(c, 0) + 1
        self.cells_of[r] = (L, big, cells)
    def _occ_remove(self, r):
        if r not in self.cells_of: return
        L, big, cells = self.cells_of.pop(r)
        g = self.occ_big[L] if big else self.occ_small[L]
        for c in cells:
            g[c] -= 1
            if g[c] == 0: del g[c]
    # ── 합법성 ──
    def cap_slot(self, chip, box=None, rot=None, at=None):
        """칩의 VCC 핀에서 바깥쪽으로 gap 1 두고 2×2 캡 자리 (col0,row0,col1,row1). at/rot 을 주면 그 후보 배치 기준."""
        p = self.P[chip]; vk = self.vcc_pin.get(chip)
        if vk is None: return None
        col, row = at if at is not None else (p['col'], p['row']); rot = p['rot'] if rot is None else rot
        d = _fp(p['_pkg'], rot); b = (col + d[0], row + d[1], col + d[2], row + d[3])
        vc, vr = pin_hole(dict(_pkg=p['_pkg'], col=col, row=row, rot=rot), vk)
        if vc >= b[2]:   return (b[2] + 2, max(b[1], vr - 1), b[2] + 3, max(b[1], vr - 1) + 1)
        if vc <= b[0]:   return (b[0] - 3, max(b[1], vr - 1), b[0] - 2, max(b[1], vr - 1) + 1)
        if vr <= b[1]:   return (vc, b[1] - 3, vc + 1, b[1] - 2)
        return (vc, b[3] + 2, vc + 1, b[3] + 3)
    def _ext_box(self, r, box):
        """합법성용 상자: 붙은 캡 자리를 포함"""
        if r not in self.attached: return box
        cs = self.cap_slot(r, box)
        if cs is None: return box
        return (min(box[0], cs[0]), min(box[1], cs[1]), max(box[2], cs[2]), max(box[3], cs[3]))
    def _legal(self, r, cand_box, layer, ignore=(), rot=None, at=None, self_out=False):
        """후보 상자가 기판 안이고, 다른 부품(간격 포함)과 안 겹치나. O(상자 넓이). self_out=True 면 r 은 이미 격자에서 빠져 있다."""
        m = self.margin
        if r in self.attached:
            cs = self.cap_slot(r, cand_box, rot, at)
            if cs: cand_box = (min(cand_box[0], cs[0]), min(cand_box[1], cs[1]), max(cand_box[2], cs[2]), max(cand_box[3], cs[3]))
        if cand_box[0] < m or cand_box[1] < m or cand_box[2] > self.W - 1 - m or cand_box[3] > self.H - 1 - m: return False
        big = is_big(self.P[r]['_pkg'])
        # 자기 자신(과 ignore) 의 칸은 잠시 뺀다
        removed = [x for x in ((() if self_out else (r,)) + tuple(ignore)) if x in self.cells_of]
        for x in removed: self._occ_remove(x)
        try:
            gb = self.occ_big.get(layer, {}); gs = self.occ_small.get(layer, {})
            g1 = self.channel if big else 1          # 상대가 큰 부품일 때 필요한 간격
            g2 = 1 if big else 0                     # 상대가 작은 부품일 때
            for x in range(cand_box[0] - g1, cand_box[2] + g1 + 1):
                for y in range(cand_box[1] - g1, cand_box[3] + g1 + 1):
                    if (x, y) in gb: return False
            for x in range(cand_box[0] - g2, cand_box[2] + g2 + 1):
                for y in range(cand_box[1] - g2, cand_box[3] + g2 + 1):
                    if (x, y) in gs: return False
            return True
        finally:
            for x in removed: self._occ_add(x)
    def _apply(self, r, col, row, rot, layer):
        self._occ_remove(r)
        p = self.P[r]; p['col'], p['row'], p['rot'], p['layer'] = col, row, rot, layer
        self.pos[r] = (col, row, rot, layer); self.box[r] = bbox(p)
        if not (self.unplaced and r in self.unplaced): self._occ_add(r)
    def _nearest_legal(self, r, col, row, rot, layer, rad=5):
        was_in = r in self.cells_of
        if was_in: self._occ_remove(r)
        try:
            for d in range(rad + 1):
                ring = [(col, row)] if d == 0 else [(col + dx, row + dy) for dx in range(-d, d + 1) for dy in (-d, d)] + [(col + dx, row + dy) for dy in range(-d + 1, d) for dx in (-d, d)]
                self.rng.shuffle(ring)
                for c, rr in ring:
                    if self._legal(r, self._try_box(r, c, rr, rot, layer), layer, rot=rot, at=(c, rr), self_out=True): return (c, rr)
            return None
        finally:
            if was_in: self._occ_add(r)
    def _try_box(self, r, col, row, rot, layer):
        d = _fp(self.P[r]['_pkg'], rot); return (col + d[0], row + d[1], col + d[2], row + d[3])

    # ── 층 초기 배정 ──
    def _net_graph(self):
        """KYLE 의 pin 연결만으로 만든 부품 그래프: 두 부품이 같은 네트를 공유하면 가중치 +1/(핀수-1).
        (핀 많은 버스 네트가 모든 쌍을 강하게 묶는 걸 막는다 — 표준 하이퍼그래프 가중치)"""
        W = {}
        for n, pins in self.netpins.items():
            refs = sorted({r for r, k in pins})
            if len(refs) < 2: continue
            w = 1.0 / (len(refs) - 1)
            for i, a in enumerate(refs):
                for b in refs[i+1:]:
                    key = (a, b) if a < b else (b, a)
                    W[key] = W.get(key, 0.0) + w
        return W

    def init_layers_graph(self, verbose=False):
        """★ KYLE 의 연결만 보고 층을 나눈다 (Kernighan–Lin 식 교환).
        목표: ①층마다 면적이 고르게 ②층을 넘는 연결(=라이저로 올려야 하는 네트)을 최소로.
        이름 규칙(PE0/PE1)에 기대지 않으므로, 어떤 회로에도 쓸 수 있다."""
        if self.L < 2: return
        movable = [r for r in self.refs if not self.P[r].get('fixed') and r not in self.attached_caps]
        if not movable: return
        Wg = self._net_graph()
        area = {r: (self.box[r][2]-self.box[r][0]+1) * (self.box[r][3]-self.box[r][1]+1) for r in self.refs}
        adj = {r: {} for r in self.refs}
        for (a, b), w in Wg.items():
            if a in adj and b in adj: adj[a][b] = w; adj[b][a] = w
        # 시작: 연결이 강한 것끼리 뭉쳐서(씨앗에서 넓혀가며) 층을 채운다.
        # 넓은 것부터 빈 층에 던지면 연결된 부품이 흩어져 오히려 이름 규칙보다 나빠진다 (실측 44 → 65.7).
        lay = {r: self.P[r]['layer'] for r in self.refs}
        for r in self.refs:
            if r not in movable: lay[r] = min(self.L, max(1, lay[r]))
        load = {L: sum(area[r] for r in self.refs if lay[r] == L and r not in movable) for L in range(1, self.L+1)}
        target = sum(area[r] for r in self.refs) / self.L
        left = set(movable)
        for L in range(1, self.L + 1):
            if not left: break
            # 씨앗: 남은 것 중 이미 배치된 이웃이 가장 적은(=새 덩어리를 시작하기 좋은) 부품
            seed_r = max(sorted(left), key=lambda r: (area[r], sum(adj[r].get(o, 0) for o in left)))
            lay[seed_r] = L; load[L] += area[seed_r]; left.discard(seed_r)
            while left and load[L] < target:
                # 이 층과 가장 강하게 연결된 부품을 끌어온다
                cand = max(sorted(left), key=lambda r: (sum(w for o, w in adj[r].items() if lay.get(o) == L), -area[r]))
                if sum(w for o, w in adj[cand].items() if lay.get(o) == L) <= 0 and load[L] > 0:
                    cand = max(sorted(left), key=lambda r: area[r])       # 연결이 끊긴 덩어리면 넓은 것부터
                lay[cand] = L; load[L] += area[cand]; left.discard(cand)
        for r in sorted(left, key=lambda x: -area[x]):
            L = min(range(1, self.L+1), key=lambda i: load[i]); lay[r] = L; load[L] += area[r]
        cap = sum(area[r] for r in self.refs) / self.L * 1.25      # 층당 면적 상한 (25% 여유)
        def cut():
            return sum(w for (a, b), w in Wg.items() if a in lay and b in lay and lay[a] != lay[b])
        best = cut()
        for _ in range(12):
            improved = False
            for r in sorted(movable, key=lambda x: -area[x]):
                cur = lay[r]
                gain_cur = sum(w for o, w in adj[r].items() if lay.get(o) == cur)
                for L2 in range(1, self.L+1):
                    if L2 == cur or load[L2] + area[r] > cap: continue
                    gain_new = sum(w for o, w in adj[r].items() if lay.get(o) == L2)
                    if gain_new > gain_cur + 1e-9:
                        load[cur] -= area[r]; load[L2] += area[r]; lay[r] = L2
                        gain_cur = gain_new; cur = L2; improved = True
            if not improved: break
        if verbose: print(f"  층 분할: 자른 연결 {best:.1f} → {cut():.1f}, 층별 면적 {[round(load[L]) for L in range(1, self.L+1)]}")
        for r in movable:
            c, rr, rot, _ = self.pos[r]; self._apply(r, c, rr, rot, lay[r])
        for chip in sorted(self.attached):
            for cp in self.attached[chip]:
                c, rr, rot, _ = self.pos[cp]; self._apply(cp, c, rr, rot, self.P[chip]['layer'])
        self.netcost = {n: self._hpwl(n) for n in self.netpins}; self._global_cache = self._global()
        self.total = sum(self.netcost.values()) + self._global_cache + self._congestion()

    def init_layers(self, mode='prefix'):
        """층>1 일 때 시작 층 배정. 'prefix': 부품 이름의 앞 토큰(PE0_, PE1_, …)으로 묶어 큰 묶음부터 층에 번갈아 배정 —
        한 PE 가 한 층에 통째로 가면 층당 밀도가 반이 되고 층을 넘는 네트는 PE 사이 연결뿐이다.
        (담금질의 한 부품씩 층 바꾸기로는 이 배정을 못 찾는다: 혼자 건너가면 자기 네트가 전부 층을 넘어 벌점만 받는다)"""
        if self.L < 2 or mode == 'none': return
        groups = {}
        for r in self.refs:
            if self.P[r].get('fixed'): continue
            key = r.split('_')[0] if '_' in r else '_misc'
            groups.setdefault(key, []).append(r)
        order = sorted(groups.items(), key=lambda kv: -len(kv[1]))
        load = [0] * self.L
        for key, refs in order:
            if key == '_misc': L = 1
            else: L = 1 + min(range(self.L), key=lambda i: load[i])
            load[L - 1] += len(refs)
            for r in refs:
                c, rr, rot, _ = self.pos[r]; self._apply(r, c, rr, rot, L)
        for chip in sorted(self.attached):
            caps = self.attached[chip]
            for cp in caps:
                c, rr, rot, _ = self.pos[cp]; self._apply(cp, c, rr, rot, self.P[chip]['layer'])
        self.netcost = {n: self._hpwl(n) for n in self.netpins}; self._global_cache = self._global()
        self.total = sum(self.netcost.values()) + self._global_cache + self._congestion()

    # ── 시작 상태 합법화 (건설식) ──
    def legalize(self, max_rad=40, keep_layer=False):
        """기판을 비우고 고정 부품 → 큰 칩(넓이순) → 작은 부품 순서로, 각자 원래 자리에서 가장 가까운 합법 자리에 다시 놓는다.
        담금질은 합법 상태에서만 출발해야 한다 (불법 쌍은 이동 검사로는 절대 안 풀린다)."""
        order = sorted([r for r in self.refs if r not in self.attached_caps],
                       key=lambda r: (0 if self.P[r].get('fixed') else 1, 0 if is_big(self.P[r]['_pkg']) else 1,
                                      -((self.box[r][2]-self.box[r][0]+1)*(self.box[r][3]-self.box[r][1]+1))))
        self.fail_place = []
        # fixed 라도 기판 밖이면 옮겨야 한다 (설계 파일이 더 큰 기판 기준이었을 때: 벌크 캡·커넥터)
        m = self.margin
        outside = {r for r in order if self.P[r].get('fixed')
                   and (self.box[r][0] < m or self.box[r][1] < m or self.box[r][2] > self.W-1-m or self.box[r][3] > self.H-1-m)}
        for r in outside: print(f"  · {r} 이(가) 기판 밖이라 fixed 를 풀고 옮긴다")
        self.unplaced = (set(order) - {r for r in order if self.P[r].get('fixed')}) | outside
        for r in self.unplaced: self._occ_remove(r)
        moved = 0
        for r in order:
            if r not in self.unplaced: continue
            self.unplaced.discard(r)
            c, rr, rot, L = self.pos[r]
            if self._legal(r, self.box[r], L, rot=rot, at=(c, rr), self_out=True): self._occ_add(r); continue
            spot = self._nearest_legal(r, c, rr, rot, L, rad=max_rad)
            if spot is None:
                rot2 = 90 if rot == 0 else 0; spot = self._nearest_legal(r, c, rr, rot2, L, rad=max_rad)
                if spot: rot = rot2
            if spot is None:
                # 근처에 없으면 기판 전체·모든 층·양쪽 회전에서 찾는다 (작은 부품은 어디든 갈 수 있다)
                for L2 in ([L] if keep_layer else [L] + [x for x in range(1, self.L + 1) if x != L]):
                    for rot3 in (rot, 90 if rot == 0 else 0):
                        spot = self._nearest_legal(r, self.W // 2, self.H // 2, rot3, L2, rad=max(self.W, self.H))
                        if spot: rot, L = rot3, L2; break
                    if spot: break
            if spot is None:
                print(f"  ! {r} 합법 자리 없음 — 기판({self.W}x{self.H} {self.L}층)이 부족하다. 기판을 키우거나 층을 늘리거나 --channel 을 줄일 것")
                self.fail_place.append(r); self._occ_add(r); continue
            self._apply(r, spot[0], spot[1], rot, L); moved += 1
        self.unplaced = set()
        self.netcost = {n: self._hpwl(n) for n in self.netpins}; self._global_cache = self._global()
        self.total = sum(self.netcost.values()) + self._global_cache + self._congestion()
        return moved

    # ── 배치 뒤: 전원 핀 행을 버스 행에 맞춰 살짝 당긴다 ──
    def align_power(self, reach=2):
        """칩을 ±reach 행 안에서 옮겨 GND 핀 행이 이미 있는 버스 행(가장 많이 쓰이는 행)에 오도록. 합법이고 비용이 크게 나빠지지 않을 때만."""
        moved = 0
        for pinmap in (self.gnd_pin, self.vcc_pin):
            rows = {}
            for r, k in pinmap.items(): rows[self._pinpos(r, k)[1]] = rows.get(self._pinpos(r, k)[1], 0) + 1
            for r, k in sorted(pinmap.items(), key=lambda kv: (rows[self._pinpos(kv[0], kv[1])[1]], kv[0])):
                if self.P[r].get('fixed'): continue
                _, pr = self._pinpos(r, k)
                if rows.get(pr, 0) >= 2: continue
                cands = sorted([rw for rw in rows if rw != pr and abs(rw - pr) <= reach], key=lambda rw: (-rows[rw], abs(rw - pr)))
                for target in cands:
                    dy = target - pr; c, rr, rot, L = self.pos[r]
                    b = self._try_box(r, c, rr + dy, rot, L)
                    if not self._legal(r, b, L, rot=rot, at=(c, rr + dy)): continue
                    before = sum(self._hpwl(n) for n in self.partnets[r])
                    self._apply(r, c, rr + dy, rot, L)
                    after = sum(self._hpwl(n) for n in self.partnets[r])
                    if after - before > 8: self._apply(r, c, rr, rot, L); continue
                    rows[pr] -= 1; rows[target] = rows.get(target, 0) + 1; moved += 1; break
        return moved

    # ── 담금질 ──
    def anneal(self, iters=40000, T0=None, verbose=False):
        movable = [r for r in self.refs if not self.P[r].get('fixed') and r not in self.attached_caps]
        if not movable: return
        if T0 is None: T0 = max(1.0, 0.01 * self.total)
        Tend = 0.02; alpha = (Tend / T0) ** (1.0 / max(1, iters))
        T = T0; best = self.total; best_pos = dict(self.pos); acc = 0
        for it in range(iters):
            kind = self.rng.random()
            r = self.rng.choice(movable)
            old = self.pos[r]; affected = sorted(self.partnets[r])   # ★ 정렬 필수: set 순회 순서는 실행마다 다르다 (문자열 해시 무작위화)
            if kind < 0.55:      # 옮기기 (반경은 온도에 비례)
                R = max(2, int(8 + 30 * (T / T0)))
                nc = old[0] + self.rng.randint(-R, R); nr = old[1] + self.rng.randint(-R, R); nrot = old[2]; nl = old[3]
                spot = self._nearest_legal(r, nc, nr, nrot, nl)
                if spot is None: continue
                nc, nr = spot
                ob=self.box[r]; self._apply(r, nc, nr, nrot, nl); moves = [(r, old)]; mb=[(ob, self.box[r], is_big(self.P[r]['_pkg']))]
            elif kind < 0.75:    # 자리 바꾸기 (같은 종류끼리)
                o = self.rng.choice(movable)
                if o == r or self.P[o].get('fixed') or self.P[o]['_pkg'] != self.P[r]['_pkg']: continue
                oo = self.pos[o]
                b1 = self._try_box(r, oo[0], oo[1], oo[2], oo[3]); b2 = self._try_box(o, old[0], old[1], old[2], old[3])
                if not self._legal(r, b1, oo[3], ignore=(o,), rot=oo[2], at=(oo[0], oo[1])) or not self._legal(o, b2, old[3], ignore=(r,), rot=old[2], at=(old[0], old[1])): continue
                if boxes_clash(b1, b2, gap_needed(self.P[r], self.P[o], self.channel)) and oo[3] == old[3]: continue
                ob1=self.box[r]; ob2=self.box[o]; self._apply(r, *oo); self._apply(o, *old); moves = [(r, old), (o, oo)]; affected = sorted(set(affected) | self.partnets[o]); mb=[(ob1, self.box[r], is_big(self.P[r]['_pkg'])), (ob2, self.box[o], is_big(self.P[o]['_pkg']))]
            elif kind < 0.90:    # 회전
                nrot = 90 if old[2] == 0 else 0
                b = self._try_box(r, old[0], old[1], nrot, old[3])
                if not self._legal(r, b, old[3], rot=nrot, at=(old[0], old[1])): continue
                ob=self.box[r]; self._apply(r, old[0], old[1], nrot, old[3]); moves = [(r, old)]; mb=[(ob, self.box[r], is_big(self.P[r]['_pkg']))]
            else:                # 층 바꾸기
                if self.L < 2: continue
                nl = self.rng.randint(1, self.L)
                if nl == old[3]: continue
                b = self._try_box(r, old[0], old[1], old[2], nl)
                if not self._legal(r, b, nl, rot=old[2], at=(old[0], old[1])): continue
                ob=self.box[r]; self._apply(r, old[0], old[1], old[2], nl); moves = [(r, old)]; mb=[(ob, self.box[r], is_big(self.P[r]['_pkg']))]
            # 비용 차이
            newnet = {n: self._hpwl(n) for n in affected}
            dnet = sum(newnet[n] - self.netcost[n] for n in affected)
            g_old = self._global_cache; g_new = self._global()
            dc = self._cong_delta(affected, mb)
            delta = dnet + (g_new - g_old) + dc
            if delta <= 0 or self.rng.random() < math.exp(-delta / T):
                self.netcost.update(newnet); self.total += delta; self._global_cache = g_new; acc += 1
                if self.total < best: best = self.total; best_pos = dict(self.pos)
            else:
                for rr, o in moves: self._apply(rr, *o)
                self._cong_delta(affected, [(nb, ob, big) for ob, nb, big in mb])      # 밀도 격자·용량도 되돌린다
            T *= alpha
            if verbose and it % 5000 == 0: print(f"  it {it:6d} T {T:8.2f} cost {self.total:9.1f} best {best:9.1f} acc {acc}")
        for r, o in best_pos.items(): self._apply(r, *o)
        self.netcost = {n: self._hpwl(n) for n in self.netpins}; self._global_cache = self._global(); self.total = sum(self.netcost.values()) + self._global_cache + self._congestion()
        return best

    # ── 배치 뒤: near 스냅, 버스 행, 라이저 ──
    def place_attached(self):
        """붙은 캡을 예약된 캡 자리에 놓는다 (세로 2핀)"""
        n = 0
        for chip in sorted(self.attached):
            caps = self.attached[chip]
            cs = self.cap_slot(chip)
            if cs is None: continue
            for i, c in enumerate(caps):
                col, row = cs[0] + (i % 2), cs[1]
                self._apply(c, col, row, 0, self.P[chip]['layer']); n += 1
        return n
    def snap_near(self, radius=6):
        """near 힌트가 있는 부품(디커플링 캡)을 그 칩의 VCC 핀 옆(가까운 합법 자리)으로 옮긴다 — R17 이 5mm 안을 요구한다"""
        moved = 0
        for r in self.refs:
            near = self.P[r].get('near')
            if not near or near not in self.P or self.P[r].get('fixed') or r in self.attached_caps: continue
            vk = self.vcc_pin.get(near)
            if vk is None: continue
            vc, vr = self._pinpos(near, vk); L = self.P[near]['layer']
            best = None
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    for rot in (0, 90):
                        c, rr = vc + dx, vr + dy
                        b = self._try_box(r, c, rr, rot, L)
                        if not self._legal(r, b, L, rot=rot, at=(c, rr)): continue
                        dist = abs((b[0] + b[2]) / 2 - vc) + abs((b[1] + b[3]) / 2 - vr)
                        if best is None or dist < best[0]: best = (dist, c, rr, rot)
            if best and (best[1], best[2], best[3], L) != self.pos[r]:
                self._apply(r, best[1], best[2], best[3], L); moved += 1
        return moved

    def bus_rows(self, reach=2, reach_hot=1):
        """전원 핀 행을 모두 덮는 최소 버스 행 집합 (탐욕). 칩마다 허용 스텁이 다르다:
        동시 스위칭 출력이 많은 칩(sso ≥ 8: 574·244)은 reach_hot(=1행, 2.54mm) — R16 이 5mm 스텁에서도 LOW 여유를 넘겼다. 나머지는 reach(=2행)."""
        def reach_of(r):
            c = self.d.chips[self.P[r]['chip']]
            sso = c.attr.get('sso', 0); sso = len(sso) if isinstance(sso, (list, tuple, set)) else sso
            return reach_hot if sso >= 8 else reach
        # ★ 버스 줄은 칩·커넥터의 핀 행 위에 놓지 않는다. 핀 행에 버스가 지나면 그 행의 신호 핀이 갇힌다:
        #   좌우는 버스 구멍(납땜점)이라 못 지나고, 위아래 통로는 이웃 핀의 전원 스텁이 차지한다 → 80×50 빈 기판에서도 미배선이 남던 원인.
        #   (버스 줄은 모든 층에 같은 행이라 층을 가리지 않고 뺀다)
        pinrows = set()
        for r in self.refs:
            if not is_big(self.P[r]['_pkg']): continue
            b = self.box[r]                                   # 세로(rot 0/180) DIP·커넥터: 상자의 모든 행에 핀. 가로(rot 90/270): 위·아래 두 행만
            if self.P[r]['rot'] in (90, 270): pinrows.update({b[1], b[3]})
            else: pinrows.update(range(b[1], b[3] + 1))
        def nearest_free(row):
            for dd in range(0, self.H):
                for x in (row - dd, row + dd):
                    if 0 <= x <= self.H - 1 and x not in pinrows: return x
            return min(self.H - 1, max(0, row))
        def cover(items):   # items: [(row, reach)]
            need = sorted(set(items)); chosen = []
            while need:
                lo = min(r - rc for r, rc in need); hi = max(r + rc for r, rc in need)
                cand = [x for x in range(max(0, lo), min(self.H - 1, hi) + 1) if x not in pinrows]
                if not cand:   # 핀 행이 전부 기판 밖이면(좁은 기판) 그 핀 행을 기판 안으로 끌어와 쓴다
                    cand = [nearest_free(need[0][0])]
                best = max(cand, key=lambda x: sum(1 for r, rc in need if abs(r - x) <= rc))
                if not any(abs(r - best) <= rc for r, rc in need):   # 아무것도 못 덮으면 첫 핀을 강제로 덮어 무한루프를 막는다
                    need = need[1:]; chosen.append(nearest_free(need[0][0] if need else best)); continue
                chosen.append(best); need = [(r, rc) for r, rc in need if abs(r - best) > rc]
            return sorted(chosen)
        g = cover([(self._pinpos(r, k)[1], reach_of(r)) for r, k in self.gnd_pin.items()])
        v = cover([(self._pinpos(r, k)[1], reach_of(r)) for r, k in self.vcc_pin.items()])
        def apart(r):                                   # VCC 가 GND 와 같은 행이면 핀 행이 아닌 가장 가까운 다른 행으로
            if r not in g: return r
            for dd in range(1, self.H):
                for x in (r - dd, r + dd):
                    if 0 <= x <= self.H - 1 and x not in g and x not in pinrows: return x
            return r
        v = [apart(r) for r in v]
        return {'GND': sorted(set(g)), 'VCC': sorted(set(v))}

    def _bus_rows_old(self):
        def cluster(rows, tol=2):
            rows = sorted(rows); out = []
            for r in rows:
                if out and r - out[-1][-1] <= tol: out[-1].append(r)
                else: out.append([r])
            return [int(round(sum(g) / len(g))) for g in out]
        g = cluster([self._pinpos(r, k)[1] for r, k in self.gnd_pin.items()])
        v = cluster([self._pinpos(r, k)[1] for r, k in self.vcc_pin.items()])
        # GND 와 VCC 가 같은 행이면 VCC 를 한 칸 위로
        v = [r - 1 if r in g and r > 0 else r for r in v]
        v = sorted(set(min(self.H - 1, max(0, r)) for r in v)); g = sorted(set(min(self.H - 1, max(0, r)) for r in g))
        return {'GND': g, 'VCC': v}

    def risers(self, spare=2):
        """층을 넘는 네트 → 라이저(HDR20) 목록 [(ref, col, row, lo, hi, [nets])]"""
        if self.L < 2: return []
        d = self.d
        cross = {}
        for n, pl in self.netpins.items():
            lays = sorted({self.P[r]['layer'] for r, k in pl})
            for a, b in zip(lays, lays[1:]):
                for L in range(a, b): cross.setdefault((L, L + 1), []).append(n)
        for L in range(1, self.L): cross.setdefault((L, L + 1), [])
        # 전원은 층마다 있어야 한다
        for k in cross: cross[k] = ['VCC', 'GND'] + cross[k]
        occ = {}
        for r in self.refs:
            b = self.box[r]; g = 1 if is_big(self.P[r]['_pkg']) else 0
            for x in range(b[0] - g, b[2] + g + 1):
                for y in range(b[1] - g, b[3] + g + 1): occ.setdefault(self.P[r]['layer'], set()).add((x, y))
        def center(n):
            if n in d.supplies: return (self.W / 2, self.H / 2)
            ps = [self._pinpos(r, k) for r, k in self.netpins[n]]
            return (sum(p[0] for p in ps) / len(ps), sum(p[1] for p in ps) / len(ps))
        def free_col(lo, hi, near, need=20):
            nc, nr = near
            for col in sorted(range(1, self.W - 1), key=lambda c: abs(c - nc)):
                for row in sorted(range(1, self.H - need - 1), key=lambda r: abs(r - nr)):
                    cells = {(col + dx, row + k) for k in range(need) for dx in (-1, 0, 1)}
                    if all(not (cells & occ.get(Lz, set())) for Lz in range(lo, hi + 1)): return col, row
            return None
        def free_any(lo, hi, near, want=20):
            """필요한 핀 수에 맞는 자리. 라이저는 층마다 자리를 먹으니 큰 것부터 쓰지 않는다"""
            sizes = [8, 20] if want <= 8 else [20, 8]
            for sz in sizes:
                spot = free_col(lo, hi, near, sz)
                if spot: return spot, sz
            return (None, 0)
        out = []; si = 0; self.riser_fail = []
        for (lo, hi), nets in sorted(cross.items()):
            real = sorted([n for n in nets if n not in d.supplies], key=lambda n: (center(n)[0], str(n)))
            sup = [n for n in nets if n in d.supplies]
            chunks = []
            while real: chunks.append(real[:20 - len(sup) - spare if len(chunks) == 0 else 20 - spare]); real = real[len(chunks[-1]):]
            if not chunks: chunks = [[]]
            queue = [((sup if ci == 0 else []) + ch) for ci, ch in enumerate(chunks)]
            while queue:
                take = queue.pop(0)
                cs = [center(n) for n in take if n not in d.supplies] or [(self.W / 2, self.H / 2)]
                near = (sum(c[0] for c in cs) / len(cs), sum(c[1] for c in cs) / len(cs))
                spot, size = free_any(lo, hi, near, len(take))
                if spot is None:
                    print(f"  ! {lo}-{hi}층 라이저 자리 없음 (네트 {len(take)}개) — 그 네트들을 한 층으로 모은다")
                    self.riser_fail.extend(n for n in take if n not in d.supplies); continue
                if len(take) > size:                       # 8핀 자리면 나머지는 다음 라이저로
                    queue.insert(0, take[size:]); take = take[:size]
                col, row = spot; si += 1
                for Lz in range(lo, hi + 1):
                    for k in range(size):
                        for dx in (-1, 0, 1): occ.setdefault(Lz, set()).add((col + dx, row + k))
                out.append((f'J{si}', col, row, lo, hi, take, size))
        return out

# ───────────────────────── KYLE 쓰기 ─────────────────────────
def write_sol(src_text, pl, risers, buses, W, H, layers, out_path):
    lines = src_text.split('\n'); out = []
    old_stack_refs = set(re.findall(r'^\s*stack\s+(\S+)', src_text, re.M))
    part_line = {}
    for r in pl.refs:
        p = pl.P[r]; c, rr, rot, L = p['col'], p['row'], p['rot'], p['layer']
        part_line[r] = (f"part    {r:<9}{p['chip']:<12}at {c:>2} {rr:>2}" + (f"  rot {rot}" if rot else '') + (f"  layer {L}" if L != 1 else '')
                        + (f"  near {p['near']}" if p.get('near') else '') + ('  fixed' if p.get('fixed') else ''))
    seen = set(); last_part_idx = None
    for ln in lines:
        t = ln.split()
        if t and t[0] == 'board' and len(t) >= 4 and t[1] == 'perf':
            out.append(f"board   perf {W} {H}" + (f" layers {layers}" if layers > 1 else '')); continue
        if t and t[0] == 'part' and len(t) > 1 and t[1] in part_line:
            out.append(part_line[t[1]]); seen.add(t[1]); last_part_idx = len(out) - 1; continue
        if t and t[0] in ('stack', 'bus'): continue
        if t and t[0] == 'pin' and any(t[1].startswith(s + '[') for s in old_stack_refs): continue
        if t and t[0] == 'route' and 'auto' in t[:3]: continue          # 자동배선은 새 배치에서 다시 (손경로는 유지)
        out.append(ln)
    ins = []
    # 설정 문장이 원본에 없으면 넣어 준다 (KYLE 한 장만으로 배선이 재현되어야 한다)
    have = {ln.split()[0] for ln in out if ln.split()}
    for key, val in (('wire', f"{pl.d.settings['wire']}mm"), ('stitch', str(pl.d.settings['stitch'])), ('share', str(pl.d.settings['share']))):
        if key not in have: ins.append(f"{key:<8}{val}")
    for r in pl.refs:
        if r not in seen: ins.append(part_line[r])
    for sref, col, row, lo, hi, nets, *sz in risers:
        ins.append(f"stack   {sref} {'HDR8' if (sz and sz[0] == 8) else 'HDR20'} at {col} {row} layers {lo}-{hi}")
    for net, rows in buses.items():
        if rows: ins.append(f"bus     {net} rows " + ','.join(map(str, rows)))
    for sref, col, row, lo, hi, nets, *sz in risers:
        for k, n in enumerate(nets, 1): ins.append(f"pin     {sref}[{k}:{n}]")
    if last_part_idx is None: out.extend([''] + ins)
    else: out[last_part_idx + 1:last_part_idx + 1] = [''] + ins
    txt = '\n'.join(out); txt = re.sub(r'\n{3,}', '\n\n', txt)
    io.open(out_path, 'w', encoding='utf-8').write(txt if txt.endswith('\n') else txt + '\n')

def run(src, out, W=None, H=None, layers=None, seed=0, iters=40000, channel=2, verbose=False, layer_init='graph'):
    d = sol.parse_design(src)
    if d.errors: raise SystemExit('입력 KYLE 오류: ' + '; '.join(d.errors[:3]))
    W = W or d.board[0]; H = H or d.board[1]; layers = layers or d.layers
    pl = Placer(d, W, H, layers, channel=channel, seed=seed)
    c0 = pl.total
    if layer_init == 'graph': pl.init_layers_graph(verbose=verbose)
    else: pl.init_layers(layer_init)
    n_leg = pl.legalize()
    if verbose: print(f"  시작 합법화: {n_leg}개 이동")
    best = pl.anneal(iters=iters, verbose=verbose)
    pl.align_power(); pl.place_attached(); pl.snap_near()
    buses = pl.bus_rows(); ris = pl.risers()
    for _ in range(3):
        if not getattr(pl, 'riser_fail', []): break
        # 라이저를 못 얻은 네트: 그 네트의 부품을 다수파 층으로 모아 층을 안 넘게 만든다 (R26 은 '물리적으로 연결 불가' 라 남기면 안 된다)
        stuck = sorted(set(pl.riser_fail)); moved = 0
        for n in stuck:
            pins = pl.netpins.get(n) or []
            if not pins: continue
            cnt = {}
            for r, k in pins: cnt[pl.P[r]['layer']] = cnt.get(pl.P[r]['layer'], 0) + 1
            keep = max(cnt, key=lambda L: cnt[L])
            for r, k in pins:
                if pl.P[r]['layer'] == keep or pl.P[r].get('fixed') or r in pl.attached_caps: continue
                c, rr, rot, _ = pl.pos[r]
                spot = pl._nearest_legal(r, c, rr, rot, keep, rad=max(pl.W, pl.H))
                if spot is None: continue
                pl._apply(r, spot[0], spot[1], rot, keep); moved += 1
                for cp in pl.attached.get(r, []): pl._apply(cp, *pl.pos[cp][:3], keep)
        if not moved: break
        pl.place_attached(); buses = pl.bus_rows(); ris = pl.risers()
    # ★ 최종 보증: 라이저가 안 나르는데 층을 걸친 네트가 남으면 그 네트의 부품을 다수파 층으로 모은다.
    #   (부품 하나를 옮기면 그 부품의 '다른' 네트가 새로 층을 걸칠 수 있다 — riser_fail 만 보고 끝내면 그걸 놓친다.
    #    R26 은 '물리적으로 연결 불가' 라 하나라도 남기면 안 되는 오류다.)
    prev_bad = None
    for _ in range(60):          # 진전이 있는 한 계속 (고정 횟수로 끊으면 '거의 다 됐는데' 남는다)
        carried = {n for (_, _, _, _, _, nets, *_) in ris for n in nets}
        bad = sorted(n for n in pl.netpins if len({pl.P[r]['layer'] for r, k in pl.netpins[n]}) > 1 and n not in carried)
        if not bad: break
        moved = 0
        for n in bad:
            pins = pl.netpins[n]; cnt = {}
            for r, k in pins: cnt[pl.P[r]['layer']] = cnt.get(pl.P[r]['layer'], 0) + 1
            keep = max(cnt, key=lambda L: cnt[L])
            for r, k in pins:
                if pl.P[r]['layer'] == keep or pl.P[r].get('fixed') or r in pl.attached_caps: continue
                c, rr, rot, _ = pl.pos[r]
                spot = pl._nearest_legal(r, c, rr, rot, keep, rad=max(pl.W, pl.H))
                if spot is None: continue
                pl._apply(r, spot[0], spot[1], rot, keep); moved += 1
        if not moved:
            # 자리가 없어 못 옮겼다 → 목표 층에서 '작은 부품' 을 잠시 치우고 자리를 낸다 (치운 부품은 다시 아무 데나 합법 자리로)
            for n in bad:
                pins = pl.netpins[n]; cnt = {}
                for r, k in pins: cnt[pl.P[r]['layer']] = cnt.get(pl.P[r]['layer'], 0) + 1
                keep = max(cnt, key=lambda L: cnt[L])
                for r, k in pins:
                    if pl.P[r]['layer'] == keep or pl.P[r].get('fixed') or r in pl.attached_caps: continue
                    victims = [o for o in pl.refs if pl.P[o]['layer'] == keep and not is_big(pl.P[o]['_pkg'])
                               and not pl.P[o].get('fixed') and o not in pl.attached_caps]
                    pl.rng.shuffle(victims)
                    for v in victims[:40]:
                        vpos = pl.pos[v]; pl._occ_remove(v)
                        c, rr, rot, _ = pl.pos[r]
                        spot = pl._nearest_legal(r, c, rr, rot, keep, rad=max(pl.W, pl.H))
                        if spot is None: pl._occ_add(v); continue
                        pl._apply(r, spot[0], spot[1], rot, keep); moved += 1
                        back = pl._nearest_legal(v, vpos[0], vpos[1], vpos[2], vpos[3], rad=max(pl.W, pl.H))
                        if back is None:
                            for L3 in range(1, pl.L + 1):
                                back = pl._nearest_legal(v, pl.W // 2, pl.H // 2, vpos[2], L3, rad=max(pl.W, pl.H))
                                if back: pl._apply(v, back[0], back[1], vpos[2], L3); break
                            if back is None: pl._apply(v, *vpos)      # 정 안 되면 원래 자리로 (다음 바퀴가 다시 시도)
                        else: pl._apply(v, back[0], back[1], vpos[2], vpos[3])
                        break
        if not moved or (prev_bad is not None and len(bad) >= prev_bad and moved == 0):
            # 마지막 수단: 합법성을 잠시 접고 층만 맞춘 뒤, 전체를 다시 합법화한다.
            # (층 걸침은 '물리적으로 연결 불가' 라 반드시 없애야 하고, 겹침은 legalize 가 풀 수 있다)
            forced = 0
            for n in bad:
                pins = pl.netpins[n]; cnt = {}
                for r, k in pins: cnt[pl.P[r]['layer']] = cnt.get(pl.P[r]['layer'], 0) + 1
                keep = max(cnt, key=lambda L: cnt[L])
                for r, k in pins:
                    if pl.P[r]['layer'] == keep or pl.P[r].get('fixed') or r in pl.attached_caps: continue
                    c, rr, rot, _ = pl.pos[r]; pl._apply(r, c, rr, rot, keep); forced += 1
            if forced:
                pl.legalize(keep_layer=True); pl.place_attached(); buses = pl.bus_rows(); ris = pl.risers()
                carried2 = {x for (_, _, _, _, _, nets, *_) in ris for x in nets}
                left = sorted(x for x in pl.netpins if len({pl.P[r]['layer'] for r, k in pl.netpins[x]}) > 1 and x not in carried2)
                if not left: prev_bad = None; continue
                bad = left
            print(f"  ! 층을 걸친 네트 {len(bad)}개를 한 층으로 못 모음 — 기판을 키우거나 --channel 을 줄일 것")
            pl.cross_fail = list(bad); break
        prev_bad = len(bad)
        pl.place_attached(); buses = pl.bus_rows(); ris = pl.risers()
    write_sol(io.open(src, encoding='utf-8').read(), pl, ris, buses, W, H, layers, out)
    return dict(cost0=c0, cost=pl.total, risers=len(ris), buses=buses,
                fail_place=list(getattr(pl, 'fail_place', [])), cross_fail=list(getattr(pl, 'cross_fail', [])))

def refine(src, out, evals=60, seed=0, verbose=True):
    """실제 배선(route_eval)을 심판으로 한 국소 탐색: 미배선 네트에 붙은 부품을 조금씩 옮겨 보고, 미배선이 줄면 채택.
    담금질 비용은 근사라 여기서 진짜 목적함수로 다듬는다. 한 번 채점에 수 초 → evals 로 예산을 정한다."""
    import route_eval, shutil, tempfile
    d = sol.parse_design(src)
    W, H, layers = d.board[0], d.board[1], d.layers
    pl = Placer(d, W, H, layers, seed=seed)
    rng = random.Random(seed)
    src_text = io.open(src, encoding='utf-8').read()
    def emit(path):
        write_sol(src_text, pl, pl.risers(), pl.bus_rows(), W, H, layers, path)
    work = tempfile.mkdtemp(prefix='sol_refine_'); cand = os.path.join(work, 'c.txt')
    for f in ('chips.txt', 'physics.txt'): os.symlink(os.path.join(HERE, f), os.path.join(work, f))
    emit(cand); cur = route_eval.evaluate(cand)
    key = lambda r: (r.get('crit', 9) + r.get('err', 9), r['fail'] if r.get('fail') is not None else 10**6, r.get('bends', 0))
    best_key = key(cur); shutil.copy(cand, out)
    if verbose: print(f"  시작: {route_eval.fmt(cur)}")
    for e in range(evals):
        failed = cur.get('failed') or []
        if not failed: break
        # 실패 네트에 붙은 부품 중 하나 (칩 우선)
        parts = [r for n in failed for r, k in pl.netpins.get(n, []) if not pl.P[r].get('fixed') and r not in pl.attached_caps]
        if not parts: break
        r = rng.choice(parts); old = pl.pos[r]; kind = rng.random(); moves = []
        if kind < 0.6:
            dx, dy = rng.choice([(1,0),(-1,0),(0,1),(0,-1),(2,0),(-2,0),(0,2),(0,-2),(3,0),(-3,0),(0,3),(0,-3)])
            spot = pl._nearest_legal(r, old[0] + dx, old[1] + dy, old[2], old[3], rad=2)
            if spot is None or spot == (old[0], old[1]): continue
            pl._apply(r, spot[0], spot[1], old[2], old[3]); moves = [(r, old)]
        elif kind < 0.8:
            nrot = 90 if old[2] == 0 else 0
            if not pl._legal(r, pl._try_box(r, old[0], old[1], nrot, old[3]), old[3], rot=nrot, at=(old[0], old[1])): continue
            pl._apply(r, old[0], old[1], nrot, old[3]); moves = [(r, old)]
        else:
            same = [o for o in pl.refs if o != r and pl.P[o]['_pkg'] == pl.P[r]['_pkg'] and not pl.P[o].get('fixed') and o not in pl.attached_caps]
            if not same: continue
            o = rng.choice(same); oo = pl.pos[o]
            if not pl._legal(r, pl._try_box(r, oo[0], oo[1], oo[2], oo[3]), oo[3], ignore=(o,), rot=oo[2], at=(oo[0], oo[1])): continue
            if not pl._legal(o, pl._try_box(o, old[0], old[1], old[2], old[3]), old[3], ignore=(r,), rot=old[2], at=(old[0], old[1])): continue
            pl._apply(r, *oo); pl._apply(o, *old); moves = [(r, old), (o, oo)]
        pl.place_attached()
        emit(cand); res = route_eval.evaluate(cand); k = key(res)
        if k < best_key:
            best_key = k; cur = res; shutil.copy(cand, out)
            if verbose: print(f"  {e+1:3d} ✔ {route_eval.fmt(res)}")
        else:
            for rr, o in moves: pl._apply(rr, *o)
            pl.place_attached()
            if verbose and (e + 1) % 10 == 0: print(f"  {e+1:3d}   (미배선 {res.get('fail')} — 되돌림)")
    shutil.rmtree(work, ignore_errors=True)
    return cur

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src'); ap.add_argument('--out', required=True)
    ap.add_argument('--board', nargs=2, type=int); ap.add_argument('--layers', type=int); ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--iters', type=int, default=40000); ap.add_argument('--channel', type=int, default=2)
    ap.add_argument('--search', type=int, default=1, help='씨앗 N개 돌려 최고를 남긴다'); ap.add_argument('--route', action='store_true', help='실제 배선으로 채점 (느림)')
    ap.add_argument('--layer-init', default='graph', choices=['graph','prefix','none'], help='층>1 시작 배정: graph=KYLE 연결을 보고 분할(기본), prefix=이름 앞 토큰별, none=그대로')
    ap.add_argument('--refine', type=int, default=0, help='실제 배선을 심판으로 N번 국소 탐색 (입력 KYLE 의 배치에서 시작)')
    ap.add_argument('-v', action='store_true')
    a = ap.parse_args()
    if a.refine:
        r = refine(a.src, a.out, evals=a.refine, seed=a.seed); import route_eval; print("→", route_eval.fmt(r)); return
    W, H = (a.board or (None, None))
    best = None
    for s in range(a.seed, a.seed + a.search):
        cand = a.out if a.search == 1 else a.out + f'.s{s}.txt'
        info = run(a.src, cand, W, H, a.layers, seed=s, iters=a.iters, channel=a.channel, verbose=a.v, layer_init=a.layer_init)
        line = f"seed {s}: 비용 {info['cost0']:.0f} → {info['cost']:.0f}  라이저 {info['risers']}  버스 GND {info['buses']['GND']} VCC {info['buses']['VCC']}"
        if info.get('fail_place'): line += f"  ★ 자리 못 찾은 부품 {len(info['fail_place'])}개"
        if info.get('cross_fail'): line += f"  ★ 층 못 모은 네트 {len(info['cross_fail'])}개"
        if a.route:
            import route_eval
            r = route_eval.evaluate(cand); line += "  | " + route_eval.fmt(r)
            key = (r.get('crit', 9) + r.get('err', 9), r['fail'] if r.get('fail') is not None else 10**6, r.get('r28_viol', 0), r.get('bends', 0))
        else:
            key = (0, info['cost'])
        print(line, flush=True)
        if best is None or key < best[0]: best = (key, cand)
    if a.search > 1:
        import shutil; shutil.copy(best[1], a.out); print(f"→ 최고: {best[1]} → {a.out}")

if __name__ == '__main__':
    main()
