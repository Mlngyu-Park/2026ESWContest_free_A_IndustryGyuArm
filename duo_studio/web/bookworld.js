// 책꽂이 공정 (11-02) — 로봇과 상관없는 '같은 공정'. 로봇은 몇 가지 동작만 채운다 (ur15b.js · spotb.js · g1b.js).
//   공정 = 블록 하나("1-1 의 책 전부 → 1-4")를 책 한 권씩의 동작으로 풀어낸다. 책 자리는 물리 엔진이 실제로 놓은 자리를 그때그때 읽는다.
//   꺼내기: 갈고리 (손끝을 책 위로 넘겨 책 뒤로 내리고 앞으로 당김 → 책이 6cm 튀어나옴) → 튀어나온 앞부분을 집어 꺼냄
//            (옆 책과 붙어 있어 손가락이 안 들어가도 된다)
//   꽂기:   앞부분 6cm 만 남기고 넣고 → 놓고 → 손을 닫아 앞면을 밀어 넣음 (옆 책이 밀리는 건 물리가 계산)
//   바닥 책: 위에서 집어 정렬대에 걸쳐 놓고 → 튀어나온 쪽을 다시 잡아 세워서 → 꽂기
//   로봇이 채울 것: base() 서는 자리 · arm IK(손 끝 '특징점' pinch/tip) · 집게 열기/닫기 · 부딪힘 설정 · 물리에 넣을 손 조각
import { initPhysics, BookPhysics } from "./phys.js";
import { SHELVES, shelfT, cellBox, parseCell, worldBoxes, standBox, STAND, BOOK, FRONT_IN, WALL, initialBooks, CELLS, shelfPanels, SHELF_D, SHELF_W, SHELF_H } from "./shelf.js";
import { mul, inv, rpyT, apply, axisT, clamp } from "./base.js";
import { scan, itemPoints, boxDepth } from "./collide.js";

export const BOOK_BLOCKS = {
  move: { name: "책 옮기기", args: ["cell", "cell", "count"], hint: "칸의 책을 다른 칸으로 옮긴다 (전부 / 한 권)" },
  floor: { name: "바닥 책 줍기", args: ["cell"], hint: "바닥에 놓인 책을 모두 주워 칸에 꽂는다" },
  wait: { name: "기다리기", args: ["sec"], hint: "잠깐 멈춘다" },
};
const BOOK_ANY = { kind: "book" };
const OUT = 0.04;                                    // 꺼낼 때·꽂을 때 책 앞이 책꽂이 밖으로 나오는 길이                  // 집게를 '책 한 권 + 여유' 만큼 벌릴 때
export const COUNTS = [["all", "전부"], ["one", "한 권"]];
export const SECS = [["1", "1초"], ["2", "2초"], ["5", "5초"]];
export const BOOK_EXAMPLE = [
  { type: "move", args: ["1-1", "1-4", "all"] },
  { type: "floor", args: ["1-6"] },
  { type: "move", args: ["2-5", "1-9", "one"] },
];

const smooth = (u) => u * u * (3 - 2 * u);
export const lerp = (a, b, u) => a.map((v, i) => v + (b[i] - v) * u);
export const col = (M, k) => [M[k], M[4 + k], M[8 + k]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const add = (a, b, s = 1) => a.map((v, i) => v + s * b[i]);
// 축 셋(x 는 y×z)과 원점으로 행렬
export const frame = (z, y, p) => { const x = cross(y, z); return [x[0], y[0], z[0], p[0], x[1], y[1], z[1], p[1], x[2], y[2], z[2], p[2], 0, 0, 0, 1]; };
export function slerpT(Ta, Tb, u) {
  const r = (i, j) => Ta[i] * Tb[j] + Ta[4 + i] * Tb[4 + j] + Ta[8 + i] * Tb[8 + j];   // (Raᵀ Rb)ij
  const th = Math.acos(clamp((r(0, 0) + r(1, 1) + r(2, 2) - 1) / 2, -1, 1));
  let ax = [r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1)];
  if (Math.hypot(...ax) < 1e-9) ax = [0, 0, 1];
  const M = mul(Ta, axisT(ax, th * u));
  M[3] = Ta[3] + (Tb[3] - Ta[3]) * u; M[7] = Ta[7] + (Tb[7] - Ta[7]) * u; M[11] = Ta[11] + (Tb[11] - Ta[11]) * u;
  return M;
}
const FLIP = rpyT(0, 0, Math.PI);                    // 손 끝 축(z)으로 180°
const KO = { "바닥": "바닥", "도킹": "책꽂이", "물건": "책", "잡는 물건": "잡을 책", "든 물건-몸": "몸", "몸끼리": "몸", "든 물건-바닥": "바닥", "든 물건-도킹": "책꽂이", "물건끼리": "다른 책", "든 물건-몸속": "몸" };

export class BookWorld {
  // 로봇 쪽이 constructor 에서 this.robot · this.sh · this.cfg(부딪힘) · this.handLinks(물리 손 조각) 를 채운 뒤 setup(R) 를 부른다
  setup(R) { this.R = R; this.refreshStatics(); this.reset(); }
  refreshStatics() { this.statics = [...worldBoxes(), standBox(), ...(this.fixtures?.() ?? [])]; }   // 책꽂이를 옮기면 다시
  reset() {
    this.resetRobot(); this.refreshStatics();
    this.books = initialBooks();
    this.phys = new BookPhysics(this.R, [standBox()], this.books, { shelves: SHELVES.map(s => ({ id: s.id, T: shelfT(s), panels: shelfPanels() })) });
    const W = this.fk();
    for (const l of this.handLinks) { const s = this.sh.L[l]; this.phys.addHand(l, s.p); this.phys.placeHand(l, W[l]); }
    this.held = null; this.exec = null; this.t = 0;
    for (let i = 0; i < 120; i++) this.phys.step(1 / 120);   // 책이 자리에 앉게
  }
  // ── 장면 읽기 (물리가 놓은 실제 자리) ──
  where(bk) { return [bk.T[3], bk.T[7], bk.T[11]]; }
  cellOf(bk) {
    for (const s of SHELVES) {
      const L = mul(inv(shelfT(s)), bk.T);
      for (let k = 1; k <= 9; k++) { const b = cellBox(k); if (L[3] > b.lo[0] - 0.08 && L[3] < b.hi[0] && L[7] > b.lo[1] && L[7] < b.hi[1] && L[11] > b.lo[2] && L[11] < b.hi[2]) return `${s.id}-${k}`; }
    }
    return null;
  }
  booksIn(cell) {                                   // 칸 안 책들, 오른쪽(열린 쪽)부터
    const c = parseCell(cell), Si = inv(shelfT(c.shelf));
    return this.books.filter(b => b !== this.held && this.cellOf(b) === cell).sort((a, b) => mul(Si, a.T)[7] - mul(Si, b.T)[7]);
  }
  floorBooks() { return this.books.filter(b => b !== this.held && !this.cellOf(b) && b.T[11] < 0.06); }
  // 정렬대 위에 걸쳐 둔 책 (바닥 책 줍기를 하다가 멈춘 것)
  standBooks() { return this.books.filter(b => b !== this.held && !this.cellOf(b) && Math.abs(b.T[3] - STAND.x) < STAND.w / 2 + 0.1 && Math.abs(b.T[7] - STAND.y) < STAND.w / 2 + 0.1 && b.T[11] > STAND.h && b.T[11] < STAND.h + 0.1); }
  // 책 한 권이 지금 어디: 칸 이름 · "floor"(바닥) · "stand"(정렬대 위) · "hand"(로봇 손) · "?"(그 밖 — 다른 책 위에 걸침 등)
  locOf(bk) {
    if (bk === this.held) return "hand";
    const c = this.cellOf(bk); if (c) return c;
    if (this.standBooks().includes(bk)) return "stand";
    return bk.T[11] < 0.06 ? "floor" : "?";
  }
  // 책꽂이 안(앞으로 8cm 튀어나온 것까지)에 있는 책들 — 책꽂이를 옮기면 같이 옮긴다
  booksInShelf(id) {
    const s = SHELVES.find(x => x.id === id), Si = inv(shelfT(s));
    return this.books.filter(b => { if (b === this.held) return false; const L = mul(Si, b.T); return L[3] > -0.09 && L[3] < SHELF_D && Math.abs(L[7]) < SHELF_W / 2 && L[11] > 0 && L[11] < SHELF_H; });
  }
  // ── 사용자가 옮기기 (3D 에서 끌어 놓기) — 물리·부딪힘 상자·책을 같이 ──
  moveShelf(id, x, y, yaw, books = this.booksInShelf(id)) {
    const s = SHELVES.find(v => v.id === id), D = mul(rpyT(0, 0, yaw, [x, y, 0]), inv(shelfT(s)));
    Object.assign(s, { x, y, yaw });
    this.phys.setShelf(id, shelfT(s)); this.refreshStatics();
    for (const b of books) this.phys.placeBook(b, mul(D, b.T));
  }
  placeBook(bk, T) { this.phys.placeBook(bk, T); }
  // 손이 책꽂이 앞 25cm 안 또는 정렬대 가까이에 있나 (다시 계산할 때 손부터 곧게 뺄지)
  handNear() {
    const T = this.feat("pinch"), sb = standBox();
    if (T[3] > sb.lo[0] - 0.3 && T[3] < sb.hi[0] + 0.3 && T[7] > sb.lo[1] - 0.3 && T[7] < sb.hi[1] + 0.3 && T[11] < sb.hi[2] + 0.25) return true;
    return SHELVES.some(sh => { const L = mul(inv(shelfT(sh)), [1, 0, 0, T[3], 0, 1, 0, T[7], 0, 0, 1, T[11], 0, 0, 0, 1]); return L[3] > -0.25 && L[3] < SHELF_D && Math.abs(L[7]) < SHELF_W / 2 + 0.1; });
  }
  // 꽂을 자리: 칸 안 책들 옆 (왼쪽 벽부터 채움) — 책 가운데의 책꽂이 좌표 y. 자리가 없으면 null
  //   avoid: 나중에 이 칸에서 꺼내야 할 책들 — 그 옆에 바짝 붙이면 손가락이 못 들어간다 (11-02 사용자가 옮긴 책을 되돌려 꽂을 때)
  //          → 그 책들과는 손가락 틈(pinchClear) 넘게 띄울 수 있는 자리 중 왼쪽 것. 없으면 그냥 붙인다
  slotY(cell, avoid = null) {
    const c = parseCell(cell), b = cellBox(c.k), Si = inv(shelfT(c.shelf)), inn = this.booksIn(cell);
    const half = (bk) => { const L = mul(Si, bk.T); return [L[7], Math.abs(L[6]) * BOOK.h / 2 + Math.abs(L[5]) * BOOK.t / 2]; };
    const ys = inn.map(bk => { const [y, h] = half(bk); return y - h; });   // 각 책의 오른쪽 끝
    const right = ys.length ? Math.min(...ys) - 0.004 : b.hi[1] - WALL;
    const y = right - BOOK.t / 2, packed = y - BOOK.t / 2 < b.lo[1] + WALL ? null : y;
    const keep = inn.filter(bk => avoid?.has(bk)); if (!keep.length) return packed;
    const need = (this.pinchClear ?? 0.02) + 0.006;
    for (let v = b.hi[1] - WALL - BOOK.t / 2; v - BOOK.t / 2 >= b.lo[1] + WALL; v -= 0.002) {
      const ok = inn.every(bk => { const [yo, h] = half(bk), g = Math.abs(yo - v) - h - BOOK.t / 2; return g >= (avoid.has(bk) ? need : 0.004); });
      if (ok) return v;
    }
    return packed;
  }
  // ── 부딪힘 ──
  //   allow: 닿아도 되는 책들(Set) · books=false 면 책은 안 본다 (물리로 미는 동작)
  //   allow 의 책은 손가락만 3mm 까지 닿아도 된다 (팔·손목이 치는 건 안 됨)
  //   pad: 책을 이만큼 부풀려서 본다 (자유롭게 옮길 땐 6mm 떨어져 지나가게 — 물리 엔진은 1mm 만 스쳐도 책을 친다)
  clash(opt = {}) {
    const p = opt.pad ?? 0;
    const items = opt.books === false ? [] : p ? this.books.map(b => b === this.held ? b : { ...b, d: b.d + 2 * p, t: b.t + 2 * p, h: b.h + 2 * p }) : this.books;
    const grip = opt.allow && !p ? [...opt.allow][0] ?? null : null;
    const c = scan(this.sh, this.fk(), items, this.held, { ...this.cfg, boxes: this.statics, grip });
    return c.length ? c.sort((a, b) => b.depth - a.depth)[0] : null;
  }
  clashText(c) { const w = c.who.split("|"); return `${KO[c.kind] ?? c.kind}${w[1] && !/_link|^l\d|grip|arm_|body|_leg/.test(w[1]) ? `(${w[1]})` : ""}에 부딪힘`; }
  // ── 한 프레임 ──
  step(dt) {
    this.t += dt;
    if (this.exec) this.exec.update(dt);
    this.stepBase?.(dt);
    const W = this.fk();
    for (const l of this.handLinks) this.phys.setHand(l, W[l]);
    if (this.held) this.phys.moveHeld(this.held, mul(this.feat("pinch", W), this.held.rel));
    this.phys.step(dt / 2); this.phys.step(dt / 2);
  }
  run(program, hooks = {}) { const G = planGoals(this, program); this.exec = new BookExec(this, program, hooks, G); }
  stop() { if (this.exec) { this.exec.stopped = true; this.exec = null; } this.stopBase?.(); }

  // ── 팔 계산 (로봇 공통) ── 로봇이 주는 것: feat(이름, W) · chainFor(이름) · baseT() · getQ/setQ · seeds()
  // 풀린 답들 (지금 자세에서 덜 움직이는 순). 같은 답은 하나만
  solveAll(feature, T, seeds = this.seeds(), max = 6) {
    const Bi = inv(this.baseT()), L = mul(Bi, T), ch = this.chainFor(feature), q0 = this.getQ(), out = [];
    for (const s of seeds) {
      this.setQ(s);
      const r = ch.solve([L[3], L[7], L[11]], [[0, col(L, 0)], [2, col(L, 2)]], 200);
      if (r.perr > 0.002 || r.oerr > 0.01) continue;
      const q = this.unwrap?.(this.getQ(), q0) ?? this.getQ(), cost = q.reduce((a, v, i) => a + Math.abs(v - q0[i]), 0);
      if (out.some(o => o.q.every((v, i) => Math.abs(v - q[i]) < 0.05))) continue;
      out.push({ cost, q }); if (out.length >= max) break;
    }
    this.setQ(q0);
    // 한 바퀴 넘게 도는 관절이 있으면 반대 방향으로 도는 같은 자세도 맨 뒤에 (덜 도는 쪽이 책꽂이를 지나갈 때 — 11-02 UR15)
    const main = out.sort((a, b) => a.cost - b.cost).map(o => o.q);
    const alt = main.flatMap(q => this.altWraps?.(q) ?? []).filter(v => !main.some(q => q.every((x, i) => Math.abs(x - v[i]) < 0.05)));
    return [...main, ...alt];
  }
  solve(feature, T, seeds) { return this.solveAll(feature, T, seeds, 1)[0] ?? null; }
  lineQ(feature, qa, Ta, Tb, n) {                   // 특징점을 Ta→Tb 로 공간에서 곧게
    const Bi = inv(this.baseT()), ch = this.chainFor(feature), q0 = this.getQ(), out = [qa]; let ok = true;
    this.setQ(qa);
    for (let k = 1; k <= n; k++) {
      const L = mul(Bi, slerpT(Ta, Tb, k / n));
      const r = ch.solve([L[3], L[7], L[11]], [[0, col(L, 0)], [2, col(L, 2)]], 60);
      if (r.perr > 0.003 || r.oerr > 0.02) ok = false;
      out.push(this.getQ());
    }
    this.setQ(q0);
    return ok ? out : null;
  }
  //   사이를 관절 0.012rad 마다 놓아 본다 (듬성듬성 보면 얇은 책을 뚫고 지나가도 모른다 — 11-02 정렬대 위 책을 쳐서 떨어뜨림)
  //   이미 닿아 있던 것(움직이기 전 자세)은 더 깊어지지만 않으면 된다 (물리가 1~2mm 눌러 놓은 상태에서 빼는 동작)
  pathClash(qs, opt) {
    const q0 = this.getQ(); let hit = null;
    const put = (q) => { this.setQ(q); if (this.held) this.held.T = mul(this.feat("pinch"), this.held.rel); };
    const base = new Map(); put(q0);
    { const items = opt?.books === false ? [] : this.books, c = scan(this.sh, this.fk(), items, this.held, { ...this.cfg, boxes: this.statics, grip: null }); for (const x of c) base.set(x.kind + x.who, x.depth);
      // 2mm 문턱 아래로 닿아 있던 것도 시작 상태로 친다 (든 책이 벽에 1.9mm 눌려 있다가 2.1mm 가 되는 정도는 같은 상태)
      if (this.held) for (const b of this.statics) { const C = itemPoints(this.held, this.held.T); let d = 0; for (const p of C) d = Math.max(d, boxDepth(b, ...p)); if (d > 0) base.set("든 물건-도킹" + `${this.held.name}|${b.name}`, Math.max(base.get("든 물건-도킹" + `${this.held.name}|${b.name}`) ?? 0, d)); } }
    const clash0 = this.clash.bind(this);
    const check = (o) => { const p = o.pad ?? 0, items = o.books === false ? [] : p ? this.books.map(b => b === this.held ? b : { ...b, d: b.d + 2 * p, t: b.t + 2 * p, h: b.h + 2 * p }) : this.books;
      const grip = o.allow && !p ? [...o.allow][0] ?? null : null;
      const c = scan(this.sh, this.fk(), items, this.held, { ...this.cfg, boxes: this.statics, grip }).filter(x => !base.has(x.kind + x.who) || x.depth > Math.max(base.get(x.kind + x.who), 0.002) + 0.001);
      return c.length ? c.sort((a, b) => b.depth - a.depth)[0] : null; };
    void clash0;
    for (let i = 0; i < qs.length && !hit; i++) {
      const sub = i ? Math.max(1, Math.ceil(Math.max(...qs[i].map((v, j) => Math.abs(v - qs[i - 1][j]))) / 0.012)) : 0;
      for (let k = i ? 1 : 0; k <= sub && !hit; k++) { put(i ? lerp(qs[i - 1], qs[i], k / sub) : qs[0]); hit = check(opt ?? {}); }
    }
    put(q0);
    return hit;
  }
}

// ── 동작 (로봇 공통) ─────────────────────────────────────────
//   계획은 동작이 시작하는 순간에 (그때의 책 자리로) 한다. 실패하면 이유를 말하고 멈춘다.
const jointPath = (a, b, n = 10) => [...Array(n + 1)].map((_, k) => lerp(a, b, k / n));
function pathDur(w, qs, speed) {                    // 관절이 가장 많이 도는 양 / 속도
  let tot = 0; for (let i = 1; i < qs.length; i++) tot += Math.max(...qs[i].map((v, j) => Math.abs(v - qs[i - 1][j])));
  return Math.max(0.4, tot / (speed ?? w.jointSpeed ?? 1.0));
}
function aFollow(w, plan, speed) {                  // plan() → 관절 값들 또는 이유 → 따라 움직임
  let qs, t = 0, dur;
  return { init() { const r = plan(); if (typeof r === "string") return r; qs = r; dur = pathDur(w, qs, speed); },
           update(dt) { t += dt; const u = smooth(Math.min(1, t / dur)) * (qs.length - 1), i = Math.min(qs.length - 2, Math.floor(u)); w.setQ(lerp(qs[i], qs[i + 1], u - i)); return t >= dur; } };
}
// 자유롭게 옮기기: 지금 → 특징점이 T 인 자세. 곧장 / 준비 자세를 거쳐
export function aReach(w, feature, Tfn, what, opt = {}) {
  return aFollow(w, () => {
    // 집게가 좌우 대칭이면 손 끝 축으로 180° 돌린 자세도 같은 잡기다 → 둘 다 풀어 본다
    const T = Tfn(), Ts = w.symmetric ? [T, mul(T, FLIP)] : [T];
    const sols = Ts.flatMap(t => w.solveAll(feature, t)); if (!sols.length) return `${what}: 팔이 닿지 않습니다`;
    const q0 = w.getQ(); let hit = null;
    for (const q1 of sols) {                        // 답마다: 곧장, 거쳐 갈 자세들을 거쳐
      const cands = [jointPath(q0, q1)];
      for (const v of w.vias?.() ?? []) cands.push([...jointPath(q0, v), ...jointPath(v, q1).slice(1)]);
      for (const qs of cands) { const h = w.pathClash(qs, { pad: 0.004, ...opt, allow: null }); if (!h) return qs; hit = hit ?? h; }
    }
    // 그래도 막히면: 지금 자리에서 곧게 위로 → 목표 위 → 곧게 내려가기
    for (const t of Ts) for (const up of [0.15, 0.25]) {
      const T0 = w.feat(feature), A = T0.slice(), B = t.slice(); A[11] += up; B[11] += up;
      const a = w.lineQ(feature, q0, T0, A, Math.ceil(up / 0.01)); if (!a) continue;
      const qa = a[a.length - 1], qb = w.solveAll(feature, B, [qa, ...w.seeds()])[0]; if (!qb) continue;
      const b = w.lineQ(feature, qb, B, t, Math.ceil(up / 0.01)); if (!b) continue;
      const qs = [...a, ...jointPath(qa, qb).slice(1), ...b.slice(1)];
      const h = w.pathClash(qs, { pad: 0.004, ...opt, allow: null }); if (!h) return qs; hit = hit ?? h;
    }
    return `${what}: ${w.clashText(hit)}`;
  });
}
// 다가가서 곧게 들어가기를 한 번에 계획: 앞 자세의 여러 팔 모양 중 그다음 곧은 줄까지 되는 것을 고른다
//   (앞 자세는 풀리는데 그 팔 모양으로는 곧게 못 들어가는 일이 있다 — 11-02 Spot)
export function aReachLine(w, feature, TpreFn, TgoalFn, what, opt = {}, lineOpt = {}) {
  let line = null;
  const reach = aFollow(w, () => {
    const P0 = TpreFn(), G0 = TgoalFn(), pairs = Array.isArray(P0[0]) ? P0.map((p, i) => [p, G0[i]]) : [[P0, G0]];   // 후보 여러 개도 된다
    const Ts = pairs.flatMap(([Tp, Tg]) => w.symmetric ? [[Tp, Tg], [mul(Tp, FLIP), mul(Tg, FLIP)]] : [[Tp, Tg]]);
    const q0 = w.getQ(); let hit = null, why = `${what}: 팔이 닿지 않습니다`;
    // 앞 자세가 안 풀리면 목표 자세의 답에서 시작해 다시 푼다 (팔이 닿는 끝자락에서는 시작 자세에 따라 풀이가 갈린다 — 11-02, 몸이 2cm 다르면 갈렸다)
    const sols = (tp, tg) => { const a = w.solveAll(feature, tp); if (a.length || !tg) return a; const g = w.solveAll(feature, tg); return g.length ? w.solveAll(feature, tp, [...g, ...w.seeds()]) : a; };
    for (const [tp, tg] of Ts) for (const q1 of sols(tp, tg)) {
      const dist = Math.hypot(tg[3] - tp[3], tg[7] - tp[7], tg[11] - tp[11]);
      const ln = w.lineQ(feature, q1, tp, tg, Math.max(4, Math.ceil(dist / 0.01))); if (!ln) { why = `${what}: 팔이 곧게 못 갑니다`; continue; }
      const hl = w.pathClash(ln, lineOpt); if (hl) { hit = hit ?? hl; continue; }
      const cands = [jointPath(q0, q1)];
      for (const v of w.vias?.() ?? []) cands.push([...jointPath(q0, v), ...jointPath(v, q1).slice(1)]);
      for (const up of [0.15, 0.25]) {                // 지금 자리에서 곧게 위로 → 앞 자세 위 → 곧게 내려와 앞 자세
        const T0 = w.feat(feature), A = T0.slice(), B = tp.slice(); A[11] += up; B[11] += up;
        const a = w.lineQ(feature, q0, T0, A, Math.ceil(up / 0.01)); if (!a) continue;
        const b = w.lineQ(feature, q1, tp, B, Math.ceil(up / 0.01)); if (!b) continue;
        cands.push([...a, ...jointPath(a[a.length - 1], b[b.length - 1]).slice(1), ...b.slice().reverse().slice(1)]);
      }
      for (const qs of cands) { const h = w.pathClash(qs, { pad: 0.004, ...opt, allow: null }); if (!h) { line = ln; return qs; } hit = hit ?? h; }
      // 그래도 막히면: 앞 자세에서 손 끝 반대로 15·30cm 떨어진 곳까지 가서 (팔을 거둬 돌고) 곧게 들어오기
      //   (칸 앞으로 비스듬히 들어오다 든 책이 판·칸막이를 친다 — 11-02 UR15 가 반대 방향으로 돌아 왔을 때)
      const z = col(tp, 2);
      for (const back of [0.15, 0.3]) {
        const B = tp.slice(); B[3] -= z[0] * back; B[7] -= z[1] * back; B[11] -= z[2] * back;
        const near = (q) => q.reduce((a, v, i) => a + Math.abs(v - q1[i]), 0);
        const qb = w.solveAll(feature, B, [q1]).sort((a, b) => near(a) - near(b))[0]; if (!qb) continue;
        const b = w.lineQ(feature, qb, B, tp, Math.ceil(back / 0.01)); if (!b) continue;
        for (const qs of [[...jointPath(q0, qb), ...b.slice(1)], ...(w.vias?.() ?? []).map(v => [...jointPath(q0, v), ...jointPath(v, qb).slice(1), ...b.slice(1)])]) {
          const h = w.pathClash(qs, { pad: 0.004, ...opt, allow: null }); if (!h) { line = ln; return qs; } hit = hit ?? h;
        }
      }
    }
    return hit ? `${what}: ${w.clashText(hit)}` : why;
  });
  const go = aFollow(w, () => line, lineOpt.speed ?? 0.6);
  return [reach, go];
}
// 관절 자세로 옮기기 (걷기 전 팔 거두기 등): 곧장 · 먼저 위로 15cm 들었다가 — 둘 다 부딪힘 검사
export function aJointTo(w, qFn, what) {
  return aFollow(w, () => {
    const q0 = w.getQ(), q1 = qFn(), cands = [jointPath(q0, q1)], T0 = w.feat("pinch"), z = col(T0, 2);
    for (const [back, up] of [[0, 0.15], [0.2, 0], [0.2, 0.12], [0, 0.25], [0.3, 0.2]]) {   // 손 끝 반대로 물러나고 / 위로 들고 → 관절로
      const A = T0.slice(); A[3] -= z[0] * back; A[7] -= z[1] * back; A[11] += up - z[2] * back;
      const a = w.lineQ("pinch", q0, T0, A, Math.ceil(Math.hypot(back, up) / 0.01)); if (a) cands.push([...a, ...jointPath(a[a.length - 1], q1).slice(1)]);
    }
    let hit = null;
    for (const qs of cands) { const h = w.pathClash(qs, { pad: 0.004 }); if (globalThis.DBG) { const T = w.feat("pinch"); console.log("   거두기 후보", qs.length, h ? h.who + " " + (h.depth * 1000).toFixed(1) : "OK", "손", [T[3], T[7], T[11]].map(v => v.toFixed(2)).join(",")); } if (!h) return qs; hit = hit ?? h; }
    return `${what}: ${w.clashText(hit)}`;
  });
}
// 곧게 옮기기: 특징점을 지금 자리 → T 로 (contact: 책은 안 보고 물리로 밂)
export function aLine(w, feature, Tfn, what, opt = {}) {
  return aFollow(w, () => {
    const Ta = w.feat(feature); let Tb = Tfn();
    if (w.symmetric) { const F = mul(Tb, FLIP); if (Ta[1] * F[1] + Ta[5] * F[5] + Ta[9] * F[9] > Ta[1] * Tb[1] + Ta[5] * Tb[5] + Ta[9] * Tb[9]) Tb = F; }   // 지금 손 방향에 가까운 쪽
    const dist = Math.hypot(Tb[3] - Ta[3], Tb[7] - Ta[7], Tb[11] - Ta[11]);
    const qs = w.lineQ(feature, w.getQ(), Ta, Tb, Math.max(4, Math.ceil(dist / 0.01))); if (!qs) return `${what}: 팔이 곧게 못 갑니다`;
    const h = w.pathClash(qs.slice(1), opt); if (h) return `${what}: ${w.clashText(h)}`;   // 지금 자리(이미 닿아 있을 수 있음)는 빼고 움직이는 길만
    return qs;
  }, opt.speed ?? 0.6);
}
export const aDo = (fn) => ({ update: () => fn() ?? true });
export function aWait(s) { let t = 0; return { update(dt) { t += dt; return t >= s; } }; }
export function aSettle(w, max = 1.5) { let t = 0; return { update(dt) { t += dt; return t > 0.3 && (w.phys.resting() || t > max); } }; }
// 화면 표시용 표지 (움직임 없음): 지금 어느 단계(이동 · 잡기 · 놓기 …)의 어느 칸(열기 · 넣기 · 닫기 …)을 하는지 실행기에 알린다
export const aLeaf = (w, leaf, chip = null) => aDo(() => { w.exec?.mark?.(leaf, chip); });
export const aChip = (w, chip) => aDo(() => { w.exec?.chip?.(chip); });
// 걷는 로봇만: 이동 단계 표지 + 서는 자리로 걷기 (UR15 처럼 받침대에 고정된 로봇은 이동 단계가 없다)
const goActs = (w, leaf, target, bk) => { const a = w.base(target, bk); return a.length ? [aLeaf(w, leaf), ...a] : []; };
// 잡기: 손가락 둘 다 책에 닿아 있어야 손에 붙인다 (빈손을 닫았는데 책이 따라오면 안 된다)
//   grasp: "front" = 앞부분을 두께로 (그대로 꽂을 수 있다) · "flat" = 바닥에서 위로 폭을 (정렬대에서 다시 잡아야 한다)
function aHold(w, bk, grasp = "front") {
  return aDo(() => {
    const W = w.fk(), g = w.fingerGaps(W, bk);
    if (g.some(v => v > 0.004)) return `${bk.name}: 손가락이 책에 안 닿았습니다 (${g.map(v => (v * 1000).toFixed(0)).join("·")}mm)`;
    // 손가락이 비스듬히 눌러 물리가 책 모서리를 바닥에 몇 mm 파묻은 채면 (11-02 Spot) 그만큼 올려서 붙인다
    const low = Math.min(...itemPoints(bk, bk.T).map(p => p[2])); if (low < 0) { const T = bk.T.slice(); T[11] -= low; w.phys.placeBook(bk, T); }
    bk.rel = mul(inv(w.feat("pinch", W)), bk.T); bk.grasp = grasp; w.held = bk; w.phys.hold(bk);
  });
}
// 다시 계산할 때 맨 먼저: 손이 책꽂이·정렬대 가까이 있으면 손 끝 반대로 곧게 빼낸다 (들어간 길을 거꾸로)
export function aBackOff(w) {
  return aFollow(w, () => {
    const q0 = w.getQ(); if (!w.handNear()) return [q0, q0];
    const T0 = w.feat("pinch"), z = col(T0, 2);
    for (const [back, up] of [[0.15, 0], [0.2, 0.02], [0.1, 0.05], [0.25, 0], [0.06, 0.08], [0, 0.1]]) {
      const A = T0.slice(); A[3] -= z[0] * back; A[7] -= z[1] * back; A[11] += up - z[2] * back;
      const a = w.lineQ("pinch", q0, T0, A, Math.max(4, Math.ceil(Math.hypot(back, up) / 0.01))); if (!a) continue;
      if (!w.pathClash(a.slice(1), { books: false })) return a;
    }
    return [q0, q0];                                // 못 빼면 그대로 두고 다음 계획에 맡긴다 (안 되면 그 계획이 이유를 말한다)
  }, 0.4);
}
function aRelease(w) { return aDo(() => { const bk = w.held; if (!bk) return; w.phys.release(bk); w.held = null; bk.rel = null; }); }

// ── 공정: 책 한 권 ───────────────────────────────────────────
//   책꽂이 좌표 S: x = 안쪽, y = 왼쪽, z = 위. 특징점 pinch(잡는 점: z = 다가가는 쪽, y = 닫히는 축) · tip(손끝: z = 가리키는 쪽)
const sx = (S) => col(S, 0), sy = (S) => col(S, 1), sz = (S) => col(S, 2);
function bookIn(S, bk) { return mul(inv(S), bk.T); }
function inS(S, p) { return apply(S, p); }
// 책 옆 틈 (책꽂이 y 로, 왼쪽·오른쪽: 옆 책 또는 칸 벽까지)
export function sideGaps(w, bk, cell) {
  const c = parseCell(cell), S = shelfT(c.shelf), Si = inv(S), b = cellBox(c.k), y = mul(Si, bk.T)[7];
  let left = b.hi[1] - (y + BOOK.t / 2), right = (y - BOOK.t / 2) - b.lo[1];
  for (const o of w.booksIn(cell)) { if (o === bk) continue; const yo = mul(Si, o.T)[7]; if (yo > y) left = Math.min(left, yo - y - BOOK.t); else right = Math.min(right, y - yo - BOOK.t); }
  return [left, right];
}
// 꺼내기: 옆 틈에 손가락이 들어가면 바로 집고, 좁으면 갈고리(책 한 권 폭 안에 드는 로봇만)로 4cm 당긴 뒤 튀어나온 앞부분을 집는다
function extractActs(w, bk, cell) {
  const S = shelfT(parseCell(cell).shelf), X = sx(S), Y = sy(S), Zd = sz(S);
  const L = () => bookIn(S, bk), top = () => L()[11] + BOOK.h / 2, back = () => L()[3] + BOOK.d / 2, front = () => L()[3] - BOOK.d / 2;
  const H = w.hookCfg;                              // 로봇마다: 책 위 높이 · 책 뒤로 얼마 · 갈고리 내리는 동작
  // 손끝 앞 = 안쪽. 닫히는 축 = 아래(UR15: 벌리면 아래 손가락이 내려간다) 또는 위(손목을 숙여 내리는 로봇)
  const tipH = (x, z) => frame(X, w.hookUpright ? Zd : Zd.map(v => -v), inS(S, [x, L()[7], z]));
  // 닫히는 축(고정 턱 → 손가락)의 방향: 한쪽만 움직이는 집게는 손가락을 넓은 틈 쪽으로
  let side = 1;
  const pinchAt = (dx) => frame(X, Y.map(v => v * side), inS(S, [front() + 0.025 + dx, L()[7], L()[11] + (w.pinchUp ?? 0)]));
  const take = [
    aDo(() => { const [l, r] = sideGaps(w, bk, cell); side = l >= (w.fingerClear ?? 0) || l >= r ? 1 : -1; }),   // 손가락은 보통 왼쪽, 왼쪽이 좁으면 오른쪽
    aChip(w, "열기"), w.gripTo(bk),
    aChip(w, "넣기"), ...aReachLine(w, "pinch", () => pinchAt(-0.09), () => pinchAt(0), "앞부분 감싸기", {}, { allow: new Set([bk]) }),
    aChip(w, "닫기"), w.gripOn(bk), aHold(w, bk),
    aChip(w, "빼기"), aLine(w, "pinch", () => { const T = w.feat("pinch").slice(), d = w.pullOut ?? 0.20; T[3] -= X[0] * d; T[7] -= X[1] * d; T[11] += 0.008 - X[2] * d; return T; }, "꺼내기", { books: false, speed: 0.3 }),   // 손 방향 그대로 곧게 뒤로
  ];
  let hookT = null, y0 = null;
  // 갈고리: 손끝을 책 위로 넘겨 책 뒤로 내리고 앞으로 4cm 당긴다
  const hook = [
    aChip(w, "갈고리"), w.gripTo("closed"),
    aReach(w, "tip", () => tipH(front() - 0.10, top() + H.above), "갈고리 앞"),
    aDo(() => { y0 = { top: top(), back: back() }; }),
    aLine(w, "tip", () => tipH(back() + H.behind, top() + H.above), "책 위로 넘기기", { allow: new Set([bk]) }),
    w.hookDown(bk, H),
    aDo(() => { hookT = w.feat("tip"); }),
    aLine(w, "tip", () => { const T = hookT.slice(), d = H.pull; T[3] -= X[0] * d; T[7] -= X[1] * d; T[11] -= X[2] * d; return T; }, "당기기", { books: false, speed: 0.25 }),
    aSettle(w, 0.8),
    aDo(() => {                                      // 당긴 책이 칸에 똑바로 남았나 (떨어졌으면 멈춘다)
      if (w.cellOf(bk) !== cell || Math.abs(bk.T[10]) < 0.97) return `${bk.name}: 당기다가 책이 칸에서 떨어졌습니다`;
      const out = -(front() - FRONT_IN); w.log?.(`${bk.name}: ${(out * 100).toFixed(1)}cm 당겨 나옴`);
    }),
    w.hookUp(bk, H),
    aLine(w, "tip", () => tipH(front() - 0.10, y0.top + H.above), "손 빼기", { allow: new Set([bk]) }),
  ];
  return [
    { update() {                                     // 어느 방법으로 꺼낼지: 지금 옆 틈을 보고
      const g = Math.min(...sideGaps(w, bk, cell));
      if (g >= w.pinchClear) { w.log?.(`${bk.name}: 옆 틈 ${(g * 100).toFixed(1)}cm → 바로 집기`); w.exec.acts.splice(1, 0, ...take); return true; }
      if (w.hookHalf > BOOK.t / 2 + g - 0.002) return `${bk.name}: 옆 틈 ${(g * 100).toFixed(1)}cm — ${w.name} 손이 ${(w.pinchClear * 100).toFixed(1)}cm 틈을 써야 하고, 갈고리도 옆 책까지 걸려(폭 ${(w.hookHalf * 200).toFixed(1)}cm) 한 권만 뺄 수 없습니다`;
      w.log?.(`${bk.name}: 옆 틈 ${(g * 100).toFixed(1)}cm → 갈고리로 당겨 꺼내기`); w.exec?.hook?.add(bk);
      w.exec.acts.splice(1, 0, ...hook, ...take); return true;
    } },
  ];
}

// 든 책을 칸에 꽂는다: 앞 6cm 남기고 넣기 → 놓기 → 닫은 손끝으로 앞면 밀기
const PRE_IN = [0.18, 0.13, 0.08, 0.23];             // 넣기 전 책이 칸 앞에서 떨어진 거리 (앞이 먼저, 안 되면 다음)
const PRE_LIFT = [0.004, 0.008, 0.012];              // 넣을 때 책 아래와 판 사이
function insertActs(w, cell, avoid = null) {
  const c = parseCell(cell), S = shelfT(c.shelf), X = sx(S), Y = sy(S), b = cellBox(c.k);
  let y = null;
  const bookT = (out, lift = 0.004) => inS(S, [FRONT_IN + BOOK.d / 2 - out, y, b.lo[2] + BOOK.h / 2 + lift]);
  // 앞 4cm 만 남긴다 (더 내밀면 놓을 때 무게중심이 판 끝에 걸려 앞으로 넘어진다)
  // 든 책이 bookT 에 똑바로 서도록 하는 잡는 점 자세 (위아래 두 가지 중 책이 서는 쪽)
  const pinchFor = (out, lift) => {
    const rel = w.held.rel, p = bookT(out, lift);
    for (const s of [1, -1]) {
      const R = frame(X, Y.map(v => v * s), [0, 0, 0]), B = mul(R, rel);
      if (Math.abs(B[10]) < 0.99) continue;          // 책 높이축이 위아래가 아니면 다른 쪽
      const o = apply(R, [rel[3], rel[7], rel[11]]);
      R[3] = p[0] - o[0]; R[7] = p[1] - o[1]; R[11] = p[2] - o[2]; return R;
    }
    return null;
  };
  const front = () => mul(inv(S), w.lastPut.T)[3] - BOOK.d / 2;
  const tipFront = (x) => { const L = mul(inv(S), w.lastPut.T); return frame(X, Y, inS(S, [x, L[7], L[11]])); };
  return [
    aChip(w, "넣기"), aDo(() => { y = w.slotY(cell, avoid?.()); if (y === null) return `${cell} 칸에 자리가 없습니다`; }),
    // 앞 자세: 책을 18cm 앞에서 곧게 넣는다. 그 자리가 팔 끝자락이면 13·8·23cm 도 (11-02: 몸이 2cm 다르면 18cm 앞이 안 닿았다)
    //   든 책 아래가 판에 몇 mm 걸리면 조금 높여서 (판 위 4 → 8 · 12mm, 놓으면 1cm 안 떨어져 앉는다 — 11-02 손에 든 자리가 2mm 다르면 걸렸다)
    ...aReachLine(w, "pinch", () => { const c = PRE_LIFT.flatMap(h => PRE_IN.map(d => pinchFor(OUT + d, h))).filter(Boolean); return c.length ? c : w.feat("pinch"); },
                  () => { const g = PRE_LIFT.flatMap(h => PRE_IN.map(() => pinchFor(OUT, h))); return g[0] ? g : null; }, `${cell} 앞부분만 남기고 넣기`, {}, { books: false, speed: 0.25 }),
    aDo(() => { w.lastPut = w.held; }), aChip(w, "열기"), aRelease(w), w.gripTo(BOOK_ANY), aSettle(w, 1.0),
    aChip(w, "손 빼기"), aLine(w, "pinch", () => { const T = w.feat("pinch"); return frame(X, Y, add([T[3], T[7], T[11]], X, -0.08)); }, "손 빼기", { books: false }),
    aChip(w, "밀어 넣기"), w.gripTo("closed"),
    ...aReachLine(w, "tip", () => tipFront(front() - 0.03), () => tipFront(FRONT_IN - 0.001), "손끝으로 밀어 넣기", {}, { books: false, speed: 0.12 }),
    aSettle(w, 1.0),
    aLine(w, "tip", () => tipFront(-0.10), "손 빼기", { books: false }),
    aTidy(w, cell),
  ];
}
// 정리: 같은 칸에서 3mm 넘게 튀어나온 책(옆에 꽂다가 마찰로 끌려 나온 것)을 하나씩 밀어 넣는다
function aTidy(w, cell, round = 0) {
  return { update() {
    if (round >= 3) return true;
    const S = shelfT(parseCell(cell).shelf), Si = inv(S), X = sx(S), Y = sy(S);
    const out = w.booksIn(cell).filter(b => mul(Si, b.T)[3] - BOOK.d / 2 < FRONT_IN - 0.003);
    if (!out.length) return true;
    const acts = [];
    for (const b of out) {
      const at = (x) => { const L = mul(Si, b.T); return frame(X, Y, inS(S, [x, L[7], L[11]])); };
      acts.push(aReach(w, "tip", () => at(mul(Si, b.T)[3] - BOOK.d / 2 - 0.03), `${b.name} 정리`),
                aLine(w, "tip", () => at(FRONT_IN - 0.001), `${b.name} 밀어 넣기`, { books: false, speed: 0.12 }), aSettle(w, 0.8),
                aLine(w, "tip", () => at(-0.10), "손 빼기", { books: false }));
    }
    w.log?.(`${cell} 정리: ${out.map(b => b.name).join(", ")} 밀어 넣음`);
    w.exec.acts.splice(1, 0, ...acts, aTidy(w, cell, round + 1));
    return true;
  } };
}
// 바닥에 누운 책: 위에서 폭으로 집기 → 정렬대에 6cm 걸쳐 놓기 → 튀어나온 쪽을 두께로 다시 잡아 세움
//   세 토막으로 나눠 둔다 — 사용자가 도중에 무언가 옮기면 그때 상태(바닥 · 손에 눕혀 듦 · 정렬대 위)에서 이어 간다
const sbEdge = () => standBox().hi[0], OVER = 0.045;  // 4.5cm 만 걸친다 (무게중심은 정렬대 위)
function floorPick(w, bk) {
  const down = [0, 0, -1], ax = (k) => col(bk.T, k);
  // 닫히는 축 = 책 깊이축, 양쪽 방향 둘 다 (좌우가 다른 집게는 책 방향에 따라 손목을 반 바퀴 돈 쪽만 닿는다 — 11-02 Spot)
  const flats = () => { const yb = ax(0), n = Math.hypot(yb[0], yb[1]) || 1; return [1, -1].map(s => frame(down, [s * yb[0] / n, s * yb[1] / n, 0], [bk.T[3], bk.T[7], w.floorPinchZ ?? 0.026])); };
  return [
    aChip(w, "열기"), w.gripTo(bk, "wide"),
    aChip(w, "하강"), ...aReachLine(w, "pinch", () => flats().map(T => { T[11] += 0.10; return T; }), flats, "바닥 책 집으러 내려가기", {}, { allow: new Set([bk]) }),
    aChip(w, "닫기"), w.gripOn(bk, "wide"), aHold(w, bk, "flat"),
  ];
}
function floorToStand(w, bk) {
  // 정렬대 가장자리(로봇 쪽 = +x)에 걸치게: 책 깊이축이 ±x 둘 다 같은 놓기 (걸치는 쪽은 그대로 +x 끝) — 팔이 닿는 쪽으로
  const restTs = () => [0, Math.PI].map(yw => rpyT(Math.PI / 2, 0, yw, [sbEdge() - BOOK.d / 2 + OVER, STAND.y, STAND.h + BOOK.t / 2 + 0.003]));
  const placePinch = () => restTs().map(R => mul(R, inv(w.held.rel)));
  const above = (Ts) => Ts.map(T => { const M = T.slice(); M[11] += 0.08; return M; });
  return [
    aChip(w, "상승"), aLine(w, "pinch", () => { const T = w.feat("pinch"); T[11] = Math.max(T[11], (w.floorPinchZ ?? 0.026) + 0.15); return T; }, "들기", { books: false }),
    ...goActs(w, "go:stand", "stand"),
    aLeaf(w, "regrip", "걸쳐 놓기"), ...aReachLine(w, "pinch", () => above(placePinch()), placePinch, "정렬대에 놓기", {}, { books: false, speed: 0.2 }),
    aRelease(w), w.gripTo(bk, "wide"), aSettle(w, 1.0),
    aLine(w, "pinch", () => { const T = w.feat("pinch"); T[11] += 0.10; return T; }, "손 들기", { books: false }),
  ];
}
function standRegrip(w, bk) {
  const regrip = () => { const L = bk.T; return frame([-1, 0, 0], [0, 0, 1], [sbEdge() + OVER - 0.025, L[7], L[11]]); };
  return [
    aChip(w, "열기"), w.gripTo(bk, "open"),
    aChip(w, "감싸기"), ...aReachLine(w, "pinch", () => { const T = regrip(); T[3] += 0.09; return T; }, regrip, "걸친 쪽 감싸기", {}, { allow: new Set([bk]) }),
    aChip(w, "닫기"), w.gripOn(bk), aHold(w, bk),
    aChip(w, "빼내기"), aLine(w, "pinch", () => { const T = w.feat("pinch"); T[3] += 0.10; T[11] += 0.06; return T; }, "빼내기", { books: false }),
  ];
}
function floorActs(w, bk) { return [...floorPick(w, bk), ...floorToStand(w, bk), ...standRegrip(w, bk)]; }
// 칸에 놓아만 두고 아직 안 민 책이 있으면 (밀다가 멈춘 것) 그 칸 앞에 가서 정리
function tidyActs(w, cell) {
  const S = shelfT(parseCell(cell).shelf), Si = inv(S);
  return w.booksIn(cell).some(b => mul(Si, b.T)[3] - BOOK.d / 2 < FRONT_IN - 0.003) ? [...w.base(cell), aTidy(w, cell)] : [];
}

// ── 목표 계산 (▶ 누르는 순간 한 번) ─────────────────────────────
//   '케어' = 책 (목표 자리를 지킨다) · '돈케어' = 책꽂이 (어디로 옮기든 칸 이름으로 찾아간다)
//   블록의 말("1-1 의 책 전부", "바닥 책", "한 권")을 지금 장면의 실제 책들로 묶고 (변수 → 값),
//   블록을 차례로 따라가 책마다 거쳐 갈 자리와 최종 목표를 정한다. 이때 안 되는 블록(빈 칸에서 옮기기 등)은 시작 전에 알린다.
//   반환: { start: 책→처음 자리, goal: 책→최종 자리, steps: 책→[{k, from, to}], bind: 블록마다 묶인 책들 }
export function planGoals(w, program) {
  const start = new Map(), state = new Map(), steps = new Map(), bind = [], inCell = new Map();
  for (const b of w.books) { const l = w.locOf(b); start.set(b, l); state.set(b, l); steps.set(b, []); }
  for (const c of CELLS) inCell.set(c, w.booksIn(c));                  // 칸 안 순서 (오른쪽부터 = 꺼내기 쉬운 순)
  program.forEach((blk, k) => {
    const [a0, a1, a2] = blk.args ?? [], no = `${k + 1}번 블록`;
    bind[k] = [];
    if (blk.type === "wait") return;
    if (blk.type === "move") {
      if (!parseCell(a0) || !parseCell(a1)) throw new Error(`${no} 책 옮기기: 칸을 고르세요`);
      if (a0 === a1) throw new Error(`${no} 책 옮기기: 같은 칸입니다`);
      const src = inCell.get(a0).filter(b => state.get(b) === a0);
      if (!src.length) throw new Error(`${no} 책 옮기기: 그때 ${a0} 칸에 책이 없습니다`);
      for (const b of a2 === "one" ? src.slice(0, 1) : src) { steps.get(b).push({ k, from: a0, to: a1 }); state.set(b, a1); inCell.get(a1).push(b); bind[k].push(b); }
      return;
    }
    if (blk.type === "floor") {
      if (!parseCell(a0)) throw new Error(`${no} 바닥 책 줍기: 칸을 고르세요`);
      const src = w.books.filter(b => ["floor", "stand", "hand"].includes(state.get(b)));
      if (!src.length) throw new Error(`${no} 바닥 책 줍기: 그때 바닥에 책이 없습니다`);
      for (const b of src) { steps.get(b).push({ k, from: state.get(b), to: a0 }); state.set(b, a0); inCell.get(a0).push(b); bind[k].push(b); }
      return;
    }
    throw new Error(`${no}: 모르는 블록 ${blk.type}`);
  });
  for (const b of w.books) if (state.get(b) === "hand") throw new Error(`로봇 손에 ${b.name} 이 있습니다 — 바닥 책 줍기 블록을 넣거나 ⌂ 원위치를 누르세요`);
  return { start, goal: state, steps, bind };
}

// ── 블록 실행기 ──
//   블록 하나 = 묶인 책들을 한 권씩. 한 권 끝날 때마다(그리고 사용자가 무언가 옮길 때마다) 책 전부를 다시 본다:
//   있어야 할 자리에 없는 책(케어)이 있으면 — 지금 블록과 상관없어도 — 그 책을 최종 목표 칸으로 옮긴다.
//   언제 고칠지는 world.fixMode (사용자가 고른다, 11-02):
//     "now"   바로 고치기   — 하던 일을 멈추고 그 책부터 (손에 든 책은 먼저 꽂는다: 손이 비어야 잡을 수 있다)
//     "book"  이 책 끝내고  — 지금 옮기는 책을 마저 꽂은 뒤
//     "block" 이 블록 끝내고 — 지금 블록의 책을 다 옮긴 뒤
//     "end"   맨 끝에       — 블록을 모두 끝낸 뒤 마무리 단계에서
//   블록을 다 하면 마무리 단계를 한 번 더 돈다: 제자리가 아닌 책이 남아 있으면 모두 고친다 (기다리기 블록 중에 건드린 책도).
//   사용자가 옮기면 interrupt(): 하던 동작을 버리고 그때 장면으로 다시 짠다. 블록은 칸 이름만 말하니 (좌표가 없다) 하나도 안 바뀐다.
export class BookExec {
  constructor(world, program, hooks, G) {
    this.w = world; this.program = program; this.hooks = hooks; this.i = -1; this.acts = []; this.done = false; this.again = false;
    this.G = G ?? planGoals(world, program);
    this.p = new Map(world.books.map(b => [b, 0]));   // 책마다 마친 걸음 수
    this.let = new Set();                             // 목표가 바닥인데 사용자가 옮긴 책 (바닥에 내려놓는 동작이 없어 그대로 둔다)
    // 화면 표시 (블록 나무): 지금 하는 일 node {bk, k, si(몇 번째 걸음), fix(고치기), from, to} · 그 안의 단계·칸 at {leaf, chip}
    //   cur = 지금 옮기는 책 ('이 책 끝내고' 에서 먼저 끝낼 책) · fixed = 고친 기록 · stepT = 걸음 마친 차례 · hook = 갈고리로 꺼낸 책
    this.node = null; this.at = null; this.cur = null; this.fixed = []; this.stepT = new Map(); this.tick = 0; this.hook = new Set();
  }
  expected(bk) { const st = this.G.steps.get(bk), n = this.p.get(bk); return n ? st[n - 1].to : this.G.start.get(bk); }
  mark(leaf, chip) { this.at = { leaf, chip }; }
  chip(c) { this.at = { leaf: this.at?.leaf ?? null, chip: c }; }
  update(dt) {
    if (this.done || this.waiting) return;           // 기다리는 중: 사람이 장면을 바꿀 때까지 (interrupt)
    let guard = 0;
    while (!this.acts.length) {
      if (++guard > 50) return;
      const more = this.more?.(); if (this.done || this.waiting) return;
      if (typeof more === "string") { this.finish(more); return; }    // 다시 짜다가 안 되는 이유
      if (more) { this.acts = more; continue; }
      this.more = null; this.i++;
      if (this.i > this.program.length) { this.finish(null); return; }
      if (this.i === this.program.length) { this.more = () => this.plan(); continue; }   // 마무리 단계
      this.hooks.block?.(this.i);
      try { this.compile(this.program[this.i]); } catch (e) { this.finish(e.message); return; }
    }
    const a = this.acts[0];
    if (!a.started) { a.started = true; const r = a.init?.(); if (typeof r === "string") { this.finish(r); return; } }
    const r = a.update(dt);
    if (typeof r === "string") { this.finish(r); return; }
    if (r) this.acts.shift();
  }
  // 장면이 바뀜 → 책 블록이면 하던 동작을 버리고 다음 update 에서 다시 짠다 (기다리기는 그대로 이어서)
  interrupt() {
    if (this.done || !this.more) return false;
    if (this.waiting) { this.waiting = null; this.hooks.resume?.(); }
    this.acts = []; this.again = true; this.w.stopBase?.();
    return true;
  }
  compile(b) {
    if (b.type === "wait") { this.acts = [aWait(+(b.args?.[0]) || 1)]; return; }
    this.more = () => this.plan();
  }
  // 다시 짤 때 앞에 붙는 것: 손 빼기 · 놓아만 두고 안 민 책이 있는 칸 정리
  head() {
    const w = this.w;
    if (!this.again) return []; this.again = false;
    const out = CELLS.filter(c => { const Si = inv(shelfT(parseCell(c).shelf)); return w.booksIn(c).some(x => mul(Si, x.T)[3] - BOOK.d / 2 < FRONT_IN - 0.003); });
    return [aLeaf(w, "pre", "물러나기"), aBackOff(w), ...out.flatMap(c => [aLeaf(w, "pre", `${c} 정리`), ...w.base(c), aTidy(w, c)])];
  }
  // 가져오기: 지금 자리에서 손에 들기까지
  fetch(bk, from) {
    const w = this.w;
    return from === "floor" ? [...goActs(w, "go:from", "floor", bk), aLeaf(w, "grab"), ...floorActs(w, bk)]
         : from === "stand" ? [...goActs(w, "go:stand", "stand"), aLeaf(w, "regrip"), ...standRegrip(w, bk)]
         : [...goActs(w, "go:from", from), aLeaf(w, "grab"), ...extractActs(w, bk, from)];
  }
  // 꽂기: fix = 고치기 (최종 목표로 바로 → 남은 걸음을 모두 마친 것으로)
  put(bk, to, fix) {
    const w = this.w, G = this.G;
    const later = () => new Set(w.books.filter(x => G.steps.get(x).slice(this.p.get(x)).some(st => st.from === to)));   // 나중에 이 칸에서 꺼낼 책
    return [...goActs(w, "go:to", to), aLeaf(w, "put"), ...insertActs(w, to, later), aDo(() => {
      const t = ++this.tick;
      if (fix) { this.fixed.push({ ...this.node, bk, t }); this.p.set(bk, G.steps.get(bk).length); }
      else { this.stepT.set(`${bk.id}:${this.p.get(bk)}`, t); this.p.set(bk, this.p.get(bk) + 1); }
      if (this.cur === bk) this.cur = null;
      this.node = null; this.at = null;
      w.log?.(`${bk.name} 꽂음 (${w.cellOf(bk) ?? "?"})`, "ok");
    })];
  }
  // 일 하나 = 책 한 권: 시작할 때 화면에 '지금 이 책' 을 알린다
  job(node, acts) { return [aDo(() => { if (this.node?.bk !== node.bk) this.at = null; this.node = node; }), ...acts]; }
  // 고치기: 있어야 할 자리에 없는 책 (사용자가 옮겼거나 물리로 떨어짐) → 최종 목표 칸으로
  //   여러 권이면 가장 최근에 건드린 책부터 (bk.touched = 사람이 옮긴 차례, 화면이 놓을 때 적는다) — 방금 옮긴 책을 먼저 (11-02 사용자)
  fix(pre) {
    const w = this.w, G = this.G;
    for (const bk of byTouch(w.books)) {
      const cur = w.locOf(bk), exp = this.expected(bk), goal = G.goal.get(bk);
      if (cur === exp || cur === "?" || this.let.has(bk)) continue;
      if (cur === goal) { this.p.set(bk, G.steps.get(bk).length); w.log?.(`${bk.name}: 이미 목표 ${goal} 에 있음`); continue; }
      if (!parseCell(goal)) { this.let.add(bk); w.log?.(`${bk.name}: 목표가 바닥이라 그대로 둡니다 (바닥에 내려놓는 동작은 아직 없음)`); continue; }
      w.log?.(`${bk.name}: 있어야 할 자리(${KO_LOC(exp)})가 아니라 ${KO_LOC(cur)} 에 있음 → 목표 ${goal} 로`, "bad");
      return this.job({ bk, k: this.i, fix: true, from: cur, to: goal }, [...pre, ...this.fetch(bk, cur), ...this.put(bk, goal, true)]);
    }
    return null;
  }
  // 다음 일 고르기 (한 권 끝날 때마다 · 사람이 무언가 옮길 때마다)
  plan() {
    const w = this.w, G = this.G, k = this.i, pre = this.head();
    const mode = k >= this.program.length ? "now" : (w.fixMode ?? "now");
    const stepOf = (bk) => G.steps.get(bk)[this.p.get(bk)];          // 이 책의 다음 걸음
    // 1) 들고 있는 책: 하던 걸음의 칸으로 (없으면 최종 목표로) — 손이 비어야 다른 책을 잡는다
    if (w.held) {
      const bk = w.held, st = stepOf(bk), to = st?.to ?? G.goal.get(bk);
      if (!parseCell(to)) return `${bk.name}: 놓을 칸이 없습니다`;
      w.log?.(`${bk.name}: 들고 있던 책 → ${to}`);
      const node = this.node?.bk === bk ? this.node : { bk, k: st?.k ?? k, si: st ? this.p.get(bk) : undefined, fix: !st, from: "hand", to };
      return this.job(node, [...pre, ...(bk.grasp === "flat" ? [aLeaf(w, "grab"), ...floorToStand(w, bk), ...standRegrip(w, bk)] : []), ...this.put(bk, to, !st)]);
    }
    // 2) 이 블록의 다음 책 (칸에 그대로 있는 것, 꺼내기 쉬운 순) — '이 책 끝내고' 면 하던 책부터
    const todo = (G.bind[k] ?? []).filter(bk => stepOf(bk)?.k === k && w.locOf(bk) === stepOf(bk).from);
    const keep = mode === "book" && this.cur && todo.includes(this.cur);
    if (mode === "now" || (mode === "book" && !keep)) { const f = this.fix(pre); if (f) return f; }
    if (todo.length) {
      const from = stepOf(todo[0]).from, order = parseCell(from) ? w.booksIn(from) : w.books;
      const bk = keep ? this.cur : todo.sort((a, c) => order.indexOf(a) - order.indexOf(c))[0], st = stepOf(bk);
      if (w.slotY(st.to) === null) return `${st.to} 칸이 꽉 찼습니다`;
      w.log?.(`${bk.name}: ${KO_LOC(st.from)} → ${st.to}`);
      this.cur = bk;
      return this.job({ bk, k, si: this.p.get(bk), fix: false, from: st.from, to: st.to }, [...pre, ...this.fetch(bk, st.from), ...this.put(bk, st.to, false)]);
    }
    // 3) '이 블록 끝내고': 이 블록의 책을 다 옮긴 뒤 ('맨 끝에' 는 마무리 단계에서 — 그때 mode 는 "now")
    if (mode === "block") { const f = this.fix(pre); if (f) return f; }
    return pre.length ? pre : null;
  }
  // 막힘: world.waitOnBlock 이면 끝내지 않고 이유를 말한 채 기다린다 (사람이 책꽂이·책을 옮겨 길을 열면 interrupt → 다시 짠다)
  //   검사(test_*.mjs)는 waitOnBlock 을 안 켠다 → 예전처럼 멈추고 이유를 남긴다
  finish(err) {
    if (this.done) return;
    if (err && this.w.waitOnBlock && this.more) { this.waiting = err; this.acts = []; this.again = true; this.w.stopBase?.(); this.hooks.wait?.(err); return; }
    this.done = true; this.w.stopBase?.();
    this.hooks.end?.(err, this.i);
    if (this.w.exec === this) this.w.exec = null;
  }
}
const byTouch = (books) => [...books].sort((a, b) => (b.touched ?? 0) - (a.touched ?? 0));   // 최근에 건드린 책부터 (같으면 책 번호 순)
const KO_LOC = (l) => ({ floor: "바닥", stand: "정렬대", hand: "손", "?": "알 수 없는 곳" })[l] ?? l;

// ── 블록 나무 (화면 표시용, 11-02) ──────────────────────────────
//   블록 → 책마다 (전부 = 책1 · 책2 · 책3 …) → 단계 (이동 · 잡기 · 이동 · 놓기) → 칸 (열기 · 넣기 · 닫기 · 빼기 …)
//   원본 ARM STUDIO 의 '잡기 = 열기 → 하강 → 닫기 → 상승' 과 같은 꼴. 책꽂이는 옆으로 꽂으니 하강·상승 자리에 넣기·빼기.
//   걷는 로봇(Spot)만 이동 단계가 있다 (UR15 는 받침대에 고정). 고치기(건드린 책)는 고를 차례에 맞춰 그 자리에 보인다.
const WALK = ["물러나기", "팔 거두기", "걷기", "몸 높이"], GRAB = ["열기", "넣기", "닫기", "빼기"], FLOORG = ["열기", "하강", "닫기", "상승"],
      REGRIP = ["열기", "감싸기", "닫기", "빼내기"], PUTC = ["넣기", "열기", "손 빼기", "밀어 넣기"];
function treeLeaves(w, bk, from, to, hook) {
  const L = [], go = (key, label, target) => { if (w.walks) L.push({ key, label, chips: WALK, go: target }); };   // go: 걸어갈 곳 (미리 보기 길 계산)
  if (parseCell(from)) { go("go:from", `이동 → ${from}`, from); L.push({ key: "grab", label: `잡기 ${bk.name}`, chips: hook ? ["갈고리", ...GRAB] : GRAB }); }
  else if (from === "floor" || (from === "hand" && bk.grasp === "flat")) {
    if (from === "floor") go("go:from", `이동 → ${bk.name}`, "floor");
    L.push({ key: "grab", label: `잡기 ${bk.name}`, chips: FLOORG });
    go("go:stand", "이동 → 정렬대", "stand");
    L.push({ key: "regrip", label: "다시 잡기", chips: ["걸쳐 놓기", ...REGRIP] });
  } else if (from === "stand") { go("go:stand", "이동 → 정렬대", "stand"); L.push({ key: "regrip", label: `다시 잡기 ${bk.name}`, chips: REGRIP }); }
  go("go:to", `이동 → ${to}`, to);
  L.push({ key: "put", label: `놓기 ${to}`, chips: PUTC });
  return L;
}
export function blockTree(w, program, G, ex, finished = false) {
  if (!G) return null;
  const mode = w.fixMode ?? "now", running = !!ex && !ex.done && !ex.stopped, P = program.length;
  const out = (n, state, id, extra = {}) => {
    const hook = ex?.hook?.has(n.bk) || (state !== "done" && parseCell(n.from) && w.locOf(n.bk) === n.from && Math.min(...sideGaps(w, n.bk, n.from)) < (w.pinchClear ?? 0));
    const L = treeLeaves(w, n.bk, n.from, n.to, hook), li = state === "cur" && ex.at ? L.findIndex(l => l.key === ex.at.leaf) : -1;
    return { id, book: n.bk.name, color: n.bk.color, from: KO_LOC(n.from), to: n.to, fix: !!n.fix, state, ...extra,
             note: state === "cur" && ex.at?.leaf === "pre" ? ex.at.chip : null,
             leaves: L.map((l, i) => ({ label: l.label, chips: l.chips, state: state === "done" || i < li ? "done" : i === li ? "cur" : "", chip: i === li ? ex.at.chip : null,
                                       go: l.go ? { target: l.go, bk: n.bk.id } : null })) };
  };
  const lists = [...program.map(() => ({ done: [], cur: [], rest: [] })), { done: [], cur: [], rest: [] }];   // 마지막 = 마무리 단계
  for (const bk of w.books) G.steps.get(bk).forEach((s, si) => {
    const n = { bk, from: s.from, to: s.to, k: s.k, si }, id = `s:${bk.id}:${si}`, L = lists[Math.min(s.k, P)];
    const t = ex?.stepT.get(`${bk.id}:${si}`);
    if (finished || (ex && ex.p.get(bk) > si)) L.done.push({ t: t ?? 1e9, o: out(n, "done", id, t == null && !finished ? { skip: true } : {}) });
    else if (running && ex.node && !ex.node.fix && ex.node.bk === bk && ex.node.si === si) L.cur.push(out(n, "cur", id));
    else L.rest.push({ o: G.bind[s.k].indexOf(bk), n: out(n, "", id) });
  });
  if (ex) for (const f of ex.fixed) lists[Math.min(f.k, P)].done.push({ t: f.t, o: out(f, "done", `f:${f.bk.id}:${f.t}`) });
  if (running && ex.node?.fix) lists[Math.min(ex.node.k, P)].cur.push(out(ex.node, "cur", `f:${ex.node.bk.id}:now`));
  // 고칠 차례를 기다리는 책: 지금 제자리가 아닌 것 (들고 있는 책 · 지금 하는 책은 빼고)
  //   고칠 차례대로 (최근에 건드린 책부터). 맨 앞 책에는 왜 아직 안 고치는지 (손에 든 책 먼저 · 고른 순서)
  const pend = [];
  if (running) for (const bk of byTouch(w.books)) {
    const l = w.locOf(bk), goal = G.goal.get(bk);
    if (bk === w.held || bk === ex.node?.bk || ex.let.has(bk) || l === "?" || l === ex.expected(bk) || l === goal || !parseCell(goal)) continue;
    const why = pend.length ? "그다음" : w.held ? "손에 든 책 꽂고 나서" : ex.i >= P ? "바로 다음" : { now: "바로 다음", book: "이 책 끝나면", block: "이 블록 끝나면", end: "맨 끝에" }[mode];
    pend.push(out({ bk, from: l, to: goal, fix: true }, "", `p:${bk.id}`, { pending: true, why }));
  }
  const ci = running ? Math.min(Math.max(ex.i, 0), P) : -1, endNow = running && ex.i >= P;
  const blocks = lists.map((L, k) => {
    const pendHere = !pend.length ? [] : endNow ? (k === P ? pend : []) : mode === "end" ? (k === P ? pend : []) : k === ci ? pend : [];
    const nodes = [...L.done.sort((a, b) => a.t - b.t).map(x => x.o), ...L.cur,
                   ...(mode !== "block" || endNow ? pendHere : []), ...L.rest.sort((a, b) => a.o - b.o).map(x => x.n), ...(mode === "block" && !endNow ? pendHere : [])];
    return { k, end: k === P, cur: running && k === ci, nodes };
  });
  // 지금 · 다음: 지금 하는 책의 지금 단계 · 칸, 그다음 단계 (이 책의 다음 단계 → 이 블록의 다음 책 → 다음 블록)
  let now = null;
  if (running) {
    const B = blocks[ci], i = B.nodes.findIndex(n => n.state === "cur"), N = B.nodes[i];
    if (N) {
      const li = N.leaves.findIndex(l => l.state === "cur"), lf = N.leaves[li];
      const M = B.nodes.slice(i + 1).find(n => n.state !== "done");
      const next = li >= 0 && li + 1 < N.leaves.length ? N.leaves[li + 1].label
                 : M ? `${M.fix ? "⟲ 고치기 " : ""}${M.book} · ${M.leaves[0]?.label ?? ""}` : ci + 1 < P ? `${ci + 2}번 블록` : "끝";
      now = { book: N.book, fix: N.fix, leaf: lf?.label ?? N.note ?? null, chip: lf?.chip ?? null, next };
    }
  }
  return { mode, now, blocks };
}

// ── 최종 자리 미리 보기 (11-02): 블록을 다 하면 책이 어느 칸 어디에 서나 → 3D 에 반투명 책 ──
//   꽂기와 같은 규칙: 칸 안 책들의 오른쪽(열린 쪽)에 4mm 띄워 붙인다 (slotY). 남은 걸음을 블록 차례대로 따라가며 칸마다 자리를 채워 본다.
//   실행 중이면 남은 걸음만 (이미 목표에 있는 책은 안 그린다). 고쳐야 할 책(다 했는데 제자리가 아님)은 지금 블록 차례에 목표로.
//   반환: [{id, color, cell, y}] — y 는 책꽂이 좌표라서 사용자가 책꽂이를 옮겨도 따라간다 (ghostT)
export function goalGhosts(w, G, ex) {
  if (!G) return [];
  const occ = new Map(CELLS.map(c => [c, new Map()]));            // 칸 → 책 → [가운데 y, 반 폭]
  for (const c of CELLS) { const Si = inv(shelfT(parseCell(c).shelf));
    for (const bk of w.booksIn(c)) { const L = mul(Si, bk.T); occ.get(c).set(bk, [L[7], Math.abs(L[6]) * BOOK.h / 2 + Math.abs(L[5]) * BOOK.t / 2]); } }
  const ev = [];
  for (const bk of w.books) {
    const steps = G.steps.get(bk), n = ex ? ex.p.get(bk) : 0, goal = G.goal.get(bk);
    if (!parseCell(goal) || ex?.let?.has(bk)) continue;
    if (n >= steps.length) { if (ex && w.locOf(bk) !== goal) ev.push({ bk, k: Math.max(0, ex.i), o: -1, to: goal }); continue; }
    steps.slice(n).forEach(s => ev.push({ bk, k: s.k, o: G.bind[s.k].indexOf(bk), to: s.to }));
  }
  ev.sort((a, b) => a.k - b.k || a.o - b.o);
  const pos = new Map();
  for (const e of ev) {
    for (const m of occ.values()) m.delete(e.bk);
    const c = parseCell(e.to); if (!c) { pos.delete(e.bk); continue; }
    const b = cellBox(c.k), ys = [...occ.get(e.to).values()].map(([y, h]) => y - h);
    const y = (ys.length ? Math.min(...ys) - 0.004 : b.hi[1] - WALL) - BOOK.t / 2;
    if (y - BOOK.t / 2 < b.lo[1] + WALL) { pos.delete(e.bk); continue; }      // 자리가 없다 (그때 가서 '꽉 찼습니다')
    occ.get(e.to).set(e.bk, [y, BOOK.t / 2]); pos.set(e.bk, { cell: e.to, y });
  }
  return [...pos].filter(([bk, p]) => p.cell === G.goal.get(bk) && w.locOf(bk) !== p.cell).map(([bk, p]) => ({ id: bk.id, color: bk.color, cell: p.cell, y: p.y }));
}
// 반투명 책의 월드 행렬 (끝까지 밀어 넣은 자리 = 앞면이 책꽂이 앞면에서 FRONT_IN 안쪽)
export function ghostT(g) { const c = parseCell(g.cell), b = cellBox(c.k); return mul(shelfT(c.shelf), rpyT(0, 0, 0, [FRONT_IN + BOOK.d / 2, g.y, b.lo[2] + BOOK.h / 2])); }
