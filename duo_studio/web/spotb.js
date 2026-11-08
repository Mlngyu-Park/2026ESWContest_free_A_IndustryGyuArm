// DUO STUDIO 책꽂이 — Spot (11-02): 같은 공정(bookworld.js)에 Spot(네 발 + 집게 팔)을 끼운다.
//   걷기: 칸 앞(책꽂이 앞면에서 0.88m)에 책꽂이를 보고 선다. 빈손이면 팔을 접고 걷는다.
//   집게: 한쪽 턱 고정 + 손가락 하나. 특징점(가상 링크)을 손목(wr1)에 붙여 공정의 pinch/tip 과 같은 뜻으로 맞춘다.
//     pinch: z = 손 끝 방향, y = 닫히는 축(고정 턱 → 손가락). 물건을 고정 턱에 붙여 잡는다.
//     tip:   손 끝. 갈고리는 손가락이 아래로 오게 뒤집어 넣고 벌린다 (손가락이 책 뒤로 내려간다).
import { Robot, Walker, drive, poseT, rpyT, mul, inv, apply, clamp, ARM, STOW, READY, GRIP_OPEN } from "./spotkin.js";
import { Chain } from "./g1kin.js";
import { BookWorld, aDo, lerp, aJointTo, frame, aChip } from "./bookworld.js";
// 이름 + 이/가 (받침 있으면 '이')
const iga = (n) => { const ch = String(n).trim().slice(-1), d = "013678"; const c = ch.charCodeAt(0);
  const jong = c >= 0xac00 && c <= 0xd7a3 ? (c - 0xac00) % 28 > 0 : /\d/.test(ch) ? d.includes(ch) : false; return `${n}${jong ? "이" : "가"}`; };
import { BOOK, SHELVES, shelfT, parseCell, cellBox, STAND, standBox, shelfFoot, standFoot, rectsOverlap, FRONT_IN } from "./shelf.js";
import { gap } from "./collide.js";

const JAW_Z = -0.032, TIP_X = 0.247, F_CLOSED = -0.05;
const smooth = (u) => u * u * (3 - 2 * u);
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const LEGS = ["front_left", "front_right", "rear_left", "rear_right"].flatMap(l => [`${l}_upper_leg`, `${l}_lower_leg`]);
const ARM_LINKS = ["arm_link_el0", "arm_link_el1", "arm_link_wr0", "arm_link_wr1", "arm_link_fngr"];
const NORMAL = { z: 0.52, pitch: 0 }, LOW = { z: 0.40, pitch: 0.12 }, MID = { z: 0.44, pitch: 0.15 }, DEEP = { z: 0.36, pitch: 0.35 };
const STAND_D = 0.88;                                // 책꽂이 앞면 → 몸통 가운데
// 공정 좌표(x = y_w, y = z_w, z = x_w) 를 손목에 붙이는 가상 링크
const VIRT = (name, xyz) => ({ name, type: "fixed", parent: "arm_link_wr1", child: name, xyz, rpy: [Math.PI / 2, 0, Math.PI / 2], axis: [1, 0, 0], lower: null, upper: null });

// ── 길 찾기 (11-02 셋째: 사용자가 책꽂이를 아무 데나 옮긴다) ──────────────────────
//   몸(다리 포함, 모델에서 잼): 앞 0.62 · 뒤 0.565 · 옆 0.224 → 4cm 여유 = 직사각형 FOOT.
//   바닥 책은 높이 2.2cm 라 몸통은 위로 지나가도 된다 — 발이 딛는 범위(앞 0.43 · 뒤 0.39 · 옆 0.25 → 2cm 여유)만 본다.
//   찾기: 바닥 10cm 칸 × 몸 방향 15°(24방향) 위에서 가장 짧은 길(A*). 움직임 = 그 방향 그대로 8방향 옆걸음 · 제자리 15° 돌기.
//   칸마다 몸 직사각형이 장애물·바닥 끝에 닿는지 본다 → 좁은 곳에서도 몸이 실제로 들어가면 길이 나온다 (예전: 지름 1.3m 원이 빈 곳에서만 돌았다).
export const FOOT = { f: 0.66, r: 0.61, s: 0.27 }, FLOOR = 2.9;
export const spotFoot = (x, y, yaw) => ({ cx: x + Math.cos(yaw) * (FOOT.f - FOOT.r) / 2, cy: y + Math.sin(yaw) * (FOOT.f - FOOT.r) / 2, hx: (FOOT.f + FOOT.r) / 2, hy: FOOT.s, yaw });
let FLOORBOOKS = () => [];
const FEET = { f: 0.45, r: 0.41, s: 0.27 };
const feetFoot = (x, y, yaw) => ({ cx: x + Math.cos(yaw) * (FEET.f - FEET.r) / 2, cy: y + Math.sin(yaw) * (FEET.f - FEET.r) / 2, hx: (FEET.f + FEET.r) / 2, hy: FEET.s, yaw });
export const obstacles = () => [...SHELVES.map(shelfFoot), standFoot(),
  ...FLOORBOOKS().map(b => ({ name: b.name, cx: b.T[3], cy: b.T[7], hx: BOOK.d / 2, hy: BOOK.h / 2, yaw: Math.atan2(b.T[4], b.T[0]), low: true }))];
const inFloor = (F) => [[-1, -1], [1, -1], [1, 1], [-1, 1]].every(([a, b]) => { const c = Math.cos(F.yaw), s = Math.sin(F.yaw);
  return Math.abs(F.cx + a * F.hx * c - b * F.hy * s) < FLOOR && Math.abs(F.cy + a * F.hx * s + b * F.hy * c) < FLOOR; });
// 이 자세로 서면 무엇에 닿나 (없으면 null) — 이유를 말할 때 이름을 쓴다
export function blockerAt(x, y, yaw, obs = obstacles()) {
  const F = spotFoot(x, y, yaw); if (!inFloor(F)) return { name: "바닥 끝" };
  const Ff = feetFoot(x, y, yaw); return obs.find(o => rectsOverlap(o.low ? Ff : F, o)) ?? null;
}
export const rectFree = (x, y, yaw, obs = obstacles()) => !blockerAt(x, y, yaw, obs);
function segFree(a, b, yaw, obs) { const n = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / 0.04)); for (let k = 0; k <= n; k++) if (!rectFree(a.x + (b.x - a.x) * k / n, a.y + (b.y - a.y) * k / n, yaw, obs)) return false; return true; }
function rotFree(x, y, y0, y1, obs) { const d = wrap(y1 - y0), n = Math.max(1, Math.ceil(Math.abs(d) / 0.13)); for (let k = 0; k <= n; k++) if (!rectFree(x, y, y0 + d * k / n, obs)) return false; return true; }
const GH = 0.1, NK = 24, DK = 2 * Math.PI / NK, GL = FLOOR - 0.3, GN = Math.round(2 * GL / GH) + 1, W_ROT = 0.08;
// P(지금) → G(설 자리, 방향 포함). 반환: 경유점 [{x, y, yaw, pass}] (pass = 멈추지 않고 지나감) 또는 null
export function planRoute(P, G) {
  const obs = obstacles(), at = (i) => -GL + i * GH;
  if (Math.abs(wrap(G.yaw - P.yaw)) < 0.05 && segFree(P, G, G.yaw, obs)) return [G];                          // 곧장
  if (rotFree(P.x, P.y, P.yaw, G.yaw, obs) && segFree(P, G, G.yaw, obs)) return [{ x: P.x, y: P.y, yaw: G.yaw }, G];   // 제자리에서 돌고 곧장
  const memo = new Map(), free = (i, j, k) => { const key = (i * GN + j) * NK + k; let v = memo.get(key); if (v === undefined) { v = rectFree(at(i), at(j), k * DK, obs); memo.set(key, v); } return v; };
  // 상태 = (칸 i, j, 방향 k, 마지막 움직임 t: 0 걷기 · 1 돌기) — 걷기↔돌기를 바꿀 때 벌점 (조금 가고 15° 돌고 … 끊기지 않게, 한 번에 몰아서 돈다)
  const SW = 0.06, id = (i, j, k, t = 0) => ((i * GN + j) * NK + k) * 2 + t, ki = (yaw) => ((Math.round(yaw / DK) % NK) + NK) % NK;
  // 출발: 지금 자리 둘레 칸들로 (방향 맞춰 돌고 → 미끄러져)
  const g = new Map(), from = new Map(), heap = [], push = (c, s) => { heap.push([c, s]); let i = heap.length - 1; while (i) { const p = (i - 1) >> 1; if (heap[p][0] <= heap[i][0]) break; [heap[p], heap[i]] = [heap[i], heap[p]]; i = p; } };
  const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === i) break; [heap[m], heap[i]] = [heap[i], heap[m]]; i = m; } } return top; };
  const hcost = (i, j, k) => Math.hypot(at(i) - G.x, at(j) - G.y) + W_ROT * Math.min(Math.abs(k - ki(G.yaw)), NK - Math.abs(k - ki(G.yaw)));
  const i0 = Math.round((P.x + GL) / GH), j0 = Math.round((P.y + GL) / GH);
  for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) for (const k of [ki(P.yaw), (ki(P.yaw) + 1) % NK, (ki(P.yaw) + NK - 1) % NK]) {
    const i = i0 + di, j = j0 + dj; if (i < 0 || j < 0 || i >= GN || j >= GN || !free(i, j, k)) continue;
    const c = { x: at(i), y: at(j) }; if (!rotFree(P.x, P.y, P.yaw, k * DK, obs) || !segFree(P, c, k * DK, obs)) continue;
    const cost = Math.hypot(c.x - P.x, c.y - P.y) + W_ROT * Math.abs(wrap(k * DK - P.yaw)) / DK, s = id(i, j, k, 0);
    if (cost < (g.get(s) ?? Infinity)) { g.set(s, cost); from.set(s, -1); push(cost + hcost(i, j, k), s); }
  }
  // 도착: G 근처 칸에서 그 방향 그대로 미끄러져 G 로 → 제자리에서 G 방향으로
  const kg = ki(G.yaw), isEnd = (i, j, k) => (k === kg || k === (kg + 1) % NK || k === (kg + NK - 1) % NK) && Math.hypot(at(i) - G.x, at(j) - G.y) < 0.16;
  const endOk = new Map(), endFree = (i, j, k) => { const s = id(i, j, k, 0); if (!endOk.has(s)) { const c = { x: at(i), y: at(j) }; endOk.set(s, segFree(c, G, k * DK, obs) && rotFree(G.x, G.y, k * DK, G.yaw, obs)); } return endOk.get(s); };
  let best = -1, n = 0;
  while (heap.length && n++ < 120000) {
    const [, s] = pop(), tt = s % 2, s0 = (s - tt) / 2, k = s0 % NK, ij = (s0 - k) / NK, i = Math.floor(ij / GN), j = ij % GN, c0 = g.get(s);
    if (isEnd(i, j, k) && endFree(i, j, k)) { best = s; break; }
    const step = (a, b, kk, t, cost) => { if (a < 0 || b < 0 || a >= GN || b >= GN || !free(a, b, kk)) return; const u = id(a, b, kk, t), c = c0 + cost + (t !== tt ? SW : 0); if (c < (g.get(u) ?? Infinity)) { g.set(u, c); from.set(u, s); push(c + hcost(a, b, kk), u); } };
    for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) {
      if (!di && !dj) continue;
      if (di && dj && !(free(i + di, j, k) && free(i, j + dj, k))) continue;        // 대각선은 양옆 칸도 비어야 (모서리를 깎지 않게)
      step(i + di, j + dj, k, 0, GH * Math.hypot(di, dj));
    }
    for (const dk of [1, NK - 1]) { const kk = (k + dk) % NK; if (rotFree(at(i), at(j), k * DK, kk * DK, obs)) step(i, j, kk, 1, W_ROT); }
  }
  if (best < 0) return null;
  const st = []; for (let s = best; s !== -1; s = from.get(s)) { const s0 = (s - s % 2) / 2, k = s0 % NK, ij = (s0 - k) / NK; st.unshift({ x: at(Math.floor(ij / GN)), y: at(ij % GN), yaw: k * DK, k }); }
  // 줄이기: 같은 방향으로 가는 구간은 곧게 갈 수 있는 만큼 건너뛰고, 방향이 바뀌는 곳은 그 자리에서 돌기
  const out = []; let a = 0;
  while (a < st.length) {
    let b = a; while (b + 1 < st.length && st[b + 1].k === st[a].k) b++;                       // 같은 방향 구간 [a, b]
    let p = a; out.push({ ...st[a], pass: false });
    while (p < b) { let q = b; while (q > p + 1 && !segFree(st[p], st[q], st[a].yaw, obs)) q--; out.push({ ...st[q], pass: q < b }); p = q; }
    a = b + 1;
  }
  const pts = out.filter((p, i) => !(i + 1 < out.length && out[i + 1].x === p.x && out[i + 1].y === p.y && i > 0 && out[i - 1].x === p.x && out[i - 1].y === p.y));   // 제자리 돌기는 처음·끝만
  return [...pts.map(({ x, y, yaw, pass }) => ({ x, y, yaw, pass })), { ...G, pass: false }];
}
// 옆걸음 몰기: 목표 자리로 곧게 (몸 방향은 목표 방향으로 천천히), 도착하면 true. pass 면 가까워지면 멈추지 않고 넘어간다
function hdrive(wk, g) {
  wk.aim?.(g);                                     // 물리 걷기: 새 목표면 도착 문턱을 처음 값으로
  const P = wk.pose, dx = g.x - P.x, dy = g.y - P.y, dist = Math.hypot(dx, dy), e = wrap((g.yaw ?? P.yaw) - P.yaw), tol = wk.tol ?? { d: 0.02, e: 0.03 };   // tol: 물리 걷기(월드 레이첼)는 3cm·0.05rad
  if (g.pass && dist < 0.08) return true;
  if (dist < tol.d && Math.abs(e) < tol.e) { wk.cmd = { vx: 0, vy: 0, wz: 0 }; return !wk.walking && Math.hypot(wk.vel.vx, wk.vel.vy, wk.vel.wz) < 1e-3; }
  const c = Math.cos(P.yaw), s = Math.sin(P.yaw), fx = dx * c + dy * s, fy = -dx * s + dy * c, sp = Math.min(0.3, 1.2 * dist);
  wk.cmd = { vx: dist > 1e-6 ? sp * fx / dist : 0, vy: dist > 1e-6 ? sp * fy / dist : 0, wz: clamp(1.5 * e, -0.6, 0.6) };
  return false;
}
export class SpotBooks extends BookWorld {
  // opt.makeWalker(robot, pose, w): 걷기를 바꿔 끼운다 (월드 레이첼 물리 걷기, world_rachel/phys_walker.mjs). 없으면 계산 걷기
  constructor(model, shapes, R, log = () => {}, opt = {}) {
    super();
    this.makeWalker = opt.makeWalker ?? null;
    this.kind = "spot"; this.sh = shapes; this.log = log; this.walks = true;   // 걷는 로봇: 블록 나무에 이동 단계가 있다
    // 잡는 점: 책 앞면이 손목 x 0.20 (턱 끝 0.236 → 앞 3.6cm 만 문다, 손목 몸통 0.145 보다 앞) · 고정 턱에 붙임
    const thin = [0.225, 0, JAW_Z + BOOK.t / 2 + 0.0025], wide = [TIP_X - 0.005 - BOOK.t / 2, 0, JAW_Z + BOOK.d / 2 + 0.0025];   // 턱 끝 부분이 2~3mm 더 높아 2.5mm 띄움
    this.model = { ...model, links: { ...model.links, pinch_thin: { visuals: [] }, pinch_wide: { visuals: [] }, tip: { visuals: [] } },
                   joints: [...model.joints, VIRT("pinch_thin", thin), VIRT("pinch_wide", wide), VIRT("tip", [0.24, 0, -0.012])] };
    this.handLinks = ["arm_link_wr1", "arm_link_fngr"];
    this.cfg = { links: [...ARM_LINKS, "body", ...LEGS], hand: new Set(this.handLinks), self: [[ARM_LINKS, ["body", ...LEGS]]], selfBody: ["body", ...LEGS] };
    this.jointSpeed = 1.2; this.mode = "thin";
    this.name = "Spot"; this.pinchClear = 0.035; this.hookHalf = 0.042; this.pullOut = 0.10; this.floorPinchZ = BOOK.t / 2; this.fingerClear = 0.043;   // 10cm 만 빼고 나머지는 몸이 물러나며   // 고정 턱·손가락 두께 3cm+여유 · 집게 폭 8.4cm (책 2.2cm 보다 넓어 갈고리는 옆 책까지 걸린다)
    this.setup(R);
    FLOORBOOKS = () => this.books.filter(b => b !== this.held && b.T[11] < 0.06);
  }
  // 몸을 바로 그 자리에 세운다 (로봇을 바꿔 끼울 때 처음 자리가 막혀 있으면)
  placeAt(x, y, yaw) { if (this.walker.placeAt) return this.walker.placeAt(x, y, yaw); Object.assign(this.walker.pose, { x, y, yaw }); for (const l of Object.keys(this.walker.feet)) this.walker.feet[l] = this.walker.nominal(l, this.walker.pose); this.walker.solveLegs(); }
  resetRobot() {
    this.robot = new Robot(this.model);
    this.walker?.dispose?.();                       // 물리 걷기는 제 물리 세계를 갖고 있다 → 버릴 때 풀어 준다
    this.walker = this.makeWalker ? this.makeWalker(this.robot, { x: -0.1, y: 0, yaw: 0 }, this) : new Walker(this.robot, { pose: { x: -0.1, y: 0, yaw: 0 } });
    this.chains = Object.fromEntries(["pinch_thin", "pinch_wide", "tip"].map(n => [n, new Chain(this.robot, "body", n, [0, 0, 0], ARM)]));
    for (const n of ARM) this.robot.q[n] = STOW[n];
    this.robot.q.arm_f1x = F_CLOSED;
    this.fingerTable();
  }
  // 손가락 각도 → 손가락 안쪽 면 높이 (손목 z, 턱 끝 부근) · 손가락 끝 높이
  fingerTable() {
    if (SpotBooks.TABLE) return;
    const T = [], q0 = this.robot.q.arm_f1x, I = rpyT(0, 0, 0);
    for (let a = 0; a >= -1.57; a -= 0.01) {
      this.robot.q.arm_f1x = a; const W = this.robot.fk(I), P = inv(W.arm_link_wr1), o = this.sh.world("arm_link_fngr", mul(P, W.arm_link_fngr));
      let inner = 9, tipz = -9;
      for (let i = 0; i < this.sh.L.arm_link_fngr.n; i++) { const x = o[3 * i], z = o[3 * i + 2]; if (x > 0.19 && x < 0.236) inner = Math.min(inner, z); tipz = Math.max(tipz, z); }
      T.push({ a, inner, tipz });
    }
    this.robot.q.arm_f1x = q0; SpotBooks.TABLE = T;
  }
  baseT() { return poseT(this.walker.pose); }
  fk() { return this.robot.fk(this.baseT()); }
  pose() { return this.walker.pose; }
  feat(name, W = this.fk()) { return W[name === "pinch" ? `pinch_${this.mode}` : "tip"]; }
  chainFor(name) { return this.chains[name === "pinch" ? `pinch_${this.mode}` : "tip"]; }
  getQ() { return ARM.map(n => this.robot.q[n]); }
  setQ(q) { ARM.forEach((n, i) => this.robot.q[n] = q[i]); }
  seeds() { const r = ARM.map(n => READY[n]); return [this.getQ(), r, [0, -1.6, 1.4, 0, 0.2, 0], [0, -0.6, 1.0, 0, -0.4, 0], [0.5, -1.2, 1.8, 0, -0.6, 1.57], [-0.5, -1.2, 1.8, 0, -0.6, -1.57], [0, -1.2, 2.0, 0, -0.8, 3.14]]; }
  vias() { return [ARM.map(n => READY[n])]; }
  stepBase(dt) { this.walker.update(dt); }
  stopBase() { this.walker.cmd = { vx: 0, vy: 0, wz: 0 }; }
  // ── 집게 ── 손가락 각도: 0 = 닫힘, -1.57 = 다 벌림
  fingerAt(gapW) { return (SpotBooks.TABLE.find(r => r.inner - JAW_Z >= gapW) ?? SpotBooks.TABLE.at(-1)).a; }
  fingerGaps(W, bk) { return [gap(this.sh, W, ["arm_link_wr1"], bk), gap(this.sh, W, ["arm_link_fngr"], bk)]; }
  widthOf(bk, mode) { return mode === "wide" ? BOOK.d / 2 : BOOK.t / 2; }
  aFinger(targetFn, dur = 0.4, before) {
    const w = this; let a, b, t = 0;
    return { init() { before?.(); a = w.robot.q.arm_f1x; b = targetFn(); }, update(dt) { t += dt; w.robot.q.arm_f1x = a + (b - a) * smooth(Math.min(1, t / dur)); return t >= dur; } };
  }
  // 책을 잡으러 벌릴 때 어느 잡는 점(얇은 쪽 · 넓은 쪽)을 쓸지 그때 정한다
  // 벌림: 책 두께(또는 폭) + 8mm — 손가락이 옆 틈에 들어가야 하니 너무 벌리지 않는다
  gripTo(arg, mode) {
    if (arg === "closed") return this.aFinger(() => F_CLOSED);
    return this.aFinger(() => this.fingerAt(2 * this.widthOf(arg, mode) + (mode === "open" ? 0.03 : 0.008)), 0.4, () => { this.mode = mode === "wide" ? "wide" : "thin"; });   // open: 옆에 책이 없을 때 넉넉히
  }
  // 닫기: 손가락 실제 모양으로 책에 처음 닿는 각도까지 + 조금 더 (물리가 책을 고정 턱에 눌러 붙인다)
  gripOn(bk) {
    const w = this;
    return this.aFinger(() => {
      const f0 = w.robot.q.arm_f1x, touch = (a) => { w.robot.q.arm_f1x = a; return gap(w.sh, w.fk(), ["arm_link_fngr"], bk) <= 0.0005; };
      let prev = f0, hit = null;
      for (let a = f0; a <= F_CLOSED + 1e-9 && hit === null; a += 0.01) { if (touch(a)) hit = a; else prev = a; }
      if (hit !== null) {
        let lo = prev, hi = hit; for (let k = 0; k < 8; k++) { const m = (lo + hi) / 2; if (touch(m)) hi = m; else lo = m; }
        // 고정 턱과 책 사이가 벌어진 만큼 손가락을 더 닫는다 (책을 턱 쪽으로 밀어 붙임)
        w.robot.q.arm_f1x = hi; const gj = gap(w.sh, w.fk(), ["arm_link_wr1"], bk);
        hit = hi;
        for (let a = hi; a <= F_CLOSED + 1e-9; a += 0.002) { w.robot.q.arm_f1x = a; hit = a; if (gap(w.sh, w.fk(), ["arm_link_fngr"], bk) <= -(gj - 0.0005)) break; }
      }
      w.robot.q.arm_f1x = f0;
      return hit ?? F_CLOSED;
    }, 0.7);
  }
  // 갈고리: 닫은 집게를 바로 세운 채(고정 턱이 아래) 책 위로 넘긴 뒤 손목을 숙여 턱 끝을 책 뒤로 내린다
  //   (뒤집으면 가운데·아랫단에 팔이 안 닿는다 — 11-02 계산). 숙이는 각도는 책꽂이·책을 안 치는 첫 것
  hookUpright = true;
  hookCfg = { above: 0.07, behind: 0.14, pull: 0.09 };
  hookDown(bk, H) {
    const w = this; let qs, t = 0, dur;
    return { init() {
               const S = shelfT(parseCell(w.cellOf(bk)).shelf), X = [S[0], S[4], S[8]], Z = [S[2], S[6], S[10]], Y = [S[1], S[5], S[9]];
               const T0 = w.feat("tip"), L = mul(inv(S), bk.T), top = L[11] + BOOK.h / 2, back = L[3] + BOOK.d / 2;
               let why = "갈고리 내리기: 손목을 숙여도 책 뒤로 못 내려갑니다";
               for (const th of [0.6, 0.75, 0.9, 1.05, 0.45]) for (const dpt of [0.035, 0.025]) {
                 const d = X.map((v, i) => v * Math.cos(th) - Z[i] * Math.sin(th)), y = Z.map((v, i) => v * Math.cos(th) + X[i] * Math.sin(th));
                 const p = apply(S, [back + H.behind - 0.01, L[7], top - dpt]);
                 const T1 = [0, 0, 0, p[0], 0, 0, 0, p[1], 0, 0, 0, p[2], 0, 0, 0, 1];
                 const xax = [y[1] * d[2] - y[2] * d[1], y[2] * d[0] - y[0] * d[2], y[0] * d[1] - y[1] * d[0]];
                 [T1[0], T1[4], T1[8], T1[1], T1[5], T1[9], T1[2], T1[6], T1[10]] = [...xax, ...y, ...d];
                 const q = w.lineQ("tip", w.getQ(), T0, T1, 12); if (!q) { if (globalThis.DBG) console.log("  숙임", th, dpt, "팔이 못 감"); continue; }
                 const h = w.pathClash(q, { allow: new Set([bk]) }); if (h) { if (globalThis.DBG) console.log("  숙임", th, dpt, h.who, (h.depth * 1000).toFixed(1)); why = `갈고리 내리기: ${w.clashText(h)}`; continue; }
                 qs = q; w.hookBack = q.slice().reverse(); dur = 0.8; return;
               }
               return why;
             },
             update(dt) { t += dt; const u = smooth(Math.min(1, t / dur)) * (qs.length - 1), i = Math.min(qs.length - 2, Math.floor(u)); w.setQ(lerp(qs[i], qs[i + 1], u - i)); return t >= dur; } };
  }
  // 당긴 뒤: 손목을 다시 들어 (당긴 만큼 앞에서) 손끝이 책 위로
  hookUp(bk, H) {
    const w = this; let qs, t = 0;
    return { init() {
               const S = shelfT(parseCell(w.cellOf(bk) ?? "1-1").shelf), Z = [S[2], S[6], S[10]], X = [S[0], S[4], S[8]], T0 = w.feat("tip");
               const L = mul(inv(S), bk.T), top = L[11] + BOOK.h / 2, p0 = [T0[3], T0[7], T0[11]], pl = mul(inv(S), [1, 0, 0, p0[0], 0, 1, 0, p0[1], 0, 0, 1, p0[2], 0, 0, 0, 1]);
               const p = apply(S, [pl[3], L[7], top + H.above]), Y = Z, d = X;
               const T1 = [0, 0, 0, p[0], 0, 0, 0, p[1], 0, 0, 0, p[2], 0, 0, 0, 1], xax = [Y[1] * d[2] - Y[2] * d[1], Y[2] * d[0] - Y[0] * d[2], Y[0] * d[1] - Y[1] * d[0]];
               [T1[0], T1[4], T1[8], T1[1], T1[5], T1[9], T1[2], T1[6], T1[10]] = [...xax, ...Y, ...d];
               qs = w.lineQ("tip", w.getQ(), T0, T1, 12); if (!qs) return "손목 들기: 팔이 곧게 못 갑니다";
             },
             update(dt) { t += dt; const u = smooth(Math.min(1, t / 0.7)) * (qs.length - 1), i = Math.min(qs.length - 2, Math.floor(u)); w.setQ(lerp(qs[i], qs[i + 1], u - i)); return t >= 0.7; } };
  }

  // ── 걷기 ── 칸 앞 / 바닥 책 앞 / 정렬대 앞. 빈손이면 팔 접고, 들고 있으면 준비 자세로
  // 설 자리 후보 (좋은 순 · cost 가 작을수록 좋다). 사용자가 책꽂이를 아무 데나 옮기니 넓게:
  //   칸: 칸 정면 (단마다 거리·자세: 윗단 0.76m · 가운데 0.84m 조금 숙여 · 아랫단 1.0m 깊이 숙여 — 갈고리·집기·밀기가 다 되는 것, 11-02 계산)
  //       → 칸을 바라본 채 칸 둘레로 10·20·30° 돌아 서기 → 옆으로 6~25cm → 조금 뒤로
  //   바닥 책: 책을 보고 0.8m (지금 쪽부터 둘레로) · 정렬대: 정렬대 +x 쪽에서 -x 를 보고 (걸친 쪽을 다시 잡는다)
  standCands(target, bk) {
    const out = [];
    if (target === "floor") {
      const p = [bk.T[3], bk.T[7]], P = this.walker.pose, y0 = Math.atan2(p[1] - P.y, p[0] - P.x);
      for (const dist of [0.80, 0.75, 0.86]) for (let a = 0; a < 16; a++) { const dy = (a % 2 ? -1 : 1) * Math.ceil(a / 2) * Math.PI / 8, yaw = y0 + dy;
        out.push({ x: p[0] - dist * Math.cos(yaw), y: p[1] - dist * Math.sin(yaw), yaw, body: LOW, cost: Math.abs(dy) + 4 * Math.abs(dist - 0.8) }); }
    } else if (target === "stand") {
      const sb = standBox(), R = [sb.hi[0], STAND.y];
      for (const ph of [0, 0.17, -0.17, 0.35, -0.35]) for (const u of [0, 0.08, -0.08, 0.16, -0.16]) {
        const D = 0.86 + 0.0, c = Math.cos(ph), sn = Math.sin(ph);
        out.push({ x: R[0] + D * c, y: R[1] + u + D * sn, yaw: Math.PI + ph, body: MID, cost: 1.5 * Math.abs(ph) + 3 * Math.abs(u) });
      }
    } else {
      const c = parseCell(target), S = shelfT(c.shelf), b = cellBox(c.k), row = Math.floor((c.k - 1) / 3);
      const [d, body] = [[0.76, NORMAL], [0.84, MID], [1.0, DEEP]][row], yc = (b.lo[1] + b.hi[1]) / 2;
      const put = (lx, ly, ph, cost) => { const q = apply(S, [lx, ly, 0]); out.push({ x: q[0], y: q[1], yaw: c.shelf.yaw + ph, body, cost }); };
      for (const dd of [0, 0.05, 0.1]) {
        for (const ph of [0, 0.17, -0.17, 0.35, -0.35, 0.52, -0.52]) put(-(d + dd) * Math.cos(ph), yc - (d + dd) * Math.sin(ph), ph, 1.5 * Math.abs(ph) + 3 * dd);   // 칸을 보고 둘레로
        for (const u of [0.06, -0.06, 0.12, -0.12, 0.18, -0.18, 0.25, -0.25]) put(-(d + dd), yc + u, 0, 3 * Math.abs(u) + 3 * dd);   // 옆으로
      }
    }
    return out.sort((a, b) => a.cost - b.cost);
  }
  // 미리 보기 (11-02): 블록대로 걸을 길 전체를 ▶ 전에 계산해 3D 바닥에 그린다 — 몸은 움직이지 않는다
  //   legs = [{id, target, bk}] (블록 나무의 이동 단계 차례). 앞 다리가 끝난 자리에서 다음 다리를 찾는다 (걸을 때와 같은 후보 · 팔 닿기 · 길 찾기)
  //   지금 책 자리로 계산하므로 실제 걸음과 조금 다를 수 있다. 못 가는 다리는 pts = null (그러면 그 자리에서 다음 다리를 찾는다)
  previewRoute(legs) {
    const P = this.walker.pose, save = { ...P }, obs = obstacles(), out = [];
    try {
      for (const L of legs) {
        let got = null, routeless = 0;
        for (const c of this.standCands(L.target, L.bk)) {
          if (blockerAt(c.x, c.y, c.yaw, obs) || !this.armReach(c, L.target, L.bk)) continue;
          const r = planRoute(P, c); if (r) { got = { id: L.id, pts: [{ x: P.x, y: P.y }, ...r], goal: c }; break; }
          if (++routeless >= 5) break;
        }
        out.push(got ?? { id: L.id, pts: null, goal: null });
        if (got) Object.assign(P, { x: got.goal.x, y: got.goal.y, yaw: got.goal.yaw });
      }
    } finally { Object.assign(P, save); }
    return out;
  }
  standFor(target, bk) { return this.standCands(target, bk).find(g => rectFree(g.x, g.y, g.yaw)) ?? null; }
  // 그 자리에 서면 팔이 닿나 (잠깐 몸을 그 자리에 두고 팔 계산만 해 본다)
  armReach(g, target, bk) {
    const P = this.walker.pose, save = { ...P }, mode = this.mode, q0 = this.getQ();
    Object.assign(P, { x: g.x, y: g.y, yaw: g.yaw, z: g.body.z, pitch: g.body.pitch, roll: 0 });
    const any = (Ts) => Ts.some(T => this.solveAll("pinch", T, this.seeds(), 1).length);
    let ok;
    try {
      if (target === "floor") { this.mode = "wide"; const yb = [bk.T[0], bk.T[4]], n = Math.hypot(...yb) || 1;
        ok = any([1, -1].map(sg => frame([0, 0, -1], [sg * yb[0] / n, sg * yb[1] / n, 0], [bk.T[3], bk.T[7], this.floorPinchZ]))); }
      else if (target === "stand") { this.mode = "thin"; const sb = standBox(); ok = any([frame([-1, 0, 0], [0, 0, 1], [sb.hi[0] + 0.02, STAND.y, STAND.h + BOOK.t / 2])]); }
      else {
        this.mode = "thin"; const c = parseCell(target), S = shelfT(c.shelf), b = cellBox(c.k), X = [S[0], S[4], S[8]], Y = [S[1], S[5], S[9]];
        ok = [0.06, -0.06].every(u => any([1, -1].map(sg => frame(X, Y.map(v => v * sg), apply(S, [FRONT_IN + 0.025, (b.lo[1] + b.hi[1]) / 2 + u, b.lo[2] + BOOK.h / 2])))));
      }
    } finally { Object.assign(P, save); this.mode = mode; this.setQ(q0); }
    return ok;
  }
  // 책을 들고 걸을 때: 손끝을 위로 세워 몸 위쪽에 (가운데에서 돌 때 책이 몸 반지름 안에 — 앞으로 내밀면 책꽂이를 쓴다).
  //   안 되면 몸 앞으로. (팔이 몸통 위에 있어 몸 가까이에서 손끝을 앞으로 하면 팔이 못 접는다)
  carryQ() {
    const B = this.baseT(), Rl = mul(inv(B), this.feat("pinch")), cand = [];
    for (const [f, up] of [[0.30, 0.50], [0.38, 0.45], [0.30, 0.60]]) for (const r of [[1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 1, 0, -1, 0, 0, 0, 0, 1]]) cand.push([f, up, r]);   // z = 위
    for (const [f, up] of [[0.60, 0.30], [0.55, 0.35]]) { cand.push([f, up, [0, 0, 1, 1, 0, 0, 0, 1, 0]], [f, up, [0, 0, 1, -1, 0, 0, 0, -1, 0]]); cand.push([f, up, [Rl[0], Rl[1], Rl[2], Rl[4], Rl[5], Rl[6], Rl[8], Rl[9], Rl[10]]]); }
    for (const [f, up, r] of cand) {
      const T = mul(B, [r[0], r[1], r[2], f, r[3], r[4], r[5], 0, r[6], r[7], r[8], up, 0, 0, 0, 1]);
      const q = this.solve("pinch", T); if (q) return q;
    }
    return ARM.map(n => READY[n]);
  }
  nearShelf() {                                      // 손이 책꽂이 앞면에서 25cm 안, 또는 정렬대에서 35cm 안에 있나
    const T = this.feat("pinch"), sb = standBox();
    if (T[3] > sb.lo[0] - 0.35 && T[3] < sb.hi[0] + 0.35 && T[7] > sb.lo[1] - 0.35 && T[7] < sb.hi[1] + 0.35 && T[11] < sb.hi[2] + 0.3) return true;
    return SHELVES.some(sh => { const L = mul(inv(shelfT(sh)), [1, 0, 0, T[3], 0, 1, 0, T[7], 0, 0, 1, T[11], 0, 0, 0, 1]); return L[3] > -0.25 && L[3] < 0.35 && Math.abs(L[7]) < 0.6; });
  }
  base(target, bk) {
    const w = this; let goal, pts, t = 0, stage = 0, n = 0;
    const armPose = () => this.held ? READY : STOW;
    let back = null, need = false;
    return [
      aChip(w, "물러나기"),
      // 책꽂이 바로 앞이면 팔은 그대로 두고 몸째 25cm 뒤로 (든 책이 책꽂이에서 곧게 빠져나온다) → 그다음 팔을 거둔다
      { init() {                                       // 뒤 공간만큼만 (몸이 다른 책꽂이·정렬대에 안 닿게)
          const P = w.walker.pose, c = Math.cos(P.yaw), s = Math.sin(P.yaw), obs = obstacles();
          const d = [0.25, 0.2, 0.15, 0.1, 0.05].find(d => segFree(P, { x: P.x - d * c, y: P.y - d * s }, P.yaw, obs)) ?? 0; back = { x: P.x - d * c, y: P.y - d * s, yaw: P.yaw };
          need = w.nearShelf(); },                     // 물러날지는 처음에 한 번만 (도중에 손이 멀어져도 끝까지 가서 선다 — 11-02 뒷걸음이 안 멈추던 것)
        update() { return need ? hdrive(w.walker, back) : true; } },
      aChip(w, "팔 거두기"), aJointTo(w, () => w.held ? w.carryQ() : ARM.map(n => STOW[n]), "걷기 전 팔 거두기"),
      aChip(w, "걷기"),
      // 몸 방향은 그대로 두고 미끄러지듯 (옆걸음) 옮기고, 방향 바꾸기는 둘레가 빈 곳에서만 (틀면서 걸으면 다리·팔이 책꽂이를 친다)
      // 설 자리 고르기: 후보를 좋은 순으로 — 몸이 닿지 않고 · 팔이 닿고 · 길이 있는 첫 것. 안 되면 무엇이 막았는지 말한다
      { init() {
          if (w.walker.reached) w.walker.target = { ...NORMAL };   // 물리 걷기: 걸을 땐 보통 높이로 (숙인 채 옆걸음은 두뇌가 아직 서툴다 — 팔은 이미 거뒀다)
          const where = target === "floor" ? (bk?.name ?? "바닥 책") : target === "stand" ? "정렬대" : target, obs = obstacles();
          let block = null, reachless = 0, routeless = 0;
          for (const c of w.standCands(target, bk)) {
            const bl = blockerAt(c.x, c.y, c.yaw, obs); if (bl) { block = block ?? bl; continue; }
            if (!w.armReach(c, target, bk)) { reachless++; continue; }
            const r = planRoute(w.walker.pose, c); if (r) { goal = c; pts = r; w.route = { pts: [{ ...w.walker.pose }, ...r], goal: c }; return; }
            if (++routeless >= 5) break;
          }
          if (routeless) return `Spot: ${where} 앞까지 갈 길이 없습니다 — 책꽂이 사이가 좁습니다 (Spot 은 폭 0.54m · 길이 1.27m)`;
          if (reachless) return `Spot: ${where} 앞에 설 수 있는 자리에서는 팔이 닿지 않습니다`;
          return `Spot: ${where} 앞 설 자리를 ${block ? iga(block.name) : "무언가가"} 막고 있습니다`;
        },
        update() { const g = pts[0]; if (!hdrive(w.walker, g)) return false; pts.shift(); return !pts.length; } },
      aChip(w, "몸 높이"), { init() { w.route = null; w.walker.target = { ...goal.body }; stage = 0; n = 0; },
        update() {
          if (!w.walker.reached) { const P = w.walker.pose; return Math.abs(P.z - goal.body.z) < 1e-3 && Math.abs(P.pitch - goal.body.pitch) < 1e-3; }
          // 물리 걷기(월드 레이첼): 몸 자세가 된 뒤 자리를 다시 맞춘다 — 숙이면 몸이 앞으로 밀려 팔 계산이 어긋난다 (11-02 실측 10cm). 8초 넘으면 그대로 간다
          if (stage === 0) { if (!w.walker.reached(goal.body) && ++n < 600) return false; stage = 1; n = 0; }   // 자세 기다리기도 10초까지
          if (++n > 480) { w.stopBase(); return true; }            // 8초 넘으면 그 자리에서 (마지막 맞추기 명령을 지운다 — 남겨 두면 팔로 일하는 동안 계속 걸었다)
          return w.walker.settleAt ? w.walker.settleAt(goal) : true;
        } },
    ];
  }
}
