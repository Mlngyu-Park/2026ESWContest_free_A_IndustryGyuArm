// DUO STUDIO — G1 (11-01): 공통 블록(scene.js)을 유니트리 G1 동작으로. G1 STUDIO world.js 에서 옮김.
//   최소한의 물리: 손·팔·든 물건이 바닥·도킹·물건·몸을 뚫지 않게 미리 계산해 보고 움직인다 (collide.js).
//     잡기 = 손을 세워 (손가락 앞, 검지 위) 물건 옆으로 내려와 손가락마다 '닿을 때까지만' 오므린다.
//     놓기 = 든 물건 밑면이 면에 닿는 자리까지 내려서 놓는다 (순간 이동 없음, 똑바로 선 채).
//     걷기 = 도킹·바닥 물건을 돌아서 간다.
import { Robot, Biped, Chain, drive, poseT, inv, mul, rpyT, clamp, wrap,
         R_ARM, L_ARM, R_HAND, L_HAND, ARM_REST, ARM_READY, HAND_OPEN, mirror, STAND_Z, HIP_W } from "./g1kin.js";
import { scan, gap } from "./collide.js";
import { axisT } from "./base.js";
import { Scene, DOCKS, DOCK_H, eul } from "./scene.js";

// 자세: 골반 높이 · 숙임 · 뒤로 빼기 · 허리 숙임
//   stance = 발 사이 반 · 발끝 벌림. 쪼그려 앉을 땐 발을 벌리고 발끝을 밖으로 (허벅지가 몸통에 안 닿게 — 11-01 부딪힘 계산)
const NARROW = { w: HIP_W, toe: 0 }, WIDE = { w: 0.27, toe: 0.6 };
export const STAND = { z: STAND_Z, pitch: 0, back: 0, waist: 0, stance: NARROW };
export const TABLE = { z: 0.68, pitch: 0.25, back: 0.045, waist: 0.2, stance: NARROW };
export const SQUAT = { z: 0.40, pitch: 0.8, back: 0.12, waist: 0.3, stance: WIDE, armsFwd: true };
export const ARM_SQUAT = [-1.0, -0.15, 0, 0.6, 0, 0, 0];      // 쪼그려 앉았을 때 쉬는 팔: 두 무릎 사이 앞으로 (바닥·허벅지에 안 닿게, 부딪힘 계산으로 고름)
export const REACH = { floor: { D: 0.34, side: 0.02 }, table: { D: 0.40, side: -0.16 } };   // 몸 자리 기준 물건 자리 (앞, 옆) — 바닥은 두 발 사이 앞
// 손 (오른손 손바닥 좌표): 손가락 = +x, 손바닥 안쪽(엄지·손가락이 감싸는 쪽) = +y, 검지 = +z
export const HAND_LOW = 0.044;                    // 손을 세웠을 때 손바닥 원점에서 손 가장 아래까지
export const GRIP = { cube: [0.062, 0.051], pen: [0.066, 0.046] };   // 물건 가운데가 오는 자리 (손바닥 x, y)
export const HAND_MAX = [0, -1.0, -1.7, 1.55, 1.7, 1.55, 1.7];      // 손가락을 이만큼까지 오므려 본다 (닿으면 멈춤)
const DIGITS = { thumb: [1, 2], middle: [3, 4], index: [5, 6] };
const DIGIT_LINKS = { thumb: ["right_hand_thumb_1_link", "right_hand_thumb_2_link"], middle: ["right_hand_middle_0_link", "right_hand_middle_1_link"],
                      index: ["right_hand_index_0_link", "right_hand_index_1_link"] };
export const ARM_LINKS = (s) => ["shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw", "hand_palm", "hand_thumb_0", "hand_thumb_1", "hand_thumb_2",
  "hand_middle_0", "hand_middle_1", "hand_index_0", "hand_index_1"].map(n => `${s}_${n}_link`);
export const LEG_LINKS = (s) => ["hip_roll", "hip_yaw", "knee", "ankle_roll"].map(n => `${s}_${n}_link`);
export const BODY_LINKS = ["pelvis", "torso_link", "head_link", ...LEG_LINKS("left"), ...LEG_LINKS("right")];
const HAND_SET = new Set(ARM_LINKS("right").slice(5));
// 오른팔 계산은 허리 돌리기·옆으로 기울이기까지 같이 (쪼그려 앉아 바닥에 손이 닿으려면 몸을 비틀어야 한다)
export const ARMX = ["waist_yaw_joint", "waist_roll_joint", ...R_ARM];
const X = (q7, wy = 0, wr = 0) => [wy, wr, ...q7];
export const ARM_TUCK = [-1.2, -0.4, 0, -0.8, 0, 0, 0];      // 손을 가슴 앞 위로 (쉬는 손이 탁자 높이라 바로 내밀면 탁자를 쓴다, 부딪힘 계산으로 고름)
const SEEDS = [X(ARM_READY), X(ARM_READY, 0.4, 0.3), X(ARM_READY, -0.4, 0.3), X([-1.2, -0.3, 0, 0.3, 0, 0.3, 0], 0.3, 0.4), X([-0.8, -0.2, 0.5, 1.0, 0.5, 0.5, 0], 0.6, 0.4)];

const smooth = (u) => u * u * (3 - 2 * u);
const lerp = (a, b, u) => a.map((v, i) => v + (b[i] - v) * u);
const palmT = (yaw, p, flip = false) => rpyT(flip ? Math.PI : 0, 0, yaw, p);   // 손을 세운 자세: 손가락 = yaw 방향 수평, 검지 = 위 (flip: 아래)
const raise = (T, dz) => { const M = T.slice(); M[11] += dz; return M; };
const NAMES = { "바닥": "바닥", "도킹": "도킹", "물건": "물건", "잡는 물건": "잡을 물건", "든 물건-몸": "몸", "몸끼리": "몸", "든 물건-바닥": "바닥",
                "든 물건-도킹": "도킹", "물건끼리": "다른 물건", "든 물건-몸속": "몸" };

export class G1World extends Scene {
  constructor(model, log = () => {}, shapes = null) {
    super();
    this.kind = "g1"; this.model = model; this.log = log; this.sh = shapes;
    this.cfg = { links: [...ARM_LINKS("right"), ...ARM_LINKS("left")], boxes: Scene.boxes(), hand: HAND_SET,
                 self: [[ARM_LINKS("right"), BODY_LINKS], [ARM_LINKS("left"), BODY_LINKS], [[...LEG_LINKS("left"), ...LEG_LINKS("right")], ["torso_link", "head_link"]]],
                 selfBody: BODY_LINKS };
    this.reset();
  }
  reset() {
    this.robot = new Robot(this.model);
    this.walker = new Biped(this.robot);
    this.arm = new Chain(this.robot, "pelvis", "right_hand_palm_link", [0, 0, 0], ARMX);
    this.carryArm = new Chain(this.robot, "pelvis", "right_hand_palm_link", [0, 0, 0], R_ARM);
    this.setArm("right", ARM_REST); this.setArm("left", ARM_REST);
    this.setHand("right", HAND_OPEN); this.setHand("left", HAND_OPEN);
    this.rArmBusy = false; this.lArmBusy = false; this.restK = 0; this.restKT = 0;
    this.resetScene();
  }
  setArm(side, v) { (side === "right" ? R_ARM : L_ARM).forEach((n, i) => this.robot.q[n] = side === "right" ? v[i] : mirror(n, v[i])); }
  getArm(side) { return (side === "right" ? R_ARM : L_ARM).map(n => side === "right" ? this.robot.q[n] : mirror(n, this.robot.q[n])); }
  setHand(side, v) { (side === "right" ? R_HAND : L_HAND).forEach((n, i) => this.robot.q[n] = side === "right" ? v[i] : mirror(n, v[i])); }
  getHand(side) { return (side === "right" ? R_HAND : L_HAND).map(n => side === "right" ? this.robot.q[n] : mirror(n, this.robot.q[n])); }
  getX() { return [this.robot.q.waist_yaw_joint, this.robot.q.waist_roll_joint, ...this.getArm("right")]; }   // 허리 2 + 오른팔 7
  setX(v) { this.robot.q.waist_yaw_joint = v[0]; this.robot.q.waist_roll_joint = v[1]; this.setArm("right", v.slice(2)); }
  // 쉬는 팔: 서 있을 땐 내려뜨리고, 쪼그려 앉을수록 앞으로 (손이 바닥·무릎에 안 닿게)
  //   (쪼그려 앉기 전에 팔을 먼저 앞으로 — 앉으면서 옮기면 손이 허벅지를 지나간다)
  restArm() { return lerp(ARM_REST, ARM_SQUAT, smooth(this.restK)); }
  //   물건이 오른손 앞(앞 D, 오른쪽 side)에 오도록 선다. 오는 쪽에서 바로 서는 자리가 첫 후보, 막히면 물건 둘레로 돌려 본다
  //   (선 자리·35cm 앞 자리가 다른 바닥 물건·도킹과 겹치면 뺀다 — 발로 물건을 밟지 않게). k = 몇 번째 후보
  standFor(id, from = this.walker.path, k = 0) {
    if (id === "home") return { x: 0, y: 0, yaw: 0 };
    const c = this.standCands(id, from); return c[Math.min(k, c.length - 1)];
  }
  standCands(id, from) {
    const [x, y, z] = this.where(id), R = z < 0.3 ? REACH.floor : REACH.table, d = Math.hypot(x - from.x, y - from.y);
    const base = Math.atan2(y - from.y, x - from.x) + Math.asin(clamp(-R.side / Math.max(d, 0.3), -0.5, 0.5));
    const at = (yaw) => { const c = Math.cos(yaw), s = Math.sin(yaw); return { x: x - (c * R.D - s * R.side), y: y - (s * R.D + c * R.side), yaw }; };
    const dockId = this.dock(id)?.id ?? this.item(id)?.on;
    const clear = (p) => this.items.every(it => it === this.held || it.id === id || this.dock(it.on) || Math.hypot(it.x - p.x, it.y - p.y) >= 0.42)
      && DOCKS.every(d => Math.hypot(d.x - p.x, d.y - p.y) >= (d.id === dockId ? 0.36 : 0.55));
    const out = [];
    for (const o of [0, 0.5, -0.5, 1.0, -1.0, 1.6, -1.6, 2.4, -2.4, Math.PI]) {
      const g = at(base + o), pre = { x: g.x - 0.35 * Math.cos(g.yaw), y: g.y - 0.35 * Math.sin(g.yaw) };
      if (clear(g) && clear(pre)) out.push(g);
    }
    return out.length ? out : [at(base)];
  }
  near(id) {
    const g = this.standFor(id), P = this.walker.path;
    return Math.hypot(g.x - P.x, g.y - P.y) < 0.06 && Math.abs(wrap(g.yaw - P.yaw)) < 0.1;
  }
  fk() { return this.robot.fk(poseT(this.walker.pelvis)); }
  palm(W = this.fk()) { return W.right_hand_palm_link; }
  // ── 부딪힘 ──
  //   지금 자세에서 팔·손·든 물건이 무엇을 뚫나 (grip = 손가락이 닿아도 되는 물건). 모양(shapes)이 없으면 검사 안 함.
  clash(grip = null, skip = null) {
    if (!this.sh) return null;
    const c = scan(this.sh, this.fk(), this.items, this.held, { ...this.cfg, grip, skipItems: skip });
    return c.length ? c.sort((a, b) => b.depth - a.depth)[0] : null;
  }
  clashText(c) { const w = c.who.split("|"); return `${NAMES[c.kind] ?? c.kind}${w[1] && !w[1].includes("_link") ? `(${w[1]})` : ""}에 부딪힘`; }
  // 걷기용 장애물 원: 도킹, 바닥 물건 — 가려는 대상의 도킹은 작게
  obstacles(targetId) {
    const tgtDock = this.dock(targetId)?.id ?? this.item(targetId)?.on;
    const out = DOCKS.map(d => ({ x: d.x, y: d.y, r: d.id === tgtDock ? 0.36 : 0.55 }));
    for (const it of this.items) if (it !== this.held && !this.dock(it.on) && it.id !== targetId) out.push({ x: it.x, y: it.y, r: 0.45 });
    return out;
  }
  route(goal, targetId, P = this.walker.path) {      // 걷는 길: 장애물 원을 피해 거쳐 갈 점들 + 마지막 자리
    const obs = this.obstacles(targetId), pts = [{ x: P.x, y: P.y }, goal];
    for (let n = 0; n < 12; n++) {
      let hit = null;
      for (let i = 0; i + 1 < pts.length && !hit; i++) {
        const a = pts[i], b = pts[i + 1], dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy || 1e-9;
        for (const o of obs) {
          if (Math.hypot(a.x - o.x, a.y - o.y) < o.r - 1e-3 || Math.hypot(b.x - o.x, b.y - o.y) < o.r - 1e-3) continue;
          const u = clamp(((o.x - a.x) * dx + (o.y - a.y) * dy) / L2, 0, 1), cx = a.x + u * dx, cy = a.y + u * dy, d = Math.hypot(cx - o.x, cy - o.y);
          if (d < o.r && u > 0 && u < 1) { hit = { i, o, cx, cy, d, dx, dy }; break; }
        }
      }
      if (!hit) break;
      const { i, o, cx, cy, d, dx, dy } = hit, L = Math.hypot(dx, dy);
      let nx = cx - o.x, ny = cy - o.y; if (d < 1e-6) { nx = -dy / L; ny = dx / L; } else { nx /= d; ny /= d; }
      pts.splice(i + 1, 0, { x: o.x + nx * (o.r + 0.12), y: o.y + ny * (o.r + 0.12) });
    }
    return pts.slice(1);
  }
  // ── 한 프레임 ──
  step(dt) {
    this.t += dt;
    if (this.exec) this.exec.update(dt);
    this.walker.update(dt);
    this.restK += clamp(this.restKT - this.restK, -dt / 0.7, dt / 0.7);
    // 걸을 때 쉬는 팔은 다리와 반대로 흔든다 (앞으로 걸을 때만 — 제자리 돌기에선 안 흔든다)
    const w = this.walker, k = clamp(Math.abs(w.vel.vx) / w.vmax, 0, 1);
    const sw = w.walking ? 0.22 * k * Math.sin(Math.PI * w.s) * (w.swing === "left" ? 1 : -1) : 0, rest = this.restArm();
    if (!this.rArmBusy) this.setX(X(rest.map((v, i) => i === 0 ? v + sw : v)));
    if (!this.lArmBusy) this.setArm("left", rest.map((v, i) => i === 0 ? v - sw : v));
    if (this.held) this.held.T = mul(this.palm(), this.held.rel);
  }
  robotCompile(b) { return compile(this, b); }
  get pose() { return this.walker.pelvis; }
}

// ── 팔 계산 ──────────────────────────────────────────────────
//   손바닥 목표 T(월드)를 여러 출발점에서 풀어 가장 좋은 답. 못 풀면 null.
function solveArm(w, T, seeds) {
  const Bi = inv(poseT(w.walker.pelvis)), L = mul(Bi, T), q0 = w.getX();
  const ax = [[0, [L[0], L[4], L[8]]], [2, [L[2], L[6], L[10]]]], tp = [L[3], L[7], L[11]];
  const ok = (b) => b && b.perr < 0.002 && b.oerr < 0.01;      // 손 기울기 0.6° 안 (손끝이 바닥으로 안 처지게)
  let best = null;
  for (const chain of [w.carryArm, w.arm]) {           // 허리는 팔만으로 안 될 때만 쓴다
    for (const s of seeds) {
      w.setX(chain === w.arm ? s : [q0[0], q0[1], ...s.slice(2)]);
      const r = chain.solve(tp, ax, 300), e = r.perr + 0.05 * r.oerr;
      if (!best || e < best.e) best = { e, perr: r.perr, oerr: r.oerr, q: w.getX() };
    }
    if (ok(best)) break;
  }
  w.setX(q0);
  return ok(best) ? best.q : null;
}
function lineQ(w, qa, Ta, Tb, n) {                   // 손바닥을 Ta→Tb 직선으로 — 관절 값 n+1개 (qa 에서 이어 풀기)
  const Bi = inv(poseT(w.walker.pelvis)), q0 = w.getX(), out = [qa]; let ok = true;
  const chain = Math.abs(qa[0] - q0[0]) + Math.abs(qa[1] - q0[1]) < 1e-9 ? w.carryArm : w.arm;   // 허리를 안 쓴 답이면 계속 안 쓴다
  w.setX(qa);
  for (let k = 1; k <= n; k++) {
    const u = k / n, T = Ta.map((v, i) => v + (Tb[i] - v) * u), L = mul(Bi, T);
    const r = chain.solve([L[3], L[7], L[11]], [[0, [L[0], L[4], L[8]]], [2, [L[2], L[6], L[10]]]], 60);
    if (r.perr > 0.002 || r.oerr > 0.01) ok = false;
    out.push(w.getX());
  }
  w.setX(q0);
  return ok ? out : null;
}
// 관절 값들을 차례로 놓아 보며 (사이도 촘촘히) 부딪힘 검사 → 처음 부딪힘 또는 null
function pathClash(w, qs, grip, sub = 3) {
  const q0 = w.getX(); let hit = null;
  for (let i = 0; i < qs.length && !hit; i++) for (let k = i ? 1 : 0; k <= (i ? sub : 0) && !hit; k++) {
    w.setX(i ? lerp(qs[i - 1], qs[i], k / sub) : qs[0]);
    if (w.held) w.held.T = mul(w.palm(), w.held.rel);
    hit = w.clash(grip);
  }
  w.setX(q0); if (w.held) w.held.T = mul(w.palm(), w.held.rel);
  return hit;
}
const jointPath = (a, b, n = 8) => [...Array(n + 1)].map((_, k) => lerp(a, b, k / n));
// 손바닥을 Ta→Tb 로 공간에서 곧게 (위치는 직선, 방향은 고르게 돌림) — 관절 값 n+1개, 못 따라가면 null
function slerpT(Ta, Tb, u) {
  const Ra = [Ta[0], Ta[1], Ta[2], Ta[4], Ta[5], Ta[6], Ta[8], Ta[9], Ta[10]], Rb = [Tb[0], Tb[1], Tb[2], Tb[4], Tb[5], Tb[6], Tb[8], Tb[9], Tb[10]];
  const r = (i, j) => Ra[i] * Rb[j] + Ra[3 + i] * Rb[3 + j] + Ra[6 + i] * Rb[6 + j];   // (Raᵀ Rb)ij
  const tr = r(0, 0) + r(1, 1) + r(2, 2), th = Math.acos(clamp((tr - 1) / 2, -1, 1));
  let ax = [r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1)];
  if (Math.hypot(...ax) < 1e-9) ax = [0, 0, 1];
  const M = mul(Ta, axisT(ax, th * u));
  M[3] = Ta[3] + (Tb[3] - Ta[3]) * u; M[7] = Ta[7] + (Tb[7] - Ta[7]) * u; M[11] = Ta[11] + (Tb[11] - Ta[11]) * u;
  return M;
}
function cartQ(w, qa, Ta, Tb, n) {
  const Bi = inv(poseT(w.walker.pelvis)), q0 = w.getX(), out = [qa]; let ok = true;
  w.setX(qa);
  for (let k = 1; k <= n; k++) {
    const L = mul(Bi, slerpT(Ta, Tb, k / n));
    const r = w.carryArm.solve([L[3], L[7], L[11]], [[0, [L[0], L[4], L[8]]], [2, [L[2], L[6], L[10]]]], 60);
    if (r.perr > 0.004 || r.oerr > 0.03) ok = false;
    out.push(w.getX());
  }
  w.setX(q0);
  return ok ? out : null;
}
// 지금 팔 → 목표(관절 q1, 손바닥 T1) 가는 길 후보: ① 곧장 ② 팔꿈치 접어 가슴 앞 → 곧장
//   ③ 가슴 앞 → 손을 공간에서 곧게 목표 위 10cm → 내려서 목표. 부딪힘 없는 첫 길 또는 {hit}
function approach(w, q1, grip, T1) {
  const q0 = w.getX(), tuck = X(ARM_TUCK, q0[0], q0[1]);
  const cands = [() => jointPath(q0, q1), () => [...jointPath(q0, tuck), ...jointPath(tuck, q1).slice(1)]];
  if (T1 && Math.abs(q1[0]) + Math.abs(q1[1]) < 1e-9) cands.push(() => {
    const qt = w.getX(); w.setX(tuck); const Tt = w.palm(); w.setX(qt);
    const Tu = raise(T1, 0.10), a = cartQ(w, tuck, Tt, Tu, 12); if (!a) return null;
    const b = cartQ(w, a[a.length - 1], Tu, T1, 6); if (!b) return null;
    return [...jointPath(q0, tuck), ...a.slice(1), ...b.slice(1)];
  });
  let hit = null;
  for (const c of cands) { const qs = c(); if (!qs) continue; const h = pathClash(w, qs, grip); if (!h) return { qs }; hit = hit ?? h; }
  return { hit: hit ?? { kind: "닿지 않음", who: "", depth: 0 } };
}

// ── 손가락 ──
function handAt(us) { const H = HAND_OPEN.slice(); for (const [d, ix] of Object.entries(DIGITS)) for (const i of ix) H[i] = HAND_OPEN[i] + (HAND_MAX[i] - HAND_OPEN[i]) * us[d]; return H; }
//   손가락마다 물건에 처음 닿는 만큼 (0 = 편 손 … 1 = 끝까지). 안 닿으면 null.
export function contactU(w, it) {
  const h0 = w.getHand("right"), out = {};
  const touch = (d, u) => { w.setHand("right", handAt({ thumb: 0, middle: 0, index: 0, [d]: u })); return gap(w.sh, w.fk(), DIGIT_LINKS[d], it) <= 0.0005; };
  for (const d of Object.keys(DIGITS)) {
    let prev = 0, hit = null;
    for (let u = 0.02; u <= 1.0001 && hit === null; u += 0.02) { if (touch(d, u)) hit = u; else prev = u; }
    if (hit !== null) { let a = prev, b = hit; for (let k = 0; k < 8; k++) { const m = (a + b) / 2; if (touch(d, m)) b = m; else a = m; } hit = b; }
    out[d] = hit;
  }
  w.setHand("right", h0);
  return out;
}

// ── 동작 ────────────────────────────────────────────────────
//   도킹 곁(손이 탁자 높이라 돌면 탁자를 쓴다)에서는 먼저 뒤로 물러나고, 다가갈 땐 앞 35cm 에서 방향을 맞춘 뒤 똑바로 들어간다
function aWalk(w, goalFn, targetId, toPre = false) {
  let pts;
  return { init() {
             const g = goalFn(), P = w.walker.path, c = Math.cos(P.yaw), s = Math.sin(P.yaw);
             if (!g) { pts = []; return; }
             pts = [];
             if (DOCKS.some(d => Math.hypot(d.x - P.x, d.y - P.y) < 0.7)) pts.push({ x: P.x - 0.28 * c, y: P.y - 0.28 * s, yaw: P.yaw, back: true });
             if (!targetId) { pts.push(g); return; }
             const pre = { x: g.x - 0.35 * Math.cos(g.yaw), y: g.y - 0.35 * Math.sin(g.yaw), yaw: g.yaw };
             const from = pts[0] ?? P, far = Math.hypot(g.x - from.x, g.y - from.y) > 0.5;
             if (far || toPre) pts.push(...w.route(pre, targetId, from));   // 끝 = pre (방향까지 맞춤)
             if (!toPre) pts.push(g);
           },
           update() {
             if (!pts.length) return true;
             const g = pts[0];
             if (pts.length > 1 && g.yaw === undefined) { const P = w.walker.path; if (Math.hypot(g.x - P.x, g.y - P.y) < 0.15) { pts.shift(); return false; } drive(w.walker, { x: g.x, y: g.y }); return false; }
             if (pts.length > 1) { if (drive(w.walker, g)) pts.shift(); return false; }
             if (drive(w.walker, g)) { pts.shift(); return true; } return false;
           } };
}
//   앉을 땐 (팔 앞으로 →) 발 벌려 디딤 → 내려가기, 일어설 땐 일어서기 → 발 모음 → (팔 내림)
//   hold: 오른손을 월드에 둔 채 몸 높이만큼만 같이 오르내린다 (물건을 든 채 앉고 일어설 때 — 팔이 허벅지를 쓸지 않게)
function aPost(w, p, hold = false) {
  const B = w.walker, same = (a, b) => Math.abs(a.w - b.w) < 1e-6 && Math.abs(a.toe - b.toe) < 1e-6;
  const at = () => ["z", "pitch", "back", "waist"].every(k => Math.abs(B.post[k] - p[k]) < 1e-3);
  const body = { z: p.z, pitch: p.pitch, back: p.back, waist: p.waist }, armK = p.armsFwd ? 1 : 0;
  let steps, T0, z0;
  const arms = () => { w.restKT = armK; return Math.abs(w.restK - armK) < 1e-3; };   // 쉬는 팔(들고 있으면 왼팔만)
  const stance = () => { if (!same(B.stance, p.stance)) B.stance = { ...p.stance }; return !B.walking && B.settled(); };
  const lower = () => { B.target = body; return at() && !B.walking; };
  return {
    init() {
      const down = p.z < B.post.z - 1e-3;
      steps = down ? [arms, stance, lower] : [lower, stance, arms];
      if (hold) { T0 = w.palm(); z0 = B.post.z; w.rArmBusy = true; }
    },
    update() {
      if (hold) {                                      // 손 높이 = 처음 높이 + 몸이 오르내린 만큼
        const T = T0.slice(); T[11] += B.post.z - z0;
        const L = mul(inv(poseT(B.pelvisPose())), T);
        w.carryArm.solve([L[3], L[7], L[11]], [[0, [L[0], L[4], L[8]]], [2, [L[2], L[6], L[10]]]], 40);   // 허리는 그대로 (비틀면 왼팔이 허벅지로)
      }
      while (steps.length && steps[0]()) steps.shift();
      return steps.length ? false : (B.legOk ? true : "다리가 그 자세를 못 합니다");
    },
  };
}
function aArmJ(w, side, targetFn, dur) {             // 오른쪽은 허리 2 + 팔 7, 왼쪽은 팔 7
  let a, b, t = 0; const R = side === "right";
  return { init() { a = R ? w.getX() : w.getArm(side); b = typeof targetFn === "function" ? targetFn() : targetFn; if (R) w.rArmBusy = true; else w.lArmBusy = true; },
           update(dt) { t += dt; const v = lerp(a, b, smooth(Math.min(1, t / dur))); if (R) w.setX(v); else w.setArm(side, v); return t >= dur; } };
}
function aArmPath(w, qsFn, dur) {                    // 미리 검사한 관절 값들을 따라 (부드럽게)
  let qs, t = 0;
  return { init() { qs = qsFn(); w.rArmBusy = true; },
           update(dt) { t += dt; const u = smooth(Math.min(1, t / dur)) * (qs.length - 1), i = Math.min(qs.length - 2, Math.floor(u));
                        w.setX(lerp(qs[i], qs[i + 1], u - i)); return t >= dur; } };
}
function aHand(w, side, target, dur) {
  let a, t = 0;
  return { init() { a = w.getHand(side); },
           update(dt) { t += dt; w.setHand(side, lerp(a, target, smooth(Math.min(1, t / dur)))); return t >= dur; } };
}
function aDo(fn) { return { update: () => fn() ?? true }; }
function aWait(s) { let t = 0; return { update(dt) { t += dt; return t >= s; } }; }
function aWave(w) {                                  // 왼손 들어 흔들기 (쉬는 자세에서 올렸다가 쉬는 자세로)
  const up = [-2.6, -0.5, 0, 0.6, 0, 0, 0];         // 팔을 바깥으로 벌려 올림 (안쪽으로 올리면 팔꿈치가 머리에 닿는다)
  let t = 0, r0;
  return { init() { w.lArmBusy = true; r0 = w.getArm("left"); },
           update(dt) {
             t += dt; const a = Math.min(1, t / 0.6), b = Math.max(0, Math.min(1, (t - 2.2) / 0.6));
             const v = lerp(lerp(r0, up, smooth(a)), w.restArm(), smooth(b));
             if (t > 0.6 && t < 2.2) v[2] += 0.45 * Math.sin((t - 0.6) * 9);
             w.setArm("left", v);
             if (t >= 2.8) { w.lArmBusy = false; return true; } return false;
           } };
}

// 잡기 계획: 손을 세워 물건 옆에 — 손가락 방향 후보를 돌려 보며, 팔이 닿고 가는 길·잡는 자리에서 아무것도 안 뚫는 것
function planGrab(w, it, ctx) {
  const P = w.walker.path, surf = w.surface(it.on), [gx, gy] = GRIP[it.kind];
  const hp = Math.max(surf + HAND_LOW + 0.008, it.z);
  let yaws;
  if (it.kind === "cube") yaws = [0, 1, 2, 3].map(k => it.yaw + k * Math.PI / 2).filter(y => Math.abs(wrap(y - P.yaw)) < 1.2);
  else yaws = [0, -0.35, 0.35, -0.7, 0.7].map(d => P.yaw + d);
  yaws.sort((a, b) => Math.abs(wrap(a - P.yaw)) - Math.abs(wrap(b - P.yaw)));
  let why = `손이 ${it.name}에 닿지 않습니다`;
  for (const yaw of yaws) for (const flip of [false, true]) {
    const T = palmT(yaw, [0, 0, 0], flip);           // 손바닥 원점 = 물건 가운데 − (x축·gx + y축·gy), 높이 hp
    T[3] = it.x - (T[0] * gx + T[1] * gy); T[7] = it.y - (T[4] * gx + T[5] * gy); T[11] = hp;
    const Tp = raise(T, 0.12);
    const qp = solveArm(w, Tp, [w.getX(), ...SEEDS]); if (!qp) continue;
    const down = lineQ(w, qp, Tp, T, 10); if (!down) continue;
    const a = approach(w, qp, it, Tp), hit = a.hit ?? pathClash(w, down, it);
    if (hit) { why = `${it.name}에 손을 넣으면 ${w.clashText(hit)}`; continue; }
    Object.assign(ctx, { to: a.qs, down, up: down.slice().reverse(), T });
    return null;
  }
  return why;
}
function planPlace(w, place, ctx) {
  const it = w.held, P = w.walker.path, [x, y, s] = w.where(place);
  const phi = Math.atan2(it.rel[4], it.rel[0]), Ri = inv(it.rel);
  let why = "손이 놓을 자리에 닿지 않습니다";
  for (const d of [0, -0.35, 0.35, -0.7, 0.7]) {
    const Ti = rpyT(0, 0, P.yaw + d + phi, [x, y, s + w.half(it) + 0.001]), T = mul(Ti, Ri), Tp = raise(T, 0.12);
    const qp = solveArm(w, Tp, [w.getX(), ...SEEDS]); if (!qp) continue;
    const down = lineQ(w, qp, Tp, T, 10); if (!down) continue;
    const a = approach(w, qp, null, Tp); let hit = a.hit ?? pathClash(w, down, null);
    if (hit) { why = `놓으러 가면 ${w.clashText(hit)}`; continue; }
    // 놓은 뒤 (손 편 채) 위로 → 팔꿈치 접기 → 쉬는 팔: 놓인 물건·탁자를 안 치나 미리 본다
    const up = down.slice().reverse(), e = up[up.length - 1], tuck = X(ARM_TUCK, e[0], e[1]);
    const back = [...up, ...jointPath(e, tuck).slice(1)];   // 쉬는 팔로는 일어선 뒤에 (쪼그린 채 내리면 손이 바닥 물건 자리로 간다)
    const h0 = w.getHand("right"), T0 = it.T; w.held = null; it.T = Ti; w.setHand("right", HAND_OPEN);
    hit = pathClash(w, back.slice(1), null);
    w.held = it; it.T = T0; w.setHand("right", h0);
    if (hit) { why = `놓고 손을 빼면 ${w.clashText(hit)}`; continue; }
    Object.assign(ctx, { to: a.qs, down, back, T });
    return null;
  }
  return why;
}
export const CARRY = { f: 0.32, sd: -0.16 };          // 들 때 손바닥 자리 (골반 기준 앞·옆, 부딪힘 계산으로 고름)
// 들고 다니는 자세: 허리는 바로, 손은 골반 앞 오른쪽에 세운 채 (물건 똑바로 — 잡을 때 손이 뒤집혔으면 뒤집힌 채)
function carryQ(w) {
  const P = w.walker.pelvis, c = Math.cos(P.yaw), s = Math.sin(P.yaw), { f, sd } = CARRY, flip = w.palm()[10] < 0;
  const T = palmT(P.yaw, [P.x + c * f - s * sd, P.y + s * f + c * sd, P.z + 0.10], flip), L = mul(inv(poseT(P)), T);
  const x0 = w.getX(); let best = null;
  for (const sd7 of [w.getArm("right"), ARM_READY]) {
    w.setX(X(sd7));
    const r = w.carryArm.solve([L[3], L[7], L[11]], [[0, [L[0], L[4], L[8]]], [2, [L[2], L[6], L[10]]]], 300);
    if (!best || r.perr + 0.05 * r.oerr < best.e) best = { e: r.perr + 0.05 * r.oerr, q: w.getX() };
  }
  w.setX(x0);
  return best.q;
}
function aCheck(w, what) {                           // 지금 자세가 아무것도 안 뚫나
  return { update() { const h = w.clash(); return h ? `${what}: ${w.clashText(h)}` : true; } };
}
function aGrip(w, it) {                              // 손가락마다 닿을 때까지만 오므린다
  let a, b, t = 0;
  return { init() {
             if (!w.sh) { a = w.getHand("right"); b = handAt({ thumb: 0.5, middle: 0.5, index: 0.5 }); return; }
             const u = contactU(w, it);
             if (u.thumb === null || (u.middle === null && u.index === null)) return `${eul(it.name)} 쥘 수 없습니다 (손가락이 닿지 않음)`;
             a = w.getHand("right"); b = handAt({ thumb: u.thumb, middle: u.middle ?? 0.6, index: u.index ?? 0.6 });
           },
           update(dt) { t += dt; w.setHand("right", lerp(a, b, smooth(Math.min(1, t / 0.5)))); return t >= 0.5; } };
}
function aAttach(w, it) { return { update() { it.rel = mul(inv(w.palm()), it.T); w.held = it; it.on = "hand"; return true; } }; }
function aDetach(w, it, place) {
  return { update() {
    const T = mul(w.palm(), it.rel); w.held = null; it.rel = null; it.T = T; it.yaw = Math.atan2(T[4], T[0]);
    w.settle(it, place, T[3], T[7]);
    w.log(`${it.name} → ${w.dock(place)?.name ?? w.mark(place)?.name}`, "ok");
    return true;
  } };
}

// 도킹(탁자) 앞으로: 35cm 앞에서 (오른손을 가슴 앞으로 올리고) 똑바로 걸어 들어간다
//   G1 은 팔이 짧아 탁자 가까이 서야 하고, 쉬는 손이 탁자 높이라 가까이 선 뒤에 손을 들면 탁자 모서리를 쓴다
function walkIn(w, id, tuck, k = 0, hold = false) {
  let g = null; const goal = () => (g = g ?? w.standFor(id, w.walker.path, k));
  return [aPost(w, STAND, hold), aWalk(w, goal, id, true), ...(tuck ? [aArmJ(w, "right", () => X(ARM_TUCK), 0.9)] : []), aWalk(w, () => goal(), null)];
}
// 계획이 막히면 서는 자리를 바꿔 다시 (최대 3번) — 옮기는 동작들을 바로 다음에 끼워 넣는다
function aPlan(w, planFn, again) {
  return { update() {
    const why = planFn(); if (!why) return true;
    if (again.k >= 3) return why;
    again.k++; w.exec.acts.splice(1, 0, ...again.acts(again.k), aPlan(w, planFn, again));
    return true;
  } };
}
// 도킹 곁에서 한 걸음 물러나기 (팔을 내리기 전에)
function aBackoff(w) {
  return aWalk(w, () => { const P = w.walker.path; return DOCKS.some(d => Math.hypot(d.x - P.x, d.y - P.y) < 0.7) ? { x: P.x - 0.3 * Math.cos(P.yaw), y: P.y - 0.3 * Math.sin(P.yaw), yaw: P.yaw } : null; }, null);
}
const tucked = (w) => w.rArmBusy && w.getX().every((v, i) => Math.abs(v - X(ARM_TUCK)[i]) < 1e-3);
function compile(w, b) {
  switch (b.type) {
    case "walk": return [aPost(w, STAND), aWalk(w, () => w.standFor(b.arg), b.arg)];
    case "turn": return [aWalk(w, () => { const P = w.walker.path; return { x: P.x, y: P.y, yaw: P.yaw + (+b.arg) * Math.PI / 180 }; })];
    case "body": return [aPost(w, b.arg === "low" ? SQUAT : STAND)];
    case "rest": return w.rArmBusy ? [aPost(w, STAND), aBackoff(w), aArmJ(w, "right", () => X(w.restArm()), 0.8), aDo(() => { w.rArmBusy = false; })] : [];
    case "wait": return [aWait(+b.arg || 1)];
    case "wave": return [aWave(w)];
    case "grab": {
      const it = w.item(b.arg), low = it.z < 0.3, ctx = {};
      const acts = [];
      if (!low) { if (!(w.near(it.id) && tucked(w))) acts.push(...walkIn(w, it.id, true)); }
      else if (!w.near(it.id)) acts.push(aPost(w, STAND), aWalk(w, () => w.standFor(it.id), it.id));
      acts.push(aPost(w, low ? SQUAT : TABLE), aHand(w, "right", HAND_OPEN, 0.3),
        aPlan(w, () => planGrab(w, it, ctx), { k: 0, acts: (k) => low
          ? [aPost(w, STAND), aWalk(w, () => w.standFor(it.id, w.walker.path, k), it.id), aPost(w, SQUAT)]
          : [...walkIn(w, it.id, true, k), aPost(w, TABLE)] }),
        aArmPath(w, () => ctx.to, 1.4),
        aArmPath(w, () => ctx.down, 1.0), aGrip(w, it), aAttach(w, it),
        aArmPath(w, () => ctx.up, 0.8), aPost(w, STAND, true),
        aArmJ(w, "right", () => carryQ(w), 0.8), aCheck(w, "들고 일어서기"));
      return acts;
    }
    case "place": {
      const it = w.held;
      const low = w.where(b.arg)[2] < 0.3, ctx = {};
      const acts = [];
      if (!w.near(b.arg)) acts.push(...(low ? [aPost(w, STAND), aWalk(w, () => w.standFor(b.arg), b.arg)] : walkIn(w, b.arg, false)));
      acts.push(aPost(w, low ? SQUAT : TABLE, true),
        aPlan(w, () => planPlace(w, b.arg, ctx), { k: 0, acts: (k) => low
          ? [aPost(w, STAND, true), aWalk(w, () => w.standFor(b.arg, w.walker.path, k), b.arg), aPost(w, SQUAT, true)]
          : [...walkIn(w, b.arg, false, k, true), aPost(w, TABLE, true)] }),
        aArmPath(w, () => ctx.to, 1.4),
        aArmPath(w, () => ctx.down, 1.0), aHand(w, "right", HAND_OPEN, 0.4), aDetach(w, it, b.arg),
        aArmPath(w, () => ctx.back, 1.4), aPost(w, STAND), aBackoff(w), aArmJ(w, "right", () => X(w.restArm()), 0.8), aDo(() => { w.rArmBusy = false; }));
      return acts;
    }
  }
  return [];
}
export const _inner = { planGrab, planPlace, solveArm, lineQ, pathClash, carryQ, approach, jointPath, X, palmT, raise };   // 검사용
