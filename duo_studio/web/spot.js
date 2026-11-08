// DUO STUDIO — Spot (11-01): 공통 블록(scene.js)을 Spot 동작으로. SPOT STUDIO world.js 에서 옮김.
//   최소한의 물리: 팔·집게·든 물건이 바닥·도킹·물건·몸을 뚫지 않게 미리 계산해 보고 움직인다 (collide.js).
//     잡기 = 물건을 고정 턱에 붙이고 손가락을 '닿을 때까지만' 내린다. 집게 끝은 바닥·도킹보다 5mm 위.
//     놓기 = 든 물건 밑면이 면에 닿는 자리까지 내려서 놓는다. 걷기 = 바닥 물건·도킹을 돌아서, 물건을 밟지 않는 자리에 선다.
import { Robot, Walker, ArmIK, drive, poseT, apply, inv, mul, rpyT, clamp,
         ARM, READY, STOW, GRIP_OPEN, GRIP_CLOSED } from "./spotkin.js";
import { scan, gap } from "./collide.js";
import { Scene, DOCKS, DOCK_H, eul } from "./scene.js";

const NORMAL = { z: 0.52, pitch: 0 }, LOW = { z: 0.40, pitch: 0.15 };
// 집게 (손목 wr1 좌표): x = 손 끝 방향, 손가락은 z 쪽에서 내려와 고정 턱(z = JAW_Z 면)에 물건을 누른다
export const JAW_Z = -0.032, TIP_X = 0.247, BODY_X = 0.145, MARGIN = 0.005;   // 턱 면 · 집게 끝 · 손목 몸통 끝 (make_shapes 모양으로 잰 값)
const ARM_LINKS = ["arm_link_el0", "arm_link_el1", "arm_link_wr0", "arm_link_wr1", "arm_link_fngr"];
const LEG_LINKS = ["front_left", "front_right", "rear_left", "rear_right"].flatMap(l => [`${l}_upper_leg`, `${l}_lower_leg`]);
const HAND = new Set(["arm_link_wr1", "arm_link_fngr"]);
const smooth = (u) => u * u * (3 - 2 * u);
const lerp = (a, b, u) => a.map((v, i) => v + (b[i] - v) * u);
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const NAMES = { "바닥": "바닥", "도킹": "도킹", "물건": "물건", "잡는 물건": "잡을 물건", "든 물건-몸": "몸", "몸끼리": "몸", "든 물건-바닥": "바닥",
                "든 물건-도킹": "도킹", "물건끼리": "다른 물건", "든 물건-몸속": "몸" };
const raise = (T, dz) => { const M = T.slice(); M[11] += dz; return M; };
const qv = (o) => ARM.map(n => o[n]);

export class SpotWorld extends Scene {
  constructor(model, log = () => {}, shapes = null) {
    super();
    this.kind = "spot"; this.model = model; this.log = log; this.sh = shapes;
    this.cfg = { links: [...ARM_LINKS, "body"], boxes: Scene.boxes(), hand: HAND, self: [[ARM_LINKS, ["body", ...LEG_LINKS]]], selfBody: ["body", ...LEG_LINKS] };
    this.reset();
  }
  reset() {
    this.robot = new Robot(this.model);
    this.walker = new Walker(this.robot);
    this.arm = new ArmIK(this.robot);
    for (const n of ARM) this.robot.q[n] = STOW[n];
    this.robot.q.arm_f1x = GRIP_CLOSED;
    this.resetScene();
  }
  getQ() { return ARM.map(n => this.robot.q[n]); }
  setQ(q) { ARM.forEach((n, i) => this.robot.q[n] = q[i]); }
  //   대상 앞에 서는 자리: 지금 오는 쪽에서 바로가 첫 후보, 다른 바닥 물건·도킹과 겹치면 대상 둘레로 돌려 본다 (발로 물건을 차지 않게)
  standFor(id, from = this.walker.pose, k = 0) {
    if (id === "home") return { x: 0, y: 0, yaw: 0 };
    const c = this.standCands(id, from); return c[Math.min(k, c.length - 1)];
  }
  standCands(id, from) {
    const [x, y, z] = this.where(id), D = z < 0.15 ? 0.72 : 0.78, base = Math.atan2(y - from.y, x - from.x);
    const dockId = this.dock(id)?.id ?? this.item(id)?.on;
    const clear = (p) => this.items.every(it => it === this.held || it.id === id || this.dock(it.on) || (this.mark(id) && it.on === id) || Math.hypot(it.x - p.x, it.y - p.y) >= 0.62)
      && DOCKS.every(d => d.id === dockId || Math.hypot(d.x - p.x, d.y - p.y) >= 0.62);
    const out = [];
    for (const o of [0, 0.5, -0.5, 1.0, -1.0, 1.6, -1.6, 2.4, -2.4, Math.PI]) {
      const yaw = base + o, g = { x: x - D * Math.cos(yaw), y: y - D * Math.sin(yaw), yaw };
      if (clear(g)) out.push(g);
    }
    return out.length ? out : [{ x: x - D * Math.cos(base), y: y - D * Math.sin(base), yaw: base }];
  }
  near(id) {                                        // 이미 잡을 만한 자리에 서 있나
    const [x, y] = this.where(id), P = this.walker.pose;
    const d = Math.hypot(x - P.x, y - P.y), head = wrap(Math.atan2(y - P.y, x - P.x) - P.yaw);
    return d > 0.55 && d < 0.92 && Math.abs(head) < 0.3;
  }
  // 걷기용 장애물 원: 도킹, 바닥 물건 (가려는 대상은 빼고)
  obstacles(targetId) {
    const tgtDock = this.dock(targetId)?.id ?? this.item(targetId)?.on;
    const out = DOCKS.filter(d => d.id !== tgtDock).map(d => ({ x: d.x, y: d.y, r: 0.62 }));
    for (const it of this.items) if (it !== this.held && !this.dock(it.on) && it.id !== targetId && !(this.mark(targetId) && it.on === targetId)) out.push({ x: it.x, y: it.y, r: 0.62 });
    return out;
  }
  route(goal, targetId, P = this.walker.pose) {      // 장애물 원을 피해 거쳐 갈 점들 + 마지막 자리
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
  // ── 손 ──
  fk() { return this.robot.fk(poseT(this.walker.pose)); }
  wrist(W = this.fk()) { return W.arm_link_wr1; }
  clash(grip = null) {
    if (!this.sh) return null;
    const c = scan(this.sh, this.fk(), this.items, this.held, { ...this.cfg, grip });
    return c.length ? c.sort((a, b) => b.depth - a.depth)[0] : null;
  }
  clashText(c) { const w = c.who.split("|"); return `${NAMES[c.kind] ?? c.kind}${w[1] && !/_link|_leg|^body$/.test(w[1]) ? `(${w[1]})` : ""}에 부딪힘`; }
  // ── 한 프레임 ──
  step(dt) {
    this.t += dt;
    if (this.exec) this.exec.update(dt);
    this.walker.update(dt);
    if (this.held) this.held.T = mul(this.wrist(), this.held.rel);
  }
  robotCompile(b) { return compile(this, b); }
  get pose() { return this.walker.pose; }
}

// ── 팔 계산 ──────────────────────────────────────────────────
//   손목 목표 T(월드) → 관절 (여러 출발점), 못 풀면 null
function solveArm(w, T, seeds) {
  const Bi = inv(poseT(w.walker.pose)), L = mul(Bi, T);
  const tx = [L[0], L[4], L[8]], ty = [L[1], L[5], L[9]], tp = [L[3], L[7], L[11]];
  let best = null;
  for (const s of seeds) {
    // ArmIK 은 잡는 점(GRIP_P) 기준이라 손목 원점 대신 그 점을 목표로
    const r = w.arm.solve(s, apply(L, [0.20, 0, 0]), tx, ty, 200);
    if (!best || r.perr + 0.05 * r.oerr < best.perr + 0.05 * best.oerr) best = r;
  }
  return best && best.perr < 0.002 && best.oerr < 0.01 ? best.q : null;
  void tp;
}
function lineQ(w, qa, Ta, Tb, n) {                   // 손목을 Ta→Tb 직선으로 (방향 그대로) — 관절 값 n+1개
  const Bi = inv(poseT(w.walker.pose)), out = [qa]; let q = qa, ok = true;
  for (let k = 1; k <= n; k++) {
    const u = k / n, T = Ta.map((v, i) => v + (Tb[i] - v) * u), L = mul(Bi, T);
    const r = w.arm.solve(q, apply(L, [0.20, 0, 0]), [L[0], L[4], L[8]], [L[1], L[5], L[9]], 60);
    if (r.perr > 0.002 || r.oerr > 0.01) ok = false;
    q = r.q; out.push(q);
  }
  return ok ? out : null;
}
function pathClash(w, qs, grip, sub = 3) {           // 관절 값들을 차례로 (사이도 촘촘히) 놓아 보며 부딪힘 검사
  const q0 = w.getQ(); let hit = null;
  for (let i = 0; i < qs.length && !hit; i++) for (let k = i ? 1 : 0; k <= (i ? sub : 0) && !hit; k++) {
    w.setQ(i ? lerp(qs[i - 1], qs[i], k / sub) : qs[0]);
    if (w.held) w.held.T = mul(w.wrist(), w.held.rel);
    hit = w.clash(grip);
  }
  w.setQ(q0); if (w.held) w.held.T = mul(w.wrist(), w.held.rel);
  return hit;
}
const jointPath = (a, b, n = 8) => [...Array(n + 1)].map((_, k) => lerp(a, b, k / n));
function approach(w, q1, grip) {                     // 지금 → 목표 관절: 곧장, 안 되면 준비 자세를 거쳐
  const q0 = w.getQ(), rd = qv(READY); let hit = null;
  for (const qs of [jointPath(q0, q1), [...jointPath(q0, rd), ...jointPath(rd, q1).slice(1)]]) {
    const h = pathClash(w, qs, grip); if (!h) return { qs }; hit = hit ?? h;
  }
  return { hit };
}
// 손목 자세: 손 끝 아래, 손가락 축(y) = 수평 방향 a → 손가락은 z = x×y 쪽에서 내려온다
function wristT(a, p) {
  const c = Math.cos(a), s = Math.sin(a);   // x = (0,0,-1), y = (c,s,0), z = x×y = (s,-c,0)
  return [0, c, s, p[0], 0, s, -c, p[1], -1, 0, 0, p[2], 0, 0, 0, 1];
}
// 집게가 물건을 무는 자리 (손목 좌표의 물건 가운데): 고정 턱에 붙이고, 집게 끝이 놓인 면보다 MARGIN 위,
//   물건 윗면이 손목 몸통보다 3mm 아래 (긴 펜을 너무 깊이 물면 손목이 펜을 뚫는다)
function gripPoint(w, it) {
  const gx = Math.max(TIP_X + MARGIN - w.half(it), BODY_X + 0.003 + w.half(it));
  return [gx, 0, JAW_Z + w.width(it) + 0.001];
}
function planGrab(w, it, ctx) {
  const P = w.walker.pose, g = gripPoint(w, it), seeds = [w.getQ(), qv(READY)];
  // 손가락 축 후보: 몸 옆 방향에 가까운 순 (큐브는 면에 맞춰 90° 단위)
  let as = it.kind === "cube" ? [0, 1, 2, 3].map(k => it.yaw + k * Math.PI / 2) : [0, 0.4, -0.4, 0.8, -0.8, Math.PI / 2, -Math.PI / 2].map(d => P.yaw + Math.PI / 2 + d);
  as = as.concat(as.map(a => a + Math.PI)).sort((a, b) => Math.abs(wrap(a - P.yaw - Math.PI / 2)) - Math.abs(wrap(b - P.yaw - Math.PI / 2)));
  let why = `팔이 ${it.name}에 닿지 않습니다`;
  for (const a of as) {
    const T0 = wristT(a, [0, 0, 0]), o = apply(T0, g);
    const T = wristT(a, [it.x - o[0], it.y - o[1], it.z - o[2]]), Tp = raise(T, 0.15);
    const qp = solveArm(w, Tp, seeds); if (!qp) continue;
    const down = lineQ(w, qp, Tp, T, 10); if (!down) continue;
    const ap = approach(w, qp, it), hit = ap.hit ?? pathClash(w, down, it);
    if (hit) { why = `${it.name}에 팔을 넣으면 ${w.clashText(hit)}`; continue; }
    Object.assign(ctx, { to: ap.qs, down, up: down.slice().reverse() });
    return null;
  }
  return why;
}
function planPlace(w, place, ctx) {
  const it = w.held, P = w.walker.pose, [x, y, s] = w.where(place), Ri = inv(it.rel), seeds = [w.getQ(), qv(READY)];
  let why = "팔이 놓을 자리에 닿지 않습니다";
  const phi = Math.atan2(it.rel[4], it.rel[0]);         // 손목 기준 물건 방향 (손목 x 가 아래라 대략값 — 후보를 고르는 데만)
  for (const d of [0, 0.4, -0.4, 0.8, -0.8, Math.PI / 2, -Math.PI / 2, Math.PI]) {
    const Ti = rpyT(0, 0, P.yaw + d + phi, [x, y, s + w.half(it) + 0.001]), T = mul(Ti, Ri), Tp = raise(T, 0.15);
    if (Math.abs(T[8] + 1) > 1e-3) continue;           // 손 끝이 아래를 보는 자세만
    const qp = solveArm(w, Tp, seeds); if (!qp) continue;
    const down = lineQ(w, qp, Tp, T, 10); if (!down) continue;
    const ap = approach(w, qp, null); let hit = ap.hit ?? pathClash(w, down, null);
    if (hit) { why = `놓으러 가면 ${w.clashText(hit)}`; continue; }
    // 놓은 뒤 (집게 편 채) 위로: 놓인 물건을 안 치나
    const up = down.slice().reverse(), f0 = w.robot.q.arm_f1x, T0 = it.T;
    w.held = null; it.T = Ti; w.robot.q.arm_f1x = GRIP_OPEN; hit = pathClash(w, up, null);
    w.held = it; it.T = T0; w.robot.q.arm_f1x = f0;
    if (hit) { why = `놓고 팔을 빼면 ${w.clashText(hit)}`; continue; }
    Object.assign(ctx, { to: ap.qs, down, up });
    return null;
  }
  return why;
}
// 집게 손가락이 물건에 처음 닿는 각도 (편 → 닫힘 쪽으로 훑어서). 안 닿으면 null
function contactF(w, it) {
  const f0 = w.robot.q.arm_f1x, touch = (f) => { w.robot.q.arm_f1x = f; return gap(w.sh, w.fk(), ["arm_link_fngr"], it) <= 0.0005; };
  let prev = GRIP_OPEN, hit = null;
  for (let f = GRIP_OPEN; f <= 0 && hit === null; f += 0.02) { if (touch(f)) hit = f; else prev = f; }
  if (hit !== null) { let a = prev, b = hit; for (let k = 0; k < 8; k++) { const m = (a + b) / 2; if (touch(m)) b = m; else a = m; } hit = b; }
  w.robot.q.arm_f1x = f0;
  return hit;
}

// ── 동작 ────────────────────────────────────────────────────
//   다가갈 땐 앞 35cm 에서 방향을 맞춘 뒤 똑바로 (바닥 물건·도킹은 돌아서)
function aWalk(w, goalFn, targetId) {
  let pts;
  return { init() {
             const g = goalFn(); pts = [];
             if (!targetId) { pts.push(g); return; }
             const P = w.walker.pose, pre = { x: g.x - 0.35 * Math.cos(g.yaw), y: g.y - 0.35 * Math.sin(g.yaw), yaw: g.yaw };
             if (Math.hypot(g.x - P.x, g.y - P.y) > 0.5) pts.push(...w.route(pre, targetId));
             pts.push(g);
           },
           update() {
             const g = pts[0];
             if (pts.length > 1 && g.yaw === undefined) { const P = w.walker.pose; if (Math.hypot(g.x - P.x, g.y - P.y) < 0.15) { pts.shift(); return false; } drive(w.walker, { x: g.x, y: g.y }); return false; }
             if (pts.length > 1) { if (drive(w.walker, g)) pts.shift(); return false; }
             return drive(w.walker, g);
           } };
}
function aBody(w, b) {
  return { init() { w.walker.target = { ...b }; },
           update() { const P = w.walker.pose; return Math.abs(P.z - b.z) < 1e-3 && Math.abs(P.pitch - b.pitch) < 1e-3; } };
}
function aArmJ(w, qt, dur) {
  let q0, t = 0;
  return { init() { q0 = ARM.map(n => w.robot.q[n]); },
           update(dt) { t += dt; const u = smooth(Math.min(1, t / dur)); ARM.forEach((n, i) => w.robot.q[n] = q0[i] + (qt[n] - q0[i]) * u); return t >= dur; } };
}
function aArmPath(w, qsFn, dur) {                    // 미리 검사한 관절 값들을 따라
  let qs, t = 0;
  return { init() { qs = qsFn(); },
           update(dt) { t += dt; const u = smooth(Math.min(1, t / dur)) * (qs.length - 1), i = Math.min(qs.length - 2, Math.floor(u));
                        w.setQ(lerp(qs[i], qs[i + 1], u - i)); return t >= dur; } };
}
function aArmTo(w, qt, dur, what) {                  // 관절 목표까지 (가는 길 검사, 막히면 준비 자세를 거쳐)
  let qs, t = 0;
  return { init() { const a = approach(w, qv(qt), null); if (a.hit) return `${what}: ${w.clashText(a.hit)}`; qs = a.qs; },
           update(dt) { t += dt; const u = smooth(Math.min(1, t / dur)) * (qs.length - 1), i = Math.min(qs.length - 2, Math.floor(u));
                        w.setQ(lerp(qs[i], qs[i + 1], u - i)); return t >= dur; } };
}
function aGrip(w, v, dur) {
  let a, t = 0;
  return { init() { a = w.robot.q.arm_f1x; }, update(dt) { t += dt; w.robot.q.arm_f1x = a + (v - a) * smooth(Math.min(1, t / dur)); return t >= dur; } };
}
function aGripOn(w, it) {                            // 손가락이 닿을 때까지만 닫는다
  let a, b, t = 0;
  return { init() {
             a = w.robot.q.arm_f1x;
             if (!w.sh) { b = it.kind === "cube" ? -0.32 : -0.22; return; }
             b = contactF(w, it); if (b === null) return `${eul(it.name)} 물 수 없습니다 (손가락이 닿지 않음)`;
             if (gap(w.sh, w.fk(), ["arm_link_wr1"], it) > 0.004) return `${eul(it.name)} 물 수 없습니다 (고정 턱이 안 닿음)`;
           },
           update(dt) { t += dt; w.robot.q.arm_f1x = a + (b - a) * smooth(Math.min(1, t / 0.4)); return t >= 0.4; } };
}
function aWait(s) { let t = 0; return { update(dt) { t += dt; return t >= s; } }; }
function aAttach(w, it) {
  return { update() { it.rel = mul(inv(w.wrist()), it.T); w.held = it; it.on = "hand"; return true; } };
}
function aDetach(w, it, place) {
  return { update() {
    const T = mul(w.wrist(), it.rel); w.held = null; it.rel = null; it.T = T;
    it.yaw = Math.atan2(T[4], T[0]);
    w.settle(it, place, T[3], T[7]);
    w.log(`${it.name} → ${w.dock(place)?.name ?? w.mark(place)?.name ?? "바닥"}`, "ok");
    return true;
  } };
}
// 계획이 막히면 서는 자리를 바꿔 다시 (최대 3번)
function aPlan(w, planFn, again) {
  return { update() {
    const why = planFn(); if (!why) return true;
    if (again.k >= 3) return why;
    again.k++; w.exec.acts.splice(1, 0, ...again.acts(again.k), aPlan(w, planFn, again));
    return true;
  } };
}

// ── 블록 → 동작 ──────────────────────────────────────────────
export const WAVE = { arm_sh0: 0.3, arm_sh1: -1.6, arm_el0: 1.2, arm_el1: 0, arm_wr0: -0.3, arm_wr1: 0 };   // 집게를 들어 흔드는 자세
function aWave(w) {                                  // 손목(집게)을 좌우로 1.6초
  let t = 0;
  return { update(dt) { t += dt; const k = Math.min(1, t / 1.6);
           w.robot.q.arm_wr1 = WAVE.arm_wr1 + 0.7 * Math.sin(t * 10) * Math.sin(Math.PI * k); return t >= 1.6; } };
}
// 빈손으로 걸을 땐 팔을 접는다 (뻗은 채 걸으면 도킹 위 물건을 친다 — 실제 Spot 도 접고 걷는다)
const stowed = (w) => ARM.every(n => Math.abs(w.robot.q[n] - STOW[n]) < 1e-3);
function stowFirst(w) { return w.held || stowed(w) ? [] : [aGrip(w, GRIP_CLOSED, 0.3), aArmTo(w, STOW, 1.4, "팔 접기")]; }
function compile(w, b) {
  switch (b.type) {
    case "walk": return [...stowFirst(w), aWalk(w, () => w.standFor(b.arg), b.arg)];
    case "turn": return [aWalk(w, () => { const P = w.walker.pose; return { x: P.x, y: P.y, yaw: P.yaw + (+b.arg) * Math.PI / 180 }; })];
    case "body": return [aBody(w, b.arg === "low" ? LOW : NORMAL)];
    case "wait": return [aWait(+b.arg || 1)];
    case "rest": return [aGrip(w, GRIP_CLOSED, 0.3), aArmTo(w, STOW, 1.6, "팔 거두기")];
    case "wave": {
      let back;
      return [{ update() { back = Object.fromEntries(ARM.map(n => [n, w.robot.q[n]])); return true; } },
              aArmTo(w, WAVE, 1.0, "손 흔들기"), aWave(w), { started: false, init() { this.inner = aArmTo(w, back, 1.0, "손 흔들기"); return this.inner.init(); }, update(dt) { return this.inner.update(dt); } }];
    }
    case "grab": {
      const it = w.item(b.arg), low = it.z < 0.15, ctx = {};
      const acts = [];
      if (!w.near(it.id)) acts.push(...stowFirst(w), aWalk(w, () => w.standFor(it.id), it.id));
      acts.push(aBody(w, low ? LOW : NORMAL), aArmTo(w, READY, 1.0, "팔 펴기"), aGrip(w, GRIP_OPEN, 0.4),
        aPlan(w, () => planGrab(w, it, ctx), { k: 0, acts: (k) => [aArmTo(w, READY, 0.6, "팔 거두기"), aWalk(w, () => w.standFor(it.id, w.walker.pose, k), it.id), aBody(w, low ? LOW : NORMAL)] }),
        aArmPath(w, () => ctx.to, 1.2), aArmPath(w, () => ctx.down, 0.8), aGripOn(w, it), aAttach(w, it),
        aArmPath(w, () => ctx.up, 0.8), aArmTo(w, READY, 1.0, "들어 올리기"), aBody(w, NORMAL));
      return acts;
    }
    case "place": {
      const it = w.held, low = w.where(b.arg)[2] < 0.15, ctx = {};
      const acts = [];
      if (!w.near(b.arg)) acts.push(aWalk(w, () => w.standFor(b.arg), b.arg));
      acts.push(aBody(w, low ? LOW : NORMAL),
        aPlan(w, () => planPlace(w, b.arg, ctx), { k: 0, acts: (k) => [aArmTo(w, READY, 0.6, "팔 거두기"), aWalk(w, () => w.standFor(b.arg, w.walker.pose, k), b.arg), aBody(w, low ? LOW : NORMAL)] }),
        aArmPath(w, () => ctx.to, 1.2), aArmPath(w, () => ctx.down, 0.8), aGrip(w, GRIP_OPEN, 0.4), aDetach(w, it, b.arg),
        aArmPath(w, () => ctx.up, 0.7), aGrip(w, GRIP_CLOSED, 0.3), aArmTo(w, READY, 1.0, "팔 거두기"), aBody(w, NORMAL));
      return acts;
    }
  }
  return [];
}
export const _inner = { planGrab, planPlace, solveArm, lineQ, pathClash, approach, gripPoint, contactF };   // 검사용
