// SPOT STUDIO 계산 — 물리 없이 계산으로 걷기 · 다리 IK · 팔 IK (11-01)
//   브라우저(app.js)와 Node 검사(test_kin.mjs)가 같이 쓴다 — three.js 에 기대지 않는다.
//   좌표: URDF 그대로 Z 가 위, 바닥 z=0, 로봇 앞 = 몸통 x.
//   행렬: 4×4, 행 우선 배열 16칸.

// ── 행렬 ─────────────────────────────────────────────────────
export const I4 = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
export function mul(A, B) {
  const C = new Array(16);
  for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) {
    C[r * 4 + c] = A[r * 4] * B[c] + A[r * 4 + 1] * B[4 + c] + A[r * 4 + 2] * B[8 + c] + A[r * 4 + 3] * B[12 + c];
  }
  return C;
}
export function rpyT(r, p, y, t = [0, 0, 0]) {       // URDF rpy: R = Rz(y)·Ry(p)·Rx(r)
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, t[0],
          sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, t[1],
          -sp, cp * sr, cp * cr, t[2], 0, 0, 0, 1];
}
export function axisT(a, q) {                       // 축 a 둘레 q 라디안 (로드리게스)
  const n = Math.hypot(a[0], a[1], a[2]) || 1, x = a[0] / n, y = a[1] / n, z = a[2] / n;
  const c = Math.cos(q), s = Math.sin(q), C = 1 - c;
  return [c + x * x * C, x * y * C - z * s, x * z * C + y * s, 0,
          y * x * C + z * s, c + y * y * C, y * z * C - x * s, 0,
          z * x * C - y * s, z * y * C + x * s, c + z * z * C, 0, 0, 0, 0, 1];
}
export function apply(M, p) {
  return [M[0] * p[0] + M[1] * p[1] + M[2] * p[2] + M[3],
          M[4] * p[0] + M[5] * p[1] + M[6] * p[2] + M[7],
          M[8] * p[0] + M[9] * p[1] + M[10] * p[2] + M[11]];
}
export function inv(M) {                            // 강체 변환의 역
  const t = [M[3], M[7], M[11]];
  const R = [M[0], M[4], M[8], 0, M[1], M[5], M[9], 0, M[2], M[6], M[10], 0, 0, 0, 0, 1];
  const ti = apply(R, t);
  R[3] = -ti[0]; R[7] = -ti[1]; R[11] = -ti[2];
  return R;
}
const col = (M, k) => [M[k], M[4 + k], M[8 + k]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
export const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

export function poseT(P) { return rpyT(P.roll, P.pitch, P.yaw, [P.x, P.y, P.z]); }

// ── 로봇 (robot.json) ────────────────────────────────────────
export class Robot {
  constructor(model) {
    this.model = model;
    this.joints = model.joints.map(j => ({ ...j, T0: rpyT(j.rpy[0], j.rpy[1], j.rpy[2], j.xyz), movable: j.type !== "fixed" }));
    this.byName = Object.fromEntries(this.joints.map(j => [j.name, j]));
    this.byChild = Object.fromEntries(this.joints.map(j => [j.child, j]));
    this.order = [];                                  // 뿌리에서 가까운 순서
    const seen = new Set([model.root]);
    while (this.order.length < this.joints.length) {
      for (const j of this.joints) if (!seen.has(j.child) && seen.has(j.parent)) { this.order.push(j); seen.add(j.child); }
    }
    this.q = {};
    for (const j of this.joints) if (j.movable) this.q[j.name] = clamp(0, j.lower, j.upper);
  }
  set(name, v) { const j = this.byName[name]; this.q[name] = clamp(v, j.lower, j.upper); return this.q[name] === v; }
  jointT(j, q) { return j.movable ? mul(j.T0, axisT(j.axis, q)) : j.T0; }
  fk(baseT) {                                       // 링크 이름 → 월드 행렬
    const W = { [this.model.root]: baseT };
    for (const j of this.order) W[j.child] = mul(W[j.parent], this.jointT(j, this.q[j.name] ?? 0));
    return W;
  }
  chain(from, to) {                                 // from 링크 → to 링크 사이 관절들
    const out = []; let l = to;
    while (l !== from) { const j = this.byChild[l]; out.unshift(j); l = j.parent; }
    return out;
  }
  chainT(joints, qmap) { let M = I4(); for (const j of joints) M = mul(M, this.jointT(j, qmap[j.name] ?? 0)); return M; }
}

// ── 다리 IK (풀이식) ─────────────────────────────────────────
//   엉덩이 x(옆으로 벌림) → 엉덩이 y(앞뒤) → 무릎 y. 발 끝 = 아랫다리 (0,0,-FOOT_L), 발 반지름 FOOT_R.
export const LEGS = ["front_left", "front_right", "rear_left", "rear_right"];
export const FOOT_L = 0.335, FOOT_R = 0.036;
export function legGeom(robot, leg) {
  const hx = robot.byName[leg + "_hip_x"], hy = robot.byName[leg + "_hip_y"], kn = robot.byName[leg + "_knee"];
  return { H: hx.xyz, s: hy.xyz[1], K: [kn.xyz[0], kn.xyz[2]], jx: hx, jy: hy, jk: kn };
}
export function legIK(g, p) {                       // p = 몸통 좌표의 발 끝 → {a,b,c, ok}
  const d = sub(p, g.H), s = g.s;
  const r2 = d[1] * d[1] + d[2] * d[2];
  const L = Math.sqrt(Math.max(r2 - s * s, 1e-6));
  const fz = -L;
  const a = Math.atan2(d[2], d[1]) - Math.atan2(fz, s);
  const ux = d[0], uz = fz;                          // 다리 평면 (x, z)
  const k = g.K, R = Math.hypot(k[0], k[1]), phi = Math.atan2(k[0], -k[1]);
  let un = Math.hypot(ux, uz), ok = true;
  const umax = R + FOOT_L - 1e-4, umin = Math.abs(R - FOOT_L) + 0.05;
  let sx = ux, sz = uz;
  if (un > umax || un < umin) { ok = false; const t = clamp(un, umin, umax) / un; sx *= t; sz *= t; un = clamp(un, umin, umax); }
  const cc = clamp((un * un - R * R - FOOT_L * FOOT_L) / (2 * R * FOOT_L), -1, 1);
  const c = -Math.acos(cc) - phi;                    // 무릎은 뒤로 굽는다 (한계가 음수)
  const vx = k[0] - FOOT_L * Math.sin(c), vz = k[1] - FOOT_L * Math.cos(c);
  const b = Math.atan2(vz, vx) - Math.atan2(sz, sx);
  const inLim = (j, v) => v >= j.lower - 1e-9 && v <= j.upper + 1e-9;
  ok = ok && inLim(g.jx, a) && inLim(g.jy, wrap(b)) && inLim(g.jk, c);
  return { a, b: wrap(b), c, ok };
}

// ── 팔 IK (수치: 감쇠 최소제곱) ───────────────────────────────
export const ARM = ["arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1"];
export const GRIP_P = [0.20, 0, 0];                // 손목(wr1) 좌표의 잡는 점 (턱 사이)
export const STOW = { arm_sh0: 0, arm_sh1: -3.1, arm_el0: 3.1, arm_el1: 0, arm_wr0: 0, arm_wr1: 0 };
export const READY = { arm_sh0: 0, arm_sh1: -1.15, arm_el0: 2.1, arm_el1: 0, arm_wr0: -0.6, arm_wr1: 0 };
export const GRIP_OPEN = -1.2, GRIP_CLOSED = -0.05;

function solve(A, b) {                               // 작은 선형식 (가우스 소거)
  const n = b.length, M = A.map((r, i) => [...r, b[i]]);
  for (let i = 0; i < n; i++) {
    let p = i; for (let r = i + 1; r < n; r++) if (Math.abs(M[r][i]) > Math.abs(M[p][i])) p = r;
    [M[i], M[p]] = [M[p], M[i]];
    for (let r = i + 1; r < n; r++) { const f = M[r][i] / M[i][i]; for (let c = i; c <= n; c++) M[r][c] -= f * M[i][c]; }
  }
  const x = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) { let s = M[i][n]; for (let c = i + 1; c < n; c++) s -= M[i][c] * x[c]; x[i] = s / M[i][i]; }
  return x;
}

export class ArmIK {
  constructor(robot) {
    this.robot = robot;
    this.chain = robot.chain(robot.model.root, "arm_link_wr1");
    this.lims = ARM.map(n => [robot.byName[n].lower, robot.byName[n].upper]);
  }
  fk(q) {                                           // 몸통 좌표: 잡는 점, 회전 행렬
    const qm = Object.fromEntries(ARM.map((n, i) => [n, q[i]]));
    const M = this.robot.chainT(this.chain, qm);
    return { p: apply(M, GRIP_P), M };
  }
  //   ty 를 주면 셋째 축(z = x×y)까지 맞춘다 — 두 축 오차의 합은 서로 상쇄될 수 있다 (11-01 G1 에서 손이 56° 기운 채 '맞음')
  err(q, tp, tx, ty, wo) {
    const { p, M } = this.fk(q);
    const ep = sub(tp, p);
    const x = col(M, 0), y = col(M, 1), z = col(M, 2);
    const eo = cross(x, tx);
    if (ty) { const a = cross(y, ty), b = cross(z, cross(tx, ty)); for (let i = 0; i < 3; i++) eo[i] = (eo[i] + a[i] + b[i]) / 2; }
    return [...ep, ...eo.map(v => v * wo)];
  }
  angle(q, tx, ty) {                                 // 목표 방향과의 실제 각도 (rad)
    const { M } = this.fk(q), ang = (c, t) => Math.acos(clamp((c[0] * t[0] + c[1] * t[1] + c[2] * t[2]) / (norm(t) || 1), -1, 1));
    return Math.max(ang(col(M, 0), tx), ty ? ang(col(M, 1), ty) : 0);
  }
  // tp: 잡는 점 목표(몸통 좌표) · tx: 손이 향할 방향(손목 x축) · ty: 턱이 벌어지는 축(손목 y축, 없으면 자유)
  solve(q0, tp, tx, ty = null, iters = 80) {
    const wo = 0.25, lam = 0.04;
    let q = q0.slice(), e = this.err(q, tp, tx, ty, wo);
    for (let it = 0; it < iters; it++) {
      const J = [];                                  // 6×6 (행 = 오차, 열 = 관절)
      for (let i = 0; i < 6; i++) {
        const qd = q.slice(); qd[i] += 1e-5;
        const ed = this.err(qd, tp, tx, ty, wo);
        J.push(ed.map((v, k) => (e[k] - v) / 1e-5));  // e = 목표 - 현재 → 미분은 부호 반대
      }
      const JJt = [...Array(6)].map((_, r) => [...Array(6)].map((_, c) => {
        let s = 0; for (let i = 0; i < 6; i++) s += J[i][r] * J[i][c]; return s + (r === c ? lam * lam : 0);
      }));
      const yv = solve(JJt, e);
      for (let i = 0; i < 6; i++) {
        let dq = 0; for (let r = 0; r < 6; r++) dq += J[i][r] * yv[r];
        q[i] = clamp(q[i] + clamp(dq, -0.25, 0.25), this.lims[i][0], this.lims[i][1]);
      }
      e = this.err(q, tp, tx, ty, wo);
      if (norm(e.slice(0, 3)) < 2e-4 && norm(e.slice(3)) < 2e-3) break;
    }
    return { q, perr: norm(e.slice(0, 3)), oerr: this.angle(q, tx, ty) };
  }
}

// ── 걷기 (트롯: 대각선 두 다리씩) ─────────────────────────────
export class Walker {
  constructor(robot, opt = {}) {
    this.robot = robot;
    this.geom = Object.fromEntries(LEGS.map(l => [l, legGeom(robot, l)]));
    this.period = opt.period ?? 0.6;               // 한 주기 (초) — 반은 A 무리, 반은 B 무리가 발을 든다
    this.stepH = opt.stepH ?? 0.08;
    this.vmax = opt.vmax ?? 0.45; this.wmax = opt.wmax ?? 0.9;
    this.acc = opt.acc ?? 0.8; this.wacc = opt.wacc ?? 2.0;
    this.pose = { x: 0, y: 0, z: 0.52, roll: 0, pitch: 0, yaw: 0, ...(opt.pose || {}) };
    this.target = { z: this.pose.z, pitch: 0 };
    this.vel = { vx: 0, vy: 0, wz: 0 }; this.cmd = { vx: 0, vy: 0, wz: 0 };
    this.walking = false; this.phase = 0; this.swingStart = {};
    this.feet = Object.fromEntries(LEGS.map(l => [l, this.nominal(l, this.pose)]));
    this.legOk = true;
    this.solveLegs();
  }
  group(leg) { return (leg === "front_left" || leg === "rear_right") ? 0 : 1; }
  nominal(leg, P) {                                  // 엉덩이 바로 아래 바닥
    const g = this.geom[leg];
    const w = apply(poseT(P), [g.H[0], g.H[1] + g.s, 0]);
    return [w[0], w[1], FOOT_R];
  }
  predict(dt) {                                      // 지금 속도로 dt 뒤 몸통 자리
    const P = { ...this.pose }, v = this.vel;
    const yaw = P.yaw + v.wz * dt / 2;
    P.x += (v.vx * Math.cos(yaw) - v.vy * Math.sin(yaw)) * dt;
    P.y += (v.vx * Math.sin(yaw) + v.vy * Math.cos(yaw)) * dt;
    P.yaw += v.wz * dt;
    return P;
  }
  settled() { return LEGS.every(l => Math.hypot(...sub(this.feet[l], this.nominal(l, this.pose)).slice(0, 2)) < 0.015); }
  moving() { const c = this.cmd, v = this.vel; return Math.abs(c.vx) + Math.abs(c.vy) + Math.abs(c.wz) + Math.abs(v.vx) + Math.abs(v.vy) + Math.abs(v.wz) > 1e-3; }
  startSwing(g) { for (const l of LEGS) if (this.group(l) === g) this.swingStart[l] = this.feet[l].slice(); }
  update(dt) {
    // 속도: 명령까지 가속 한계로
    const v = this.vel, c = this.cmd, step = (a, b, m) => a + clamp(b - a, -m * dt, m * dt);
    v.vx = step(v.vx, clamp(c.vx, -this.vmax, this.vmax), this.acc);
    v.vy = step(v.vy, clamp(c.vy, -this.vmax, this.vmax), this.acc);
    v.wz = step(v.wz, clamp(c.wz, -this.wmax, this.wmax), this.wacc);
    // 몸통: 발이 땅에 있을 때만 앞으로 (걷는 중이 아니면 서 있다)
    if (this.walking || !this.moving()) {
      const P = this.predict(dt); this.pose.x = P.x; this.pose.y = P.y; this.pose.yaw = P.yaw;
    }
    this.pose.z = step(this.pose.z, this.target.z, 0.18);
    this.pose.pitch = step(this.pose.pitch, this.target.pitch, 0.5);
    // 걸음 시계
    const need = this.moving() || !this.settled();
    if (!this.walking) {
      if (need) { this.walking = true; this.phase = 0; this.startSwing(0); }
    } else {
      const before = Math.floor(this.phase * 2);
      this.phase += dt / this.period;
      const after = Math.floor(this.phase * 2);
      if (after !== before) {                        // 반 주기 끝 = 들던 발이 다 내려옴
        for (const l of LEGS) if (this.group(l) === (before % 2)) { const p = this.feet[l]; p[2] = FOOT_R; }
        if (!(this.moving() || !this.settled())) { this.walking = false; this.phase = 0; }
        else this.startSwing(after % 2);
        if (this.phase >= 1) this.phase -= 1;
      }
    }
    if (this.walking) {
      const g = Math.floor(this.phase * 2) % 2, s = (this.phase * 2) % 1;
      const half = this.period / 2;
      const P = this.predict((1 - s) * half + half / 2);  // 내려 디딘 뒤 반쯤 지난 몸통 자리
      for (const l of LEGS) {
        if (this.group(l) !== g) continue;
        const a = this.swingStart[l], b = this.nominal(l, P);
        const u = s * s * (3 - 2 * s);
        this.feet[l] = [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, FOOT_R + this.stepH * Math.sin(Math.PI * s)];
      }
    }
    this.solveLegs();
  }
  solveLegs() {
    const Bi = inv(poseT(this.pose)); let ok = true;
    for (const l of LEGS) {
      const r = legIK(this.geom[l], apply(Bi, this.feet[l]));
      this.robot.q[l + "_hip_x"] = r.a; this.robot.q[l + "_hip_y"] = r.b; this.robot.q[l + "_knee"] = r.c;
      ok = ok && r.ok;
    }
    this.legOk = ok;
  }
}

// ── 목표 자리까지 몰기 (돌고 → 가고 → 마지막 방향 맞추기) ──────
export function drive(w, goal) {
  const P = w.pose, dx = goal.x - P.x, dy = goal.y - P.y, dist = Math.hypot(dx, dy);
  if (dist > 0.03) {
    const head = Math.atan2(dy, dx), e = wrap(head - P.yaw);
    if (Math.abs(e) > 0.35 && dist > 0.25) { w.cmd = { vx: 0, vy: 0, wz: clamp(2.0 * e, -w.wmax, w.wmax) }; return false; }
    // 가까우면 옆걸음도 섞어 정확히 맞춘다
    const fx = dx * Math.cos(P.yaw) + dy * Math.sin(P.yaw), fy = -dx * Math.sin(P.yaw) + dy * Math.cos(P.yaw);
    const sp = Math.min(w.vmax, 1.2 * dist);
    w.cmd = { vx: sp * fx / dist, vy: dist < 0.25 ? sp * fy / dist : 0, wz: clamp(2.0 * (dist < 0.25 ? wrap((goal.yaw ?? P.yaw) - P.yaw) : e), -w.wmax, w.wmax) };
    return false;
  }
  const e = wrap((goal.yaw ?? P.yaw) - P.yaw);
  if (Math.abs(e) > 0.03) { w.cmd = { vx: 0, vy: 0, wz: clamp(2.0 * e, -w.wmax, w.wmax) }; return false; }
  w.cmd = { vx: 0, vy: 0, wz: 0 };
  return !w.walking && Math.hypot(w.vel.vx, w.vel.vy, w.vel.wz) < 1e-3;
}
