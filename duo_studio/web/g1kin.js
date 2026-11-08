// G1 STUDIO 계산 — 물리 없이 계산으로 2족 걷기 · 다리 IK(6관절) · 팔 IK(7관절) · 손가락 (11-01)
//   브라우저(app.js)와 Node 검사(test_*.mjs)가 같이 쓴다. 좌표: Z 위, 바닥 z=0, 로봇 앞 = 골반 x.
import { mul, rpyT, apply, inv, col, cross, sub, norm, clamp, wrap, poseT, Robot } from "./base.js";
export { Robot, poseT, mul, rpyT, apply, inv, clamp, wrap };

// ── 사슬 IK (감쇠 최소제곱, 관절 수 무관) ─────────────────────
//   from 링크 좌표에서 to 링크의 점 off 를 tp 로, 그 링크의 축들을 목표 방향으로.
//   axes: [[링크 축 번호 0=x 1=y 2=z, 목표 방향], …]
function solve6(A, b) {
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
export class Chain {
  constructor(robot, from, to, off = [0, 0, 0], vary = null) {
    this.robot = robot; this.joints = robot.chain(from, to); this.off = off;
    this.vary = (vary || this.joints.filter(j => j.movable).map(j => j.name));
    this.lims = this.vary.map(n => [robot.byName[n].lower, robot.byName[n].upper]);
  }
  fk(qv) {
    const qm = { ...this.robot.q }; this.vary.forEach((n, i) => qm[n] = qv[i]);
    return this.robot.chainT(this.joints, qm);
  }
  //   축 두 개만 주면 셋째 축을 채워 세 축 다 맞춘다 (두 축 오차의 합은 서로 상쇄될 수 있다 — 11-01: 손이 56° 기운 채 '맞음' 으로 나왔다)
  static full(axes) {
    if (axes.length !== 2) return axes;
    const [[a, ta], [b, tb]] = axes, k = 3 - a - b, sgn = ((b - a + 3) % 3 === 1) ? 1 : -1;   // x×y=z, y×z=x, z×x=y
    const t = cross(ta, tb).map(v => v * sgn);
    return [...axes, [k, t]];
  }
  err(qv, tp, axes, wo) {
    const M = this.fk(qv), p = apply(M, this.off), e = sub(tp, p);
    const o = [0, 0, 0];
    for (const [k, t] of axes) { const c = cross(col(M, k), t); o[0] += c[0] / 2; o[1] += c[1] / 2; o[2] += c[2] / 2; }
    return [...e, ...o.map(v => v * wo)];
  }
  // 목표 방향과의 실제 각도 (rad)
  angle(qv, axes) {
    if (!axes.length) return 0;
    const M = this.fk(qv); let worst = 0;
    for (const [k, t] of axes) { const c = col(M, k), d = (c[0] * t[0] + c[1] * t[1] + c[2] * t[2]) / (norm(t) || 1); worst = Math.max(worst, Math.acos(clamp(d, -1, 1))); }
    return worst;
  }
  solve(tp, axes, iters = 40, wo = 0.3, lam = 0.03) {
    axes = Chain.full(axes);
    let q = this.vary.map(n => this.robot.q[n]);
    let e = this.err(q, tp, axes, wo); const N = q.length;
    for (let it = 0; it < iters; it++) {
      if (norm(e.slice(0, 3)) < 1e-4 && norm(e.slice(3)) < 1e-3) break;
      const J = [];
      for (let i = 0; i < N; i++) { const qd = q.slice(); qd[i] += 1e-5; const ed = this.err(qd, tp, axes, wo); J.push(ed.map((v, k) => (e[k] - v) / 1e-5)); }
      const A = [...Array(6)].map((_, r) => [...Array(6)].map((_, c) => { let s = 0; for (let i = 0; i < N; i++) s += J[i][r] * J[i][c]; return s + (r === c ? lam * lam : 0); }));
      const y = solve6(A, e);
      for (let i = 0; i < N; i++) { let d = 0; for (let r = 0; r < 6; r++) d += J[i][r] * y[r]; q[i] = clamp(q[i] + clamp(d, -0.3, 0.3), this.lims[i][0], this.lims[i][1]); }
      e = this.err(q, tp, axes, wo);
    }
    this.vary.forEach((n, i) => this.robot.q[n] = q[i]);
    return { perr: norm(e.slice(0, 3)), oerr: this.angle(q, axes) };
  }
}

// ── 몸 ──────────────────────────────────────────────────────
export const SIDES = ["left", "right"];
export const ANKLE_H = 0.035;                     // 발목(ankle_roll) 원점이 발바닥보다 이만큼 위
export const FOOT_X = 0.035;                      // 발 가운데가 발목보다 이만큼 앞
export const HIP_W = 0.118;                       // 두 발 사이 반 (서 있을 때)
export const WALK_Z = 0.72, STAND_Z = 0.74;       // 골반 높이
export const R_ARM = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
                      "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"];
export const L_ARM = R_ARM.map(n => n.replace("right", "left"));
export const R_HAND = ["right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
                       "right_hand_middle_0_joint", "right_hand_middle_1_joint", "right_hand_index_0_joint", "right_hand_index_1_joint"];
export const L_HAND = R_HAND.map(n => n.replace("right", "left"));
export const GRIP_P = [0.075, 0.05, 0];           // 오른손 손바닥 좌표의 쥐는 점 (손가락이 감싸는 쪽 = +y)
// 팔 자세 (오른팔 기준, 왼팔은 거울)
export const ARM_REST = [0.25, -0.6, 0, 0.7, 0, 0, 0];       // 내려뜨림 (걸으며 흔들거나 탁자 앞에서 숙여도 엄지가 엉덩이에 안 박히게 — 11-01 부딪힘 계산으로 고름)
export const ARM_READY = [-0.55, -0.15, 0.1, 0.55, 0, 0.3, 0]; // 앞으로 내밈
export const HAND_OPEN = [0, 0.3, 0, 0, 0, 0, 0];
export const HAND_CLOSE = [0, -0.45, -0.95, 1.05, 0.95, 1.05, 0.95];
export const mirror = (name, v) => {                // 오른쪽 값 → 왼쪽 관절 값 (roll·yaw·손가락 굽힘은 부호 반대)
  if (/shoulder_pitch|elbow|wrist_pitch|thumb_0/.test(name)) return v;
  return -v;
};

// ── 2족 걷기 ────────────────────────────────────────────────
//   발 한 번 = 한 걸음 (T 초). 앞 DS 비율은 두 발 디딤, 나머지는 한 발 디딤 + 다른 발 들기.
//   골반은 디딘 발 쪽으로 sin 모양으로 옮긴다 (무게중심을 디딘 발 위로 — 물리는 없지만 그렇게 보이게).
export class Biped {
  constructor(robot, opt = {}) {
    this.robot = robot;
    this.T = opt.T ?? 0.55; this.DS = opt.DS ?? 0.2; this.stepH = opt.stepH ?? 0.06; this.sway = opt.sway ?? 0.035;
    this.vmax = opt.vmax ?? 0.32; this.wmax = opt.wmax ?? 0.7; this.acc = 0.6; this.wacc = 1.6;
    this.path = { x: 0, y: 0, yaw: 0 };              // 걷는 길 위의 '몸 자리' (두 발 가운데)
    this.vel = { vx: 0, vy: 0, wz: 0 }; this.cmd = { vx: 0, vy: 0, wz: 0 };
    this.post = { z: STAND_Z, pitch: 0, back: 0, waist: 0 };  // 자세 (지금)
    this.target = { ...this.post };                           // 자세 (목표)
    this.stance = { w: HIP_W, toe: 0 };
    this.legs = Object.fromEntries(SIDES.map(s => [s, new Chain(robot, "pelvis", s + "_ankle_roll_link")]));
    for (const s of SIDES) { const q = robot.q; q[s + "_hip_pitch_joint"] = -0.3; q[s + "_knee_joint"] = 0.6; q[s + "_ankle_pitch_joint"] = -0.3; }
    this.feet = Object.fromEntries(SIDES.map(s => [s, this.nominal(s, this.path)]));
    this.walking = false; this.s = 0; this.swing = null; this.start = null; this.lat = 0;
    this.pelvis = null; this.legOk = true; this.legErr = 0;
    this.solve();
  }
  nominal(side, P) {                                // 발 제자리 (stance: 발 사이 반 · 발끝 벌림 — 쪼그려 앉을 땐 넓게)
    const L = side === "left", w = L ? this.stance.w : -this.stance.w, c = Math.cos(P.yaw), s = Math.sin(P.yaw);
    return { x: P.x - s * w, y: P.y + c * w, yaw: P.yaw + (L ? this.stance.toe : -this.stance.toe) };
  }
  predict(dt) {
    const P = { ...this.path }, v = this.vel, yaw = P.yaw + v.wz * dt / 2;
    P.x += (v.vx * Math.cos(yaw) - v.vy * Math.sin(yaw)) * dt; P.y += (v.vx * Math.sin(yaw) + v.vy * Math.cos(yaw)) * dt; P.yaw += v.wz * dt;
    return P;
  }
  moving() { const c = this.cmd, v = this.vel; return Math.abs(c.vx) + Math.abs(c.vy) + Math.abs(c.wz) + Math.abs(v.vx) + Math.abs(v.vy) + Math.abs(v.wz) > 1e-3; }
  settled() { return SIDES.every(s => { const n = this.nominal(s, this.path), f = this.feet[s]; return Math.hypot(f.x - n.x, f.y - n.y) < 0.012 && Math.abs(wrap(f.yaw - n.yaw)) < 0.03; }); }
  pickSwing() {                                     // 먼저 들 발: 제자리에서 더 먼 발 (같으면 가는 쪽 반대 발)
    const d = (s) => { const n = this.nominal(s, this.predict(this.T)), f = this.feet[s]; return Math.hypot(f.x - n.x, f.y - n.y) + 0.1 * Math.abs(wrap(f.yaw - n.yaw)); };
    if (this.swing) return this.swing === "left" ? "right" : "left";
    if (Math.abs(this.cmd.vy) > 1e-3) return this.cmd.vy > 0 ? "left" : "right";
    if (Math.abs(this.cmd.wz) > 1e-3) return this.cmd.wz > 0 ? "left" : "right";
    return d("left") >= d("right") ? "left" : "right";
  }
  update(dt) {
    const v = this.vel, c = this.cmd, step = (a, b, m) => a + clamp(b - a, -m * dt, m * dt);
    v.vx = step(v.vx, clamp(c.vx, -this.vmax, this.vmax), this.acc);
    v.vy = step(v.vy, clamp(c.vy, -0.2, 0.2), this.acc);
    v.wz = step(v.wz, clamp(c.wz, -this.wmax, this.wmax), this.wacc);
    if (this.walking) { const P = this.predict(dt); Object.assign(this.path, P); }
    else { v.vx = 0; v.vy = 0; v.wz = 0; }
    for (const k of ["z", "pitch", "back", "waist"]) this.post[k] = step(this.post[k], this.target[k], k === "z" || k === "back" ? 0.25 : 0.6);
    const need = this.moving() || !this.settled();
    if (!this.walking) {
      if (need) { this.walking = true; this.s = 0; this.swing = null; this.swing = this.pickSwing(); this.start = { ...this.feet[this.swing] }; }
    } else {
      this.s += dt / this.T;
      if (this.s >= 1) {                             // 한 걸음 끝 — 든 발이 내려옴
        this.s -= 1; this.feet[this.swing].lift = 0;
        if (!(this.moving() || !this.settled())) { this.walking = false; this.s = 0; this.swing = null; }
        else { this.swing = this.pickSwing(); this.start = { ...this.feet[this.swing] }; }
      }
    }
    let lat = 0;
    if (this.walking) {
      const u = clamp((this.s - this.DS) / (1 - this.DS), 0, 1), us = u * u * (3 - 2 * u);
      const P = this.predict((1 - this.s) * this.T + this.T / 2);
      const n = this.nominal(this.swing, P), a = this.start;
      this.feet[this.swing] = { x: a.x + (n.x - a.x) * us, y: a.y + (n.y - a.y) * us, yaw: a.yaw + wrap(n.yaw - a.yaw) * us, lift: this.stepH * Math.sin(Math.PI * u) };
      lat = (this.swing === "left" ? -1 : 1) * this.sway * Math.sin(Math.PI * this.s);   // 디딘 발(반대쪽)으로
    }
    this.lat = lat;
    this.solve();
  }
  pelvisPose() {                                     // 골반 = 길 자리 + 옆 흔들림 + 뒤로 빼기 + 높이·숙임
    const P = this.path, c = Math.cos(P.yaw), s = Math.sin(P.yaw), b = -this.post.back;
    return { x: P.x + c * b - s * this.lat, y: P.y + s * b + c * this.lat, z: this.post.z, roll: 0, pitch: this.post.pitch, yaw: P.yaw };
  }
  solve() {
    this.pelvis = this.pelvisPose();
    this.robot.q.waist_pitch_joint = this.post.waist;
    const Bi = inv(poseT(this.pelvis)), R = Bi.slice(); R[3] = R[7] = R[11] = 0;
    let ok = true, worst = 0;
    for (const sd of SIDES) {
      const f = this.feet[sd], cy = Math.cos(f.yaw), sy = Math.sin(f.yaw);
      const tp = apply(Bi, [f.x, f.y, ANKLE_H + (f.lift || 0)]);
      const r = this.legs[sd].solve(tp, [[0, apply(R, [cy, sy, 0])], [2, apply(R, [0, 0, 1])]], 25);
      const air = this.walking && sd === this.swing && (f.lift || 0) > 1e-3;     // 든 발은 바닥에 안 닿으니 조금 기울어도 됨
      worst = Math.max(worst, r.perr); ok = ok && r.perr < 2e-3 && r.oerr < (air ? 0.05 : 0.02);
    }
    this.legOk = ok; this.legErr = worst;
  }
}

// ── 목표 자리까지 몰기 ───────────────────────────────────────
export function drive(w, goal) {
  const P = w.path, dx = goal.x - P.x, dy = goal.y - P.y, dist = Math.hypot(dx, dy);
  if (dist > 0.03) {
    const head = Math.atan2(dy, dx), e = wrap(head - P.yaw);
    if (Math.abs(e) > 0.35 && dist > 0.3) { w.cmd = { vx: 0, vy: 0, wz: clamp(1.8 * e, -w.wmax, w.wmax) }; return false; }
    const fx = dx * Math.cos(P.yaw) + dy * Math.sin(P.yaw), fy = -dx * Math.sin(P.yaw) + dy * Math.cos(P.yaw);
    const sp = Math.min(w.vmax, 1.0 * dist), near = dist < 0.3;
    w.cmd = { vx: sp * fx / dist, vy: near ? Math.min(0.15, sp) * fy / dist : 0,
              wz: clamp(1.8 * (near ? wrap((goal.yaw ?? P.yaw) - P.yaw) : e), -w.wmax, w.wmax) };
    return false;
  }
  const e = wrap((goal.yaw ?? P.yaw) - P.yaw);
  if (Math.abs(e) > 0.03) { w.cmd = { vx: 0, vy: 0, wz: clamp(1.8 * e, -w.wmax, w.wmax) }; return false; }
  w.cmd = { vx: 0, vy: 0, wz: 0 };
  return !w.walking;
}
