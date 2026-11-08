// 월드 레이첼 — Spot 이 물리 속에서 스스로 서고 걷게 하는 AI 의 연습장 (11-02)
//   물리 엔진은 앱(duo_studio · arm_studio_multi)과 같은 Rapier 0.14 — 여기서 배운 그대로 앱에 넣는다.
//   몸통(팔은 접은 채 몸통에 붙임) + 다리 4개 × (엉덩이·윗다리·아랫다리) = 물체 13개, 관절 12개.
//   관절 힘은 실제 로봇처럼 직접 계산한다: 힘 = 세기 × (목표각 − 지금각) − 감쇠 × 각속도, 한도 안에서.
//   AI 가 내는 것 = 관절 12개의 목표각 (선 자세에서 얼마나 벗어날지). 1초에 60번 정하고, 물리는 240번 돈다.
//   브라우저·Node 같은 코드 (fs 를 쓰지 않는다: 모델은 밖에서 넣어 준다).
import { Robot, Walker, LEGS, FOOT_L, FOOT_R, STOW, GRIP_CLOSED, poseT } from "../duo_studio/web/spotkin.js";

export const JN = LEGS.flatMap(l => [`${l}_hip_x`, `${l}_hip_y`, `${l}_knee`]);
export const ACT = 12;
// v1 = 걷기·돌기만 (관찰 51) · v2 = + 몸 높이·숙임 명령, 팔 무게가 움직인다 (관찰 53)
export const OBS = 51, OBS_OF = { 1: 51, 2: 53, 3: 53, 4: 53, 5: 53, 6: 53 };   // v3 = v2 와 같은 관찰, 점수만 바꿈 (이어서 배운다)
// 팔 링크 무게 (합 6.2kg). v2 는 팔 모양만 몸통에 두고(0.7kg) 나머지 무게는 '팔 무게 점' 하나로 실제 팔 무게중심에 둔다
export const ARM_MASS = { arm_link_sh0: 1.5, arm_link_sh1: 0.5, arm_link_hr0: 0.3, arm_link_el0: 1.4, arm_link_el1: 1.2, arm_link_wr0: 0.5, arm_link_wr1: 0.5, arm_link_fngr: 0.3 };
export const ARM_SHELL = 0.7;                        // v2: 몸통에 붙은 팔 모양의 무게
export const DT = 1 / 240, DECIM = 4, CTRL_DT = DT * DECIM;   // 물리 240번/초, AI 60번/초
export const Z0 = 0.52;                              // 선 자세 몸통 높이 (spotkin Walker 와 같음)
export const GAIN = { kp: 250, kd: 6 }, ACT_SCALE = 0.3;   // 80 이면 몸통이 16cm 주저앉는다 (11-02 실측)
const TMAX = { hip_x: 45, hip_y: 45, knee: 115 };   // 관절 힘 한도 (N·m, Spot 실물 값)
export const MASS = { body: 12, arm: 6.2, hip: 1.68, upper_leg: 2.34, lower_leg: 1.0, foot: 0.5 };   // 아랫다리는 실물(0.35)보다 무겁게: Rapier 는 닿는 물체가 가벼우면 바닥에 1.5cm 파묻힌다
const PERIOD = 0.6;                                  // 걸음 시계 (spotkin 트롯과 같음)
const G_ROBOT = (0x0002 << 16) | 0x0001, G_GROUND = (0x0001 << 16) | 0xffff;   // 로봇끼리는 안 부딪힘

// ── 사원수·행렬 ──
export function quatOf(M) {
  const m00 = M[0], m01 = M[1], m02 = M[2], m10 = M[4], m11 = M[5], m12 = M[6], m20 = M[8], m21 = M[9], m22 = M[10];
  const tr = m00 + m11 + m22; let x, y, z, w;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = s / 4; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s; }
  else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = s / 4; y = (m01 + m10) / s; z = (m02 + m20) / s; }
  else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = s / 4; z = (m12 + m21) / s; }
  else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = s / 4; }
  return { x, y, z, w };
}
const qmul = (a, b) => ({ w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                          y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w });
const qconj = (q) => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w });
export function qrot(q, v) {                         // 사원수로 벡터 돌리기
  const { x, y, z, w } = q, tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}
const qinvrot = (q, v) => qrot(qconj(q), v);
const r3 = (M) => [M[0], M[1], M[2], M[4], M[5], M[6], M[8], M[9], M[10]];
const mm3 = (A, B) => { const C = new Array(9); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j]; return C; };
const quat3 = (R) => quatOf([R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0, 0, 0, 0, 1]);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

// 몸 만들기 정보 (한 번만 계산해 모든 연습장이 같이 쓴다)
export function spotBlueprint(model, shapes, opt = {}) {
  const ptsOf = (l) => shapes.L ? shapes.L[l]?.p : shapes[l]?.pts;   // shapes.json 그대로 또는 collide.js 의 Shapes
  const robot = new Robot(model); new Walker(robot);  // 선 자세 다리 각도 (spotkin 이 푼 값)
  for (const [k, v] of Object.entries(STOW)) robot.q[k] = v;
  robot.q.arm_f1x = GRIP_CLOSED;
  const Q0 = JN.map(n => robot.q[n]);
  const W = robot.fk(poseT({ x: 0, y: 0, z: Z0, roll: 0, pitch: 0, yaw: 0 }));
  const legLinks = new Set(LEGS.flatMap(l => [`${l}_hip`, `${l}_upper_leg`, `${l}_lower_leg`]));
  // 링크 → 물체: 다리 링크는 제 물체, 나머지(팔·레일)는 몸통에 붙임
  const owner = (link) => legLinks.has(link) ? link : model.root;
  const kindOf = (link) => link === model.root ? "body" : legLinks.has(link) ? link.replace(/^(front|rear)_(left|right)_/, "") : /^arm/.test(link) ? "arm" : null;
  const armLinks = Object.keys(W).filter(l => kindOf(l) === "arm" && ptsOf(l)?.length);
  const bodies = [model.root, ...legLinks].map(name => ({ name, o: [W[name][3], W[name][7], W[name][11]], Rs: r3(W[name]) }));
  const colliders = [], armCenters = {};
  for (const link of Object.keys(W)) {
    const kind = kindOf(link); if (!kind) continue;
    const b = owner(link), o = W[b], M = W[link];
    const tr = [M[3] - o[3], M[7] - o[7], M[11] - o[11]];
    if (kind === "lower_leg") {                       // 아랫다리 = 정강이 막대 + 발 공 (손으로 만든 단순 모양)
      const foot = [M[2] * -FOOT_L, M[6] * -FOOT_L, M[10] * -FOOT_L], len = Math.hypot(...foot), d = foot.map(v => v / len);
      const h = (len - FOOT_R) / 2 - 0.03, c = d.map((v, i) => v * (len - FOOT_R) / 2);
      // 막대(y 축) 를 다리 방향으로: y → d 회전
      const ax = cross([0, 1, 0], d), s = Math.hypot(...ax), cth = d[1];
      const rot = s < 1e-9 ? { x: 0, y: 0, z: 0, w: 1 } : (() => { const ang = Math.atan2(s, cth), k = Math.sin(ang / 2) / s; return { x: ax[0] * k, y: ax[1] * k, z: ax[2] * k, w: Math.cos(ang / 2) }; })();
      colliders.push({ body: b, type: "capsule", h, r: 0.02, tr: c, rot, mass: MASS.lower_leg });
      colliders.push({ body: b, type: "ball", r: FOOT_R, tr: foot, rot: { x: 0, y: 0, z: 0, w: 1 }, mass: MASS.foot, foot: link.replace("_lower_leg", "") });
      continue;
    }
    if (!ptsOf(link)?.length) continue;
    const mass = kind === "arm" ? (opt.armMass ?? MASS.arm) / armLinks.length : MASS[kind];
    // 단순 모양(상자)도 같이 둔다: 링크 좌표에서 점들을 감싸는 상자 → 계산이 훨씬 가볍다
    const P = ptsOf(link), lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < P.length; i += 3) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], P[i + k]); hi[k] = Math.max(hi[k], P[i + k]); }
    const cl = lo.map((v, k) => (v + hi[k]) / 2), half = lo.map((v, k) => Math.max(0.005, (hi[k] - v) / 2));
    const cw = [M[0] * cl[0] + M[1] * cl[1] + M[2] * cl[2], M[4] * cl[0] + M[5] * cl[1] + M[6] * cl[2], M[8] * cl[0] + M[9] * cl[1] + M[10] * cl[2]];
    colliders.push({ body: b, type: "hull", pts: new Float32Array(P), tr, rot: quatOf(M), mass, box: { tr: tr.map((v, k) => v + cw[k]), half } });
    if (kind === "arm") armCenters[link] = cl;
  }
  const joints = JN.map((n, i) => {
    const j = robot.byName[n], M = W[j.child], a = j.axis;
    const ax = [M[0] * a[0] + M[1] * a[1] + M[2] * a[2], M[4] * a[0] + M[5] * a[1] + M[6] * a[2], M[8] * a[0] + M[9] * a[1] + M[10] * a[2]];
    return { name: n, parent: owner(j.parent), child: j.child, at: [M[3], M[7], M[11]], axis: ax,
             lo: j.lower - Q0[i], hi: j.upper - Q0[i], tmax: TMAX[n.replace(/^(front|rear)_(left|right)_/, "")] };
  });
  const feet = LEGS.map(l => { const M = W[`${l}_lower_leg`]; return { leg: l, body: `${l}_lower_leg`, off: [M[2] * -FOOT_L, M[6] * -FOOT_L, M[10] * -FOOT_L] }; });
  // 화면용: 링크마다 붙은 물체와 그 물체 기준 위치·회전 (선 자세에서 잰 값)
  const links = Object.keys(model.links).filter(l => W[l]).map(name => {
    const b = owner(name), o = W[b], M = W[name];
    return { name, body: b, off: [M[3] - o[3], M[7] - o[7], M[11] - o[11]], Rs: r3(M) };
  });
  const bp = { robot, Q0, W, bodies, colliders, joints, feet, links, root: model.root, armCenters };
  bp.armStow = armLoad(bp, W).p;                     // 접은 팔의 무게중심 (몸통 좌표)
  return bp;
}
// 팔 무게중심 (몸통 좌표) · 팔 무게 — W = 링크 월드 행렬 (팔 자세가 들어간 FK). 앱이 매 장면 이걸로 '팔 무게 점' 을 옮긴다
export function armLoad(bp, W, book = 0) {
  const B = W[bp.root]; let m = 0; const c = [0, 0, 0];
  for (const [l, cl] of Object.entries(bp.armCenters)) {
    const M = W[l], mm = ARM_MASS[l] ?? 0; if (!M || !mm) continue;
    const p = [M[0] * cl[0] + M[1] * cl[1] + M[2] * cl[2] + M[3], M[4] * cl[0] + M[5] * cl[1] + M[6] * cl[2] + M[7], M[8] * cl[0] + M[9] * cl[1] + M[10] * cl[2] + M[11]];
    for (let k = 0; k < 3; k++) c[k] += p[k] * mm; m += mm;
  }
  const w = c.map(v => v / m), d = [w[0] - B[3], w[1] - B[7], w[2] - B[11]];
  return { p: [B[0] * d[0] + B[4] * d[1] + B[8] * d[2], B[1] * d[0] + B[5] * d[1] + B[9] * d[2], B[2] * d[0] + B[6] * d[1] + B[10] * d[2]], m: m - ARM_SHELL + book };
}

// 여러 연습장이 쓰는 난수 (같은 씨앗 → 같은 결과)
export function rng(seed) { let s = seed >>> 0 || 1; return () => { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s / 4294967296; }; }

export class SpotEnv {
  constructor(R, bp, opt = {}) {
    this.R = R; this.bp = bp; this.rand = rng(opt.seed ?? 1);
    this.train = opt.train ?? true;                  // 연습 중: 명령 바꾸기·밀기·짐 바꾸기·넘어지면 끝
    this.maxT = opt.maxT ?? 20; this.ver = opt.ver ?? 1; this.nobs = OBS_OF[this.ver];
    this.kp = opt.kp ?? GAIN.kp; this.kd = opt.kd ?? GAIN.kd; this.multibody = opt.multibody ?? false;   // 충격식 이음: 막 흔들어도 벌어짐 3.6mm, 다관절보다 1.5배 빠름
    const w = this.world = new R.World({ x: 0, y: 0, z: -9.81 });
    w.timestep = DT;
    if (opt.iters) w.integrationParameters.numSolverIterations = opt.iters;
    if (opt.pgs) w.integrationParameters.numInternalPgsIterations = opt.pgs;
    if (opt.allowErr) w.integrationParameters.normalizedAllowedLinearError = opt.allowErr;
    const g = w.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(0, 0, -0.1));
    this.ground = w.createCollider(R.ColliderDesc.cuboid(200, 200, 0.1).setFriction(0.8).setCollisionGroups(G_GROUND), g);
    this.body = new Map();
    for (const b of bp.bodies) {
      const rb = w.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(...b.o).setCanSleep(false)
        .setAdditionalMassProperties(0.01, { x: 0, y: 0, z: 0 }, { x: 1e-5, y: 1e-5, z: 1e-5 }, { w: 1, x: 0, y: 0, z: 0 }));
      this.body.set(b.name, rb);
    }
    this.footCol = {};
    for (const c of bp.colliders) {
      const box = c.type === "hull" && (opt.simple ?? true);   // 기본 = 상자 (껍질은 4배 느림, 걷기엔 발·정강이만 땅에 닿는다)
      const d = box ? R.ColliderDesc.cuboid(...c.box.half) : c.type === "hull" ? R.ColliderDesc.convexHull(c.pts) : c.type === "ball" ? R.ColliderDesc.ball(c.r) : R.ColliderDesc.capsule(c.h, c.r);
      if (!d) continue;
      const col = w.createCollider(d.setTranslation(...(box ? c.box.tr : c.tr)).setRotation(c.rot).setMass(c.mass).setFriction(c.foot ? 0.9 : 0.5).setCollisionGroups(G_ROBOT), this.body.get(c.body));
      if (c.foot) this.footCol[c.foot] = col; else (this.nonFoot ??= []).push(col);
    }
    // 짐 (책을 든 것처럼): 몸통에 붙은 보이지 않는 무게. 부딪히지 않는다
    this.load = w.createCollider(R.ColliderDesc.ball(0.03).setMass(0.001).setCollisionGroups(0), this.body.get(bp.root));
    this.jt = bp.joints.map(j => {
      const pb = this.body.get(j.parent), cb = this.body.get(j.child), po = pb.translation(), co = cb.translation();
      const a = { x: j.axis[0], y: j.axis[1], z: j.axis[2] };
      const data = R.JointData.revolute({ x: j.at[0] - po.x, y: j.at[1] - po.y, z: j.at[2] - po.z }, { x: j.at[0] - co.x, y: j.at[1] - co.y, z: j.at[2] - co.z }, a);
      // 다관절 이음(뿌리→끝을 한 덩어리로 푸는 방식)은 비교용 — 벌어짐 0 이지만 1.5배 느림
      if (this.multibody) w.createMultibodyJoint(data, pb, cb, true).setContactsEnabled(false);
      else { const ij = w.createImpulseJoint(data, pb, cb, true); ij.setLimits(j.lo, j.hi); }
      return { ...j, pb, cb };
    });
    this.root = this.body.get(bp.root);
    this.legBodies = bp.bodies.map(b => this.body.get(b.name));
    this.q = new Float32Array(ACT); this.qd = new Float32Array(ACT); this.qdPrev = new Float32Array(ACT);
    this.act = new Float32Array(ACT); this.actPrev = new Float32Array(ACT); this.tau = new Float32Array(ACT);
    this.target = new Float32Array(ACT);
    this.cmd = [0, 0, 0]; this.obsBuf = new Float32Array(this.nobs);
    this.bodyGoal = { z: Z0, pitch: 0 }; this.bodyNow = { z: Z0, pitch: 0 };
    this.loadGoal = bp.armStow.slice(); this.loadAt = bp.armStow.slice(); this.loadMass = 0.001;
    this.air = new Float32Array(4); this.contact = new Uint8Array(4);
    this.terms = {};
    this.reset();
  }

  // 앱용 (v2): 팔 무게 점을 몸통 좌표 p 에, 무게 m (팔 나머지 + 든 책)
  setArmLoad(p, m) { this.loadAt = p.slice(); this.loadMass = m; this.load.setMass(Math.max(0.001, m)); this.load.setTranslationWrtParent({ x: p[0], y: p[1], z: p[2] }); }
  // 화면용 링크 행렬 (4×4 행 우선): 물체 회전 × (선 자세 링크 회전), 위치 = 물체 위치 + 물체 회전 × 떨어진 거리
  linkMatrices(out = {}) {
    for (const L of this.bp.links) {
      const rb = this.body.get(L.body), t = rb.translation(), q = rb.rotation();
      const { x, y, z, w } = q, Rb = [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)];
      const R = mm3(Rb, L.Rs), o = qrot(q, L.off);
      out[L.name] = [R[0], R[1], R[2], t.x + o[0], R[3], R[4], R[5], t.y + o[1], R[6], R[7], R[8], t.z + o[2], 0, 0, 0, 1];
    }
    return out;
  }
  // 관절 각도·각속도 읽기 (선 자세 = 0)
  readJoints() {
    for (let i = 0; i < ACT; i++) {
      const j = this.jt[i], qp = j.pb.rotation(), qc = j.cb.rotation(), rel = qmul(qconj(qp), qc);
      const s = rel.x * j.axis[0] + rel.y * j.axis[1] + rel.z * j.axis[2];
      let ang = 2 * Math.atan2(s, rel.w); if (ang > Math.PI) ang -= 2 * Math.PI; if (ang < -Math.PI) ang += 2 * Math.PI;
      this.q[i] = ang;
      const wp = j.pb.angvel(), wc = j.cb.angvel(), aw = qrot(qp, j.axis);
      this.qd[i] = (wc.x - wp.x) * aw[0] + (wc.y - wp.y) * aw[1] + (wc.z - wp.z) * aw[2];
    }
  }
  // 몸 자세를 통째로 놓기: q = 관절(선 자세 기준), 몸통 위치·방향
  place(q, pos, yaw, tilt = [0, 0]) {
    const bp = this.bp, robot = bp.robot;
    const save = JN.map(n => robot.q[n]);
    JN.forEach((n, i) => { robot.q[n] = bp.Q0[i] + q[i]; });
    const W = robot.fk(poseT({ x: pos[0], y: pos[1], z: pos[2], roll: tilt[0], pitch: tilt[1], yaw }));
    JN.forEach((n, i) => { robot.q[n] = save[i]; });
    for (const b of bp.bodies) {
      const rb = this.body.get(b.name), M = W[b.name], Rw = mm3(r3(M), transpose3(b.Rs));   // 물체 회전 = 링크 회전 × (선 자세 링크 회전)⁻¹
      rb.setTranslation({ x: M[3], y: M[7], z: M[11] }, true); rb.setRotation(quat3(Rw), true);
      rb.setLinvel({ x: 0, y: 0, z: 0 }, true); rb.setAngvel({ x: 0, y: 0, z: 0 }, true);
    }
  }
  reset() {
    const r = this.rand, q = new Float32Array(ACT);
    if (this.train) for (let i = 0; i < ACT; i++) q[i] = (r() - 0.5) * 0.2;
    // 발이 바닥에 닿게 높이를 맞춘다: 발 끝 중 가장 낮은 것이 FOOT_R
    this.place(q, [0, 0, Z0], this.train ? (r() - 0.5) * 2 * Math.PI : 0);
    let lo = Infinity; for (const f of this.bp.feet) lo = Math.min(lo, this.footPos(f)[2]);
    const dz = FOOT_R + 0.005 - lo;
    for (const rb of this.legBodies) { const t = rb.translation(); rb.setTranslation({ x: t.x, y: t.y, z: t.z + dz }, true); }
    if (this.train) this.ground.setFriction(0.4 + 0.8 * r());
    this.bodyGoal = { z: Z0, pitch: 0 }; this.bodyNow = { z: Z0, pitch: 0 };
    if (this.ver === 1) {
      if (this.train) {
        const m = r() < 0.5 ? 0 : 3 * r();             // 짐: 절반은 빈 몸, 절반은 0~3kg
        this.load.setMass(Math.max(0.001, m));
        this.load.setTranslationWrtParent({ x: -0.1 + 0.4 * r(), y: (r() - 0.5) * 0.1, z: 0.1 * r() });
      }
    } else {                                          // v2: 팔 무게 점 = 접은 팔 자리, 팔 나머지 5.5kg + 책 (연습 땐 0~2.5kg)
      const stow = this.bp.armStow;
      this.setArmLoad(stow, (MASS.arm - ARM_SHELL) + (this.train && r() < 0.5 ? 2.5 * r() : 0));
      this.loadGoal = stow.slice(); this.nextLoad = 1 + 3 * r();
    }
    this.odo = [0, 0, 0];
    this.t = 0; this.phase = 0; this.nextCmd = 0; this.nextPush = 3 + 5 * r();
    this.act.fill(0); this.actPrev.fill(0); this.air.fill(0); this.contact.fill(1);
    this.readJoints(); this.qdPrev.set(this.qd);
    if (this.train) this.newCmd(); else this.cmd = [0, 0, 0];
    for (const k in this.terms) this.terms[k] = 0;
    this.epRet = 0;
    return this.observe();
  }
  desync() { this.t = this.rand() * this.maxT; this.nextCmd = this.t + 4 * this.rand(); this.nextPush = this.t + 3 + 5 * this.rand(); this.nextLoad = this.t + 3 * this.rand(); }
  newCmd() {
    const r = this.rand;
    if (this.ver >= 2) {                            // 작업 자세(서서 높이·숙임) 25% · 그냥 서기 15% · 걷기 60%
      const u = r();
      if (u < (this.ver >= 3 ? 0.3 : 0.4)) {
        this.cmd = [0, 0, 0];
        if (u < 0.25) { const z = 0.34 + 0.22 * r(); this.bodyGoal = { z, pitch: -0.1 + (z < 0.46 ? 0.5 : 0.3) * r() }; }
        else this.bodyGoal = { z: Z0, pitch: 0 };
      } else {
        this.cmd = [-0.5 + 1.3 * r(), (r() - 0.5) * 0.8, (r() - 0.5) * 2.0];
        const fine = this.ver >= 4 ? r() : 1;
        if (fine < 0.1) this.cmd = [0, 0, Math.sign(r() - 0.5) * (0.1 + 0.25 * r())];          // v4: 제자리 조금 돌기 (앱이 마지막 몇 도를 맞출 때)
        else if (fine < 0.2) { const a = 2 * Math.PI * r(), v = 0.03 + 0.07 * r(); this.cmd = [v * Math.cos(a), v * Math.sin(a), 0]; }   // v4: 조금 옮기기
        else if (r() < 0.3) {                        // 느린 맞춤 걸음 (앱이 마지막 몇 cm 를 0.02~0.15m/s 로 다가간다)
          const k = (0.02 + 0.13 * r()) / (Math.hypot(this.cmd[0], this.cmd[1]) || 1); this.cmd[0] *= k; this.cmd[1] *= k; this.cmd[2] *= 0.2 * r();
        }
        const b = r();
        if (this.ver >= 5 && b < 0.25) {             // v5: 숙인 채 천천히 (책을 칸에서 빼며 뒤로 물러나기 — 앱이 몸 높이를 그대로 둔 채 25cm 뒤로)
          const z = 0.36 + 0.10 * r(); this.bodyGoal = { z, pitch: (z < 0.42 ? 0.35 : 0.2) * r() };
          const lin = Math.hypot(this.cmd[0], this.cmd[1]); if (lin > 0.2) { this.cmd[0] *= 0.2 / lin; this.cmd[1] *= 0.2 / lin; }
          this.cmd[2] = Math.max(-0.4, Math.min(0.4, this.cmd[2]));
        } else this.bodyGoal = b < (this.ver >= 5 ? 0.6 : 0.7) ? { z: Z0, pitch: 0 } : { z: 0.44 + 0.11 * r(), pitch: -0.05 + 0.15 * r() };
      }
      this.nextCmd = this.t + 4;
      return;
    }
    if (r() < 0.2) this.cmd = [0, 0, 0];
    else {
      this.cmd = [-0.5 + 1.3 * r(), (r() - 0.5) * 0.8, (r() - 0.5) * 2.0];
      if (Math.hypot(this.cmd[0], this.cmd[1]) < 0.1) { this.cmd[0] = 0; this.cmd[1] = 0; }
      if (Math.abs(this.cmd[2]) < 0.1) this.cmd[2] = 0;
    }
    this.nextCmd = this.t + 4;
  }
  footPos(f) { const rb = this.body.get(f.body), t = rb.translation(), o = qrot(rb.rotation(), f.off); return [t.x + o[0], t.y + o[1], t.z + o[2]]; }
  footVel(f) {
    const rb = this.body.get(f.body), v = rb.linvel(), w = rb.angvel(), o = qrot(rb.rotation(), f.off);
    const c = cross([w.x, w.y, w.z], o); return [v.x + c[0], v.y + c[1], v.z + c[2]];
  }
  moving() { return Math.abs(this.cmd[0]) + Math.abs(this.cmd[1]) + Math.abs(this.cmd[2]) > (this.ver >= 2 ? 0.005 : 0.05); }   // v2: 아주 느린 명령도 걷기
  observe() {
    const o = this.obsBuf, rot = this.root.rotation(), v = this.root.linvel(), w = this.root.angvel();
    const vb = qinvrot(rot, [v.x, v.y, v.z]), wb = qinvrot(rot, [w.x, w.y, w.z]), g = qinvrot(rot, [0, 0, -1]);
    let k = 0;
    o[k++] = vb[0] * 2; o[k++] = vb[1] * 2; o[k++] = vb[2] * 2;
    o[k++] = wb[0] * 0.25; o[k++] = wb[1] * 0.25; o[k++] = wb[2] * 0.25;
    o[k++] = g[0]; o[k++] = g[1]; o[k++] = g[2];
    o[k++] = this.cmd[0] * 2; o[k++] = this.cmd[1] * 2; o[k++] = this.cmd[2] * 0.25;
    for (let i = 0; i < ACT; i++) o[k++] = this.q[i];
    for (let i = 0; i < ACT; i++) o[k++] = this.qd[i] * 0.05;
    for (let i = 0; i < ACT; i++) o[k++] = this.act[i];
    o[k++] = (this.root.translation().z - Z0) * 5;
    const m = this.moving() ? 1 : 0;
    o[k++] = m * Math.sin(2 * Math.PI * this.phase); o[k++] = m * Math.cos(2 * Math.PI * this.phase);
    if (this.ver >= 2) { o[k++] = (this.bodyNow.z - Z0) * 5; o[k++] = this.bodyNow.pitch * 2; }
    return o;
  }
  // 한 걸음 (AI 60번/초 한 번 = 물리 4번). a = 관절 12개 (-3 ~ 3)
  step(a) {
    for (let i = 0; i < ACT; i++) { this.actPrev[i] = this.act[i]; this.act[i] = Math.max(-3, Math.min(3, a[i])); this.target[i] = this.act[i] * ACT_SCALE; }
    let tauSq = 0;
    for (let s = 0; s < DECIM; s++) {
      this.readJoints();
      for (const rb of this.legBodies) rb.resetTorques(false);
      for (let i = 0; i < ACT; i++) {
        const j = this.jt[i];
        let tq = this.kp * (this.target[i] - this.q[i]) - this.kd * this.qd[i];
        if (this.multibody) { if (this.q[i] > j.hi) tq -= 2000 * (this.q[i] - j.hi) + 20 * Math.max(0, this.qd[i]); else if (this.q[i] < j.lo) tq += 2000 * (j.lo - this.q[i]) - 20 * Math.min(0, this.qd[i]); }
        tq = Math.max(-j.tmax, Math.min(j.tmax, tq)); this.tau[i] = tq; tauSq += tq * tq / DECIM;
        const aw = qrot(j.pb.rotation(), j.axis);    // 다관절은 순간 충격을 무시한다 → 한 물리 걸음 동안 가해지는 힘으로
        j.cb.addTorque({ x: aw[0] * tq, y: aw[1] * tq, z: aw[2] * tq }, true);
        j.pb.addTorque({ x: -aw[0] * tq, y: -aw[1] * tq, z: -aw[2] * tq }, true);
      }
      this.world.step();
    }
    this.t += CTRL_DT; this.phase = (this.phase + CTRL_DT / PERIOD) % 1;
    this.readJoints();
    if (this.ver >= 2) {
      const toward = (a, b, m) => a + Math.max(-m * CTRL_DT, Math.min(m * CTRL_DT, b - a));
      this.bodyNow.z = toward(this.bodyNow.z, this.bodyGoal.z, 0.18); this.bodyNow.pitch = toward(this.bodyNow.pitch, this.bodyGoal.pitch, 0.5);
      if (this.train) {                              // 팔이 움직이는 것처럼: 무게 점이 새 자리로 0.6m/s
        const r = this.rand;
        if (this.t >= this.nextLoad) {
          // 팔 무게중심 실측: 앞 0.23~0.68 · 위 0.23~0.48 (11-02). v3: 걷는 중엔 앱처럼 접음 또는 책 든 자세(앞 0.5 · 위 0.3) 근처만
          if (this.ver >= 3 && this.moving() && this.bodyGoal.z > 0.465) this.loadGoal = r() < 0.5 ? this.bp.armStow.slice() : [0.45 + 0.15 * r(), (r() - 0.5) * 0.1, 0.2 + 0.15 * r()];
          else this.loadGoal = r() < 0.4 ? this.bp.armStow.slice() : [0.75 * r(), (r() - 0.5) * 0.5, -0.2 + 0.7 * r()];
          this.nextLoad = this.t + 1.5 + 3.5 * r();
        }
        const d = this.loadGoal.map((v, k) => v - this.loadAt[k]), n = Math.hypot(...d), st = Math.min(n, 0.6 * CTRL_DT);
        if (n > 1e-6) this.setArmLoad(this.loadAt.map((v, k) => v + d[k] / n * st), this.loadMass);
      }
    }
    if (this.train && this.t >= this.nextCmd) this.newCmd();
    if (this.train && this.t >= this.nextPush) {     // 가끔 밀기 (사람이 툭 치는 것처럼)
      const r = this.rand, v = this.root.linvel();
      this.root.setLinvel({ x: v.x + (r() - 0.5) * 1.0, y: v.y + (r() - 0.5) * 1.0, z: v.z }, true);
      this.nextPush = this.t + 4 + 5 * r();
    }
    // ── 점수 ──
    const rot = this.root.rotation(), pz = this.root.translation().z, lv = this.root.linvel(), av = this.root.angvel();
    const vb = qinvrot(rot, [lv.x, lv.y, lv.z]), wb = qinvrot(rot, [av.x, av.y, av.z]), g = qinvrot(rot, [0, 0, -1]);
    const c = this.cmd, mv = this.moving(), dt = CTRL_DT;
    const T = {};
    // v3: 따라가기를 엄격하게 (v2 는 0.4 를 0.2 로 걸어도 15% 만 잃어서 느리게 걷는 게 이득이었다)
    const strict = this.ver >= 3, sg = strict ? 0.1 : 0.25;
    T.track_v = (strict ? 2.0 : 1.5) * Math.exp(-((c[0] - vb[0]) ** 2 + (c[1] - vb[1]) ** 2) / sg);
    T.track_w = (strict ? 1.2 : 0.8) * Math.exp(-((c[2] - wb[2]) ** 2) / sg);
    T.up_down = -2.0 * vb[2] ** 2;
    T.wobble = -0.05 * (wb[0] ** 2 + wb[1] ** 2);
    if (this.ver >= 2) {                            // 몸 높이·숙임을 명령대로 (옆 기울기는 0)
      T.tilt = -2.0 * g[1] ** 2;
      T.height = 1.0 * Math.exp(-((pz - this.bodyNow.z) ** 2) / 0.0016);
      T.pitch = 0.8 * Math.exp(-((g[0] - Math.sin(this.bodyNow.pitch)) ** 2) / 0.01);
    } else {
      T.tilt = -2.0 * (g[0] ** 2 + g[1] ** 2);
      T.height = -10 * (pz - Z0) ** 2;
    }
    T.torque = -1e-5 * tauSq;
    if (this.ver >= 6) {                             // v6: 발 말고 다른 데(무릎·허벅지·몸통)가 바닥에 닿으면 감점 (v5 는 깊게 숙일 때 앞 무릎을 꿇었다)
      let touch = 0;
      for (const col of this.nonFoot) this.world.contactPair(this.ground, col, (m) => { if (m.numContacts() > 0) touch++; });
      T.touch = -1.0 * touch;
    }
    if (this.ver >= 4) {
      const k = Math.exp(-dt / 1.0), o = this.odo ?? (this.odo = [0, 0, 0]);
      o[0] = o[0] * k + (c[0] - vb[0]) * dt; o[1] = o[1] * k + (c[1] - vb[1]) * dt; o[2] = o[2] * k + (c[2] - wb[2]) * dt;
      T.odo = 1.0 * Math.exp(-(o[0] ** 2 + o[1] ** 2) / 0.0025 - o[2] ** 2 / 0.01);
    }
    let acc = 0, rate = 0, still = 0, lim = 0;
    for (let i = 0; i < ACT; i++) {
      acc += ((this.qd[i] - this.qdPrev[i]) / dt) ** 2; rate += (this.act[i] - this.actPrev[i]) ** 2; still += Math.abs(this.q[i]);
      const j = this.jt[i], soft = 0.9; lim += Math.max(0, this.q[i] - j.hi * soft) + Math.max(0, j.lo * soft - this.q[i]);
    }
    this.qdPrev.set(this.qd);
    T.joint_acc = -2.5e-7 * acc; T.jerky = -0.01 * rate; T.limits = -5 * lim;
    T.stand_still = mv || this.ver >= 2 ? 0 : -0.5 * still;   // v2 는 높이·숙임 때문에 관절이 선 자세와 달라도 된다 → 발이 안 움직이는 것으로
    let air = 0, slip = 0, gait = 0, feetMove = 0;
    const stanceA = Math.sin(2 * Math.PI * this.phase) >= 0;
    this.bp.feet.forEach((f, i) => {
      const p = this.footPos(f), on = p[2] < FOOT_R + 0.01;
      if (on && !this.contact[i] && mv) air += this.air[i] - 0.25;
      this.air[i] = on ? 0 : this.air[i] + dt; this.contact[i] = on ? 1 : 0;
      const fv = this.footVel(f); if (on) slip += fv[0] ** 2 + fv[1] ** 2; feetMove += Math.hypot(fv[0], fv[1]);
      const groupA = f.leg === "front_left" || f.leg === "rear_right";
      const want = !mv || (groupA ? stanceA : !stanceA);
      gait += (on === want) ? 0.25 : 0;
    });
    T.air_time = 0.5 * air / dt; T.slip = -0.1 * slip; T.gait = 0.3 * gait;
    if (this.ver >= 2 && !mv) T.feet_still = -0.5 * feetMove;
    let rew = 0; for (const k in T) { rew += T[k] * dt; this.terms[k] = (this.terms[k] ?? 0) + T[k] * dt; }
    const fell = pz < (this.ver >= 2 ? 0.25 : 0.28) || g[2] > -0.5 || !isFinite(pz);
    const timeout = !fell && this.t >= this.maxT;
    if (fell) { rew -= 2; this.terms.fall = (this.terms.fall ?? 0) - 2; }
    this.epRet += rew;
    return { obs: this.observe(), rew, done: fell || timeout, timeout, fell };
  }
}
function transpose3(R) { return [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]]; }
