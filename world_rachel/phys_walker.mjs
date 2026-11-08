// 월드 레이첼 걷기 — 책꽂이 앱(spotb.js)의 '계산 걷기(Walker)' 자리에 그대로 끼우는 물리 걷기.
//   앱이 주는 것은 그대로: cmd(앞·옆·돌기 속도) · target(몸 높이·숙임). 앱이 읽는 것도 그대로: pose · vel · walking.
//   안에서는: 물리 Spot(Rapier, 1초 240번) + 학습된 두뇌(1초 60번)가 다리 12개를 움직인다.
//   팔은 지금처럼 계산(로봇의 팔 각도)으로 그리고, 팔 무게만 '팔 무게 점' 으로 물리에 넣는다 (든 책 무게 포함).
//   다리 각도는 물리에서 읽어 robot.q 에 써 준다.
//
//   두 가지 몸 자세 (11-02):
//   - phys = 물리가 놓은 실제 몸. 걷는 동안 앱의 pose 는 이것 (길 따라가기가 실제 자리를 본다)
//   - 서 있는 동안(앱 명령 0) 앱의 pose 는 '멈춘 자리 + 명령한 높이·숙임' (계획한 몸). 실제 몸은 '자리 지키기' 로 1.5cm 안에 붙어 있다.
//     팔·손 계산은 계획한 몸으로 한다 → 손은 계획대로 간다. 진짜 Spot 이 몸이 흔들려도 손을 세상에 고정하는 것(몸 전체 제어)과 같은 가정.
//     (실제 몸으로 팔을 계산하면 몸이 1cm 밀릴 때 손도 밀려 손가락이 책을 놓쳤다 — 6mm)
import { SpotEnv, spotBlueprint, armLoad, ARM_SHELL, JN, ACT, CTRL_DT, Z0 } from "./spot_env.mjs";
import { Policy } from "./policy.mjs";
import { I4, LEGS, legGeom, legIK, poseT, inv, apply } from "../duo_studio/web/spotkin.js";

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const MIN_V = 0.1, MIN_W = 0.25;                     // 두뇌가 확실히 따라가는 가장 느린 걸음 · 돌기 (더 작은 명령은 '서 있기' 로 받아들인다)
const toward = (dx, dy, yaw, ey, vmax, wmax) => {    // 그 자리로 가는 몸 기준 명령 (P 제어)
  const d = Math.hypot(dx, dy), c = Math.cos(yaw), s = Math.sin(yaw), fx = dx * c + dy * s, fy = -dx * s + dy * c, sp = Math.min(vmax, 1.5 * d);
  return { vx: d > 1e-6 ? sp * fx / d : 0, vy: d > 1e-6 ? sp * fy / d : 0, wz: clamp(1.5 * ey, -wmax, wmax) };
};

export class PhysWalker {
  // R = Rapier · robot = 앱의 Robot (팔 각도를 읽고 다리 각도를 쓴다) · model/shapes = Spot 모델 · brain = 학습된 두뇌 JSON (관찰 53)
  constructor(R, robot, model, shapes, brain, opt = {}) {
    if (brain.obs !== 53) throw new Error("월드 레이첼 v2 이상 두뇌(관찰 53)가 필요합니다");
    this.robot = robot; this.book = opt.book ?? (() => 0);
    this.bp = spotBlueprint(model, shapes, { armMass: ARM_SHELL });
    this.env = new SpotEnv(R, this.bp, { train: false, ver: 2, maxT: 1e9 });
    this.pol = new Policy(brain); this.act = new Float32Array(ACT);
    this.vmax = 0.45; this.wmax = 0.9;               // spotkin Walker 와 같은 한도 (drive 가 읽는다)
    this.cmd = { vx: 0, vy: 0, wz: 0 }; this.target = { z: Z0, pitch: 0 };
    this.vel = { vx: 0, vy: 0, wz: 0 }; this.walking = false; this.feet = {};
    this.pose = { x: 0, y: 0, z: Z0, roll: 0, pitch: 0, yaw: 0 };   // 앱이 보는 몸 (위 설명)
    this.phys = { ...this.pose };                    // 물리 몸
    this.acc = 0; this.still = 0; this.fell = false; this.calm = 0; this.last = null;
    this.hold = false; this.anchor = null; this.keeping = false; this.fine = false;
    const p = opt.pose ?? {}; this.placeAt(p.x ?? 0, p.y ?? 0, p.yaw ?? 0);
  }
  dispose() { this.env.world.free(); }
  nominal() { return [0, 0, 0]; }
  solveLegs() {}
  settled() { return !this.walking; }
  moving() { const c = this.cmd; return Math.abs(c.vx) + Math.abs(c.vy) + Math.abs(c.wz) > 1e-3; }
  // 그 자리에 바로 세운다 (발이 바닥에 닿게)
  placeAt(x, y, yaw) {
    const e = this.env; e.place(new Float32Array(ACT), [x, y, Z0], yaw);
    let lo = Infinity; for (const f of this.bp.feet) lo = Math.min(lo, e.footPos(f)[2]);
    const dz = 0.036 + 0.005 - lo; for (const rb of e.legBodies) { const t = rb.translation(); rb.setTranslation({ x: t.x, y: t.y, z: t.z + dz }, true); }
    e.act.fill(0); e.bodyGoal = { ...this.target }; e.bodyNow = { ...this.target }; e.readJoints(); e.qdPrev.set(e.qd);
    this.fell = false; this.anchor = null; this.blend = null; this.wasIdeal = undefined; this.sync();
  }
  // 도착 판정 (spotb hdrive 가 읽는다): 들어갈 땐 2cm·0.04rad (계산 걷기와 같음), 그 목표에 한 번 들어온 뒤엔 3.5cm·0.07rad 까지 '도착'.
  //   (물리 몸은 서 있어도 1~2mm 흔들려 문턱에서 '도착/아님' 이 매 장면 바뀌면, 명령이 0↔움직임으로 깜빡여 두뇌가 발을 못 뗀다)
  //   넓힌 문턱은 같은 목표에서만 — 새 목표면 처음 값 (안 그러면 5cm 물러나기를 '이미 도착' 으로 건너뛴다, 11-02)
  get tol() { return this.hold ? { d: 0.035, e: 0.07 } : { d: 0.02, e: 0.04 }; }
  aim(g) { if (g !== this.aimG) { this.aimG = g; this.hold = false; } }
  update(dt) {
    this.acc += dt;
    while (this.acc >= CTRL_DT - 1e-9) { this.acc -= CTRL_DT; this.ctrl(); }
    this.sync();
  }
  ctrl() {
    const e = this.env, P = this.phys; let c = this.cmd;
    if (!this.moving()) this.hold = true;
    // 자리 지키기: 앱이 '서 있어' 라고 하는 동안 멈춘 자리를 기억하고, 1.5cm·0.03rad 넘게 밀리면 되돌아간다 (되돌기 시작하면 6mm 까지)
    //   (팔 무게가 움직이면 몸이 조금씩 밀린다 — 실측 3초에 4cm)
    if (this.moving()) { this.anchor = null; this.keeping = false; }
    else {
      if (!this.anchor) this.anchor = { x: P.x, y: P.y, yaw: P.yaw };
      const A = this.anchor, dx = A.x - P.x, dy = A.y - P.y, d = Math.hypot(dx, dy), ey = wrap(A.yaw - P.yaw);
      this.keeping = this.keeping ? (d > 0.006 || Math.abs(ey) > 0.015) : (d > 0.015 || Math.abs(ey) > 0.03);
      if (this.keeping) c = toward(dx, dy, P.yaw, ey, 0.15, 0.4);
    }
    let vx = c.vx, vy = c.vy, wz = c.wz; const lin = Math.hypot(vx, vy);
    if (lin > 0.003 && lin < MIN_V) { vx *= MIN_V / lin; vy *= MIN_V / lin; }
    if (Math.abs(wz) > 0.01 && Math.abs(wz) < MIN_W) wz = Math.sign(wz) * MIN_W;
    e.cmd = [clamp(vx, -this.vmax, this.vmax), clamp(vy, -this.vmax, this.vmax), clamp(wz, -this.wmax, this.wmax)];
    e.bodyGoal = { ...this.target };
    const L = armLoad(this.bp, this.robot.fk(I4()), this.book()); e.setArmLoad(L.p, L.m);
    if (this.fell) this.act.fill(0); else this.pol.act(e.observe(), this.act);
    const r = e.step(this.act);
    if (r.fell) this.fell = true;
  }
  // 물리 → 앱: 몸 자세 · 속도 · 다리 각도
  sync() {
    const e = this.env, t = e.root.translation(), q = e.root.rotation(), P = this.phys;
    const { x, y, z, w } = q;                         // 사원수 → URDF rpy (R = Rz·Ry·Rx)
    P.x = t.x; P.y = t.y; P.z = t.z;
    P.roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    P.pitch = Math.asin(clamp(2 * (w * y - z * x), -1, 1));
    P.yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    // 앱이 보는 몸: 서 있으면 멈춘 자리, 높이·숙임은 자세가 다 된 뒤에만 명령값 (그 전엔 실제 — 숙이는 중에 명령값을 쓰면 다리가 바닥을 13cm 뚫어 보였다)
    const A = this.anchor, tg = this.target;
    if (this.postureFor && (this.postureFor.z !== tg.z || this.postureFor.pitch !== tg.pitch)) this.postureFor = null;
    const ok = !!this.postureFor;
    const ideal = !!(A && !this.moving());
    const want = ideal ? { x: A.x, y: A.y, yaw: A.yaw, z: ok ? tg.z : P.z, pitch: ok ? tg.pitch : P.pitch, roll: ok ? 0 : P.roll } : { ...P };
    // 계획한 몸 → 실제 몸 (걷기 시작, 책꽂이 앞에서 물러날 땐 손이 아직 칸 안): 앱의 몸(→ 손)이 최대 1.5cm 튀지 않게 차이를 0.5초에 걸쳐 줄인다.
    //   실제 → 계획 (도착) 은 바로 바꾼다 — 그 뒤 팔 계산이 바로 시작되는데, 그동안 몸이 바뀌면 손이 1cm 밀려 책을 놓쳤다 (11-02)
    if (ideal) this.blend = null;
    else if (this.wasIdeal === true) {
      const prev = this.pose; this.blend = { x: prev.x - want.x, y: prev.y - want.y, yaw: wrap(prev.yaw - want.yaw), z: prev.z - want.z, pitch: prev.pitch - want.pitch, roll: prev.roll - want.roll };
    }
    this.wasIdeal = ideal;
    const B = this.blend;
    if (B) { const k = Math.exp(-CTRL_DT / 0.5); for (const key in B) { want[key] += B[key]; B[key] *= k; } if (Math.abs(B.x) + Math.abs(B.y) + Math.abs(B.z) < 1e-4 && Math.abs(B.yaw) + Math.abs(B.pitch) + Math.abs(B.roll) < 1e-4) this.blend = null; }
    Object.assign(this.pose, want);
    if (!ideal) JN.forEach((n, i) => { this.robot.q[n] = this.bp.Q0[i] + e.q[i]; });
    else {                                            // 계획한 몸으로 그릴 땐 다리를 실제 발 자리에 맞춰 다시 푼다 (발이 바닥을 뚫거나 뜨지 않게)
      const Bi = inv(poseT(this.pose)), G = this.geom ?? (this.geom = Object.fromEntries(LEGS.map(l => [l, legGeom(this.robot, l)])));
      this.bp.feet.forEach(f => { const r = legIK(G[f.leg], apply(Bi, e.footPos(f)));
        this.robot.q[`${f.leg}_hip_x`] = r.a; this.robot.q[`${f.leg}_hip_y`] = r.b; this.robot.q[`${f.leg}_knee`] = r.c; });
    }
    const v = e.root.linvel(), av = e.root.angvel(), cy = Math.cos(P.yaw), sy = Math.sin(P.yaw);
    const fx = v.x * cy + v.y * sy, fy = -v.x * sy + v.y * cy, dead = (u, d) => Math.abs(u) < d ? 0 : u;
    // 속도는 0.25초로 고르게 (서 있어도 몸이 조금씩 흔들린다) → 작은 흔들림은 0
    const k = 1 - Math.exp(-CTRL_DT / 0.25), sm = this.sm ?? (this.sm = { vx: 0, vy: 0, wz: 0 });
    sm.vx += (fx - sm.vx) * k; sm.vy += (fy - sm.vy) * k; sm.wz += (av.z - sm.wz) * k;
    this.vel = { vx: dead(sm.vx, 0.04), vy: dead(sm.vy, 0.04), wz: dead(sm.wz, 0.08) };
    // 걷는 중 = 명령이 있거나, 몸이 아직 움직이거나, 멈춘 지 0.3초가 안 됨
    const busy = this.moving() || this.vel.vx || this.vel.vy || this.vel.wz;
    this.still = busy ? 0 : this.still + CTRL_DT;
    this.walking = this.still < 0.3;
    // 몸 자세가 멈췄나: 0.2초로 고른 높이·숙임이 1초에 1.2cm · 0.06rad 보다 느리게 바뀌는 채 0.3초
    //   (장면마다 비교하면 물리 몸의 잔떨림 때문에 멈춘 적이 없다고 나와 140초를 기다렸다 — 11-02)
    const kz = 1 - Math.exp(-CTRL_DT / 0.2), Lz = this.last;
    const sz = Lz ? Lz.z + (P.z - Lz.z) * kz : P.z, sp = Lz ? Lz.pitch + (P.pitch - Lz.pitch) * kz : P.pitch;
    this.calm = Lz && Math.abs(sz - Lz.z) < 2e-4 && Math.abs(sp - Lz.pitch) < 1e-3 ? this.calm + CTRL_DT : 0;
    this.last = { z: sz, pitch: sp };
  }
  // 몸 자세를 둔 채 자리를 다시 맞춘다 (spotb '몸 높이' 단계가 자세가 된 뒤 부른다): 2.5cm·0.05rad 안에 들어와 멈추면 true.
  //   숙이면 몸이 앞으로 밀린다 (실측 10cm). 한 번 들어오면 4cm·0.07rad 까지는 도착으로 친다 (흔들림으로 깜빡이지 않게)
  settleAt(g) {
    const P = this.phys, dx = g.x - P.x, dy = g.y - P.y, d = Math.hypot(dx, dy), e = wrap((g.yaw ?? P.yaw) - P.yaw);
    const tD = this.fine ? 0.04 : 0.025, tE = this.fine ? 0.07 : 0.05;
    if (d < tD && Math.abs(e) < tE) {
      this.cmd = { vx: 0, vy: 0, wz: 0 }; this.fine = true;
      if (!this.walking) { this.anchor = { x: g.x, y: g.y, yaw: g.yaw ?? P.yaw }; return true; }   // 지킬 자리 = 계획한 자리 (2.5cm 안)
      return false;
    }
    this.fine = false; this.cmd = toward(dx, dy, P.yaw, e, 0.15, 0.4);
    return false;
  }
  // 몸 높이·숙임이 목표에 왔나 (spotb '몸 높이' 단계가 묻는다): 명령이 다 갔고, 몸이 멈췄고, 3cm·0.1rad 안
  reached(body) {
    const P = this.phys, b = this.env.bodyNow;
    const ok = Math.abs(b.z - body.z) < 1e-3 && Math.abs(b.pitch - body.pitch) < 1e-3 && this.calm > 0.3 && !this.walking
      && Math.abs(P.z - body.z) < 0.03 && Math.abs(P.pitch - body.pitch) < 0.1;
    if (ok) this.postureFor = { ...this.target };     // 이 자세가 됐다 → 앱이 보는 몸은 명령한 높이·숙임
    return ok;
  }
}
