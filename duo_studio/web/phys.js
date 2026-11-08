// 책·책꽂이 물리 (11-02) — Rapier(강체 물리 엔진, Apache-2.0, vendor/rapier). 브라우저·Node 같은 코드.
//   물리를 쓰는 건 책뿐이다: 책 = 움직이는 상자, 책꽂이·바닥 = 고정 상자, 로봇 손 = '계산으로 움직이는' 물체(키네마틱).
//   로봇은 지금처럼 계산으로 움직이고, 손이 책을 밀거나 누르면 물리가 책을 움직인다.
//   잡은 책은 손에 붙여(키네마틱) 같이 움직이고, 놓으면 다시 물리로 돌린다.
import RAPIER from "./vendor/rapier/rapier.es.js";

let ready = null;
export const initPhysics = () => (ready = ready ?? RAPIER.init().then(() => RAPIER));

function quatOf(M) {                                  // 회전 행렬(4×4 행 우선) → 사원수
  const m00 = M[0], m01 = M[1], m02 = M[2], m10 = M[4], m11 = M[5], m12 = M[6], m20 = M[8], m21 = M[9], m22 = M[10];
  const tr = m00 + m11 + m22; let x, y, z, w;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = s / 4; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s; }
  else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = s / 4; y = (m01 + m10) / s; z = (m02 + m20) / s; }
  else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = s / 4; z = (m12 + m21) / s; }
  else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = s / 4; }
  return { x, y, z, w };
}
function matOf(t, q) {
  const { x, y, z, w } = q;
  return [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x,
          2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y,
          2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z, 0, 0, 0, 1];
}

export class BookPhysics {
  // statics: [{lo, hi}] 월드 축 맞춤 상자 (정렬대 등), books: [{T, d, t, h, mass}]
  // opt.shelves: [{id, T, panels:[{lo, hi}]}] — 책꽂이 하나 = 고정 물체 하나 (판들은 책꽂이 좌표 상자). 사용자가 끌어 옮기면 setShelf
  constructor(R, statics, books, opt = {}) {
    this.R = R; this.world = new R.World({ x: 0, y: 0, z: -9.81 });
    this.world.integrationParameters.numSolverIterations = opt.iters ?? 8;
    const fix = (lo, hi) => {
      const b = this.world.createRigidBody(R.RigidBodyDesc.fixed().setTranslation((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2));
      this.world.createCollider(R.ColliderDesc.cuboid((hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, (hi[2] - lo[2]) / 2).setFriction(0.5), b);
    };
    fix([-6, -6, -0.1], [6, 6, 0]);                    // 바닥
    for (const s of statics) fix(s.lo, s.hi);
    this.shelf = new Map();
    for (const sh of opt.shelves ?? []) {
      const b = this.world.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(sh.T[3], sh.T[7], sh.T[11]).setRotation(quatOf(sh.T)));
      for (const p of sh.panels) this.world.createCollider(R.ColliderDesc.cuboid((p.hi[0] - p.lo[0]) / 2, (p.hi[1] - p.lo[1]) / 2, (p.hi[2] - p.lo[2]) / 2)
        .setTranslation((p.lo[0] + p.hi[0]) / 2, (p.lo[1] + p.hi[1]) / 2, (p.lo[2] + p.hi[2]) / 2).setFriction(0.5), b);
      this.shelf.set(sh.id, b);
    }
    this.books = books; this.body = new Map();
    for (const bk of books) {
      const body = this.world.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(bk.T[3], bk.T[7], bk.T[11]).setRotation(quatOf(bk.T))
        .setLinearDamping(0.05).setAngularDamping(0.1).setCanSleep(true));
      const vol = bk.d * bk.t * bk.h;
      this.world.createCollider(R.ColliderDesc.cuboid(bk.d / 2, bk.t / 2, bk.h / 2).setDensity(bk.mass / vol).setFriction(0.45).setRestitution(0), body);
      this.body.set(bk, body);
    }
    this.kin = new Map();                             // 손 조각 이름 → 키네마틱 몸
  }
  // 로봇 손 조각: 그 링크 좌표의 점들로 볼록 껍질을 만들어 둔다. 매 프레임 setHand 로 링크 자세를 준다
  addHand(name, pts) {
    const R = this.R, body = this.world.createRigidBody(R.RigidBodyDesc.kinematicPositionBased());
    const desc = R.ColliderDesc.convexHull(new Float32Array(pts)) ?? R.ColliderDesc.ball(0.02);
    this.world.createCollider(desc.setFriction(0.8), body);
    this.kin.set(name, body);
  }
  setHand(name, M) { const b = this.kin.get(name); if (!b) return; b.setNextKinematicTranslation({ x: M[3], y: M[7], z: M[11] }); b.setNextKinematicRotation(quatOf(M)); }
  placeHand(name, M) { const b = this.kin.get(name); if (!b) return; b.setTranslation({ x: M[3], y: M[7], z: M[11] }, true); b.setRotation(quatOf(M), true); }
  // 잡기: 책을 손에 붙인다 (키네마틱) / 놓기: 다시 물리로
  hold(bk) { const b = this.body.get(bk); b.setBodyType(this.R.RigidBodyType.KinematicPositionBased, true); }
  moveHeld(bk, M) { const b = this.body.get(bk); b.setNextKinematicTranslation({ x: M[3], y: M[7], z: M[11] }); b.setNextKinematicRotation(quatOf(M)); }
  release(bk) {
    const b = this.body.get(bk); b.setBodyType(this.R.RigidBodyType.Dynamic, true);
    b.setLinvel({ x: 0, y: 0, z: 0 }, true); b.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }
  // 사용자가 옮김: 책꽂이 통째로 / 책 하나 (순간 이동, 멈춘 채로)
  setShelf(id, T) { const b = this.shelf.get(id); if (!b) return; b.setTranslation({ x: T[3], y: T[7], z: T[11] }, true); b.setRotation(quatOf(T), true); }
  placeBook(bk, T) {
    const b = this.body.get(bk); bk.T = T.slice();
    b.setTranslation({ x: T[3], y: T[7], z: T[11] }, true); b.setRotation(quatOf(T), true);
    if (b.isDynamic()) { b.setLinvel({ x: 0, y: 0, z: 0 }, true); b.setAngvel({ x: 0, y: 0, z: 0 }, true); }
  }
  wakeAll() { for (const bk of this.books) this.body.get(bk).wakeUp(); }
  step(dt) {
    this.world.timestep = dt; this.world.step();
    for (const bk of this.books) { const b = this.body.get(bk); bk.T = matOf(b.translation(), b.rotation()); }
  }
  moving(bk) { const b = this.body.get(bk), v = b.linvel(), w = b.angvel(); return Math.hypot(v.x, v.y, v.z) + 0.05 * Math.hypot(w.x, w.y, w.z); }
  resting() { return this.books.every(bk => this.body.get(bk).isSleeping() || this.moving(bk) < 0.01); }
}
