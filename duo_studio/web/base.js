// 공용 계산 (SPOT STUDIO kin.js 에서 가져옴, 11-01) — 행렬 · FK · Robot. three.js 에 기대지 않는다.
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
export const col = (M, k) => [M[k], M[4 + k], M[8 + k]];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const norm = (a) => Math.hypot(a[0], a[1], a[2]);
export const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
export const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

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
  jointT(j, q) {                                    // 회전 관절 · 미끄럼 관절(prismatic: 축 방향으로 q m)
    if (!j.movable) return j.T0;
    if (j.type === "prismatic") { const n = Math.hypot(...j.axis) || 1; return mul(j.T0, [1, 0, 0, j.axis[0] / n * q, 0, 1, 0, j.axis[1] / n * q, 0, 0, 1, j.axis[2] / n * q, 0, 0, 0, 1]); }
    return mul(j.T0, axisT(j.axis, q));
  }
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

