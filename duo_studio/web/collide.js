// 부딪힘 계산 (11-01) — shapes.json (make_shapes.py) 을 읽어서. three.js 에 기대지 않는다.
//   점 = 링크 겉면 점, 칸 = 링크 속(5mm, 겉에서 한 칸 깎음). 월드: 바닥 z=0, 도킹 = 상자들, 물건 = 상자·원기둥.
export const TOL = 0.002;

const b64 = (s) => typeof atob === "function" ? Uint8Array.from(atob(s), c => c.charCodeAt(0)) : new Uint8Array(Buffer.from(s, "base64"));
export class Shapes {
  constructor(json) {
    this.L = {};
    for (const [name, s] of Object.entries(json)) {
      const p = Float32Array.from(s.pts), v = s.vox, bits = b64(v.bits);
      let cx = 0, cy = 0, cz = 0; const n = p.length / 3;
      for (let i = 0; i < n; i++) { cx += p[3 * i]; cy += p[3 * i + 1]; cz += p[3 * i + 2]; }
      const c = [cx / n, cy / n, cz / n]; let r = 0;
      for (let i = 0; i < n; i++) r = Math.max(r, Math.hypot(p[3 * i] - c[0], p[3 * i + 1] - c[1], p[3 * i + 2] - c[2]));
      this.L[name] = { p, n, c, r, lo: v.lo, h: v.pitch, d: v.dims, bits, filled: s.filled };
    }
  }
  has(link) { return !!this.L[link]; }
  // 링크 점들을 월드로 (out 에 [x,y,z,…])
  world(link, M) {
    const s = this.L[link], o = new Float64Array(s.n * 3), p = s.p;
    for (let i = 0; i < s.n; i++) {
      const x = p[3 * i], y = p[3 * i + 1], z = p[3 * i + 2];
      o[3 * i] = M[0] * x + M[1] * y + M[2] * z + M[3]; o[3 * i + 1] = M[4] * x + M[5] * y + M[6] * z + M[7]; o[3 * i + 2] = M[8] * x + M[9] * y + M[10] * z + M[11];
    }
    return o;
  }
  center(link, M) { const c = this.L[link].c; return [M[0] * c[0] + M[1] * c[1] + M[2] * c[2] + M[3], M[4] * c[0] + M[5] * c[1] + M[6] * c[2] + M[7], M[8] * c[0] + M[9] * c[1] + M[10] * c[2] + M[11]]; }
  // 월드 점 하나가 링크(행렬 M) 속에 있나
  inside(link, M, x, y, z) {
    const s = this.L[link]; if (!s.filled) return false;
    const dx = x - M[3], dy = y - M[7], dz = z - M[11];
    const lx = M[0] * dx + M[4] * dy + M[8] * dz, ly = M[1] * dx + M[5] * dy + M[9] * dz, lz = M[2] * dx + M[6] * dy + M[10] * dz;
    const i = Math.round((lx - s.lo[0]) / s.h), j = Math.round((ly - s.lo[1]) / s.h), k = Math.round((lz - s.lo[2]) / s.h);
    if (i < 0 || j < 0 || k < 0 || i >= s.d[0] || j >= s.d[1] || k >= s.d[2]) return false;
    const idx = (i * s.d[1] + j) * s.d[2] + k;
    return (s.bits[idx >> 3] >> (7 - (idx & 7))) & 1;
  }
}

// 물건 속 깊이 (밖이면 음수 ≈ 떨어진 거리)
export function itemDepth(it, T, x, y, z) {
  const dx = x - T[3], dy = y - T[7], dz = z - T[11];
  const qx = T[0] * dx + T[4] * dy + T[8] * dz, qy = T[1] * dx + T[5] * dy + T[9] * dz, qz = T[2] * dx + T[6] * dy + T[10] * dz;
  if (it.kind === "cube") { const h = it.size / 2; return Math.min(h - Math.abs(qx), h - Math.abs(qy), h - Math.abs(qz)); }
  if (it.kind === "book") return Math.min(it.d / 2 - Math.abs(qx), it.t / 2 - Math.abs(qy), it.h / 2 - Math.abs(qz));   // 책 = 깊이·두께·높이 상자
  return Math.min(it.r - Math.hypot(qx, qy), it.h / 2 - Math.abs(qz));
}
export function itemPoints(it, T) {                 // 물건 겉 점 (모서리·테두리)
  const P = [];
  if (it.kind === "cube") { const h = it.size / 2; for (const a of [-h, 0, h]) for (const b of [-h, 0, h]) for (const c of [-h, h]) P.push([a, b, c], [a, c, b], [c, a, b]); }
  else if (it.kind === "book") { const a = it.d / 2, b = it.t / 2, c = it.h / 2; for (const x of [-a, 0, a]) for (const y of [-b, b]) for (const z of [-c, 0, c]) P.push([x, y, z]); }
  else for (let k = 0; k < 16; k++) for (const z of [-it.h / 2, 0, it.h / 2]) P.push([it.r * Math.cos(k * Math.PI / 8), it.r * Math.sin(k * Math.PI / 8), z]);
  return P.map(p => [T[0] * p[0] + T[1] * p[1] + T[2] * p[2] + T[3], T[4] * p[0] + T[5] * p[1] + T[6] * p[2] + T[7], T[8] * p[0] + T[9] * p[1] + T[10] * p[2] + T[11]]);
}
// 상자 안으로 들어간 깊이. 상자에 Ti(월드→상자 좌표 행렬)가 있으면 돌아간 상자 (아무 각도로 놓인 책꽂이 판)
export const boxDepth = (b, x, y, z) => {
  if (b.Ti) { const M = b.Ti, X = M[0] * x + M[1] * y + M[2] * z + M[3], Y = M[4] * x + M[5] * y + M[6] * z + M[7]; z = M[8] * x + M[9] * y + M[10] * z + M[11]; x = X; y = Y; }
  return Math.min(x - b.lo[0], b.hi[0] - x, y - b.lo[1], b.hi[1] - y, z - b.lo[2], b.hi[2] - z);
};
export const itemRadius = (it) => it.kind === "cube" ? it.size * 0.87 : it.kind === "book" ? Math.hypot(it.d, it.t, it.h) / 2 : Math.hypot(it.r, it.h / 2);

// 지금 자세의 부딪힘 전부. cfg = { links:[검사할 링크], boxes, self:[[팔 링크들],[몸 링크들]], hand:Set, grip:물건(손가락 닿아도 되는), skipItems:Set }
//   → [{kind, who, depth}]   (depth: m, 칸 검사는 들어간 점 수 × 1mm)
export function scan(sh, W, items, held, cfg) {
  const out = [], add = (kind, who, depth) => out.push({ kind, who, depth });
  const pts = {};
  const P = (l) => pts[l] || (pts[l] = sh.world(l, W[l]));
  for (const l of cfg.links) {
    if (!sh.has(l) || !W[l]) continue;
    const s = sh.L[l], c = sh.center(l, W[l]), o = P(l), n = s.n;
    if (c[2] - s.r < TOL) { let m = 0; for (let i = 0; i < n; i++) m = Math.min(m, o[3 * i + 2]); if (m < -TOL) add("바닥", l, -m); }
    for (const b of cfg.boxes) {
      const lo = b.alo ?? b.lo, hi = b.ahi ?? b.hi;            // 걸러내기는 월드 축 맞춤 상자로
      if (c[0] + s.r < lo[0] || c[0] - s.r > hi[0] || c[1] + s.r < lo[1] || c[1] - s.r > hi[1] || c[2] + s.r < lo[2] || c[2] - s.r > hi[2]) continue;
      let d = 0; for (let i = 0; i < n; i++) d = Math.max(d, boxDepth(b, o[3 * i], o[3 * i + 1], o[3 * i + 2])); if (d > TOL) add("도킹", `${l}|${b.name}`, d);
    }
    for (const it of items) {
      if (it === held || cfg.skipItems?.has(it)) continue;
      if (Math.hypot(c[0] - it.T[3], c[1] - it.T[7], c[2] - it.T[11]) > s.r + itemRadius(it)) continue;
      let d = -1; for (let i = 0; i < n; i++) d = Math.max(d, itemDepth(it, it.T, o[3 * i], o[3 * i + 1], o[3 * i + 2]));
      const lim = it === cfg.grip && cfg.hand?.has(l) ? 0.003 : TOL;
      if (d > lim) add(it === cfg.grip ? "잡는 물건" : "물건", `${l}|${it.name}`, d);
    }
    if (held && !cfg.hand?.has(l) && Math.hypot(c[0] - held.T[3], c[1] - held.T[7], c[2] - held.T[11]) < s.r + itemRadius(held)) {
      let d = -1; for (let i = 0; i < n; i++) d = Math.max(d, itemDepth(held, held.T, o[3 * i], o[3 * i + 1], o[3 * i + 2]));
      if (d > TOL) add("든 물건-몸", `${l}|${held.name}`, d);
    }
  }
  for (const [A, B] of cfg.self || []) for (const a of A) for (const b of B) {
    if (!sh.has(a) || !sh.has(b) || !W[a] || !W[b]) continue;
    const ca = sh.center(a, W[a]), cb = sh.center(b, W[b]);
    if (Math.hypot(ca[0] - cb[0], ca[1] - cb[1], ca[2] - cb[2]) > sh.L[a].r + sh.L[b].r) continue;
    const o = P(a); let k = 0;
    for (let i = 0; i < sh.L[a].n; i++) if (sh.inside(b, W[b], o[3 * i], o[3 * i + 1], o[3 * i + 2])) k++;
    if (k) add("몸끼리", `${a}|${b}`, k * 0.001);
  }
  if (held) {
    const C = itemPoints(held, held.T);
    let m = 0; for (const p of C) m = Math.min(m, p[2]); if (m < -TOL) add("든 물건-바닥", held.name, -m);
    for (const b of cfg.boxes) { let d = 0; for (const p of C) d = Math.max(d, boxDepth(b, ...p)); if (d > TOL) add("든 물건-도킹", `${held.name}|${b.name}`, d); }
    for (const it of items) { if (it === held) continue; let d = 0; for (const p of C) d = Math.max(d, itemDepth(it, it.T, ...p)); if (d > TOL) add("물건끼리", `${held.name}|${it.name}`, d); }
    for (const b of cfg.selfBody || []) {                 // 든 물건이 몸 속으로
      if (!sh.has(b) || !W[b]) continue; let k = 0; for (const p of C) if (sh.inside(b, W[b], ...p)) k++; if (k) add("든 물건-몸속", `${held.name}|${b}`, k * 0.001);
    }
  }
  return out;
}
// 손가락 한 묶음(링크들)과 물건 사이 가장 가까운 거리 (음수 = 파고듦)
export function gap(sh, W, links, it) {
  let g = Infinity;
  for (const l of links) { if (!sh.has(l)) continue; const o = sh.world(l, W[l]); for (let i = 0; i < sh.L[l].n; i++) g = Math.min(g, -itemDepth(it, it.T, o[3 * i], o[3 * i + 1], o[3 * i + 2])); }
  return g;
}
