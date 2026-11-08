// 책꽂이 장면 (11-02) — 책꽂이 2개 × 칸 9개(3단 × 3열), 책(얇은 직육면체). three.js·물리 엔진 없이 숫자만.
//   책꽂이 좌표: x = 안쪽(깊이, 앞면 0 → 뒤 DEPTH), y = 왼쪽(앞에서 보아), z = 위. 칸 이름 = "책꽂이-칸" (1-1 = 1번 책꽂이 맨 윗단 왼쪽, 1-9 = 맨 아랫단 오른쪽)
//   책 좌표: x = 깊이(안쪽), y = 두께(줄 방향), z = 높이. 책꽂이에 꽂힌 책은 책꽂이와 같은 방향.
import { rpyT, mul, apply, inv } from "./base.js";

export const CELL = { W: 0.30, H: 0.32, D: 0.30 };            // 칸 안쪽 너비·높이·깊이 (책 위 18cm·뒤 17cm 는 갈고리가 들어갈 자리)
export const PANEL = 0.02, BACK = 0.01, BASE_Z = 0.20;         // 판 두께 · 뒤판 · 맨 아랫단 바닥 높이
export const BOOK = { t: 0.022, d: 0.12, h: 0.14, mass: 0.25 }; // 두께 · 깊이 · 높이
export const FRONT_IN = 0.01;                                  // 꽂힌 책 앞면이 책꽂이 앞면보다 이만큼 안쪽
export const WALL = 0.04;                                      // 칸 벽에서 이만큼은 비워 둔다 (손목이 벽에 안 닿게 갈고리를 넣으려면)
export const SHELF_W = 3 * CELL.W + 4 * PANEL, SHELF_H = BASE_Z + 3 * CELL.H + 3 * PANEL;
export const R_FRONT = 0.80;                                   // 가운데에서 책꽂이 앞면까지
// 책꽂이 둘이 가운데를 보고 ㄱ 자로 선다 (가운데 = UR15 자리, 걷는 로봇은 열린 쪽에서 들어온다)
//   자리(x, y, yaw)는 사용자가 3D 에서 끌어 옮길 수 있다 → 이 배열을 그 자리에서 고친다. 처음 자리는 SHELVES0
const SHELVES0 = [
  { id: 1, name: "1번 책꽂이", x: R_FRONT, y: 0, yaw: 0, color: "#8a6a4a" },
  { id: 2, name: "2번 책꽂이", x: 0, y: R_FRONT, yaw: Math.PI / 2, color: "#7a5f45" },
];
export const SHELVES = SHELVES0.map(s => ({ ...s }));
export function resetShelves() { SHELVES0.forEach((s, i) => Object.assign(SHELVES[i], s)); }
export const shelfT = (s) => rpyT(0, 0, s.yaw, [s.x, s.y, 0]);

// 정렬대: 바닥에 누운 책을 걸쳐 놓고 다시 잡는 작은 받침 (열린 쪽 -x)
export const STAND = { x: -1.30, y: 0, w: 0.30, h: 0.40, name: "정렬대" };   // Spot 뒷다리(몸 가운데에서 0.57m 뒤)가 물러날 때 안 닿게
export const standBox = () => ({ name: STAND.name, lo: [STAND.x - STAND.w / 2, STAND.y - STAND.w / 2, 0], hi: [STAND.x + STAND.w / 2, STAND.y + STAND.w / 2, STAND.h] });
// 칸 k (1..9) → 단 r (0 = 맨 위), 열 c (0 = 왼쪽)
export const cellRC = (k) => [Math.floor((k - 1) / 3), (k - 1) % 3];
export function cellBox(k) {                                   // 책꽂이 좌표의 칸 안쪽 상자 {lo, hi}
  const [r, c] = cellRC(k), z0 = BASE_Z + (2 - r) * (CELL.H + PANEL), y1 = SHELF_W / 2 - PANEL - c * (CELL.W + PANEL);
  return { lo: [0, y1 - CELL.W, z0], hi: [CELL.D, y1, z0 + CELL.H] };
}
export function parseCell(name) {                              // "1-4" → {shelf, k}
  const m = /^(\d)-(\d)$/.exec(String(name).trim()); if (!m) return null;
  const s = SHELVES.find(x => x.id === +m[1]), k = +m[2];
  return s && k >= 1 && k <= 9 ? { shelf: s, k, name: `${m[1]}-${m[2]}` } : null;
}
export const CELLS = SHELVES0.flatMap(s => [...Array(9)].map((_, i) => `${s.id}-${i + 1}`));

// 책꽂이 판들 (책꽂이 좌표 상자) — 물리(고정 물체)·부딪힘 검사 둘 다 이걸 쓴다
export function shelfPanels() {
  const P = [], W = SHELF_W, Dp = CELL.D + BACK;
  P.push({ name: "옆판 왼", lo: [0, W / 2 - PANEL, 0], hi: [Dp, W / 2, SHELF_H] });
  P.push({ name: "옆판 오른", lo: [0, -W / 2, 0], hi: [Dp, -W / 2 + PANEL, SHELF_H] });
  P.push({ name: "뒤판", lo: [CELL.D, -W / 2, 0], hi: [Dp, W / 2, SHELF_H] });
  P.push({ name: "받침", lo: [0, -W / 2, 0], hi: [Dp, W / 2, BASE_Z - PANEL] });
  for (let r = 0; r <= 3; r++) {                               // 단 판 (맨 아래 받침 위 판 포함)
    const z = BASE_Z - PANEL + r * (CELL.H + PANEL);
    P.push({ name: `단 판 ${r}`, lo: [0, -W / 2, z], hi: [Dp, W / 2, z + PANEL] });
  }
  for (const c of [1, 2]) {                                    // 열 칸막이
    const y = W / 2 - c * (CELL.W + PANEL);
    P.push({ name: `칸막이 ${c}`, lo: [0, y - PANEL / 2 - 0.0, BASE_Z], hi: [Dp, y + PANEL / 2, SHELF_H - PANEL] });
  }
  return P.map(p => ({ ...p, lo: p.lo.slice(), hi: p.hi.slice() }));
}
// 월드 좌표 상자 — 책꽂이는 아무 각도로 돌 수 있다: 판 상자는 책꽂이 좌표 그대로(lo, hi) + 월드→책꽂이 행렬(Ti) + 걸러내기용 월드 축 맞춤 상자(alo, ahi)
export function worldBoxes() {
  const out = [];
  for (const s of SHELVES) {
    const T = shelfT(s), Ti = inv(T);
    for (const p of shelfPanels()) {
      const cs = [0, 1, 2, 3, 4, 5, 6, 7].map(k => apply(T, [k & 1 ? p.hi[0] : p.lo[0], k & 2 ? p.hi[1] : p.lo[1], k & 4 ? p.hi[2] : p.lo[2]]));
      out.push({ name: `${s.name} ${p.name}`, shelf: s.id, lo: p.lo, hi: p.hi, Ti,
                 alo: [0, 1, 2].map(i => Math.min(...cs.map(c => c[i]))), ahi: [0, 1, 2].map(i => Math.max(...cs.map(c => c[i]))) });
    }
  }
  return out;
}
// 바닥에서 본 자리 (걷는 로봇 길 찾기 · 끌어 놓기 겹침 검사): 가운데 · 반 길이 · 방향
export const SHELF_D = CELL.D + BACK;
export function shelfFoot(s) { const T = shelfT(s); return { name: s.name, cx: T[3] + T[0] * SHELF_D / 2, cy: T[7] + T[4] * SHELF_D / 2, hx: SHELF_D / 2, hy: SHELF_W / 2, yaw: s.yaw }; }
export const standFoot = () => ({ name: STAND.name, cx: STAND.x, cy: STAND.y, hx: STAND.w / 2, hy: STAND.w / 2, yaw: 0 });
// 바닥 위 두 직사각형(가운데·반 길이·방향)이 겹치나 (분리축)
export function rectsOverlap(A, B, margin = 0) {
  const ax = [[Math.cos(A.yaw), Math.sin(A.yaw)], [-Math.sin(A.yaw), Math.cos(A.yaw)]], bx = [[Math.cos(B.yaw), Math.sin(B.yaw)], [-Math.sin(B.yaw), Math.cos(B.yaw)]];
  const d = [B.cx - A.cx, B.cy - A.cy];
  for (const n of [...ax, ...bx]) {
    const ra = A.hx * Math.abs(n[0] * ax[0][0] + n[1] * ax[0][1]) + A.hy * Math.abs(n[0] * ax[1][0] + n[1] * ax[1][1]);
    const rb = B.hx * Math.abs(n[0] * bx[0][0] + n[1] * bx[0][1]) + B.hy * Math.abs(n[0] * bx[1][0] + n[1] * bx[1][1]);
    if (Math.abs(d[0] * n[0] + d[1] * n[1]) > ra + rb + margin) return false;
  }
  return true;
}
// 점에서 직사각형까지 거리 (안이면 0)
export function rectDist(A, x, y) {
  const c = Math.cos(A.yaw), s = Math.sin(A.yaw), dx = x - A.cx, dy = y - A.cy, u = dx * c + dy * s, v = -dx * s + dy * c;
  return Math.hypot(Math.max(0, Math.abs(u) - A.hx), Math.max(0, Math.abs(v) - A.hy));
}
// 칸 k 안의 자리: 왼쪽 벽에서 u 만큼 (책 가운데), 앞면에서 FRONT_IN 안쪽에 선 책의 월드 행렬
export function slotT(shelf, k, u, frontIn = FRONT_IN) {
  const b = cellBox(k);
  return mul(shelfT(shelf), rpyT(0, 0, 0, [frontIn + BOOK.d / 2, b.hi[1] - u, b.lo[2] + BOOK.h / 2]));
}
// 처음 놓인 책들 — 칸 이름, 왼쪽부터 간격
const COLORS = ["#c0392b", "#2e86c1", "#28b463", "#d4ac0d", "#8e44ad", "#d35400", "#17a589", "#7f8c8d", "#e84393", "#2d3436"];
export function initialBooks() {
  const books = [];
  const put = (cell, n, gap) => {
    const { shelf, k } = parseCell(cell);
    for (let i = 0; i < n; i++) {
      const id = `b${books.length + 1}`;
      books.push({ id, name: `책 ${books.length + 1}`, color: COLORS[books.length % COLORS.length], kind: "book", ...BOOK,
                   T: slotT(shelf, k, WALL + BOOK.t / 2 + i * (BOOK.t + gap)) });
    }
  };
  put("1-1", 3, 0.045);          // 띄엄띄엄 (손가락이 들어간다)
  put("1-4", 2, 0.003);          // 붙어 있음
  put("2-5", 3, 0.04);
  put("2-8", 3, 0.003);          // 꽉 붙은 세 권 (갈고리로만 뺄 수 있다)
  // 바닥에 누운 책 (줍기)
  books.push({ id: `b${books.length + 1}`, name: `책 ${books.length + 1}`, color: COLORS[books.length % COLORS.length], kind: "book", ...BOOK,
               T: rpyT(Math.PI / 2, 0, 0.4, [-0.62, 0.52, BOOK.t / 2]) });
  return books;
}
