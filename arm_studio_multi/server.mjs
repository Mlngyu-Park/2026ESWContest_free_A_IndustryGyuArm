// ARM STUDIO 다중 로봇 (11-02) — ARM STUDIO 프로 화면을 복사해 G1 · Spot · UR15 에 맞춘 판.
//   구조는 ARM STUDIO 와 같다: 서버가 계산(로봇 동작·책 물리)을 돌리고,
//   화면(ui/)은 /api/meta · /api/state · /api/call 로, 3D 뷰어(viewer.html)는 /viewer/state · /mesh/… 로 받아 그린다.
//   계산 코드는 duo_studio/web 의 것을 그대로 쓴다 (도킹 장면: g1.js · spot.js, 책꽂이 장면: ur15b.js · spotb.js · bookworld.js).
//   ARM STUDIO 원본(Industry_gyu_arm)은 건드리지 않는다.
import http from "http";
import fs from "fs";
import path from "path";
import { fileURLToPath, pathToFileURL } from "url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const WEB = path.join(HERE, "..", "duo_studio", "web");
const imp = (f) => import(pathToFileURL(path.join(WEB, f)).href);
const { mul, rpyT, inv, apply } = await imp("base.js");
const { Shapes, scan } = await imp("collide.js");
const { initPhysics } = await imp("phys.js");
const SC = await imp("scene.js");
const { G1World } = await imp("g1.js");
const { SpotWorld } = await imp("spot.js");
const SH = await imp("shelf.js");
const BW = await imp("bookworld.js");
const { Ur15Books, PEDESTAL } = await imp("ur15b.js");
const { SpotBooks, spotFoot, rectFree: spotFree } = await imp("spotb.js");
const R = await initPhysics();
// 월드 레이첼 (Spot 물리 걷기 AI, ../world_rachel) — 두뇌 파일이 있으면 '걷기: AI' 를 고를 수 있다
const WR = { PW: null, brain: null, name: null };
try {
  const D = path.join(HERE, "..", "world_rachel");
  WR.PW = (await import(pathToFileURL(path.join(D, "phys_walker.mjs")).href)).PhysWalker;
  WR.brain = JSON.parse(fs.readFileSync(path.join(D, "policies", "v2.json"))); WR.name = `v${WR.brain.ver ?? 2}`;   // 파일 이름은 그대로, 판은 두뇌 안의 표시
} catch { /* 없으면 계산 걷기만 */ }
const WALKS = { calc: "계산 걷기", ai: "AI 걷기 (월드 레이첼)" };
function spotOpt(m, s) {                              // 책꽂이 Spot: AI 걷기면 물리 걷기를 끼운다
  if (S.walk !== "ai" || !WR.brain) return {};
  return { makeWalker: (robot, pose, w) => new WR.PW(R, robot, m, s, WR.brain, { pose, book: () => w.held ? (w.held.mass ?? 0.5) : 0 }) };
}

const PORT = +(process.env.PORT || 7910);
const ROBOT_NAMES = { g1: "유니트리 G1", spot: "Spot", ur15: "UR15" };
// 장면마다 쓸 수 있는 로봇 (같은 블록 · 같은 공정을 로봇만 바꿔 끼운다)
const SCENES = {
  books: { name: "책꽂이 공정", robots: ["ur15", "spot"], soon: ["g1"],
           make: { ur15: (m, s, log) => new Ur15Books(m, s, R, log), spot: (m, s, log) => new SpotBooks(m, s, R, log, spotOpt(m, s)) } },
  docks: { name: "도킹 옮기기", robots: ["g1", "spot"], soon: [],
           make: { g1: (m, s, log) => new G1World(m, log, s), spot: (m, s, log) => new SpotWorld(m, log, s) } },
};
const EXAMPLES = {
  // 11-06 (사용자: "기본 공정이 너무 길다, 1-1 의 책을 2-5 로 옮기는 것만") — 시작·[예제] 모두 이 한 블록.
  //   '한 권': 2-5 는 5권까지라 (이미 6·7·8) '전부' 면 2권 넣고 '꽉 찼습니다' 로 멈춘다. 검사(test_books)용 BW.BOOK_EXAMPLE 은 그대로.
  //   실행 중 책꽂이·책 끌어 옮기기(돈케어/케어 다시 짜기)는 원래대로 된다.
  books: [{ type: "move", args: ["1-1", "2-5", "one"] }],
  docks: SC.EXAMPLE.map(b => ({ type: b.type, args: b.arg != null ? [b.arg] : [] })),
};

// ── 로봇 모델 (한 번 읽어 둔다) ──
const MODELS = {};
function model(r) {
  if (!MODELS[r]) {
    const d = path.join(WEB, "assets", r);
    MODELS[r] = { m: JSON.parse(fs.readFileSync(path.join(d, "robot.json"))), sh: new Shapes(JSON.parse(fs.readFileSync(path.join(d, "shapes.json")))), dir: d };
  }
  return MODELS[r];
}

// ── 지금 상태 ──
const S = { scene: "books", robot: process.env.MULTI_ROBOT || "ur15", program: { books: EXAMPLES.books.map(b => ({ ...b, args: b.args.slice() })), docks: EXAMPLES.docks.map(b => ({ ...b, args: b.args.slice() })) },
            speed: 1.0, gen: 0, fixGen: 0, fixMode: "book", showPlan: true, walk: "calc" };   // walk: Spot 걷기 (계산 · AI)   // showPlan: 3D 에 최종 자리 · 갈 길 미리 보기   // fixMode: 건드린 책을 언제 고치나 (바로 · 이 책 끝내고 · 이 블록 끝내고 · 맨 끝에)
const FIX_MODES = { now: "바로 고치기", book: "이 책 끝내고", block: "이 블록 끝내고", end: "맨 끝에" };
const LOG = []; let LOGN = 0;
const log = (msg, cls = "") => { LOG.push({ n: ++LOGN, t: world ? +world.t.toFixed(1) : 0, msg: String(msg), cls }); if (LOG.length > 300) LOG.shift(); };
const RUN = { running: false, blk: -1, err: null, t0: 0, done: false, wait: null };
let world = null, DRAG = null, LASTG = null, LASTX = null;    // LASTG · LASTX: 끝난 실행의 목표 · 실행기 (끝난 뒤에도 ① 목록 · 블록 나무에 보여 준다)
// 3D 미리 보기 (책꽂이 장면): ghosts = 최종 자리(반투명 책) · legs = Spot 이 블록대로 걸을 다리들(▶ 전에 계산) · view = 지금 그릴 다리
//   key = 블록 · 책꽂이 · 책 · 로봇 자리. 바뀔 때만 길을 다시 계산한다 (Spot 만, 실행 중이 아닐 때, 끄는 중이 아닐 때)
const PV = { ghosts: [], legs: [], view: [], key: "", stale: false };
let TOUCH = 0;                                         // 사용자가 책을 놓은 차례 (bk.touched)

function build() {
  const M = model(S.robot);
  world = SCENES[S.scene].make[S.robot](M.m, M.sh, log);
  world.waitOnBlock = S.scene === "books";          // 막히면 끝내지 않고 이유를 말한 채 기다린다 (사람이 길을 열어 주면 이어서)
  world.fixMode = S.fixMode; LASTG = LASTX = null; Object.assign(PV, { ghosts: [], legs: [], view: [], key: "", stale: false });
  S.gen++; S.fixGen++;
  Object.assign(RUN, { running: false, blk: -1, err: null, done: false });
  log(`${SCENES[S.scene].name} · ${ROBOT_NAMES[S.robot]} 준비`);
}
build();

// 서버 쪽 계산 루프 (ARM STUDIO 처럼 화면과 따로 돈다) — 1/60초 고정 걸음
let lastT = Date.now(), acc = 0;
setInterval(() => {
  const now = Date.now(); acc += Math.min(0.1, (now - lastT) / 1000) * S.speed; lastT = now;
  let n = 0;
  if (DRAG) { acc = 0; return; }                      // 사용자가 끄는 동안 로봇·물리 멈춤
  while (acc >= 1 / 60 && n < 12) { world.step(1 / 60); acc -= 1 / 60; n++; }
  if (world.walker?.fell && world.fellSaid !== world.walker) { world.fellSaid = world.walker; log("Spot 이 넘어졌습니다 — ⌂ 처음 상태로 다시 세우세요", "bad"); if (RUN.running) stop(); }
  if (n >= 12) acc = 0;                               // 계산이 밀리면 버린다 (느려져도 순간 이동은 없음)
}, 16);

// ── 프로그램 실행 ──
function toEngine(b) { return S.scene === "books" ? { type: b.type, args: b.args } : { type: b.type, arg: b.args[0] ?? null }; }
function run() {
  if (RUN.running) return { ok: false, msg: "이미 실행 중" };
  const prog = S.program[S.scene]; if (!prog.length) return { ok: false, msg: "파이프라인이 비었습니다" };
  const empty = prog.findIndex(b => b.args.some(a => a == null));
  if (empty >= 0) { const msg = `${empty + 1}번 블록 '${blockText(prog[empty])}' 의 빈칸(◯)을 채워 주세요`; return { ok: false, msg }; }
  if (DRAG) return { ok: false, msg: "옮기는 중에는 시작할 수 없습니다" };
  const hooks = {
    wait(msg) { RUN.wait = msg; log(`⏸ 기다림: ${msg} — 책꽂이·책을 옮겨 길을 열어 주면 다시 해 봅니다 (■ 정지 = 끝)`, "bad"); },
    resume() { RUN.wait = null; log("▶ 장면이 바뀌어 다시 해 봅니다"); },
    block(i) { RUN.blk = i; const b = prog[i]; log(`▶ ${i + 1}/${prog.length} ${blockText(b)}`); },
    end(err, i) { LASTG = world.exec?.G ?? null; LASTX = world.exec ?? null; RUN.wait = null; RUN.running = false; RUN.err = err; RUN.done = !err; RUN.blk = err ? i : prog.length; log(err ? `■ 멈춤: ${err}` : "✔ 끝", err ? "bad" : "ok"); },
  };
  try { world.run(prog.map(toEngine), hooks); }        // 책꽂이: ▶ 순간 최종 목표를 먼저 계산 — 안 되는 블록은 여기서 걸린다
  catch (e) { log(`■ 시작 못 함: ${e.message}`, "bad"); return { ok: false, msg: e.message }; }
  Object.assign(RUN, { running: true, blk: 0, err: null, t0: world.t, done: false, wait: null });
  log("▶ 미리보기 시작" + (S.scene === "books" ? " — 책마다 목표 칸을 정했습니다 (① 물건 목록)" : ""));
  return { ok: true };
}
function stop() { if (world.exec) { LASTG = world.exec.G ?? LASTG; LASTX = world.exec.G ? world.exec : LASTX; world.stop(); RUN.running = false; RUN.wait = null; RUN.err = "정지"; log("■ 정지", "bad"); } return { ok: true }; }

// ── 블록 정의 (화면 팔레트·파이프라인 글자) ──
const BLOCKS = {
  books: Object.fromEntries(Object.entries(BW.BOOK_BLOCKS).map(([k, v]) => [k, { name: v.name, args: v.args, hint: v.hint }])),
  docks: Object.fromEntries(Object.entries(SC.BLOCKS).map(([k, v]) => [k, { name: v.name, args: v.arg ? [v.arg] : [], hint: v.hint }])),
};
function options(kind) {
  if (S.scene === "books") return kind === "cell" ? SH.CELLS.map(c => [c, c]) : kind === "count" ? BW.COUNTS : kind === "sec" ? BW.SECS : [];
  if (kind === "target") return world.targets();
  if (kind === "item") return world.items.map(i => [i.id, i.name]);
  if (kind === "place") return [...SC.DOCKS.map(d => [d.id, d.name]), ...SC.MARKS.map(m => [m.id, m.name])];
  return { angle: SC.ANGLES, posture: SC.POSTURES, sec: SC.SECS }[kind] ?? [];
}
function argName(kind, v) { const o = options(kind).find(x => x[0] === v); return o ? o[1] : v ?? "◯"; }
function blockText(b) { const B = BLOCKS[S.scene][b.type]; return `${B?.name ?? b.type} ${(B?.args ?? []).map((k, j) => argName(k, b.args[j])).join(" ")}`.trim(); }

// ── /viewer/state: 링크(겉모양마다) · 움직이는 물건 · 고정물 ──
const MESH = {};                                       // 겉모양 id → STL 파일
function linkState() {
  const W = world.fk(), M = model(S.robot), out = [];
  for (const [ln, L] of Object.entries(M.m.links)) {
    if (!W[ln]) continue;
    (L.visuals || []).forEach((v, vi) => {
      const id = `${S.robot}.${ln}.${vi}`; MESH[id] = path.join(M.dir, v.mesh);
      const T = mul(W[ln], rpyT(v.rpy[0], v.rpy[1], v.rpy[2], v.xyz));
      out.push({ id, color: v.color || L.color || "#c8ccd2", s: (v.scale || [1])[0], m: T.map(x => +x.toFixed(5)) });
    });
  }
  return out;
}
function bodyState() {
  const out = [];
  if (S.scene === "books") for (const b of world.books) out.push({ id: b.id, type: "box", size: [b.d, b.t, b.h], color: b.color, m: b.T.map(x => +x.toFixed(5)), spine: true });
  else for (const it of world.items) out.push(it.kind === "cube" ? { id: it.id, type: "box", size: [it.size, it.size, it.size], color: it.color, m: it.T.map(x => +x.toFixed(5)) }
                                               : { id: it.id, type: "cyl", r: it.r, h: it.h, color: it.color, m: it.T.map(x => +x.toFixed(5)) });
  return out;
}
function fixedState() {                                // 장면이 바뀔 때만 다시 보낸다 (3D 에 한글 이름표는 안 붙인다 — 칸 번호만)
  const boxes = [], labels = [], shelves = [];
  if (S.scene === "books") {
    // 책꽂이 = 책꽂이 좌표의 판들 + 칸 번호. 자리(행렬)는 /viewer/state 로 매번 (사용자가 끌어 옮긴다)
    for (const sh of SH.SHELVES) shelves.push({ id: sh.id, color: sh.color, panels: SH.shelfPanels().map(p => ({ lo: p.lo, hi: p.hi })),
      labels: [...Array(9)].map((_, i) => { const b = SH.cellBox(i + 1); return { text: `${sh.id}-${i + 1}`, p: [-0.015, (b.lo[1] + b.hi[1]) / 2, b.hi[2] - 0.03] }; }) });
    const sb = SH.standBox(); boxes.push({ lo: sb.lo, hi: sb.hi, color: "#6b7280" });
    if (S.robot === "ur15") boxes.push({ lo: [-PEDESTAL.w / 2, -PEDESTAL.w / 2, 0], hi: [PEDESTAL.w / 2, PEDESTAL.w / 2, PEDESTAL.h], color: "#4b5563" });
  } else {
    for (const d of SC.DOCKS) {
      boxes.push({ lo: [d.x - SC.POST_W / 2, d.y - SC.POST_W / 2, 0], hi: [d.x + SC.POST_W / 2, d.y + SC.POST_W / 2, SC.DOCK_H - 0.04], color: "#5b616d" });
      boxes.push({ lo: [d.x - SC.DOCK_W / 2, d.y - SC.DOCK_W / 2, SC.DOCK_H - 0.04], hi: [d.x + SC.DOCK_W / 2, d.y + SC.DOCK_W / 2, SC.DOCK_H], color: d.color });
    }
    for (const k of SC.MARKS) boxes.push({ lo: [k.x - 0.12, k.y - 0.12, 0], hi: [k.x + 0.12, k.y + 0.12, 0.003], color: "#d8dde6", flat: true });
  }
  // 처음 시점 (로봇 좌표 z-up). 책꽂이 = 책꽂이가 없는 -x 쪽 위에서, 도킹 = 마당 전체가 보이게
  const view = S.scene === "books" ? { cam: [-2.3, -0.7, 2.0], target: [0.1, 0, 0.35] } : { cam: [-3.6, -3.2, 3.2], target: [0.2, 0.3, 0.2] };
  return { gen: S.fixGen, boxes, labels, shelves, view };
}

// ── 끌어 옮기기 (3D 에서 책꽂이 · 책) ──────────────────────────────
//   잡는 동안 로봇·물리는 멈춘다. 놓을 때 겹치면 제자리로 되돌리고, 받아들이면 로봇이 그때 장면으로 다시 계산한다.
//   책꽂이 = 돈케어 (로봇이 칸 이름으로 찾아간다) · 책 = 케어 (▶ 때 정한 목표 칸에 없으면 로봇이 고쳐 놓는다)
//   ray: 뷰어가 보낸 마우스 광선 {o, d} (로봇 좌표)
// 로봇 자리: UR15 = 받침대 둘레 (팔 밑동까지 8cm 여유). Spot = 지금 서 있는 자리만 (걸어 다니니 처음 자리를 비워 둘 까닭이 없다)
const UR_ZONE = () => ({ cx: 0, cy: 0, hx: PEDESTAL.w / 2 + 0.08, hy: PEDESTAL.w / 2 + 0.08, yaw: 0 });
const rayAt = (ray, z) => { const dz = ray.d[2]; if (dz > -1e-6) return null; const t = (z - ray.o[2]) / dz; return [ray.o[0] + t * ray.d[0], ray.o[1] + t * ray.d[1]]; };
const bookFoot = (x, y, yaw) => ({ cx: x, cy: y, hx: SH.BOOK.d / 2, hy: SH.BOOK.h / 2, yaw });
const nameOf = (D) => D.kind === "shelf" ? SH.SHELVES.find(s => s.id === D.id).name : D.bk.name;
// 놓을 수 있나 (바닥 위 겹침 + 로봇 팔·몸 겹침) → 이유 또는 null
function blockers(F, skip = {}) {
  if ([[-1, -1], [1, -1], [1, 1], [-1, 1]].some(([a, b]) => { const c = Math.cos(F.yaw), s = Math.sin(F.yaw), x = F.cx + a * F.hx * c - b * F.hy * s, y = F.cy + a * F.hx * s + b * F.hy * c; return Math.abs(x) > 2.9 || Math.abs(y) > 2.9; })) return "바닥 밖입니다";
  for (const o of SH.SHELVES) if (o.id !== skip.shelf && SH.rectsOverlap(F, SH.shelfFoot(o), 0.01)) return `${o.name}와 겹칩니다`;
  if (SH.rectsOverlap(F, SH.standFoot(), 0.01)) return "정렬대와 겹칩니다";
  if (S.robot === "ur15" && SH.rectsOverlap(F, UR_ZONE())) return "UR15 받침대와 겹칩니다";
  if (S.robot === "spot") { const P = world.pose(); if (SH.rectsOverlap(F, spotFoot(P.x, P.y, P.yaw))) return "Spot 과 겹칩니다"; }
  return null;
}
function checkDrag() {
  const D = DRAG;
  if (D.kind === "shelf") {
    const F = SH.shelfFoot(SH.SHELVES.find(s => s.id === D.id)), why = blockers(F, { shelf: D.id }); if (why) return why;
    const lone = world.books.find(b => !D.books.includes(b) && b !== world.held && b.T[11] < SH.SHELF_H && SH.rectDist(F, b.T[3], b.T[7]) < 0.07); if (lone) return `${lone.name}과 겹칩니다`;
    const hit = scan(world.sh, world.fk(), D.books, world.held, { ...world.cfg, boxes: world.statics.filter(b => b.shelf === D.id), grip: null });
    if (hit.length) return "로봇 팔과 겹칩니다";
    return null;
  }
  if (!D.place) return "놓을 자리가 아닙니다";
  if (D.place.why) return D.place.why;
  if (D.place.where === "floor") {
    const T = D.place.T, why = blockers(bookFoot(T[3], T[7], D.yaw)); if (why) return why;
    const o = world.books.find(b => b !== D.bk && b.T[11] < 0.06 && SH.rectsOverlap(bookFoot(b.T[3], b.T[7], Math.atan2(b.T[4], b.T[0])), bookFoot(T[3], T[7], D.yaw), 0.004)); if (o) return `${o.name}과 겹칩니다`;
  }
  if (scan(world.sh, world.fk(), [D.bk], null, { ...world.cfg, boxes: [], grip: null }).some(c => c.kind === "물건")) return "로봇과 겹칩니다";
  return null;
}
// 책 놓을 자리: 책꽂이 칸 입구를 가리키면 그 칸에 세워서 (가까운 빈 자리), 아니면 바닥에 눕혀서
function bookPlace(bk, ray, yaw) {
  for (const sh of SH.SHELVES) {
    const T = SH.shelfT(sh), Ti = inv(T), o = apply(Ti, ray.o), d = [0, 1, 2].map(i => Ti[4 * i] * ray.d[0] + Ti[4 * i + 1] * ray.d[1] + Ti[4 * i + 2] * ray.d[2]);
    if (o[0] > -0.01 || d[0] < 1e-6) continue;          // 책꽂이 앞에서 안쪽을 볼 때만
    const t = -o[0] / d[0], p = [0, o[1] + t * d[1], o[2] + t * d[2]];
    for (let k = 1; k <= 9; k++) {
      const b = SH.cellBox(k); if (p[1] < b.lo[1] || p[1] > b.hi[1] || p[2] < b.lo[2] || p[2] > b.hi[2]) continue;
      const cell = `${sh.id}-${k}`, ys = world.books.filter(x => x !== bk && world.cellOf(x) === cell).map(x => mul(Ti, x.T)[7]);
      const lo = b.lo[1] + SH.BOOK.t / 2 + 0.002, hi = b.hi[1] - SH.BOOK.t / 2 - 0.002, free = (y) => ys.every(v => Math.abs(v - y) >= SH.BOOK.t + 0.003);
      let y = null; for (let dd = 0; dd <= SH.CELL.W && y === null; dd += 0.002) for (const c of [p[1] + dd, p[1] - dd]) if (c >= lo && c <= hi && free(c)) { y = c; break; }
      if (y === null) return { where: cell, why: `${cell} 칸에 자리가 없습니다` };
      return { where: cell, T: mul(T, rpyT(0, 0, 0, [SH.FRONT_IN + SH.BOOK.d / 2, y, b.lo[2] + SH.BOOK.h / 2 + 0.002])) };
    }
  }
  const q = rayAt(ray, SH.BOOK.t / 2); if (!q) return null;
  return { where: "floor", T: rpyT(Math.PI / 2, 0, yaw, [q[0], q[1], SH.BOOK.t / 2 + 0.001]) };
}
function dragApply() {
  const D = DRAG; if (!D.ray) return;
  if (D.kind === "shelf") { const q = rayAt(D.ray, D.z); if (q) world.moveShelf(D.id, q[0] + D.off[0], q[1] + D.off[1], D.yaw, D.books); }
  else { D.place = bookPlace(D.bk, D.ray, D.yaw); if (D.place?.T) world.placeBook(D.bk, D.place.T); }
  D.why = checkDrag(); D.ok = !D.why; D.moved = true;
}
const DRAGS = {
  drag_start(kind, id, ray, hit) {
    if (S.scene !== "books") return { ok: false, msg: "이 장면은 아직 끌어 옮기기가 없습니다" };
    if (DRAG) DRAGS.drag_end();
    if (kind === "shelf") {
      const sh = SH.SHELVES.find(s => s.id === +id); if (!sh) return { ok: false, msg: "없는 책꽂이" };
      // 잡은 점 높이의 수평면을 따라 끈다 (바닥 점을 따르면 윗판을 잡았을 때 원근 때문에 마우스보다 훨씬 멀리 간다)
      const z = Array.isArray(hit) ? Math.max(0, Math.min(SH.SHELF_H, hit[2])) : 0, q = Array.isArray(hit) ? hit : rayAt(ray, z) ?? [sh.x, sh.y];
      DRAG = { kind, id: sh.id, pose0: { x: sh.x, y: sh.y, yaw: sh.yaw }, books: world.booksInShelf(sh.id), off: [sh.x - q[0], sh.y - q[1]], z, yaw: sh.yaw, ok: true };
    } else {
      const bk = world.books.find(b => b.id === id); if (!bk) return { ok: false, msg: "없는 책" };
      const wasHeld = world.held === bk; if (wasHeld) world.held = null;       // 손에서 빼서 (놓을 때 받아들이면 물리로, 아니면 다시 손에)
      DRAG = { kind, id, bk, T0: bk.T.slice(), wasHeld, yaw: bk.T[11] < 0.06 ? Math.atan2(bk.T[4], bk.T[0]) : 0, ok: true };
    }
    log(`✋ ${nameOf(DRAG)} 옮기는 중 — 로봇 멈춤`);
  },
  drag_move(ray) { if (!DRAG) return; DRAG.ray = ray; dragApply(); return { ok: true, valid: DRAG.ok, why: DRAG.why ?? null }; },
  drag_rot(d) { if (!DRAG) return; DRAG.yaw += +d || 0; dragApply(); },
  drag_end() {
    const D = DRAG; if (!D) return; DRAG = null;
    if (!D.moved) { if (D.wasHeld) world.held = D.bk; log(`${nameOf(D)} 그대로`); return; }
    if (!D.ok) {                                     // 되돌리기
      if (D.kind === "shelf") world.moveShelf(D.id, D.pose0.x, D.pose0.y, D.pose0.yaw, D.books);
      else { world.placeBook(D.bk, D.T0); if (D.wasHeld) world.held = D.bk; }
      log(`↩ ${nameOf(D)}: ${D.why} — 제자리로`, "bad"); return { ok: false, msg: D.why };
    }
    if (D.kind === "book" && D.wasHeld) { world.phys.release(D.bk); D.bk.rel = null; }
    if (D.kind === "book") D.bk.touched = ++TOUCH;    // 건드린 차례 — 고칠 책이 여럿이면 최근 것부터
    world.phys.wakeAll();
    const where = D.kind === "shelf" ? `(${D.pose0.x.toFixed(2)}, ${D.pose0.y.toFixed(2)}) → (${SH.SHELVES.find(s => s.id === D.id).x.toFixed(2)}, ${SH.SHELVES.find(s => s.id === D.id).y.toFixed(2)})` : D.place.where === "floor" ? "바닥" : D.place.where;
    log(`✋ ${nameOf(D)} → ${where}`);
    if (world.exec?.interrupt()) { log("로봇이 바뀐 장면을 다시 보고 계산합니다"); PV.stale = true; }   // ▶ 때 미리 본 길은 이제 낡았다 → 숨김
  },
};

// ── /api/call ──
const FILL_FIRST = new Set(["count", "sec", "angle", "posture"]);
const UNDO = { books: [], docks: [] };                 // Ctrl+Z (원본과 같음) — 블록 편집 직전 프로그램을 쌓는다
// 블록 고치기: 실행 중에는 안 받는다 (▶ 때 정한 목표 · 블록 나무가 바뀐 블록과 어긋난다). 끝난 뒤 고치면 지난 결과를 지우고 새로 미리 본다
const EDIT = (fn) => (...a) => {
  if (RUN.running) return { ok: false, msg: "실행 중에는 블록을 바꿀 수 없습니다 — ■ 정지한 뒤 바꾸고 ▶ 하면, 책이 지금 있는 자리에서 새로 시작합니다" };
  if (RUN.done || RUN.err) { LASTG = LASTX = null; Object.assign(RUN, { blk: -1, err: null, done: false }); }
  return fn(...a);
};
function snap() { const U = UNDO[S.scene]; U.push(S.program[S.scene].map(b => ({ ...b, args: b.args.slice() }))); if (U.length > 50) U.shift(); }
const CALLS = {
  scene(s) { if (!SCENES[s] || s === S.scene) return; DRAGS.drag_end(); stop(); S.scene = s; if (!SCENES[s].robots.includes(S.robot)) S.robot = SCENES[s].robots[0]; SH.resetShelves(); build(); },
  // 로봇만 바꿔 끼운다: 책꽂이 자리 · 책 자리는 그대로 (같은 장면, 같은 공정)
  robot(r) { if (!SCENES[S.scene].robots.includes(r)) return { ok: false, msg: `${ROBOT_NAMES[r] ?? r} 는 이 장면에서 아직 못 씁니다` }; if (r === S.robot) return; DRAGS.drag_end(); stop();
    const keep = S.scene === "books" ? new Map(world.books.map(b => [b.id, b.T.slice()])) : null;
    // UR15 은 받침대에 고정 — 그 자리를 책꽂이·책이 막고 있으면 바꾸지 않는다 (Spot 은 비어 있는 데 세운다)
    if (keep && r === "ur15") {
      const Z = UR_ZONE(), sh = SH.SHELVES.find(o => SH.rectsOverlap(Z, SH.shelfFoot(o), 0.01)), bk = world.books.find(b => b.T[11] < 0.06 && SH.rectDist(Z, b.T[3], b.T[7]) < 0.08);
      if (sh || bk) return { ok: false, msg: `UR15 받침대 자리를 ${(sh ?? bk).name} 가 막고 있습니다 — 옮기거나 ⌂ 원위치를 누르세요` };
    }
    S.robot = r; build();
    if (keep) { for (const b of world.books) if (keep.has(b.id)) world.placeBook(b, keep.get(b.id)); world.phys.wakeAll(); for (let i = 0; i < 60; i++) world.phys.step(1 / 60); }
    if (keep && r === "spot" && !spotFree(-0.1, 0, 0)) {                // 처음 자리가 막혀 있으면 가까운 빈 자리
      let at = null;
      for (let rr = 0.1; rr < 2.6 && !at; rr += 0.1) for (let a = 0; a < 24 && !at; a++) { const x = -0.1 + rr * Math.cos(a * Math.PI / 12), y = rr * Math.sin(a * Math.PI / 12);
        for (const yaw of [0, Math.PI / 2, Math.PI, -Math.PI / 2]) if (spotFree(x, y, yaw)) { at = [x, y, yaw]; break; } }
      if (at) { world.placeAt(...at); log(`Spot 처음 자리가 막혀 (${at[0].toFixed(2)}, ${at[1].toFixed(2)}) 에 세웠습니다`); }
    } },
  ...DRAGS,
  seq_insert: EDIT(function (at, json) { const b = JSON.parse(json), P = S.program[S.scene]; const B = BLOCKS[S.scene][b.type]; if (!B) return { ok: false, msg: "모르는 블록" };
    // 새 블록: 고를 게 뻔한 칸(권수·초·각도·자세)만 첫 값, 칸·물건·자리는 빈칸(◯) — 원본처럼 사용자가 채운다
    b.args = B.args.map((k, j) => b.args?.[j] ?? (FILL_FIRST.has(k) ? options(k)[0]?.[0] ?? null : null)); snap(); P.splice(Math.max(0, Math.min(P.length, +at)), 0, b); }),
  seq_set: EDIT(function (i, j, v) { const b = S.program[S.scene][+i]; if (b) { snap(); b.args[+j] = v; } }),
  seq_del: EDIT(function (i) { snap(); S.program[S.scene].splice(+i, 1); }),
  seq_move: EDIT(function (i, to) { snap(); const P = S.program[S.scene], [b] = P.splice(+i, 1); if (b) P.splice(Math.max(0, Math.min(P.length, +to)), 0, b); }),
  seq_clear: EDIT(function () { snap(); S.program[S.scene] = []; }),
  example: EDIT(function () { snap(); S.program[S.scene] = EXAMPLES[S.scene].map(b => ({ ...b, args: b.args.slice() })); }),
  undo: EDIT(function () { const U = UNDO[S.scene]; if (!U.length) return { ok: false, msg: "되돌릴 것이 없습니다" }; S.program[S.scene] = U.pop(); log("↶ 되돌리기"); }),
  run, stop,
  home() { DRAGS.drag_end(); stop(); LASTG = LASTX = null; SH.resetShelves(); world.reset(); S.fixGen++; Object.assign(RUN, { running: false, blk: -1, err: null, done: false }); log("⌂ 처음 상태로 (책꽂이 · 책 · 로봇)"); },
  // 건드린 책을 언제 고치나 — 실행 중에 바꿔도 다음 고를 때부터 따른다
  show_plan(on) { S.showPlan = !!on; PV.key = ""; },
  // Spot 걷기 바꾸기: 책·책꽂이·Spot 자리는 그대로 두고 걷는 방식만 바꿔 끼운다
  walk_mode(m) {
    if (!WALKS[m]) return { ok: false, msg: "모르는 걷기" };
    if (m === "ai" && !WR.brain) return { ok: false, msg: "학습된 두뇌(world_rachel/policies/v2.json)가 아직 없습니다" };
    if (m === S.walk) return;
    DRAGS.drag_end(); stop(); S.walk = m;
    if (S.scene === "books" && S.robot === "spot") {
      const keep = new Map(world.books.map(b => [b.id, b.T.slice()])), P = { ...world.walker.pose };
      build();
      for (const b of world.books) if (keep.has(b.id)) world.placeBook(b, keep.get(b.id));
      world.phys.wakeAll(); for (let i = 0; i < 60; i++) world.phys.step(1 / 60);
      world.placeAt(P.x, P.y, P.yaw);
    }
    log(`Spot 걷기: ${WALKS[m]}${m === "ai" ? " — 다리는 물리, 두뇌가 균형을 잡습니다" : ""}`);
  },
  fix_mode(m) { if (!FIX_MODES[m]) return { ok: false, msg: "모르는 순서" }; S.fixMode = m; world.fixMode = m; log(`건드린 책: ${FIX_MODES[m]}`); },
  speed(x) { S.speed = Math.max(0.25, Math.min(8, parseFloat(x) || 1)); },
};

// 화면 ① 탭: 물건(케어 — 지금 자리 · 목표 칸) · 설비(돈케어) · 끌기 상태 · 시작 전 목표 계산 오류
function sceneInfo(prog) {
  if (S.scene !== "books") return {
    objects: world.items.map(i => ({ id: i.id, name: i.name, color: i.color, where: i.on === "hand" ? "손" : (SC.DOCKS.find(d => d.id === i.on)?.name ?? SC.MARKS.find(m => m.id === i.on)?.name ?? "바닥") })),
    fixtures: [...SC.DOCKS.map(d => ({ name: d.name })), ...SC.MARKS.map(m => ({ name: m.name }))], drag: null, planErr: null };
  let G = world.exec?.G ?? (RUN.done || RUN.err ? LASTG : null), planErr = null;
  const EX = world.exec ?? (G && G === LASTG ? LASTX : null);     // 나무 · 최종 자리에 쓸 실행기 (끝난 실행이면 그 기록)
  if (!G && prog.length && prog.every(b => b.args.every(a => a != null))) { try { G = BW.planGoals(world, prog.map(toEngine)); } catch (e) { planErr = e.message; } }
  const ko = (l) => ({ floor: "바닥", stand: "정렬대", hand: "손", "?": "?" })[l] ?? l;
  return {
    objects: world.books.map(b => ({ id: b.id, name: b.name, color: b.color, where: ko(world.locOf(b)), goal: G ? ko(G.goal.get(b)) : null, care: true })),
    fixtures: [...SH.SHELVES.map(s => ({ name: `${s.name} (${s.id}-1 ~ ${s.id}-9)`, care: false })), { name: SH.STAND.name, care: false, fixed: true }, ...(S.robot === "ur15" ? [{ name: "UR15 받침대", care: false, fixed: true }] : [])],
    drag: DRAG ? { name: nameOf(DRAG), ok: DRAG.ok, why: DRAG.why ?? null } : null, planErr, goalLive: !!world.exec,
    // 블록 나무: 블록 → 책마다 → 단계(이동 · 잡기 · 놓기) → 칸(열기 · 넣기 …). 실행 전이면 지금 장면으로 미리 짠 것
    tree: preview(BW.blockTree(world, prog.map(toEngine), G, EX, RUN.done), G, EX),
  };
}
// 3D 미리 보기 갱신 (화면이 /api/state 를 부를 때마다, 1초에 4번) — 나무를 그대로 돌려준다
function preview(tree, G, ex) {
  PV.ghosts = S.showPlan && G ? BW.goalGhosts(world, G, ex) : [];
  if (!S.showPlan || !world.previewRoute || !tree) { Object.assign(PV, { legs: [], view: [], key: "" }); return tree; }
  if (RUN.running) {                                 // 실행 중: ▶ 전에 미리 본 다리 중 아직 안 간 것만 (지금 걷는 다리는 진짜 길이 따로 그려진다)
    if (PV.stale) { PV.view = []; return tree; }
    const st = new Map(); for (const b of tree.blocks) for (const n of b.nodes) n.leaves.forEach((l, i) => st.set(`${n.id}|${i}`, n.state === "done" ? "done" : l.state));
    PV.view = PV.legs.filter(L => L.pts && st.has(L.id) && !st.get(L.id));
    return tree;
  }
  if (DRAG) return tree;
  const legs = [];                                   // 블록 나무의 이동 단계 = Spot 이 걸을 다리들 (차례대로)
  for (const b of tree.blocks) for (const n of b.nodes) if (n.state !== "done") n.leaves.forEach((l, i) => { if (l.go) legs.push({ id: `${n.id}|${i}`, target: l.go.target, bk: world.books.find(x => x.id === l.go.bk) }); });
  const P = world.walker.pose, r2 = (v) => +v.toFixed(2), q5 = (v) => Math.round(v / 0.05);   // 로봇 자리는 5cm·0.05rad 단위 (물리 걷기는 서 있어도 조금씩 흔들린다)
  const key = JSON.stringify([legs.map(L => [L.id, L.target]), SH.SHELVES.map(s => [s.x, s.y, s.yaw].map(v => +v.toFixed(3))), world.books.map(b => [b.T[3], b.T[7], b.T[11]].map(r2)), [P.x, P.y, P.yaw].map(q5)]);
  if (key === PV.key) return tree;
  PV.key = key; PV.stale = false;
  try { PV.legs = world.previewRoute(legs); } catch (e) { PV.legs = []; log(`미리 보기 길 계산 실패: ${e.message}`, "bad"); }
  PV.view = PV.legs.filter(L => L.pts);
  return tree;
}

// ── HTTP ──
const TYPES = { ".html": "text/html; charset=utf-8", ".css": "text/css", ".js": "application/javascript", ".json": "application/json", ".stl": "application/octet-stream", ".STL": "application/octet-stream" };
function send(res, code, body, type = "application/json") { res.writeHead(code, { "Content-Type": type, "Cache-Control": "no-store" }); res.end(typeof body === "string" || Buffer.isBuffer(body) ? body : JSON.stringify(body)); }
function file(res, f) { if (!fs.existsSync(f) || !fs.statSync(f).isFile()) return send(res, 404, "없음", "text/plain"); send(res, 200, fs.readFileSync(f), TYPES[path.extname(f)] ?? "application/octet-stream"); }
const server = http.createServer(async (req, res) => {
  const u = new URL(req.url, "http://x"), p = decodeURIComponent(u.pathname);
  try {
    if (p === "/") return file(res, path.join(HERE, "ui", "index.html"));
    if (p.startsWith("/ui/")) return file(res, path.join(HERE, "ui", path.normalize(p.slice(4)).replace(/^(\.\.[/\\])+/, "")));
    if (p.startsWith("/vendor/")) return file(res, path.join(WEB, "vendor", path.normalize(p.slice(8)).replace(/^(\.\.[/\\])+/, "")));
    if (p === "/viewer/" || p === "/viewer") return file(res, path.join(HERE, "viewer.html"));
    if (p === "/viewer/state") return send(res, 200, { gen: S.gen, links: linkState(), bodies: bodyState(), fixgen: S.fixGen,
      shelves: S.scene === "books" ? SH.SHELVES.map(s => ({ id: s.id, m: SH.shelfT(s).map(x => +x.toFixed(5)) })) : [],
      drag: DRAG ? { kind: DRAG.kind, id: DRAG.id, ok: DRAG.ok } : null,
      route: world.route ? { pts: world.route.pts.map(p => [+p.x.toFixed(3), +p.y.toFixed(3)]), goal: spotFoot(world.route.goal.x, world.route.goal.y, world.route.goal.yaw) } : null,
      // 미리 보기: 최종 자리 (반투명 책 — 책꽂이를 옮기면 따라간다) · Spot 이 블록대로 걸을 길 전체 (흐린 점선)
      ghosts: S.scene === "books" ? PV.ghosts.map(g => ({ id: g.id, color: g.color, size: [SH.BOOK.d, SH.BOOK.t, SH.BOOK.h], m: BW.ghostT(g).map(x => +x.toFixed(4)) })) : [],
      plan: S.scene === "books" ? PV.view.map(L => ({ pts: L.pts.map(p => [+p.x.toFixed(3), +p.y.toFixed(3)]), goal: spotFoot(L.goal.x, L.goal.y, L.goal.yaw) })) : [] });
    if (p === "/viewer/fixed") return send(res, 200, fixedState());
    if (p.startsWith("/mesh/")) { const f = MESH[p.slice(6)]; return f ? file(res, f) : send(res, 404, "없음", "text/plain"); }
    if (p === "/api/meta") return send(res, 200, {
      scenes: Object.entries(SCENES).map(([k, v]) => ({ id: k, name: v.name, robots: v.robots, soon: v.soon })), robots: ROBOT_NAMES,
      blocks: BLOCKS, speeds: ["0.5x", "1.0x", "2.0x", "4.0x"], viewer_url: "/viewer/" });
    if (p === "/api/state") {
      const prog = S.program[S.scene];
      const kinds = [...new Set(Object.values(BLOCKS[S.scene]).flatMap(b => b.args))];
      return send(res, 200, {
        scene: S.scene, robot: S.robot, speed: S.speed, t: +world.t.toFixed(1), fixMode: S.fixMode, fixModes: FIX_MODES, walk: S.walk, walks: WALKS, walkBrain: WR.name, fell: !!world.walker?.fell, showPlan: S.showPlan, planLegs: PV.legs.length, planMiss: PV.legs.filter(L => !L.pts).length,
        seq: prog.map(b => ({ type: b.type, args: b.args, text: blockText(b), argNames: (BLOCKS[S.scene][b.type]?.args ?? []).map((k, j) => argName(k, b.args[j])) })),
        options: Object.fromEntries(kinds.map(k => [k, options(k)])),
        run: { ...RUN, elapsed: RUN.running ? +(world.t - RUN.t0).toFixed(1) : 0 },
        held: world.held?.name ?? null, log: LOG.slice(-60), ...sceneInfo(prog),
      });
    }
    if (p === "/api/call" && req.method === "POST") {
      let body = ""; for await (const c of req) body += c;
      const { fn, args = [] } = JSON.parse(body || "{}");
      if (!CALLS[fn]) return send(res, 200, { ok: false, msg: `모르는 동작: ${fn}` });
      const r = CALLS[fn](...args); return send(res, 200, r ?? { ok: true });
    }
    send(res, 404, "없음", "text/plain");
  } catch (e) { send(res, 500, { ok: false, msg: String(e?.message ?? e) }); }
});
server.listen(PORT, process.env.HOST || "127.0.0.1", () => console.log(`ARM STUDIO 다중 로봇 → http://127.0.0.1:${PORT}`));
