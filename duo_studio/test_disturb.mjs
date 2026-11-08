// 실행 중 건드리기 검사 (11-02) — [MODE=now|book|block|end] node test_disturb.mjs [ur15|spot] [시나리오 이름 앞부분]
//   MODE = 건드린 책을 언제 고치나 (바로 · 이 책 끝내고 · 이 블록 끝내고 · 맨 끝에). TREE=1 이면 끝난 뒤 블록 나무를 찍는다
//   사용자가 3D 에서 끌어 옮기는 것을 흉내: 실행 도중 t초에 책꽂이를 옮기거나 책을 다른 데 놓고 interrupt().
//   통과 = 끝까지 돌고, 모든 책이 ▶ 누를 때 정한 최종 목표 자리에 있고, 로봇이 책꽂이·바닥·몸을 안 뚫음.
//   책꽂이 = 돈케어 (옮겨도 칸 이름으로 찾아감) · 책 = 케어 (있어야 할 자리에 없으면 목표로 고쳐 놓음)
import fs from "fs";
import { initPhysics } from "./web/phys.js";
import { Shapes } from "./web/collide.js";
import { BOOK_EXAMPLE, blockTree } from "./web/bookworld.js";
import { Ur15Books } from "./web/ur15b.js";
import { SpotBooks } from "./web/spotb.js";
import { SHELVES, resetShelves, BOOK, slotT, parseCell } from "./web/shelf.js";
import { rpyT } from "./web/base.js";
const load = (r) => [JSON.parse(fs.readFileSync(new URL(`./web/assets/${r}/robot.json`, import.meta.url))),
                     new Shapes(JSON.parse(fs.readFileSync(new URL(`./web/assets/${r}/shapes.json`, import.meta.url))))];
const ROB = { ur15: Ur15Books, spot: SpotBooks };
const book = (w, id) => w.books.find(b => b.id === id);
const shelf = (id) => SHELVES.find(s => s.id === id);
const floorAt = (x, y, yaw) => rpyT(Math.PI / 2, 0, yaw, [x, y, BOOK.t / 2 + 0.001]);
// 사용자가 놓는 것처럼: 들고 있던 책이면 손에서 뺀다
let TOUCH = 0;
function userPut(w, bk, T) { if (w.held === bk) { w.held = null; w.phys.release(bk); bk.rel = null; } w.placeBook(bk, T); w.phys.wakeAll(); bk.touched = ++TOUCH; }
const SC = [
  ["책꽂이 1 옮기고 돌리기", 12, (w) => { const s = shelf(1); w.moveShelf(1, s.x + 0.08, s.y - 0.12, s.yaw - 0.3); }],
  ["상관없는 책 9 를 바닥에", 12, (w) => userPut(w, book(w, "b9"), floorAt(-0.45, -0.62, 0.7))],
  ["들고 가는 책을 빼서 바닥에", 0, (w) => userPut(w, w.held, floorAt(0.25, -0.62, -0.4)), (w) => !!w.held],
  ["책 4 를 다른 칸(2-2)에", 20, (w) => { const sh = parseCell("2-2").shelf; userPut(w, book(w, "b4"), slotT(sh, 2, 0.12)); }],
  ["나중에 꺼낼 칸의 책 6 을 바닥에", 14, (w) => userPut(w, book(w, "b6"), floorAt(-0.45, -0.62, 0.7))],
  // 두 권: 책 3 을 먼저, 책 9 를 나중에 던짐 → 고칠 때는 최근 것(책 9)부터
  ["두 권 던지기 (책 3 다음 책 9)", 14, (w) => { userPut(w, book(w, "b3"), floorAt(0.25, -0.62, -0.4)); userPut(w, book(w, "b9"), floorAt(-0.45, -0.62, 0.7)); }],
  ["책꽂이 2 를 멀리", 30, (w) => { w.moveShelf(2, -0.35, 0.95, Math.PI / 2 + 0.35); }],
  // 걷는 로봇만 (UR15 는 팔이 안 닿는 거리 — 그건 맞는 반응)
  ["사용자 로그 배치 (둘 다 왼쪽 멀리)", 20, (w) => { w.moveShelf(1, -1.09, 0.98, 0); w.moveShelf(2, -1.31, 2.58, Math.PI / 2); }, null, "spot"],
  ["좁은 통로 (마주 보게 1.6m)", 20, (w) => { w.moveShelf(1, 0.2, -0.85, -Math.PI / 2); }, null, "spot"],
];
const R = await initPhysics();
// WALK=ai 이면 Spot 이 월드 레이첼 물리 걷기로 (BRAIN=두뇌 JSON, 기본 world_rachel/policies/v2.json)
const AI = process.env.WALK === "ai" ? { PW: (await import("../world_rachel/phys_walker.mjs")).PhysWalker, brain: JSON.parse(fs.readFileSync(process.env.BRAIN ?? new URL("../world_rachel/policies/v2.json", import.meta.url))) } : null;
const optFor = (r, model, sh) => r === "spot" && AI ? { makeWalker: (robot, pose, w) => new AI.PW(R, robot, model, sh, AI.brain, { pose, book: () => w.held ? (w.held.mass ?? 0.5) : 0 }) } : {};
const robots = process.argv[2] && ROB[process.argv[2]] ? [process.argv[2]] : Object.keys(ROB), only = process.argv[3];
let bad = 0;
for (const r of robots) {
  const [model, sh] = load(r);
  for (const [name, at, act, when, onlyRobot] of SC) {
    if (only && !name.startsWith(only)) continue;
    if (onlyRobot && onlyRobot !== r) continue;
    resetShelves();
    const logs = [], w = new ROB[r](model, sh, R, (m) => logs.push(`${w.t.toFixed(1)}초 ${m}`), optFor(r, model, sh));
    let err, ended = false, worst = null, fired = false, t0 = Date.now();
    w.fixMode = process.env.MODE ?? "now";
    let blk = -1, ex = null, treeBad = null, nT = 0;
    w.run(BOOK_EXAMPLE, { end: (e) => { err = e; ended = true; ex = w.exec; }, block: (i) => { blk = i; } });
    const G = w.exec.G;
    while (!ended && w.t < 1200) {
      w.step(1 / 60);
      if (!fired && w.t >= at && (!when || when(w))) { fired = true; act(w); logs.push(`${w.t.toFixed(1)}초 ✋ 사용자: ${name}`); w.exec?.interrupt(); }
      const c = w.clash({ books: false }); if (c && (!worst || c.depth > worst.depth)) worst = { ...c, t: w.t };
      if (++nT % 30 === 0 && w.exec) {                 // 블록 나무: 계산이 깨지지 않고, '지금' 은 한 군데만
        try { const T = blockTree(w, BOOK_EXAMPLE, G, w.exec), cur = T.blocks.flatMap(b => b.nodes).filter(n => n.state === "cur");
              if (cur.length > 1) treeBad = treeBad ?? `${w.t.toFixed(1)}초 지금이 ${cur.length}곳`; }
        catch (e) { treeBad = treeBad ?? `${w.t.toFixed(1)}초 ${e.message}`; }
      }
    }
    if (process.env.TREE) { const T = blockTree(w, BOOK_EXAMPLE, G, ex, !err);
      T.blocks.forEach(b => { console.log(b.end ? "  [마무리]" : `  [${b.k + 1}번 블록]`); b.nodes.forEach(n => console.log(`     ${n.fix ? "⟲ 고치기 " : ""}${n.book} ${n.from} → ${n.to} ${n.state}${n.skip ? " (건너뜀)" : ""} | ${n.leaves.map(l => l.label + "[" + l.chips.join(" ") + "]").join(" ")}`)); }); }
    const wrong = w.books.filter(b => w.locOf(b) !== G.goal.get(b)).map(b => `${b.name}: ${w.locOf(b)} (목표 ${G.goal.get(b)})`);
    console.log(`\n[${r} · ${name}] ${err ? "멈춤: " + err : "끝"} · ${w.t.toFixed(1)}초 (계산 ${((Date.now() - t0) / 1000).toFixed(0)}초)${fired ? "" : " · 건드리기 안 함!"}`);
    (process.env.ALL ? logs : logs.filter(l => /✋|있어야|들고 있던|이미 목표|그대로|멈춤|꽂음|→ /.test(l))).forEach(l => console.log("  " + l));
    console.log("  목표와 다른 책:", wrong.length ? wrong.join(" · ") : "없음");
    console.log("  로봇이 책꽂이·바닥·몸 뚫음:", worst ? `${worst.kind} ${worst.who} ${(worst.depth * 1000).toFixed(1)} (${worst.t.toFixed(1)}초)` : "없음");
    console.log("  블록 나무:", treeBad ?? "괜찮음");
    if (err || wrong.length || worst || !fired || treeBad) bad++;
  }
}
process.exit(bad ? 1 : 0);
