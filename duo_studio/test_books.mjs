// 책꽂이 공정 검사 — node test_books.mjs [ur15|spot|g1] [프로그램 이름 앞부분]
import fs from "fs";
import { initPhysics } from "./web/phys.js";
import { Shapes } from "./web/collide.js";
import { BOOK_EXAMPLE } from "./web/bookworld.js";
import { Ur15Books } from "./web/ur15b.js";
import { SpotBooks } from "./web/spotb.js";
import { shelfT, parseCell, FRONT_IN, BOOK } from "./web/shelf.js";
import { mul, inv } from "./web/base.js";
const load = (r) => [JSON.parse(fs.readFileSync(new URL(`./web/assets/${r}/robot.json`, import.meta.url))),
                     new Shapes(JSON.parse(fs.readFileSync(new URL(`./web/assets/${r}/shapes.json`, import.meta.url))))];
const ROB = { ur15: Ur15Books, spot: SpotBooks };
export const PROGS = [
  ["1-1 전부 → 1-4", [{ type: "move", args: ["1-1", "1-4", "all"] }]],
  ["바닥 → 1-6", [{ type: "floor", args: ["1-6"] }]],
  ["꽉 찬 2-8 한 권 → 1-2", [{ type: "move", args: ["2-8", "1-2", "one"] }]],
  ["예제", BOOK_EXAMPLE],
];
const R = await initPhysics(); globalThis.DBG = !!process.env.DBG;
// WALK=ai 이면 Spot 이 월드 레이첼 물리 걷기로 (BRAIN=두뇌 JSON, 기본 world_rachel/policies/v2.json)
const AI = process.env.WALK === "ai" ? { PW: (await import("../world_rachel/phys_walker.mjs")).PhysWalker, brain: JSON.parse(fs.readFileSync(process.env.BRAIN ?? new URL("../world_rachel/policies/v2.json", import.meta.url))) } : null;
const optFor = (r, model, sh) => r === "spot" && AI ? { makeWalker: (robot, pose, w) => new AI.PW(R, robot, model, sh, AI.brain, { pose, book: () => w.held ? (w.held.mass ?? 0.5) : 0 }) } : {};
const robots = process.argv[2] && ROB[process.argv[2]] ? [process.argv[2]] : Object.keys(ROB), only = process.argv[3];
let bad = 0;
for (const r of robots) {
  const [model, sh] = load(r);
  for (const [name, prog] of PROGS) {
    if (only && !name.startsWith(only)) continue;
    const logs = [], w = new ROB[r](model, sh, R, (m) => logs.push(`${w.t.toFixed(1)}초 ${m}`), optFor(r, model, sh));
    if (process.env.WHO) { const f = w.clashText.bind(w); w.clashText = (c) => f(c) + ` [${c.who} ${(c.depth * 1000).toFixed(0)}]`; }
    let err, ended = false, worst = null, t0 = Date.now();
    w.run(prog, { end: (e) => { err = e; ended = true; } });
    while (!ended && w.t < 900) {
      w.step(1 / 60);
      const c = w.clash({ books: false }); if (c && (!worst || c.depth > worst.depth)) worst = { ...c, t: w.t };
    }
    const where = {}; for (const b of w.books) (where[w.cellOf(b) ?? (b.T[11] < 0.06 ? "바닥" : "?")] ??= []).push(b.name.replace("책 ", ""));
    const tilt = w.books.map(b => Math.acos(Math.min(1, Math.abs(b.T[10]))) * 57.3);
    console.log(`\n[${r} · ${name}] ${err ? "멈춤: " + err : "끝"} · ${w.t.toFixed(1)}초 (계산 ${((Date.now() - t0) / 1000).toFixed(0)}초)`);
    logs.forEach(l => console.log("  " + l));
    console.log("  칸별:", Object.entries(where).map(([k, v]) => `${k}[${v.join(",")}]`).join(" "));
    // 꽂힌 책: 기울기 · 앞면 깊이(목표 FRONT_IN) · 옆 책과 겹침
    let worstTilt = 0, outMax = 0, inMax = 0;
    for (const b of w.books) {
      const c = w.cellOf(b); if (!c) continue;
      const L = mul(inv(shelfT(parseCell(c).shelf)), b.T), f = L[3] - BOOK.d / 2 - FRONT_IN;
      worstTilt = Math.max(worstTilt, Math.acos(Math.min(1, Math.abs(b.T[10]))) * 57.3);
      outMax = Math.max(outMax, -f); inMax = Math.max(inMax, f);
    }
    console.log(`  꽂힌 책: 가장 기운 것 ${worstTilt.toFixed(1)}° · 튀어나온 것 ${(Math.max(0, outMax) * 1000).toFixed(1)}mm · 더 들어간 것 ${(Math.max(0, inMax) * 1000).toFixed(1)}mm`);
    if (outMax > 0.003 || worstTilt > 3) bad++;
    console.log("  로봇이 책꽂이·바닥·몸 뚫음:", worst ? `${worst.kind} ${worst.who} ${(worst.depth * 1000).toFixed(1)} (${worst.t.toFixed(1)}초)` : "없음");
    if (err || worst) bad++;
  }
}
process.exit(bad ? 1 : 0);
