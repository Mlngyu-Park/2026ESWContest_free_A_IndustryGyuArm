// 잘못된 프로그램은 두 로봇 다 같은 말로 멈추나 — node test_errors.mjs
import fs from "fs";
import { G1World } from "./web/g1.js";
import { SpotWorld } from "./web/spot.js";
import { Shapes } from "./web/collide.js";
const load = (r) => [JSON.parse(fs.readFileSync(new URL(`./web/assets/${r}/robot.json`, import.meta.url))),
                     new Shapes(JSON.parse(fs.readFileSync(new URL(`./web/assets/${r}/shapes.json`, import.meta.url))))];
const cases = [
  ["빈손으로 놓기", [{ type: "place", arg: "yellow" }]],
  ["두 개 잡기", [{ type: "grab", arg: "redcube" }, { type: "grab", arg: "bluecube" }]],
  ["찬 자리에 놓기", [{ type: "grab", arg: "redcube" }, { type: "place", arg: "blue" }]],
  ["든 채로 팔 거두기", [{ type: "grab", arg: "redcube" }, { type: "rest" }]],
  ["든 물건으로 걷기", [{ type: "grab", arg: "redcube" }, { type: "walk", arg: "redcube" }]],
];
let bad = 0;
const W = { g1: G1World, spot: SpotWorld }, got = {};
for (const r of ["g1", "spot"]) {
  const [model, sh] = load(r);
  for (const [name, prog] of cases) {
    const w = new W[r](model, () => {}, sh); let err = null, ended = false;
    w.run(prog, { end: (e) => { err = e; ended = true; } });
    while (!ended && w.t < 150) w.step(1 / 60);
    (got[name] ??= {})[r] = err;
  }
}
for (const [name] of cases) {
  const { g1, spot } = got[name], ok = !!g1 && g1 === spot;
  if (!ok) bad++;
  console.log(`  ${ok ? "통과" : "실패"} ${name}: G1 「${g1}」 · Spot 「${spot}」`);
}
process.exit(bad ? 1 : 0);
