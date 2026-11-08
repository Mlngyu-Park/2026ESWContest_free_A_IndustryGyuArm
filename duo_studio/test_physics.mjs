// 같은 블록 프로그램을 G1·Spot 둘 다로 — 최소한의 물리 검사. node test_physics.mjs [g1|spot] [프로그램 이름 앞부분]
import fs from "fs";
import { EXAMPLE, BLOCKS, DOCKS, DOCK_H, Scene } from "./web/scene.js";
import { G1World } from "./web/g1.js";
import { SpotWorld } from "./web/spot.js";
import { Shapes } from "./web/collide.js";
import { makeChecker, report } from "./physics.mjs";
const load = (r) => [JSON.parse(fs.readFileSync(new URL(`./web/assets/${r}/robot.json`, import.meta.url))),
                     new Shapes(JSON.parse(fs.readFileSync(new URL(`./web/assets/${r}/shapes.json`, import.meta.url))))];
const surface = (on) => DOCKS.some(d => d.id === on) ? DOCK_H : 0;
// 감사 설정 (로봇마다 몸 링크 이름만 다르다)
const g1arm = (s) => ["shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw", "hand_palm", "hand_thumb_0", "hand_thumb_1", "hand_thumb_2",
  "hand_middle_0", "hand_middle_1", "hand_index_0", "hand_index_1"].map(n => `${s}_${n}_link`);
const g1leg = (s) => ["hip_roll", "hip_yaw", "knee", "ankle_roll"].map(n => `${s}_${n}_link`);
const g1body = ["pelvis", "torso_link", "head_link", ...g1leg("left"), ...g1leg("right")];
const spLeg = (l) => ["upper_leg", "lower_leg"].map(n => `${l}_${n}`);
const spArm = ["arm_link_el0", "arm_link_el1", "arm_link_wr0", "arm_link_wr1", "arm_link_fngr"];
const SP = ["front_left", "front_right", "rear_left", "rear_right"];
export const ROBOT = {
  g1: { World: G1World, scene: (sh) => ({ cfg: { links: Object.keys(sh.L), boxes: Scene.boxes(), selfBody: g1body, hand: new Set(g1arm("right").slice(5)),
          self: [[g1arm("right"), g1body], [g1arm("left"), g1body], [g1arm("right"), g1arm("left")], [g1leg("left"), g1leg("right")], [[...g1leg("left"), ...g1leg("right")], ["torso_link", "head_link"]]] },
        thumb: ["right_hand_thumb_1_link", "right_hand_thumb_2_link"], fingers: ["right_hand_middle_0_link", "right_hand_middle_1_link", "right_hand_index_0_link", "right_hand_index_1_link"], surface }) },
  spot: { World: SpotWorld, scene: (sh) => ({ cfg: { links: Object.keys(sh.L), boxes: Scene.boxes(), selfBody: ["body", ...SP.flatMap(spLeg)], hand: new Set(["arm_link_wr1", "arm_link_fngr"]),
          self: [[spArm, ["body", ...SP.flatMap(spLeg)]], [spLeg("front_left"), spLeg("rear_left")], [spLeg("front_right"), spLeg("rear_right")], [spLeg("front_left"), spLeg("front_right")], [spLeg("rear_left"), spLeg("rear_right")]] },
        thumb: ["arm_link_wr1"], fingers: ["arm_link_fngr"], surface }) },
};
// 두 로봇에 똑같이 돌리는 프로그램
export const PROGS = [
  ["예제", EXAMPLE],
  ["탁자→탁자", [{ type: "grab", arg: "bluecube" }, { type: "place", arg: "red" }, { type: "rest" }]],
  ["바닥→바닥", [{ type: "grab", arg: "redcube" }, { type: "place", arg: "markB" }]],
  ["바닥→탁자→바닥", [{ type: "grab", arg: "redcube" }, { type: "place", arg: "red" }, { type: "grab", arg: "redcube" }, { type: "place", arg: "markA" }]],
  ["펜 탁자→바닥→탁자", [{ type: "grab", arg: "greenpen" }, { type: "place", arg: "markB" }, { type: "grab", arg: "greenpen" }, { type: "place", arg: "yellow" }]],
  ["자세·돌기·손 흔들기·팔 거두기", [{ type: "turn", arg: "90" }, { type: "body", arg: "low" }, { type: "wave" }, { type: "body", arg: "stand" }, { type: "wave" }, { type: "rest" }]],
  ["가로질러 걷기", [{ type: "walk", arg: "purple" }, { type: "walk", arg: "yellow" }, { type: "walk", arg: "markB" }, { type: "walk", arg: "red" }, { type: "walk", arg: "home" }]],
];
if (import.meta.url === `file://${process.argv[1]}`) {
  const robots = process.argv[2] && ROBOT[process.argv[2]] ? [process.argv[2]] : ["g1", "spot"], only = process.argv[3];
  let total = 0;
  for (const r of robots) {
    const [model, sh] = load(r), scene = ROBOT[r].scene(sh);
    for (const [name, prog] of PROGS) {
      if (only && !name.startsWith(only)) continue;
      const w = new ROBOT[r].World(model, () => {}, sh); let ended = false, err;
      if (process.env.WHO) { const f = w.clashText.bind(w); w.clashText = (c) => f(c) + ` [${c.who}]`; }
      w.blockName = () => w.exec ? `${w.exec.i + 1}.${BLOCKS[w.exec.program[w.exec.i]?.type]?.name ?? ""}` : "";
      const chk = makeChecker(w, sh, scene);
      w.run(prog, { end: (e) => { err = e; ended = true; } });
      while (!ended && w.t < 400) { w.step(1 / 60); chk.frame(); }
      total += report(`${r} · ${name} — ${err ? "오류: " + err : "끝 " + w.t.toFixed(1) + "초"}`, chk);
      if (err) total++;
    }
  }
  console.log(`\n${total ? "문제 있음" : "통과"} (문제 ${total})`);
  process.exit(total ? 1 : 0);
}
