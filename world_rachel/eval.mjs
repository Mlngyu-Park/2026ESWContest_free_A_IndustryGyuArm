// 월드 레이첼 시험 — 학습된 두뇌로 정해진 명령 순서를 따라 걷게 하고 잘했는지 숫자로 본다 (밀기·짐 없음, 앱과 같은 조건)
//   node eval.mjs runs/v1/policy_latest.json [씨앗]
import fs from "fs";
import RAPIER from "../duo_studio/web/vendor/rapier/rapier.es.js";
import { SpotEnv, spotBlueprint, armLoad, ACT, CTRL_DT, Z0, ARM_SHELL } from "./spot_env.mjs";
import { READY, STOW, poseT } from "../duo_studio/web/spotkin.js";
import { Policy } from "./policy.mjs";

await RAPIER.init();
const A = new URL("../duo_studio/web/assets/spot/", import.meta.url);
const PJ = JSON.parse(fs.readFileSync(process.argv[2])), VER = PJ.obs === 53 ? 2 : 1;
const bp = spotBlueprint(JSON.parse(fs.readFileSync(new URL("robot.json", A))), JSON.parse(fs.readFileSync(new URL("shapes.json", A))), VER === 2 ? { armMass: ARM_SHELL } : {});
const pol = new Policy(PJ);
const env = new SpotEnv(RAPIER, bp, { train: false, seed: +process.argv[3] || 1, maxT: 1e9, ver: VER });
// 팔 자세 → 팔 무게 점 (v2): 몸통 기준 FK
function armAt(Q, book = 0) { const rb = bp.robot, save = { ...rb.q }; Object.assign(rb.q, Q); const W = rb.fk(poseT({ x: 0, y: 0, z: Z0, roll: 0, pitch: 0, yaw: 0 })); Object.assign(rb.q, save); return armLoad(bp, W, book); }
const ARM_STOW = armAt(STOW), ARM_READY = armAt(READY, 0.5);

// [이름, 초, vx, vy, wz, 몸 자세?, 팔?]  (몸 자세·팔은 v2 만: spotb 의 NORMAL·LOW·MID·DEEP)
const NORMAL = { z: 0.52, pitch: 0 }, LOW = { z: 0.40, pitch: 0.12 }, MID = { z: 0.44, pitch: 0.15 }, DEEP = { z: 0.36, pitch: 0.35 };
const PLAN2 = [
  ["가운데 숙임 + 팔 뻗기", 4, 0, 0, 0, MID, ARM_READY], ["깊게 숙임 + 팔 뻗기", 4, 0, 0, 0, DEEP, ARM_READY], ["낮게 + 팔 뻗기", 4, 0, 0, 0, LOW, ARM_READY],
  ["낮게 숙인 채 뒤로 0.15", 3, -0.15, 0, 0, LOW, ARM_READY], ["깊게 숙인 채 뒤로 0.1", 3, -0.1, 0, 0, DEEP, ARM_READY],
  ["보통 + 팔 접기", 3, 0, 0, 0, NORMAL, ARM_STOW], ["제자리 조금 돌기 0.15", 3, 0, 0, 0.15, NORMAL, ARM_STOW], ["조금 옆으로 0.05", 3, 0, 0.05, 0, NORMAL, ARM_STOW],
  ["책 들고 앞으로 0.3", 5, 0.3, 0, 0, NORMAL, ARM_READY], ["팔 접고 멈춤", 3, 0, 0, 0, NORMAL, ARM_STOW],
];
export const PLAN = [
  ["서 있기", 3, 0, 0, 0], ["앞으로 0.4", 5, 0.4, 0, 0], ["멈춤", 2, 0, 0, 0], ["왼쪽 돌기 0.6", 4, 0, 0, 0.6],
  ["옆으로 0.25", 4, 0, 0.25, 0], ["뒤로 0.3", 4, -0.3, 0, 0], ["앞+돌기 0.3/0.4", 5, 0.3, 0, 0.4], ["오른쪽 돌기 0.8", 3, 0, 0, -0.8], ["멈춤", 3, 0, 0, 0],
];
const a = new Float32Array(ACT); let fell = false;
const rows = [];
for (const [name, secs, vx, vy, wz, body, arm] of VER === 2 ? [...PLAN, ...PLAN2] : PLAN) {
  env.cmd = [vx, vy, wz];
  if (VER === 2) { env.bodyGoal = { ...(body ?? NORMAL) }; if (arm) env.setArmLoad(arm.p, arm.m); else env.setArmLoad(ARM_STOW.p, ARM_STOW.m); }
  let ev = 0, ew = 0, n = 0, zmin = 9, zmax = 0, slip = 0, tilt = 0, start = null, tq = 0, ez = 0, ep = 0;
  const p0 = env.root.translation();
  for (let s = 0; s < Math.round(secs / CTRL_DT); s++) {
    pol.act(env.observe(), a);
    const r = env.step(a);
    if (r.fell) { fell = true; break; }
    if (s < 30) continue;                             // 명령이 바뀐 뒤 0.5초는 따라가는 중
    const rot = env.root.rotation(), lv = env.root.linvel(), av = env.root.angvel();
    const inv = (v) => { const q = { x: -rot.x, y: -rot.y, z: -rot.z, w: rot.w }, { x, y, z, w } = q, tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
      return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)]; };
    const vb = inv([lv.x, lv.y, lv.z]), wb = inv([av.x, av.y, av.z]), g = inv([0, 0, -1]);
    ev += Math.hypot(vb[0] - vx, vb[1] - vy); ew += Math.abs(wb[2] - wz); n++;
    const z = env.root.translation().z; zmin = Math.min(zmin, z); zmax = Math.max(zmax, z);
    tilt = Math.max(tilt, Math.acos(Math.min(1, -g[2])) * 57.3);
    if (VER === 2) { ez += Math.abs(z - env.bodyNow.z); ep += Math.abs(Math.asin(Math.max(-1, Math.min(1, g[0]))) - env.bodyNow.pitch); }
    tq = Math.max(tq, ...env.tau.map(Math.abs));
  }
  const p1 = env.root.translation();
  rows.push(`  ${name.padEnd(16)} 속도 오차 ${(ev / Math.max(n, 1)).toFixed(3)}m/s · 돌기 오차 ${(ew / Math.max(n, 1)).toFixed(3)}rad/s · 높이 ${zmin.toFixed(2)}~${zmax.toFixed(2)} · 기울기 최대 ${tilt.toFixed(0)}° · 힘 최대 ${tq.toFixed(0)} · 간 거리 ${Math.hypot(p1.x - p0.x, p1.y - p0.y).toFixed(2)}m${VER === 2 ? ` · 높이 오차 ${(ez / Math.max(n, 1) * 100).toFixed(1)}cm · 숙임 오차 ${(ep / Math.max(n, 1) * 57.3).toFixed(1)}°` : ""}`);
  if (fell) { rows.push(`  ✗ '${name}' 중 넘어짐 (${env.t.toFixed(1)}초)`); break; }
}
console.log(`[${process.argv[2]}] ${fell ? "넘어짐" : "끝까지 섬"} · ${env.t.toFixed(1)}초`);
rows.forEach(r => console.log(r));
