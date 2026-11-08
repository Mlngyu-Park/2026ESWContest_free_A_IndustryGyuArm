// 월드 레이첼 연습장 일꾼 — 연습장 N 개를 돌리고, 학습기(train.py)와 표준 입출력으로 숫자를 주고받는다.
//   node worker.mjs N 씨앗 [판 1|2]
//   받는 것: [명령 int32, 바이트 수 int32] + 내용.  1 = 처음부터 → 관찰 / 2 = 한 걸음(행동 N×12) → 관찰·점수·끝남·시간끝 / 3 = 통계(JSON) / 9 = 끝
import fs from "fs";
import RAPIER from "../duo_studio/web/vendor/rapier/rapier.es.js";
import { SpotEnv, spotBlueprint, OBS_OF, ACT, ARM_SHELL } from "./spot_env.mjs";

const N = +process.argv[2] || 16, SEED = +process.argv[3] || 1, VER = +process.argv[4] || 1, OBS = OBS_OF[VER];
await RAPIER.init();
const A = new URL("../duo_studio/web/assets/spot/", import.meta.url);
const bp = spotBlueprint(JSON.parse(fs.readFileSync(new URL("robot.json", A))), JSON.parse(fs.readFileSync(new URL("shapes.json", A))), VER >= 2 ? { armMass: ARM_SHELL } : {});
const envs = Array.from({ length: N }, (_, i) => new SpotEnv(RAPIER, bp, { train: true, seed: SEED * 1000 + i + 1, ver: VER }));
envs.forEach(e => e.desync());

const out = new Float32Array(N * (OBS + 3));
const ep = [];                                        // 끝난 판들
function finish(e, fell) {
  const s = Math.max(e.t, 1e-6), terms = {};
  for (const k in e.terms) terms[k] = e.terms[k] / s;
  ep.push({ ret: e.epRet, len: e.t, fell: fell ? 1 : 0, terms });
}
function reply(buf) { process.stdout.write(Buffer.from(buf.buffer, buf.byteOffset, buf.byteLength)); }

let pend = Buffer.alloc(0);
process.stdin.on("data", (chunk) => {
  pend = pend.length ? Buffer.concat([pend, chunk]) : chunk;
  while (pend.length >= 8) {
    const cmd = pend.readInt32LE(0), n = pend.readInt32LE(4);
    if (pend.length < 8 + n) break;
    const body = pend.subarray(8, 8 + n); pend = pend.subarray(8 + n);
    if (cmd === 1) {
      const o = new Float32Array(N * OBS);
      envs.forEach((e, i) => o.set(e.observe(), i * OBS));
      reply(o);
    } else if (cmd === 2) {
      const act = new Float32Array(body.buffer.slice(body.byteOffset, body.byteOffset + body.byteLength));
      const obs = out.subarray(0, N * OBS), rew = out.subarray(N * OBS, N * OBS + N), done = out.subarray(N * OBS + N, N * OBS + 2 * N), tout = out.subarray(N * OBS + 2 * N);
      envs.forEach((e, i) => {
        const r = e.step(act.subarray(i * ACT, (i + 1) * ACT));
        rew[i] = r.rew; done[i] = r.done ? 1 : 0; tout[i] = r.timeout ? 1 : 0;
        if (r.done) { finish(e, r.fell); e.reset(); }
        obs.set(e.observe(), i * OBS);
      });
      reply(out);
    } else if (cmd === 3) {
      const js = Buffer.from(JSON.stringify(ep.splice(0)));
      const h = Buffer.alloc(4); h.writeInt32LE(js.length, 0);
      process.stdout.write(Buffer.concat([h, js]));
    } else if (cmd === 9) process.exit(0);
  }
});
