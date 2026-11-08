// 최소한의 물리 검사 (11-01) — 화면 없이 매 프레임 web/collide.js 로 본다.
//   바닥·도킹·물건 뚫기, 몸끼리 뚫기(팔↔몸통·다리, 다리↔다리), 든 물건이 바닥·도킹·물건·몸을 뚫기,
//   잡는 순간 엄지와 맞은편 손가락이 물건에 닿았나, 놓는 순간 물건이 떠 있거나 파묻혀 있다가 순간 이동하나.
//   SPOT·G1 이 같이 쓴다 (scene 설정만 다름).
import { scan, gap, itemPoints } from "./web/collide.js";

const TOUCH = 0.004;                                 // 이 안이면 닿았다고 본다

export function makeChecker(world, sh, scene) {
  const issues = new Map(), events = [];
  const add = (kind, who, depth, extra = "") => {
    const k = kind + "|" + who, o = issues.get(k);
    if (!o) issues.set(k, { kind, who, n: 1, worst: depth, first: world.t, block: world.blockName?.() ?? "", extra });
    else { o.n++; if (depth > o.worst) { o.worst = depth; if (extra) o.extra = extra; } }
  };
  let prevHeld = null, prevHeldT = null;
  function frame() {
    const W = world.fk(), held = world.held;
    const blk = world.exec ? world.exec.program[world.exec.i] : null;
    const grip = blk && blk.type === "grab" ? world.item(blk.arg) : null;
    for (const c of scan(sh, W, world.items, held, { ...scene.cfg, grip })) add(c.kind, c.who, c.depth);
    if (held && held !== prevHeld) {                 // 잡는 순간
      const gt = gap(sh, W, scene.thumb, held), gf = gap(sh, W, scene.fingers, held);
      events.push({ t: world.t, kind: "잡기", item: held.name, thumb: gt, finger: gf });
      if (gt > TOUCH || gf > TOUCH) add("안 닿고 잡음", held.name, Math.max(gt, gf), `엄지 ${(gt * 1000).toFixed(0)}mm · 맞은편 ${(gf * 1000).toFixed(0)}mm 떨어짐`);
    }
    if (!held && prevHeld) {                         // 놓는 순간
      const it = prevHeld, surf = scene.surface(it.on);
      const before = Math.min(...itemPoints(it, prevHeldT).map(p => p[2]));
      const jump = Math.hypot(it.T[3] - prevHeldT[3], it.T[7] - prevHeldT[7], it.T[11] - prevHeldT[11]);
      const tilt = Math.acos(Math.min(1, Math.abs(prevHeldT[10])));
      events.push({ t: world.t, kind: "놓기", item: it.name, gap: before - surf, jump, tilt });
      if (Math.abs(before - surf) > 0.004) add("놓을 때 순간 이동", it.name, Math.abs(before - surf), `밑면이 면보다 ${((before - surf) * 1000).toFixed(0)}mm ${before > surf ? "위 (떠 있다 떨어짐)" : "아래 (파묻혀 있었음)"}`);
      if (tilt > 0.05) add("놓을 때 저절로 바로 섬", it.name, tilt, `${(tilt * 57.3).toFixed(0)}° 기운 채로 놓음`);
    }
    prevHeld = held; prevHeldT = held ? held.T.slice() : null;
  }
  return { frame, issues, events };
}

export function report(name, chk, verbose = true) {
  const L = [...chk.issues.values()].sort((a, b) => b.worst - a.worst);
  console.log(`\n[${name}] 문제 ${L.length}가지`);
  for (const o of verbose ? L : L.slice(0, 12)) {
    const unit = o.kind.includes("바로 섬") ? `${(o.worst * 57.3).toFixed(0)}°` : o.kind === "몸끼리" || o.kind === "든 물건-몸속" ? `${Math.round(o.worst * 1000)}점` : `${(o.worst * 1000).toFixed(1)}mm`;
    console.log(`  ${o.kind} · ${o.who} · 최대 ${unit} · ${o.n}프레임 · 처음 ${o.first.toFixed(1)}초 ${o.block}${o.extra ? " · " + o.extra : ""}`);
  }
  for (const e of chk.events) console.log(e.kind === "잡기"
    ? `  (잡기 ${e.t.toFixed(1)}초 ${e.item}: 엄지 ${(e.thumb * 1000).toFixed(1)}mm · 맞은편 ${(e.finger * 1000).toFixed(1)}mm)`
    : `  (놓기 ${e.t.toFixed(1)}초 ${e.item}: 밑면-면 ${(e.gap * 1000).toFixed(1)}mm · 이동 ${(e.jump * 1000).toFixed(1)}mm · 기울기 ${(e.tilt * 57.3).toFixed(0)}°)`);
  return L.length;
}
