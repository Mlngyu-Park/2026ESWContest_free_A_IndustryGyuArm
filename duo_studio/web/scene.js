// DUO STUDIO 장면 + 공통 블록 (11-01) — G1 이든 Spot 이든 같은 블록 프로그램이 돈다. three.js 없이 (Node 검사도 같은 코드)
//   블록은 '무엇을' 만 말한다 (걷기·잡기·놓기·돌기·자세·손 흔들기·팔 거두기·기다리기).
//   '어떻게' (쪼그려 앉을지 몸을 낮출지, 손으로 쥘지 집게로 물지) 는 로봇마다 g1.js · spot.js 가 계산한다.
import { rpyT } from "./base.js";

// 도킹 = 기둥 + 윗판 (두 로봇 다 손이 닿는 높이)
export const DOCK_H = 0.50, DOCK_W = 0.30, POST_W = 0.12;
export const DOCKS = [
  { id: "red", name: "빨간 도킹", color: "#e04848", x: 1.9, y: -0.9 },
  { id: "yellow", name: "노란 도킹", color: "#f2c12e", x: 2.1, y: 1.2 },
  { id: "blue", name: "파란 도킹", color: "#3f7be0", x: -0.5, y: 1.9 },
  { id: "purple", name: "보라 도킹", color: "#9b59d0", x: -1.5, y: -1.4 },
];
export const MARKS = [
  { id: "markA", name: "바닥 A", x: 0.8, y: 2.0 },
  { id: "markB", name: "바닥 B", x: -1.9, y: 0.4 },
];
export const ITEMS0 = [
  { id: "redcube", name: "빨간 큐브", kind: "cube", color: "#e04848", size: 0.05, on: "floor", x: 1.2, y: -0.3 },
  { id: "greenpen", name: "초록 펜", kind: "pen", color: "#3cb86a", r: 0.016, h: 0.12, on: "blue" },
  { id: "bluecube", name: "파란 큐브", kind: "cube", color: "#3f7be0", size: 0.05, on: "purple" },
];
export const BLOCKS = {
  walk: { name: "걷기", arg: "target", hint: "물건·도킹·바닥 자리 앞까지 걸어간다 (도킹·물건은 돌아서)" },
  grab: { name: "잡기", arg: "item", hint: "물건을 잡는다 (멀면 먼저 걸어가고, 낮으면 몸을 낮춘다)" },
  place: { name: "놓기", arg: "place", hint: "든 물건을 도킹 위·바닥 자리에 놓는다" },
  turn: { name: "돌기", arg: "angle", hint: "제자리에서 돈다" },
  body: { name: "자세", arg: "posture", hint: "서기 · 낮추기 (G1 은 쪼그려 앉기, Spot 은 몸 낮추기)" },
  wave: { name: "손 흔들기", arg: null, hint: "손을 들어 흔든다 (G1 왼손, Spot 집게)" },
  rest: { name: "팔 거두기", arg: null, hint: "빈 팔을 몸 쪽으로 거둔다 (G1 내려뜨리기, Spot 접기)" },
  wait: { name: "기다리기", arg: "sec", hint: "잠깐 멈춘다" },
};
export const ANGLES = [["90", "왼쪽 90°"], ["-90", "오른쪽 90°"], ["180", "뒤로 180°"], ["45", "왼쪽 45°"], ["-45", "오른쪽 45°"]];
export const POSTURES = [["stand", "서기"], ["low", "낮추기"]];
export const SECS = [["1", "1초"], ["2", "2초"], ["3", "3초"], ["5", "5초"]];
export const EXAMPLE = [
  { type: "wave" },
  { type: "walk", arg: "redcube" }, { type: "grab", arg: "redcube" },
  { type: "walk", arg: "yellow" }, { type: "place", arg: "yellow" },
  { type: "walk", arg: "greenpen" }, { type: "grab", arg: "greenpen" },
  { type: "walk", arg: "markA" }, { type: "place", arg: "markA" },
  { type: "rest" }, { type: "walk", arg: "home" }, { type: "wave" },
];
export const ROBOTS = [["g1", "유니트리 G1 (두 발 · 손)"], ["spot", "Spot (네 발 · 집게 팔)"]];

const jong = (w) => { const c = w.charCodeAt(w.length - 1) - 0xac00; return c >= 0 && c <= 11171 && c % 28 !== 0; };
export const eul = (w) => w + (jong(w) ? "을" : "를"), iga = (w) => w + (jong(w) ? "이" : "가");

// 장면 상태 (물건 자리) + 블록 실행기. 로봇 쪽은 이걸 물려받아 robotCompile(블록) → 동작들 을 채운다.
export class Scene {
  static boxes() {
    return DOCKS.flatMap(d => [
      { name: d.name + " 판", dock: d.id, lo: [d.x - DOCK_W / 2, d.y - DOCK_W / 2, DOCK_H - 0.04], hi: [d.x + DOCK_W / 2, d.y + DOCK_W / 2, DOCK_H] },
      { name: d.name + " 기둥", dock: d.id, lo: [d.x - POST_W / 2, d.y - POST_W / 2, 0], hi: [d.x + POST_W / 2, d.y + POST_W / 2, DOCK_H - 0.04] }]);
  }
  resetScene() {
    this.items = ITEMS0.map(it => ({ ...it }));
    for (const it of this.items) this.settle(it, it.on, it.x, it.y);
    this.held = null; this.exec = null; this.t = 0;
  }
  dock(id) { return DOCKS.find(d => d.id === id); }
  mark(id) { return MARKS.find(m => m.id === id); }
  item(id) { return this.items.find(i => i.id === id); }
  half(it) { return it.kind === "cube" ? it.size / 2 : it.h / 2; }
  width(it) { return it.kind === "cube" ? it.size / 2 : it.r; }       // 옆에서 무는 방향 반 폭
  surface(on) { return this.dock(on) ? DOCK_H : 0; }
  settle(it, on, x, y) {                            // on 위에 똑바로 세운다
    const d = this.dock(on), m = this.mark(on);
    it.on = on;
    it.x = x ?? (d ? d.x : m ? m.x : it.x); it.y = y ?? (d ? d.y : m ? m.y : it.y);
    it.z = this.surface(on) + this.half(it); it.yaw = it.yaw ?? 0; it.T = rpyT(0, 0, it.yaw, [it.x, it.y, it.z]);
  }
  occupied(place) { return this.items.find(i => i.on === place && i !== this.held); }
  targets() { return [...this.items.map(i => [i.id, i.name]), ...DOCKS.map(d => [d.id, d.name]), ...MARKS.map(m => [m.id, m.name]), ["home", "처음 자리"]]; }
  where(id) {                                       // [x, y, 높이]
    if (id === "home") return [0, 0, 0];
    const it = this.item(id); if (it) return [it.x, it.y, it.z];
    const d = this.dock(id); if (d) return [d.x, d.y, DOCK_H];
    const m = this.mark(id); if (m) return [m.x, m.y, 0];
    return null;
  }
  // 블록 → 동작들. 로봇과 상관없는 잘못은 여기서 먼저 (같은 프로그램이면 두 로봇이 같은 말로 멈춘다)
  compile(b) {
    switch (b.type) {
      case "walk":
        if (!this.where(b.arg)) throw new Error("걷기: 대상을 고르세요");
        if (this.held && b.arg === this.held.id) throw new Error("이미 들고 있는 물건입니다");
        break;
      case "grab":
        if (!this.item(b.arg)) throw new Error("잡기: 물건을 고르세요");
        if (this.held) throw new Error(`잡기: 이미 ${eul(this.held.name)} 들고 있습니다`);
        break;
      case "place": {
        if (!this.held) throw new Error("놓기: 들고 있는 물건이 없습니다");
        if (!this.dock(b.arg) && !this.mark(b.arg)) throw new Error("놓기: 자리를 고르세요");
        const o = this.occupied(b.arg); if (o) throw new Error(`놓기: ${iga(o.name)} 이미 있습니다`);
        break;
      }
      case "rest": if (this.held) throw new Error("팔 거두기: 물건을 든 채로는 거둘 수 없습니다"); break;
      case "turn": case "body": case "wave": case "wait": break;
      default: throw new Error("모르는 블록: " + b.type);
    }
    return this.robotCompile(b);
  }
  run(program, hooks = {}) { this.exec = new Exec(this, program, hooks); }
  stop() { if (this.exec) { this.exec.stopped = true; this.exec = null; } this.walker.cmd = { vx: 0, vy: 0, wz: 0 }; }
}

export class Exec {
  constructor(world, program, hooks) { this.w = world; this.program = program; this.hooks = hooks; this.i = -1; this.acts = []; this.done = false; this.error = null; }
  update(dt) {
    if (this.done) return;
    while (!this.acts.length) {
      this.i++;
      if (this.i >= this.program.length) { this.finish(null); return; }
      this.hooks.block?.(this.i);
      try { this.acts = this.w.compile(this.program[this.i]); } catch (e) { this.finish(e.message); return; }
    }
    const a = this.acts[0];
    if (!a.started) { a.started = true; const r = a.init?.(); if (typeof r === "string") { this.finish(r); return; } }
    const r = a.update(dt);
    if (typeof r === "string") { this.finish(r); return; }
    if (r) this.acts.shift();
  }
  finish(err) {
    this.done = true; this.error = err; this.w.walker.cmd = { vx: 0, vy: 0, wz: 0 };
    this.hooks.end?.(err, this.i);
    if (this.w.exec === this) this.w.exec = null;
  }
}
