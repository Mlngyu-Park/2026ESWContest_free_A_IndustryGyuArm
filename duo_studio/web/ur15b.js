// DUO STUDIO 책꽂이 — UR15 (11-02): 같은 공정(bookworld.js)에 UR15 팔 + 직접 만든 평행 집게를 끼운다.
//   UR15 는 걷지 못한다 → 가운데 받침대(높이 0.45m) 위에 고정, 책꽂이 셋·정렬대가 다 팔 닿는 곳에 있다.
//   특징점: pinch = 집게 tcp (손가락 끝에서 2cm 안, z = 손 끝 방향, y = 닫히는 축), tip = 손가락 끝.
import { Robot, rpyT, mul, inv } from "./base.js";
import { Chain } from "./g1kin.js";
import { BookWorld, aDo, lerp } from "./bookworld.js";
import { BOOK } from "./shelf.js";
import { gap } from "./collide.js";

export const J = ["j1", "j2", "j3", "j4", "j5", "j6"];
export const PEDESTAL = { h: 0.45, w: 0.32 };
const STROKE = 0.085, PAD = 0.012;                  // 손가락 하나 벌림 한계 · 책보다 이만큼 더 벌려 다가감
const smooth = (u) => u * u * (3 - 2 * u);
let rs = 12345; const rnd = () => ((rs = (rs * 1103515245 + 12345) % 2147483648) / 2147483648) * 2 * Math.PI - Math.PI;
const SEEDS = [...Array(24)].map(() => [rnd(), rnd(), rnd(), rnd(), rnd(), rnd()]);

export class Ur15Books extends BookWorld {
  constructor(model, shapes, R, log = () => {}) {
    super();
    this.kind = "ur15"; this.model = model; this.sh = shapes; this.log = log;
    this.handLinks = ["grip_base", "grip_finger_l", "grip_finger_r"];
    const fore = ["l4", "l5", "l6", "grip_base", "grip_finger_l", "grip_finger_r"];
    this.cfg = { links: ["l2", "l3", ...fore], hand: new Set(this.handLinks), self: [[fore, ["base_link", "l1", "l2"]]], selfBody: ["base_link", "l1", "l2", "l3"] };
    this.fixtures = () => [{ name: "받침대", lo: [-PEDESTAL.w / 2, -PEDESTAL.w / 2, 0], hi: [PEDESTAL.w / 2, PEDESTAL.w / 2, PEDESTAL.h - 0.002] }];
    this.jointSpeed = 1.1; this.symmetric = true;     // 평행 집게는 좌우 대칭
    this.name = "UR15"; this.pinchClear = 0.016; this.hookHalf = 0.011; this.pullOut = 0.15;   // 손가락 두께 1.2cm+여유 · 갈고리(손가락) 폭 2.2cm · 꺼낼 때 15cm (아랫단에서 더 빼면 팔이 받침대에 닿는다)
    this.setup(R);
  }
  resetRobot() {
    this.robot = new Robot(this.model);
    this.pinchCh = new Chain(this.robot, "base_link", "tcp", [0, 0, 0], J);
    this.tipCh = new Chain(this.robot, "base_link", "tcp", [0, 0, 0.02], J);
    this.grip(0);
    if (!Ur15Books.HOME) {                          // 처음 자세: 손 끝이 앞(+x)을 보고 가슴 높이 (한 번 풀어 둔다)
      this.setQ([0, -0.4, 1.2, 0.8, 0, 0]);
      this.pinchCh.solve([0.45, 0, 0.45], [[0, [0, 0, -1]], [2, [1, 0, 0]]], 300);
      Ur15Books.HOME = this.getQ();
    }
    this.setQ(Ur15Books.HOME);
  }
  baseT() { return rpyT(0, 0, 0, [0, 0, PEDESTAL.h]); }
  fk() { return this.robot.fk(this.baseT()); }
  pose() { return { x: 0, y: 0, yaw: 0, z: PEDESTAL.h }; }
  feat(name, W = this.fk()) { return name === "tip" ? mul(W.tcp, rpyT(0, 0, 0, [0, 0, 0.02])) : W.tcp; }
  chainFor(name) { return name === "tip" ? this.tipCh : this.pinchCh; }
  getQ() { return J.map(n => this.robot.q[n]); }
  setQ(q) { J.forEach((n, i) => this.robot.q[n] = q[i]); }
  // 출발점: 지금 · 처음 자세 · 고정된 흩어진 자세들 (같은 결과가 나오게 의사난수 고정)
  seeds() { return [this.getQ(), Ur15Books.HOME, ...SEEDS]; }
  // 관절은 ±2π 까지 돌 수 있다 → 지금 자세에 가장 가까운 같은 각도로
  unwrap(q, ref) { return q.map((v, i) => { let x = v; while (x - ref[i] > Math.PI && x - 2 * Math.PI >= -2 * Math.PI) x -= 2 * Math.PI; while (ref[i] - x > Math.PI && x + 2 * Math.PI <= 2 * Math.PI) x += 2 * Math.PI; return x; }); }
  altWraps(q) { return [2 * Math.PI, -2 * Math.PI].map(d => [q[0] + d, ...q.slice(1)]).filter(v => Math.abs(v[0]) <= 2 * Math.PI); }   // 밑동(j1)을 반대로 돌아서
  vias() { const q = this.getQ(), h = Ur15Books.HOME; return [[q[0], h[1], h[2], h[3], h[4], h[5]], h]; }   // 팔을 거둬 돌고 다시 뻗기
  base() { return []; }                              // 고정 로봇
  // 갈고리: 닫은 집게를 세로로 세워(아래 손가락 = grip_l) 책 위로 넘긴 뒤 벌리면 아래 손가락이 책 뒤로 3.5cm 내려간다
  //   손가락 길이 10cm 가 다 책 뒤에 있어야 내려갈 때 책 윗면을 안 친다 → 손끝을 책 뒷면에서 10.5cm 뒤까지
  hookCfg = { above: 0.06, behind: 0.105, pull: 0.055, s: 0.083 };   // 당기기 5.5cm = 손가락이 뒷면에 닿기까지 5mm + 책이 4cm 나오게
  hookDown(bk, H) {
    const w = this; let t = 0;
    return { init() { const s0 = w.gripS(); w.grip(H.s); const h = w.clash({ allow: new Set() }); w.grip(s0); if (h) return `갈고리 내리기: ${w.clashText(h)}`; },
             update(dt) { t += dt; w.grip(H.s * smooth(Math.min(1, t / 0.5))); return t >= 0.5; } };
  }
  hookUp() { return this.gripTo("closed"); }
  // ── 집게 ──
  grip(s) { this.robot.q.grip_l = s; this.robot.q.grip_r = s; }
  gripS() { return this.robot.q.grip_l; }
  fingerGaps(W, bk) { return ["grip_finger_l", "grip_finger_r"].map(l => gap(this.sh, W, [l], bk)); }
  widthOf(bk, mode) { return mode === "wide" ? BOOK.d / 2 : BOOK.t / 2; }
  gripTo(arg, mode) {                               // "closed" · 책(그 책보다 PAD 더 벌림)
    const w = this; let a, b, t = 0;
    return { init() { a = w.gripS(); b = arg === "closed" ? 0 : Math.min(STROKE, w.widthOf(arg, mode) + PAD); },
             update(dt) { t += dt; w.grip(a + (b - a) * smooth(Math.min(1, t / 0.4))); return t >= 0.4; } };
  }
  gripOn(bk, mode) {                                // 책 폭보다 0.5mm 더 닫는다 → 덜 맞은 쪽 손가락이 물리로 책을 밀어 가운데로
    const w = this; let a, b, t = 0;
    return { init() { a = w.gripS(); b = w.widthOf(bk, mode) - 0.0005; },
             update(dt) { t += dt; w.grip(a + (b - a) * smooth(Math.min(1, t / 0.5))); return t >= 0.75; } };
  }
}
