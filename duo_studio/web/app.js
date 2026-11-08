// DUO STUDIO 화면 (11-01) — 같은 블록 프로그램을 G1 · Spot 중 고른 로봇으로. 장면·블록은 scene.js, 로봇 계산은 g1.js · spot.js.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { rpyT } from "./base.js";
import { DOCKS, MARKS, DOCK_H, DOCK_W, POST_W, BLOCKS, ANGLES, POSTURES, SECS, EXAMPLE, ROBOTS } from "./scene.js";
import { G1World } from "./g1.js";
import { SpotWorld } from "./spot.js";
import { Shapes } from "./collide.js";

const $ = (s) => document.querySelector(s);
const logBox = $("#log");
function log(msg, cls = "") {
  const d = document.createElement("div"); d.className = cls;
  d.textContent = `${world ? world.t.toFixed(1).padStart(5) + "초 " : ""}${msg}`;
  logBox.appendChild(d); logBox.scrollTop = logBox.scrollHeight;
}

// ── 장면 ────────────────────────────────────────────────────
const view = $("#view");
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
renderer.shadowMap.enabled = true;
view.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color("#20242c");
const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 100);
camera.position.set(-2.2, 1.9, 2.8);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.6, 0);
scene.add(new THREE.HemisphereLight("#ffffff", "#3a3f4a", 1.1));
const sun = new THREE.DirectionalLight("#ffffff", 1.6);
sun.position.set(3, 6, 2); sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -6, right: 6, top: 6, bottom: -6 });
scene.add(sun, sun.target);

// URDF 는 Z 가 위 → three 는 Y 가 위. 모든 것을 이 묶음 안에 Z 위 좌표로 둔다.
const Z = new THREE.Group(); Z.rotation.x = -Math.PI / 2; scene.add(Z);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(12, 12), new THREE.MeshStandardMaterial({ color: "#3a3f48", roughness: 0.95 }));
floor.receiveShadow = true; Z.add(floor);
const grid = new THREE.GridHelper(12, 24, "#59606d", "#4a505b"); grid.rotation.x = Math.PI / 2; grid.position.z = 0.001; Z.add(grid);

function setM(obj, M) { obj.matrix.set(...M); obj.matrixWorldNeedsUpdate = true; }
function label(text, color = "#fff") {
  const c = document.createElement("canvas"); c.width = 256; c.height = 64;
  const g = c.getContext("2d"); g.font = "bold 30px sans-serif"; g.textAlign = "center"; g.textBaseline = "middle";
  g.fillStyle = "#000a"; g.fillRect(0, 8, 256, 48); g.fillStyle = color; g.fillText(text, 128, 33);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
  s.scale.set(0.5, 0.125, 1); return s;
}
for (const d of DOCKS) {
  // 탁자 높이 받침: 기둥 + 색 윗판
  const leg = new THREE.Mesh(new THREE.BoxGeometry(POST_W, POST_W, DOCK_H - 0.04), new THREE.MeshStandardMaterial({ color: "#5b616d", roughness: 0.7 }));
  leg.position.set(d.x, d.y, (DOCK_H - 0.04) / 2); leg.castShadow = true; Z.add(leg);
  const top = new THREE.Mesh(new THREE.BoxGeometry(DOCK_W, DOCK_W, 0.04), new THREE.MeshStandardMaterial({ color: d.color, roughness: 0.6 }));
  top.position.set(d.x, d.y, DOCK_H - 0.02); top.castShadow = top.receiveShadow = true; Z.add(top);
  const l = label(d.name); l.position.set(d.x, d.y, DOCK_H + 0.35); Z.add(l);
}
for (const k of MARKS) {
  const m = new THREE.Mesh(new THREE.RingGeometry(0.11, 0.14, 4, 1), new THREE.MeshBasicMaterial({ color: "#d8dde6" }));
  m.rotation.z = Math.PI / 4; m.position.set(k.x, k.y, 0.002); Z.add(m);
  const l = label(k.name, "#d8dde6"); l.position.set(k.x, k.y, 0.25); Z.add(l);
}

// ── 로봇 (골라 끼우기) ──────────────────────────────────────
//   프로그램은 그대로 두고 로봇만 바꾼다. 모델·모양은 한 번 읽으면 기억해 둔다.
const WORLD = { g1: G1World, spot: SpotWorld };
const DRIVE = { g1: [0.3, 0.15, 0.6], spot: [0.4, 0.25, 0.8] };     // 직접 몰기 속도 (앞뒤 · 옆 · 돌기)
const loaded = {};
const loader = new STLLoader();
let world = null, robot = null, linkObj = {};
async function load(r) {
  if (loaded[r]) return loaded[r];
  const base = `assets/${r}/`, model = await (await fetch(base + "robot.json")).json();
  const shapes = new Shapes(await (await fetch(base + "shapes.json")).json());   // 부딪힘 계산용 모양
  const groups = {};
  await Promise.all(Object.entries(model.links).map(async ([name, L]) => {
    const g = new THREE.Group(); g.matrixAutoUpdate = false; groups[name] = g;
    for (const v of L.visuals) {
      const geo = loader.parse(await (await fetch(base + v.mesh)).arrayBuffer());
      geo.computeVertexNormals();
      const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: v.color || L.color, roughness: 0.5, metalness: 0.15 }));
      mesh.castShadow = true;
      mesh.matrixAutoUpdate = false; mesh.matrix.set(...rpyT(...v.rpy, v.xyz)); mesh.scale.set(...v.scale);
      g.add(mesh);
    }
  }));
  return (loaded[r] = { model, shapes, groups });
}
async function setRobot(r) {
  if (world?.exec) world.stop();
  $("#robot").disabled = true; $("#status").textContent = "로봇 불러오는 중…";
  const L = await load(r);
  for (const g of Object.values(linkObj)) Z.remove(g);
  linkObj = L.groups; for (const g of Object.values(linkObj)) Z.add(g);
  robot = r; world = new WORLD[r](L.model, log, L.shapes);
  buildItems(); rows().forEach(row => row.className = "");
  $("#robot").value = r; $("#robot").disabled = false; $("#status").textContent = "준비 — ▶ 실행을 누르세요";
  try { localStorage.setItem("duo.robot", r); } catch {}
  log(`로봇: ${ROBOTS.find(x => x[0] === r)[1]} (링크 ${Object.keys(L.model.links).length})`);
  if (window.DUO) window.DUO.world = world;
}

// ── 물건 ────────────────────────────────────────────────────
const itemObj = {};
function buildItems() {
  for (const o of Object.values(itemObj)) Z.remove(o);
  for (const it of world.items) {
    const geo = it.kind === "cube" ? new THREE.BoxGeometry(it.size, it.size, it.size)
                                   : new THREE.CylinderGeometry(it.r, it.r, it.h, 20).rotateX(Math.PI / 2);
    const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: it.color, roughness: 0.4 }));
    m.castShadow = true; m.matrixAutoUpdate = false; Z.add(m); itemObj[it.id] = m;
  }
}
function draw() {
  const W = world.fk();
  for (const [n, g] of Object.entries(linkObj)) if (W[n]) setM(g, W[n]);
  for (const it of world.items) setM(itemObj[it.id], it.T);
  const P = world.pose;
  if ($("#follow").checked) {
    const tgt = new THREE.Vector3(P.x, 0.6, -P.y);
    camera.position.add(tgt.clone().sub(controls.target)); controls.target.copy(tgt);
  }
  $("#pose").textContent = `${robot === "g1" ? "골반" : "몸통"} x ${P.x.toFixed(2)} · y ${P.y.toFixed(2)} · 방향 ${(P.yaw * 57.3).toFixed(0)}° · 높이 ${P.z.toFixed(2)}m` +
    `${world.held ? " · 든 것: " + world.held.name : ""}${world.walker.legOk ? "" : " · ⚠ 다리 한계"}`;
}

// ── 블록 코딩 판 ─────────────────────────────────────────────
let program = EXAMPLE.map(b => ({ ...b }));
function options(kind) {
  if (kind === "target") return world.targets();
  if (kind === "item") return world.items.map(i => [i.id, i.name]);
  if (kind === "place") return [...DOCKS.map(d => [d.id, d.name]), ...MARKS.map(m => [m.id, m.name])];
  if (kind === "angle") return ANGLES; if (kind === "posture") return POSTURES; if (kind === "sec") return SECS;
  return [];
}
function renderProgram() {
  const ol = $("#program"); ol.innerHTML = "";
  program.forEach((b, i) => {
    const li = document.createElement("li"); const B = BLOCKS[b.type];
    li.title = B.hint;
    li.innerHTML = `<span class="name">${B.name}</span>`;
    if (B.arg) {
      const sel = document.createElement("select");
      for (const [v, t] of options(B.arg)) sel.add(new Option(t, v, false, v === b.arg));
      if (!b.arg) b.arg = sel.value;
      sel.onchange = () => { b.arg = sel.value; };
      li.appendChild(sel);
    } else li.appendChild(Object.assign(document.createElement("span"), { style: "flex:1" }));
    for (const [t, f] of [["↑", () => i > 0 && program.splice(i - 1, 0, program.splice(i, 1)[0])],
                          ["↓", () => i < program.length - 1 && program.splice(i + 1, 0, program.splice(i, 1)[0])],
                          ["✕", () => program.splice(i, 1)]]) {
      const btn = document.createElement("button"); btn.className = "mini"; btn.textContent = t;
      btn.onclick = () => { if (world.exec) return; f(); renderProgram(); }; li.appendChild(btn);
    }
    ol.appendChild(li);
  });
}
for (const [type, B] of Object.entries(BLOCKS)) {
  const btn = document.createElement("button"); btn.textContent = "+ " + B.name; btn.title = B.hint;
  btn.onclick = () => { if (world.exec) return; program.push({ type, arg: null }); renderProgram(); };
  $("#palette").appendChild(btn);
}
$("#example").onclick = () => { if (world.exec) return; program = EXAMPLE.map(b => ({ ...b })); renderProgram(); };
$("#clear").onclick = () => { if (world.exec) return; program = []; renderProgram(); };
const rows = () => [...document.querySelectorAll("#program li")];
$("#run").onclick = () => {
  if (world.exec || !program.length) return;
  rows().forEach(r => r.className = "");
  log("실행 시작");
  world.run(program, {
    block(i) { rows().forEach((r, k) => r.className = k < i ? "done" : k === i ? "run" : ""); $("#status").textContent = `실행 중: ${i + 1}. ${BLOCKS[program[i].type].name}`; },
    end(err, i) {
      if (err) { rows()[i]?.classList.add("fail"); log(err, "bad"); $("#status").textContent = "멈춤: " + err; }
      else { rows().forEach(r => r.className = "done"); log("프로그램 끝", "ok"); $("#status").textContent = "끝"; }
    },
  });
};
$("#stop").onclick = () => { if (world.exec) { world.stop(); log("멈춤"); $("#status").textContent = "멈춤"; } };
$("#robot").onchange = () => setRobot($("#robot").value);
$("#reset").onclick = () => { world.stop(); world.reset(); buildItems(); rows().forEach(r => r.className = ""); log("처음 자리로"); $("#status").textContent = "준비"; };

// 직접 몰기 (프로그램이 안 돌 때)
const keys = new Set();
addEventListener("keydown", (e) => { if (e.target.tagName !== "SELECT") keys.add(e.key.toLowerCase()); });
addEventListener("keyup", (e) => keys.delete(e.key.toLowerCase()));
function manual() {
  if (world.exec) return;
  const k = (c) => keys.has(c) ? 1 : 0;
  const [a, b, c] = DRIVE[robot];
  world.walker.cmd = { vx: a * (k("w") - k("s")), vy: b * (k("q") - k("e")), wz: c * (k("a") - k("d")) };
}

// ── 돌리기 ──────────────────────────────────────────────────
function resize() {
  const r = view.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false); camera.aspect = r.width / r.height; camera.updateProjectionMatrix();
}
addEventListener("resize", resize); resize();
let last = performance.now();
function frame(now) {
  const dt = Math.min(0.05, (now - last) / 1000); last = now;
  const n = +$("#speed").value;
  if (!world) { requestAnimationFrame(frame); return; }
  manual();
  for (let i = 0; i < n * 2; i++) world.step(dt / 2);  // 반 프레임씩 (걷기 계산을 촘촘히)
  draw(); controls.update(); renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
for (const [v, t] of ROBOTS) $("#robot").add(new Option(t, v));
let first = new URLSearchParams(location.search).get("robot");
try { first = first ?? localStorage.getItem("duo.robot"); } catch {}
await setRobot(WORLD[first] ? first : "g1");
renderProgram();
window.DUO = { world, program, run: () => $("#run").click(), camera, controls, follow: $("#follow"), setRobot };   // 화면 검사용
requestAnimationFrame(frame);
