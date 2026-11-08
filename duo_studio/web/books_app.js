// 책꽂이 공정 화면 (11-02) — 같은 블록·같은 공정을 고른 로봇으로. 책·책꽂이는 물리 엔진(Rapier), 로봇은 계산.
//   장면·칸은 shelf.js, 공정은 bookworld.js, 로봇은 ur15b.js (· spotb.js · g1b.js).
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { rpyT } from "./base.js";
import { Shapes } from "./collide.js";
import { initPhysics } from "./phys.js";
import { SHELVES, shelfT, shelfPanels, cellBox, CELLS, STAND, standBox } from "./shelf.js";
import { BOOK_BLOCKS, BOOK_EXAMPLE, COUNTS, SECS } from "./bookworld.js";
import { Ur15Books, PEDESTAL } from "./ur15b.js";
import { SpotBooks } from "./spotb.js";

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
camera.position.set(-2.4, 2.0, 1.8);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.2, 0.6, 0);
scene.add(new THREE.HemisphereLight("#ffffff", "#3a3f4a", 1.1));
const sun = new THREE.DirectionalLight("#ffffff", 1.5);
sun.position.set(-2, 6, 3); sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -3, right: 3, top: 3, bottom: -3 });
scene.add(sun, sun.target);
const Z = new THREE.Group(); Z.rotation.x = -Math.PI / 2; scene.add(Z);          // URDF 처럼 Z 위
const floor = new THREE.Mesh(new THREE.PlaneGeometry(8, 8), new THREE.MeshStandardMaterial({ color: "#3a3f48", roughness: 0.95 }));
floor.receiveShadow = true; Z.add(floor);
const grid = new THREE.GridHelper(8, 32, "#59606d", "#4a505b"); grid.rotation.x = Math.PI / 2; grid.position.z = 0.001; Z.add(grid);
const setM = (obj, M) => { obj.matrix.set(...M); obj.matrixWorldNeedsUpdate = true; };
function label(text, color = "#fff", w = 0.32) {
  const c = document.createElement("canvas"); c.width = 256; c.height = 64;
  const g = c.getContext("2d"); g.font = "bold 34px sans-serif"; g.textAlign = "center"; g.textBaseline = "middle";
  g.fillStyle = "#000a"; g.fillRect(0, 8, 256, 48); g.fillStyle = color; g.fillText(text, 128, 33);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
  s.scale.set(w, w / 4, 1); return s;
}
const boxMesh = (lo, hi, color, opts = {}) => {
  const m = new THREE.Mesh(new THREE.BoxGeometry(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]), new THREE.MeshStandardMaterial({ color, roughness: 0.75, ...opts }));
  m.position.set((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2); m.castShadow = m.receiveShadow = true; return m;
};
// 책꽂이: 판들 + 칸 이름표 (칸 앞 윗쪽에 작게)
for (const s of SHELVES) {
  const g = new THREE.Group(); g.matrixAutoUpdate = false; setM(g, shelfT(s)); Z.add(g);
  for (const p of shelfPanels()) g.add(boxMesh(p.lo, p.hi, s.color));
  for (let k = 1; k <= 9; k++) { const b = cellBox(k), l = label(`${s.id}-${k}`, "#ffe9a8", 0.16); l.material.depthTest = true; l.position.set(-0.012, (b.lo[1] + b.hi[1]) / 2, b.hi[2] - 0.03); g.add(l); }
  const l = label(s.name, "#fff", 0.42); l.position.set(0, 0, 1.36); g.add(l);
}
{ const b = standBox(); Z.add(boxMesh(b.lo, b.hi, "#6b7280")); const l = label(STAND.name, "#d8dde6", 0.3); l.position.set(STAND.x, STAND.y, STAND.h + 0.2); Z.add(l); }
const fixtures = new THREE.Group(); Z.add(fixtures);           // 로봇마다 다른 고정물 (UR15 받침대)

// ── 로봇 ────────────────────────────────────────────────────
const R = await initPhysics();
const ROBOTS = [["ur15", "UR15 (고정 팔 · 직접 만든 집게)"], ["spot", "Spot (네 발 · 집게 팔)"]];
const WORLD = { ur15: Ur15Books, spot: SpotBooks };
const loaded = {}, loader = new STLLoader();
let world = null, robot = null, linkObj = {};
async function load(r) {
  if (loaded[r]) return loaded[r];
  const base = `assets/${r}/`, model = await (await fetch(base + "robot.json")).json();
  const shapes = new Shapes(await (await fetch(base + "shapes.json")).json());
  const groups = {};
  await Promise.all(Object.entries(model.links).map(async ([name, L]) => {
    const g = new THREE.Group(); g.matrixAutoUpdate = false; groups[name] = g;
    for (const v of L.visuals) {
      const geo = loader.parse(await (await fetch(base + v.mesh)).arrayBuffer()); geo.computeVertexNormals();
      const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: v.color || L.color, roughness: 0.5, metalness: 0.15 }));
      mesh.castShadow = true; mesh.matrixAutoUpdate = false; mesh.matrix.set(...rpyT(...v.rpy, v.xyz)); mesh.scale.set(...v.scale);
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
  fixtures.clear();
  if (r === "ur15") fixtures.add(boxMesh([-PEDESTAL.w / 2, -PEDESTAL.w / 2, 0], [PEDESTAL.w / 2, PEDESTAL.w / 2, PEDESTAL.h], "#4b5563"));
  robot = r; world = new WORLD[r](L.model, L.shapes, R, log);
  buildBooks(); rows().forEach(row => row.className = "");
  $("#robot").value = r; $("#robot").disabled = false; $("#status").textContent = "준비 — ▶ 실행을 누르세요";
  try { localStorage.setItem("books.robot", r); } catch {}
  log(`로봇: ${ROBOTS.find(x => x[0] === r)[1]}`);
  if (window.BOOKS) window.BOOKS.world = world;
}

// ── 책 ──────────────────────────────────────────────────────
const bookObj = new Map();
function buildBooks() {
  for (const o of bookObj.values()) Z.remove(o); bookObj.clear();
  for (const b of world.books) {
    const g = new THREE.Group(); g.matrixAutoUpdate = false;
    const m = new THREE.Mesh(new THREE.BoxGeometry(b.d, b.t, b.h), new THREE.MeshStandardMaterial({ color: b.color, roughness: 0.6 }));
    m.castShadow = m.receiveShadow = true; g.add(m);
    const spine = new THREE.Mesh(new THREE.BoxGeometry(0.002, b.t * 0.7, b.h * 0.25), new THREE.MeshBasicMaterial({ color: "#f5f0e1" }));
    spine.position.set(-b.d / 2 - 0.001, 0, b.h * 0.2); g.add(spine);       // 앞면(책등) 표시
    Z.add(g); bookObj.set(b, g);
  }
}
function draw() {
  const W = world.fk();
  for (const [n, g] of Object.entries(linkObj)) if (W[n]) setM(g, W[n]);
  for (const [b, g] of bookObj) setM(g, b.T);
  const P = world.pose();
  if ($("#follow").checked) { const tgt = new THREE.Vector3(P.x, 0.6, -P.y); camera.position.add(tgt.clone().sub(controls.target)); controls.target.copy(tgt); }
  $("#pose").textContent = `${world.held ? "든 책: " + world.held.name : "빈손"} · 바닥 책 ${world.floorBooks().length}권`;
}

// ── 블록 판 ─────────────────────────────────────────────────
let program = BOOK_EXAMPLE.map(b => ({ ...b, args: b.args.slice() }));
const OPTS = { cell: CELLS.map(c => [c, c]), count: COUNTS, sec: SECS };
function renderProgram() {
  const ol = $("#program"); ol.innerHTML = "";
  program.forEach((b, i) => {
    const li = document.createElement("li"), B = BOOK_BLOCKS[b.type];
    li.title = B.hint; li.className = B.args.length > 1 ? "multi" : "";
    li.innerHTML = `<span class="name">${B.name}</span>`;
    B.args.forEach((kind, j) => {
      if (j === 1 && b.type === "move") li.appendChild(Object.assign(document.createElement("span"), { className: "arrow", textContent: "→" }));
      const sel = document.createElement("select");
      for (const [v, t] of OPTS[kind]) sel.add(new Option(t, v, false, v === b.args[j]));
      if (b.args[j] == null) b.args[j] = sel.value;
      sel.onchange = () => { b.args[j] = sel.value; };
      li.appendChild(sel);
    });
    for (const [t, f] of [["↑", () => i > 0 && program.splice(i - 1, 0, program.splice(i, 1)[0])],
                          ["↓", () => i < program.length - 1 && program.splice(i + 1, 0, program.splice(i, 1)[0])],
                          ["✕", () => program.splice(i, 1)]]) {
      const btn = document.createElement("button"); btn.className = "mini"; btn.textContent = t;
      btn.onclick = () => { if (world.exec) return; f(); renderProgram(); }; li.appendChild(btn);
    }
    ol.appendChild(li);
  });
}
for (const [type, B] of Object.entries(BOOK_BLOCKS)) {
  const btn = document.createElement("button"); btn.textContent = "+ " + B.name; btn.title = B.hint;
  btn.onclick = () => { if (world.exec) return; program.push({ type, args: B.args.map(() => null) }); renderProgram(); };
  $("#palette").appendChild(btn);
}
$("#example").onclick = () => { if (world.exec) return; program = BOOK_EXAMPLE.map(b => ({ ...b, args: b.args.slice() })); renderProgram(); };
$("#clear").onclick = () => { if (world.exec) return; program = []; renderProgram(); };
const rows = () => [...document.querySelectorAll("#program li")];
$("#run").onclick = () => {
  if (world.exec || !program.length) return;
  rows().forEach(r => r.className = r.className.replace(/ ?(run|done|fail)/g, ""));
  log("실행 시작");
  world.run(program, {
    block(i) { rows().forEach((r, k) => { r.classList.toggle("done", k < i); r.classList.toggle("run", k === i); }); $("#status").textContent = `실행 중: ${i + 1}. ${BOOK_BLOCKS[program[i].type].name}`; },
    end(err, i) {
      if (err) { rows()[i]?.classList.add("fail"); log(err, "bad"); $("#status").textContent = "멈춤: " + err; }
      else { rows().forEach(r => { r.classList.remove("run"); r.classList.add("done"); }); log("프로그램 끝", "ok"); $("#status").textContent = "끝"; }
    },
  });
};
$("#stop").onclick = () => { if (world.exec) { world.stop(); log("멈춤"); $("#status").textContent = "멈춤"; } };
$("#reset").onclick = () => { world.stop(); world.reset(); buildBooks(); rows().forEach(r => r.className = r.className.replace(/ ?(run|done|fail)/g, "")); log("처음 자리로"); $("#status").textContent = "준비"; };
$("#robot").onchange = () => setRobot($("#robot").value);

// ── 돌리기 ──────────────────────────────────────────────────
function resize() { const r = view.getBoundingClientRect(); renderer.setSize(r.width, r.height, false); camera.aspect = r.width / r.height; camera.updateProjectionMatrix(); }
addEventListener("resize", resize); resize();
let last = performance.now();
function frame(now) {
  const dt = Math.min(0.05, (now - last) / 1000); last = now;
  if (world) { const n = +$("#speed").value; for (let i = 0; i < n * 2; i++) world.step(dt / 2); draw(); }
  controls.update(); renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
for (const [v, t] of ROBOTS) $("#robot").add(new Option(t, v));
let first = new URLSearchParams(location.search).get("robot");
try { first = first ?? localStorage.getItem("books.robot"); } catch {}
await setRobot(WORLD[first] ? first : "ur15");
renderProgram();
window.BOOKS = { world, run: () => $("#run").click(), camera, controls, follow: $("#follow"), setRobot, get program() { return program; } };   // 화면 검사용
requestAnimationFrame(frame);
