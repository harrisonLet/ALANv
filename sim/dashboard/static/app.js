// ALAN simulator dashboard: 3D view, controls, charts and consoles.
// World frame in three.js: x = east, -z = north, y = up, 1 unit = 1 metre.
// Boat frame: forward = -z, starboard = +x. A body angle d (positive to starboard) is rotation.y = -d.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { initStability, initPointsOfSail } from "/static/panels.js";
import { initBench } from "/static/bench.js";
import { initRoute } from "/static/route.js";
import * as X from "/static/scene_extras.js";

const $ = (id) => document.getElementById(id);
// ?no3d in the URL turns the 3D view off (for slow machines or headless testing)
const NO_3D = new URLSearchParams(location.search).has("no3d");
const rad = (d) => (d * Math.PI) / 180;
const wrap180 = (d) => ((((d + 180) % 360) + 360) % 360) - 180;

// ---------------------------------------------------------------------------------------------
// API helpers
// ---------------------------------------------------------------------------------------------
async function post(path, body = {}) {
  const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  return r.json();
}
const cmd = (text) => post("/api/cmd", { cmd: text });
function debounce(fn, ms = 120) {
  let t;
  return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); };
}

// ---------------------------------------------------------------------------------------------
// 3D scene
// ---------------------------------------------------------------------------------------------
const view = $("view");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(1.5, window.devicePixelRatio));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.0;
view.appendChild(renderer.domElement);

const scene = new THREE.Scene();
// soft studio reflections so metal, clearcoat and foam read correctly
scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.55;
scene.background = new THREE.Color(0x9cc9e8);
scene.fog = new THREE.Fog(0x9cc9e8, 60, 220);

const camera = new THREE.PerspectiveCamera(50, 1, 0.05, 1000);
camera.position.set(4, 3, 6);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x335577, 1.4));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(30, 50, 20);
scene.add(sun);

// Water: a large plane that follows the boat; the wave pattern uses world coordinates so it stays continuous
const WATER_SIZE = 240;
const waterGeo = new THREE.PlaneGeometry(WATER_SIZE, WATER_SIZE, 1, 1);   // flat: the ripple texture gives the detail
waterGeo.rotateX(-Math.PI / 2);
// Ripples: a water normal map scrolled in time on top of the wave shape (cheap, keeps the view fast)
const waterNormals = new THREE.TextureLoader().load(
  "https://cdn.jsdelivr.net/npm/three@0.170.0/examples/textures/waternormals.jpg",
  (t) => { t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(WATER_SIZE / 2.5, WATER_SIZE / 2.5); });   // 2.5 m tiles: the 10 m jumps of the plane stay seamless
const water = new THREE.Mesh(waterGeo, new THREE.MeshStandardMaterial({
  color: 0x1d5f8a, roughness: 0.18, metalness: 0.05, transparent: true, opacity: 0.94,
  normalMap: waterNormals, normalScale: new THREE.Vector2(0.55, 0.55),
}));
scene.add(water);
const grid = new THREE.GridHelper(WATER_SIZE, WATER_SIZE / 10, 0xffffff, 0xffffff);
grid.material.transparent = true;
grid.material.opacity = 0.12;
grid.position.y = 0.02;
scene.add(grid);
let waveAmp = 0.05;
function waveHeight(x, z, t) {
  return waveAmp * (Math.sin(0.55 * x + 1.3 * t) * 0.6 + Math.sin(0.4 * z - 1.1 * t) * 0.4);
}

// Boat (procedural; replaced part by part with STL files when sim/dashboard/models/boat.json exists)
const boat = new THREE.Group();
scene.add(boat);
const parts = {};
const mat = (c) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.6 });

{
  const s = new THREE.Shape();
  s.moveTo(0, 0.5);
  s.quadraticCurveTo(0.14, 0.25, 0.13, -0.2);
  s.lineTo(0.1, -0.5);
  s.lineTo(-0.1, -0.5);
  s.lineTo(-0.13, -0.2);
  s.quadraticCurveTo(-0.14, 0.25, 0, 0.5);
  const g = new THREE.ExtrudeGeometry(s, { depth: 0.1, bevelEnabled: true, bevelSize: 0.01, bevelThickness: 0.02, bevelSegments: 2 });
  g.rotateX(-Math.PI / 2);
  g.translate(0, -0.05, 0);
  parts.hull = new THREE.Mesh(g, mat(0xf8fafc));
  boat.add(parts.hull);

  const keel = new THREE.Group();
  const fin = new THREE.Mesh(new THREE.BoxGeometry(0.015, 0.3, 0.12), mat(0x334155));
  fin.position.y = -0.2;
  const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.035, 16, 12), mat(0x1e293b));
  bulb.scale.z = 2.2;
  bulb.position.y = -0.36;
  keel.add(fin, bulb);
  parts.keel = keel;
  boat.add(keel);
}

const rudderPivot = new THREE.Group();
rudderPivot.position.set(0, -0.02, 0.45);
boat.add(rudderPivot);
parts.rudder = new THREE.Mesh(new THREE.BoxGeometry(0.01, 0.18, 0.07), mat(0xdc2626));
parts.rudder.position.set(0, -0.09, 0.035);
rudderPivot.add(parts.rudder);

const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.01, 1.0, 10), mat(0x9ca3af));
mast.position.set(0, 0.55, -0.05);
boat.add(mast);
parts.mast = mast;

const wingPivot = new THREE.Group();
wingPivot.position.set(0, 0.1, -0.05);
boat.add(wingPivot);
parts.wing = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.85, 0.26), mat(0xfacc15));
parts.wing.position.set(0, 0.47, 0.065);
wingPivot.add(parts.wing);
const boom = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.008, 0.26), mat(0x9ca3af));
boom.position.set(0, 0.3, 0.32);
wingPivot.add(boom);
const tailPivot = new THREE.Group();
tailPivot.position.set(0, 0.3, 0.45);
wingPivot.add(tailPivot);
parts.tail = new THREE.Mesh(new THREE.BoxGeometry(0.01, 0.2, 0.08), mat(0xf97316));
parts.tail.position.set(0, 0, 0.04);
tailPivot.add(parts.tail);

// Wind vane at the masthead points into the apparent wind
const vanePivot = new THREE.Group();
vanePivot.position.set(0, 1.07, -0.05);
boat.add(vanePivot);
vanePivot.add(X.windVaneModel());

// Wind streaks (true wind): short lines that drift with the wind, longer when it blows harder.
// Current: dots drifting on the surface.
const WIND_N = 1500, WIND_BOX = 60, WIND_TOP = 8;
const windSeeds = new Float32Array(WIND_N * 3);
const windSeg = new Float32Array(WIND_N * 6);
for (let i = 0; i < WIND_N; i++) {
  windSeeds[i * 3] = (Math.random() - 0.5) * WIND_BOX;
  windSeeds[i * 3 + 1] = 0.15 + Math.pow(Math.random(), 1.6) * WIND_TOP;   // denser near the water
  windSeeds[i * 3 + 2] = (Math.random() - 0.5) * WIND_BOX;
}
const windGeo = new THREE.BufferGeometry();
windGeo.setAttribute("position", new THREE.BufferAttribute(windSeg, 3));
const windMat = new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.55 });
const windLines = new THREE.LineSegments(windGeo, windMat);
windLines.frustumCulled = false;
scene.add(windLines);
let windDensity = 1;      // fraction of streaks shown (UI slider)

const CUR_N = 400;
const curPts = new Float32Array(CUR_N * 3);
const curGeo = new THREE.BufferGeometry();
curGeo.setAttribute("position", new THREE.BufferAttribute(curPts, 3));
const curDots = new THREE.Points(curGeo, new THREE.PointsMaterial({ color: 0xbfe9ff, size: 0.07 }));
curDots.frustumCulled = false;
scene.add(curDots);
for (let i = 0; i < CUR_N; i++) {
  curPts[i * 3] = (Math.random() - 0.5) * 40;
  curPts[i * 3 + 1] = 0.05;
  curPts[i * 3 + 2] = (Math.random() - 0.5) * 40;
}

// Track over ground: a thick line that keeps the same on-screen width in every camera view
const TRACK_MAX = 3000;
let track = [];                       // flat [x, y, z, x, y, z, ...]
let trackDirty = false;
const trackMat = new LineMaterial({ color: 0xfde047, linewidth: 4, worldUnits: false, transparent: true, opacity: 0.95, depthTest: false });
let trackLine = null;
function rebuildTrack() {
  if (trackLine) { scene.remove(trackLine); trackLine.geometry.dispose(); trackLine = null; }
  if (track.length < 6) return;
  const g = new LineGeometry();
  g.setPositions(track);
  trackLine = new Line2(g, trackMat);
  trackLine.computeLineDistances();
  trackLine.frustumCulled = false;
  trackLine.renderOrder = 10;           // draw over the waves
  scene.add(trackLine);
}
function addTrack(x, z) {
  const n = track.length;
  if (n >= 3 && Math.hypot(track[n - 3] - x, track[n - 1] - z) < 0.25) return;
  track.push(x, 0.06, z);
  if (track.length > TRACK_MAX * 3) {
    // keep the shape but halve the resolution of the oldest half
    const keep = [];
    const half = Math.floor(track.length / 6) * 3;
    for (let i = 0; i < half; i += 6) keep.push(track[i], track[i + 1], track[i + 2]);
    track = keep.concat(track.slice(half));
  }
  trackDirty = true;
}
let lastTrackBuild = 0;
function maybeRebuildTrack(now) {
  if (trackDirty && now - lastTrackBuild > 1000) { trackDirty = false; lastTrackBuild = now; rebuildTrack(); }
}
function clearTrack() { track = []; rebuildTrack(); }
$("clear-track").onclick = clearTrack;

// CAD boat (sim/dashboard/models/boat.json, made by sim/tools/prepare_cad.py). Every STL is in the same assembly
// coordinates; boat.json gives the axes, the hinge points, and (from the server) the CG, waterline and trim.
// The boat's origin is the CG on the waterline, so it floats exactly as the hydrostatics computed.
const UNIT = { mm: 0.001, cm: 0.01, m: 1, in: 0.0254 };
const AXIS = { "+x": [1, 0, 0], "-x": [-1, 0, 0], "+y": [0, 1, 0], "-y": [0, -1, 0], "+z": [0, 0, 1], "-z": [0, 0, -1] };
let cadTrimDeg = 0;
const vaneMounts = {};          // mount name -> position in the boat frame
let vaneMountGroup = null;
function cadTransform(cfg) {
  const f = new THREE.Vector3(...AXIS[cfg.forward || "+x"]);
  const u = new THREE.Vector3(...AXIS[cfg.up || "+z"]);
  const st = new THREE.Vector3().crossVectors(f, u);
  const k = UNIT[cfg.units || "mm"];
  const cg = new THREE.Vector3(...cfg.cg);
  const origin = cg.clone();
  if (cfg.waterline != null) origin.addScaledVector(u, cfg.waterline - cg.dot(u));
  const R = new THREE.Matrix4().set(st.x, st.y, st.z, 0, u.x, u.y, u.z, 0, -f.x, -f.y, -f.z, 0, 0, 0, 0, 1);
  return new THREE.Matrix4().makeScale(k, k, k).multiply(R).multiply(new THREE.Matrix4().makeTranslation(-origin.x, -origin.y, -origin.z));
}
let deckAtMast = 0.14;
let stopParts = null;
function buildHardStops(limitDeg) {
  if (stopParts) { boat.remove(stopParts.blocks); wingPivot.remove(stopParts.arm); }
  stopParts = X.hardStops(Math.min(limitDeg, 179));
  stopParts.blocks.position.set(wingPivot.position.x, deckAtMast, wingPivot.position.z);
  stopParts.arm.position.set(0, deckAtMast - wingPivot.position.y, 0);
  boat.add(stopParts.blocks);
  wingPivot.add(stopParts.arm);
  stopParts.blocks.visible = stopParts.arm.visible = limitDeg < 180;
  stopParts.limit = limitDeg;
}
let forwardBoomCache = null;
function forwardBoomY() { return forwardBoomCache; }
const bench = { group: new THREE.Group(), fan: null, floor: null, on: false };
scene.add(bench.group);
bench.group.visible = false;
function buildBench(halfBeam, floorY, cradleY, stations) {
  bench.group.clear();
  bench.floorY = floorY;
  bench.stand = X.woodenStand(stations, halfBeam, floorY, cradleY);
  bench.floor = X.labFloor(floorY);
  bench.fan = X.boxFan();
  bench.group.add(bench.stand, bench.floor, bench.fan);
}
// Bench mode: the boat on its stand indoors, a fan instead of wind, no water
function setBenchScene(on) {
  bench.on = on;
  bench.group.visible = on;
  water.visible = grid.visible = curDots.visible = !on;
  if (trackLine) trackLine.visible = !on;
  scene.background = new THREE.Color(on ? 0xdfe3e8 : 0x9cc9e8);
  scene.fog = on ? new THREE.Fog(0xdfe3e8, 8, 30) : new THREE.Fog(0x9cc9e8, 60, 220);
  windDensity = on ? 0.35 : +($("streaks")?.value ?? 1);
}
async function loadModels() {
  const note = $("model-note");
  try {
    const cfg = await (await fetch("/api/boat")).json();
    if (!cfg.available) {
      note.textContent = "Placeholder boat. Run sim/tools/prepare_cad.py to load your CAD.";
      return;
    }
    cadTrimDeg = cfg.trim_deg || 0;
    const M = cadTransform(cfg);
    const toView = (p) => new THREE.Vector3(...p).applyMatrix4(M);
    const pv = {};
    for (const [n, p] of Object.entries(cfg.pivots || {})) pv[n] = toView(p);
    for (const n of ["wing", "rudder", "tail"]) if (cfg.parts?.[n]?.pivot) pv[n] = toView(cfg.parts[n].pivot);
    if (pv.wing) wingPivot.position.copy(pv.wing);
    if (pv.rudder) rudderPivot.position.copy(pv.rudder);
    const tailWorld = pv.tail || tailPivot.position.clone().add(wingPivot.position);
    tailPivot.position.copy(tailWorld.clone().sub(wingPivot.position));
    const hinge = { rudder: rudderPivot.position, wing: wingPivot.position, tail: tailWorld };
    const host = { boat, rudder: rudderPivot, wing: wingPivot, tail: tailPivot };
    for (const name of Object.keys(parts)) parts[name].visible = false;   // hide the placeholder boat
    boom.visible = false; mast.visible = false;
    const loader = new STLLoader();
    const loaded = [];
    const bounds = {};
    await Promise.all(Object.entries(cfg.parts || {}).map(async ([name, p]) => {
      if (!p.file) return;
      const geo = await loader.loadAsync(`/models/${p.file}`);
      geo.applyMatrix4(M);
      geo.computeVertexNormals();
      geo.computeBoundingBox();
      bounds[name] = geo.boundingBox.clone();
      if (name === "booms") {
        // height of the forward rod: average y of the vertices in the forward-most 30 mm
        const pos = geo.attributes.position; let sy = 0, n = 0;
        for (let i = 0; i < pos.count; i++) if (pos.getZ(i) < geo.boundingBox.min.z + 0.03) { sy += pos.getY(i); n++; }
        forwardBoomCache = n ? sy / n : null;
      }
      const mesh = new THREE.Mesh(geo, X.makeMaterial(p.material, p.color || "#e5e7eb", geo));
      const attach = p.attach || name;
      if (hinge[attach]) mesh.position.copy(hinge[attach]).negate();      // turns about its hinge
      (host[attach] || boat).add(mesh);
      loaded.push(name);
    }));
    // whip antenna on the top of the wing near the leading edge (photo)
    if (cfg.antenna) {
      const base = toView(cfg.antenna.base).sub(wingPivot.position);
      const ant = X.whipAntenna((cfg.antenna.length_mm || 300) * 0.001);
      ant.position.copy(base);
      wingPivot.add(ant);
    }
    const ex = cfg.extras || {};
    // counterweight at the forward end of the forward boom (it turns with the wing)
    if (ex.counterweight && bounds.booms) {
      const cw = X.counterweight(ex.counterweight);
      const b = bounds.booms;
      const lenM = (ex.counterweight.length_mm || 100) / 1000;
      const fwd = new THREE.Vector3((b.min.x + b.max.x) / 2, 0, b.min.z + lenM / 2);
      // the forward boom is the one reaching furthest forward: take its height from the boom mesh
      fwd.y = forwardBoomY(cfg) ?? (b.min.y + b.max.y) / 2;
      cw.position.copy(fwd).sub(wingPivot.position);
      wingPivot.add(cw);
    }
    if (ex.servo_box) {
      const sb = X.servoBox(ex.servo_box.size_mm.map((v) => v / 1000), ex.servo_box.color);
      sb.position.copy(toView(ex.servo_box.at_mm));
      boat.add(sb);
    }
    // hard stops: blocks on the deck at the mast base, an arm on the mast collar
    deckAtMast = bounds.deck ? bounds.deck.max.y : wingPivot.position.y;
    buildHardStops(S ? S.wing_limit : 90);
    // bench scene from the boat's real size: floor just under the keel, cradles at the hull sides
    const keelMin = Math.min(bounds.bulb?.min.y ?? 0, bounds.keel?.min.y ?? 0);
    const hb = bounds.hull;
    const stations = (ex.stand?.cradles_y_mm || [-260, 330]).map((y) => toView([0, y, 0]).z);
    buildBench(hb ? (hb.max.x - hb.min.x) / 2 : 0.155, keelMin - 0.09, hb ? hb.min.y + (hb.max.y - hb.min.y) * 0.62 : 0.05, stations);
    for (const [n, p] of Object.entries(cfg.vane_mounts || {})) vaneMounts[n] = toView(p);
    vaneMounts.masthead = new THREE.Vector3(wingPivot.position.x, (cfg.antenna ? toView(cfg.antenna.base).y : 1.2) + 0.06, wingPivot.position.z);
    placeVane(S ? S.vane_mount : "stern");
    note.textContent = `Your CAD: ${loaded.length} parts, floating at the computed waterline (trim ${Math.abs(cadTrimDeg).toFixed(1)}° ${cadTrimDeg < 0 ? "bow up" : "bow down"}).`;
  } catch (e) {
    note.textContent = `Could not load CAD models: ${e.message}`;
  }
}
// The wind vane on a short pole at its mount (bow / stern), or on top of the wing for "masthead"
function placeVane(mount) {
  const p = vaneMounts[mount];
  if (!p) return;
  if (!vaneMountGroup) {
    vaneMountGroup = new THREE.Group();
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, 0.16, 8), mat(0x111827));
    pole.position.y = -0.08;
    pole.name = "pole";
    vaneMountGroup.add(pole);
    boat.add(vaneMountGroup);
  }
  if (mount === "masthead") {
    if (vanePivot.parent !== wingPivot) wingPivot.add(vanePivot);
    vanePivot.position.copy(p).sub(wingPivot.position);
    vaneMountGroup.visible = false;
  } else {
    if (vanePivot.parent !== boat) boat.add(vanePivot);
    vanePivot.position.copy(p);
    vaneMountGroup.position.copy(p);
    vaneMountGroup.visible = true;
  }
  vanePivot.userData.mount = mount;
}
if (!NO_3D) loadModels();

// Cameras (scroll to zoom in Chase and Top)
let camMode = new URLSearchParams(location.search).get("cam") || "chase";   // ?cam=top opens in Top view
let chaseDist = 2.6, topHeight = 25;
renderer.domElement.addEventListener("wheel", (e) => {
  if (camMode === "free" || camMode === "bench") return;
  e.preventDefault();
  const k = Math.exp(e.deltaY * 0.0015);
  if (camMode === "chase") chaseDist = Math.min(40, Math.max(1.2, chaseDist * k));
  else topHeight = Math.min(300, Math.max(4, topHeight * k));
}, { passive: false });
document.querySelectorAll("[data-cam]").forEach((x) => x.classList.toggle("active", x.dataset.cam === camMode));
document.querySelectorAll("[data-cam]").forEach((b) => {
  b.onclick = () => {
    camMode = b.dataset.cam;
    document.querySelectorAll("[data-cam]").forEach((x) => x.classList.toggle("active", x === b));
  };
});

function resize() {
  const host = renderer.domElement.parentElement || view;
  const w = host.clientWidth, h = host.clientHeight;
  renderer.setSize(w, h);
  trackMat.resolution.set(w, h);
  camera.aspect = w / Math.max(1, h);
  camera.updateProjectionMatrix();
  for (const c of charts) c.resize();
}
window.addEventListener("resize", resize);

// ---------------------------------------------------------------------------------------------
// Live state
// ---------------------------------------------------------------------------------------------
let S = null;                 // latest snapshot from the server
const shown = { x: 0, z: 0, heading: 0, wing: 0, tail: 0, rudder: 0, awa: 0 };
let synced = false;

function lerpAngle(a, b, k) { return a + wrap180(b - a) * k; }

const clock = new THREE.Clock();
let frame = 0;
function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(0.1, clock.getDelta());
  const tNow = clock.elapsedTime;

  if (S) {
    const k = 1 - Math.exp(-dt * 12);
    const tx = S.east, tz = -S.north;
    if (Math.hypot(tx - shown.x, tz - shown.z) > 5) { shown.x = tx; shown.z = tz; }
    shown.x += (tx - shown.x) * k;
    shown.z += (tz - shown.z) * k;
    shown.heading = lerpAngle(shown.heading, S.heading, k);
    shown.wing = lerpAngle(shown.wing, S.wing, k);
    shown.tail += (S.tail - shown.tail) * k;
    shown.rudder += (S.rudder - shown.rudder) * k;
    shown.awa = lerpAngle(shown.awa, S.vane ?? S.awa, k);
    shown.heel = (shown.heel || 0) + ((S.heel || 0) - (shown.heel || 0)) * k;

    boat.position.set(shown.x, bench.on ? 0 : waveHeight(shown.x, shown.z, tNow), shown.z);
    if (bench.on && bench.fan) {
      // fan 1.8 m away on the side the wind comes from, at wing height, facing the boat
      const dist = 1.8;
      bench.group.position.set(shown.x, 0, shown.z);
      bench.group.rotation.set(0, -rad(shown.heading), 0);
      const rel = rad(S.wind_dir - shown.heading);
      bench.fan.position.set(Math.sin(rel) * dist, 0.55, -Math.cos(rel) * dist);
      bench.fan.lookAt(new THREE.Vector3(shown.x, 0.55, shown.z));
      bench.fan.userData.spin(dt, S.wind_speed);
    }
    if (stopParts && S.wing_limit !== stopParts.limit) buildHardStops(S.wing_limit);
    // heading, then trim (bow up/down), then heel (positive = starboard down); waves add a little roll
    const slope = bench.on ? 0 : (waveHeight(shown.x + 0.5, shown.z, tNow) - waveHeight(shown.x - 0.5, shown.z, tNow));
    boat.rotation.order = "YXZ";
    boat.rotation.set(-rad(cadTrimDeg), -rad(shown.heading), -rad(shown.heel) + slope * 0.6);
    if (S.vane_mount && vanePivot.userData.mount !== S.vane_mount) placeVane(S.vane_mount);
    rudderPivot.rotation.y = -rad(shown.rudder);
    wingPivot.rotation.y = -rad(shown.wing);
    tailPivot.rotation.y = -rad(shown.tail);
    // the vane points into the wind it measures; on the masthead it rides on the rotating wing
    vanePivot.rotation.y = -rad(vanePivot.parent === wingPivot ? shown.awa - shown.wing : shown.awa);
    addTrack(shown.x, shown.z);

    // Wind streaks move with the true wind; current dots with the water
    const wd = rad(S.wind_dir), ws = S.wind_speed;
    const wvx = -Math.sin(wd) * ws, wvz = Math.cos(wd) * ws;   // air moves toward wind_dir + 180
    const len = 0.25 + 0.18 * ws;                                 // streak length grows with wind speed
    const ux = ws > 0.01 ? wvx / ws : 0, uz = ws > 0.01 ? wvz / ws : 0;
    const shownN = Math.floor(WIND_N * windDensity);
    const hw = WIND_BOX / 2;
    for (let i = 0; i < WIND_N; i++) {
      let x = windSeeds[i * 3] + wvx * dt, z = windSeeds[i * 3 + 2] + wvz * dt;
      if (x - shown.x > hw) x -= WIND_BOX; if (x - shown.x < -hw) x += WIND_BOX;
      if (z - shown.z > hw) z -= WIND_BOX; if (z - shown.z < -hw) z += WIND_BOX;
      windSeeds[i * 3] = x; windSeeds[i * 3 + 2] = z;
      const y = windSeeds[i * 3 + 1], o = i * 6;
      if (i < shownN) {
        windSeg[o] = x; windSeg[o + 1] = y; windSeg[o + 2] = z;
        windSeg[o + 3] = x - ux * len; windSeg[o + 4] = y; windSeg[o + 5] = z - uz * len;
      } else {
        windSeg[o] = windSeg[o + 3] = x; windSeg[o + 1] = windSeg[o + 4] = -50; windSeg[o + 2] = windSeg[o + 5] = z;
      }
    }
    windGeo.attributes.position.needsUpdate = true;
    windMat.opacity = Math.min(0.75, 0.25 + ws * 0.06);
    const cd = rad(S.current_dir), cs = S.current_speed;
    const cvx = Math.sin(cd) * cs, cvz = -Math.cos(cd) * cs;
    for (let i = 0; i < CUR_N; i++) {
      let x = curPts[i * 3] + cvx * dt, z = curPts[i * 3 + 2] + cvz * dt;
      if (x - shown.x > 20) x -= 40; if (x - shown.x < -20) x += 40;
      if (z - shown.z > 20) z -= 40; if (z - shown.z < -20) z += 40;
      curPts[i * 3] = x; curPts[i * 3 + 1] = 0.04 + waveHeight(x, z, tNow); curPts[i * 3 + 2] = z;
    }
    curGeo.attributes.position.needsUpdate = true;
  }

  // ripples drift with the wind; stronger wind = choppier water
  if (waterNormals.image && S) {
    const wd = rad(S.wind_dir), k = 0.0009 * (0.4 + S.wind_speed);
    waterNormals.offset.x -= Math.sin(wd) * k * dt * 60;
    waterNormals.offset.y += Math.cos(wd) * k * dt * 60;
    const chop = Math.min(1.1, 0.3 + S.wind_speed * 0.09);
    water.material.normalScale.set(chop, chop);
  }

  // Water follows the boat in 10 m steps
  const cx = Math.round(shown.x / 10) * 10, cz = Math.round(shown.z / 10) * 10;
  water.position.set(cx, 0, cz);
  grid.position.set(cx, 0.02, cz);
  frame++;

  maybeRebuildTrack(performance.now());

  // Camera (Top view: north up, east right)
  camera.up.set(0, camMode === "top" ? 0 : 1, camMode === "top" ? -1 : 0);
  const target = new THREE.Vector3(shown.x, 0.4, shown.z);
  if (camMode === "chase") {
    const h = rad(shown.heading);
    const behind = new THREE.Vector3(shown.x - Math.sin(h) * chaseDist, 0.45 * chaseDist, shown.z + Math.cos(h) * chaseDist);
    camera.position.lerp(behind, 1 - Math.exp(-dt * 4));
    controls.target.copy(target);
    camera.lookAt(target);
  } else if (camMode === "bench") {
    if (!bench.camInit) {
      camera.position.set(shown.x + 1.6, 0.9, shown.z + 1.4);
      bench.camInit = true;
    }
    const delta = new THREE.Vector3(shown.x, 0.35, shown.z).sub(controls.target);
    controls.target.add(delta);
    camera.position.add(delta);
    controls.update();
  } else if (camMode === "top") {
    camera.position.lerp(new THREE.Vector3(shown.x, topHeight, shown.z), 1 - Math.exp(-dt * 4));
    controls.target.copy(target);
    camera.lookAt(target);
  } else {
    const delta = target.clone().sub(controls.target);
    controls.target.add(delta);
    camera.position.add(delta);
    controls.update();
  }
  renderer.render(scene, camera);
}

// ---------------------------------------------------------------------------------------------
// Charts
// ---------------------------------------------------------------------------------------------
class StripChart {
  constructor(canvas, series, opts = {}) {
    this.c = canvas; this.ctx = canvas.getContext("2d");
    this.series = series; this.opts = opts; this.data = []; this.window = opts.window || 60;
  }
  resize() {
    const r = this.c.getBoundingClientRect();
    this.c.width = r.width * devicePixelRatio; this.c.height = r.height * devicePixelRatio;
  }
  push(t, values) {
    if (this.data.length && t < this.data[this.data.length - 1][0]) this.data = [];
    this.data.push([t, values]);
    while (this.data.length && this.data[0][0] < t - this.window) this.data.shift();
  }
  draw() {
    const r = this.c.getBoundingClientRect();
    if (Math.abs(r.width * devicePixelRatio - this.c.width) > 1 || Math.abs(r.height * devicePixelRatio - this.c.height) > 1) this.resize();
    const { ctx, c } = this; const W = c.width, H = c.height, d = devicePixelRatio;
    ctx.clearRect(0, 0, W, H);
    if (this.data.length < 2) return;
    const t1 = this.data[this.data.length - 1][0];
    const t0 = Math.max(t1 - this.window, this.data[0][0]);
    const span = Math.max(1, t1 - t0);
    let lo = Infinity, hi = -Infinity;
    for (const [, v] of this.data) for (const x of v) if (x != null) { lo = Math.min(lo, x); hi = Math.max(hi, x); }
    if (this.opts.min != null) lo = Math.min(lo, this.opts.min);
    if (this.opts.max != null) hi = Math.max(hi, this.opts.max);
    if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
    const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
    const L = 38 * d, R = W - 8 * d, T = 16 * d, B = H - 6 * d;
    const X = (t) => L + ((t - t0) / span) * (R - L);
    const Y = (v) => B - ((v - lo) / (hi - lo)) * (B - T);
    ctx.strokeStyle = "#334155"; ctx.lineWidth = 1; ctx.font = `${10 * d}px system-ui`; ctx.fillStyle = "#94a3b8";
    for (let i = 0; i <= 3; i++) {
      const v = lo + ((hi - lo) * i) / 3; const y = Y(v);
      ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(R, y); ctx.stroke();
      ctx.fillText(v.toFixed(Math.abs(hi - lo) < 5 ? 1 : 0), 2 * d, y + 3 * d);
    }
    let lx = L;
    this.series.forEach((s, si) => {
      ctx.strokeStyle = s.color; ctx.lineWidth = 1.5 * d; ctx.beginPath();
      let started = false;
      for (const [t, v] of this.data) {
        if (v[si] == null) { started = false; continue; }
        const x = X(t), y = Y(v[si]);
        if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.fillStyle = s.color; ctx.fillText(s.label, lx, 11 * d); lx += ctx.measureText(s.label).width + 14 * d;
    });
  }
}
const charts = [
  new StripChart($("c-heading"), [{ label: "heading °", color: "#e5e7eb" }, { label: "course over ground °", color: "#38bdf8" }]),
  new StripChart($("c-servos"), [{ label: "rudder °", color: "#a78bfa" }, { label: "tail °", color: "#f97316" }, { label: "wing AoA °", color: "#facc15" }]),
  new StripChart($("c-speed"), [{ label: "boat speed m/s", color: "#38bdf8" }, { label: "wind m/s", color: "#22c55e" }], { min: 0 }),
  new StripChart($("c-vane"), [{ label: "true apparent wind °", color: "#e5e7eb" }, { label: "wind vane reads °", color: "#22c55e" }]),
];
let lastChartT = -1;


// ---------------------------------------------------------------------------------------------
// Consoles
// ---------------------------------------------------------------------------------------------
const consoles = { fw: { el: $("out-fw"), lines: [] }, sys: { el: $("out-sys"), lines: [] } };
let activeTab = "fw";
function appendLines(which, lines) {
  if (!lines.length) return;
  const c = consoles[which];
  c.lines.push(...lines);
  if (c.lines.length > 600) c.lines.splice(0, c.lines.length - 600);
  c.dirty = true;
}
function renderConsole(which) {
  const c = consoles[which];
  if (!c.dirty) return;
  c.dirty = false;
  const f = $("filter").value.toLowerCase();
  const lines = f ? c.lines.filter((l) => l.toLowerCase().includes(f)) : c.lines;
  if (c.el.offsetParent === null) { c.dirty = true; return; }      // not on screen: render when it is
  c.el.textContent = lines.slice(-400).join("\n");
  if ($("autoscroll").checked) c.el.scrollTop = c.el.scrollHeight;
}
$("filter").oninput = () => { consoles.fw.dirty = consoles.sys.dirty = true; renderConsole(activeTab); };
document.querySelectorAll("[data-tab]").forEach((b) => {
  b.onclick = () => {
    activeTab = b.dataset.tab;
    document.querySelectorAll("[data-tab]").forEach((x) => x.classList.toggle("active", x === b));
    $("out-fw").hidden = activeTab !== "fw"; $("out-sys").hidden = activeTab !== "sys";
    consoles[activeTab].dirty = true;
    renderConsole(activeTab);
  };
});

// ---------------------------------------------------------------------------------------------
// Header, HUD, telemetry
// ---------------------------------------------------------------------------------------------
function fmt(v, d = 1, unit = "") { return v == null ? "–" : `${Number(v).toFixed(d)}${unit}`; }
function updatePanels(s) {
  const mode = $("mode");
  mode.textContent = s.mode === "firmware" ? "FIRMWARE IN THE LOOP" : "FIRMWARE OFF · sliders steer";
  mode.classList.toggle("firmware", s.mode === "firmware");
  $("h-time").textContent = s.t.toFixed(1);
  $("h-renode").textContent = { stopped: "not running", building: "compiling…", starting: "starting…",
                                running: s.mode === "firmware" ? "running" : "booting…" }[s.renode] || s.renode;
  const fwBtn = $("h-fw");
  const busy = s.renode === "building" || s.renode === "starting";
  fwBtn.disabled = busy;
  fwBtn.textContent = s.renode === "stopped" ? "▶ Run my firmware" : busy ? "Starting…" : "■ Stop firmware";
  fwBtn.dataset.running = s.renode === "stopped" ? "" : "1";
  $("h-log").textContent = s.log;
  $("pause").textContent = s.paused ? "Resume" : "Pause";

  $("hud").innerHTML = [
    ["heading", fmt(s.heading, 0, "°")], ["speed", fmt(s.sog, 2, " m/s")],
    ["apparent wind", `${fmt(s.awa, 0, "°")} @ ${fmt(s.aws, 1)}`], ["wing AoA", fmt(s.aoa, 1, "°")],
    ["tail / rudder", `${fmt(s.tail, 0, "°")} / ${fmt(s.rudder, 0, "°")}`], ["heel", fmt(s.heel, 0, "°")],
  ].map(([k, v]) => `<div><span>${k}</span><b>${v}</b></div>`).join("");

  const fwPulse = (u) => (u == null ? "–" : u > 2600 || u < 400 ? `${u} (no pulse)` : `${u} µs`);
  $("telemetry").innerHTML = [
    ["time", fmt(s.t, 1, " s")], ["position", `${s.lat.toFixed(6)}, ${s.lon.toFixed(6)}`],
    ["north / east", `${fmt(s.north, 1)} / ${fmt(s.east, 1)} m`], ["heading (true)", fmt(s.heading, 1, "°")],
    ["course over ground", fmt(s.cog, 1, "°")], ["speed over ground", fmt(s.sog, 2, " m/s")],
    ["speed through water", fmt(s.stw, 2, " m/s")], ["yaw rate", fmt(s.yaw_rate, 1, " °/s")],
    ["true wind", `${fmt(s.wind_dir, 0, "°")} @ ${fmt(s.wind_speed, 1, " m/s")}`],
    ["apparent wind", `${fmt(s.awa, 0, "°")} @ ${fmt(s.aws, 1, " m/s")}`],
    ["wing angle", fmt(s.wing, 1, "°")], ["wing AoA", fmt(s.aoa, 1, "°")],
    ["tail (cmd)", `${fmt(s.tail, 1, "°")} (${fmt(s.tail_cmd, 1, "°")})`],
    ["rudder (cmd)", `${fmt(s.rudder, 1, "°")} (${fmt(s.rudder_cmd, 1, "°")})`],
    ["firmware tail PWM", fwPulse(s.fw_tail_us)], ["firmware rudder PWM", fwPulse(s.fw_rudder_us)],
    ["heel", fmt(s.heel, 1, "°")],
    [`wind vane (${s.vane_mount})`, `${fmt(s.vane, 0, "°")} (error ${fmt(wrap180((s.vane ?? 0) - (s.awa ?? 0)), 1, "°")})`],
    ["current", `${fmt(s.current_dir, 0, "°")} @ ${fmt(s.current_speed, 2, " m/s")}`],
    ["scenario", s.scenario],
  ].map(([k, v]) => `<span>${k}</span><b>${v}</b>`).join("");

  updateWho(s);
  setToggle($("gps-toggle"), s.gps_ok, "GPS");
  setToggle($("vane-toggle"), s.vane_ok, "Wind vane");
  $("override").checked = s.override;

  if (!synced) {
    setSlider("wind-dir", s.wind_mean_dir); setSlider("wind-speed", s.wind_mean_speed);
    setSlider("gust", s.gust); setSlider("shift", s.shift);
    setSlider("cur-dir", s.current_dir); setSlider("cur-speed", s.current_speed);
    $("speed").value = String(s.speed);
    synced = true;
  }
}
function updateWho(s) {
  const who = $("who");
  const fw = s.firmware;
  who.classList.toggle("firmware", s.mode === "firmware" && !s.override);
  who.classList.toggle("override", s.mode === "firmware" && s.override);
  if (s.mode === "firmware" && fw) {
    const code = `<b>${fw.branch} @ ${fw.commit}</b>${fw.dirty ? " + your uncommitted edits" : ""}`;
    who.innerHTML = s.override
      ? `Your firmware (${code}) is running, but you have overridden the servos: the sliders are steering.`
      : `Your firmware (${code}) is sailing the boat: the servos follow the PWM your code writes, and its sensors read this virtual boat.`;
  } else if (s.renode === "building") {
    who.innerHTML = "Compiling your CM7 firmware…";
  } else if (s.renode === "running" || s.renode === "starting") {
    who.innerHTML = "Renode is starting your firmware; the boat switches to it as soon as it begins sending servo commands.";
  } else {
    who.innerHTML = "No firmware is running. You are steering with the Manual servos sliders. Click <b>Build &amp; start</b> to sail it with your code.";
  }
  const card = $("fw-card");
  if (fw) {
    card.innerHTML = `<div>Loaded: <b>${fw.branch} @ ${fw.commit}</b> <span class="hint">“${fw.subject || ""}”</span></div>` +
      `<div>Built ${fw.built} · sensors: ${fw.sensors}</div>` +
      (fw.dirty ? `<div class="dirty">Includes uncommitted edits: ${fw.changed.join(", ")}</div>` : "<div>No uncommitted edits: exactly what is in git.</div>") +
      `<div class="hint">${fw.elf}</div>`;
  } else {
    card.innerHTML = '<div class="hint">Nothing loaded yet. Build &amp; start compiles the code in CM7/ and runs it here.</div>';
  }
  document.querySelectorAll("[data-vane]").forEach((b) => b.classList.toggle("active", b.dataset.vane === s.vane_mount));
}
function setToggle(btn, on, label) {
  btn.classList.toggle("on", on); btn.classList.toggle("off", !on);
  btn.textContent = `${label} ${on ? "on" : "OFF"}`;
}

// ---------------------------------------------------------------------------------------------
// Controls
// ---------------------------------------------------------------------------------------------
const units = {
  "wind-dir": (v) => `${v}°`, "wind-speed": (v) => `${(+v).toFixed(1)} m/s`, gust: (v) => `${(+v).toFixed(1)} m/s`,
  shift: (v) => `${v}°`, "cur-dir": (v) => `${v}°`, "cur-speed": (v) => `${(+v).toFixed(2)} m/s`,
  waves: (v) => `${(+v).toFixed(2)} m`, tail: (v) => `${v}°`, rudder: (v) => `${v}°`,
  streaks: (v) => `${Math.round(v * 100)}%`, "track-width": (v) => `${v} px`,
};
function setSlider(id, v) { $(id).value = v; $(`${id}-v`).textContent = units[id]($(id).value); }
for (const id of Object.keys(units)) {
  $(id).addEventListener("input", () => { $(`${id}-v`).textContent = units[id]($(id).value); });
}
const sendWind = debounce(() => cmd(`wind ${$("wind-dir").value} ${$("wind-speed").value}`));
$("wind-dir").addEventListener("input", sendWind);
$("wind-speed").addEventListener("input", sendWind);
$("gust").addEventListener("input", debounce(() => cmd(`gust ${$("gust").value}`)));
$("shift").addEventListener("input", debounce(() => cmd(`shift ${$("shift").value}`)));
const sendCurrent = debounce(() => cmd(`current ${$("cur-dir").value} ${$("cur-speed").value}`));
$("cur-dir").addEventListener("input", sendCurrent);
$("cur-speed").addEventListener("input", sendCurrent);
$("waves").addEventListener("input", () => { waveAmp = +$("waves").value; });
$("streaks").addEventListener("input", () => { windDensity = +$("streaks").value; });
$("track-width").addEventListener("input", () => { trackMat.linewidth = +$("track-width").value; });

const sendServos = debounce(() => {
  if (S && S.mode === "firmware" && !S.override) cmd("override on");   // moving a slider takes the servos over
  cmd(`servo ${$("tail").value} ${$("rudder").value}`);
}, 60);
$("tail").addEventListener("input", sendServos);
$("rudder").addEventListener("input", sendServos);
$("center").onclick = () => { setSlider("tail", 0); setSlider("rudder", 0); sendServos(); };
$("override").onchange = () => { cmd(`override ${$("override").checked ? "on" : "off"}`); if ($("override").checked) sendServos(); };

document.querySelectorAll("[data-vane]").forEach((b) => { b.onclick = () => cmd(`vanemount ${b.dataset.vane}`); });

// View tabs: Live / Bench / Stability / Points of sail / How it works
const stability = initStability(post);
const pointsOfSail = initPointsOfSail();
let camBefore = "chase";
const benchTab = initBench({
  cmd, post,
  consoleLines: () => consoles.fw.lines,
  enter(host) {
    host.appendChild(renderer.domElement);
    setBenchScene(true);
    camBefore = camMode;
    camMode = "bench";
    resize();
  },
  leave() {
    view.insertBefore(renderer.domElement, view.firstChild);
    setBenchScene(false);
    camMode = camBefore;
    // the server puts the outdoor wind back on "bench off"; re-read the sliders from it
    synced = false;
    resize();
  },
});
const routeTab = initRoute({ cmd });
document.querySelectorAll("[data-view]").forEach((b) => {
  b.onclick = () => {
    document.querySelectorAll("[data-view]").forEach((x) => x.classList.toggle("active", x === b));
    for (const v of ["live", "bench", "route", "stability", "pos", "help"]) $(`view-${v}`).hidden = v !== b.dataset.view;
    if (b.dataset.view !== "bench") benchTab.hide();
    if (b.dataset.view === "route") routeTab.show(); else routeTab.hide();
    if (b.dataset.view === "bench") benchTab.show();
    if (b.dataset.view === "live") resize();
    if (b.dataset.view === "stability") stability.show();
    if (b.dataset.view === "pos") pointsOfSail.show();
  };
});
const startView = new URLSearchParams(location.search).get("view");
if (startView) document.querySelector(`[data-view="${startView}"]`)?.click();

$("gps-toggle").onclick = () => cmd(`gps ${S && S.gps_ok ? "off" : "on"}`);
$("vane-toggle").onclick = () => cmd(`vane ${S && S.vane_ok ? "off" : "on"}`);
$("stop-boat").onclick = () => cmd("stop");
$("apply-heading").onclick = () => { const h = $("set-heading").value; if (h !== "") cmd(`heading ${h}`); };

$("pause").onclick = () => post("/api/pause", {});
$("speed").onchange = () => post("/api/speed", { speed: +$("speed").value });

const sensors = () => [...document.querySelectorAll(".sensor:checked")].map((c) => c.value).join(",") || "none";
$("h-fw").onclick = () => (
  $("h-fw").dataset.running ? post("/api/renode/stop") : post("/api/renode/start", { sensors: sensors(), build: true }));
$("fw-start").onclick = () => post("/api/renode/start", { sensors: sensors(), build: true });
$("fw-restart").onclick = () => post("/api/renode/start", { sensors: sensors(), build: false });
$("fw-stop").onclick = () => post("/api/renode/stop");
const sendMonitor = () => { const v = $("monitor").value.trim(); if (v) post("/api/renode/monitor", { cmd: v }); $("monitor").value = ""; };
$("monitor-send").onclick = sendMonitor;
$("monitor").addEventListener("keydown", (e) => { if (e.key === "Enter") sendMonitor(); });

async function loadScenarios() {
  const list = await (await fetch("/api/scenarios")).json();
  const sel = $("scenario");
  sel.innerHTML = list.map((s) => `<option value="${s.file}">${s.file.replace(".json", "")}</option>`).join("");
  const desc = () => { $("scenario-desc").textContent = (list.find((s) => s.file === sel.value) || {}).description || ""; };
  sel.onchange = desc;
  if (S) sel.value = S.scenario;
  desc();
}
$("reset").onclick = async () => {
  await post("/api/reset", { scenario: $("scenario").value });
  clearTrack(); synced = false;
  charts.forEach((c) => (c.data = []));
};

// ---------------------------------------------------------------------------------------------
// Stream
// ---------------------------------------------------------------------------------------------
let lastDraw = 0;
function connect() {
  const es = new EventSource("/api/stream");
  es.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    const first = !S;
    S = msg.state;
    if (first) {
      shown.x = S.east; shown.z = -S.north; shown.heading = S.heading;
      const h = rad(S.heading);
      camera.position.set(shown.x - Math.sin(h) * 2.6, 1.2, shown.z + Math.cos(h) * 2.6);
      loadScenarios();
    }
    updatePanels(S);
    benchTab.update(S);
    routeTab.update(S);
    if (S.t !== lastChartT) {
      lastChartT = S.t;
      charts[0].push(S.t, [S.heading, S.cog]);
      charts[1].push(S.t, [S.rudder, S.tail, S.aoa]);
      charts[2].push(S.t, [S.sog, S.wind_speed]);
      charts[3].push(S.t, [S.awa, S.vane]);
    }
    appendLines("fw", msg.console);
    appendLines("sys", msg.log);
    // Redraw from the data stream itself (about 10 Hz) so background tabs and slow machines stay consistent
    const now = performance.now();
    if (now - lastDraw > 200) {
      lastDraw = now;
      charts.forEach((c) => c.draw());
      renderConsole(activeTab);
    }
  };
  es.onerror = () => { $("mode").textContent = "DISCONNECTED"; };
}

resize();
connect();
if (NO_3D) {
  view.querySelector("canvas").style.display = "none";
  $("model-note").textContent = "3D view off (?no3d)";
} else {
  animate();
}
