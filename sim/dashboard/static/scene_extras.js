// Realism for the ALAN 3D view: surface materials, details from the photos that are not in the CAD,
// and the bench scene (wooden stand, lab floor, fan).
import * as THREE from "three";

// ---------------------------------------------------------------------------------------------
// Procedural textures (drawn once on a canvas; no image files needed)
// ---------------------------------------------------------------------------------------------
function canvasTex(w, h, draw, repeat = true) {
  const c = document.createElement("canvas");
  c.width = w; c.height = h;
  draw(c.getContext("2d"), w, h);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  if (repeat) t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 4;
  return t;
}
function speckle(ctx, w, h, base, n, spread) {
  ctx.fillStyle = base; ctx.fillRect(0, 0, w, h);
  const c = new THREE.Color(base);
  for (let i = 0; i < n; i++) {
    const k = (Math.random() - 0.5) * spread;
    ctx.fillStyle = `rgb(${Math.round(255 * Math.min(1, Math.max(0, c.r + k)))},${Math.round(255 * Math.min(1, Math.max(0, c.g + k)))},${Math.round(255 * Math.min(1, Math.max(0, c.b + k)))})`;
    const r = Math.random() * 1.8 + 0.4;
    ctx.beginPath(); ctx.arc(Math.random() * w, Math.random() * h, r, 0, Math.PI * 2); ctx.fill();
  }
}
const TEX = {};
function foamTex(color) {
  return (TEX["foam" + color] ??= canvasTex(256, 256, (ctx, w, h) => speckle(ctx, w, h, color, 5000, 0.16)));
}
function deckTex(color) {
  // pink foam board with the printed "FOAM" lettering and small print, as on the real deck
  return (TEX["deck" + color] ??= canvasTex(1024, 320, (ctx, w, h) => {
    speckle(ctx, w, h, color, 9000, 0.12);
    ctx.save();
    ctx.translate(w * 0.56, h * 0.52);
    ctx.fillStyle = "rgba(40,30,40,.62)";
    ctx.font = "900 150px Arial Black, Arial, sans-serif";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText("FOAM", 0, 0);
    ctx.font = "600 22px Arial, sans-serif";
    for (let i = 0; i < 6; i++) ctx.fillText("INSULATION BOARD · R-5 · KEEP AWAY FROM FLAME ·", -w * 0.38 + (i % 2) * 40, -h * 0.4 + i * 26 + (i > 2 ? 150 : 0));
    ctx.restore();
  }, false));
}
function carbonTex() {
  return (TEX.carbon ??= canvasTex(64, 64, (ctx, w, h) => {
    ctx.fillStyle = "#16161a"; ctx.fillRect(0, 0, w, h);
    for (let y = 0; y < h; y += 8) for (let x = 0; x < w; x += 8) {
      ctx.fillStyle = ((x + y) / 8) % 2 ? "#26262c" : "#1c1c21";
      ctx.fillRect(x, y, 8, 8);
    }
  }));
}
function woodTex() {
  return (TEX.wood ??= canvasTex(256, 64, (ctx, w, h) => {
    ctx.fillStyle = "#c9a26b"; ctx.fillRect(0, 0, w, h);
    for (let i = 0; i < 40; i++) {
      ctx.strokeStyle = `rgba(120,80,40,${0.08 + Math.random() * 0.18})`;
      ctx.lineWidth = 0.5 + Math.random() * 1.5;
      const y = Math.random() * h;
      ctx.beginPath(); ctx.moveTo(0, y);
      for (let x = 0; x <= w; x += 16) ctx.lineTo(x, y + Math.sin(x / 30 + i) * 2.5);
      ctx.stroke();
    }
  }));
}
function floorTex() {
  // grey vinyl tiles with darker chevrons, like the lab floor in the photos
  return (TEX.floor ??= canvasTex(512, 512, (ctx, w, h) => {
    speckle(ctx, w, h, "#b9bcc0", 6000, 0.08);
    ctx.fillStyle = "rgba(90,95,100,.35)";
    ctx.beginPath(); ctx.moveTo(0, h); ctx.lineTo(w * 0.5, h * 0.5); ctx.lineTo(w, h); ctx.fill();   // grey chevron like the lab floor
    ctx.strokeStyle = "rgba(60,60,60,.25)"; ctx.lineWidth = 2; ctx.strokeRect(0, 0, w, h);
  }));
}

// Texture coordinates for STL meshes (they have none): project each face on its dominant axis
export function boxUV(geo, scale = 1) {
  const p = geo.attributes.position, n = geo.attributes.normal;
  const uv = new Float32Array(p.count * 2);
  for (let i = 0; i < p.count; i++) {
    const ax = Math.abs(n.getX(i)), ay = Math.abs(n.getY(i)), az = Math.abs(n.getZ(i));
    let u, v;
    if (ax >= ay && ax >= az) { u = p.getZ(i); v = p.getY(i); } else if (ay >= az) { u = p.getX(i); v = p.getZ(i); } else { u = p.getX(i); v = p.getY(i); }
    uv[i * 2] = u * scale; uv[i * 2 + 1] = v * scale;
  }
  geo.setAttribute("uv", new THREE.BufferAttribute(uv, 2));
}
// Top-down projection stretched over the part (for the deck lettering)
function topUV(geo) {
  geo.computeBoundingBox();
  const b = geo.boundingBox, p = geo.attributes.position;
  const uv = new Float32Array(p.count * 2);
  for (let i = 0; i < p.count; i++) {
    uv[i * 2] = (p.getZ(i) - b.min.z) / (b.max.z - b.min.z);          // along the hull: the lettering reads bow to stern
    uv[i * 2 + 1] = (p.getX(i) - b.min.x) / (b.max.x - b.min.x);
  }
  geo.setAttribute("uv", new THREE.BufferAttribute(uv, 2));
}

// ---------------------------------------------------------------------------------------------
// Materials by name (boat.json "material")
// ---------------------------------------------------------------------------------------------
export function makeMaterial(kind, color, geo) {
  switch (kind) {
    case "foam":
      boxUV(geo, 4);
      return new THREE.MeshStandardMaterial({ color: "#ffffff", map: foamTex(color), roughness: 1, metalness: 0 });
    case "foam-deck":
      topUV(geo);
      return new THREE.MeshStandardMaterial({ color: "#ffffff", map: deckTex(color), roughness: 0.95 });
    case "fiberglass":
      return new THREE.MeshPhysicalMaterial({ color, roughness: 0.42, clearcoat: 0.5, clearcoatRoughness: 0.35, sheen: 0.2 });
    case "carbon":
      boxUV(geo, 40);
      return new THREE.MeshStandardMaterial({ color: "#ffffff", map: carbonTex(), roughness: 0.35, metalness: 0.25 });
    case "metal":
      return new THREE.MeshStandardMaterial({ color, roughness: 0.28, metalness: 0.9 });
    default:
      return new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.02 });
  }
}

// ---------------------------------------------------------------------------------------------
// Details from the photos
// ---------------------------------------------------------------------------------------------
const plain = (c, r = 0.55, m = 0) => new THREE.MeshStandardMaterial({ color: c, roughness: r, metalness: m });

// Counterweight: grey cylinder on the forward carbon rod
export function counterweight(spec) {
  const g = new THREE.Group();
  const r = (spec.diameter_mm || 95) / 2000, len = (spec.length_mm || 100) / 1000;
  const body = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 40), plain(spec.color || "#55585e", 0.45, 0.35));
  body.rotation.x = Math.PI / 2;
  const ringMat = plain("#3a3d42", 0.4, 0.4);
  for (const s of [-1, 1]) {
    const ring = new THREE.Mesh(new THREE.TorusGeometry(r * 0.98, 0.003, 8, 40), ringMat);
    ring.position.z = (s * len) / 2;
    g.add(ring);
  }
  g.add(body);
  return g;
}

// Whip antenna: thin black tapered rod with a thicker base
export function whipAntenna(lengthM) {
  const g = new THREE.Group();
  const mat = plain("#0b0b0c", 0.5);
  const base = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.009, 0.035, 12), mat);
  base.position.y = 0.0175;
  const whip = new THREE.Mesh(new THREE.CylinderGeometry(0.0018, 0.0045, lengthM, 8), mat);
  whip.position.y = 0.035 + lengthM / 2;
  g.add(base, whip);
  return g;
}

// RS485 wind vane (DFRobot SEN0482 look): black body, arm with a tail fin, small nose weight.
// Built pointing its nose along -z (into the wind); the caller rotates it.
export function windVaneModel() {
  const g = new THREE.Group();
  const mat = plain("#111214", 0.5);
  const body = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.024, 0.055, 24), mat);
  body.position.y = -0.03;
  const cap = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.02, 0.012, 20), mat);
  cap.position.y = 0.003;
  const arm = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.006, 0.2), mat);
  arm.position.set(0, 0.012, 0.03);
  const fin = new THREE.Mesh(new THREE.BoxGeometry(0.002, 0.045, 0.06), mat);
  fin.position.set(0, 0.022, 0.12);
  const nose = new THREE.Mesh(new THREE.SphereGeometry(0.008, 12, 10), mat);
  nose.position.set(0, 0.012, -0.07);
  g.add(body, cap, arm, fin, nose);
  return g;
}

// Hard stops at the mast base: two blocks on the deck and an arm on the mast collar that turns with the wing
export function hardStops(limitDeg) {
  const blocks = new THREE.Group();
  const arm = new THREE.Group();
  const mat = plain("#1d1d20", 0.6);
  const R = 0.055;
  // The arm points aft (+z). The wing pivot turns by -wing, so at wing = s*limit the arm tip is at
  // (-s*sin(limit), cos(limit)) * R: the blocks sit just past that, one per side.
  const L = THREE.MathUtils.degToRad(limitDeg), Rb = R + 0.012;
  for (const s of [-1, 1]) {
    const blk = new THREE.Mesh(new THREE.BoxGeometry(0.022, 0.03, 0.022), mat);
    blk.position.set(-s * Math.sin(L) * Rb, 0.015, Math.cos(L) * Rb);
    blk.rotation.y = s * L;
    blocks.add(blk);
  }
  const collar = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.02, 24), plain("#f1f1f1", 0.5));
  collar.position.y = 0.012;
  const bar = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.01, R), mat);
  bar.position.set(0, 0.012, R / 2);                                   // aft of the mast
  arm.add(collar, bar);
  return { blocks, arm };
}

export function servoBox(sizeM, color) {
  const g = new THREE.Group();
  const box = new THREE.Mesh(new THREE.BoxGeometry(sizeM[0], sizeM[2], sizeM[1]), plain(color || "#7f1d2d", 0.5));
  box.position.y = sizeM[2] / 2;
  const lid = new THREE.Mesh(new THREE.BoxGeometry(sizeM[0] * 1.04, 0.004, sizeM[1] * 1.04), plain("#111", 0.5));
  lid.position.y = sizeM[2];
  g.add(box, lid);
  return g;
}

// ---------------------------------------------------------------------------------------------
// Bench scene: wooden stand from the photo, lab floor, box fan
// ---------------------------------------------------------------------------------------------
function beam(a, b, w, h, mat) {
  const d = new THREE.Vector3().subVectors(b, a);
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d.length()), mat);
  m.position.copy(a).addScaledVector(d, 0.5);
  m.lookAt(b);
  return m;
}
// stations: z positions of the two cradles; halfBeam: hull half-width at the cradles; floorY, cradleY in the boat frame
export function woodenStand(stations, halfBeam, floorY, cradleY) {
  const g = new THREE.Group();
  const wood = new THREE.MeshStandardMaterial({ map: woodTex(), roughness: 0.8 });
  const black = plain("#18181b", 0.6);
  const spread = halfBeam + 0.17;
  for (const z of stations) {
    for (const s of [-1, 1]) {
      g.add(beam(new THREE.Vector3(s * spread, floorY, z), new THREE.Vector3(s * (halfBeam + 0.02), cradleY - 0.02, z), 0.038, 0.089, wood));
      const cradle = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.11, 0.075), black);
      cradle.position.set(s * (halfBeam + 0.012), cradleY + 0.02, z);
      g.add(cradle);
    }
    g.add(beam(new THREE.Vector3(-spread - 0.02, floorY + 0.045, z), new THREE.Vector3(spread + 0.02, floorY + 0.045, z), 0.038, 0.089, wood));
  }
  for (const s of [-1, 1]) {
    g.add(beam(new THREE.Vector3(s * (spread - 0.03), floorY + 0.14, stations[0]), new THREE.Vector3(s * (spread - 0.03), floorY + 0.14, stations[1]), 0.038, 0.089, wood));
  }
  return g;
}

export function labFloor(y) {
  const t = floorTex();
  t.repeat.set(10, 10);
  const m = new THREE.Mesh(new THREE.PlaneGeometry(24, 24), new THREE.MeshStandardMaterial({ map: t, roughness: 0.9 }));
  m.rotation.x = -Math.PI / 2;
  m.position.y = y;
  return m;
}

// Box fan facing the boat; call spin(dt, speed) every frame
export function boxFan() {
  const g = new THREE.Group();
  const frameMat = plain("#e8e8e8", 0.5);
  const S = 0.5, T = 0.05;
  for (const [x, y, w, h] of [[0, S / 2, S, T], [0, -S / 2, S, T], [S / 2, 0, T, S], [-S / 2, 0, T, S]]) {
    const b = new THREE.Mesh(new THREE.BoxGeometry(w + T, h + T, 0.12), frameMat);
    b.position.set(x, y, 0);
    g.add(b);
  }
  const grill = new THREE.Mesh(new THREE.RingGeometry(0.03, 0.23, 32, 1), new THREE.MeshStandardMaterial({ color: "#999", wireframe: true }));
  grill.position.z = -0.05;
  g.add(grill);
  const rotor = new THREE.Group();
  for (let i = 0; i < 5; i++) {
    const blade = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.2, 0.006), plain("#cfd8e3", 0.4));
    blade.position.y = 0.11;
    blade.rotation.y = 0.35;
    const arm = new THREE.Group();
    arm.add(blade);
    arm.rotation.z = (i / 5) * Math.PI * 2;
    rotor.add(arm);
  }
  rotor.add(new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.04, 20).rotateX(Math.PI / 2), plain("#333", 0.5)));
  g.add(rotor);
  for (const s of [-1, 1]) {
    const leg = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.03, 0.3), frameMat);
    leg.position.set(s * 0.18, -S / 2 - 0.04, 0);
    g.add(leg);
  }
  g.userData.spin = (dt, speed) => { rotor.rotation.z += dt * speed * 6; };
  return g;
}
