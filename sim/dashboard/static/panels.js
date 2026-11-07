// Stability and Points-of-sail panels for the ALAN simulator dashboard.
const $ = (id) => document.getElementById(id);
const rad = (d) => (d * Math.PI) / 180;
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();

function fitCanvas(c) {
  const r = c.getBoundingClientRect();
  const d = devicePixelRatio || 1;
  if (c.width !== Math.round(r.width * d) || c.height !== Math.round(r.height * d)) {
    c.width = Math.round(r.width * d);
    c.height = Math.round(r.height * d);
  }
  const ctx = c.getContext("2d");
  ctx.setTransform(d, 0, 0, d, 0, 0);
  return { ctx, W: r.width, H: r.height };
}
function marker(ctx, x, y, color, label) {
  ctx.fillStyle = color; ctx.strokeStyle = "#0f172a"; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(x, y, 5.5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  ctx.fillStyle = "#e5e7eb"; ctx.font = "12px system-ui"; ctx.fillText(label, x + 8, y - 6);
}
function arrow(ctx, x0, y0, x1, y1, color) {
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
  const a = Math.atan2(y1 - y0, x1 - x0);
  ctx.beginPath(); ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - 8 * Math.cos(a - 0.4), y1 - 8 * Math.sin(a - 0.4));
  ctx.lineTo(x1 - 8 * Math.cos(a + 0.4), y1 - 8 * Math.sin(a + 0.4));
  ctx.fill();
}

// =============================================================================================
// Stability
// =============================================================================================
export function initStability(post) {
  let H = null;          // last /api/hydro response
  let busy = false, pendingHeel = null;

  async function load(heel) {
    if (busy) { pendingHeel = heel; return; }
    busy = true;
    try {
      const r = await (await fetch(`/api/hydro?heel=${heel}`)).json();
      if (!r.available) {
        $("st-summary").innerHTML = "<span>No CAD/mass data. Run sim/tools/prepare_cad.py.</span><b></b>";
        return;
      }
      H = r;
      draw();
    } finally {
      busy = false;
      if (pendingHeel !== null) { const h = pendingHeel; pendingHeel = null; load(h); }
    }
  }

  function draw() {
    if (!H) return;
    drawFront(); drawSide(); drawGZ(); summary();
  }

  function drawFront() {
    const { ctx, W, H: Hc } = fitCanvas($("st-front"));
    const s = H.silhouettes, res = H.result, c = s.cell_mm;
    const wingZ = H.outlines.wing ? [Math.min(...H.outlines.wing.map((p) => p[1])), Math.max(...H.outlines.wing.map((p) => p[1]))] : [300, 1300];
    const zg = res.cg_mm[2], wl = -s.cg_front[1];
    const heel = rad(H.heel);
    const wingPt = (z) => [-(z - zg) * Math.sin(heel), (z - zg) * Math.cos(heel) - wl];
    const pts = [...s.front.map(([i, j]) => [i * c, j * c]), wingPt(wingZ[0]), wingPt(wingZ[1])];
    const xs = pts.map((p) => p[0]), zs = pts.map((p) => p[1]);
    const span = Math.max(Math.max(...xs) - Math.min(...xs), 500) * 1.15, spanZ = (Math.max(...zs) - Math.min(...zs)) * 1.1;
    const k = Math.min(W / span, Hc / spanZ);
    const ox = W / 2, oy = 14 + Math.max(...zs) * k;
    const X = (x) => ox - x * k, Y = (z) => oy - z * k;        // starboard (negative x) drawn on the right
    ctx.clearRect(0, 0, W, Hc);
    ctx.fillStyle = "#0c4a6e"; ctx.fillRect(0, Y(0), W, Hc - Y(0));
    ctx.strokeStyle = "#38bdf8"; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(0, Y(0)); ctx.lineTo(W, Y(0)); ctx.stroke();
    ctx.fillStyle = "rgba(226,232,240,.85)";
    for (const [i, j] of s.front) ctx.fillRect(X(i * c + c / 2), Y(j * c + c / 2), c * k + 0.6, c * k + 0.6);
    const [wx0, wz0] = wingPt(wingZ[0]), [wx1, wz1] = wingPt(wingZ[1]);
    ctx.strokeStyle = "#d8b4e2"; ctx.lineWidth = Math.max(3, 40 * k); ctx.lineCap = "round";
    ctx.beginPath(); ctx.moveTo(X(wx0), Y(wz0)); ctx.lineTo(X(wx1), Y(wz1)); ctx.stroke(); ctx.lineCap = "butt";
    const cg = s.cg_front, cb = s.cb_front;
    arrow(ctx, X(cg[0]), Y(cg[1]), X(cg[0]), Y(cg[1]) + 46, "#f8fafc");
    arrow(ctx, X(cb[0]), Y(cb[1]), X(cb[0]), Y(cb[1]) - 46, "#38bdf8");
    marker(ctx, X(cg[0]), Y(cg[1]), "#111827", "CG");
    marker(ctx, X(cb[0]), Y(cb[1]), "#38bdf8", "CB");
    const row = res.gz_curve.find((p) => p.heel === Math.round(Math.abs(H.heel) / 5) * 5);
    ctx.fillStyle = "#e5e7eb"; ctx.font = "12px system-ui";
    ctx.fillText(H.heel === 0 ? "upright" : `heel ${Math.abs(H.heel)}° to ${H.heel > 0 ? "starboard" : "port"}`, 10, 18);
    if (row) ctx.fillText(`righting arm GZ ${row.gz_mm.toFixed(0)} mm, moment ${row.moment_nm.toFixed(2)} N·m`, 10, 34);
    ctx.fillStyle = "#94a3b8"; ctx.fillText("port", 10, Hc - 10); ctx.fillText("starboard", W - 70, Hc - 10);
  }

  function drawSide() {
    const { ctx, W, H: Hc } = fitCanvas($("st-side"));
    const s = H.silhouettes, res = H.result, c = s.cell_mm;
    const yg = res.cg_mm[1], wlz = res.waterline_z_mm;
    const outl = H.outlines;
    const all = [...s.side.map(([i, j]) => [i * c, j * c])];
    const polys = (n) => (n === "booms" ? outl[n] || [] : outl[n] ? [outl[n]] : []);
    for (const n of ["wing", "tail", "booms", "rudder"]) for (const poly of polys(n)) for (const [y, z] of poly) all.push([y - yg, z - wlz]);
    const ys = all.map((p) => p[0]), zs = all.map((p) => p[1]);
    const k = Math.min(W / ((Math.max(...ys) - Math.min(...ys)) * 1.08), Hc / ((Math.max(...zs) - Math.min(...zs)) * 1.12));
    const ox = W / 2 - ((Math.max(...ys) + Math.min(...ys)) / 2) * k, oy = 10 + Math.max(...zs) * k;
    const X = (y) => ox + y * k, Y = (z) => oy - z * k;               // bow (negative y) on the left
    ctx.clearRect(0, 0, W, Hc);
    ctx.fillStyle = "#0c4a6e"; ctx.fillRect(0, Y(0), W, Hc - Y(0));
    ctx.strokeStyle = "#38bdf8"; ctx.beginPath(); ctx.moveTo(0, Y(0)); ctx.lineTo(W, Y(0)); ctx.stroke();
    const colors = { wing: "rgba(216,180,226,.8)", tail: "rgba(216,180,226,.8)", booms: "#cbd5e1", rudder: "#f97316" };
    for (const n of ["wing", "tail", "booms"]) {
      for (const poly of polys(n)) {
        ctx.fillStyle = colors[n]; ctx.beginPath();
        poly.forEach(([y, z], i) => (i ? ctx.lineTo(X(y - yg), Y(z - wlz)) : ctx.moveTo(X(y - yg), Y(z - wlz))));
        ctx.closePath(); ctx.fill();
        if (n === "booms") { ctx.strokeStyle = colors[n]; ctx.lineWidth = 2; ctx.stroke(); }
      }
    }
    ctx.fillStyle = "rgba(226,232,240,.85)";
    for (const [i, j] of s.side) ctx.fillRect(X(i * c - c / 2), Y(j * c + c / 2), c * k + 0.6, c * k + 0.6);
    marker(ctx, X(0), Y(res.cg_mm[2] - wlz), "#111827", "CG");
    marker(ctx, X(res.cb_mm[1] - yg), Y(res.cb_mm[2] - wlz), "#38bdf8", "CB");
    ctx.fillStyle = "#94a3b8"; ctx.font = "12px system-ui"; ctx.fillText("bow", 6, Hc - 8); ctx.fillText("stern", W - 40, Hc - 8);
  }

  function heelingMoment(v, heelDeg) {
    // side force on the wing at a good angle of attack (side-force coefficient ~1.0), lever shortens with heel
    const c = Math.cos(rad(heelDeg));
    return 0.5 * 1.2 * H.wing_area_m2 * c * v * v * 1.0 * H.lever_m * c;
  }
  function equilibrium(v) {
    const curve = H.result.gz_curve;
    for (let i = 1; i < curve.length; i++) {
      const a = curve[i - 1], b = curve[i];
      const fa = a.moment_nm - heelingMoment(v, a.heel), fb = b.moment_nm - heelingMoment(v, b.heel);
      if (fa <= 0 && fb > 0 && i === 1) return 0;
      if (fa < 0 && fb >= 0) return a.heel + (b.heel - a.heel) * (-fa / (fb - fa));
      if (i === 1 && fa >= 0) return 0;
    }
    return null;
  }

  function drawGZ() {
    const { ctx, W, H: Hc } = fitCanvas($("st-gz"));
    const curve = H.result.gz_curve, res = H.result;
    const maxH = curve[curve.length - 1].heel, v = +$("st-wind").value;
    const mMax = Math.max(...curve.map((p) => p.moment_nm), heelingMoment(v, 0)) * 1.1;
    const L = 46, R = W - 10, T = 12, B = Hc - 28;
    const X = (h) => L + (h / maxH) * (R - L), Y = (m) => B - (m / mMax) * (B - T);
    ctx.clearRect(0, 0, W, Hc);
    ctx.strokeStyle = "#334155"; ctx.fillStyle = "#94a3b8"; ctx.font = "11px system-ui"; ctx.lineWidth = 1;
    for (let h = 0; h <= maxH; h += 15) { ctx.beginPath(); ctx.moveTo(X(h), T); ctx.lineTo(X(h), B); ctx.stroke(); ctx.fillText(`${h}°`, X(h) - 8, B + 14); }
    for (let i = 0; i <= 4; i++) { const m = (mMax * i) / 4; ctx.beginPath(); ctx.moveTo(L, Y(m)); ctx.lineTo(R, Y(m)); ctx.stroke(); ctx.fillText(i === 4 ? "N·m" : m.toFixed(1), 6, Y(m) + 4); }
    ctx.fillText("heel", R - 26, B + 24);
    const mark = (h, label, col) => { if (h == null) return; ctx.strokeStyle = col; ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(X(h), T); ctx.lineTo(X(h), B); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col; ctx.fillText(label, X(h) + 4, T + 12); };
    mark(res.deck_edge_deg, "deck edge under", "#f59e0b");
    ctx.strokeStyle = "#38bdf8"; ctx.lineWidth = 2.2; ctx.beginPath();
    curve.forEach((p, i) => (i ? ctx.lineTo(X(p.heel), Y(p.moment_nm)) : ctx.moveTo(X(p.heel), Y(p.moment_nm)))); ctx.stroke();
    ctx.strokeStyle = "#f97316"; ctx.lineWidth = 2; ctx.beginPath();
    for (let h = 0; h <= Math.min(90, maxH); h += 2) (h ? ctx.lineTo(X(h), Y(heelingMoment(v, h))) : ctx.moveTo(X(h), Y(heelingMoment(v, h))));
    ctx.stroke();
    ctx.font = "12px system-ui"; ctx.fillStyle = "#38bdf8"; ctx.fillText("righting (keel + hull)", L + 6, T + 28);
    ctx.fillStyle = "#f97316"; ctx.fillText(`heeling, ${v.toFixed(1)} m/s apparent wind`, L + 6, T + 44);
    const eq = equilibrium(v);
    if (eq != null) { ctx.fillStyle = "#f8fafc"; ctx.beginPath(); ctx.arc(X(eq), Y(heelingMoment(v, eq)), 5, 0, Math.PI * 2); ctx.fill(); }
    $("st-wind-note").textContent = eq == null
      ? `At ${v} m/s the wing overpowers the keel before the deck edge: ease the wing (smaller angle of attack) or reef.`
      : `At ${v} m/s apparent wind, sailing with the wing fully powered, the boat settles at about ${eq.toFixed(0)}° of heel` +
        (res.deck_edge_deg && eq > res.deck_edge_deg ? " (deck edge under water)." : ".");
  }

  function summary() {
    const r = H.result;
    const rows = [
      ["mass", `${r.mass_kg.toFixed(3)} kg`], ["CG (x, y, z)", r.cg_mm.map((v) => v.toFixed(0)).join(", ") + " mm"],
      ["waterline height", `${r.waterline_z_mm.toFixed(0)} mm above hull bottom`], ["trim", `${Math.abs(r.trim_deg).toFixed(1)}° ${r.trim_deg < 0 ? "bow up" : "bow down"}`],
      ["draft (keel tip)", `${r.draft_mm.toFixed(0)} mm`], ["freeboard", `${r.freeboard_mm.toFixed(0)} mm`],
      ["GM (initial stiffness)", `${r.gm_mm} mm`], ["deck edge under at", r.deck_edge_deg ? `${r.deck_edge_deg}°` : "–"],
      ["capsizes beyond", r.vanishing_deg ? `${r.vanishing_deg}°` : "never (self-righting)"],
      ["reserve buoyancy", `${r.reserve_buoyancy_l.toFixed(1)} kg before it sinks`],
    ];
    $("st-summary").innerHTML = rows.map(([k, v]) => `<span>${k}</span><b>${v}</b>`).join("");
    if (!document.activeElement || !$("st-mass").contains(document.activeElement)) massTable(r.items);
  }

  function massTable(items) {
    const tb = $("st-mass").querySelector("tbody");
    tb.innerHTML = "";
    for (const it of items) addRow(it);
  }
  function addRow(it = { name: "new part", kg: 0, at_mm: [0, 0, 0] }) {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td><input value="${it.name.replace(/"/g, "&quot;")}" title="${(it.note || "").replace(/"/g, "&quot;")}"></td>` +
      `<td><input type="number" step="0.001" value="${it.kg}"></td>` +
      it.at_mm.map((v) => `<td><input type="number" step="1" value="${v}"></td>`).join("") +
      `<td><button title="remove">×</button></td>`;
    tr.querySelector("button").onclick = () => tr.remove();
    tr.dataset.note = it.note || "";
    $("st-mass").querySelector("tbody").appendChild(tr);
  }
  $("st-add").onclick = () => addRow();
  $("st-save").onclick = async () => {
    const items = [...$("st-mass").querySelectorAll("tbody tr")].map((tr) => {
      const v = [...tr.querySelectorAll("input")].map((i) => i.value);
      return { name: v[0], kg: +v[1], at_mm: [+v[2], +v[3], +v[4]], note: tr.dataset.note };
    });
    $("st-msg").textContent = "recomputing…";
    const r = await post("/api/mass", { items });
    $("st-msg").textContent = r.error ? r.error : "saved; simulation restarted with the new mass";
    if (!r.error) { H = r; $("st-heel").value = 0; $("st-heel-v").textContent = "0°"; draw(); }
  };

  $("st-heel").addEventListener("input", () => { $("st-heel-v").textContent = `${$("st-heel").value}°`; load(+$("st-heel").value); });
  $("st-wind").addEventListener("input", () => { $("st-wind-v").textContent = `${(+$("st-wind").value).toFixed(1)} m/s`; drawGZ(); });
  window.addEventListener("resize", draw);
  return { show: () => (H ? draw() : load(+$("st-heel").value)) };
}

// =============================================================================================
// Points of sail
// =============================================================================================
export function initPointsOfSail() {
  let rows = null, hover = null, layout = [];

  async function compute() {
    $("pos-msg").textContent = $("pos-opt").checked ? "sailing every heading at several angles of attack…" : "sailing every heading…";
    $("pos-run").disabled = true;
    try {
      const q = `tws=${$("pos-tws").value}&aoa=${$("pos-aoa").value}&optimize=${$("pos-opt").checked ? 1 : 0}`;
      const r = await (await fetch(`/api/polar?${q}`)).json();
      rows = r.rows;
      $("pos-msg").textContent = "";
      draw();
    } finally {
      $("pos-run").disabled = false;
    }
  }

  // The physics sails with the wind on the port side; mirror for the starboard tack.
  function both() {
    const out = [];
    for (const r of rows) {
      out.push({ ...r, heading: r.twa, wing: r.wing_deg, tail: r.tail_deg, tack: "port tack" });
      if (r.twa > 0 && r.twa < 180) out.push({ ...r, heading: -r.twa, wing: -r.wing_deg, tail: -r.tail_deg, awa_deg: -r.awa_deg, tack: "starboard tack" });
    }
    return out;
  }

  function drawBoat(ctx, x, y, headingDeg, wingDeg, tailDeg, scale, highlight) {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(rad(headingDeg));
    const L = 60 * scale, B = 17 * scale;
    ctx.fillStyle = highlight ? "#38bdf8" : "#5eead4"; ctx.strokeStyle = "#0f172a"; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, -L / 2);
    ctx.quadraticCurveTo(B, -L * 0.15, B * 0.75, L / 2);
    ctx.lineTo(-B * 0.75, L / 2);
    ctx.quadraticCurveTo(-B, -L * 0.15, 0, -L / 2);
    ctx.fill(); ctx.stroke();
    // wing: pivots at the mast (14% of the length ahead of centre), chord drawn from the leading edge aft
    const mastY = -0.14 * L, chord = 0.34 * L;
    ctx.translate(0, mastY);
    ctx.rotate(rad(wingDeg));
    ctx.strokeStyle = "#d4d4d8"; ctx.lineWidth = 4.5 * scale; ctx.lineCap = "round";
    ctx.beginPath(); ctx.moveTo(0, -chord * 0.25); ctx.lineTo(0, chord * 0.75); ctx.stroke();
    ctx.strokeStyle = "#64748b"; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.moveTo(0, chord * 0.75); ctx.lineTo(0, chord * 1.45); ctx.stroke();
    ctx.translate(0, chord * 1.45);
    ctx.rotate(rad(tailDeg));
    ctx.strokeStyle = "#f97316"; ctx.lineWidth = 3.2 * scale;
    ctx.beginPath(); ctx.moveTo(0, -chord * 0.1); ctx.lineTo(0, chord * 0.32); ctx.stroke();
    ctx.restore();
  }

  function draw() {
    if (!rows) return;
    const { ctx, W, H } = fitCanvas($("pos-ring"));
    ctx.clearRect(0, 0, W, H);
    const cx = W / 2, cy = H / 2 + 28, R = Math.min(W * 0.28, H * 0.35), scale = Math.min(1.2, R / 190);
    ctx.fillStyle = "#e5e7eb"; ctx.font = "600 22px system-ui"; ctx.textAlign = "center";
    ctx.fillText("Points", cx, cy - 18); ctx.fillText("of sail", cx, cy + 10);
    ctx.font = "13px system-ui"; ctx.fillStyle = "#94a3b8";
    ctx.fillText(`${(+$("pos-tws").value).toFixed(1)} m/s true wind`, cx, cy + 32);
    arrow(ctx, 30, 10, 30, 56, "#f8fafc"); arrow(ctx, 48, 10, 48, 56, "#f8fafc");
    ctx.fillStyle = "#f8fafc"; ctx.textAlign = "left"; ctx.font = "600 16px system-ui"; ctx.fillText("Wind", 60, 38);
    layout = [];
    for (const b of both()) {
      const a = rad(b.heading);
      const x = cx + R * Math.sin(a), y = cy - R * Math.cos(a);
      const hl = hover && hover.heading === b.heading;
      drawBoat(ctx, x, y, b.heading, b.wing, b.tail, scale, hl);
      layout.push({ x, y, b });
      // names once per side at a representative heading (with its speed); other boats get just their speed
      const sa = Math.sin(a), ca = Math.cos(a);
      const align = Math.abs(sa) < 0.25 ? "center" : sa > 0 ? "left" : "right";
      const NAMED = { 45: "Close hauled", 75: "Close reach", 90: "Beam reach", 135: "Broad reach", 165: "Running" };
      const place = (text, dist, font, color, dy = 0) => {
        ctx.font = font; ctx.fillStyle = color; ctx.textAlign = align;
        const w = ctx.measureText(text).width;
        let x = cx + dist * sa;
        if (align === "left") x = Math.min(x, W - 6 - w);
        if (align === "right") x = Math.max(x, 6 + w);
        ctx.fillText(text, x, cy - dist * ca + 4 + dy);
      };
      if (NAMED[b.twa]) {
        place(NAMED[b.twa], R + 46 * scale, "600 13px system-ui", "#e5e7eb", -7);
        place(`${b.speed_ms.toFixed(2)} m/s`, R + 46 * scale, "11px system-ui", "#94a3b8", 8);
      } else if (b.twa !== 0) {
        place(`${b.speed_ms.toFixed(2)}`, R + 40 * scale, "10px system-ui", "#64748b");
      }
    }
    ctx.font = "600 13px system-ui"; ctx.fillStyle = "#e5e7eb"; ctx.textAlign = "center";
    ctx.fillText("In irons (into the wind)", cx, cy - R - 46 * scale);
    ctx.textAlign = "left";
    drawPolar();
    detail(hover ? hover : both().find((b) => b.twa === 90));
  }

  function drawPolar() {
    const { ctx, W, H } = fitCanvas($("pos-polar"));
    ctx.clearRect(0, 0, W, H);
    const vmax = Math.max(...rows.map((r) => r.speed_ms), 0.1) * 1.15;
    const cx = W / 2, cy = 16, R = Math.min(W / 2 - 20, H - 36);
    ctx.strokeStyle = "#334155"; ctx.fillStyle = "#94a3b8"; ctx.font = "11px system-ui"; ctx.lineWidth = 1;
    for (let i = 1; i <= 4; i++) {
      const r = (R * i) / 4;
      ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI); ctx.stroke();
      ctx.fillText(`${((vmax * i) / 4).toFixed(2)}`, cx + 3, cy + r - 2);
    }
    for (let t = 0; t <= 180; t += 30) {
      const a = rad(t);
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + R * Math.sin(a), cy + R * Math.cos(a)); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx - R * Math.sin(a), cy + R * Math.cos(a)); ctx.stroke();
    }
    ctx.fillText("upwind", cx - 18, cy - 4); ctx.fillText("downwind", cx - 26, cy + R + 14); ctx.fillText("m/s", cx + R - 18, cy + 12);
    ctx.strokeStyle = "#38bdf8"; ctx.lineWidth = 2.2;
    for (const side of [1, -1]) {
      ctx.beginPath();
      rows.forEach((r, i) => {
        const a = rad(r.twa), rr = (Math.max(0, r.speed_ms) / vmax) * R;
        const x = cx + side * rr * Math.sin(a), y = cy + rr * Math.cos(a);
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      });
      ctx.stroke();
    }
    const best = rows.reduce((a, b) => (b.vmg_ms > a.vmg_ms ? b : a));
    ctx.fillStyle = "#e5e7eb"; ctx.font = "12px system-ui";
    ctx.fillText(`best upwind VMG ${best.vmg_ms.toFixed(2)} m/s at ${best.twa}° to the wind`, 10, H - 8);
  }

  function detail(b) {
    if (!b) return;
    $("pos-detail").innerHTML = [
      ["point of sail", `${b.point} (${b.tack || ""})`], ["true wind angle", `${Math.abs(b.heading)}°`],
      ["boat speed", `${b.speed_ms.toFixed(2)} m/s`], ["apparent wind", `${b.awa_deg.toFixed(0)}° @ ${b.aws_ms.toFixed(1)} m/s`],
      ["wing angle (to hull)", `${b.wing.toFixed(0)}°`], ["wing angle of attack", `${b.aoa_deg.toFixed(1)}° (target ${b.aoa_target}°)`],
      ["tail angle", `${b.tail.toFixed(1)}°`], ["heel", `${Math.abs(b.heel_deg).toFixed(0)}°`], ["leeway", `${b.leeway_deg.toFixed(1)}°`],
    ].map(([k, v]) => `<span>${k}</span><b>${v}</b>`).join("");
  }

  $("pos-ring").addEventListener("mousemove", (e) => {
    const r = $("pos-ring").getBoundingClientRect();
    const x = e.clientX - r.left, y = e.clientY - r.top;
    let best = null, bd = 40;
    for (const l of layout) { const d = Math.hypot(l.x - x, l.y - y); if (d < bd) { bd = d; best = l.b; } }
    if (best !== hover) { hover = best; draw(); }
  });
  $("pos-run").onclick = compute;
  for (const id of ["pos-tws", "pos-aoa"]) {
    $(id).addEventListener("input", () => { $(`${id}-v`).textContent = id === "pos-tws" ? `${(+$(id).value).toFixed(1)} m/s` : `${$(id).value}°`; });
    $(id).addEventListener("change", compute);
  }
  $("pos-opt").addEventListener("change", compute);
  window.addEventListener("resize", draw);
  return { show: () => (rows ? draw() : compute()) };
}
