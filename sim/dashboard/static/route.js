// Route tab: pick A and B on a map, plan waypoints locally, sail them with the sim autopilot in changing wind.
const $ = (id) => document.getElementById(id);
const rad = (d) => (d * Math.PI) / 180;

export function initRoute({ cmd }) {
  let S = null, active = false;
  let A = { n: 0, e: 0 }, B = { n: 120, e: 25 };
  let plan = null, picking = "B";
  const view = { n: 50, e: 10, scale: 3 };          // map centre (m) and pixels per metre
  let track = [], drag = null, startT = null;

  // ---- map drawing ---------------------------------------------------------------------------------------
  function toPx(n, e, W, H) { return [W / 2 + (e - view.e) * view.scale, H / 2 - (n - view.n) * view.scale]; }
  function toM(x, y, W, H) { return { e: view.e + (x - W / 2) / view.scale, n: view.n - (y - H / 2) / view.scale }; }

  function draw() {
    const c = $("route-map"), r = c.getBoundingClientRect(), d = devicePixelRatio || 1;
    if (c.width !== Math.round(r.width * d) || c.height !== Math.round(r.height * d)) { c.width = Math.round(r.width * d); c.height = Math.round(r.height * d); }
    const ctx = c.getContext("2d"); ctx.setTransform(d, 0, 0, d, 0, 0);
    const W = r.width, H = r.height, P = (p) => toPx(p.n ?? p[0], p.e ?? p[1], W, H);
    ctx.fillStyle = "#0e3a5a"; ctx.fillRect(0, 0, W, H);
    // grid every 10 m (50 m bold)
    const m0 = toM(0, H, W, H), m1 = toM(W, 0, W, H);
    for (let e = Math.floor(m0.e / 10) * 10; e <= m1.e; e += 10) {
      const [x] = toPx(0, e, W, H); ctx.strokeStyle = e % 50 === 0 ? "rgba(255,255,255,.16)" : "rgba(255,255,255,.06)";
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
    }
    for (let n = Math.floor(m0.n / 10) * 10; n <= m1.n; n += 10) {
      const [, y] = toPx(n, 0, W, H); ctx.strokeStyle = n % 50 === 0 ? "rgba(255,255,255,.16)" : "rgba(255,255,255,.06)";
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    }
    // corridor and plan
    const margin = +$("route-margin").value;
    if (plan) {
      const [ax, ay] = P(A), [bx, by] = P(B);
      const len = Math.hypot(bx - ax, by - ay) || 1, nx = -(by - ay) / len, ny = (bx - ax) / len, w = margin * view.scale;
      ctx.fillStyle = "rgba(56,189,248,.10)"; ctx.beginPath();
      ctx.moveTo(ax + nx * w, ay + ny * w); ctx.lineTo(bx + nx * w, by + ny * w); ctx.lineTo(bx - nx * w, by - ny * w); ctx.lineTo(ax - nx * w, ay - ny * w); ctx.fill();
      ctx.strokeStyle = "#38bdf8"; ctx.lineWidth = 2; ctx.setLineDash([6, 5]); ctx.beginPath();
      plan.waypoints.forEach((p, i) => { const [x, y] = P(p); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); ctx.setLineDash([]);
      const ap = S && S.autopilot;
      plan.waypoints.forEach((p, i) => {
        const [x, y] = P(p);
        ctx.fillStyle = ap && i === ap.index ? "#fde047" : ap && i < ap.index ? "#64748b" : "#bae6fd";
        ctx.beginPath(); ctx.arc(x, y, i === 0 || i === plan.waypoints.length - 1 ? 0 : 4, 0, Math.PI * 2); ctx.fill();
      });
    }
    // no-go zone at B (where the boat cannot point)
    if (S) {
      const [bx, by] = P(B), twd = S.wind_dir, R = 60;
      ctx.fillStyle = "rgba(239,68,68,.12)"; ctx.beginPath(); ctx.moveTo(bx, by);
      for (let a = twd - 45; a <= twd + 45; a += 3) ctx.lineTo(bx + Math.sin(rad(a)) * R, by - Math.cos(rad(a)) * R);
      ctx.closePath(); ctx.fill();
    }
    // track and boat
    ctx.strokeStyle = "#fde047"; ctx.lineWidth = 2.5; ctx.beginPath();
    track.forEach((p, i) => { const [x, y] = P(p); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();
    for (const [p, label, col] of [[A, "A", "#22c55e"], [B, "B", "#ef4444"]]) {
      const [x, y] = P(p);
      ctx.fillStyle = col; ctx.beginPath(); ctx.arc(x, y, 8, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = "#0f172a"; ctx.font = "700 11px system-ui"; ctx.textAlign = "center"; ctx.fillText(label, x, y + 4);
    }
    if (S) {
      const [x, y] = P({ n: S.north, e: S.east });
      const ap = S.autopilot;
      if (ap && ap.target_heading != null && !ap.done) {
        ctx.strokeStyle = "rgba(253,224,71,.6)"; ctx.setLineDash([3, 4]); ctx.beginPath(); ctx.moveTo(x, y);
        ctx.lineTo(x + Math.sin(rad(ap.target_heading)) * 40, y - Math.cos(rad(ap.target_heading)) * 40); ctx.stroke(); ctx.setLineDash([]);
      }
      ctx.save(); ctx.translate(x, y); ctx.rotate(rad(S.heading));
      const L = Math.max(10, 1.0 * view.scale);
      ctx.fillStyle = "#f1f5f9"; ctx.strokeStyle = "#0f172a"; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(0, -L); ctx.lineTo(L * 0.35, L * 0.6); ctx.lineTo(-L * 0.35, L * 0.6); ctx.closePath(); ctx.fill(); ctx.stroke();
      ctx.restore();
      // wind arrow
      const wx = W - 60, wy = 60, ws = 28;
      ctx.strokeStyle = "#e5e7eb"; ctx.fillStyle = "#e5e7eb"; ctx.lineWidth = 3;
      const tx = wx - Math.sin(rad(S.wind_dir)) * ws, ty = wy + Math.cos(rad(S.wind_dir)) * ws;
      const fx = wx + Math.sin(rad(S.wind_dir)) * ws, fy = wy - Math.cos(rad(S.wind_dir)) * ws;
      ctx.beginPath(); ctx.moveTo(fx, fy); ctx.lineTo(tx, ty); ctx.stroke();
      const a = Math.atan2(ty - fy, tx - fx);
      ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(tx - 10 * Math.cos(a - 0.4), ty - 10 * Math.sin(a - 0.4)); ctx.lineTo(tx - 10 * Math.cos(a + 0.4), ty - 10 * Math.sin(a + 0.4)); ctx.fill();
      ctx.font = "12px system-ui"; ctx.textAlign = "center";
      ctx.fillText(`wind ${S.wind_dir.toFixed(0)}° ${S.wind_speed.toFixed(1)} m/s`, wx, wy + 48);
    }
    ctx.fillStyle = "#94a3b8"; ctx.font = "11px system-ui"; ctx.textAlign = "left";
    ctx.fillText(`click: set ${picking} · drag: pan · scroll: zoom · grid 10 m · north up`, 10, H - 10);
    // scale bar
    const bar = 50 * view.scale; ctx.strokeStyle = "#e5e7eb"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(10, H - 28); ctx.lineTo(10 + bar, H - 28); ctx.stroke(); ctx.fillText("50 m", 14, H - 33);
  }

  // ---- interaction ---------------------------------------------------------------------------------------
  const map = $("route-map");
  map.addEventListener("pointerdown", (e) => { drag = { x: e.clientX, y: e.clientY, n: view.n, e: view.e, moved: false }; map.setPointerCapture(e.pointerId); });
  map.addEventListener("pointermove", (e) => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    if (Math.hypot(dx, dy) > 4) drag.moved = true;
    if (drag.moved) { view.e = drag.e - dx / view.scale; view.n = drag.n + dy / view.scale; draw(); }
  });
  map.addEventListener("pointerup", (e) => {
    if (drag && !drag.moved) {
      const r = map.getBoundingClientRect(), p = toM(e.clientX - r.left, e.clientY - r.top, r.width, r.height);
      if (picking === "A") { A = p; picking = "B"; } else { B = p; picking = "A"; }
      plan = null; updateText();
    }
    drag = null; draw();
  });
  map.addEventListener("wheel", (e) => { e.preventDefault(); view.scale = Math.min(30, Math.max(0.4, view.scale * Math.exp(-e.deltaY * 0.0015))); draw(); }, { passive: false });

  async function doPlan(from) {
    const a = from || A;
    const q = `an=${a.n}&ae=${a.e}&bn=${B.n}&be=${B.e}&margin=${$("route-margin").value}`;
    plan = await (await fetch(`/api/plan?${q}`)).json();
    if (from) { A = { ...from }; }
    updateText(); draw();
    return plan;
  }
  $("route-plan").onclick = () => doPlan();
  $("route-go").onclick = async () => {
    if (!plan) await doPlan();
    track = []; startT = S ? S.t : 0;
    const h = plan.waypoints.length > 1 ? (Math.atan2(plan.waypoints[1][1] - A.e, plan.waypoints[1][0] - A.n) * 180 / Math.PI + 360) % 360 : 0;
    await cmd(`place ${A.n.toFixed(2)} ${A.e.toFixed(2)} ${h.toFixed(0)}`);
    await cmd(`route ${plan.waypoints.map((p) => `${(+p[0]).toFixed(2)} ${(+p[1]).toFixed(2)}`).join(" ")}`);
    await cmd(`autopilot aoa ${$("route-aoa").value}`);
  };
  $("route-replan").onclick = async () => {
    if (!S) return;
    const here = { n: S.north, e: S.east };
    await doPlan(here);
    await cmd(`route ${plan.waypoints.map((p) => `${(+p[0]).toFixed(2)} ${(+p[1]).toFixed(2)}`).join(" ")}`);
  };
  $("route-stop").onclick = () => cmd("autopilot off");
  $("route-shift-l").onclick = () => S && cmd(`wind ${((S.wind_mean_dir - 20) + 360) % 360} ${S.wind_mean_speed}`);
  $("route-shift-r").onclick = () => S && cmd(`wind ${(S.wind_mean_dir + 20) % 360} ${S.wind_mean_speed}`);
  $("route-swap").onclick = () => { [A, B] = [B, A]; plan = null; updateText(); draw(); };
  for (const id of ["route-margin", "route-aoa"]) {
    $(id).addEventListener("input", () => {
      $(`${id}-v`).textContent = id === "route-margin" ? `${$(id).value} m` : `${$(id).value}°`;
      if (id === "route-aoa" && S && S.autopilot) cmd(`autopilot aoa ${$(id).value}`);
      if (id === "route-margin") { plan = null; draw(); }
    });
  }

  function updateText() {
    const d = Math.hypot(B.n - A.n, B.e - A.e);
    let t = `A (${A.n.toFixed(0)} N, ${A.e.toFixed(0)} E) → B (${B.n.toFixed(0)} N, ${B.e.toFixed(0)} E): ${d.toFixed(0)} m. `;
    if (plan) t += plan.mode === "direct" ? "B can be sailed straight." :
      `${plan.mode === "beat" ? "B is upwind" : "B is dead downwind"}: ${plan.legs} legs (${plan.mode === "beat" ? "tacks" : "gybes"}) inside ±${$("route-margin").value} m.`;
    $("route-summary").textContent = t;
  }

  function update(state) {
    S = state;
    if (!active) return;
    const ap = S.autopilot;
    if (ap && !ap.done) {
      const last = track[track.length - 1];
      if (!last || Math.hypot(last.n - S.north, last.e - S.east) > 0.5) track.push({ n: S.north, e: S.east });
      if (track.length > 4000) track.shift();
    }
    const dB = Math.hypot(B.n - S.north, B.e - S.east);
    const vmg = S.sog * Math.cos(rad(S.cog - (Math.atan2(B.e - S.east, B.n - S.north) * 180 / Math.PI)));
    $("route-status").innerHTML = [
      ["autopilot", ap ? (ap.done ? "arrived at B" : ap.status) : "off"],
      ["distance to B", `${dB.toFixed(1)} m`], ["speed", `${S.sog.toFixed(2)} m/s`], ["speed toward B", `${vmg.toFixed(2)} m/s`],
      ["elapsed", ap && startT != null ? `${(S.t - startT).toFixed(0)} s` : "–"], ["heel", `${S.heel.toFixed(0)}°`],
      ["wind", `${S.wind_dir.toFixed(0)}° @ ${S.wind_speed.toFixed(1)} m/s`],
      ["who steers", ap && !ap.done ? "sim autopilot (stand-in for your LTP + PIDs)" : S.mode === "firmware" ? "your firmware" : "sliders"],
    ].map(([k, v]) => `<span>${k}</span><b>${v}</b>`).join("");
    draw();
  }

  return {
    show() { active = true; updateText(); draw(); },
    hide() { active = false; },
    update,
  };
}
