// Firmware route tab: place waypoints on a map, send them to the emulated board as ground-station commands,
// and watch the firmware's Live Tack Planner (LTP) sail them inside each leg's corridor.
//
// The commands take the real path (sim/dashboard/fwlink.py): the plant bridge writes them into the IPC ring in
// SRAM4, Commander pops them, the Route Manager checks the CRC, and LTP picks up the new route version.
// What is drawn here is what LTP checks against:
//   corridor   +/- margin_m of the leg that ends at each waypoint (cross-track error from the leg's line)
//   tack lines +/- NAV_TACK_XTE_FRACTION x margin: upwind, LTP tacks when it reaches one heading outward
//   warning    +/- (margin + margin_warning_m): beyond it LTP latches a corridor fault and asks for SAFE
//   circles    arrive_m around each waypoint
// Leg 0 starts where the boat is when LTP takes the route, not at a waypoint.
const $ = (id) => document.getElementById(id);
const rad = (d) => (d * Math.PI) / 180;

// From CM7/App/Inc/navigationTack.h
const NAV_NO_GO_DEG = 45;
const NAV_TACK_XTE_FRACTION = 0.8;
const MAX_WAYPOINTS = 32;

const RESULT_OK = 1;

export function initFwRoute({ post }) {
  let S = null, active = false;
  let draft = [];                         // [{n, e, margin}], what you are editing
  let sentKey = null;                     // draft as it was when last sent, to tell if it changed since
  const view = { n: 40, e: 30, scale: 3, follow: true };
  let drag = null, track = [], trackRouteId = null;

  const key = (wps) => JSON.stringify(wps.map((w) => [+w.n.toFixed(2), +w.e.toFixed(2), w.margin]));
  const margin = () => +$("fwr-margin").value, arrive = () => +$("fwr-arrive").value, warn = () => +$("fwr-warn").value;

  // ---- what to draw -------------------------------------------------------------------------------------
  // The route the firmware has if the draft has not changed since it was sent, otherwise the draft as a preview.
  function shown() {
    const fw = S && S.fw, r = fw && fw.route;
    if (r && sentKey !== null && key(draft) === sentKey) {
      return { wps: r.waypoints, arrive: r.arrive_m, warn: r.warning_m, start: r.leg0_start || r.sent_from, sent: true, route: r };
    }
    const start = S ? { n: S.north, e: S.east } : { n: 0, e: 0 };
    return { wps: draft, arrive: arrive(), warn: warn(), start, sent: false, route: null };
  }

  function toPx(n, e, W, H) { return [W / 2 + (e - view.e) * view.scale, H / 2 - (n - view.n) * view.scale]; }
  function toM(x, y, W, H) { return { e: view.e + (x - W / 2) / view.scale, n: view.n - (y - H / 2) / view.scale }; }

  function band(ctx, P, a, b, off) {
    const [ax, ay] = P(a), [bx, by] = P(b);
    const len = Math.hypot(bx - ax, by - ay) || 1, nx = -(by - ay) / len, ny = (bx - ax) / len, w = off * view.scale;
    return [[ax + nx * w, ay + ny * w], [bx + nx * w, by + ny * w], [bx - nx * w, by - ny * w], [ax - nx * w, ay - ny * w]];
  }
  function poly(ctx, pts) { ctx.beginPath(); pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y))); ctx.closePath(); }
  function side(ctx, pts, i, j) { ctx.beginPath(); ctx.moveTo(...pts[i]); ctx.lineTo(...pts[j]); ctx.stroke(); }

  function draw() {
    const c = $("fwr-map"), r = c.getBoundingClientRect(), d = devicePixelRatio || 1;
    if (!r.width) return;
    if (c.width !== Math.round(r.width * d) || c.height !== Math.round(r.height * d)) { c.width = Math.round(r.width * d); c.height = Math.round(r.height * d); }
    const ctx = c.getContext("2d"); ctx.setTransform(d, 0, 0, d, 0, 0);
    const W = r.width, H = r.height, P = (p) => toPx(p.n, p.e, W, H);
    if (view.follow && S && !drag) { view.n = S.north; view.e = S.east; }

    ctx.fillStyle = "#0e3a5a"; ctx.fillRect(0, 0, W, H);
    const m0 = toM(0, H, W, H), m1 = toM(W, 0, W, H);
    for (let e = Math.floor(m0.e / 10) * 10; e <= m1.e; e += 10) {
      const [x] = toPx(0, e, W, H); ctx.strokeStyle = e % 50 === 0 ? "rgba(255,255,255,.16)" : "rgba(255,255,255,.06)";
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
    }
    for (let n = Math.floor(m0.n / 10) * 10; n <= m1.n; n += 10) {
      const [, y] = toPx(n, 0, W, H); ctx.strokeStyle = n % 50 === 0 ? "rgba(255,255,255,.16)" : "rgba(255,255,255,.06)";
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    }

    const R = shown(), ltp = S && S.fw && S.fw.ltp;
    const legNow = R.sent && ltp ? ltp.leg : -1;
    const ev = S && S.fw && S.fw.event, done = R.sent && ((ltp && ltp.state === 5) || (ev && ev.why === "route finished"));
    // corridors, widest first so the inner bands stay visible
    R.wps.forEach((wp, i) => {
      const a = i === 0 ? R.start : R.wps[i - 1], m = wp.margin, cur = i === legNow && !done;
      const warnPts = band(ctx, P, a, wp, m + R.warn), corr = band(ctx, P, a, wp, m), trig = band(ctx, P, a, wp, m * NAV_TACK_XTE_FRACTION);
      ctx.fillStyle = cur ? "rgba(56,189,248,.20)" : "rgba(56,189,248,.08)"; poly(ctx, corr); ctx.fill();
      ctx.strokeStyle = cur ? "rgba(239,68,68,.9)" : "rgba(239,68,68,.45)"; ctx.lineWidth = 1.5; ctx.setLineDash([7, 5]);
      side(ctx, warnPts, 0, 1); side(ctx, warnPts, 2, 3);
      ctx.strokeStyle = cur ? "rgba(56,189,248,.9)" : "rgba(56,189,248,.45)"; ctx.setLineDash([]); side(ctx, corr, 0, 1); side(ctx, corr, 2, 3);
      ctx.strokeStyle = "rgba(253,224,71,.45)"; ctx.setLineDash([2, 4]); side(ctx, trig, 0, 1); side(ctx, trig, 2, 3);
      ctx.strokeStyle = cur ? "#e0f2fe" : "rgba(224,242,254,.5)"; ctx.setLineDash(i === 0 && !R.sent ? [4, 4] : []); ctx.lineWidth = cur ? 2 : 1.2;
      ctx.beginPath(); ctx.moveTo(...P(a)); ctx.lineTo(...P(wp)); ctx.stroke(); ctx.setLineDash([]);
    });
    // waypoints and arrival circles
    R.wps.forEach((wp, i) => {
      const [x, y] = P(wp), reached = R.sent && ((ltp && i < ltp.leg) || done);
      ctx.strokeStyle = "rgba(255,255,255,.5)"; ctx.setLineDash([3, 3]); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(x, y, Math.max(3, R.arrive * view.scale), 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = reached ? "#64748b" : i === legNow ? "#fde047" : "#fb923c";
      ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = "#0f172a"; ctx.font = "700 11px system-ui"; ctx.textAlign = "center"; ctx.fillText(String(i), x, y + 4);
    });
    if (R.wps.length && R.start) {
      const [x, y] = P(R.start);
      ctx.strokeStyle = "#e5e7eb"; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.stroke();
    }

    // track since the route was sent
    ctx.strokeStyle = "#fde047"; ctx.lineWidth = 2.5; ctx.beginPath();
    track.forEach((p, i) => { const [x, y] = P(p); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();

    if (S) {
      const [x, y] = P({ n: S.north, e: S.east });
      // the firmware's no-go zone, around the wind it has estimated
      if (ltp && ltp.state !== 0) {
        const RR = 70; ctx.fillStyle = "rgba(239,68,68,.13)"; ctx.beginPath(); ctx.moveTo(x, y);
        for (let a = ltp.twd - NAV_NO_GO_DEG; a <= ltp.twd + NAV_NO_GO_DEG; a += 3) ctx.lineTo(x + Math.sin(rad(a)) * RR, y - Math.cos(rad(a)) * RR);
        ctx.closePath(); ctx.fill();
      }
      if (ltp && ltp.target != null && ltp.enabled) {
        ctx.strokeStyle = "rgba(167,139,250,.95)"; ctx.lineWidth = 2; ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(x, y);
        ctx.lineTo(x + Math.sin(rad(ltp.target)) * 60, y - Math.cos(rad(ltp.target)) * 60); ctx.stroke(); ctx.setLineDash([]);
      }
      ctx.save(); ctx.translate(x, y); ctx.rotate(rad(S.heading));
      const L = Math.max(10, 1.0 * view.scale);
      ctx.fillStyle = "#f1f5f9"; ctx.strokeStyle = "#0f172a"; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(0, -L); ctx.lineTo(L * 0.35, L * 0.6); ctx.lineTo(-L * 0.35, L * 0.6); ctx.closePath(); ctx.fill(); ctx.stroke();
      ctx.restore();
      // true wind, top right
      const wx = W - 60, wy = 60, ws = 28;
      ctx.strokeStyle = "#e5e7eb"; ctx.fillStyle = "#e5e7eb"; ctx.lineWidth = 3;
      const tx = wx - Math.sin(rad(S.wind_dir)) * ws, ty = wy + Math.cos(rad(S.wind_dir)) * ws;
      const fx = wx + Math.sin(rad(S.wind_dir)) * ws, fy = wy - Math.cos(rad(S.wind_dir)) * ws;
      ctx.beginPath(); ctx.moveTo(fx, fy); ctx.lineTo(tx, ty); ctx.stroke();
      const an = Math.atan2(ty - fy, tx - fx);
      ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(tx - 10 * Math.cos(an - 0.4), ty - 10 * Math.sin(an - 0.4)); ctx.lineTo(tx - 10 * Math.cos(an + 0.4), ty - 10 * Math.sin(an + 0.4)); ctx.fill();
      ctx.font = "12px system-ui"; ctx.textAlign = "center";
      ctx.fillText(`true wind ${S.wind_dir.toFixed(0)}° ${S.wind_speed.toFixed(1)} m/s`, wx - 20, wy + 48);
    }

    ctx.fillStyle = "#94a3b8"; ctx.font = "11px system-ui"; ctx.textAlign = "left";
    ctx.fillText("click: add waypoint · drag a waypoint: move · right-click: delete · drag: pan · scroll: zoom · grid 10 m · north up", 10, H - 10);
    const bar = 20 * view.scale; ctx.strokeStyle = "#e5e7eb"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(10, H - 28); ctx.lineTo(10 + bar, H - 28); ctx.stroke(); ctx.fillText("20 m", 14, H - 33);
    ctx.fillStyle = R.sent ? "#86efac" : "#fbbf24"; ctx.font = "600 12px system-ui";
    ctx.fillText(R.wps.length ? (R.sent ? "showing the route the firmware has" : "draft, not sent: leg 0 would start at the boat") : "no waypoints", 10, 20);
  }

  // ---- editing ---------------------------------------------------------------------------------------------
  const map = $("fwr-map");
  function hit(px, py, W, H) {
    for (let i = draft.length - 1; i >= 0; i--) {
      const [x, y] = toPx(draft[i].n, draft[i].e, W, H);
      if (Math.hypot(px - x, py - y) <= 11) return i;
    }
    return -1;
  }
  map.addEventListener("pointerdown", (e) => {
    if (e.button !== 0) return;
    const r = map.getBoundingClientRect(), i = hit(e.clientX - r.left, e.clientY - r.top, r.width, r.height);
    drag = { x: e.clientX, y: e.clientY, n: view.n, e: view.e, moved: false, wp: i };
    map.setPointerCapture(e.pointerId);
  });
  map.addEventListener("pointermove", (e) => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    if (Math.hypot(dx, dy) > 4) drag.moved = true;
    if (!drag.moved) return;
    if (drag.wp >= 0) {
      const r = map.getBoundingClientRect(), p = toM(e.clientX - r.left, e.clientY - r.top, r.width, r.height);
      draft[drag.wp].n = p.n; draft[drag.wp].e = p.e; list();
    } else {
      view.follow = false; $("fwr-follow").checked = false;
      view.e = drag.e - dx / view.scale; view.n = drag.n + dy / view.scale;
    }
    draw();
  });
  map.addEventListener("pointerup", (e) => {
    if (drag && !drag.moved && drag.wp < 0 && draft.length < MAX_WAYPOINTS) {
      const r = map.getBoundingClientRect(), p = toM(e.clientX - r.left, e.clientY - r.top, r.width, r.height);
      draft.push({ n: p.n, e: p.e, margin: margin() }); list();
    }
    drag = null; draw();
  });
  map.addEventListener("contextmenu", (e) => {
    e.preventDefault();
    const r = map.getBoundingClientRect(), i = hit(e.clientX - r.left, e.clientY - r.top, r.width, r.height);
    if (i >= 0) { draft.splice(i, 1); list(); draw(); }
  });
  map.addEventListener("wheel", (e) => { e.preventDefault(); view.scale = Math.min(30, Math.max(0.4, view.scale * Math.exp(-e.deltaY * 0.0015))); draw(); }, { passive: false });

  function list() {
    const el = $("fwr-list");
    el.innerHTML = draft.length ? draft.map((w, i) =>
      `<div class="fwr-wp"><b>${i}</b><span>${w.n.toFixed(0)} N, ${w.e.toFixed(0)} E</span>` +
      `<label>± <input type="number" min="1" max="255" value="${w.margin}" data-i="${i}"> m</label>` +
      `<button data-del="${i}" title="delete">✕</button></div>`).join("") : `<p class="hint">Click the map to add the first waypoint.</p>`;
    el.querySelectorAll("input[data-i]").forEach((inp) => inp.addEventListener("change", () => {
      draft[+inp.dataset.i].margin = Math.max(1, Math.min(255, Math.round(+inp.value || 1))); list(); draw();
    }));
    el.querySelectorAll("button[data-del]").forEach((b) => (b.onclick = () => { draft.splice(+b.dataset.del, 1); list(); draw(); }));
    $("fwr-count").textContent = `${draft.length} / ${MAX_WAYPOINTS}`;
  }

  for (const [id, unit] of [["fwr-margin", " m"], ["fwr-arrive", " m"], ["fwr-warn", " m"]]) {
    $(id).addEventListener("input", () => { $(`${id}-v`).textContent = `${$(id).value}${unit}`; draw(); });
  }
  $("fwr-margin-all").onclick = () => { draft.forEach((w) => (w.margin = margin())); list(); draw(); };
  $("fwr-clear").onclick = () => { draft = []; list(); draw(); };
  $("fwr-box").onclick = () => {
    // NAV_TEST_ROUTE's 80 m x 60 m box around the boat: one leg in each direction, so one is always upwind
    if (!S) return;
    const m = margin();
    draft = [[80, 0], [80, 60], [0, 60], [0, 0]].map(([n, e]) => ({ n: S.north + n, e: S.east + e, margin: m }));
    list(); draw();
  };
  $("fwr-follow").onchange = () => { view.follow = $("fwr-follow").checked; draw(); };

  async function send(auto) {
    if (!draft.length) { msg("Add waypoints first.", true); return; }
    const r = await post("/api/fw/route", { waypoints: draft, arrive_m: arrive(), warning_m: warn(), auto });
    if (r.error) { msg(r.error, true); return; }
    sentKey = key(draft); track = [];
    msg(`Sent ${r.commands} commands (route CRC 0x${r.crc.toString(16).toUpperCase().padStart(4, "0")})${auto ? ", then AUTO" : ""}.`);
  }
  async function mode(m) {
    const r = await post("/api/fw/mode", { mode: m });
    msg(r.error ? r.error : `Sent mode ${m.toUpperCase()}.`, !!r.error);
  }
  $("fwr-send-auto").onclick = () => send(true);
  $("fwr-send").onclick = () => send(false);
  $("fwr-auto").onclick = () => mode("auto");
  $("fwr-safe").onclick = () => mode("safe");
  function msg(text, bad) { const el = $("fwr-msg"); el.textContent = text; el.style.color = bad ? "var(--bad)" : ""; }

  // ---- status ----------------------------------------------------------------------------------------------
  function status() {
    const fw = S && S.fw, r = fw && fw.route, ltp = fw && fw.ltp;
    let route = "none sent";
    if (r) {
      route = r.commit_result == null ? "sent, waiting for the commit result"
        : r.commit_result === RESULT_OK ? (r.version != null ? `accepted, version ${r.version}` : "committed, waiting for LTP")
        : `rejected: ${(fw.results.find((x) => x.seq === r.commit_seq) || {}).result_name || r.commit_result}`;
      if (sentKey !== null && key(draft) !== sentKey) route += " (map edited since)";
    }
    const R = shown(), wp = ltp && R.sent && R.wps[ltp.leg];
    const dist = wp && S ? Math.hypot(wp.n - S.north, wp.e - S.east) : null;
    const v = ltp && ltp.valid;
    $("fwr-status").innerHTML = [
      ["firmware", !S ? "–" : S.mode === "firmware" ? "running" : S.renode === "stopped" ? "not running" : "starting"],
      ["Commander mode", fw && fw.mode_name ? fw.mode_name : "AUTO at boot (no reply yet)"],
      ["route", route],
      ["LTP state", ltp ? `${ltp.state_name}${ltp.enabled ? "" : " (rudder not in AUTO)"}` : "no LTP output yet"],
      ["LTP asked for SAFE", fw && fw.event ? `t ${fw.event.t} s: ${fw.event.why}` : "–"],
      ["leg", ltp && R.sent ? `${ltp.leg} of ${R.wps.length}` : "–"],
      ["cross-track (firmware)", ltp ? `${ltp.xte} m${wp ? ` of ±${wp.margin}` : ""}` : "–"],
      ["to the waypoint (true)", dist != null ? `${dist.toFixed(0)} m` : "–"],
      ["heading → target", ltp ? `${ltp.heading}° → ${ltp.target == null ? "none" : ltp.target + "°"}` : "–"],
      ["wind: true / firmware", S && ltp ? `${S.wind_dir.toFixed(0)}° / ${ltp.twd}°` : "–"],
      ["inputs gps / imu / wind", v ? `${v.gps} / ${v.imu} / ${v.wind}` : "–"],
    ].map(([k, val]) => `<span>${k}</span><b>${val}</b>`).join("");
    $("fwr-results").innerHTML = fw && fw.results.length
      ? fw.results.slice(0, 6).map((x) => `<div class="${x.result === RESULT_OK ? "" : "bad"}">t ${x.t}s · seq ${x.seq} · ${x.what} → ${x.result_name}</div>`).join("")
      : `<div class="hint">No command results yet.</div>`;
  }

  function update(state) {
    S = state;
    const r = S.fw && S.fw.route;
    // A page opened (or reloaded) while the firmware already has a route starts from that route
    if (r && !draft.length && sentKey === null) { draft = r.waypoints.map((w) => ({ ...w })); sentKey = key(draft); list(); }
    if (r && r.id !== trackRouteId) { trackRouteId = r.id; track = []; }
    if (r) {
      const last = track[track.length - 1];
      if (!last || Math.hypot(last.n - S.north, last.e - S.east) > 0.4) track.push({ n: S.north, e: S.east });
      if (track.length > 6000) track.shift();
    }
    if (!active) return;
    status(); draw();
  }

  list();
  return {
    show() { active = true; list(); if (S) status(); draw(); },
    hide() { active = false; },
    update,
  };
}
