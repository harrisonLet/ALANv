// map.js - satellite map, route planner and boat commands for gs_web.py
//
// The map is Web Mercator (the tiles' projection) drawn on a canvas. Tiles come from /tiles/z/x/y,
// which gs_web.py serves from its cache. Missing tiles fall back to coarser cached ones, then to a grid.
//
// A route is a list of waypoints, each with a margin: the corridor half-width LTP keeps the boat in on the
// leg that ends at that waypoint. The first leg starts wherever the boat is when AUTO picks the route up.
(() => {
  const $ = id => document.getElementById(id);
  const T = 256, TILE_MAX_Z = 19, MAX_Z = 21, MIN_Z = 2;
  const R = 6378137, M_PER_DEG = 111320;
  const COL = {
    boat: '#e3b341', desired: '#bc8cff', track: 'rgba(227,179,65,.85)',
    draft: 'rgba(188,140,255,.28)', draftLine: '#bc8cff',
    active: 'rgba(63,185,80,.22)', activeLine: '#3fb950', activeLeg: 'rgba(63,185,80,.38)',
    warn: 'rgba(210,153,34,.14)', warnLine: 'rgba(210,153,34,.6)', text: '#e6edf3',
  };

  const store = {
    get(k, d) { try { const v = localStorage.getItem(k); return v ? JSON.parse(v) : d; } catch (e) { return d; } },
    set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) { /* private window */ } },
  };

  const canvas = $('map');
  const ctx = canvas.getContext('2d');
  let W = 0, H = 0;
  let view = store.get('alanv.view', { lat: 0, lon: 0, z: 3 });
  let follow = store.get('alanv.follow', true);
  let draft = store.get('alanv.draft', { wps: [], margin: 20, arrive: 8, warning: 10 });
  let telem = null, history = [], cmds = null, modes = {}, headingModes = [], maxWps = 32;
  let selected = -1, centeredOnce = false;
  const tiles = new Map();

  // ── projection ──────────────────────────────────────────────────────────
  const world = (lat, lon, z) => {
    const n = T * 2 ** z, s = Math.sin(lat * Math.PI / 180);
    return [(lon + 180) / 360 * n, (0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * n];
  };
  const unworld = (x, y, z) => {
    const n = T * 2 ** z;
    return { lat: Math.atan(Math.sinh(Math.PI * (1 - 2 * y / n))) * 180 / Math.PI, lon: x / n * 360 - 180 };
  };
  const toScreen = (lat, lon) => {
    const [x, y] = world(lat, lon, view.z), [cx, cy] = world(view.lat, view.lon, view.z);
    return [W / 2 + x - cx, H / 2 + y - cy];
  };
  const fromScreen = (sx, sy) => {
    const [cx, cy] = world(view.lat, view.lon, view.z);
    return unworld(cx + sx - W / 2, cy + sy - H / 2, view.z);
  };
  const mpp = () => 2 * Math.PI * R * Math.cos(view.lat * Math.PI / 180) / (T * 2 ** view.z);   // metres per pixel
  const distM = (a, b) => {
    const dn = (b.lat - a.lat) * M_PER_DEG, de = (b.lon - a.lon) * M_PER_DEG * Math.cos(a.lat * Math.PI / 180);
    return Math.hypot(dn, de);
  };
  const bearing = (a, b) => {
    const dn = (b.lat - a.lat) * M_PER_DEG, de = (b.lon - a.lon) * M_PER_DEG * Math.cos(a.lat * Math.PI / 180);
    return (Math.atan2(de, dn) * 180 / Math.PI + 360) % 360;
  };

  // ── tiles ───────────────────────────────────────────────────────────────
  function tile(z, x, y) {
    const key = `${z}/${x}/${y}`;
    let t = tiles.get(key);
    if (!t) {
      t = { img: new Image(), ok: false, failed: false };
      t.img.onload = () => { t.ok = true; draw(); };
      t.img.onerror = () => { t.failed = true; };
      t.img.src = '/tiles/' + key;
      tiles.set(key, t);
    }
    return t;
  }

  function drawTiles() {
    ctx.fillStyle = '#0f2a3f';
    ctx.fillRect(0, 0, W, H);
    const tz = Math.max(0, Math.min(TILE_MAX_Z, Math.round(view.z)));
    let any = false;
    // coarse layers first, so a gap in the cache still shows something
    for (const z of [tz - 3, tz - 2, tz - 1, tz]) {
      if (z < 0) continue;
      const scale = 2 ** (view.z - z), size = T * scale, n = 2 ** z;
      const [cx, cy] = world(view.lat, view.lon, z);
      const x0 = Math.floor((cx - W / 2 / scale) / T), x1 = Math.floor((cx + W / 2 / scale) / T);
      const y0 = Math.max(0, Math.floor((cy - H / 2 / scale) / T)), y1 = Math.min(n - 1, Math.floor((cy + H / 2 / scale) / T));
      if ((x1 - x0 + 1) * (y1 - y0 + 1) > 120) continue;
      for (let x = x0; x <= x1; x++) {
        for (let y = y0; y <= y1; y++) {
          const t = tile(z, ((x % n) + n) % n, y);
          if (!t.ok) continue;
          any = true;
          ctx.drawImage(t.img, Math.floor(W / 2 + (x * T - cx) * scale), Math.floor(H / 2 + (y * T - cy) * scale),
                        Math.ceil(size) + 1, Math.ceil(size) + 1);
        }
      }
    }
    if (!any) drawGrid();
  }

  function drawGrid() {
    const m = mpp();
    const step = [5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 50000, 100000].find(s => s / m > 70) || 100000;
    const p = step / m;
    const [ox, oy] = toScreen(Math.round(view.lat * 1000) / 1000, Math.round(view.lon * 1000) / 1000);
    ctx.strokeStyle = 'rgba(139,148,158,.13)';
    ctx.lineWidth = 1;
    for (let x = ((ox % p) + p) % p; x < W; x += p) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke(); }
    for (let y = ((oy % p) + p) % p; y < H; y += p) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke(); }
    ctx.fillStyle = 'rgba(139,148,158,.6)';
    ctx.font = '12px system-ui';
    ctx.fillText('no map tiles cached here (python3 gs_web.py --prefetch LAT,LON)', 12, 22);
  }

  // ── route drawing ───────────────────────────────────────────────────────
  function routePoints(wps, start) {
    return (start ? [start] : []).concat(wps);
  }

  function band(a, b, halfWidthM, style) {
    const [ax, ay] = toScreen(a.lat, a.lon), [bx, by] = toScreen(b.lat, b.lon);
    ctx.strokeStyle = style;
    ctx.lineWidth = Math.max(2, 2 * halfWidthM / mpp());
    ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
  }

  function drawRoute(r, kind, currentLeg) {
    if (!r || !r.wps.length) return;
    const start = r.start;
    const pts = routePoints(r.wps, start);
    const off = start ? 1 : 0;
    // warning zone, then corridor, then centre line
    for (let i = 1; i < pts.length; i++) {
      const wp = r.wps[i - off];
      if (r.warning > 0) band(pts[i - 1], pts[i], wp.margin + r.warning, COL.warn);
    }
    for (let i = 1; i < pts.length; i++) {
      const wp = r.wps[i - off];
      const cur = kind === 'active' && i - off === currentLeg;
      band(pts[i - 1], pts[i], wp.margin, kind === 'active' ? (cur ? COL.activeLeg : COL.active) : COL.draft);
    }
    ctx.lineWidth = 2;
    ctx.lineCap = 'butt';
    ctx.strokeStyle = kind === 'active' ? COL.activeLine : COL.draftLine;
    for (let i = 1; i < pts.length; i++) {
      const [ax, ay] = toScreen(pts[i - 1].lat, pts[i - 1].lon), [bx, by] = toScreen(pts[i].lat, pts[i].lon);
      ctx.setLineDash(i === 1 && start ? [6, 5] : []);
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
    }
    ctx.setLineDash([]);
    // waypoints: arrival circle and number
    r.wps.forEach((w, i) => {
      const [x, y] = toScreen(w.lat, w.lon);
      const rad = Math.max(7, (r.arrive || 8) / mpp());
      ctx.lineWidth = 2;
      ctx.strokeStyle = kind === 'active' ? COL.activeLine : COL.draftLine;
      ctx.fillStyle = kind === 'draft' && i === selected ? 'rgba(188,140,255,.55)' : 'rgba(13,17,23,.55)';
      ctx.beginPath(); ctx.arc(x, y, rad, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      ctx.fillStyle = COL.text;
      ctx.font = 'bold 12px system-ui';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(String(i + 1), x, y);
      ctx.textAlign = 'start'; ctx.textBaseline = 'alphabetic';
    });
  }

  function drawBoat() {
    const pts = history.filter(p => p.gps_ok);
    if (pts.length > 1) {
      ctx.strokeStyle = COL.track; ctx.lineWidth = 2; ctx.beginPath();
      pts.forEach((p, i) => { const [x, y] = toScreen(p.latitude, p.longitude); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.stroke();
    }
    const b = boatPos();
    if (!b) return;
    const [x, y] = toScreen(b.lat, b.lon);
    if (telem.desired_heading !== 0xFFFF) {
      const a = telem.desired_heading * Math.PI / 180;
      ctx.strokeStyle = COL.desired; ctx.lineWidth = 2; ctx.setLineDash([4, 4]);
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.sin(a) * 60, y - Math.cos(a) * 60); ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(telem.heading * Math.PI / 180);
    ctx.fillStyle = COL.boat; ctx.strokeStyle = '#0d1117'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(0, -14); ctx.lineTo(8, 10); ctx.lineTo(0, 5); ctx.lineTo(-8, 10); ctx.closePath();
    ctx.fill(); ctx.stroke();
    ctx.restore();
  }

  function drawOverlay() {
    const m = mpp();
    const step = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000].find(s => s / m > 70) || 100000;
    const len = step / m;
    ctx.fillStyle = 'rgba(13,17,23,.7)';
    ctx.fillRect(8, H - 34, len + 20, 26);
    ctx.strokeStyle = COL.text; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(18, H - 14); ctx.lineTo(18 + len, H - 14); ctx.stroke();
    ctx.fillStyle = COL.text; ctx.font = '11px ui-monospace, monospace';
    ctx.fillText(step >= 1000 ? step / 1000 + ' km' : step + ' m', 18, H - 19);
    ctx.fillStyle = 'rgba(13,17,23,.7)';
    ctx.fillRect(W - 30, 8, 22, 22);
    ctx.fillStyle = COL.text; ctx.font = 'bold 12px system-ui'; ctx.textAlign = 'center';
    ctx.fillText('N', W - 19, 24);
    ctx.textAlign = 'start';
    ctx.fillStyle = 'rgba(230,237,243,.55)'; ctx.font = '10px system-ui';
    ctx.fillText('Imagery © Esri', W - 82, H - 8);
  }

  function draw() {
    if (!W) return;
    drawTiles();
    const act = activeRoute();
    drawRoute(act, 'active', telem ? telem.route_segment : -1);
    if (!sameAsActive()) drawRoute(draftRoute(), 'draft');
    drawBoat();
    drawOverlay();
  }

  // ── data ────────────────────────────────────────────────────────────────
  function boatPos() {
    if (!telem || !(0x01 <= telem.gps_valid && telem.gps_valid < 0x10) || !(telem.latitude || telem.longitude)) return null;
    return { lat: telem.latitude, lon: telem.longitude };
  }

  function activeRoute() {
    const a = cmds && cmds.route && cmds.route.active;
    if (!a) return null;
    return { wps: a.waypoints, arrive: a.arrive_m, warning: a.warning_m, start: a.start || null };
  }

  function draftRoute() {
    return { wps: draft.wps, arrive: draft.arrive, warning: draft.warning, start: boatPos() };
  }

  function sameAsActive() {
    const a = activeRoute();
    if (!a || a.wps.length !== draft.wps.length || a.arrive !== draft.arrive || a.warning !== draft.warning) return false;
    return a.wps.every((w, i) => Math.abs(w.lat - draft.wps[i].lat) < 1e-7 && Math.abs(w.lon - draft.wps[i].lon) < 1e-7
                                  && w.margin === draft.wps[i].margin);
  }

  function saveDraft() { store.set('alanv.draft', draft); renderRoute(); draw(); }
  function saveView() { store.set('alanv.view', view); }

  // ── interaction ─────────────────────────────────────────────────────────
  let drag = null;
  const hitWaypoint = (sx, sy) => {
    for (let i = draft.wps.length - 1; i >= 0; i--) {
      const [x, y] = toScreen(draft.wps[i].lat, draft.wps[i].lon);
      if (Math.hypot(sx - x, sy - y) <= Math.max(12, (draft.arrive || 8) / mpp())) return i;
    }
    return -1;
  };
  const pos = ev => { const r = canvas.getBoundingClientRect(); return [ev.clientX - r.left, ev.clientY - r.top]; };

  canvas.addEventListener('pointerdown', ev => {
    if (ev.button !== 0) return;
    const [x, y] = pos(ev);
    const i = hitWaypoint(x, y);
    drag = { x, y, moved: false, wp: i, start: { ...view } };
    try { canvas.setPointerCapture(ev.pointerId); } catch (e) { /* synthetic event */ }
    canvas.focus();
  });
  canvas.addEventListener('pointermove', ev => {
    if (!drag) {
      const [x, y] = pos(ev);
      canvas.style.cursor = hitWaypoint(x, y) >= 0 ? 'grab' : ($('map-add').checked ? 'crosshair' : 'move');
      return;
    }
    const [x, y] = pos(ev);
    if (Math.hypot(x - drag.x, y - drag.y) > 4) drag.moved = true;
    if (!drag.moved) return;
    if (drag.wp >= 0) {
      const p = fromScreen(x, y);
      draft.wps[drag.wp].lat = p.lat; draft.wps[drag.wp].lon = p.lon;
      selected = drag.wp;
      renderRoute(); draw();
    } else {
      const [cx, cy] = world(drag.start.lat, drag.start.lon, view.z);
      const c = unworld(cx - (x - drag.x), cy - (y - drag.y), view.z);
      view.lat = c.lat; view.lon = c.lon;
      setFollow(false);
      draw();
    }
  });
  canvas.addEventListener('pointerup', ev => {
    if (!drag) return;
    const [x, y] = pos(ev);
    if (drag.wp >= 0) {
      selected = drag.wp;
      if (drag.moved) saveDraft(); else { renderRoute(); draw(); }
    } else if (!drag.moved && $('map-add').checked) {
      if (draft.wps.length >= maxWps) { status(`at most ${maxWps} waypoints`, true); }
      else {
        const p = fromScreen(x, y);
        draft.wps.push({ lat: p.lat, lon: p.lon, margin: draft.margin });
        selected = draft.wps.length - 1;
        saveDraft();
      }
    } else if (drag.moved) {
      saveView();
    }
    drag = null;
  });
  canvas.addEventListener('contextmenu', ev => {
    const [x, y] = pos(ev);
    const i = hitWaypoint(x, y);
    if (i < 0) return;
    ev.preventDefault();
    removeWp(i);
  });
  canvas.addEventListener('keydown', ev => {
    if ((ev.key === 'Delete' || ev.key === 'Backspace') && selected >= 0) { ev.preventDefault(); removeWp(selected); }
  });
  canvas.addEventListener('wheel', ev => {
    ev.preventDefault();
    const [x, y] = pos(ev);
    const before = fromScreen(x, y);
    view.z = Math.max(MIN_Z, Math.min(MAX_Z, view.z - Math.sign(ev.deltaY) * 0.5));
    // keep the point under the cursor where it is
    const [px, py] = world(before.lat, before.lon, view.z);
    const c = unworld(px - (x - W / 2), py - (y - H / 2), view.z);
    view.lat = c.lat; view.lon = c.lon;
    if (follow && boatPos()) setFollow(false);
    saveView(); draw();
  }, { passive: false });

  function removeWp(i) {
    draft.wps.splice(i, 1);
    selected = Math.min(selected, draft.wps.length - 1);
    saveDraft();
  }

  function setFollow(on) {
    follow = on;
    $('map-follow').checked = on;
    store.set('alanv.follow', on);
  }

  function centerOn(lat, lon, z) {
    view.lat = lat; view.lon = lon;
    if (z) view.z = z;
    saveView(); draw();
  }

  function fit() {
    const pts = draft.wps.slice();
    const b = boatPos();
    if (b) pts.push(b);
    const a = activeRoute();
    if (a) pts.push(...a.wps);
    if (!pts.length) return;
    const lat0 = Math.min(...pts.map(p => p.lat)), lat1 = Math.max(...pts.map(p => p.lat));
    const lon0 = Math.min(...pts.map(p => p.lon)), lon1 = Math.max(...pts.map(p => p.lon));
    const span = Math.max(distM({ lat: lat0, lon: lon0 }, { lat: lat1, lon: lon0 }),
                          distM({ lat: lat0, lon: lon0 }, { lat: lat0, lon: lon1 }) * H / W, 60);
    const z = Math.log2(2 * Math.PI * R * Math.cos((lat0 + lat1) / 2 * Math.PI / 180) * H / (T * span * 1.5));
    setFollow(false);
    centerOn((lat0 + lat1) / 2, (lon0 + lon1) / 2, Math.max(MIN_Z, Math.min(MAX_Z, z)));
  }

  $('map-follow').checked = follow;
  $('map-follow').onchange = () => { setFollow($('map-follow').checked); const b = boatPos(); if (follow && b) centerOn(b.lat, b.lon); };
  $('map-center').onclick = () => { const b = boatPos(); if (b) { setFollow(true); centerOn(b.lat, b.lon, Math.max(view.z, 17)); } else status('no GPS fix from the boat yet', true); };
  $('map-fit').onclick = fit;
  $('map-locate').onclick = () => {
    if (!navigator.geolocation) return status('this browser has no location', true);
    navigator.geolocation.getCurrentPosition(p => { setFollow(false); centerOn(p.coords.latitude, p.coords.longitude, 17); },
                                             e => status('location: ' + e.message, true), { timeout: 8000 });
  };

  // ── route panel ─────────────────────────────────────────────────────────
  const num = (id, lo, hi) => { const v = Math.round(Number($(id).value)); return Number.isFinite(v) ? Math.max(lo, Math.min(hi, v)) : lo; };

  function renderRoute() {
    $('rt-margin').value = draft.margin;
    $('rt-arrive').value = draft.arrive;
    $('rt-warning').value = draft.warning;
    const b = boatPos();
    let prev = b, total = 0;
    $('rt-list').innerHTML = draft.wps.map((w, i) => {
      const d = prev ? distM(prev, w) : null;
      const brg = prev ? bearing(prev, w) : null;
      if (d !== null) total += d;
      prev = w;
      return `<div class="wp${i === selected ? ' sel' : ''}" data-i="${i}">
        <span class="n">${i + 1}</span>
        <span class="mono dim">${d === null ? '–' : Math.round(d) + ' m'} ${brg === null ? '' : Math.round(brg) + '°'}</span>
        <label>± <input type="number" min="1" max="255" value="${w.margin}" data-margin="${i}"> m</label>
        <button data-del="${i}" title="Remove waypoint">✕</button></div>`;
    }).join('') || '<div class="dim">Click the map to add waypoints. Drag to move one, right-click to remove.</div>';
    $('rt-total').textContent = draft.wps.length ? `${draft.wps.length} waypoints, ${Math.round(total)} m${b ? ' from the boat' : ''}` : '';
    $('rt-list').querySelectorAll('[data-margin]').forEach(inp => inp.onchange = () => {
      draft.wps[+inp.dataset.margin].margin = Math.max(1, Math.min(255, Math.round(+inp.value) || 1));
      saveDraft();
    });
    $('rt-list').querySelectorAll('[data-del]').forEach(btn => btn.onclick = () => removeWp(+btn.dataset.del));
    $('rt-list').querySelectorAll('.wp').forEach(row => row.onclick = ev => {
      if (ev.target.tagName === 'INPUT' || ev.target.tagName === 'BUTTON') return;
      selected = +row.dataset.i; renderRoute(); draw();
    });
    const same = sameAsActive();
    $('rt-state').textContent = !draft.wps.length ? '' : same ? 'this is the route the boat has' : 'not sent yet';
    $('rt-state').className = same ? 'ok' : 'warn';
  }

  $('rt-margin').onchange = () => { draft.margin = num('rt-margin', 1, 255); saveDraft(); };
  $('rt-arrive').onchange = () => { draft.arrive = num('rt-arrive', 1, 255); saveDraft(); };
  $('rt-warning').onchange = () => { draft.warning = num('rt-warning', 0, 255); saveDraft(); };
  $('rt-margin-all').onclick = () => { draft.margin = num('rt-margin', 1, 255); draft.wps.forEach(w => w.margin = draft.margin); saveDraft(); };
  $('rt-clear').onclick = () => { if (!draft.wps.length || confirm('Remove every waypoint from the plan?')) { draft.wps = []; selected = -1; saveDraft(); } };
  $('rt-load').onclick = () => {
    const a = cmds && cmds.route && cmds.route.active;
    if (!a) return status('the boat has no route from this session', true);
    draft = { wps: a.waypoints.map(w => ({ ...w })), margin: draft.margin, arrive: a.arrive_m, warning: a.warning_m };
    saveDraft();
  };
  $('rt-send').onclick = () => sendRoute(false);
  $('rt-send-auto').onclick = () => sendRoute(true);

  async function post(path, body) {
    try {
      const r = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body || {}) });
      const d = await r.json();
      if (!d.ok) status(d.error || 'failed', true);
      return d;
    } catch (e) { status('ground station not reachable', true); return { ok: false }; }
  }

  async function sendRoute(auto) {
    if (!draft.wps.length) return status('add waypoints first', true);
    if (auto && !confirm(`Send ${draft.wps.length} waypoints and switch the boat to AUTO?\nLeg 1 starts where the boat is now.`)) return;
    const d = await post('/route', { waypoints: draft.wps, arrive_m: draft.arrive, warning_m: draft.warning, auto });
    if (d.ok) status(`sending route: ${d.commands} commands, CRC ${d.crc.toString(16).toUpperCase().padStart(4, '0')}`);
  }

  function status(text, bad) { const el = $('rt-msg'); el.textContent = text; el.className = bad ? 'bad' : 'dim'; }

  // ── modes and heading ───────────────────────────────────────────────────
  const MODE_BUTTONS = [['STANDBY', 'Standby'], ['AUTO', 'Auto (route)'], ['SAFE', 'Safe'], ['FULL_MAN', 'Manual'],
                        ['SEMI_RUDDER_AUTO_SAIL', 'Hold heading']];
  $('md-btns').innerHTML = MODE_BUTTONS.map(([m, l]) => `<button data-mode="${m}">${l}</button>`).join('');
  $('md-btns').querySelectorAll('[data-mode]').forEach(b => b.onclick = () => {
    const m = b.dataset.mode;
    if (m === 'AUTO' && !(cmds && cmds.route && cmds.route.active) && !confirm('The boat has no route from this session. AUTO without a route holds the current heading. Switch anyway?')) return;
    post('/mode', { mode: m });
  });
  $('hd-send').onclick = () => {
    const v = Number($('hd-input').value);
    if (!Number.isFinite(v) || v < 0 || v > 359) return status('heading must be 0 to 359', true);
    if (telem && !headingModes.includes(telem.mode)) status('the boat only takes a heading in a semi rudder mode (Hold heading)', true);
    post('/heading', { deg: Math.round(v) });
  };
  $('q-cancel').onclick = () => post('/cancel');
  $('q-hb').onchange = () => post('/heartbeat', { on: $('q-hb').checked });

  // ── status panel ────────────────────────────────────────────────────────
  let lastNav = null;
  function renderBoat() {
    const t = telem;
    if (!t) return;
    const active = activeRoute();
    $('bt-mode').textContent = t.mode_name;
    $('bt-mode').className = 'mode-' + t.mode_name.toLowerCase();
    const inRoute = active && t.mode_name === 'AUTO';
    $('bt-leg').textContent = active ? `${Math.min(t.route_segment + 1, active.wps.length)} of ${active.wps.length}` : 'no route';
    $('bt-drift').textContent = inRoute ? `${t.drift_distance > 0 ? '+' : ''}${t.drift_distance} m ${t.drift_distance > 0 ? 'stbd' : t.drift_distance < 0 ? 'port' : ''}` : '–';
    if (inRoute) {
      const wp = active.wps[Math.min(t.route_segment, active.wps.length - 1)];
      const lim = wp.margin;
      $('bt-drift').className = Math.abs(t.drift_distance) > lim ? 'bad' : Math.abs(t.drift_distance) > 0.8 * lim ? 'warn' : '';
      const b = boatPos();
      $('bt-dist').textContent = b ? `${Math.round(distM(b, wp))} m, ${Math.round(bearing(b, wp))}°` : '–';
    } else {
      $('bt-drift').className = '';
      $('bt-dist').textContent = '–';
    }
    $('bt-dhdg').textContent = t.desired_heading === 0xFFFF ? 'none' : t.desired_heading + '°';
    if (t.nav_state_name !== undefined) {
      // LTP goes IDLE once the boat leaves AUTO, so keep the last state it had in AUTO: that is why it stopped
      if (t.mode_name === 'AUTO' && t.nav_state !== 0) lastNav = t.nav_state_name;
      let s = t.nav_state_name;
      if (t.nav_corridor) s += ', left the corridor';
      if (t.nav_tack_slow) s += ', slow tack';
      if (t.mode_name !== 'AUTO' && lastNav) s += ` (last in AUTO: ${lastNav})`;
      $('bt-nav').textContent = s;
      $('bt-nav').className = (t.nav_corridor || t.nav_state === 6) ? 'bad' : t.nav_tack_slow ? 'warn' : '';
    }
    $('bt-hdg').textContent = t.heading + '°';
    $('bt-last').textContent = `seq ${t.last_cmd_seq & 0xFF}: ${t.last_cmd_result_name}`;
    if ('rx_age_s' in t) {
      $('bt-rx').textContent = t.rx_age_s === 255 ? 'nothing yet' : `${t.rx_age_s} s ago, ${t.rx_last_rssi} dBm, SNR ${t.rx_last_snr} dB`;
    }
  }

  function renderCmds() {
    if (!cmds) return;
    const up = cmds.route && cmds.route.upload;
    $('rt-upload').innerHTML = up
      ? `<div class="bar"><i style="width:${Math.round(100 * up.done / up.total)}%"></i></div><span>${up.status} (${up.done}/${up.total})</span>`
      : '';
    $('q-list').innerHTML = cmds.log.slice(0, 14).map(c => {
      const cls = c.status === 'ok' ? 'ok' : c.status === 'failed' || c.status === 'gave up' ? 'bad' : c.status === 'cancelled' ? 'dim' : 'warn';
      const time = new Date(c.t * 1000).toLocaleTimeString([], { hour12: false });
      return `<div><span class="dim">${time}</span> ${c.label} <span class="dim">${c.seq === null ? '' : '#' + c.seq}${c.tries > 1 ? ' ×' + c.tries : ''}</span> <b class="${cls}">${c.result && c.status !== 'ok' ? c.result : c.status}</b></div>`;
    }).join('') || '<div class="dim">no commands sent</div>';
    $('q-count').textContent = cmds.queued ? `${cmds.queued} waiting` : '';
    renderRoute();
  }

  // ── hooks for index.html ────────────────────────────────────────────────
  function resize() {
    const r = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    W = r.width; H = r.height;
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    draw();
  }
  window.addEventListener('resize', resize);

  window.MAP = {
    snapshot(d) {
      modes = d.modes || {}; headingModes = d.heading_modes || []; maxWps = (d.limits && d.limits.waypoints) || 32;
      history = d.history || []; cmds = d.cmds || null; telem = d.telem;
      if (d.link) $('q-hb').checked = d.link.heartbeat;
      this.telem(d.telem);
      renderCmds();
    },
    telem(t, sample) {
      telem = t;
      if (sample) { history.push(sample); if (history.length > 600) history.shift(); }
      const b = boatPos();
      if (b && (follow || !centeredOnce)) {
        if (!centeredOnce && !follow && draft.wps.length) { centeredOnce = true; }
        else { centeredOnce = true; view.lat = b.lat; view.lon = b.lon; if (view.z < 15) view.z = 17; saveView(); }
      }
      renderBoat();
      draw();
    },
    cmds(c) { cmds = c; renderCmds(); renderBoat(); draw(); },
  };

  renderRoute();
  resize();
})();
