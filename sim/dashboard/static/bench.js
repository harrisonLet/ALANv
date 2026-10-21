// Bench tab: the boat on its stand with a fan. Shows how the tail sets the wing, lets you test each servo,
// and compare wind-vane positions, without water.
const $ = (id) => document.getElementById(id);
const rad = (d) => (d * Math.PI) / 180;
const wrap180 = (d) => ((((d + 180) % 360) + 360) % 360) - 180;

export function initBench({ cmd, post, enter, leave, consoleLines }) {
  let S = null, active = false, dragging = false;
  // tell the server this page is still showing the bench (it leaves bench mode if the page goes away)
  setInterval(() => { if (active) post("/api/bench/ping"); }, 1000);
  let fanDir = 60, fanSpeed = 4;

  const setFan = () => cmd(`fan ${fanDir.toFixed(0)} ${fanSpeed.toFixed(1)}`);

  // ---- top-view diagram: boat, wing, tail, fan; drag around the ring to move the fan ---------------
  function dial() {
    const c = $("bench-dial");
    const r = c.getBoundingClientRect(), d = devicePixelRatio || 1;
    if (c.width !== Math.round(r.width * d)) { c.width = Math.round(r.width * d); c.height = Math.round(r.height * d); }
    const ctx = c.getContext("2d");
    ctx.setTransform(d, 0, 0, d, 0, 0);
    const W = r.width, H = r.height, cx = W / 2, cy = H / 2, R = Math.min(W, H) / 2 - 26;
    ctx.clearRect(0, 0, W, H);
    // ring with angle marks (relative to the bow, clockwise)
    ctx.strokeStyle = "#334155"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = "#64748b"; ctx.font = "11px system-ui"; ctx.textAlign = "center";
    for (let a = 0; a < 360; a += 30) {
      const x = cx + Math.sin(rad(a)) * (R + 13), y = cy - Math.cos(rad(a)) * (R + 13);
      ctx.fillText(`${a}°`, x, y + 4);
      ctx.beginPath(); ctx.moveTo(cx + Math.sin(rad(a)) * (R - 5), cy - Math.cos(rad(a)) * (R - 5));
      ctx.lineTo(cx + Math.sin(rad(a)) * R, cy - Math.cos(rad(a)) * R); ctx.stroke();
    }
    // the hard-stop arc: where the wing can point
    const lim = S ? S.wing_limit : 90;
    if (lim < 180) {
      ctx.strokeStyle = "rgba(245,158,11,.35)"; ctx.lineWidth = 6;
      ctx.beginPath(); ctx.arc(cx, cy - R * 0.28, R * 0.42, -Math.PI / 2 - rad(lim), -Math.PI / 2 + rad(lim)); ctx.stroke();
    }
    // boat (bow up)
    const L = R * 1.15, B = L * 0.31;
    ctx.save(); ctx.translate(cx, cy);
    ctx.fillStyle = "#e9e4d4"; ctx.strokeStyle = "#0f172a"; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, -L / 2); ctx.quadraticCurveTo(B * 0.75, -L * 0.25, B / 2, L * 0.42); ctx.lineTo(B * 0.32, L / 2);
    ctx.lineTo(-B * 0.32, L / 2); ctx.lineTo(-B / 2, L * 0.42); ctx.quadraticCurveTo(-B * 0.75, -L * 0.25, 0, -L / 2);
    ctx.fill(); ctx.stroke();
    // rudder at the stern
    ctx.save(); ctx.translate(0, L / 2); ctx.rotate(rad(-(S?.rudder ?? 0)));
    ctx.strokeStyle = "#ff5a1f"; ctx.lineWidth = 4; ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(0, L * 0.08); ctx.stroke(); ctx.restore();
    // wing about the mast (14% of the length ahead of centre), chord from the leading edge aft
    const mastY = -0.14 * L, chord = 0.35 * L;
    const drawWing = (angle, tail, ghost) => {
      ctx.save(); ctx.translate(0, mastY); ctx.rotate(rad(angle));
      ctx.globalAlpha = ghost ? 0.45 : 1;
      if (ghost) ctx.setLineDash([5, 4]);
      ctx.strokeStyle = ghost ? "#a78bfa" : "#b9a3b5"; ctx.lineWidth = ghost ? 2 : 7; ctx.lineCap = "round";
      ctx.beginPath(); ctx.moveTo(0, -chord * 0.4); ctx.lineTo(0, chord * 0.6); ctx.stroke();
      if (!ghost) {
        ctx.strokeStyle = "#16161a"; ctx.lineWidth = 1.6;
        ctx.beginPath(); ctx.moveTo(0, -chord * 0.4); ctx.lineTo(0, -chord * 0.85); ctx.stroke();   // counterweight rod
        ctx.fillStyle = "#55585e"; ctx.fillRect(-5, -chord * 0.95, 10, 12);
        ctx.beginPath(); ctx.moveTo(0, chord * 0.6); ctx.lineTo(0, chord * 1.2); ctx.stroke();      // tail boom
        ctx.translate(0, chord * 1.2); ctx.rotate(rad(tail));
        ctx.strokeStyle = "#f97316"; ctx.lineWidth = 6; ctx.beginPath(); ctx.moveTo(0, -chord * 0.12); ctx.lineTo(0, chord * 0.42); ctx.stroke();
      }
      ctx.restore();
    };
    if (S && S.hold_wing != null) drawWing(S.hold_wing, 0, true);
    drawWing(S?.wing ?? 0, S?.tail ?? 0, false);
    ctx.fillStyle = "#0f172a"; ctx.beginPath(); ctx.arc(0, mastY, 4, 0, Math.PI * 2); ctx.fill();
    ctx.restore();
    // fan on the ring and the airflow toward the boat
    const fx = cx + Math.sin(rad(fanDir)) * R, fy = cy - Math.cos(rad(fanDir)) * R;
    ctx.strokeStyle = "rgba(56,189,248,.5)"; ctx.lineWidth = 1.5;
    for (const off of [-14, 0, 14]) {
      const px = -Math.cos(rad(fanDir)) * off, py = -Math.sin(rad(fanDir)) * off;
      ctx.beginPath(); ctx.moveTo(fx + px, fy + py); ctx.lineTo(cx + px + (fx - cx) * 0.35, cy + py + (fy - cy) * 0.35); ctx.stroke();
    }
    ctx.save(); ctx.translate(fx, fy); ctx.rotate(rad(fanDir));
    ctx.fillStyle = "#e8e8e8"; ctx.strokeStyle = "#0f172a"; ctx.fillRect(-16, -6, 32, 12); ctx.strokeRect(-16, -6, 32, 12);
    ctx.restore();
    ctx.fillStyle = "#e5e7eb"; ctx.font = "600 12px system-ui"; ctx.textAlign = "left";
    ctx.fillText(`fan ${fanDir.toFixed(0)}° off the bow · ${fanSpeed.toFixed(1)} m/s`, 8, 16);
    if (S && S.wing_on_stop) { ctx.fillStyle = "#f59e0b"; ctx.fillText(`wing on the ${S.wing_on_stop > 0 ? "starboard" : "port"} hard stop`, 8, 32); }
  }

  function fanFromMouse(e) {
    const c = $("bench-dial"), r = c.getBoundingClientRect();
    const x = e.clientX - r.left - r.width / 2, y = e.clientY - r.top - r.height / 2;
    fanDir = (Math.atan2(x, -y) * 180 / Math.PI + 360) % 360;
    $("bench-fan-dir").value = fanDir.toFixed(0);
    $("bench-fan-dir-v").textContent = `${fanDir.toFixed(0)}°`;
    setFan(); dial();
  }
  $("bench-dial").addEventListener("pointerdown", (e) => { dragging = true; $("bench-dial").setPointerCapture(e.pointerId); fanFromMouse(e); });
  $("bench-dial").addEventListener("pointermove", (e) => { if (dragging) fanFromMouse(e); });
  $("bench-dial").addEventListener("pointerup", () => { dragging = false; });

  // ---- controls -------------------------------------------------------------------------------------------
  const bind = (id, fmt, fn) => {
    $(id).addEventListener("input", () => { $(`${id}-v`).textContent = fmt($(id).value); fn(+$(id).value); });
  };
  bind("bench-fan-dir", (v) => `${v}°`, (v) => { fanDir = v; setFan(); dial(); });
  bind("bench-fan", (v) => `${(+v).toFixed(1)} m/s`, (v) => { fanSpeed = v; setFan(); });
  bind("bench-wing", (v) => `${v}°`, (v) => { if ($("bench-hold").classList.contains("on")) cmd(`holdwing ${v}`); });
  $("bench-hold").onclick = () => {
    const on = !$("bench-hold").classList.contains("on");
    cmd(on ? `holdwing ${$("bench-wing").value}` : "holdwing off");
  };
  bind("bench-limit", (v) => (+v >= 180 ? "none (360°)" : `±${v}° (${2 * v}° swing)`), (v) => cmd(`winglimit ${v}`));

  // firmware console: type into USART3
  const fwSay = (text) => post("/api/renode/uart", { text });
  const fwSend = () => { const v = $("bench-fwcmd").value.trim(); if (v) fwSay(v); $("bench-fwcmd").value = ""; };
  $("bench-fwsend").onclick = fwSend;
  $("bench-fwcmd").addEventListener("keydown", (e) => { if (e.key === "Enter") fwSend(); });

  // servo tests: either override the firmware and drive the servos directly, or send console commands to it
  const viaFw = () => $("bench-viafw").checked && S && S.mode === "firmware";
  let lastTail = null, lastRudder = null;
  const servo = () => {
    const t = +$("bench-tail").value, r = +$("bench-rudder").value;
    if (viaFw()) {
      cmd("override off");
      if (t !== lastTail) fwSay(`tail ${t}`);
      if (r !== lastRudder) fwSay(`rudder ${r}`);
      lastTail = t; lastRudder = r;
    } else {
      cmd("override on");
      cmd(`servo ${t} ${r}`);
    }
  };
  bind("bench-tail", (v) => `${v}°`, () => { cmd("holdwing off"); servo(); });
  bind("bench-rudder", (v) => `${v}°`, () => servo());
  $("bench-center").onclick = () => { $("bench-tail").value = 0; $("bench-rudder").value = 0; $("bench-tail-v").textContent = "0°"; $("bench-rudder-v").textContent = "0°"; servo(); };
  $("bench-release").onclick = () => cmd("override off");
  let sweepTimer = null;
  function sweep(id, lo, hi) {
    clearInterval(sweepTimer);
    cmd("holdwing off");
    if (viaFw()) { fwSay(id === "bench-tail" ? "sweep tail" : "sweep rudder"); return; }   // the firmware sweeps it
    cmd("override on");
    const seq = [0, lo, hi, 0];
    let i = 0, v = 0;
    sweepTimer = setInterval(() => {
      const target = seq[i], step = Math.sign(target - v) * 3;
      v = Math.abs(target - v) <= 3 ? target : v + step;
      $(id).value = v; $(`${id}-v`).textContent = `${v}°`; servo();
      if (v === target) i++;
      if (i >= seq.length) clearInterval(sweepTimer);
    }, 60);
  }
  $("bench-sweep-tail").onclick = () => sweep("bench-tail", -90, 90);
  $("bench-sweep-rudder").onclick = () => sweep("bench-rudder", -35, 35);

  // wind vane position
  document.querySelectorAll("[data-bvane]").forEach((b) => { b.onclick = () => cmd(`vanemount ${b.dataset.bvane}`); });
  const vanePos = () => cmd(`vanepos ${$("bench-vx").value} ${$("bench-vz").value}`);
  bind("bench-vx", (v) => `${(+v).toFixed(2)} m ${+v >= 0 ? "forward" : "aft"}`, vanePos);
  bind("bench-vz", (v) => `${(+v).toFixed(2)} m up`, vanePos);

  // ---- readouts ---------------------------------------------------------------------------------------------
  function update(state) {
    S = state;
    if (!active) return;
    dial();
    const t = S.trim_calc || {};
    const hold = S.hold_wing != null;
    $("bench-hold").classList.toggle("on", hold);
    $("bench-hold").textContent = hold ? "Holding the sail angle (sim helper)" : "Hold this sail angle";
    const want = hold ? S.hold_wing : +$("bench-wing").value;
    const alpha = wrap180(S.awa - want), tail = alpha / (t.gain || 0.5);
    const ok = Math.abs(tail) <= (t.limit || 90) && Math.abs(want) <= S.wing_limit;
    $("bench-math").innerHTML =
      `<div>Apparent wind at the wing: <b>${S.awa.toFixed(1)}°</b> off the bow</div>` +
      `<div>Wanted sail (wing) angle: <b>${want.toFixed(0)}°</b> to the hull</div>` +
      `<div>Angle of attack α = wind − sail = ${S.awa.toFixed(1)} − ${want.toFixed(0)} = <b>${alpha.toFixed(1)}°</b></div>` +
      `<div>Tail = α ÷ ${(t.gain || 0.5).toFixed(2)} (tail-to-AoA ratio) = <b>${tail.toFixed(1)}°</b></div>` +
      (ok ? `<div class="ok">Reachable: the tail can do this (limit ±${t.limit || 90}°).</div>`
          : `<div class="bad">Not reachable: ${Math.abs(want) > S.wing_limit ? `the hard stops stop the wing at ±${S.wing_limit}°` : `needs ${tail.toFixed(0)}° of tail, beyond ±${t.limit || 90}°`}.</div>`) +
      `<div class="now">Now: wing ${S.wing.toFixed(1)}°, tail ${S.tail.toFixed(1)}°, α ${S.aoa.toFixed(1)}°${S.wing_on_stop ? " (on a hard stop)" : ""}</div>`;
    const err = wrap180(S.vane - S.awa);
    $("bench-vane").innerHTML =
      `<span>mount</span><b>${S.vane_mount}${S.vane_mount === "custom" ? ` (${S.vane_custom[0].toFixed(2)}, ${S.vane_custom[1].toFixed(2)} m)` : ""}</b>` +
      `<span>true apparent wind (at the wing)</span><b>${S.awa.toFixed(1)}°</b>` +
      `<span>airflow at the vane</span><b>${S.vane_flow.toFixed(1)}°</b>` +
      `<span>vane reads</span><b>${S.vane.toFixed(1)}°</b>` +
      `<span>error</span><b>${err >= 0 ? "+" : ""}${err.toFixed(1)}°</b>`;
    document.querySelectorAll("[data-bvane]").forEach((b) => b.classList.toggle("active", b.dataset.bvane === S.vane_mount));
    const out = (consoleLines() || []).filter((l) => l.includes("[CONSOLE]") || l.includes("Setting Servo")).slice(-8);
    $("bench-fwout").textContent = S.mode === "firmware"
      ? (out.join("\n") || "(waiting for the firmware's console…)")
      : "Firmware not running: click ▶ Run my firmware in the header.";
    $("bench-fwsend").disabled = S.mode !== "firmware";
    $("bench-who").textContent = S.mode === "firmware"
      ? (S.override ? "Firmware running; your sliders are driving the servos." : "Your firmware is driving the servos (move a slider to take over).")
      : (S.override || hold ? "Sim controls are driving the servos." : "No firmware running: the sliders drive the servos.");
  }

  return {
    show() {
      active = true;
      enter($("bench-view"));
      cmd("bench on");
      setFan();
      dial();
    },
    async hide() {
      if (!active) return;
      active = false;
      clearInterval(sweepTimer);
      await cmd("holdwing off");
      await cmd("bench off");
      leave();
    },
    update,
  };
}
