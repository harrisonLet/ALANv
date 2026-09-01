"""
ALAN Ground Station — run with: python app.py [port]
Opens http://localhost:5000. Connect button in the UI handles serial.
"""

import sys, glob, threading, queue, time
import serial
import serial.tools.list_ports
from flask import Flask, Response, request, jsonify

BAUD = 115200

# ── State ─────────────────────────────────────────────────────────────────────

ser = None
ser_lock   = threading.Lock()
reader_thr = None

conn_state = {"connected": False, "port": None, "error": None}
telemetry  = {"rudder": 0, "sail": 0, "leds": 0}

sse_queues = []          # one queue per open SSE client
sse_lock   = threading.Lock()

def broadcast(msg: str):
    with sse_lock:
        dead = []
        for q in sse_queues:
            try:
                q.put_nowait(msg)
            except queue.Full:
                dead.append(q)
        for q in dead:
            sse_queues.remove(q)

def push_raw(line: str):
    broadcast(f"event:raw\ndata:{line}\n\n")

def push_telemetry():
    t = telemetry
    broadcast(f"event:telemetry\ndata:{t['rudder']},{t['sail']},{t['leds']}\n\n")

def push_conn():
    broadcast(f"event:conn\ndata:{'1' if conn_state['connected'] else '0'},{conn_state['port'] or ''},{conn_state['error'] or ''}\n\n")

# ── Serial reader thread ───────────────────────────────────────────────────────

def serial_reader():
    global ser
    while True:
        with ser_lock:
            s = ser
        if s is None or not conn_state["connected"]:
            time.sleep(0.1)
            continue
        try:
            raw = s.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            push_raw(line)
            if line.startswith("STATUS,"):
                parts = line.split(",")
                if len(parts) == 4:
                    telemetry["rudder"] = int(parts[1])
                    telemetry["sail"]   = int(parts[2])
                    telemetry["leds"]   = int(parts[3])
                    push_telemetry()
        except Exception as e:
            conn_state["connected"] = False
            conn_state["error"] = str(e)
            push_conn()
            push_raw(f"[disconnected: {e}]")
            with ser_lock:
                ser = None
            time.sleep(0.1)

# ── Flask ──────────────────────────────────────────────────────────────────────

app = Flask(__name__)

@app.route("/")
def index():
    return HTML

@app.route("/ports")
def ports():
    detected = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")
    all_ports = [p.device for p in serial.tools.list_ports.comports()]
    merged = list(dict.fromkeys(detected + all_ports))
    return jsonify(ports=merged)

@app.route("/connect", methods=["POST"])
def connect():
    global ser
    port = request.json.get("port", "").strip()
    if not port:
        return jsonify(ok=False, error="No port specified"), 400
    with ser_lock:
        if ser and ser.is_open:
            ser.close()
        try:
            ser = serial.Serial(port, BAUD, timeout=1)
            conn_state["connected"] = True
            conn_state["port"]      = port
            conn_state["error"]     = None
            push_conn()
            push_raw(f"[connected on {port} @ {BAUD}]")
            return jsonify(ok=True)
        except Exception as e:
            conn_state["connected"] = False
            conn_state["error"]     = str(e)
            ser = None
            push_conn()
            return jsonify(ok=False, error=str(e)), 500

@app.route("/disconnect", methods=["POST"])
def disconnect():
    global ser
    with ser_lock:
        if ser:
            try:
                ser.close()
            except Exception:
                pass
            ser = None
    conn_state["connected"] = False
    conn_state["error"]     = None
    push_conn()
    push_raw("[disconnected]")
    return jsonify(ok=True)

@app.route("/stream")
def stream():
    q = queue.Queue(maxsize=200)
    with sse_lock:
        sse_queues.append(q)

    def generate():
        # send current state immediately
        t = telemetry
        yield f"event:telemetry\ndata:{t['rudder']},{t['sail']},{t['leds']}\n\n"
        yield f"event:conn\ndata:{'1' if conn_state['connected'] else '0'},{conn_state['port'] or ''},{conn_state['error'] or ''}\n\n"
        while True:
            try:
                yield q.get(timeout=25)
            except queue.Empty:
                yield ": keep-alive\n\n"

    def cleanup(r):
        with sse_lock:
            if q in sse_queues:
                sse_queues.remove(q)
        return r

    return cleanup(Response(generate(), mimetype="text/event-stream",
                            headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"}))

@app.route("/cmd", methods=["POST"])
def cmd():
    key = request.json.get("key", "")
    if len(key) != 1:
        return jsonify(ok=False, error="bad key"), 400
    with ser_lock:
        s = ser
    if not s or not conn_state["connected"]:
        return jsonify(ok=False, error="not connected"), 400
    try:
        s.write(key.encode())
        push_raw(f"[tx] '{key}'")
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

# ── HTML ───────────────────────────────────────────────────────────────────────

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>ALAN Ground Station</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  font-family: system-ui, sans-serif; background: #fff; color: #111;
  display: flex; flex-direction: column; align-items: center;
  min-height: 100vh; padding: 28px 16px; gap: 24px;
}
h1 { font-size: 1.35rem; font-weight: 700; letter-spacing: .04em; }

/* ── connection bar ── */
.conn-bar {
  display: flex; align-items: center; gap: 10px; flex-wrap: wrap;
  justify-content: center;
  background: #f9fafb; border: 1px solid #e5e7eb; border-radius: 10px;
  padding: 10px 16px; width: 100%; max-width: 600px;
}
.conn-status {
  width: 10px; height: 10px; border-radius: 50%; background: #d1d5db; flex-shrink: 0;
}
.conn-status.on { background: #22c55e; }
.conn-status.err { background: #ef4444; }
#conn-label { font-size: .85rem; color: #555; flex: 1; min-width: 120px; }
#port-select {
  font-size: .82rem; border: 1px solid #d1d5db; border-radius: 6px;
  padding: 5px 8px; background: #fff; color: #111; min-width: 180px;
}
.conn-btn {
  padding: 6px 14px; border-radius: 6px; border: 1.5px solid #d1d5db;
  font-size: .82rem; font-weight: 600; cursor: pointer; background: #f9fafb;
  transition: background .1s;
}
.conn-btn.connect { border-color: #22c55e; color: #16a34a; }
.conn-btn.connect:hover { background: #f0fdf4; }
.conn-btn.disconnect { border-color: #ef4444; color: #dc2626; }
.conn-btn.disconnect:hover { background: #fef2f2; }
.refresh-btn {
  padding: 6px 10px; border-radius: 6px; border: 1px solid #d1d5db;
  font-size: .82rem; cursor: pointer; background: #f9fafb;
}
.refresh-btn:hover { background: #f0f0f0; }

/* ── LED dots ── */
.led-row { display: flex; align-items: center; gap: 10px; font-size: .82rem; color: #555; }
.dot { width: 11px; height: 11px; border-radius: 50%; background: #e5e7eb; display: inline-block; }
.dot.on-green  { background: #22c55e; box-shadow: 0 0 6px #22c55e88; }
.dot.on-red    { background: #ef4444; box-shadow: 0 0 6px #ef444488; }
.dot.on-yellow { background: #eab308; box-shadow: 0 0 6px #eab30888; }

/* ── gauges ── */
.gauges { display: flex; gap: 48px; flex-wrap: wrap; justify-content: center; }
.gauge-wrap { display: flex; flex-direction: column; align-items: center; gap: 6px; }
.gauge-label { font-size: .72rem; font-weight: 700; letter-spacing: .1em;
               text-transform: uppercase; color: #6b7280; }
.angle-val { font-size: 1.55rem; font-weight: 700; color: #111; min-width: 72px; text-align: center; }

/* ── controls ── */
.controls { display: flex; gap: 36px; flex-wrap: wrap; justify-content: center; }
.ctrl-group { display: flex; flex-direction: column; align-items: center; gap: 8px; }
.ctrl-title { font-size: .72rem; font-weight: 700; letter-spacing: .1em;
              text-transform: uppercase; color: #6b7280; }
.btn-grid { display: grid; grid-template-columns: repeat(2, 46px); gap: 6px; }
button.ctrl {
  width: 46px; height: 40px; border: 1.5px solid #d1d5db; border-radius: 7px;
  background: #f9fafb; font-size: .78rem; font-weight: 600; cursor: pointer;
  transition: background .1s, border-color .1s; color: #111;
}
button.ctrl:active { transform: scale(.96); }
button.rudder { border-color: #93c5fd; }
button.rudder:hover { background: #eff6ff; border-color: #3b82f6; }
button.sail   { border-color: #86efac; }
button.sail:hover   { background: #f0fdf4; border-color: #22c55e; }
.btn-grid-leds { display: grid; grid-template-columns: repeat(3, 46px); gap: 6px; }
.hint { font-size: .72rem; color: #9ca3af; letter-spacing: .02em; }

/* ── serial terminal ── */
.terminal-wrap { width: 100%; max-width: 600px; }
.terminal-header {
  display: flex; align-items: center; justify-content: space-between;
  margin-bottom: 6px;
}
.terminal-title { font-size: .72rem; font-weight: 700; letter-spacing: .1em;
                  text-transform: uppercase; color: #6b7280; }
.clear-btn {
  font-size: .72rem; padding: 2px 8px; border-radius: 5px;
  border: 1px solid #e5e7eb; background: #f9fafb; cursor: pointer; color: #555;
}
.clear-btn:hover { background: #f0f0f0; }
.terminal {
  width: 100%; height: 160px; overflow-y: auto;
  background: #0f172a; border-radius: 8px;
  padding: 10px 14px; font-family: 'SF Mono', 'Fira Mono', monospace;
  font-size: .75rem; color: #94a3b8; line-height: 1.7;
}
.terminal p { margin: 0; white-space: pre-wrap; word-break: break-all; }
.terminal p.rx  { color: #60a5fa; }
.terminal p.tx  { color: #4ade80; }
.terminal p.sys { color: #f59e0b; }
</style>
</head>
<body>

<h1>ALAN Ground Station</h1>

<!-- Connection bar -->
<div class="conn-bar">
  <span class="conn-status" id="conn-dot"></span>
  <span id="conn-label">Disconnected</span>
  <select id="port-select"><option value="">— select port —</option></select>
  <button class="refresh-btn" onclick="loadPorts()" title="Refresh ports">⟳</button>
  <button class="conn-btn connect" id="conn-btn" onclick="toggleConnect()">Connect</button>
</div>

<!-- LED status -->
<div class="led-row">
  <span class="dot" id="dot-green"></span> Green
  <span class="dot" id="dot-red"></span> Red
  <span class="dot" id="dot-yellow"></span> Yellow
</div>

<!-- Gauges -->
<div class="gauges">
  <div class="gauge-wrap">
    <div class="gauge-label">Rudder</div>
    <svg width="180" height="105" viewBox="0 0 180 105">
      <path d="M18,100 A78,78 0 1,1 162,100" fill="none" stroke="#e5e7eb" stroke-width="8" stroke-linecap="round"/>
      <path id="rudder-arc" d="M18,100 A78,78 0 1,1 162,100" fill="none" stroke="#3b82f6"
            stroke-width="8" stroke-linecap="round" stroke-dasharray="0 1000"/>
      <line id="rudder-needle" x1="90" y1="100" x2="90" y2="26"
            stroke="#1d4ed8" stroke-width="3" stroke-linecap="round" transform="rotate(0,90,100)"/>
      <circle cx="90" cy="100" r="5" fill="#1d4ed8"/>
    </svg>
    <div class="angle-val" id="rudder-val">0°</div>
  </div>

  <div class="gauge-wrap">
    <div class="gauge-label">Sail</div>
    <svg width="180" height="105" viewBox="0 0 180 105">
      <path d="M18,100 A78,78 0 1,1 162,100" fill="none" stroke="#e5e7eb" stroke-width="8" stroke-linecap="round"/>
      <path id="sail-arc" d="M18,100 A78,78 0 1,1 162,100" fill="none" stroke="#22c55e"
            stroke-width="8" stroke-linecap="round" stroke-dasharray="0 1000"/>
      <line id="sail-needle" x1="90" y1="100" x2="90" y2="26"
            stroke="#15803d" stroke-width="3" stroke-linecap="round" transform="rotate(0,90,100)"/>
      <circle cx="90" cy="100" r="5" fill="#15803d"/>
    </svg>
    <div class="angle-val" id="sail-val">0°</div>
  </div>
</div>

<!-- Controls -->
<div class="controls">
  <div class="ctrl-group">
    <div class="ctrl-title">Rudder</div>
    <div class="btn-grid">
      <button class="ctrl rudder" onclick="send('q')">+20</button>
      <button class="ctrl rudder" onclick="send('e')">−20</button>
      <button class="ctrl rudder" onclick="send('a')">+10</button>
      <button class="ctrl rudder" onclick="send('d')">−10</button>
      <button class="ctrl rudder" onclick="send('z')">+5</button>
      <button class="ctrl rudder" onclick="send('c')">−5</button>
    </div>
    <div class="hint">q/e &nbsp;·&nbsp; a/d &nbsp;·&nbsp; z/c</div>
  </div>

  <div class="ctrl-group">
    <div class="ctrl-title">Sail</div>
    <div class="btn-grid">
      <button class="ctrl sail" onclick="send('w')">+20</button>
      <button class="ctrl sail" onclick="send('r')">−20</button>
      <button class="ctrl sail" onclick="send('s')">+10</button>
      <button class="ctrl sail" onclick="send('f')">−10</button>
      <button class="ctrl sail" onclick="send('x')">+5</button>
      <button class="ctrl sail" onclick="send('v')">−5</button>
    </div>
    <div class="hint">w/r &nbsp;·&nbsp; s/f &nbsp;·&nbsp; x/v</div>
  </div>

  <div class="ctrl-group">
    <div class="ctrl-title">LEDs</div>
    <div class="btn-grid-leds">
      <button class="ctrl" style="border-color:#86efac;color:#16a34a" onclick="send('g')">G</button>
      <button class="ctrl" style="border-color:#fca5a5;color:#dc2626" onclick="send('r')">R</button>
      <button class="ctrl" style="border-color:#fde68a;color:#ca8a04" onclick="send('y')">Y</button>
    </div>
    <div class="hint">g &nbsp;·&nbsp; r &nbsp;·&nbsp; y</div>
  </div>
</div>

<!-- Serial terminal -->
<div class="terminal-wrap">
  <div class="terminal-header">
    <span class="terminal-title">Serial Monitor</span>
    <button class="clear-btn" onclick="clearLog()">Clear</button>
  </div>
  <div class="terminal" id="terminal"></div>
</div>

<script>
const MAX_RANGE  = 135;
const ARC_TOTAL  = Math.PI * 78 * (270 / 180);
let connected    = false;

// ── Gauge helpers ────────────────────────────────────────────────────────────
function setNeedle(id, angle) {
  document.getElementById(id).setAttribute('transform', `rotate(${angle},90,100)`);
}
function setArc(id, angle) {
  const fill = ((angle + MAX_RANGE) / (2 * MAX_RANGE)) * ARC_TOTAL;
  document.getElementById(id).style.strokeDasharray = `${fill} ${ARC_TOTAL}`;
}
function updateGauge(prefix, angle) {
  setNeedle(`${prefix}-needle`, angle);
  setArc(`${prefix}-arc`, angle);
  document.getElementById(`${prefix}-val`).textContent = `${angle > 0 ? '+' : ''}${angle}°`;
}
function updateLeds(mask) {
  document.getElementById('dot-green').className  = 'dot' + ((mask & 1) ? ' on-green'  : '');
  document.getElementById('dot-red').className    = 'dot' + ((mask & 2) ? ' on-red'    : '');
  document.getElementById('dot-yellow').className = 'dot' + ((mask & 4) ? ' on-yellow' : '');
}

// ── Connection UI ─────────────────────────────────────────────────────────────
function setConnUI(on, label, isErr) {
  connected = on;
  const dot = document.getElementById('conn-dot');
  dot.className = 'conn-status' + (on ? ' on' : (isErr ? ' err' : ''));
  document.getElementById('conn-label').textContent = label;
  const btn = document.getElementById('conn-btn');
  btn.textContent = on ? 'Disconnect' : 'Connect';
  btn.className   = 'conn-btn ' + (on ? 'disconnect' : 'connect');
}

async function loadPorts() {
  const res  = await fetch('/ports');
  const data = await res.json();
  const sel  = document.getElementById('port-select');
  const cur  = sel.value;
  sel.innerHTML = '<option value="">— select port —</option>';
  data.ports.forEach(p => {
    const o = document.createElement('option');
    o.value = o.textContent = p;
    if (p === cur) o.selected = true;
    sel.appendChild(o);
  });
  if (!cur && data.ports.length) sel.value = data.ports[0];
}

async function toggleConnect() {
  if (connected) {
    await fetch('/disconnect', { method: 'POST' });
  } else {
    const port = document.getElementById('port-select').value;
    if (!port) { log('[error] select a port first', 'sys'); return; }
    const res  = await fetch('/connect', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ port })
    });
    if (!res.ok) {
      const d = await res.json();
      log(`[error] ${d.error}`, 'sys');
    }
  }
}

// ── SSE ───────────────────────────────────────────────────────────────────────
const es = new EventSource('/stream');

es.addEventListener('telemetry', e => {
  const [rudder, sail, leds] = e.data.split(',').map(Number);
  updateGauge('rudder', rudder);
  updateGauge('sail',   sail);
  updateLeds(leds);
});

es.addEventListener('conn', e => {
  const [status, port, err] = e.data.split(',');
  const on = status === '1';
  setConnUI(on,
    on ? `Connected · ${port}` : (err ? `Error: ${err}` : 'Disconnected'),
    !!err && !on);
});

es.addEventListener('raw', e => {
  const line = e.data;
  const cls  = line.startsWith('[tx]') ? 'tx' : line.startsWith('[') ? 'sys' : 'rx';
  log(line, cls);
});

// ── Commands ──────────────────────────────────────────────────────────────────
async function send(key) {
  const res = await fetch('/cmd', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ key })
  });
  if (!res.ok) {
    const d = await res.json();
    log(`[error] ${d.error}`, 'sys');
  }
}

const KEYS = new Set(['q','e','a','d','z','c','w','r','s','f','x','v','g','y']);
document.addEventListener('keydown', ev => {
  if (['INPUT','SELECT','TEXTAREA'].includes(document.activeElement.tagName)) return;
  if (KEYS.has(ev.key)) { ev.preventDefault(); send(ev.key); }
});

// ── Terminal log ──────────────────────────────────────────────────────────────
const term = document.getElementById('terminal');
function log(msg, cls) {
  const p = document.createElement('p');
  if (cls) p.className = cls;
  const now = new Date().toLocaleTimeString('en-US', { hour12: false });
  p.textContent = `${now}  ${msg}`;
  term.appendChild(p);
  while (term.children.length > 300) term.firstChild.remove();
  term.scrollTop = term.scrollHeight;
}
function clearLog() { term.innerHTML = ''; }

// ── Init ──────────────────────────────────────────────────────────────────────
loadPorts();
log('Ground station ready. Select a port and click Connect.', 'sys');
</script>
</body>
</html>"""

# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    port_hint = sys.argv[1] if len(sys.argv) > 1 else (
        glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*") or [None])[0]
    if port_hint:
        print(f"[alan] detected port: {port_hint} (connect in browser)")
    print("[alan] dashboard → http://localhost:8080")
    app.run(host="127.0.0.1", port=8080, threaded=True)
