#!/usr/bin/env python3
"""
ALAN simulation dashboard: one local web page that runs the whole virtual boat.

  python3 sim/dashboard/server.py            then open http://localhost:8080

It hosts:
  - the boat physics (sim/plant) running in real time, or in lockstep with the firmware when Renode is connected
  - the Renode lockstep link on port 5555 (sim/renode/models/AlanPlantBridge.cs connects to it)
  - Renode itself: build, start, stop, firmware console, monitor commands
  - a live data stream to the browser (Server-Sent Events) and a command API

Python standard library only.
"""
import argparse
import collections
import json
import math
import os
import re
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "sim", "plant"))

from alan_plant import cad  # noqa: E402
from alan_plant.geo import wrap360  # noqa: E402
from alan_plant.autopilot import NO_GO_DEG, plan_route  # noqa: E402
from alan_plant.polar import polar  # noqa: E402
from alan_plant.server import Simulation, load_scenario  # noqa: E402

SCENARIO_DIR = os.path.join(REPO, "sim", "plant", "scenarios")
LOG_DIR = os.path.join(REPO, "sim", "plant", "logs")
STATIC_DIR = os.path.join(HERE, "static")
MODELS_DIR = os.path.join(HERE, "models")
ANSI = re.compile(r"\x1b\[[0-9;]*m")
UART_LINE = re.compile(r"usart3: \[[^\]]*\] ?(.*)$")
NOISY = ("Unhandled", "non existing peripheral", "Translation cache", "Pseudorandom")


def find_renode():
    for c in (os.environ.get("RENODE"), "/Applications/Renode.app/Contents/MacOS/renode"):
        if c and os.path.exists(c):
            return c
    from shutil import which
    return which("renode")


def _bodies(tris):
    """Split triangles into connected bodies (shared vertices)."""
    parent = {}

    def find(a):
        while parent.setdefault(a, a) != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    keys = [tuple(tuple(round(c, 3) for c in v) for v in t) for t in tris]
    for a, b, c in keys:
        ra = find(a)
        parent[find(b)] = ra
        parent[find(c)] = ra
    groups = {}
    for t, k in zip(tris, keys):
        groups.setdefault(find(k[0]), []).append(t)
    return list(groups.values())


def _hull2d(pts):
    """Convex hull (monotone chain) of 2-D points."""
    if len(pts) < 3:
        return [list(p) for p in pts]
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return [list(p) for p in lower[:-1] + upper[:-1]]


class Feed:
    """A numbered ring buffer of text lines, so each browser tab can ask for 'everything after N'."""

    def __init__(self, size=2000):
        self.lines = collections.deque(maxlen=size)
        self.count = 0
        self.lock = threading.Lock()

    def add(self, text):
        with self.lock:
            self.count += 1
            self.lines.append((self.count, text))

    def since(self, n, limit=80):
        with self.lock:
            out = [t for i, t in self.lines if i > n]
            if len(out) > limit:
                # too fast to show everything: keep the console replies, then the newest lines
                keep = [t for t in out[:-limit] if "[CONSOLE]" in t]
                out = keep + [f"… {len(out) - limit - len(keep)} lines skipped (output too fast to show) …"] + out[-limit:]
            return out, self.count


class Hub:
    def __init__(self, scenario_path, lockstep_port):
        self.scenario_path = scenario_path
        self.lockstep_port = lockstep_port
        self.sim = Simulation(load_scenario(scenario_path), LOG_DIR)
        self.paused = False
        self.speed = 1.0
        self.renode_proc = None
        self.renode_state = "stopped"          # stopped | building | starting | running
        self.renode_linked = False
        self.link_offset = None
        self.last_link = 0.0
        self.firmware = None                   # what is loaded in the emulated STM32 (branch, commit, sensors, build time)
        self.console = Feed()                  # firmware printf output (USART3)
        self.syslog = Feed(500)                # Renode and dashboard messages
        self.lock = threading.RLock()
        self.bench_seen = 0.0                  # last time a page showing the Bench tab checked in

    # ---- simulation ----------------------------------------------------------------------------

    def reset(self, scenario_path=None):
        with self.lock:
            if scenario_path is not None:
                self.scenario_path = scenario_path or None
            old = self.sim
            self.sim = Simulation(load_scenario(self.scenario_path), LOG_DIR)
            self.sim.boat.servo_override = old.boat.servo_override
            old.close()
            self.link_offset = None
        self.syslog.add(f"Simulation reset ({os.path.basename(self.scenario_path or 'default')}); log {self.sim.log_path}")

    def command(self, text):
        with self.lock:
            self.sim.command(text)
        self.syslog.add(f"> {text.strip()}")

    def physics_loop(self):
        """Real-time stepping when the firmware is not driving the clock."""
        last = time.time()
        while True:
            time.sleep(0.02)
            now = time.time()
            dt = min(0.1, now - last) * self.speed
            last = now
            if self.renode_linked and now - self.last_link > 2.0:
                self.renode_linked = False
                self.syslog.add("Firmware link idle; physics back to real time")
            # bench mode belongs to a page that is showing the Bench tab; if it goes away, go back outside
            if self.sim.boat.bench and now - self.bench_seen > 4.0:
                with self.lock:
                    self.sim.command("bench off")
                self.syslog.add("Bench page closed: back on the water")
            if self.paused or self.renode_linked:
                continue
            with self.lock:
                self.sim.advance_to(self.sim.t + dt)

    # ---- Renode lockstep link -----------------------------------------------------------------

    def lockstep_server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", self.lockstep_port))
        srv.listen(1)
        while True:
            conn, _ = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.syslog.add("Firmware connected: the boat now follows the emulated STM32")
            self.link_offset = None
            f = conn.makefile("rw", newline="\n")
            try:
                for line in f:
                    msg = dict(kv.split("=", 1) for kv in line.split() if "=" in kv)
                    t_fw = float(msg.get("t", 0))
                    while self.paused:
                        time.sleep(0.05)          # pausing the dashboard also freezes the firmware
                    with self.lock:
                        if self.link_offset is None:
                            self.link_offset = self.sim.t - t_fw
                        msg["t"] = f"{t_fw + self.link_offset:.4f}"
                        reply = self.sim.handle_line(" ".join(f"{k}={v}" for k, v in msg.items()))
                    self.renode_linked = True
                    self.last_link = time.time()
                    f.write(reply + "\n")
                    f.flush()
            except (ConnectionError, OSError, ValueError):
                pass
            finally:
                conn.close()
                self.renode_linked = False
                self.syslog.add("Firmware disconnected; physics back to real time")

    # ---- Renode process -----------------------------------------------------------------------

    def renode_start(self, sensors, build=True):
        if self.renode_proc and self.renode_proc.poll() is None:
            return "Renode is already running"
        renode = find_renode()
        if not renode:
            return "Renode not found: install it in /Applications or set RENODE=/path/to/renode"
        threading.Thread(target=self._renode_run, args=(renode, sensors, build), daemon=True).start()
        return "starting"

    def _renode_run(self, renode, sensors, build):
        if build:
            self.renode_state = "building"
            self.syslog.add(f"Building firmware with sensors: {sensors or 'none'}")
            r = subprocess.run([os.path.join(REPO, "sim", "renode", "run.sh"), "--build-only", "--sensors", sensors or "none"],
                               cwd=REPO, capture_output=True, text=True)
            for line in (r.stdout + r.stderr).splitlines():
                self.syslog.add(line)
            if r.returncode != 0:
                self.renode_state = "stopped"
                self.syslog.add("Build failed; Renode not started")
                return
        self.firmware = self._firmware_info(sensors)
        fw = self.firmware
        self.syslog.add(f"Firmware: {fw['elf']} built {fw['built']} from {fw['branch']} @ {fw['commit']}"
                        f"{' + uncommitted edits' if fw['dirty'] else ''}")
        self.renode_state = "starting"
        self.syslog.add("Starting Renode")
        self.renode_proc = subprocess.Popen(
            [renode, "--disable-gui", "--console", "-e", "include @sim/renode/alan_cm7.resc; start"],
            cwd=REPO, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.renode_state = "running"
        for raw in self.renode_proc.stdout:
            line = ANSI.sub("", raw.rstrip())
            m = UART_LINE.search(line)
            if m:
                self.console.add(m.group(1))
            elif line and not any(n in line for n in NOISY):
                self.syslog.add(line)
        self.renode_state = "stopped"
        self.syslog.add(f"Renode exited (code {self.renode_proc.wait()})")

    def _firmware_info(self, sensors):
        """Identify exactly which code the emulated chip is running."""
        def git(*args):
            r = subprocess.run(["git", "-C", REPO, *args], capture_output=True, text=True)
            return r.stdout.strip() if r.returncode == 0 else ""
        elf = os.path.join(REPO, "sim", "renode", "build", "cm7", "Autonomaus_Sailboat_CM7.elf")
        changed = [l[3:] for l in git("status", "--porcelain", "--", "CM7", "CM4", "Common").splitlines() if l.strip()]
        return {
            "elf": os.path.relpath(elf, REPO),
            "built": time.strftime("%H:%M:%S", time.localtime(os.path.getmtime(elf))) if os.path.exists(elf) else "not built",
            "branch": git("rev-parse", "--abbrev-ref", "HEAD") or "?",
            "commit": git("rev-parse", "--short", "HEAD") or "?",
            "subject": git("log", "-1", "--format=%s"),
            "dirty": bool(changed),
            "changed": changed[:12],
            "sensors": sensors or "none",
            "source": "CM7/ (your firmware), compiled by sim/renode/run.sh --build-only",
        }

    def renode_stop(self):
        p = self.renode_proc
        if not p or p.poll() is not None:
            return "not running"
        try:
            p.stdin.write("quit\n")
            p.stdin.flush()
            p.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            p.kill()
        return "stopped"

    def renode_monitor(self, text):
        p = self.renode_proc
        if not p or p.poll() is not None:
            return "Renode is not running"
        p.stdin.write(text.strip() + "\n")
        p.stdin.flush()
        self.syslog.add(f"(monitor) {text.strip()}")
        return "sent"

    # ---- stability, CAD ----------------------------------------------------------------------------

    def _outlines(self):
        """Side-view outlines (y, z in CAD mm) of the parts above the water, for the stability drawing."""
        if hasattr(self, "_outline_cache"):
            return self._outline_cache
        out = {}
        for name in ("wing", "tail", "booms", "rudder", "electronics"):
            path = os.path.join(MODELS_DIR, f"{name}.stl")
            if not os.path.exists(path):
                continue
            tris = cad.read_stl(path)
            if name == "booms":
                # three separate rods: one outline each, not one block around all of them
                out[name] = [_hull2d(sorted({(round(v[1]), round(v[2])) for t in body for v in t})) for body in _bodies(tris)]
                continue
            pts = sorted({(round(v[1]), round(v[2])) for t in tris for v in t})
            out[name] = _hull2d(pts)
            xs = [v[0] for t in cad.read_stl(path) for v in t] if name in ("wing", "tail") else None
            if xs:
                out[name + "_x"] = [min(xs), max(xs)]
        self._outline_cache = out
        return out

    def hydro_view(self, heel):
        h = self.sim.hydro
        if not h:
            return {"available": False}
        with self.lock:
            sil = h.silhouettes(heel)
            res = dict(h.result)
            p = self.sim.boat.p
        return {"available": True, "result": res, "silhouettes": sil, "outlines": self._outlines(), "heel": heel,
                "wing_area_m2": p.wing_area_m2, "lever_m": p.wing_ce_z_m - p.clr_z_m}

    def boat_info(self):
        try:
            with open(os.path.join(MODELS_DIR, "boat.json")) as f:
                cfg = json.load(f)
        except OSError:
            return {"available": False}
        h = self.sim.hydro
        if h:
            cfg["cg"] = h.result["cg_mm"]
            cfg["waterline"] = h.result["waterline_z_mm"]
            cfg["trim_deg"] = h.result["trim_deg"]
        cfg["available"] = True
        return cfg

    def save_mass(self, items):
        clean = []
        for i in items:
            try:
                clean.append({"name": str(i.get("name", "part")), "kg": float(i["kg"]),
                              "at_mm": [float(v) for v in i["at_mm"]], **({"note": i["note"]} if i.get("note") else {})})
            except (KeyError, TypeError, ValueError):
                return {"error": f"bad item: {i}"}
        if not clean or sum(i["kg"] for i in clean) <= 0:
            return {"error": "total mass must be positive"}
        path = os.path.join(MODELS_DIR, "mass.json")
        with open(path) as f:
            cfg = json.load(f)
        cfg["items"] = clean
        with open(path, "w") as f:
            json.dump(cfg, f, indent=2)
        self.syslog.add(f"Saved mass.json ({sum(i['kg'] for i in clean):.3f} kg); recomputing stability")
        self.reset()
        return self.hydro_view(0.0)

    # ---- state for the browser ----------------------------------------------------------------

    def snapshot(self):
        with self.lock:
            sim, b = self.sim, self.sim.boat
            s = b.s
            lat, lon = sim.frame.to_latlon(s.north_m, s.east_m)
            return {
                "t": round(sim.t, 3),
                "north": round(s.north_m, 3), "east": round(s.east_m, 3), "lat": lat, "lon": lon,
                "heading": round(s.heading_deg, 2), "cog": round(b.cog_deg, 1),
                "sog": round(b.sog_ms, 3), "stw": round(b.stw_ms, 3), "yaw_rate": round(math.degrees(s.r), 2),
                "wing": round(s.wing_deg, 2), "aoa": round(s.aoa_deg, 2),
                "awa": round(s.awa_deg, 1), "aws": round(s.aws_ms, 2),
                "tail": round(s.tail_deg, 2), "rudder": round(s.rudder_deg, 2),
                "tail_cmd": round(s.tail_cmd_deg, 2), "rudder_cmd": round(s.rudder_cmd_deg, 2),
                "fw_tail_us": getattr(b, "fw_tail_us", None), "fw_rudder_us": getattr(b, "fw_rudder_us", None),
                "override": b.servo_override,
                "wind_dir": round(wrap360(sim.wind.current_direction_deg), 1), "wind_speed": round(sim.wind.current_speed_ms, 2),
                "wind_mean_dir": sim.wind.direction_deg, "wind_mean_speed": sim.wind.speed_ms,
                "gust": sim.wind.gust_ms, "shift": sim.wind.shift_deg,
                "current_dir": sim.current.direction_deg, "current_speed": sim.current.speed_ms,
                "gps_ok": sim.sensors.gps_ok, "vane_ok": sim.sensors.vane_ok,
                "heel": round(s.heel_deg, 1), "vane_mount": b.noise.vane_mount,
                "vane_custom": [b.noise.vane_custom_x_m, b.noise.vane_custom_z_m],
                "bench": b.bench, "hold_wing": b.hold_wing,
                "wing_limit": b.p.wing_limit_deg, "wing_on_stop": s.wing_on_stop,
                "trim_calc": (lambda t: {"wing": b.hold_wing, "awa": round(s.awa_deg, 1), "alpha": round(t[1], 1),
                                         "tail": round(t[0], 1), "reachable": t[2],
                                         "gain": b.p.tail_sign * b.p.tail_to_aoa, "limit": b.p.tail_limit_deg})(
                    b.tail_for_wing(b.hold_wing if b.hold_wing is not None else s.wing_deg)),
                "vane": round(s.vane_deg, 1), "vane_flow": round(s.vane_flow_deg, 1),
                "firmware": self.firmware if self.renode_state != "stopped" else None,
                "autopilot": sim.autopilot.to_dict() if sim.autopilot else None,
                "paused": self.paused, "speed": self.speed,
                "mode": "firmware" if self.renode_linked else "physics",
                "renode": self.renode_state,
                "scenario": os.path.basename(self.scenario_path) if self.scenario_path else "default.json",
                "log": os.path.relpath(sim.log_path, REPO),
            }


def _q(path):
    from urllib.parse import parse_qs, urlparse
    return {k: v[0] for k, v in parse_qs(urlparse(path).query).items()}


def make_handler(hub: Hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            if os.environ.get("ALAN_HTTP_LOG"):
                print(time.strftime("%H:%M:%S"), self.address_string(), fmt % args, flush=True)

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _file(self, path, ctype):
            try:
                with open(path, "rb") as f:
                    body = f.read()
            except OSError:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                return self._file(os.path.join(STATIC_DIR, "index.html"), "text/html; charset=utf-8")
            if path.startswith("/static/"):
                name = os.path.basename(path)
                ctype = {"js": "text/javascript", "css": "text/css"}.get(name.rsplit(".", 1)[-1], "application/octet-stream")
                return self._file(os.path.join(STATIC_DIR, name), ctype)
            if path.startswith("/models/"):
                name = os.path.basename(path)
                ctype = "application/json" if name.endswith(".json") else "application/octet-stream"
                return self._file(os.path.join(MODELS_DIR, name), ctype)
            if path == "/api/state":
                return self._json(hub.snapshot())
            if path == "/api/scenarios":
                files = sorted(f for f in os.listdir(SCENARIO_DIR) if f.endswith(".json"))
                out = []
                for f in files:
                    try:
                        with open(os.path.join(SCENARIO_DIR, f)) as fh:
                            out.append({"file": f, "description": json.load(fh).get("description", "")})
                    except (OSError, ValueError):
                        pass
                return self._json(out)
            if path == "/api/hydro":
                return self._json(hub.hydro_view(float(_q(self.path).get("heel", 0))))
            if path == "/api/polar":
                q = _q(self.path)
                with hub.lock:
                    p, righting = hub.sim.boat.p, hub.sim.boat.righting
                rows = polar(p, righting, tws=float(q.get("tws", 3)), aoa=float(q.get("aoa", 12)),
                             optimize=q.get("optimize") == "1", step=int(q.get("step", 15)))
                return self._json({"rows": rows, "tail_to_aoa": p.tail_to_aoa, "tail_sign": p.tail_sign,
                                   "tail_limit": p.tail_limit_deg})
            if path == "/api/plan":
                q = _q(self.path)
                with hub.lock:
                    twd = float(q["twd"]) if "twd" in q else hub.sim.wind.direction_deg
                plan = plan_route((float(q["an"]), float(q["ae"])), (float(q["bn"]), float(q["be"])), twd,
                                  margin_m=float(q.get("margin", 12)))
                plan["twd"] = twd
                plan["no_go"] = NO_GO_DEG
                return self._json(plan)
            if path == "/api/boat":
                return self._json(hub.boat_info())
            if path == "/api/models":
                names = sorted(os.listdir(MODELS_DIR)) if os.path.isdir(MODELS_DIR) else []
                return self._json(names)
            if path == "/api/stream":
                return self._stream()
            self.send_error(404)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            con_n = max(0, hub.console.count - 200)
            log_n = max(0, hub.syslog.count - 100)
            try:
                while True:
                    con, con_n = hub.console.since(con_n)
                    logs, log_n = hub.syslog.since(log_n)
                    msg = {"state": hub.snapshot(), "console": con, "log": logs}
                    self.wfile.write(f"data: {json.dumps(msg)}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                return self._json({"error": "bad JSON"}, 400)
            path = self.path
            if path == "/api/cmd":
                text = body.get("cmd", "")
                if text.strip().startswith("bench on"):
                    hub.bench_seen = time.time()
                hub.command(text)
                return self._json({"ok": True})
            if path == "/api/bench/ping":
                hub.bench_seen = time.time()
                return self._json({"ok": True})
            if path == "/api/pause":
                hub.paused = bool(body.get("paused", not hub.paused))
                return self._json({"paused": hub.paused})
            if path == "/api/speed":
                hub.speed = max(0.1, min(10.0, float(body.get("speed", 1.0))))
                if hub.renode_proc and hub.renode_proc.poll() is None:
                    hub.renode_monitor(f"plant SpeedFactor {hub.speed}")
                return self._json({"speed": hub.speed})
            if path == "/api/reset":
                scen = body.get("scenario")
                hub.reset(os.path.join(SCENARIO_DIR, os.path.basename(scen)) if scen else None)
                return self._json({"ok": True})
            if path == "/api/mass":
                return self._json(hub.save_mass(body.get("items", [])))
            if path == "/api/renode/start":
                return self._json({"result": hub.renode_start(body.get("sensors", "wind,imu,encoder,gps"), body.get("build", True))})
            if path == "/api/renode/stop":
                return self._json({"result": hub.renode_stop()})
            if path == "/api/renode/uart":
                # type a line into the firmware's serial console (USART3), as if from a terminal
                text = str(body.get("text", "")).replace("\\", "").replace('"', "'")[:60]
                return self._json({"result": hub.renode_monitor(f'sysbus.usart3 WriteLine "{text}"')})
            if path == "/api/renode/monitor":
                return self._json({"result": hub.renode_monitor(body.get("cmd", ""))})
            self.send_error(404)

    return Handler


def main():
    ap = argparse.ArgumentParser(description="ALAN simulation dashboard")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--lockstep-port", type=int, default=5555)
    ap.add_argument("--scenario", default=os.path.join(SCENARIO_DIR, "default.json"))
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--firmware", nargs="?", const="wind,encoder,gps", metavar="SENSORS",
                    help="build and run your firmware in Renode right away (default sensors: wind,encoder,gps)")
    args = ap.parse_args()

    hub = Hub(args.scenario, args.lockstep_port)
    threading.Thread(target=hub.physics_loop, daemon=True).start()
    threading.Thread(target=hub.lockstep_server, daemon=True).start()

    class QuietServer(ThreadingHTTPServer):
        daemon_threads = True

        def handle_error(self, request, client_address):
            # a browser tab closing mid-download is normal; anything else is worth printing
            if not isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError)):
                super().handle_error(request, client_address)

    httpd = QuietServer(("127.0.0.1", args.port), make_handler(hub))
    if args.firmware:
        print(f"Building and starting your firmware (sensors: {args.firmware})…")
        hub.renode_start(args.firmware, build=True)
    url = f"http://localhost:{args.port}"
    print(f"ALAN dashboard on {url}  (Ctrl-C to stop)")
    if not args.no_browser:
        def open_browser():
            if sys.platform == "darwin":
                subprocess.run(["open", url], check=False)   # more reliable than webbrowser on macOS
            else:
                webbrowser.open(url)
        threading.Timer(0.8, open_browser).start()
        print("If no browser tab opened, Cmd-click the link above.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        hub.renode_stop()
        hub.sim.close()
        print(f"Log saved: {hub.sim.log_path}")


if __name__ == "__main__":
    main()
