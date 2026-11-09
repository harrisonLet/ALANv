"""
Lockstep link to Renode plus the simulation loop, scenario events, live commands and logging.

Protocol (one line each way, every 20 ms of virtual time):
  Renode -> plant:  t=12.340 tail=1555 rudder=1500        (servo pulse widths in microseconds)
  plant  -> Renode: heading=91.2 yaw_rate=0.4 wing=271.0 vane=88.5 vane_ok=1 lat=40.76 lon=-111.84 gps_fix=1 gps_ok=1
"""
import csv
import json
import math
import os
import socket
import threading
import time
from dataclasses import fields

from . import cad
from .boat import Boat, BoatParams, BoatState, NoiseParams
from .geo import LocalFrame, wrap360
from .sensors import Sensors
from .autopilot import Autopilot, plan_route
from .wind import Current, Wind

PHYSICS_DT = 0.004
LOG_PERIOD = 0.1

HELP = """Commands (type while the simulation runs):
  wind DIR SPEED      true wind FROM direction (deg) and speed (m/s)
  current DIR SPEED   water current flowing TOWARD direction (deg) at speed (m/s)
  gust SIGMA          gust strength, m/s           shift SIGMA   direction wobble, deg
  gps on|off          GPS dropout                  vane on|off   wind vane dropout
  heading DEG         turn the boat instantly      stop          zero the boat's speed
  servo TAIL RUDDER   servo angles in degrees (physics-only mode, or with override on)
  override on|off     ignore the firmware's servo commands
  vanemount bow|stern|masthead|custom   where the wind vane is mounted
  vanepos X Z         custom vane mount: X m forward of the CG, Z m above the waterline
  bench on|off        boat on its stand: hull held still, fan instead of wind
  fan DIR SPEED       steady wind from DIR degrees relative to the bow (use on the bench)
  holdwing DEG|off    sim helper: move the tail so the wing sits at DEG to the hull
  winglimit DEG       wing hard stops at +/-DEG (90 = 180 degree swing; 180 = free 360)
  route N E N E ...   sim autopilot sails these waypoints (metres north/east of the origin)
  autopilot off       stop the sim autopilot     place N E [HDG]   move the boat
  status              print the current state      help          this text
  quit                stop the plant"""


def _apply_overrides(obj, overrides):
    names = {f.name for f in fields(obj)}
    for k, v in (overrides or {}).items():
        if k not in names:
            raise KeyError(f"unknown parameter '{k}' for {type(obj).__name__}")
        setattr(obj, k, v)
    return obj


DEFAULT_MODELS_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "dashboard", "models")


class Simulation:
    def __init__(self, scenario: dict, log_dir: str, models_dir: str = DEFAULT_MODELS_DIR):
        self.scenario = scenario
        lat0, lon0 = scenario.get("origin", [40.7649, -111.8421])
        self.frame = LocalFrame(lat0, lon0)
        # Boat parameters: defaults, then measurements from the CAD (boat.json), then the scenario's own overrides
        cad_overrides, self.cad_report, self.hydro = cad.load(models_dir)
        self.models_dir = models_dir
        params = _apply_overrides(_apply_overrides(BoatParams(), cad_overrides), scenario.get("boat"))
        noise = _apply_overrides(NoiseParams(), scenario.get("noise"))
        state = _apply_overrides(BoatState(), scenario.get("initial"))
        righting = (lambda h: self.hydro.righting_moment(h)) if self.hydro else None
        self.boat = Boat(params, noise, state, righting)
        w = scenario.get("wind", {})
        self.wind = Wind(w.get("direction_deg", 270.0), w.get("speed_ms", 3.0), w.get("gust_ms", 0.5),
                         w.get("shift_deg", 5.0), w.get("tau_s", 8.0), seed=scenario.get("seed", 1))
        c = scenario.get("current", {})
        self.current = Current(c.get("direction_deg", 0.0), c.get("speed_ms", 0.0))
        self.sensors = Sensors(self.boat, self.frame, seed=scenario.get("seed", 1) + 1)
        self.events = sorted(scenario.get("events", []), key=lambda e: e["t"])
        self.autopilot = None
        self.t = 0.0
        self.lock = threading.Lock()
        self.running = True
        self._next_log = 0.0

        os.makedirs(log_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.log_path = os.path.join(log_dir, f"run-{stamp}.csv")
        self._log_file = open(self.log_path, "w", newline="")
        self._log = csv.writer(self._log_file)
        self._log.writerow(["t", "lat", "lon", "north_m", "east_m", "heading_deg", "cog_deg", "sog_ms", "yaw_rate_dps",
                            "wing_deg", "aoa_deg", "awa_deg", "aws_ms", "twd_deg", "tws_ms",
                            "tail_cmd_deg", "tail_deg", "rudder_cmd_deg", "rudder_deg", "gps_ok", "vane_ok",
                            "heel_deg", "vane_deg", "vane_mount"])
        latest = os.path.join(log_dir, "latest.csv")
        try:
            if os.path.lexists(latest):
                os.remove(latest)
            os.symlink(os.path.basename(self.log_path), latest)
        except OSError:
            pass

    # ---- stepping ----------------------------------------------------------------------------

    def advance_to(self, t_target):
        while self.t < t_target - 1e-9:
            dt = min(PHYSICS_DT, t_target - self.t)
            self.wind.step(dt)
            if self.autopilot:
                self.autopilot.update(self.boat, self.wind, dt)
            self.boat.step(dt, self.wind, self.current.velocity_ne())
            self.t += dt
            while self.events and self.events[0]["t"] <= self.t:
                ev = self.events.pop(0)
                print(f"[plant t={self.t:7.2f}] event: {ev['cmd']}")
                self.command(ev["cmd"])
            if self.t >= self._next_log:
                self._write_log()
                self._next_log += LOG_PERIOD

    def _write_log(self):
        s = self.boat.s
        lat, lon = self.frame.to_latlon(s.north_m, s.east_m)
        self._log.writerow([f"{self.t:.3f}", f"{lat:.7f}", f"{lon:.7f}", f"{s.north_m:.2f}", f"{s.east_m:.2f}",
                            f"{s.heading_deg:.1f}", f"{self.boat.cog_deg:.1f}", f"{self.boat.sog_ms:.3f}",
                            f"{math.degrees(s.r):.2f}", f"{s.wing_deg:.1f}", f"{s.aoa_deg:.1f}", f"{s.awa_deg:.1f}",
                            f"{s.aws_ms:.2f}", f"{wrap360(self.wind.current_direction_deg):.1f}",
                            f"{self.wind.current_speed_ms:.2f}", f"{s.tail_cmd_deg:.1f}", f"{s.tail_deg:.1f}",
                            f"{s.rudder_cmd_deg:.1f}", f"{s.rudder_deg:.1f}",
                            int(self.sensors.gps_ok), int(self.sensors.vane_ok),
                            f"{s.heel_deg:.1f}", f"{s.vane_deg:.1f}", self.boat.noise.vane_mount])
        self._log_file.flush()

    def handle_line(self, line):
        msg = dict(kv.split("=", 1) for kv in line.split() if "=" in kv)
        t = float(msg.get("t", self.t))
        with self.lock:
            self.boat.set_servo_pulses(int(msg["tail"]) if "tail" in msg else None,
                                       int(msg["rudder"]) if "rudder" in msg else None)
            prev = self.t
            self.advance_to(t)
            out = self.sensors.read(max(1e-3, self.t - prev))
        return " ".join(f"{k}={v:.7f}" if k in ("lat", "lon") else f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}"
                        for k, v in out.items())

    # ---- commands ----------------------------------------------------------------------------

    def command(self, text):
        parts = text.strip().split()
        if not parts:
            return
        c, args = parts[0].lower(), parts[1:]
        s = self.boat.s
        try:
            if self.boat.bench and c in ("wind", "gust", "shift"):
                return                      # on the bench the fan sets the wind (use "fan")
            if c == "wind":
                self.wind.direction_deg = float(args[0])
                if len(args) > 1:
                    self.wind.speed_ms = float(args[1])
            elif c == "current":
                self.current.direction_deg = float(args[0])
                if len(args) > 1:
                    self.current.speed_ms = float(args[1])
            elif c == "vanemount":
                if args[0] not in ("bow", "stern", "masthead", "custom"):
                    raise ValueError
                self.boat.noise.vane_mount = args[0]
            elif c == "vanepos":
                self.boat.noise.vane_custom_x_m, self.boat.noise.vane_custom_z_m = float(args[0]), float(args[1])
                self.boat.noise.vane_mount = "custom"
            elif c == "bench":
                on = args[0] == "on"
                if on and not self.boat.bench:
                    # remember the outdoor conditions; "bench off" puts them back
                    self._outdoor = (self.wind.direction_deg, self.wind.speed_ms, self.wind.gust_ms, self.wind.shift_deg,
                                     s.heading_deg)
                if not on and self.boat.bench:
                    self.boat.hold_wing = None
                    if getattr(self, "_outdoor", None):
                        d, v, g, sh, hdg = self._outdoor
                        self.wind.direction_deg, self.wind.speed_ms, self.wind.gust_ms, self.wind.shift_deg = d, v, g, sh
                        s.heading_deg = hdg
                self.boat.bench = on
                if on:
                    s.u = s.v = s.r = 0.0
                    s.heel_deg = 0.0
                    s.heading_deg = 0.0           # the stand points the bow north, so the fan angle is relative to the bow
            elif c == "fan":
                # bench fan: blows FROM this angle relative to the bow, steady (no gusts or shifts)
                self.wind.direction_deg = wrap360(float(args[0]) + (s.heading_deg if not self.boat.bench else 0.0))
                if len(args) > 1:
                    self.wind.speed_ms = float(args[1])
                self.wind.gust_ms = 0.0
                self.wind.shift_deg = 0.0
            elif c == "route":
                # route N1 E1 N2 E2 ...  (metres north/east of the origin) -> the autopilot follows it
                v = [float(x) for x in args]
                pts = [(v[i], v[i + 1]) for i in range(0, len(v) - 1, 2)]
                aoa = self.autopilot.aoa if self.autopilot else 12.0
                self.autopilot = Autopilot(pts, aoa) if len(pts) >= 2 else None
            elif c == "autopilot":
                if args[0] == "off":
                    self.autopilot = None
                    self.boat.servo_override = False
                elif args[0] == "aoa" and self.autopilot:
                    self.autopilot.aoa = float(args[1])
            elif c == "place":
                s.north_m, s.east_m = float(args[0]), float(args[1])
                if len(args) > 2:
                    s.heading_deg = wrap360(float(args[2]))
                s.u = s.v = s.r = 0.0
            elif c == "winglimit":
                self.boat.p.wing_limit_deg = float(args[0])
            elif c == "holdwing":
                self.boat.hold_wing = None if args[0] == "off" else float(args[0])
            elif c == "override":
                self.boat.servo_override = args[0] == "on"
            elif c == "gust":
                self.wind.gust_ms = float(args[0])
            elif c == "shift":
                self.wind.shift_deg = float(args[0])
            elif c == "gps":
                self.sensors.gps_ok = args[0] == "on"
            elif c == "vane":
                self.sensors.vane_ok = args[0] == "on"
            elif c == "heading":
                s.heading_deg = wrap360(float(args[0]))
            elif c == "stop":
                s.u = s.v = s.r = 0.0
            elif c == "servo":
                s.tail_cmd_deg, s.rudder_cmd_deg = float(args[0]), float(args[1])
            elif c == "status":
                print(self.status())
            elif c == "help":
                print(HELP)
            elif c == "quit":
                self.running = False
            else:
                print(f"unknown command '{c}' (type help)")
        except (IndexError, ValueError):
            print(f"bad arguments for '{c}' (type help)")

    def status(self):
        s = self.boat.s
        lat, lon = self.frame.to_latlon(s.north_m, s.east_m)
        return (f"t={self.t:7.2f}s  pos={lat:.6f},{lon:.6f}  heading={s.heading_deg:5.1f}  sog={self.boat.sog_ms:.2f} m/s  "
                f"wind {wrap360(self.wind.current_direction_deg):5.1f}@{self.wind.current_speed_ms:.1f}  awa={s.awa_deg:6.1f}  "
                f"wing={s.wing_deg:6.1f} aoa={s.aoa_deg:5.1f}  tail={s.tail_deg:5.1f}  rudder={s.rudder_deg:5.1f}  "
                f"gps={'on' if self.sensors.gps_ok else 'OFF'} vane={'on' if self.sensors.vane_ok else 'OFF'}")

    def close(self):
        self._log_file.close()


def load_scenario(path):
    if not path:
        return {}
    with open(path) as f:
        return json.load(f)


def serve(sim: Simulation, host="127.0.0.1", port=5555, status_every=5.0):
    """Wait for Renode, then answer its lockstep requests until it disconnects or 'quit'."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    srv.settimeout(0.5)
    print(f"Plant listening on {host}:{port}; start Renode (sim/renode/run.sh). Log: {sim.log_path}")
    next_status = status_every
    while sim.running:
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        print(f"Renode connected from {addr[0]}:{addr[1]}")
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        f = conn.makefile("rw", newline="\n")
        try:
            for line in f:
                f.write(sim.handle_line(line) + "\n")
                f.flush()
                if status_every and sim.t >= next_status:
                    print(sim.status())
                    next_status += status_every
                if not sim.running:
                    break
        except (ConnectionError, OSError):
            pass
        finally:
            conn.close()
        print("Renode disconnected; waiting for a new connection (Ctrl-C to stop)")
    srv.close()


def run_standalone(sim: Simulation, seconds, realtime=True):
    """Run the physics without Renode (servo angles from the 'servo' command)."""
    print(f"Standalone plant for {seconds:.0f} s. Log: {sim.log_path}")
    start = time.time()
    next_status = 0.0
    while sim.running and sim.t < seconds:
        with sim.lock:
            sim.advance_to(sim.t + 0.02)
        if sim.t >= next_status:
            print(sim.status())
            next_status += 5.0
        if realtime:
            time.sleep(max(0.0, start + sim.t - time.time()))
