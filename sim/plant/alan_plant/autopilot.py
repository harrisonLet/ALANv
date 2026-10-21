"""
Route planning and a stand-in autopilot for the simulator.

plan_route():  point A -> point B in the current wind. Straight if B can be sailed directly; otherwise a zig-zag
               of tacks (upwind) or gybes (dead downwind) that stays inside a corridor around the A-B line.
Autopilot:     follows the waypoints. Heading: a PD controller on the rudder. Sail: the tail is set so the wing
               meets the apparent wind at a target angle of attack on the correct side (feathered in irons).

This is NOT the team's firmware. It shows what the Live Tack Planner + rudder/sail PIDs should achieve, so the
firmware can be compared against it.
"""
import math

from .geo import wrap180, wrap360

NO_GO_DEG = 45.0          # closest the boat sails to the true wind
RUN_LIMIT_DEG = 160.0     # beyond this off the wind, gybe downwind instead of running dead downwind
ARRIVE_M = 3.0            # waypoint reached when this close


def _bearing(n0, e0, n1, e1):
    return wrap360(math.degrees(math.atan2(e1 - e0, n1 - n0)))


def plan_route(a, b, twd, margin_m=12.0, no_go=NO_GO_DEG, run_limit=RUN_LIMIT_DEG):
    """
    a, b: (north_m, east_m). twd: true wind FROM direction (deg). margin_m: corridor half-width.
    Returns {"waypoints": [(n, e), ...] (A first, B last), "mode": "direct"|"beat"|"gybe", "legs": n}.
    """
    dn, de = b[0] - a[0], b[1] - a[1]
    dist = math.hypot(dn, de)
    if dist < 1e-6:
        return {"waypoints": [a, b], "mode": "direct", "legs": 1}
    brg = _bearing(a[0], a[1], b[0], b[1])
    off = wrap180(brg - twd)
    if abs(off) >= no_go and abs(off) <= run_limit:
        return {"waypoints": [a, b], "mode": "direct", "legs": 1}

    upwind = abs(off) < no_go
    half = (no_go + 2.0) if upwind else (180.0 - run_limit + 5.0)
    base = twd if upwind else wrap360(twd + 180.0)
    h1, h2 = wrap360(base + half), wrap360(base - half)            # the two sailable headings
    u1 = (math.cos(math.radians(h1)), math.sin(math.radians(h1)))
    u2 = (math.cos(math.radians(h2)), math.sin(math.radians(h2)))
    det = u1[0] * u2[1] - u1[1] * u2[0]
    k1 = (dn * u2[1] - de * u2[0]) / det                           # A->B = k1*u1 + k2*u2
    k2 = (u1[0] * de - u1[1] * dn) / det
    # sideways reach of one full leg on each tack, measured from the A-B line
    ab = (dn / dist, de / dist)
    side = lambda u, k: abs(k * (u[0] * ab[1] - u[1] * ab[0]))
    reach = max(side(u1, k1), side(u2, k2), 1e-6)
    n = max(1, math.ceil(reach / max(1.0, margin_m)))              # split into n zig-zags so each stays in the corridor
    pts = [a]
    n_, e_ = a
    first, second = (u1, k1 / n), (u2, k2 / n)
    if abs(off) > 1e-9 and (off > 0) != (wrap180(h1 - brg) > 0):   # start on the tack that points closer to B
        first, second = second, first
    for _ in range(n):
        for u, k in (first, second):
            n_, e_ = n_ + u[0] * k, e_ + u[1] * k
            pts.append((n_, e_))
    pts[-1] = b
    return {"waypoints": pts, "mode": "beat" if upwind else "gybe", "legs": 2 * n, "headings": [h1, h2]}


class Autopilot:
    def __init__(self, waypoints, aoa_deg=12.0):
        self.waypoints = list(waypoints)
        self.index = 1 if len(self.waypoints) > 1 else 0
        self.aoa = aoa_deg
        self.active = True
        self.done = False
        self.status = "sailing"
        self.target_heading = None

    def update(self, boat, wind, dt):
        if not self.active or self.done:
            return
        s, p = boat.s, boat.p
        boat.servo_override = True                 # the autopilot drives the servos (firmware PWM ignored)
        wp = self.waypoints[self.index]
        dist = math.hypot(wp[0] - s.north_m, wp[1] - s.east_m)
        while dist < ARRIVE_M:
            if self.index >= len(self.waypoints) - 1:
                self.done = True
                self.status = "arrived"
                s.tail_cmd_deg = 0.0
                s.rudder_cmd_deg = 0.0
                boat.servo_override = False
                return
            self.index += 1
            wp = self.waypoints[self.index]
            dist = math.hypot(wp[0] - s.north_m, wp[1] - s.east_m)

        twd = wind.current_direction_deg
        brg = _bearing(s.north_m, s.east_m, wp[0], wp[1])
        off = wrap180(brg - twd)
        if abs(off) < NO_GO_DEG:
            # the wind shifted and this leg is no longer sailable: close-hauled on the side nearer the waypoint
            side = 1.0 if off >= 0 else -1.0
            want = wrap360(twd + side * (NO_GO_DEG + 3.0))
            self.status = "beating (leg not sailable after a wind shift)"
        elif abs(off) > RUN_LIMIT_DEG:
            side = 1.0 if off >= 0 else -1.0
            want = wrap360(twd + side * (RUN_LIMIT_DEG - 3.0))
            self.status = "broad reaching (avoiding a dead run)"
        else:
            want = brg
            self.status = f"leg {self.index} of {len(self.waypoints) - 1}"
        self.target_heading = want

        # rudder: PD on heading (positive rudder turns to port with rudder_sign = 1)
        err = wrap180(want - s.heading_deg)
        rate = math.degrees(s.r)
        cmd = 1.2 * err - 0.35 * rate
        s.rudder_cmd_deg = max(-30.0, min(30.0, -p.rudder_sign * cmd))

        # sail: angle of attack on the side the wind comes from; more when broad reaching; feathered in irons
        gain = p.tail_sign * p.tail_to_aoa
        if abs(s.awa_deg) < 25.0:
            alpha = 0.0
        else:
            alpha = math.copysign(self.aoa if abs(s.awa_deg) < 110 else min(40.0, self.aoa * 2.5), s.awa_deg)
        s.tail_cmd_deg = max(-p.tail_limit_deg, min(p.tail_limit_deg, alpha / gain))

    def to_dict(self):
        return {"waypoints": self.waypoints, "index": self.index, "done": self.done, "active": self.active,
                "status": self.status, "target_heading": self.target_heading, "aoa": self.aoa}
