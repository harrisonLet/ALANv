"""
Points of sail: sail the physics model at fixed headings and record how the tail sets the wing at each one.

For every true wind angle (TWA) the boat is held on that heading (yaw locked, as if the rudder were perfect),
the tail is set to give the wing a target angle of attack on the correct side, and the model runs until it
settles. The result is a polar (speed vs TWA) plus the wing and tail angles a controller would use.
"""
import copy
import math

from .boat import Boat, BoatState
from .geo import wrap180
from .wind import Wind

NO_GO_DEG = 25.0      # closer than this to the wind the wing is feathered (no drive)

POINTS = [(0, 30, "In irons"), (30, 60, "Close hauled"), (60, 80, "Close reach"), (80, 100, "Beam reach"),
          (100, 150, "Broad reach"), (150, 181, "Running")]


def point_of_sail(twa):
    a = abs(twa)
    for lo, hi, name in POINTS:
        if lo <= a < hi:
            return name
    return "Running"


def sail_at(params, righting, twa, tws, aoa, seconds=24.0, dt=0.03):
    """Steady state at one heading. Wind comes from north (0 deg); the boat heads TWA (positive = starboard tack)."""
    b = Boat(copy.deepcopy(params), None, BoatState(heading_deg=twa % 360, wing_deg=wrap180(-twa)), righting)
    w = Wind(0.0, tws, gust_ms=0.0, shift_deg=0.0)
    gain = params.tail_sign * params.tail_to_aoa
    t = 0.0
    while t < seconds:
        s = b.s
        side = 1.0 if s.awa_deg >= 0 else -1.0
        want = 0.0 if abs(s.awa_deg) < NO_GO_DEG else side * aoa
        s.tail_cmd_deg = max(-params.tail_limit_deg, min(params.tail_limit_deg, want / gain))
        s.rudder_cmd_deg = 0.0
        b.step(dt, w)
        s.r = 0.0
        s.heading_deg = twa % 360
        t += dt
    s = b.s
    speed = s.u
    return {
        "twa": twa, "point": point_of_sail(twa), "speed_ms": round(speed, 3),
        "vmg_ms": round(speed * math.cos(math.radians(twa)), 3),
        "leeway_deg": round(math.degrees(math.atan2(s.v, max(0.05, s.u))), 1),
        "awa_deg": round(s.awa_deg, 1), "aws_ms": round(s.aws_ms, 2),
        "wing_deg": round(s.wing_deg, 1), "aoa_deg": round(s.aoa_deg, 1),
        "tail_deg": round(s.tail_deg, 1), "heel_deg": round(s.heel_deg, 1), "aoa_target": aoa,
    }


def polar(params, righting, tws=3.0, aoa=12.0, optimize=False, step=15):
    out = []
    for twa in range(0, 181, step):
        if optimize and twa >= NO_GO_DEG:
            best = None
            for a in (4, 7, 10, 13, 16, 20, 25, 32, 40, 45):     # the 90-degree tail allows up to ~45 deg AoA
                r = sail_at(params, righting, twa, tws, a, seconds=16.0, dt=0.04)
                if best is None or r["speed_ms"] > best["speed_ms"]:
                    best = r
            out.append(sail_at(params, righting, twa, tws, best["aoa_target"]))
        else:
            out.append(sail_at(params, righting, twa, tws, aoa))
    return out
