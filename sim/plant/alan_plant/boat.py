"""
3-degree-of-freedom sailboat model (surge, sway, yaw) with a free-rotating wing sail trimmed by a tail.

Frames and signs (match the team's conventions):
  - Body frame: x forward, y to starboard.
  - Heading psi: degrees clockwise from true north. Yaw rate r: positive turning to starboard.
  - AWA (apparent wind angle): where the wind comes FROM relative to the bow, positive from starboard.
  - Wing angle delta_w: wing chord relative to the hull centreline, positive leading edge to starboard.
  - Angle of attack alpha = AWA - delta_w.
  - Rudder angle: positive turns the boat to port (leading edge to starboard); flip with rudder_sign.
  - u, v are speeds through the WATER. Water current is added on top for motion over ground and apparent wind.

Every foil (wing, keel, rudder) uses the same flat-plate-plus-thin-airfoil model, so lift, drag and stall
come out of one function. Parameter defaults are guesses for a ~1 m boat: replace them with measured values.
"""
import math
from dataclasses import dataclass, field

from .geo import wrap180, wrap360

RHO_AIR = 1.2
RHO_WATER = 1000.0


@dataclass
class BoatParams:
    mass_kg: float = 4.0
    sway_mass_kg: float = 6.0          # mass plus added mass for sideways motion
    yaw_inertia: float = 0.5           # kg m^2

    wing_area_m2: float = 0.15
    wing_x_m: float = 0.05             # mast position, ahead of the centre of rotation
    wing_tau_s: float = 0.4            # how fast the wing swings to its trim angle
    tail_to_aoa: float = 0.5           # wing angle of attack (deg) per degree of tail deflection
    tail_limit_deg: float = 90.0          # the tail servo can swing 90 degrees each way
    wing_limit_deg: float = 90.0          # hard stops at the mast base: the wing swings +/-90 deg (180 total), not 360
    tail_sign: float = 1.0             # flip if a positive tail servo command gives negative angle of attack

    keel_area_m2: float = 0.02
    keel_x_m: float = 0.0

    rudder_area_m2: float = 0.008
    rudder_x_m: float = -0.45
    rudder_limit_deg: float = 35.0
    rudder_sign: float = 1.0           # flip if a positive rudder servo command should turn to starboard

    hull_drag_lin: float = 0.5         # N per m/s
    hull_drag_quad: float = 1.5        # N per (m/s)^2
    hull_sway_quad: float = 3.0
    yaw_damp_lin: float = 0.2          # N m per rad/s
    yaw_damp_quad: float = 0.3

    wing_ce_z_m: float = 0.75          # wing centre of effort above the waterline (heeling lever, top)
    clr_z_m: float = -0.15             # centre of lateral resistance below the waterline (heeling lever, bottom)
    vane_bow_x_m: float = 0.45         # wind vane mount positions (forward of the CG) and heights above the waterline
    vane_bow_z_m: float = 0.3
    vane_stern_x_m: float = -0.45
    vane_stern_z_m: float = 0.3

    servo_rate_dps: float = 450.0      # MG90S: about 0.1 s per 60 degrees
    servo_us_center: float = 1500.0    # must match the firmware's pulse mapping
    servo_us_per_90deg: float = 1000.0


@dataclass
class NoiseParams:
    heading_deg: float = 1.0
    yaw_rate_dps: float = 0.5
    encoder_deg: float = 0.2
    vane_deg: float = 2.0
    gps_m: float = 2.0
    declination_deg: float = 10.5      # magnetic declination; the IMU reports magnetic heading
    vane_on_wing: bool = False         # True if the wind vane rotates with the wing
    vane_mount: str = "stern"          # bow | stern | masthead | custom: where the wind vane sits (changes what it reads)
    vane_custom_x_m: float = 0.0       # custom mount: metres forward of the CG, and height above the waterline
    vane_custom_z_m: float = 0.4
    vane_upwash: float = 0.5           # how much of the wing's 2-D flow bending reaches the vane (finite wing, height)


@dataclass
class BoatState:
    north_m: float = 0.0
    east_m: float = 0.0
    heading_deg: float = 0.0
    u: float = 0.0                     # surge speed, m/s
    v: float = 0.0                     # sway speed, m/s (positive to starboard)
    r: float = 0.0                     # yaw rate, rad/s
    wing_deg: float = 0.0
    rudder_deg: float = 0.0
    tail_deg: float = 0.0
    rudder_cmd_deg: float = 0.0
    tail_cmd_deg: float = 0.0
    awa_deg: float = 0.0
    aws_ms: float = 0.0
    aoa_deg: float = 0.0
    heel_deg: float = 0.0              # positive = starboard side down
    wing_on_stop: int = 0              # -1 / +1 when the wing is pressed against the port / starboard hard stop
    vane_flow_deg: float = 0.0         # flow direction at the wind vane (relative to the bow), before vane dynamics
    vane_deg: float = 0.0              # where the vane actually points (it swings with some lag and overshoot)
    vane_rate: float = 0.0
    extra: dict = field(default_factory=dict)


def foil_force(flow_x, flow_y, chord_deg, area, rho, cd0=0.02):
    """
    Force on a foil from the fluid flowing past it.
    flow: fluid velocity relative to the foil (where the fluid is going), body frame.
    chord_deg: direction the leading edge points, body frame.
    """
    speed = math.hypot(flow_x, flow_y)
    if speed < 1e-6:
        return 0.0, 0.0
    c = math.radians(chord_deg)
    cx, cy = math.cos(c), math.sin(c)
    nx, ny = -cy, cx
    wn = flow_x * nx + flow_y * ny
    wt = flow_x * cx + flow_y * cy
    s = abs(wn) / speed                               # |sin(angle of attack)|
    cn = min(2.0 * math.pi * s, 0.8 + 0.4 * s)       # thin airfoil, then flat-plate stall
    q = 0.5 * rho * area * speed
    fn = q * speed * cn * math.copysign(1.0, wn)
    ft = q * cd0 * wt
    return fn * nx + ft * cx, fn * ny + ft * cy


class Boat:
    def __init__(self, params=None, noise=None, state=None, righting=None):
        """righting(heel_deg) -> righting moment N m (from the hydrostatics). Default: GM = 0.2 m."""
        self.p = params or BoatParams()
        self.noise = noise or NoiseParams()
        self.s = state or BoatState()
        self.righting = righting or (lambda h: self.p.mass_kg * 9.81 * 0.2 * math.sin(math.radians(h)))
        self.bench = False             # on the stand: hull held still, wing/tail/rudder/vane still work
        self.hold_wing = None          # sim helper: move the tail so the wing settles at this angle (deg, to the hull)

    # ---- actuators -------------------------------------------------------------------------

    def pulse_to_deg(self, pulse_us):
        """Servo angle the firmware is commanding, or None if the pulse is not a valid servo pulse."""
        if pulse_us is None or not (400 <= pulse_us <= 2600):
            return None
        return (pulse_us - self.p.servo_us_center) / self.p.servo_us_per_90deg * 90.0

    servo_override = False             # True: ignore the firmware's servo pulses (manual control from the dashboard)

    def set_servo_pulses(self, tail_us, rudder_us):
        """Servo pulses read from the firmware's PWM registers."""
        self.fw_tail_us, self.fw_rudder_us = tail_us, rudder_us
        if self.servo_override:
            return
        tail = self.pulse_to_deg(tail_us)
        rudder = self.pulse_to_deg(rudder_us)
        if tail is not None:
            self.s.tail_cmd_deg = tail
        if rudder is not None:
            self.s.rudder_cmd_deg = rudder

    @staticmethod
    def _slew(current, target, max_step):
        return current + max(-max_step, min(max_step, target - current))

    # ---- physics ---------------------------------------------------------------------------

    def tail_for_wing(self, wing_deg):
        """
        Tail angle that makes the free wing settle at wing_deg (to the hull) in the current apparent wind.
        The wing weathervanes until its angle of attack is alpha = tail_sign * tail_to_aoa * tail, and
        alpha = AWA - wing angle, so tail = (AWA - wing) / (tail_sign * tail_to_aoa).
        Returns (tail_deg, alpha_deg, reachable).
        """
        p = self.p
        alpha = wrap180(self.s.awa_deg - wing_deg)
        tail = alpha / (p.tail_sign * p.tail_to_aoa)
        return tail, alpha, abs(tail) <= p.tail_limit_deg and abs(wing_deg) <= p.wing_limit_deg

    def step(self, dt, wind, current_ne=(0.0, 0.0)):
        """Advance dt seconds. current_ne: water current velocity (where the water flows), m/s north and east."""
        p, s = self.p, self.s
        if self.hold_wing is not None:
            tail, _, _ = self.tail_for_wing(self.hold_wing)
            s.tail_cmd_deg = max(-p.tail_limit_deg, min(p.tail_limit_deg, tail))
        if self.bench:
            current_ne = (0.0, 0.0)
        servo_step = p.servo_rate_dps * dt
        s.tail_deg = self._slew(s.tail_deg, max(-p.tail_limit_deg, min(p.tail_limit_deg, s.tail_cmd_deg)), servo_step)
        s.rudder_deg = self._slew(s.rudder_deg, max(-p.rudder_limit_deg, min(p.rudder_limit_deg, s.rudder_cmd_deg)), servo_step)

        psi = math.radians(s.heading_deg)
        cos_p, sin_p = math.cos(psi), math.sin(psi)

        # Water current in the body frame
        cn, ce = current_ne
        cur_bx = cn * cos_p + ce * sin_p
        cur_by = -cn * sin_p + ce * cos_p

        # Apparent wind in the body frame (boat velocity over ground = through water + current)
        wn, we = wind.velocity_ne()
        wind_bx = wn * cos_p + we * sin_p
        wind_by = -wn * sin_p + we * cos_p
        air_x = wind_bx - (s.u + cur_bx)
        air_y = wind_by - (s.v + cur_by + s.r * p.wing_x_m)
        s.aws_ms = math.hypot(air_x, air_y)
        s.awa_deg = math.degrees(math.atan2(-air_y, -air_x)) if s.aws_ms > 1e-6 else 0.0

        # Self-trimming wing: it swings until its angle of attack matches what the tail asks for
        aoa_trim = p.tail_sign * p.tail_to_aoa * s.tail_deg
        target = s.awa_deg - aoa_trim
        if p.wing_limit_deg < 180.0:
            # With hard stops the wing cannot go "the short way round" through the stern; it swings toward the
            # target inside its +/- limit, and rests against a stop if the target is beyond it.
            t = wrap180(target)
            if abs(t) <= p.wing_limit_deg:
                want = t
            else:
                # The wing would like to point past a stop (wind from behind): it stays pressed on the stop it is
                # nearest to, instead of flipping across to the other stop every time the wind wobbles past dead aft.
                side = s.wing_deg if abs(s.wing_deg) > 1.0 else t
                want = math.copysign(p.wing_limit_deg, side)
            s.wing_deg += (want - s.wing_deg) * min(1.0, dt / p.wing_tau_s)
            s.wing_on_stop = (1 if s.wing_deg >= p.wing_limit_deg - 0.5 else -1 if s.wing_deg <= -p.wing_limit_deg + 0.5 else 0)
        else:
            s.wing_deg = wrap180(s.wing_deg + wrap180(target - s.wing_deg) * min(1.0, dt / p.wing_tau_s))
            s.wing_on_stop = 0
        s.aoa_deg = wrap180(s.awa_deg - s.wing_deg)

        fx, fy, n = 0.0, 0.0, 0.0

        # Wing force; heeling tilts the wing, so it presents less area to the wind
        heel_k = max(0.0, math.cos(math.radians(s.heel_deg)))
        wfx, wfy = foil_force(air_x, air_y, s.wing_deg, p.wing_area_m2 * heel_k, RHO_AIR)
        fx += wfx
        fy += wfy
        n += p.wing_x_m * wfy

        # Heel: the wing's side force times the lever between the wing's centre of effort and the keel's
        # centre of lateral resistance, against the righting moment from the hydrostatics. Quasi-static with lag.
        heel_moment = wfy * (p.wing_ce_z_m - p.clr_z_m)
        target = self._heel_for(heel_moment)
        s.heel_deg += (target - s.heel_deg) * min(1.0, dt / 0.6)

        if self.bench:
            heel_moment = 0.0
            s.heel_deg = 0.0
        self._update_vane(dt, wind_bx, wind_by, cur_bx, cur_by, air_x, air_y, wfx, wfy)

        kfx, kfy = foil_force(-s.u, -(s.v + s.r * p.keel_x_m), 0.0, p.keel_area_m2, RHO_WATER, cd0=0.01)
        fx += kfx
        fy += kfy
        n += p.keel_x_m * kfy

        rudder_chord = p.rudder_sign * s.rudder_deg
        rfx, rfy = foil_force(-s.u, -(s.v + s.r * p.rudder_x_m), rudder_chord, p.rudder_area_m2, RHO_WATER, cd0=0.01)
        fx += rfx
        fy += rfy
        n += p.rudder_x_m * rfy

        fx += -p.hull_drag_lin * s.u - p.hull_drag_quad * s.u * abs(s.u)
        fy += -p.hull_sway_quad * s.v * abs(s.v)
        n += -p.yaw_damp_lin * s.r - p.yaw_damp_quad * s.r * abs(s.r)

        # Rigid-body equations with the Coriolis coupling between surge, sway and yaw
        u_dot = (fx + p.sway_mass_kg * s.v * s.r) / p.mass_kg
        v_dot = (fy - p.mass_kg * s.u * s.r) / p.sway_mass_kg
        r_dot = n / p.yaw_inertia

        if self.bench:                 # the stand holds the hull: no surge, sway or yaw
            u_dot = v_dot = r_dot = 0.0
            s.u = s.v = s.r = 0.0
        s.u += u_dot * dt
        s.v += v_dot * dt
        s.r += r_dot * dt
        s.heading_deg = wrap360(s.heading_deg + math.degrees(s.r) * dt)

        psi = math.radians(s.heading_deg)
        s.north_m += (s.u * math.cos(psi) - s.v * math.sin(psi) + cn) * dt
        s.east_m += (s.u * math.sin(psi) + s.v * math.cos(psi) + ce) * dt
        self._current_ne = (cn, ce)

    def _heel_for(self, moment):
        """Heel angle where the righting moment balances a heeling moment (N m, positive = to starboard)."""
        if abs(moment) < 1e-4:
            return 0.0
        lo, hi = 0.0, 85.0
        m = abs(moment)
        if self.righting(hi) < m * math.cos(math.radians(hi)):
            return math.copysign(hi, moment)
        for _ in range(25):
            mid = (lo + hi) / 2
            # the heeling lever shortens as the boat leans (cos), the righting moment grows
            if self.righting(mid) < m * math.cos(math.radians(mid)):
                lo = mid
            else:
                hi = mid
        return math.copysign((lo + hi) / 2, moment)

    def _update_vane(self, dt, wind_bx, wind_by, cur_bx, cur_by, air_x, air_y, wfx, wfy):
        """
        What the wind vane sees at its mount: the wind is weaker low down (wind gradient), turning the boat moves
        the bow and stern sideways, and the wing bends the flow around it (upwash ahead, downwash behind).
        Then the vane itself swings to that direction with some lag and overshoot.
        """
        p, s, nz = self.p, self.s, self.noise
        mount = nz.vane_mount
        if mount == "masthead":
            xv, zv = p.wing_x_m, p.wing_ce_z_m * 2.0
        elif mount == "custom":
            xv, zv = nz.vane_custom_x_m, nz.vane_custom_z_m
        elif mount == "bow":
            xv, zv = p.vane_bow_x_m, p.vane_bow_z_m
        else:
            xv, zv = p.vane_stern_x_m, p.vane_stern_z_m
        grad = (max(0.05, zv) / max(0.05, p.wing_ce_z_m)) ** (1.0 / 7.0)   # wind speed vs height (1/7 power law)
        fx = wind_bx * grad - (s.u + cur_bx)
        fy = wind_by * grad - (s.v + cur_by + s.r * xv)

        # Wing as a vortex: circulation from its lift (Kutta-Joukowski), induced flow at the vane
        v2 = air_x * air_x + air_y * air_y
        dx = xv - p.wing_x_m
        if mount != "masthead" and v2 > 0.01 and abs(dx) > 0.05:
            span = 1.0
            gamma = (wfx * air_y - wfy * air_x) / (RHO_AIR * v2 * span)
            fy += nz.vane_upwash * gamma / (2.0 * math.pi * dx)
        s.vane_flow_deg = math.degrees(math.atan2(-fy, -fx))

        # Vane dynamics: a weathervane is a damped spring; it gets stiffer as the wind gets stronger
        spd = math.hypot(fx, fy)
        wn = 2.5 * spd + 0.5
        err = wrap180(s.vane_flow_deg - s.vane_deg)
        s.vane_rate += (wn * wn * math.radians(err) - 2 * 0.35 * wn * s.vane_rate) * dt
        s.vane_deg = wrap180(s.vane_deg + math.degrees(s.vane_rate) * dt)

    # ---- derived values --------------------------------------------------------------------

    def _ground_velocity_ne(self):
        psi = math.radians(self.s.heading_deg)
        cn, ce = getattr(self, "_current_ne", (0.0, 0.0))
        vn = self.s.u * math.cos(psi) - self.s.v * math.sin(psi) + cn
        ve = self.s.u * math.sin(psi) + self.s.v * math.cos(psi) + ce
        return vn, ve

    @property
    def sog_ms(self):
        """Speed over ground (what GPS measures), m/s."""
        return math.hypot(*self._ground_velocity_ne())

    @property
    def stw_ms(self):
        """Speed through the water, m/s."""
        return math.hypot(self.s.u, self.s.v)

    @property
    def cog_deg(self):
        vn, ve = self._ground_velocity_ne()
        return wrap360(math.degrees(math.atan2(ve, vn)))
