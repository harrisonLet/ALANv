"""Turns the true boat state into what each sensor would report: noise, magnetic declination, dropouts."""
import math
import random

from .geo import LocalFrame, wrap360


class Sensors:
    def __init__(self, boat, frame: LocalFrame, seed=2):
        self.boat = boat
        self.frame = frame
        self.rng = random.Random(seed)
        self.gps_ok = True
        self.vane_ok = True
        self._gps_bias_n = 0.0
        self._gps_bias_e = 0.0

    def _g(self, sigma):
        return self.rng.gauss(0.0, sigma) if sigma > 0 else 0.0

    def read(self, dt):
        s, n = self.boat.s, self.boat.noise

        # GPS error wanders slowly, like a real receiver (correlation time 30 s)
        a = math.exp(-dt / 30.0)
        b = math.sqrt(1.0 - a * a) * n.gps_m
        self._gps_bias_n = a * self._gps_bias_n + b * self.rng.gauss(0.0, 1.0)
        self._gps_bias_e = a * self._gps_bias_e + b * self.rng.gauss(0.0, 1.0)
        lat, lon = self.frame.to_latlon(s.north_m + self._gps_bias_n, s.east_m + self._gps_bias_e)

        vane = s.vane_deg - (s.wing_deg if n.vane_on_wing else 0.0)

        return {
            "heading": wrap360(s.heading_deg - n.declination_deg + self._g(n.heading_deg)),
            "yaw_rate": math.degrees(s.r) + self._g(n.yaw_rate_dps),
            "wing": wrap360(s.wing_deg + self._g(n.encoder_deg)),
            "vane": wrap360(vane + self._g(n.vane_deg)),
            "vane_ok": 1 if self.vane_ok else 0,
            "lat": lat,
            "lon": lon,
            "gps_fix": 1 if self.gps_ok else 0,
            "gps_ok": 1 if self.gps_ok else 0,
        }
