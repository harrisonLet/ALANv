"""True wind: a mean direction and speed, plus random gusts and direction shifts."""
import math
import random


class Wind:
    """
    Wind the boat sails in.

    direction_deg: where the wind comes FROM, degrees from true north (meteorological convention)
    speed_ms:      mean speed, m/s
    gust_ms:       standard deviation of speed fluctuations, m/s
    shift_deg:     standard deviation of direction fluctuations, degrees
    tau_s:         how quickly gusts and shifts change (correlation time), seconds
    """

    def __init__(self, direction_deg=270.0, speed_ms=3.0, gust_ms=0.5, shift_deg=5.0, tau_s=8.0, seed=1):
        self.direction_deg = direction_deg
        self.speed_ms = speed_ms
        self.gust_ms = gust_ms
        self.shift_deg = shift_deg
        self.tau_s = tau_s
        self._rng = random.Random(seed)
        self._gust = 0.0
        self._shift = 0.0

    def step(self, dt):
        """Advance the gust and shift processes (Ornstein-Uhlenbeck)."""
        a = math.exp(-dt / self.tau_s)
        b = math.sqrt(1.0 - a * a)
        self._gust = a * self._gust + b * self.gust_ms * self._rng.gauss(0.0, 1.0)
        self._shift = a * self._shift + b * self.shift_deg * self._rng.gauss(0.0, 1.0)

    @property
    def current_direction_deg(self):
        return self.direction_deg + self._shift

    @property
    def current_speed_ms(self):
        return max(0.0, self.speed_ms + self._gust)

    def velocity_ne(self):
        """Velocity of the air (where it is going), metres/second north and east."""
        d = math.radians(self.current_direction_deg)
        s = self.current_speed_ms
        return -s * math.cos(d), -s * math.sin(d)


class Current:
    """Water current: the direction it flows TOWARD (nautical "set"), degrees true, and its speed (m/s)."""

    def __init__(self, direction_deg=0.0, speed_ms=0.0):
        self.direction_deg = direction_deg
        self.speed_ms = speed_ms

    def velocity_ne(self):
        d = math.radians(self.direction_deg)
        return self.speed_ms * math.cos(d), self.speed_ms * math.sin(d)
