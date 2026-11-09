"""Angle helpers and a local flat-earth frame (north/east metres around an origin)."""
import math

EARTH_RADIUS_M = 6_371_000.0


def wrap180(deg):
    """Wrap an angle to (-180, 180]."""
    deg = math.fmod(deg, 360.0)
    if deg <= -180.0:
        deg += 360.0
    elif deg > 180.0:
        deg -= 360.0
    return deg


def wrap360(deg):
    """Wrap an angle to [0, 360)."""
    deg = math.fmod(deg, 360.0)
    return deg + 360.0 if deg < 0.0 else deg


class LocalFrame:
    """Converts between lat/lon and metres north/east of an origin. Accurate to well under a metre over a lake."""

    def __init__(self, lat0, lon0):
        self.lat0 = lat0
        self.lon0 = lon0
        self._cos_lat0 = math.cos(math.radians(lat0))

    def to_latlon(self, north_m, east_m):
        lat = self.lat0 + math.degrees(north_m / EARTH_RADIUS_M)
        lon = self.lon0 + math.degrees(east_m / (EARTH_RADIUS_M * self._cos_lat0))
        return lat, lon

    def to_local(self, lat, lon):
        north = math.radians(lat - self.lat0) * EARTH_RADIUS_M
        east = math.radians(lon - self.lon0) * EARTH_RADIUS_M * self._cos_lat0
        return north, east
