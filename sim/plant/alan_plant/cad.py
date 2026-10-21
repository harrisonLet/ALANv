"""
Read the boat's CAD description (sim/dashboard/models/boat.json + STL files) and turn it into physics parameters.

boat.json describes the CAD frame once (units, which axis is forward and up), the centre of gravity, the
waterline and the hinge points. Every STL is exported in the SAME assembly coordinates, so parts are already in
the right place relative to each other; nothing has to be oriented by hand.

From the STLs it measures, in the boat's own frame (x forward from the CG):
  - wing, keel and rudder lateral (side-view) area  -> wing_area_m2, keel_area_m2, rudder_area_m2
  - mast and rudder post positions                  -> wing_x_m, rudder_x_m
  - keel centre                                     -> keel_x_m
  - hull volume (for a displacement check)
"""
import json
import math
import os
import struct

UNIT_SCALE = {"mm": 0.001, "cm": 0.01, "m": 1.0, "in": 0.0254}
AXES = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def read_stl(path):
    """Triangles from a binary or ASCII STL, as a list of (v0, v1, v2)."""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + n * 50 == len(data):
            tris = []
            for i in range(n):
                v = struct.unpack_from("<12f", data, 84 + i * 50)
                tris.append((v[3:6], v[6:9], v[9:12]))
            return tris
    tris, verts = [], []
    for line in data.decode("utf-8", "ignore").splitlines():
        parts = line.split()
        if parts[:1] == ["vertex"]:
            verts.append(tuple(float(x) for x in parts[1:4]))
            if len(verts) == 3:
                tris.append(tuple(verts))
                verts = []
    return tris


class CadFrame:
    """Converts CAD coordinates to the boat frame: x forward, y starboard, z up, metres, origin at the CG on the waterline."""

    def __init__(self, cfg):
        self.scale = UNIT_SCALE[cfg.get("units", "mm")]
        self.f = AXES[cfg.get("forward", "+x")]
        self.u = AXES[cfg.get("up", "+z")]
        if abs(_dot(self.f, self.u)) > 1e-9:
            raise ValueError("boat.json: 'forward' and 'up' must be different axes")
        self.s = _cross(self.f, self.u)                      # starboard
        cg = cfg.get("cg")
        if cg is None:
            raise ValueError("boat.json: 'cg' (centre of gravity, CAD coordinates) is required")
        wl = cfg.get("waterline")
        up_cg = _dot(cg, self.u)
        # origin: the CG moved straight up/down onto the waterline
        shift = (wl - up_cg) if wl is not None else 0.0
        self.origin = tuple(cg[i] + self.u[i] * shift for i in range(3))

    def to_body(self, p):
        d = _sub(p, self.origin)
        return (_dot(d, self.f) * self.scale, _dot(d, self.s) * self.scale, _dot(d, self.u) * self.scale)


def measure(tris, frame: CadFrame):
    """Lateral area (m^2), centroid (body frame, m), volume (m^3) and bounding box of a mesh."""
    lat_area, area_sum, cx, cy, cz, vol = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for t in tris:
        a, b, c = (frame.to_body(v) for v in t)
        n = _cross(_sub(b, a), _sub(c, a))
        area = 0.5 * math.sqrt(_dot(n, n))
        lat_area += 0.5 * abs(n[1])                 # area projected on the side (x-z) plane
        area_sum += area
        cx += area * (a[0] + b[0] + c[0]) / 3
        cy += area * (a[1] + b[1] + c[1]) / 3
        cz += area * (a[2] + b[2] + c[2]) / 3
        vol += _dot(a, _cross(b, c)) / 6.0
        for p in (a, b, c):
            for i in range(3):
                lo[i] = min(lo[i], p[i])
                hi[i] = max(hi[i], p[i])
    centroid = (cx / area_sum, cy / area_sum, cz / area_sum) if area_sum else (0, 0, 0)
    # A closed surface projects twice (both sides), so halve it to get the side-view area
    return {"lateral_area_m2": lat_area / 2, "centroid": centroid, "volume_m3": abs(vol), "min": lo, "max": hi}


def load(models_dir):
    """
    Returns (physics overrides for BoatParams, report lines, Hydrostatics or None).
    Empty if there is no boat.json. Mass, CG and waterline come from mass.json via the hydrostatics.
    """
    path = os.path.join(models_dir, "boat.json")
    if not os.path.exists(path):
        return {}, [], None
    with open(path) as f:
        cfg = json.load(f)
    out, report = {}, []

    hydro = None
    if os.path.exists(os.path.join(models_dir, "hydro_voxels.bin")) and os.path.exists(os.path.join(models_dir, "mass.json")):
        from .hydro import Hydrostatics
        hydro = Hydrostatics(models_dir)
        r = hydro.result
        cfg["cg"] = r["cg_mm"]
        cfg["waterline"] = r["waterline_z_mm"]
        out["mass_kg"] = r["mass_kg"]
        out["sway_mass_kg"] = 1.5 * r["mass_kg"]
        report.append(f"mass {r['mass_kg']} kg, CG {r['cg_mm']} mm, waterline z={r['waterline_z_mm']} mm, "
                      f"trim {r['trim_deg']} deg, GM {r['gm_mm']} mm, deck edge under at {r['deck_edge_deg']} deg")
    elif "mass_kg" in cfg:
        out["mass_kg"] = float(cfg["mass_kg"])
        out["sway_mass_kg"] = 1.5 * out["mass_kg"]
    if "yaw_inertia" in cfg:
        out["yaw_inertia"] = float(cfg["yaw_inertia"])
    frame = CadFrame(cfg)
    parts = cfg.get("parts", {})

    pivots = dict(cfg.get("pivots", {}))
    for name in ("wing", "rudder"):
        if parts.get(name, {}).get("pivot"):
            pivots[name] = parts[name]["pivot"]
    for name in ("wing", "rudder"):
        if name in pivots:
            out[f"{name}_x_m"] = frame.to_body(pivots[name])[0]
    for mount, p in cfg.get("vane_mounts", {}).items():
        b = frame.to_body(p)
        out[f"vane_{mount}_x_m"] = b[0]
        out[f"vane_{mount}_z_m"] = b[2]

    measured = {}
    for name, p in parts.items():
        file = p.get("file")
        if not file or not os.path.exists(os.path.join(models_dir, file)):
            continue
        measured[name] = measure(read_stl(os.path.join(models_dir, file)), frame)
        m = measured[name]
        size = [m["max"][i] - m["min"][i] for i in range(3)]
        report.append(f"{name:7s} length {size[0]*1000:6.0f} mm  beam {size[1]*1000:5.0f} mm  height {size[2]*1000:5.0f} mm  "
                      f"side area {m['lateral_area_m2']*1e4:7.1f} cm^2  centre x {m['centroid'][0]*1000:+6.0f} mm")

    # The keel is exported as fin + bulb for colouring; for the physics they act together
    if "keel" in measured and "bulb" in measured:
        k, b = measured["keel"], measured["bulb"]
        a = k["lateral_area_m2"] + b["lateral_area_m2"]
        measured["keel"] = dict(k, lateral_area_m2=a, centroid=tuple(
            (k["centroid"][i] * k["lateral_area_m2"] + b["centroid"][i] * b["lateral_area_m2"]) / a for i in range(3)))
    for name in ("wing", "keel", "rudder"):
        if name in measured:
            out[f"{name}_area_m2"] = measured[name]["lateral_area_m2"]
    if "keel" in measured:
        out["keel_x_m"] = measured["keel"]["centroid"][0]
    if "wing" in measured:
        out["wing_ce_z_m"] = measured["wing"]["centroid"][2]          # centre of effort height above the waterline
    if "keel" in measured:
        out["clr_z_m"] = measured["keel"]["centroid"][2]              # centre of lateral resistance (below: negative)
    if "hull" in measured:
        v = measured["hull"]["volume_m3"]
        report.append(f"hull volume {v*1e3:.2f} L (fully submerged it could float {v*1000:.1f} kg)")
    report.append("physics from CAD: " + ", ".join(f"{k}={v:.4g}" for k, v in out.items()))
    return out, report, hydro
