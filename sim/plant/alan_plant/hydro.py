"""
Hydrostatics from the voxel model of the boat (sim/dashboard/models/hydro_voxels.bin + mass.json).

Answers "how does it float?" without water:
  - centre of gravity from the part mass list
  - upright waterline and trim (bow up/down) where buoyancy balances weight
  - righting arm GZ and righting moment vs heel angle, metacentric height GM,
    deck-edge immersion angle and the angle where it would capsize (vanishing stability)

Coordinates: CAD millimetres (x port+, y aft+, z up). Heel positive = starboard side down.
Pure Python (no numpy), so the dashboard and plant can run it directly.
"""
import json
import math
import os
import struct

RHO_WATER = 1000.0
G = 9.81


def load_mass(models_dir):
    path = os.path.join(models_dir, "mass.json")
    with open(path) as f:
        cfg = json.load(f)
    items = cfg.get("items", [])
    total = sum(float(i["kg"]) for i in items)
    if total <= 0:
        raise ValueError("mass.json: total mass is zero")
    cg = [sum(float(i["kg"]) * float(i["at_mm"][k]) for i in items) / total for k in range(3)]
    return total, cg, items


class Hydrostatics:
    def __init__(self, models_dir):
        self.models_dir = models_dir
        with open(os.path.join(models_dir, "boat.json")) as f:
            self.boat = json.load(f)
        meta = self.boat["hydro"]
        self.cell = meta["cell_mm"]
        self.dv = (self.cell / 1000.0) ** 3                       # m^3 per voxel
        with open(os.path.join(models_dir, "hydro_voxels.bin"), "rb") as f:
            raw = f.read()
        flat = struct.unpack(f"<{len(raw) // 4}f", raw)
        self.vox = [(flat[i], flat[i + 1], flat[i + 2]) for i in range(0, len(flat), 3)]
        n0 = meta["parts"]["hull"]
        self.deck_z = max(v[2] for v in self.vox[:n0])
        self.hull_vox = n0
        # Deck edge: the top voxel of each hull column along the outer edge (gunwale), for the deck-immersion angle
        top = {}
        for x, y, z in self.vox[:n0]:
            k = (x, y)
            if k not in top or z > top[k]:
                top[k] = z
        half_beam = {}
        for (x, y) in top:
            half_beam[y] = max(half_beam.get(y, 0.0), abs(x))
        self.gunwale = [(x, y, z) for (x, y), z in top.items() if abs(x) >= half_beam[y] - self.cell]
        self.reload_mass()

    # ---- mass ------------------------------------------------------------------------------------

    def reload_mass(self):
        self.mass, self.cg, self.items = load_mass(self.models_dir)
        self.result = self._solve()

    # ---- core: float the voxels at a given heel and trim -----------------------------------------

    def _float(self, heel_deg, trim_deg):
        """Rotate about the CG, sink until displacement = mass. Returns waterline height and centre of buoyancy (rotated frame, mm)."""
        xg, yg, zg = self.cg
        ch, sh = math.cos(math.radians(heel_deg)), math.sin(math.radians(heel_deg))
        ct, st = math.cos(math.radians(trim_deg)), math.sin(math.radians(trim_deg))
        pts = []
        for x, y, z in self.vox:
            dx, dy, dz = x - xg, y - yg, z - zg
            # trim about the lateral axis (bow = -y goes down for positive trim)
            dy, dz = dy * ct - dz * st, dy * st + dz * ct
            # heel about the fore-aft axis (starboard = -x goes down for positive heel)
            dx, dz = dx * ch - dz * sh, dx * sh + dz * ch
            pts.append((dz, dx, dy))
        pts.sort()
        need = self.mass / RHO_WATER / self.dv                    # voxels to submerge
        n = int(need)
        frac = need - n
        if n >= len(pts):
            return None
        sx = sum(p[1] for p in pts[:n]) + frac * pts[n][1]
        sy = sum(p[2] for p in pts[:n]) + frac * pts[n][2]
        sz = sum(p[0] for p in pts[:n]) + frac * pts[n][0]
        wl = pts[n][0]
        return {"waterline": wl, "cb": (sx / need, sy / need, sz / need), "pts": pts, "n": n}

    def _solve(self):
        # Trim: rotate until the centre of buoyancy is straight under the CG fore-and-aft.
        # CB aft of the CG lifts the stern, so the bow goes down: increase the (bow-down) trim.
        lo, hi = -15.0, 15.0
        for _ in range(18):
            mid = (lo + hi) / 2
            if self._float(0.0, mid)["cb"][1] > 0:
                lo = mid
            else:
                hi = mid
        trim = (lo + hi) / 2
        up = self._float(0.0, trim)
        xg, yg, zg = self.cg
        wl_cad = up["waterline"] + zg                                  # waterline height in CAD z (level boat)

        # GZ curve: heel in 5 degree steps until the righting arm turns negative
        curve = []
        deck_edge = None
        vanish = None
        for heel in range(0, 181, 5):
            f = self._float(float(heel), trim)
            if f is None:
                break
            gz_mm = -f["cb"][0]                                        # CB on the low (starboard) side -> righting
            curve.append({"heel": heel, "gz_mm": round(gz_mm, 2), "moment_nm": round(self.mass * G * gz_mm / 1000, 3)})
            if deck_edge is None and heel > 0:
                # deck edge under: any gunwale point below the waterline
                ch, sh = math.cos(math.radians(heel)), math.sin(math.radians(heel))
                ct, st = math.cos(math.radians(trim)), math.sin(math.radians(trim))
                for x, y, z in self.gunwale:
                    dy, dz = (y - yg) * ct - (z - zg) * st, (y - yg) * st + (z - zg) * ct
                    if (x - xg) * sh + dz * ch < f["waterline"]:
                        deck_edge = heel
                        break
            if heel > 0 and gz_mm < 0 and vanish is None:
                vanish = heel
                break
        gm_mm = curve[1]["gz_mm"] / math.sin(math.radians(5)) if len(curve) > 1 else None

        lowest = min(v[2] for v in self.vox)
        hull_bottom = min(v[2] for v in self.vox[: self.hull_vox])
        return {
            "mass_kg": round(self.mass, 3),
            "cg_mm": [round(c, 1) for c in self.cg],
            "displacement_l": round(self.mass / RHO_WATER * 1000, 2),
            "waterline_z_mm": round(wl_cad, 1),
            "trim_deg": round(trim, 2),
            "draft_mm": round(wl_cad - lowest, 1),
            "hull_draft_mm": round(wl_cad - hull_bottom, 1),
            "freeboard_mm": round(self.deck_z - wl_cad, 1),
            "reserve_buoyancy_l": round(len(self.vox) * self.dv * 1000 - self.mass, 2),
            "cb_mm": [round(up["cb"][0] + xg, 1), round(up["cb"][1] + yg, 1), round(up["cb"][2] + zg, 1)],
            "gm_mm": round(gm_mm, 1) if gm_mm is not None else None,
            "deck_edge_deg": deck_edge,
            "vanishing_deg": vanish,
            "gz_curve": curve,
            "items": self.items,
        }

    # ---- helpers for the live physics and the dashboard ------------------------------------------

    def righting_moment(self, heel_deg):
        """Righting moment (N m) at a heel angle, interpolated from the GZ curve (sign follows the heel)."""
        c = self.result["gz_curve"]
        a = abs(heel_deg)
        for p, q in zip(c, c[1:]):
            if p["heel"] <= a <= q["heel"]:
                t = (a - p["heel"]) / (q["heel"] - p["heel"])
                m = p["moment_nm"] + t * (q["moment_nm"] - p["moment_nm"])
                return math.copysign(m, heel_deg)
        return math.copysign(c[-1]["moment_nm"], heel_deg)

    def silhouettes(self, heel_deg=0.0):
        """Occupied cells for drawing: front view (x, z) and side view (y, z), CAD mm, in the floating attitude."""
        res = self.result
        xg, yg, zg = self.cg
        ch, sh = math.cos(math.radians(heel_deg)), math.sin(math.radians(heel_deg))
        f = self._float(heel_deg, res["trim_deg"])
        front, side = set(), set()
        c = self.cell
        for x, y, z in self.vox:
            dx, dz = (x - xg) * ch - (z - zg) * sh, (x - xg) * sh + (z - zg) * ch
            front.add((round(dx / c), round((dz - f["waterline"]) / c)))
            side.add((round((y - yg) / c), round((z - zg - (res["waterline_z_mm"] - zg)) / c)))
        return {
            "cell_mm": c,
            "front": sorted(front),
            "side": sorted(side),
            "cb_front": [round(f["cb"][0], 1), round(f["cb"][2] - f["waterline"], 1)],
            "cg_front": [0.0, round(-f["waterline"], 1)],
        }
