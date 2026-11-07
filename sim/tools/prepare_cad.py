#!/usr/bin/env python3
"""
Turn the team's CAD exports into simulator parts (sim/dashboard/models/*.stl + boat.json).

  sim/.venv/bin/python sim/tools/prepare_cad.py                 read the exports from ~/Downloads
  sim/.venv/bin/python sim/tools/prepare_cad.py --src DIR --mass 4.2 --cg-y 35

What it does (re-run it whenever the CAD changes):
  - Vaka (hull) and Keel are used as exported: they share the assembly coordinates.
  - Sail v3 is split into wing (+ mast), tail, and booms, then raised so the wing sits on the deck
    (in the export the wing starts at the hull bottom; see SAIL_RAISE_MM).
  - Rudder v5 is split into the fixed transom bracket and the moving blade + post, and moved onto the transom.
  - The rudder mount plate and electronics box are moved into place (they were exported in part coordinates).
  - The underwater parts (hull envelope, keel, rudder) are filled with 8 mm voxels (hydro_voxels.bin) so the
    simulator can compute buoyancy, waterline, trim and righting moment at any heel (sim/plant/alan_plant/hydro.py).
  - mass.json (part masses -> centre of gravity) is created with estimates if it does not exist yet. Edit it,
    or edit the masses in the dashboard's Stability tab.

CAD frame of the exports: millimetres, +z up, bow toward -y, starboard toward -x.
Everything placed by an estimate is marked ESTIMATE below: check it against the real boat.
"""
import argparse
import json
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "dashboard", "models")

FILES = {
    "hull": "Vaka v2 (1).stl",
    "keel": "Keel v2 (1).stl",
    "sail": "Sail v3 (1).stl",
    "rudder": "Rudder v5 (1).stl",
    "plate": "Rudder Mount (Plate) (1).stl",
    "electronics": "Electronics Box V2 (1).stl",
}

# Offsets that move each part export onto the mated assembly ("Sail Boat (14).stl"), measured by matching bodies.
SAIL_RAISE_MM = 252.6          # wing, tail and all three booms agree to 0.2 mm
SAIL_SHIFT_Y_MM = -139.2
KEEL_MOVE_MM = (0.0, 1.6, -6.3)             # keel top plate sits under the hull bottom
ELECTRONICS_MOVE_MM = (92.7, 160.0, 82.4)
TRANSOM_Y_MM = 467.9           # aft face of the hull (max y of the Vaka export); used only without the assembly
RUDDER_RAISE_MM = 155.3        # used only without the assembly
RUDDER_POST_Y_MM = 15.0
TAIL_HINGE_Y_MM = 421.6 + SAIL_SHIFT_Y_MM   # tail hinge at the end of the tail booms (they reach 58 mm into the tail)
FILES_ASSEMBLY = "Sail Boat (14).stl"       # mated assembly: rudder pieces and deck hardware come from here
VANE_MOUNTS_MM = {"bow": (0.0, -470.0, 380.0), "stern": (0.0, 430.0, 380.0)}  # ESTIMATE: vane on a short pole
ONSHAPE_YAW_INERTIA = 74877.219 * 0.45359237e-6   # Lzz from Onshape (lb mm^2 -> kg m^2), about the CG


def load(path):
    d = open(path, "rb").read()
    n = struct.unpack_from("<I", d, 80)[0]
    if 84 + n * 50 == len(d):
        a = np.frombuffer(d, dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n, offset=84)
        return a["v"].reshape(-1, 3, 3).astype(np.float64)
    vs = [list(map(float, l.split()[1:4])) for l in d.decode("utf8", "ignore").splitlines() if l.strip().startswith("vertex")]
    return np.array(vs).reshape(-1, 3, 3)


def save(path, tris):
    tris = np.asarray(tris, dtype=np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    rec["n"] = n
    rec["v"] = tris.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"ALAN simulator part".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def decimate(tris, cell=2.0):
    """Lighter display mesh: snap vertices to a grid (vertex clustering) and drop collapsed triangles."""
    v = tris.reshape(-1, 3)
    key = np.round(v / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1, 3)
    ok = (inv[:, 0] != inv[:, 1]) & (inv[:, 1] != inv[:, 2]) & (inv[:, 0] != inv[:, 2])
    # representative position of each cluster: the mean of its vertices
    sums = np.zeros((inv.max() + 1, 3))
    np.add.at(sums, inv.ravel(), v)
    counts = np.bincount(inv.ravel())[:, None]
    rep = sums / counts
    out = rep[inv[ok]]
    # drop exact duplicate triangles
    _, first = np.unique(np.sort(inv[ok], axis=1), axis=0, return_index=True)
    return out[np.sort(first)]


def components(tris):
    """Label each triangle with the connected body it belongs to (bodies share no vertices)."""
    v = np.round(tris.reshape(-1, 3), 3)
    _, inv = np.unique(v, axis=0, return_inverse=True)
    inv = inv.reshape(-1, 3)
    parent = np.arange(inv.max() + 1)

    def find(a):
        root = a
        while parent[root] != root:
            root = parent[root]
        while parent[a] != root:
            parent[a], a = root, parent[a]
        return root

    for a, b, c in inv:
        ra = find(a)
        parent[find(b)] = ra
        parent[find(c)] = ra
    return np.array([find(i) for i in inv[:, 0]])


def surface_samples(tris, spacing=3.0):
    """Points spread over a mesh surface about every `spacing` mm (vertices alone miss big flat faces)."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    k = np.clip(np.ceil(np.sqrt(area) / spacing), 1, 60).astype(int)
    pts = [tris.reshape(-1, 3)]
    for kk in np.unique(k):
        sel = k == kk
        i, j = np.meshgrid(np.arange(kk + 1), np.arange(kk + 1))
        m = (i + j) <= kk
        u, v = (i[m] / kk)[None, :, None], (j[m] / kk)[None, :, None]
        A, B, C = a[sel][:, None], b[sel][:, None], c[sel][:, None]
        pts.append((A + u * (B - A) + v * (C - A)).reshape(-1, 3))
    return np.concatenate(pts)


def column_voxels(tris, cell, extra_top=None):
    """
    Fill a body as vertical columns: in every (x, y) cell, solid from the lowest to the highest surface point.
    Right for hulls (one bottom, one deck) and fins. Returns voxel centres (mm).
    """
    pts = surface_samples(tris)
    if extra_top is not None:
        pts = np.concatenate([pts, surface_samples(extra_top)])
    ij = np.floor(pts[:, :2] / cell).astype(np.int64)
    key = ij[:, 0] * 100000 + ij[:, 1]
    order = np.argsort(key)
    key, z, ij = key[order], pts[order, 2], ij[order]
    starts = np.r_[0, np.flatnonzero(np.diff(key)) + 1]
    zmin = np.minimum.reduceat(z, starts)
    zmax = np.maximum.reduceat(z, starts)
    cells = ij[starts]
    out = []
    for (ix, iy), lo, hi in zip(cells, zmin, zmax):
        zs = np.arange(np.floor(lo / cell) * cell + cell / 2, hi, cell)
        zs = zs[zs >= lo]
        if len(zs):
            out.append(np.column_stack([np.full(len(zs), (ix + 0.5) * cell), np.full(len(zs), (iy + 0.5) * cell), zs]))
    return np.concatenate(out) if out else np.zeros((0, 3))


def submerged_volume(tris, h):
    """Volume of a closed mesh below the plane z = h (clip each triangle; reference point on the plane)."""
    ref = np.array([0.0, 0.0, h])
    vol = 0.0
    below = tris[:, :, 2] <= h
    full = below.all(1)
    t = tris[full] - ref
    vol += np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6.0
    for tri, b in zip(tris[below.any(1) & ~full], below[below.any(1) & ~full]):
        poly = []
        for i in range(3):
            p, q = tri[i], tri[(i + 1) % 3]
            if b[i]:
                poly.append(p)
            if b[i] != b[(i + 1) % 3]:
                s = (h - p[2]) / (q[2] - p[2])
                poly.append(p + s * (q - p))
        for k in range(1, len(poly) - 1):
            a, bb, c = poly[0] - ref, poly[k] - ref, poly[k + 1] - ref
            vol += np.dot(a, np.cross(bb, c)) / 6.0
    return abs(vol)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default=os.path.expanduser("~/Downloads"))
    ap.add_argument("--cg-y", type=float, default=35.0, help="fallback CG along the hull (mm, CAD y) if mass.json is missing")
    ap.add_argument("--cg-z", type=float, default=120.0, help="fallback CG height (mm, CAD z)")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    src = lambda k: os.path.join(args.src, FILES[k])
    for k in FILES:
        if not os.path.exists(src(k)):
            raise SystemExit(f"missing {src(k)}")

    hull, keel = load(src("hull")), load(src("keel")) + np.array(KEEL_MOVE_MM)
    # Display meshes: the hull export holds the shell, the foam deck and small fittings; colour them separately.
    hcomp = components(hull)
    hids, hcounts = np.unique(hcomp, return_counts=True)
    order = hids[np.argsort(-hcounts)]
    shell_id, deck_id = order[0], order[1]
    save(os.path.join(OUT, "hull.stl"), decimate(hull[hcomp == shell_id], 2.5))
    save(os.path.join(OUT, "deck.stl"), decimate(hull[hcomp == deck_id], 2.5))
    save(os.path.join(OUT, "hull_fittings.stl"), decimate(hull[(hcomp != shell_id) & (hcomp != deck_id)], 1.0))
    # Keel: white fin, metal bulb (split at the top of the bulb)
    kc = keel.mean(1)[:, 2]
    bulb_top = -180.0
    save(os.path.join(OUT, "keel.stl"), decimate(keel[kc >= bulb_top], 1.5))
    save(os.path.join(OUT, "bulb.stl"), decimate(keel[kc < bulb_top], 1.5))

    # Sail: wing (+mast) is the biggest body; the tail is the body behind the transom with depth; the rest are booms
    sail = load(src("sail"))
    sail[:, :, 2] += SAIL_RAISE_MM
    sail[:, :, 1] += SAIL_SHIFT_Y_MM
    comp = components(sail)
    ids, counts = np.unique(comp, return_counts=True)
    wing_id = ids[np.argmax(counts)]
    # tail: the tallest body that is not the wing (booms are thin rods)
    tail_id, tallest = None, 0.0
    for i in ids:
        v = sail[comp == i].reshape(-1, 3)
        h = v[:, 2].max() - v[:, 2].min()
        if i != wing_id and h > max(200.0, tallest):
            tail_id, tallest = i, h
    save(os.path.join(OUT, "wing.stl"), sail[comp == wing_id])
    save(os.path.join(OUT, "tail.stl"), sail[comp == tail_id])
    booms_tris = sail[(comp != wing_id) & (comp != tail_id)]
    save(os.path.join(OUT, "booms.stl"), booms_tris)
    wing_v = sail[comp == wing_id].reshape(-1, 3)
    mast_xy = (0.0, float(np.round(wing_v[(np.abs(wing_v[:, 0]) < 6) & (np.abs(wing_v[:, 1] - SAIL_SHIFT_Y_MM) < 8)][:, 1].mean(), 1)))
    wing_top = float(wing_v[:, 2].max())

    asm_path = os.path.join(args.src, FILES_ASSEMBLY)
    if os.path.exists(asm_path):
        # Rudder and deck hardware straight from the mated assembly
        asm = load(asm_path)
        acomp = components(asm)
        bodies = []
        for i in np.unique(acomp):
            t = asm[acomp == i]
            v = t.reshape(-1, 3)
            bodies.append((t, v.min(0), v.max(0)))
        # the rudder blade + post: the tall narrow body behind the transom
        blade = max((b for b in bodies if b[1][1] > 460 and (b[2][2] - b[1][2]) > 250), key=lambda b: b[2][2] - b[1][2])[0]
        bv = blade.reshape(-1, 3)
        top = bv[bv[:, 2] > bv[:, 2].max() - 60]                    # post near the top
        post = np.array([np.median(top[:, 0]), np.median(top[:, 1])])
        low = bv[bv[:, 2] < bv[:, 2].min() + 80]                    # blade near the bottom
        d = low[:, :2] - low[:, :2].mean(0)
        _, _, vt = np.linalg.svd(d, full_matrices=False)
        chord = vt[0] if vt[0][1] >= 0 else -vt[0]                  # chord direction, pointing aft
        ang = np.arctan2(chord[0], chord[1])                        # rotation away from straight aft
        c, sn = np.cos(-ang), np.sin(-ang)
        rel = blade.copy()
        rel[:, :, 0] -= post[0]
        rel[:, :, 1] -= post[1]
        x, y = rel[:, :, 0].copy(), rel[:, :, 1].copy()
        rel[:, :, 0] = x * c + y * sn
        rel[:, :, 1] = -x * sn + y * c
        rel[:, :, 1] += post[1]                                      # straightened, post on the centreline
        save(os.path.join(OUT, "rudder.stl"), rel)
        rudder_post = [0.0, round(float(post[1]), 1), round(float(bv[:, 2].mean()), 1)]
        print(f"rudder blade straightened by {np.degrees(ang):+.1f} deg about its post at y={post[1]:.1f} mm")
        moving_rud = rel

        static = [b[0] for b in bodies if b[1][1] > 460 and b[0] is not blade]
        save(os.path.join(OUT, "rudder_bracket.stl"), decimate(np.concatenate(static), 0.8))
        save(os.path.join(OUT, "rudder_plate.stl"), load(src("plate")) + np.array([0.0, TRANSOM_Y_MM, RUDDER_RAISE_MM]))
        # deck hardware: small bodies sitting on the deck (hatch clips, mast step)
        deck = [b[0] for b in bodies if b[1][2] >= 200 and b[2][2] < 260 and b[1][1] < 460
                and (b[2] - b[1]).max() < 120]
        save(os.path.join(OUT, "deck_hardware.stl"), decimate(np.concatenate(deck), 1.0))
        print(f"deck hardware: {len(deck)} pieces")
    else:
        # Rudder: blade + post move; the U bracket stays on the transom
        rud = load(src("rudder"))
        cen = rud.mean(1)
        moving = (np.abs(cen[:, 0]) < 22) & ((cen[:, 2] < -85) | ((np.abs(cen[:, 0]) < 12) & (cen[:, 2] < 70)))
        shift = np.array([0.0, TRANSOM_Y_MM, RUDDER_RAISE_MM])
        moving_rud = rud[moving] + shift
        save(os.path.join(OUT, "rudder.stl"), moving_rud)
        save(os.path.join(OUT, "rudder_bracket.stl"), rud[~moving] + shift)
        save(os.path.join(OUT, "rudder_plate.stl"), load(src("plate")) + shift)
        rudder_post = [0.0, TRANSOM_Y_MM + RUDDER_POST_Y_MM, RUDDER_RAISE_MM]
    save(os.path.join(OUT, "electronics.stl"), decimate(load(src("electronics")) + np.array(ELECTRONICS_MOVE_MM), 2.0))

    # Voxel model of everything that can be underwater. The hull export is a thin shell plus a foam deck,
    # so the hull is filled column by column between the shell bottom and the deck top.
    comp = components(hull)
    ids, counts = np.unique(comp, return_counts=True)
    big = ids[np.argsort(-counts)][:2]                     # outer shell + foam deck insert
    shell = hull[np.isin(comp, big)]
    cell = 8.0
    vox = {
        "hull": column_voxels(shell, cell),
        "keel": column_voxels(keel, cell),
        "rudder": column_voxels(moving_rud, cell),
    }
    with open(os.path.join(OUT, "hydro_voxels.bin"), "wb") as f:
        for name in ("hull", "keel", "rudder"):
            f.write(vox[name].astype("<f4").tobytes())
    hydro_meta = {"cell_mm": cell, "parts": {k: int(len(v)) for k, v in vox.items()}, "order": ["hull", "keel", "rudder"]}
    for k, v in vox.items():
        print(f"voxels {k:6s} {len(v):7d}  enclosed volume {len(v) * cell ** 3 / 1e6:6.2f} L")

    # Mass list -> CG. Created once from the Onshape assembly; never overwritten (edit it, or use the dashboard).
    mass_path = os.path.join(OUT, "mass.json")
    if not os.path.exists(mass_path):
        boom = booms_tris.reshape(-1, 3)
        fwd_boom = boom[boom[:, 1] < -150]
        cw_at = [0.0, round(float(fwd_boom[:, 1].min()) + 10, 1), round(float(fwd_boom[:, 2].mean()), 1)]
        items = [
            {"name": "Onshape assembly (as modelled)", "kg": 4.657, "at_mm": [0.341, 11.252, -140.883],
             "note": "10.267 lb from Onshape Mass properties; check materials (rods: carbon vs aluminium)"},
            {"name": "counterweight (not in CAD)", "kg": 0.25, "at_mm": cw_at,
             "note": "ESTIMATE position: end of the forward boom"},
        ]
        with open(mass_path, "w") as f:
            json.dump({"_note": "Masses in kg at CAD mm (x port+/starboard-, y bow-/stern+, z up; z=0 hull bottom). "
                                "The simulator computes the CG, waterline and stability from this list. "
                                "Add corrections as extra items (negative kg removes mass).",
                       "items": items}, f, indent=2)
        print(f"wrote {mass_path}")

    cfg = {
        "_note": "Generated by sim/tools/prepare_cad.py. Edit the constants there (or pass --mass/--cg-y) and re-run.",
        "units": "mm",
        "forward": "-y",
        "up": "+z",
        "yaw_inertia": round(ONSHAPE_YAW_INERTIA + 0.25 * 0.4 ** 2, 4),   # + counterweight ~0.4 m from the CG
        "hydro": hydro_meta,
        "pivots": {
            "wing": [mast_xy[0], mast_xy[1], 300.0],
            "rudder": rudder_post,
            "tail": [0.0, TAIL_HINGE_Y_MM, 750.0],
        },
        "vane_mounts": {k: list(v) for k, v in VANE_MOUNTS_MM.items()},
        # whip antenna: on the top of the wing near the leading edge (photo), about 30 cm
        "antenna": {"base": [0.0, round(float(wing_v[wing_v[:, 2] > wing_top - 40][:, 1].min()) + 55, 1), wing_top], "length_mm": 200},
        # Colours and finishes from the photos of the real boat ("material" picks a look in the 3D view)
        "parts": {
            "hull": {"file": "hull.stl", "attach": "boat", "color": "#e9e4d4", "material": "fiberglass"},
            "deck": {"file": "deck.stl", "attach": "boat", "color": "#c9a9b6", "material": "foam-deck"},
            "hull_fittings": {"file": "hull_fittings.stl", "attach": "boat", "color": "#1c1c1e", "material": "plastic"},
            "keel": {"file": "keel.stl", "attach": "boat", "color": "#f4f4f2", "material": "plastic"},
            "bulb": {"file": "bulb.stl", "attach": "boat", "color": "#b8bcc2", "material": "metal"},
            "electronics": {"file": "electronics.stl", "attach": "boat", "color": "#1f2937", "material": "plastic"},
            "rudder_bracket": {"file": "rudder_bracket.stl", "attach": "boat", "color": "#f4f4f2", "material": "plastic"},
            "deck_hardware": {"file": "deck_hardware.stl", "attach": "boat", "color": "#ff6a13", "material": "plastic"},
            "rudder": {"file": "rudder.stl", "attach": "rudder", "color": "#ff5a1f", "material": "plastic"},
            "wing": {"file": "wing.stl", "attach": "wing", "color": "#b9a3b5", "material": "foam"},
            "booms": {"file": "booms.stl", "attach": "wing", "color": "#16161a", "material": "carbon"},
            "tail": {"file": "tail.stl", "attach": "tail", "color": "#b9a3b5", "material": "foam"},
        },
        # Details from the photos that are not in the CAD (drawn by the 3D view)
        "extras": {
            "counterweight": {"at": "forward_boom_end", "diameter_mm": 95, "length_mm": 100, "color": "#55585e"},
            "servo_box": {"at_mm": [0.0, 445.0, 232.0], "size_mm": [55, 30, 45], "color": "#7f1d2d"},
            "stand": {"cradles_y_mm": [-260.0, 330.0]},
        },
    }
    with open(os.path.join(OUT, "boat.json"), "w") as f:
        json.dump(cfg, f, indent=2)

    hull_len = hull[:, :, 1].max() - hull[:, :, 1].min()
    print(f"hull {hull_len:.0f} mm long, volume {submerged_volume(hull, 1e6) / 1e6:.2f} L")
    print(f"mast at y={mast_xy[1]} mm, wing top z={wing_top:.0f} mm, rudder post y={rudder_post[1]:.0f} mm")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
