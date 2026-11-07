# Boat model (generated)

Everything in this folder except this README is generated from the Onshape exports by:

```bash
sim/.venv/bin/python sim/tools/prepare_cad.py            # reads the exports from ~/Downloads
sim/.venv/bin/python sim/tools/prepare_cad.py --src DIR  # or from another folder
```

(One-time setup: `python3 -m venv sim/.venv && sim/.venv/bin/pip install numpy matplotlib`.)

## Inputs (Onshape → Export → STL, millimetres)

| File | What | How it is used |
|---|---|---|
| `Vaka v2 (1).stl` | hull | as exported (assembly coordinates) |
| `Keel v2 (1).stl` | keel + bulb | moved onto the hull (offset measured from the assembly) |
| `Sail v3 (1).stl` | wing + mast, tail, booms | split into wing / tail / booms and moved onto the mast step |
| `Rudder v5 (1).stl`, `Rudder Mount (Plate) (1).stl` | rudder | fallback only if the mated assembly is missing |
| `Electronics Box V2 (1).stl` | electronics | moved into the aft hatch |
| `Sail Boat (14).stl` | **mated assembly** | rudder blade, bracket and deck hardware are taken from it; the offsets above were measured against it |

If the CAD changes, re-export these and re-run the script. If a part moves in the assembly, update the offsets at the
top of `sim/tools/prepare_cad.py` (each is commented with how it was measured).

## Outputs

| File | Used by |
|---|---|
| `*.stl` | 3D view (Live tab) and the stability drawings |
| `boat.json` | frame (forward −y, up +z, mm), hinge points (mast, rudder post, tail), vane mounts, antenna, yaw inertia |
| `hydro_voxels.bin` | buoyancy: the hull, keel and rudder filled with 8 mm cubes |
| `mass.json` | **edit this** (or the Stability tab): part masses and positions → CG, waterline, trim, stability |

`mass.json` is created once and never overwritten. It starts with the Onshape mass properties (4.657 kg at
(0.34, 11.25, −140.88) mm) plus the 250 g counterweight that is not in the CAD. When you weigh the boat or fix the
materials in Onshape, update it there.
