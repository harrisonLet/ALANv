#!/usr/bin/env python3
"""
Live view of a simulation run (or replay of a finished one).

  sim/.venv/bin/python sim/plant/viz.py                       follow the run in progress (logs/latest.csv)
  sim/.venv/bin/python sim/plant/viz.py logs/run-XXXX.csv     replay a finished run
  sim/.venv/bin/python sim/plant/viz.py --png out.png         save a snapshot instead of opening a window
"""
import argparse
import csv
import math
import os

import matplotlib

HERE = os.path.dirname(os.path.abspath(__file__))


def read_rows(path, start=0):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            if i >= start and row.get("tail_deg") not in (None, ""):
                rows.append({k: float(v) for k, v in row.items()})
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", default=os.path.join(HERE, "logs", "latest.csv"))
    ap.add_argument("--png", help="save one image of the whole run and exit")
    ap.add_argument("--window", type=float, default=60.0, help="seconds shown in the time plots")
    args = ap.parse_args()

    if args.png:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig = plt.figure(figsize=(13, 8))
    fig.canvas.manager.set_window_title("ALAN simulation")
    ax_map = fig.add_subplot(1, 2, 1)
    ax_hdg = fig.add_subplot(3, 2, 2)
    ax_srv = fig.add_subplot(3, 2, 4)
    ax_spd = fig.add_subplot(3, 2, 6)

    data = []

    def draw(_frame=None):
        nonlocal data
        if not os.path.exists(args.log):
            return
        data += read_rows(args.log, start=len(data))
        if not data:
            return
        t = [r["t"] for r in data]
        last = data[-1]

        ax_map.clear()
        ax_map.plot([r["east_m"] for r in data], [r["north_m"] for r in data], color="#2563eb", lw=1.5)
        e, n = last["east_m"], last["north_m"]
        h = math.radians(last["heading_deg"])
        ax_map.annotate("", xy=(e + 3 * math.sin(h), n + 3 * math.cos(h)), xytext=(e, n),
                        arrowprops=dict(arrowstyle="-|>", color="#111827", lw=2))
        w = math.radians(last["wing_deg"] + last["heading_deg"])
        ax_map.plot([e - 1.5 * math.sin(w), e + 1.5 * math.sin(w)], [n - 1.5 * math.cos(w), n + 1.5 * math.cos(w)],
                    color="#dc2626", lw=3, label="wing")
        twd = math.radians(last["twd_deg"])
        x0, x1 = ax_map.get_xlim()
        y0, y1 = ax_map.get_ylim()
        span = max(x1 - x0, y1 - y0, 20)
        cx, cy = x0 + 0.15 * span, y1 - 0.15 * span
        ax_map.annotate("", xy=(cx - 0.08 * span * math.sin(twd), cy - 0.08 * span * math.cos(twd)), xytext=(cx, cy),
                        arrowprops=dict(arrowstyle="-|>", color="#16a34a", lw=2))
        ax_map.text(cx, cy, f" wind {last['twd_deg']:.0f}° {last['tws_ms']:.1f} m/s", color="#16a34a", fontsize=9)
        ax_map.set_aspect("equal", adjustable="datalim")
        ax_map.set_xlabel("east (m)")
        ax_map.set_ylabel("north (m)")
        ax_map.set_title(f"t = {last['t']:.1f} s   speed {last['sog_ms']:.2f} m/s   "
                         f"{'GPS OFF  ' if not last['gps_ok'] else ''}{'VANE OFF' if not last['vane_ok'] else ''}")
        ax_map.grid(alpha=0.3)
        ax_map.legend(loc="lower right", fontsize=8)

        lo = max(0.0, t[-1] - args.window)
        sel = [r for r in data if r["t"] >= lo]
        ts = [r["t"] for r in sel]

        ax_hdg.clear()
        ax_hdg.plot(ts, [r["heading_deg"] for r in sel], label="heading", color="#111827")
        ax_hdg.plot(ts, [r["cog_deg"] for r in sel], label="course over ground", color="#2563eb", alpha=0.6)
        ax_hdg.set_ylabel("deg (true)")
        ax_hdg.legend(loc="upper left", fontsize=8)
        ax_hdg.grid(alpha=0.3)

        ax_srv.clear()
        ax_srv.plot(ts, [r["rudder_deg"] for r in sel], label="rudder", color="#7c3aed")
        ax_srv.plot(ts, [r["tail_deg"] for r in sel], label="tail", color="#dc2626")
        ax_srv.plot(ts, [r["aoa_deg"] for r in sel], label="wing angle of attack", color="#f59e0b", alpha=0.8)
        ax_srv.set_ylabel("deg")
        ax_srv.legend(loc="upper left", fontsize=8)
        ax_srv.grid(alpha=0.3)

        ax_spd.clear()
        ax_spd.plot(ts, [r["sog_ms"] for r in sel], label="boat speed", color="#2563eb")
        ax_spd.plot(ts, [r["tws_ms"] for r in sel], label="wind speed", color="#16a34a", alpha=0.6)
        ax_spd.set_ylabel("m/s")
        ax_spd.set_xlabel("time (s)")
        ax_spd.legend(loc="upper left", fontsize=8)
        ax_spd.grid(alpha=0.3)
        fig.tight_layout()

    if args.png:
        draw()
        fig.savefig(args.png, dpi=110)
        print(f"saved {args.png}")
        return
    anim = FuncAnimation(fig, draw, interval=500, cache_frame_data=False)  # noqa: F841 (keep a reference)
    plt.show()


if __name__ == "__main__":
    main()
