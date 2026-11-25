#!/usr/bin/env python3
"""
plot_field_log.py - PNG graphs from gs_web field-test logs

Decodes the RX-RAW / TX-RAW bytes of gs_web (or gs_term) logs with gs_web's own decoder, so the telemetry
layout always matches ../terminal/telemetry.h. One PNG per graph:
  01-16  each log's LTP run (longest stretch in AUTO): route map, cross-track error, nav state, waypoint distance,
         speed, heading PID, sail and wind, heel, point of sail, speed polar
  17-30  every log side by side in one PNG: attitude, IMU calibration, sensor validity, GPS, battery, the boat's view
         of the radio link, coverage map, commands, link age, radio counters, event timeline
  31-33  across all logs: signal and delivery against distance, noise floor (with any --baseline logs)
  34     every log side by side: compass heading against the course worked out from the boat's GPS fixes

Usage:  python3 plot_field_log.py [--out DIR] [--ref LAT,LON] [--baseline LABEL=LOG] LABEL=LOG [LABEL=LOG ...]
  e.g.  python3 plot_field_log.py water=log/gs_log_20261124_151620.txt land=log/gs_log_20261124_160953.txt \
            --baseline bench=log/gs_log_20261124_142747.txt
A --baseline log (a bench run) only joins the noise-floor comparison: no per-log graphs, no distances.
Output: DIR (default: plots_<date of the first log>/ next to it), NN_name.png (LTP graphs: NN_name_<label>.png)
Distances are from --ref, or else from the boat's first GPS fix in each log, which is the ground station only
if it stayed where the boat started.
"""
import argparse
import math
import os
import re
import struct
import sys
import textwrap
import warnings
from datetime import datetime, timedelta

import matplotlib
matplotlib.use('Agg')
import matplotlib.dates as mdates
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gs_web as GW  # noqa: E402  telemetry layout and the mode, nav state and result names
BL = GW.BL

# ─── Style: reference palette on the light chart surface ────────────────────
SURFACE, INK, INK2, MUTED, GRID, AXIS, NEUTRAL = '#fcfcfb', '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#c3c2b7', '#f0efec'
S = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']  # categorical, fixed order
GOOD, WARN, SERIOUS, CRIT = '#0ca30c', '#fab219', '#ec835a', '#d03b3b'                    # status only
SEQ = LinearSegmentedColormap.from_list('blue', ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b'])
ORDINAL = ['#86b6ef', '#5598e7', '#2a78d6', '#1c5cab', '#104281']                           # ordered classes, light to dark

# Colour follows the entity: each mode and nav state keeps its colour in every graph
MODE_COLOR = {0: S[0], 1: S[1], 5: S[2], 8: S[3], 6: S[4], 7: S[5], 3: S[6], 2: S[7]}
NAV_COLOR = {1: S[0], 2: S[1], 3: S[2], 4: S[3], 5: S[4], 6: S[7]}
NAV_MARK = {1: 'o', 2: '^', 3: 'v', 4: 's', 5: 'D', 6: 'X'}
VALIDITY_COLOR = {3: GOOD, 2: SERIOUS, 1: WARN, 0: CRIT}
LORA_MODES = {1, 3, 4, 6, 7, 8, 9, 10}      # manual and semi modes: these drop to SAFE after the 10 s link timeout
LINK_TIMEOUT_S = 10
RUDDER_LIMIT = 35                           # PID_HEADING_RUDDER_LIMIT
SF8_SNR_LIMIT = -10                         # SX1276 demodulation floor at SF8
HALO = [pe.withStroke(linewidth=3, foreground=SURFACE)]   # keeps labels readable over the track
GAP_S = 5                                   # longer than this without telemetry breaks lines and spans

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE, 'savefig.dpi': 150,
    'font.family': 'sans-serif', 'font.size': 9.5, 'text.color': INK, 'axes.labelcolor': INK2,
    'axes.titlesize': 10, 'axes.titlelocation': 'left', 'axes.titlecolor': INK2,
    'axes.edgecolor': AXIS, 'axes.linewidth': 0.8, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'grid.linestyle': '-', 'axes.axisbelow': True,
    'xtick.color': MUTED, 'ytick.color': MUTED, 'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
    'lines.linewidth': 1.3, 'lines.solid_capstyle': 'round', 'legend.frameon': False, 'legend.fontsize': 8.5,
    'axes.prop_cycle': matplotlib.cycler(color=S),
})


# ─── Decode ─────────────────────────────────────────────────────────────────

def command_names():
    """{type: name} from commands.h, so new commands get their name without editing this script."""
    names = {v: k[len('COMMAND_'):].lower().replace('_', ' ') for k, v in BL._E.items() if k.startswith('COMMAND_')}
    names.update({0x10 + v: 'mode ' + k for k, v in BL.MODES.items()})
    # What these do, rather than their names in commands.h
    names.update({BL.COMMAND_NVS_LOAD_START: 'route start', BL.COMMAND_NVS_LOAD: 'waypoint',
                  BL.COMMAND_NVS_LOAD_STOP: 'route commit', BL.COMMAND_RUDDER_PID: 'heading target',
                  BL.COMMAND_SAIL_PID: 'sail target'})
    return names


CMD_NAMES = command_names()


def parse(path):
    """Telemetry packets, sent packets and event lines of one log, the same way gs_web's reader splits the stream."""
    m = re.search(r'(\d{8})_\d{6}', os.path.basename(path))
    day = datetime.strptime(m.group(1), '%Y%m%d') if m else datetime(2000, 1, 1)
    rx, tx, events, buf = [], [], [], bytearray()
    for line in open(path, errors='replace'):
        p = line.split()
        if len(p) < 2 or not re.fullmatch(r'\d\d:\d\d:\d\d(\.\d+)?', p[0]):
            continue
        h, mi, s = p[0].split(':')
        t = day + timedelta(hours=int(h), minutes=int(mi), seconds=float(s))
        if p[1] == 'TX-RAW':
            data = bytes(int(b, 16) for b in p[3:])
            if len(data) >= 5:
                tx.append({'t': t, 'seq': data[2], 'type': data[4], 'payload': data[5:]})
        elif p[1] == 'RX-RAW':
            buf += bytes(int(b, 16) for b in p[3:])
            i = 0
            while len(buf) - i >= GW.PKT_LEN:
                if buf[i] == GW.ADDR_GROUND and buf[i + 1] == GW.ADDR_BOAT and buf[i + 3] == 1 + GW.TELEM_LEN:
                    pkt = bytes(buf[i:i + GW.PKT_LEN])
                    d = GW.unpack(pkt[GW.HDR_LEN:])
                    d.update(t=t, seq=pkt[2], flags=pkt[4])
                    rx.append(d)
                    i += GW.PKT_LEN
                else:
                    i += 1
            del buf[:i]
        else:
            events.append((t, p[1], ' '.join(p[2:])))
    return rx, tx, events


def wrap180(a):
    return (np.asarray(a, float) + 180) % 360 - 180


class Log:
    def __init__(self, label, path, ref):
        self.label, self.path = label, path
        self.rx, self.tx, self.events = parse(path)
        rx = self.rx
        self.n = len(rx)
        if not self.n:
            return
        self.dt = [r['t'] for r in rx]
        self.tn = mdates.date2num(self.dt)
        self.ts = np.array([(t - self.dt[0]).total_seconds() for t in self.dt])
        self.f = lambda k: np.array([r[k] for r in rx], float)
        self.mode = self.f('mode').astype(int)
        self.nav = self.f('nav_state').astype(int)
        self.age = self.f('rx_age_s')
        fix = np.array([bool(r['gps_valid'] == 3 and (r['latitude'] or r['longitude'])) for r in rx])
        lat, lon = np.where(fix, self.f('latitude'), np.nan), np.where(fix, self.f('longitude'), np.nan)
        self.ref = ref or ((lat[fix][0], lon[fix][0]) if fix.any() else None)
        self.ref_given = ref is not None
        if self.ref:
            self.x, self.y = self.enu(lat, lon)
        else:
            self.x = self.y = np.full(self.n, np.nan)
        self.dist = np.hypot(self.x, self.y)
        self.sog, self.cog, _ = self.motion()
        up = self.f('uptime_ms')
        self.reboot = np.r_[False, np.diff(up) < 0]
        rf = (self.f('flags').astype(int) & GW.FLAG_RADIO_REINIT) > 0
        self.reinit = rf & ~np.r_[False, rf[:-1]]
        self.noise = np.where(self.f('rx_noise') < -150, np.nan, self.f('rx_noise'))   # -157: register read 0
        fresh = self.age == 0
        self.rssi = np.where(fresh, self.f('rx_last_rssi'), np.nan)
        self.snr = np.where(fresh, self.f('rx_last_snr'), np.nan)
        self.cmds = self.commands()
        self.safes = self.safe_events()

    def enu(self, lat, lon):
        lat0, lon0 = self.ref
        x = 6371000 * np.radians(np.asarray(lon) - lon0) * math.cos(math.radians(lat0))
        return x, 6371000 * np.radians(np.asarray(lat) - lat0)

    def motion(self, k=3):
        """Speed and course over ground from GPS fixes k samples either side (k=3: about 6 s, so jitter averages out),
        and the compass heading averaged over the same samples, to compare with that course."""
        sog, cog, hdg = np.full(self.n, np.nan), np.full(self.n, np.nan), np.full(self.n, np.nan)
        h = np.radians(self.f('heading'))
        idx = np.where(np.isfinite(self.x))[0]
        for p in range(k, len(idx) - k):
            a, b = idx[p - k], idx[p + k]
            dt = self.ts[b] - self.ts[a]
            if 0 < dt <= 4 * k:
                dx, dy = self.x[b] - self.x[a], self.y[b] - self.y[a]
                sog[idx[p]] = math.hypot(dx, dy) / dt
                cog[idx[p]] = math.degrees(math.atan2(dx, dy)) % 360
                hdg[idx[p]] = math.degrees(math.atan2(np.sin(h[a:b + 1]).mean(), np.cos(h[a:b + 1]).mean())) % 360
        return sog, cog, hdg

    def commands(self):
        """Each command with its sends (gs_web resends with the same seq) and the telemetry that acknowledged it."""
        cmds = []
        for x in self.tx:
            if x['type'] == 0:
                continue
            c = cmds[-1] if cmds else None
            if c and c['seq'] == x['seq'] and c['type'] == x['type'] and (x['t'] - c['sends'][-1]).total_seconds() < 15:
                c['sends'].append(x['t'])
                continue
            cmds.append({'seq': x['seq'], 'type': x['type'], 'payload': x['payload'], 'sends': [x['t']],
                         'name': CMD_NAMES.get(x['type'], f'0x{x["type"]:02X}')})
        for c in cmds:
            t0, t1 = c['sends'][0], c['sends'][-1] + timedelta(seconds=10)
            ack = next((r for r in self.rx if t0 < r['t'] <= t1 and r['last_cmd_seq'] == c['seq']), None)
            c['ack_t'] = ack['t'] if ack else None
            c['result'] = ack['last_cmd_result_name'] if ack else None
            c['outcome'] = 'acknowledged' if c['result'] == 'OK' else ('refused' if ack else 'never acknowledged')
            i = int(np.argmin(np.abs(self.tn - mdates.date2num(t0))))
            c['dist'] = self.dist[i]
        return cmds

    def safe_events(self):
        """(index, cause) for every switch into SAFE."""
        out = []
        for i in np.where((self.mode == 2) & np.r_[False, self.mode[:-1] != 2] & ~self.reboot)[0]:
            r = self.rx[i]
            sent = [c for c in self.cmds if c['type'] == 0x12 and c['ack_t'] and abs((c['ack_t'] - r['t']).total_seconds()) < 3]
            # The corridor flag holds until a new route, so it only explains a SAFE straight out of AUTO
            if self.mode[i - 1] == 0 and (r['nav_corridor'] or self.rx[i - 1]['nav_corridor']):
                cause = 'left corridor'
            elif self.mode[i - 1] in LORA_MODES and self.age[i] >= LINK_TIMEOUT_S:
                cause = 'link timeout'
            elif sent:
                cause = 'SAFE command'
            elif self.nav[i - 1] == 5:
                cause = 'route finished'
            else:
                cause = 'other'
            out.append((i, cause))
        return out

    def auto_run(self, min_s=60):
        """(first, last) index of the longest stretch in AUTO without a reboot, or None."""
        best, start = None, None
        for i in range(self.n + 1):
            inside = i < self.n and self.mode[i] == 0 and not (self.reboot[i] and start is not None)
            if inside and start is None:
                start = i
            elif not inside and start is not None:
                if self.ts[i - 1] - self.ts[start] >= min_s and (best is None or i - 1 - start > best[1] - best[0]):
                    best = (start, i - 1)
                start = i if (i < self.n and self.mode[i] == 0) else None
        return best

    def route_before(self, i0):
        """Waypoints and commit of the last route uploaded before index i0."""
        t0 = self.dt[i0]
        commits = [x for x in self.tx if x['type'] == 0x33 and x['t'] <= t0]
        if not commits:
            return None
        commit = commits[-1]
        starts = [x for x in self.tx if x['type'] == 0x31 and x['t'] <= commit['t']]
        since = starts[-1]['t'] if starts else self.dt[0]
        wps = {}
        for x in self.tx:
            if x['type'] == 0x32 and since <= x['t'] <= commit['t']:
                idx, margin, lat, lon = struct.unpack('<HHii', x['payload'][:12])
                wps[idx] = {'margin': margin, 'lat': lat / 1e7, 'lon': lon / 1e7}
        count, _, arrive, warning, _ = struct.unpack('<HHBBH', commit['payload'][:8])
        wps = [wps[k] for k in sorted(wps)][:count]
        for w in wps:
            w['x'], w['y'] = (float(v) for v in self.enu(w['lat'], w['lon']))
        return {'wps': wps, 'arrive': arrive or 8, 'warning': warning}


# ─── Drawing helpers ────────────────────────────────────────────────────────

def header(fig, title, sub):
    """Title and a subtitle wrapped to the figure width, above the plots."""
    sub = textwrap.fill(sub, width=int(fig.get_figwidth() * 15.5))
    h = fig.get_figheight()
    fig.text(0.012, 1 - 0.12 / h, title, fontsize=12.5, weight='bold', color=INK, va='top')
    fig.text(0.012, 1 - 0.36 / h, sub, fontsize=9, color=INK2, va='top', linespacing=1.3)
    fig.tight_layout(rect=(0, 0, 1, 1 - (0.5 + 0.17 * sub.count('\n') + 0.12) / h))


def time_axis(ax):
    ax.xaxis.set_major_locator(mdates.AutoDateLocator(minticks=4, maxticks=9))
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))


def gapped(tn, y, jump=None):
    """Line data with NaN breaks where telemetry stopped for GAP_S, or an angle wrapped past `jump`."""
    tn, y = np.asarray(tn, float), np.asarray(y, float)
    br = np.diff(tn) * 86400 > GAP_S
    if jump:
        br |= np.abs(np.diff(y)) > jump
    cut = np.where(br)[0] + 1
    return np.insert(tn, cut, np.nan), np.insert(y, cut, np.nan)


def spans(L, values, i0=0, i1=None):
    """[(t0, t1, value)] runs of equal values, broken where telemetry stopped."""
    i1 = L.n - 1 if i1 is None else i1
    out, start = [], i0
    for i in range(i0 + 1, i1 + 2):
        end = i > i1 or values[i] != values[start] or (L.ts[i] - L.ts[i - 1]) > GAP_S
        if end:
            t_end = L.tn[i] if i <= i1 and (L.ts[i] - L.ts[i - 1]) <= GAP_S else L.tn[i - 1]
            out.append((L.tn[start], t_end, values[start]))
            start = i
    return out


def strip(ax, sp, color, name, row=0, height=0.8):
    """Coloured spans in one row, named inside when there is room (identity is never colour alone)."""
    xr = (max(s[1] for s in sp) - min(s[0] for s in sp)) or 1
    for t0, t1, v in sp:
        c = color(v)
        if c is None:
            continue
        ax.broken_barh([(t0, max(t1 - t0, 1 / 86400))], (row - height / 2, height), facecolors=c, edgecolor=SURFACE, linewidth=1)
        if (t1 - t0) / xr > 0.0075 * len(name(v)) + 0.01:     # only when the name fits inside
            ax.text((t0 + t1) / 2, row, name(v), ha='center', va='center', fontsize=7.5,
                    color=INK if c in (S[3], WARN, NEUTRAL) else '#ffffff')


def mode_strip(ax, L, i0=0, i1=None):
    strip(ax, spans(L, L.mode, i0, i1), lambda v: MODE_COLOR.get(v, MUTED), lambda v: BL.MODE_NAMES.get(v, str(v)))
    ax.set_yticks([0]); ax.set_yticklabels(['mode']); ax.set_ylim(-0.5, 0.5); ax.grid(False)
    ax.spines['left'].set_visible(False); ax.tick_params(axis='y', length=0)


def nav_name(v):
    return GW.NAV_STATES.get(v, f'state {v}')


def nav_strip(ax, L, i0, i1):
    strip(ax, spans(L, L.nav, i0, i1), lambda v: NAV_COLOR.get(v, MUTED), nav_name)
    ax.set_yticks([0]); ax.set_yticklabels(['LTP']); ax.set_ylim(-0.5, 0.5); ax.grid(False)
    ax.spines['left'].set_visible(False); ax.tick_params(axis='y', length=0)


def hline(ax, y, text, color=INK2, side='right', va='bottom'):
    ax.axhline(y, color=color, linewidth=0.9, zorder=1)
    x = 0.995 if side == 'right' else 0.005
    ax.text(x, y, text, transform=ax.get_yaxis_transform(), ha=side, va=va, fontsize=7.5, color=INK2, path_effects=HALO)


def binned(x, y, edges, min_n=5):
    """Bin centres and medians; NaN for thin bins, so a median line breaks across gaps instead of bridging them."""
    c, med = [], []
    for a, b in zip(edges, edges[1:]):
        m = (x >= a) & (x < b) & np.isfinite(y)
        c.append((a + b) / 2)
        med.append(np.median(y[m]) if m.sum() >= min_n else np.nan)
    return np.array(c), np.array(med)


def legend_right(ax, **kw):
    ax.legend(loc='upper left', bbox_to_anchor=(1.0, 1.0), **kw)


def gs_marker(ax, L):
    ax.plot(0, 0, marker='^', color=INK, markersize=8, linestyle='none', zorder=6)
    ax.annotate('ground station' + ('' if L.ref_given else ' (first fix)'), (0, 0), xytext=(6, -10),
                textcoords='offset points', fontsize=7.5, color=INK2, path_effects=HALO)


def hms(t):
    return mdates.num2date(t).strftime('%H:%M:%S')


# ─── 01-16: the LTP run ─────────────────────────────────────────────────────

def ltp_graphs(L, run, out):
    i0, i1 = run
    w = slice(i0, i1 + 1)
    route = L.route_before(i0)
    tn, ts = L.tn[w], L.ts[w]
    when = f'{L.label}, LTP run {hms(L.tn[i0])[:5]}-{hms(L.tn[i1])[:5]}'
    made = []
    nav = L.nav[w]
    x, y = L.x[w], L.y[w]
    seg = L.f('route_segment')[w].astype(int)
    drift = np.where(seg == 255, np.nan, L.f('drift_distance')[w])
    heading = L.f('heading')[w]
    desired = L.f('desired_heading')[w]
    desired = np.where(desired == 0xFFFF, np.nan, desired)
    err = wrap180(desired - heading)
    rudder = L.f('rudder_servo_angle')[w]
    tail = L.f('sail_servo_angle')[w]
    sail = L.f('sail_ang_forward')[w]
    awa = L.f('ang_wind')[w]
    roll, sog, cog = L.f('roll')[w], L.sog[w], L.cog[w]
    upwind = np.isin(nav, (2, 3, 4))

    def save(fig, name):
        path = os.path.join(out, f'{name}_{L.label}.png')
        fig.savefig(path)
        plt.close(fig)
        made.append(path)

    # Route geometry: leg 1 starts where AUTO picked the route up, leg k+1 runs from waypoint k to k+1
    legs = []
    if route and route['wps']:
        first = next((k for k in range(i0, i1 + 1) if np.isfinite(L.x[k])), None)
        pts = [(L.x[first], L.y[first])] + [(p['x'], p['y']) for p in route['wps']]
        legs = [(pts[k], pts[k + 1], route['wps'][k]['margin']) for k in range(len(route['wps']))]
    warn = route['warning'] if route else 0

    # 01 route map
    if legs:
        fig, ax = plt.subplots(figsize=(8.6, 8.2))
        # Every warning band first, then every corridor, so a later leg's band never covers an earlier leg's corridor
        for z, (pad, fc) in enumerate(((warn, NEUTRAL), (0, '#cde2fb'))):
            for (a, b, margin) in legs:
                u = np.subtract(b, a) / (np.hypot(*np.subtract(b, a)) or 1)
                nrm = np.array([-u[1], u[0]]) * (margin + pad)
                ax.add_patch(Polygon([a + nrm, b + nrm, b - nrm, a - nrm], facecolor=fc, edgecolor='none', zorder=z * 0.1))
        for (a, b, _) in legs:
            ax.plot([a[0], b[0]], [a[1], b[1]], color=MUTED, linewidth=0.8, zorder=1)
        for k, p in enumerate(route['wps']):
            ax.add_patch(Circle((p['x'], p['y']), route['arrive'], facecolor='none', edgecolor=INK2, linewidth=0.8, zorder=2))
            ax.plot(p['x'], p['y'], marker='s', color=INK, markersize=6, zorder=5)
            ax.annotate(f'WP{k + 1}', (p['x'], p['y']), xytext=(7, 5), textcoords='offset points', fontsize=9, weight='bold', path_effects=HALO)
        for v in sorted(set(nav)):
            m = nav == v
            ax.scatter(x[m], y[m], s=11, marker=NAV_MARK.get(v, 'o'), color=NAV_COLOR.get(v, MUTED), linewidths=0,
                       label=nav_name(v), zorder=3)
        f0 = np.where(np.isfinite(x))[0]
        ax.annotate('AUTO start', (x[f0[0]], y[f0[0]]), xytext=(-60, -16), textcoords='offset points', fontsize=8, path_effects=HALO,
                    arrowprops=dict(arrowstyle='-', color=MUTED, linewidth=0.8))
        end = next((i for i, c in L.safes if i > i1 - 1), None)
        if end is not None and np.isfinite(L.x[end]):
            ax.plot(L.x[end], L.y[end], marker='X', color=CRIT, markersize=9, zorder=6, linestyle='none')
            ax.annotate(f'SAFE ({dict(L.safes)[end]})', (L.x[end], L.y[end]), xytext=(8, -4), textcoords='offset points',
                        fontsize=8, color=INK, path_effects=HALO)
        gs_marker(ax, L)
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel('east (m)'); ax.set_ylabel('north (m)')
        handles, labels = ax.get_legend_handles_labels()
        handles += [Patch(facecolor='#cde2fb', label=f'safe corridor (±{legs[0][2]} m)'),
                    Patch(facecolor=NEUTRAL, label=f'warning zone (+{warn} m, fault beyond)'),
                    Line2D([], [], color=INK2, linewidth=0.8, label=f'arrival radius {route["arrive"]} m')]
        ax.legend(handles=handles, loc='best', fontsize=8)
        header(fig, 'Route and track', f'{when}. Track coloured by LTP state; corridors from the uploaded route')
        save(fig, '01_route_map')

    # 02 cross-track error
    if np.isfinite(drift).any():
        fig, (ax, axs) = plt.subplots(2, 1, figsize=(10, 4.6), sharex=True, gridspec_kw={'height_ratios': [5, 0.6]})
        margin = legs[0][2] if legs else 20
        for sgn in (1, -1):
            ax.axhspan(sgn * margin, sgn * (margin + warn), color=NEUTRAL, zorder=0)
        hline(ax, margin, f'safe corridor ±{margin} m', va='bottom'); ax.axhline(-margin, color=INK2, linewidth=0.9)
        hline(ax, margin + warn, f'fault ±{margin + warn} m (SAFE)', color=CRIT); ax.axhline(-(margin + warn), color=CRIT, linewidth=0.9)
        ax.axhline(0, color=AXIS, linewidth=0.8)
        ax.plot(*gapped(tn, drift), color=S[0])
        prev = None
        for k in np.where(seg != 255)[0]:
            if seg[k] != prev:                  # each new leg once, ignoring the no-leg samples between
                ax.axvline(tn[k], color=MUTED, linewidth=0.8)
                ax.text(tn[k], ax.get_ylim()[0], f' leg {seg[k] + 1}', fontsize=7.5, color=INK2, va='bottom')
                prev = seg[k]
        ax.set_ylabel('cross-track error (m, + = starboard)')
        nav_strip(axs, L, i0, i1); time_axis(axs)
        header(fig, 'Cross-track error', f'{when}. Distance off the planned leg; past the warning zone LTP faults and requests SAFE')
        save(fig, '02_cross_track_error')

    # 03 nav state and route segment
    fig = plt.figure(figsize=(10, 3.8))
    gs = fig.add_gridspec(3, 1, height_ratios=[0.7, 0.7, 2.2], hspace=0.6)
    a1, a2, a3 = fig.add_subplot(gs[0]), fig.add_subplot(gs[1]), fig.add_subplot(gs[2])
    nav_strip(a1, L, i0, i1); time_axis(a1)
    strip(a2, spans(L, L.f('route_segment').astype(int), i0, i1), lambda v: MUTED if v == 255 else ORDINAL[min(v, 4)],
          lambda v: 'none' if v == 255 else f'to WP{v + 1}')
    a2.set_yticks([0]); a2.set_yticklabels(['leg']); a2.set_ylim(-0.5, 0.5); a2.grid(False); time_axis(a2)
    a2.spines['left'].set_visible(False); a2.set_xlim(a1.get_xlim())
    states = sorted(set(nav))
    secs = [np.sum(np.diff(np.r_[ts, ts[-1]])[nav == v]) for v in states]
    a3.barh([nav_name(v) for v in states], secs, color=[NAV_COLOR.get(v, MUTED) for v in states], height=0.55)
    for k, s_ in enumerate(secs):
        a3.text(s_, k, f'  {s_:.0f} s ({100 * s_ / max(sum(secs), 1):.0f}%)', va='center', fontsize=8, color=INK2)
    a3.set_xlabel('time in state (s)'); a3.grid(axis='y', visible=False); a3.invert_yaxis()
    header(fig, 'LTP state and leg', f'{when}. Which planner state and leg the boat was in, and for how long')
    save(fig, '03_nav_state_timeline')

    # 04 distance to the active waypoint
    if route and route['wps']:
        wx = np.array([route['wps'][s_]['x'] if s_ < len(route['wps']) else np.nan for s_ in seg])
        wy = np.array([route['wps'][s_]['y'] if s_ < len(route['wps']) else np.nan for s_ in seg])
        dwp = np.hypot(wx - x, wy - y)
        fig, ax = plt.subplots(figsize=(10, 3.8))
        for v in sorted(set(seg) - {255}):
            m = np.where(seg == v, dwp, np.nan)
            ax.plot(*gapped(tn, m), color=ORDINAL[min(v + 1, 4)], label=f'to WP{v + 1}')
        hline(ax, route['arrive'], f'arrival radius {route["arrive"]} m')
        ax.set_ylabel('distance to waypoint (m)'); ax.set_ylim(bottom=0); time_axis(ax); legend_right(ax)
        header(fig, 'Distance to the active waypoint', f'{when}. From GPS and the uploaded waypoints')
        save(fig, '04_distance_to_waypoint')
        brg = np.degrees(np.arctan2(wx - x, wy - y))
        vmg = sog * np.cos(np.radians(cog - brg))
    else:
        vmg = np.full_like(sog, np.nan)

    # 05 speed over ground and VMG
    if np.isfinite(sog).any():
        fig, ax = plt.subplots(figsize=(10, 3.8))
        ax.axhline(0, color=AXIS, linewidth=0.8)
        ax.plot(*gapped(tn, sog), color=S[0], label='speed over ground')
        ax.plot(*gapped(tn, vmg), color=S[1], label='speed made good to the waypoint')
        ax.set_ylabel('m/s'); time_axis(ax); legend_right(ax)
        med = np.nanmedian(sog)
        header(fig, 'Boat speed', f'{when}. From GPS over about 6 s; median {med:.2f} m/s ({med * 1.944:.1f} kn)')
        save(fig, '05_speed_vmg')

    # 06 heading against course over ground
    moving = sog > 0.3
    if moving.any():
        fig, (ax, ad) = plt.subplots(2, 1, figsize=(10, 5.4), sharex=True, gridspec_kw={'height_ratios': [3, 2]})
        ax.plot(*gapped(tn, heading, 180), color=S[0], label='heading (compass)')
        ax.plot(*gapped(tn, np.where(moving, cog, np.nan), 180), color=S[1], label='course over ground (GPS)')
        ax.set_ylabel('deg'); ax.set_ylim(0, 360); ax.set_yticks(range(0, 361, 90)); legend_right(ax)
        ad.axhline(0, color=AXIS, linewidth=0.8)
        ad.plot(*gapped(tn, np.where(moving, wrap180(cog - heading), np.nan)), color=S[0])
        ad.set_ylabel('course - heading (deg)'); time_axis(ad)
        header(fig, 'Heading against course over ground', f'{when}. The difference is leeway plus current (shown while moving > 0.3 m/s)')
        save(fig, '06_heading_vs_cog')

    # 07 track coloured by speed
    if legs and np.isfinite(sog).any():
        fig, ax = plt.subplots(figsize=(8.6, 8.2))
        for (a, b, _) in legs:
            ax.plot([a[0], b[0]], [a[1], b[1]], color=MUTED, linewidth=0.8)
        for k, p in enumerate(route['wps']):
            ax.plot(p['x'], p['y'], marker='s', color=INK, markersize=6)
            ax.annotate(f'WP{k + 1}', (p['x'], p['y']), xytext=(7, 5), textcoords='offset points', fontsize=9, weight='bold', path_effects=HALO)
        sc = ax.scatter(x, y, c=sog, cmap=SEQ, s=12, linewidths=0, vmin=0, vmax=np.nanpercentile(sog, 98))
        fig.colorbar(sc, ax=ax, shrink=0.7, label='speed over ground (m/s)')
        ax.set_aspect('equal', adjustable='datalim'); ax.set_xlabel('east (m)'); ax.set_ylabel('north (m)')
        header(fig, 'Track coloured by speed', when)
        save(fig, '07_track_speed')

    # 08 desired against actual heading
    if np.isfinite(desired).any():
        fig, ax = plt.subplots(figsize=(10, 3.8))
        ax.plot(*gapped(tn, heading, 180), color=S[0], label='actual heading')
        ax.plot(*gapped(tn, desired, 180), color=S[1], label='desired heading (LTP)')
        ax.set_ylabel('deg'); ax.set_ylim(0, 360); ax.set_yticks(range(0, 361, 90)); time_axis(ax); legend_right(ax)
        header(fig, 'Heading: desired against actual', f'{when}. The heading PID steers the rudder to close the gap')
        save(fig, '08_heading_desired_actual')

        # 09 heading error and rudder
        stats = []
        for m, lab in ((~upwind & (nav == 1), 'DIRECT'), (upwind, 'upwind')):
            e = np.abs(err[m & np.isfinite(err)])
            if e.size:
                stats.append(f'{lab} median {np.median(e):.0f}°, 90% within {np.percentile(e, 90):.0f}°')
        fig, (ax, ar) = plt.subplots(2, 1, figsize=(10, 5.4), sharex=True)
        ax.axhline(0, color=AXIS, linewidth=0.8); ax.plot(*gapped(tn, err), color=S[0])
        ax.set_ylabel('heading error (deg)'); ax.set_title('desired - actual heading: ' + '; '.join(stats), fontsize=8.5)
        ar.axhline(0, color=AXIS, linewidth=0.8); ar.plot(*gapped(tn, rudder), color=S[1])
        hline(ar, RUDDER_LIMIT, f'PID limit ±{RUDDER_LIMIT}°'); ar.axhline(-RUDDER_LIMIT, color=INK2, linewidth=0.9)
        at_limit = 100 * np.mean(np.abs(rudder) >= RUDDER_LIMIT - 1)
        ar.set_ylabel('rudder servo (deg)'); ar.set_ylim(-RUDDER_LIMIT - 8, RUDDER_LIMIT + 8); time_axis(ar)
        header(fig, 'Heading error and rudder', f'{when}. Rudder at the PID limit {at_limit:.0f}% of the time')
        save(fig, '09_heading_error_and_rudder')

        # 10 rudder against heading error
        fig, ax = plt.subplots(figsize=(7.5, 5.6))
        m = np.isfinite(err)
        ax.scatter(err[m], rudder[m], s=9, color=S[0], alpha=0.35, linewidths=0, label='each second')
        c, med = binned(err, rudder, np.arange(-90, 91, 5))
        ax.plot(c, med, color=S[1], linewidth=2, label='median per 5°')
        ax.axhline(0, color=AXIS, linewidth=0.8); ax.axvline(0, color=AXIS, linewidth=0.8)
        ax.set_xlim(-90, 90); ax.set_xlabel('heading error (deg)'); ax.set_ylabel('rudder servo (deg)')
        ax.legend(loc='upper left')
        header(fig, 'Rudder against heading error', f'{when}. The slope near zero is the PID\'s effective gain; flat ends are the ±{RUDDER_LIMIT}° limit')
        save(fig, '10_rudder_vs_error')

        # 11 heading tracking lag. LTP moves the target a few degrees a second, not in steps, so instead of step
        # responses: how far the actual heading trails the target, as the shift that best lines the two up
        v = np.isfinite(desired) & np.isfinite(heading)
        if v.sum() > 60:
            tv = ts[v]
            want = np.degrees(np.unwrap(np.radians(desired[v])))
            have = np.degrees(np.unwrap(np.radians(heading[v])))
            lags = np.arange(-15, 31)
            rms = []
            for lag in lags:
                m = (tv + lag <= tv[-1]) & (tv + lag >= tv[0])
                rms.append(np.sqrt(np.mean(wrap180(want[m] - np.interp(tv[m] + lag, tv, have)) ** 2)))
            best = int(lags[int(np.argmin(rms))])
            zero = rms[int(np.where(lags == 0)[0][0])]
            fig, ax = plt.subplots(figsize=(9, 4.2))
            ax.plot(lags, rms, color=S[0], linewidth=2, marker='o', markersize=4)
            ax.axvline(best, color=INK2, linewidth=0.9)
            what = (f'actual {best} s behind the target' if best > 0 else
                    'no lag: the boat follows within a second' if best == 0 else
                    f'actual {-best} s ahead: the target partly follows the boat')
            ax.text(0.02, 0.96, f'best match (line): {what}\nRMS error {min(rms):.0f}° there, {zero:.0f}° with no shift',
                    transform=ax.transAxes, fontsize=8, color=INK, va='top', path_effects=HALO)
            ax.set_xlabel('shift applied to the actual heading (s, + = actual later)'); ax.set_ylabel('RMS heading error (deg)')
            ax.set_ylim(bottom=0)
            header(fig, 'Heading tracking lag', f'{when}. The target moves smoothly, so the lag is how late the boat follows it')
            save(fig, '11_heading_tracking_lag')

    # 12 apparent wind
    if np.isfinite(awa).any():
        fig = plt.figure(figsize=(11, 4.2))
        ax = fig.add_subplot(1, 2, 1)
        ax.plot(*gapped(tn, awa, 180), color=S[0]); ax.axhline(0, color=AXIS, linewidth=0.8)
        ax.set_ylim(-180, 180); ax.set_yticks(range(-180, 181, 90)); ax.set_ylabel('apparent wind vs bow (deg, + = starboard)')
        time_axis(ax)
        ap = fig.add_subplot(1, 2, 2, projection='polar')
        ap.set_theta_zero_location('N'); ap.set_theta_direction(-1)
        edges = np.radians(np.arange(-180, 181, 10))
        cnt, _ = np.histogram(np.radians(awa[np.isfinite(awa)]), bins=edges)
        ap.bar(edges[:-1] + np.radians(5), 100 * cnt / max(cnt.sum(), 1), width=np.radians(10), color=S[0],
               edgecolor=SURFACE, linewidth=1.5)
        ap.set_title('where the wind came from (% of time), bow up', fontsize=8.5, pad=14)
        ap.tick_params(labelsize=7.5); ap.grid(color=GRID)
        fig.subplots_adjust(wspace=0.35)
        header(fig, 'Apparent wind', f'{when}. From the wind vane, relative to the bow')
        save(fig, '12_apparent_wind')

    # 13 sail and tail servo against apparent wind
    if np.isfinite(awa).any():
        fig, axs = plt.subplots(1, 2, figsize=(12, 5), sharex=True)
        for ax, ys, yl in ((axs[0], sail, 'sail vs centreline (deg)'), (axs[1], tail, 'tail servo (deg)')):
            ax.scatter(awa, ys, s=9, color=S[0], alpha=0.35, linewidths=0, label='each second')
            c, med = binned(awa, ys, np.arange(-180, 181, 10))
            ax.plot(c, med, color=S[1], linewidth=2, label='median per 10°')
            ax.axhline(0, color=AXIS, linewidth=0.8); ax.axvline(0, color=AXIS, linewidth=0.8)
            ax.set_xlim(-180, 180); ax.set_xticks(range(-180, 181, 90))
            ax.set_xlabel('apparent wind vs bow (deg, + = starboard)'); ax.set_ylabel(yl); ax.legend(loc='upper left')
        axs[0].set_title('sail angle: the trim the sail controller produced', fontsize=8.5)
        axs[1].set_title('tail servo: what drives the wing to that angle', fontsize=8.5)
        header(fig, 'Sail and tail servo against apparent wind', when)
        save(fig, '13_sail_and_tail_vs_awa')

    # 14 heel
    fig, (ax, as_) = plt.subplots(1, 2, figsize=(11, 4), gridspec_kw={'width_ratios': [3, 2]})
    ax.axhline(0, color=AXIS, linewidth=0.8); ax.plot(*gapped(tn, roll), color=S[0])
    ax.set_ylabel('roll (deg)'); time_axis(ax)
    as_.scatter(awa, roll, s=9, color=S[0], alpha=0.35, linewidths=0)
    c, med = binned(awa, roll, np.arange(-180, 181, 15))
    as_.plot(c, med, color=S[1], linewidth=2, label='median per 15°')
    as_.axhline(0, color=AXIS, linewidth=0.8); as_.set_xlabel('apparent wind vs bow (deg)'); as_.set_ylabel('roll (deg)')
    as_.legend(loc='upper left', fontsize=7.5)
    header(fig, 'Heel', f'{when}. Range {np.nanmin(roll):.0f} to {np.nanmax(roll):.0f}°; right: against the apparent wind angle')
    save(fig, '14_heel')

    # 15 point of sail, classed from the apparent wind angle
    classes = [(0, 30, 'head to wind'), (30, 60, 'close-hauled'), (60, 110, 'reach'), (110, 150, 'broad reach'), (150, 181, 'run')]
    cls = np.array([next((k for k, (a, b, _) in enumerate(classes) if a <= abs(v) < b), -1) if np.isfinite(v) else -1 for v in awa])
    full = np.full(L.n, -1); full[w] = cls
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 3.6), gridspec_kw={'height_ratios': [0.8, 2.2]})
    strip(a1, spans(L, full, i0, i1), lambda v: ORDINAL[v] if v >= 0 else None, lambda v: classes[v][2] if v >= 0 else '')
    a1.set_yticks([]); a1.set_ylim(-0.5, 0.5); a1.grid(False); a1.spines['left'].set_visible(False); time_axis(a1)
    share = [100 * np.mean(cls == k) for k in range(len(classes))]
    a2.barh([c[2] for c in classes], share, color=ORDINAL, height=0.55)
    for k, v in enumerate(share):
        a2.text(v, k, f'  {v:.0f}%', va='center', fontsize=8, color=INK2)
    a2.set_xlabel('% of the run'); a2.invert_yaxis(); a2.grid(axis='y', visible=False)
    header(fig, 'Point of sail', f'{when}. Classed from the apparent wind angle, so a rough guide')
    save(fig, '15_point_of_sail')

    # 16 speed polar
    if np.isfinite(sog).any():
        fig = plt.figure(figsize=(7, 6.6))
        ap = fig.add_subplot(projection='polar')
        ap.set_theta_zero_location('N'); ap.set_theta_direction(-1)
        m = np.isfinite(sog) & np.isfinite(awa)
        ap.scatter(np.radians(awa[m]), sog[m], s=8, color=S[0], alpha=0.35, linewidths=0, label='each second')
        c, med = binned(np.abs(awa), sog, np.arange(0, 181, 15))
        for sgn in (1, -1):
            ap.plot(np.radians(sgn * c), med, color=S[1], linewidth=2, label='median per 15° (both tacks)' if sgn > 0 else None)
        ap.set_rlabel_position(135); ap.tick_params(labelsize=7.5); ap.grid(color=GRID)
        ap.legend(loc='lower left', bbox_to_anchor=(-0.1, -0.12), fontsize=7.5)
        header(fig, 'Speed polar', f'{when}. Speed over ground (m/s, radius) by apparent wind angle, bow up')
        save(fig, '16_speed_polar')
    return made


# ─── 17-30: one graph per kind, a column (or row) per log ──────────────────

def log_span(L):
    return f'{L.label} {hms(L.tn[0])[:5]}-{hms(L.tn[-1])[:5]}'


def per_log_graphs(logs, out):
    """Graphs that every log has, with all the logs side by side in one PNG so runs compare at a glance."""
    made = []
    n = len(logs)
    when = ', '.join(log_span(L) for L in logs)

    def cols(nrows, h, ratios=None, sharey=False, w=7.2, sharex='col'):
        """One column per log, panels stacked within it; returns the figure and each log's axes, top to bottom."""
        fig, axs = plt.subplots(nrows, n, figsize=(w * n, h), sharex=sharex, sharey=sharey, squeeze=False,
                                gridspec_kw={'height_ratios': ratios} if ratios else None)
        for k, L in enumerate(logs):
            axs[0, k].set_title(log_span(L), fontsize=9.5, weight='bold', color=INK)
            if k:
                for ax in axs[:, k]:
                    ax.set_ylabel('')
        return fig, [list(axs[:, k]) for k in range(n)]

    def rows(ncols, h, ratios=None, w=12):
        """One row per log, for graphs that are wide rather than tall."""
        fig, axs = plt.subplots(n, ncols, figsize=(w, h * n), squeeze=False,
                                gridspec_kw={'width_ratios': ratios} if ratios else None)
        for k, L in enumerate(logs):
            axs[k, 0].set_title(log_span(L), fontsize=9.5, weight='bold', color=INK)
        return fig, [list(axs[k, :]) for k in range(n)]

    def save(fig, name):
        path = os.path.join(out, f'{name}.png')
        fig.savefig(path)
        plt.close(fig)
        made.append(path)

    def first(k, ax, text):
        """Y label on the first column only, the others share it."""
        if k == 0:
            ax.set_ylabel(text)

    # 17 roll and pitch
    fig, A = cols(1, 4, sharey='row')
    for k, (L, (ax,)) in enumerate(zip(logs, A)):
        ax.axhline(0, color=AXIS, linewidth=0.8)
        ax.plot(*gapped(L.tn, L.f('roll')), color=S[0], label='roll')
        ax.plot(*gapped(L.tn, L.f('pitch')), color=S[1], label='pitch')
        first(k, ax, 'deg'); time_axis(ax)
    legend_right(A[-1][0])
    header(fig, 'Roll and pitch', when)
    save(fig, '17_roll_pitch')

    # 18 IMU calibration (BNO055 CALIB_STAT: sys, gyro, accel, mag, 2 bits each)
    fig, A = cols(4, 5.2, sharey='row')
    for k, (L, axs) in enumerate(zip(logs, A)):
        cal = L.f('imu_cal').astype(int)
        for j, (ax, name) in enumerate(zip(axs, ('system', 'gyro', 'accel', 'mag'))):
            ax.plot(*gapped(L.tn, (cal >> (6 - 2 * j)) & 3), color=S[0], drawstyle='steps-post')
            ax.set_ylim(-0.3, 3.3); ax.set_yticks([0, 3])
            if k == 0:
                ax.set_ylabel(name, rotation=0, ha='right', va='center')
        time_axis(axs[-1])
    header(fig, 'IMU calibration', f'{when}. BNO055 calibration status, 0 = uncalibrated, 3 = fully calibrated')
    save(fig, '18_imu_calibration')

    # 19 sensor validity
    sensors = [('battery', 'battery_valid'), ('sail encoder', 'encoder_valid'), ('wind vane', 'wind_valid'),
               ('GPS', 'gps_valid'), ('IMU', 'imu_valid'), ('orientation', 'orientation_valid')]
    fig, A = cols(1, 3.6, sharey='row')
    for k, (L, (ax,)) in enumerate(zip(logs, A)):
        for j, (_, key) in enumerate(sensors):
            strip(ax, spans(L, L.f(key).astype(int)), lambda v: VALIDITY_COLOR.get(v, MUTED), lambda v: '', row=j, height=0.7)
        ax.set_yticks(range(len(sensors))); ax.set_yticklabels([s_[0] for s_ in sensors]); ax.set_ylim(len(sensors) - 0.5, -0.5)
        ax.grid(False); ax.spines['left'].set_visible(False); ax.tick_params(axis='y', length=0); time_axis(ax)
    A[-1][0].legend(handles=[Patch(color=VALIDITY_COLOR[v], label=GW.VALIDITY[v]) for v in (3, 2, 1, 0)],
                    loc='upper left', bbox_to_anchor=(1.0, 1.0))
    header(fig, 'Sensor validity', f'{when}. What each sensor reported about its own data')
    save(fig, '19_sensor_validity')

    # 20 GPS satellites and fix quality
    fig, A = cols(2, 4.6, sharey='row')
    for k, (L, (a1, a2)) in enumerate(zip(logs, A)):
        a1.plot(*gapped(L.tn, L.f('satellites')), color=S[0], drawstyle='steps-post'); a1.set_ylim(bottom=0)
        a2.plot(*gapped(L.tn, L.f('fix_valid')), color=S[0], drawstyle='steps-post')
        a2.set_yticks([0, 1, 2]); a2.set_yticklabels(['none', 'GPS', 'DGPS']); a2.set_ylim(-0.3, 2.5)
        first(k, a1, 'satellites'); first(k, a2, 'fix quality'); time_axis(a2)
    header(fig, 'GPS quality', f'{when}. Satellites used and GGA fix quality')
    save(fig, '20_gps_quality')

    # 21 GPS jitter while the boat was still: the longest stretch moving under 0.2 m/s
    fig, A = cols(1, 6, w=6.2)
    for L, (ax,) in zip(logs, A):
        still = (L.sog < 0.2) & np.isfinite(L.x)
        best, start = (0, 0), None
        for i in range(L.n + 1):
            if i < L.n and still[i]:
                start = i if start is None else start
            elif start is not None:
                best = (start, i) if i - start > best[1] - best[0] else best
                start = None
        a, b = best
        if b - a < 60:
            ax.text(0.5, 0.5, 'never still for a minute', transform=ax.transAxes, ha='center', color=INK2)
            continue
        px, py = L.x[a:b] - np.nanmean(L.x[a:b]), L.y[a:b] - np.nanmean(L.y[a:b])
        r = np.hypot(px, py)
        ax.scatter(px, py, s=12, color=S[0], alpha=0.6, linewidths=0)
        for q in (50, 95):
            rad = np.nanpercentile(r, q)
            ax.add_patch(Circle((0, 0), rad, facecolor='none', edgecolor=INK2, linewidth=0.9))
            ax.text(0, rad, f' {q}% within {rad:.1f} m', fontsize=8, color=INK2, va='bottom', path_effects=HALO)
        ax.set_title(f'{L.label} {hms(L.tn[a])[:5]}-{hms(L.tn[b - 1])[:5]}, {b - a} fixes', fontsize=9.5, weight='bold', color=INK)
        ax.set_aspect('equal', adjustable='datalim'); ax.set_xlabel('east (m)'); ax.set_ylabel('north (m)')
    header(fig, 'GPS jitter while still', 'Each log\'s longest stretch with the boat moving under 0.2 m/s')
    save(fig, '21_gps_jitter')

    # 22 battery, 23 battery around events
    if any((L.f('battery_voltage') > 1000).any() for L in logs):
        fig, A = cols(3, 6.4, ratios=[0.5, 2.5, 2.5], sharey='row')
        for k, (L, (am, ap, ac)) in enumerate(zip(logs, A)):
            pack = L.f('battery_voltage')
            mode_strip(am, L)
            ap.plot(*gapped(L.tn, np.where(pack > 1000, pack / 1000, np.nan)), color=S[0])
            cells = np.array([r['cell_voltage'] for r in L.rx], float)
            for j in range(cells.shape[1]):
                ac.plot(*gapped(L.tn, np.where(cells[:, j] > 1000, cells[:, j] / 1000, np.nan)), color=S[j], label=f'cell {j + 1}')
            first(k, ap, 'pack (V)'); first(k, ac, 'cell (V)'); time_axis(ac)
        legend_right(A[-1][2])
        header(fig, 'Battery', f'{when}. Pack and cell voltages, with the mode above')
        save(fig, '22_battery')

        fig, A = cols(1, 4.6, sharey='row')
        for k, (L, (ax,)) in enumerate(zip(logs, A)):
            pack = L.f('battery_voltage')
            ev = [(i, 'boat reboot', S[0]) for i in np.where(L.reboot)[0]] + \
                 [(i, 'radio re-init', S[1]) for i in np.where(L.reinit)[0]] + [(i, 'SAFE', S[2]) for i, _ in L.safes]
            for i, lab, c in ev:
                m = (L.ts >= L.ts[i] - 30) & (L.ts <= L.ts[i] + 30) & (pack > 1000)
                ax.plot(L.ts[m] - L.ts[i], pack[m] / 1000, color=c, linewidth=1.2, alpha=0.8)
            ax.axvline(0, color=INK2, linewidth=0.9); ax.set_xlabel('seconds from the event'); first(k, ax, 'pack (V)')
            if not ev:
                ax.text(0.5, 0.5, 'no reboots, radio re-inits or SAFE', transform=ax.transAxes, ha='center', color=INK2)
        A[-1][0].legend(handles=[Line2D([], [], color=c, label=lab) for lab, c in
                                 (('boat reboot', S[0]), ('radio re-init', S[1]), ('SAFE', S[2]))],
                        loc='upper left', bbox_to_anchor=(1.0, 1.0))
        header(fig, 'Battery around events', f'{when}. Pack voltage 30 s either side of each reboot, radio re-init and SAFE '
               '(1 Hz, so short dips can hide)')
        save(fig, '23_battery_at_events')

    # 24 the boat's view of the ground station over time
    have_d = any(np.isfinite(L.dist).any() for L in logs)
    fig, A = cols(3 if have_d else 2, 7.4 if have_d else 5.2, sharey='row')
    for k, (L, axs) in enumerate(zip(logs, A)):
        axs[0].plot(*gapped(L.tn, L.rssi), color=S[0], label='signal (RSSI)')
        axs[0].plot(*gapped(L.tn, L.noise), color=S[1], label='noise floor')
        axs[1].axhline(0, color=AXIS, linewidth=0.8); axs[1].plot(*gapped(L.tn, L.snr), color=S[0])
        hline(axs[1], SF8_SNR_LIMIT, f'SF8 decode limit {SF8_SNR_LIMIT} dB', va='top')
        first(k, axs[0], 'dBm'); first(k, axs[1], 'SNR (dB)')
        if have_d:
            axs[2].plot(*gapped(L.tn, L.dist), color=S[0]); axs[2].set_ylim(bottom=0); first(k, axs[2], 'distance (m)')
        time_axis(axs[-1])
    legend_right(A[-1][0])
    header(fig, 'Ground station signal at the boat', f'{when}. Measured by the boat\'s radio on each packet it heard (gaps: heard nothing)')
    save(fig, '24_uplink_signal')

    # 25 coverage map
    if have_d:
        fig, A = cols(1, 7, w=7)
        for L, (ax,) in zip(logs, A):
            deaf = np.isfinite(L.x) & (L.age >= 2)
            ax.scatter(L.x[deaf], L.y[deaf], s=12, marker='x', color=MUTED, linewidths=0.8, label='boat not hearing the ground station')
            sc = ax.scatter(L.x, L.y, c=L.snr, cmap=SEQ, s=14, linewidths=0, vmin=-15, vmax=10)
            gs_marker(ax, L)
            ax.set_aspect('equal', adjustable='datalim'); ax.set_xlabel('east (m)'); ax.set_ylabel('north (m)')
        A[0][0].legend(loc='best', fontsize=8)
        fig.colorbar(sc, ax=A[-1][0], shrink=0.7, label=f'SNR at the boat (dB), decode limit {SF8_SNR_LIMIT}')
        header(fig, 'Command coverage', f'{when}. Where the boat heard the ground station, coloured by SNR')
        save(fig, '25_coverage_map')

    # 26 commands, 27 command acknowledgement
    outcome_style = {'acknowledged': (GOOD, 'o'), 'refused': (WARN, '^'), 'never acknowledged': (CRIT, 'X')}
    with_cmds = [L for L in logs if L.cmds]
    if with_cmds:
        most = max(len({c['name'] for c in L.cmds}) for L in logs)
        fig, A = cols(2 if have_d else 1, 2.6 + most * 0.4, ratios=[max(most, 3) * 0.4, 2.2] if have_d else None)
        for k, (L, axs) in enumerate(zip(logs, A)):
            names = sorted({c['name'] for c in L.cmds})
            ax = axs[0]
            for o, (c, mk) in outcome_style.items():
                sel = [c_ for c_ in L.cmds if c_['outcome'] == o]
                if sel:
                    ax.scatter([mdates.date2num(c_['sends'][0]) for c_ in sel], [names.index(c_['name']) for c_ in sel],
                               color=c, marker=mk, s=42, linewidths=0, zorder=3)
            ax.set_yticks(range(len(names))); ax.set_yticklabels(names); ax.set_ylim(-0.7, max(len(names), 1) - 0.3)
            counts = {o: sum(1 for c_ in L.cmds if c_['outcome'] == o) for o in outcome_style}
            ax.set_title(f'{log_span(L)}: ' + ', '.join(f'{v} {o}' for o, v in counts.items() if v), fontsize=9.5,
                         weight='bold', color=INK)
            if have_d:
                ad = axs[1]
                ad.plot(*gapped(L.tn, L.dist), color=S[0])
                for o, (c, mk) in outcome_style.items():
                    sel = [c_ for c_ in L.cmds if c_['outcome'] == o]
                    ad.scatter([mdates.date2num(c_['sends'][0]) for c_ in sel], [c_['dist'] for c_ in sel], color=c,
                               marker=mk, s=36, linewidths=0, zorder=3)
                ad.set_ylim(bottom=0); first(k, ad, 'boat distance (m)')
            time_axis(axs[-1])
        A[-1][0].legend(handles=[Line2D([], [], color=c, marker=mk, linestyle='none', label=o) for o, (c, mk) in outcome_style.items()],
                        loc='upper left', bbox_to_anchor=(1.0, 1.0))
        header(fig, 'Commands', f'{when}. Each command at its first send; gs_web resends up to 5 times')
        save(fig, '26_commands')

        fig, A = rows(2, 3.6)
        for L, (a1, a2) in zip(logs, A):
            lat = [(c['ack_t'] - c['sends'][0]).total_seconds() for c in L.cmds if c['ack_t']]
            if lat:
                a1.hist(lat, bins=np.arange(0, max(lat) + 1, 0.5), color=S[0], edgecolor=SURFACE, linewidth=1.2)
                a1.text(0.98, 0.95, f'median {np.median(lat):.1f} s', transform=a1.transAxes, ha='right', va='top', fontsize=8.5, color=INK2)
            a1.set_xlabel('first send to acknowledgement (s)'); a1.set_ylabel('commands')
            tries = np.arange(1, 6)
            for j, (o, (c, _)) in enumerate(outcome_style.items()):
                cnt = [sum(1 for c_ in L.cmds if c_['outcome'] == o and len(c_['sends']) == t) for t in tries]
                a2.bar(tries + (j - 1) * 0.27, cnt, width=0.25, color=c, label=o)
            a2.set_xticks(tries); a2.set_xlabel('sends'); a2.set_ylabel('commands')
        A[0][1].legend(loc='upper center', fontsize=7.5)
        header(fig, 'Command acknowledgement', 'Left: how long acknowledged commands took. Right: sends per command')
        save(fig, '27_command_ack')

    # 28 seconds since the boat last heard the ground station
    fig, A = cols(2, 5, ratios=[0.5, 4], sharey='row')
    for k, (L, (am, ax)) in enumerate(zip(logs, A)):
        mode_strip(am, L)
        ax.plot(*gapped(L.tn, np.minimum(L.age, 60)), color=S[0])
        hline(ax, LINK_TIMEOUT_S, f'link timeout {LINK_TIMEOUT_S} s (manual and semi modes go to SAFE)', color=CRIT)
        for i, cause in L.safes:
            ax.axvline(L.tn[i], color=INK2, linewidth=0.8)
            ax.text(L.tn[i], 58, f' SAFE: {cause}', rotation=90, fontsize=7, color=INK2, va='top', ha='right', path_effects=HALO)
        ax.set_ylim(0, 60); first(k, ax, 'seconds (clipped at 60)'); time_axis(ax)
    header(fig, 'Time since the boat last heard the ground station', when)
    save(fig, '28_link_age')

    # 29 running totals
    fig, A = cols(2, 5.6)
    for k, (L, (a1, a2)) in enumerate(zip(logs, A)):
        def total(v):
            inc = np.diff(v) % 65536
            inc[L.reboot[1:]] = v[1:][L.reboot[1:]]          # counters restart from 0 after a reboot
            return np.r_[0, np.cumsum(inc)]
        sent_t = mdates.date2num([x['t'] for x in L.tx])
        a1.plot(sent_t, np.arange(1, len(sent_t) + 1), color=S[0], label='sent by the ground station')
        a1.plot(L.tn, total(L.f('rx_ok')), color=S[1], label='received by the boat (good CRC)')
        a2.plot(L.tn, total(L.f('rx_crc_err')), color=S[0])
        first(k, a1, 'packets'); first(k, a2, 'CRC errors at the boat'); time_axis(a2)
    legend_right(A[-1][0])
    header(fig, 'Radio counters', f'{when}. Running totals; the gap between the two upper lines is ground station packets the boat never got')
    save(fig, '29_rx_counters')

    # 30 event timeline
    fig, A = rows(1, 3.2, w=13)
    for L, (ax,) in zip(logs, A):
        strip(ax, spans(L, L.mode), lambda v: MODE_COLOR.get(v, MUTED), lambda v: BL.MODE_NAMES.get(v, str(v)), row=0)
        ltp = np.where(np.isin(L.nav, list(NAV_COLOR)), L.nav, -1)
        strip(ax, spans(L, ltp), lambda v: NAV_COLOR.get(v), lambda v: nav_name(v) if v >= 0 else '', row=1)
        for idx, mk, c in ((np.where(L.reboot)[0], '^', INK), (np.where(L.reinit)[0], 'D', S[1])):
            ax.scatter(L.tn[idx], np.full(len(idx), 2), marker=mk, color=c, s=40, zorder=3)
        for i, cause in L.safes:
            ax.scatter(L.tn[i], 2, marker='X', color=CRIT, s=46, zorder=3)
            ax.annotate(cause, (L.tn[i], 2), xytext=(0, 8), textcoords='offset points', fontsize=7, ha='center', color=INK2,
                        path_effects=HALO)
        route_t = mdates.date2num([x['t'] for x in L.tx if x['type'] == 0x33])
        ax.scatter(route_t, np.full(len(route_t), 2), marker='s', color=S[6], s=36, zorder=3)
        for o, (c, _) in outcome_style.items():
            ts_ = [mdates.date2num(c_['sends'][0]) for c_ in L.cmds if c_['outcome'] == o]
            ax.scatter(ts_, np.full(len(ts_), 3), marker='|', color=c, s=120, linewidths=1.6)
        ax.set_yticks(range(4)); ax.set_yticklabels(['mode', 'LTP', 'events', 'commands']); ax.set_ylim(3.6, -0.6)
        ax.grid(axis='y', visible=False); ax.spines['left'].set_visible(False); ax.tick_params(axis='y', length=0)
        time_axis(ax)
    A[0][0].legend(handles=[Line2D([], [], marker='^', color=INK, linestyle='none', label='boat reboot'),
                            Line2D([], [], marker='D', color=S[1], linestyle='none', label='radio re-init'),
                            Line2D([], [], marker='s', color=S[6], linestyle='none', label='route committed'),
                            Line2D([], [], marker='X', color=CRIT, linestyle='none', label='SAFE (cause above)')] +
                   [Line2D([], [], marker='|', color=c, linestyle='none', markersize=10, markeredgewidth=1.6, label=f'command {o}')
                    for o, (c, _) in outcome_style.items()],
                   loc='upper left', bbox_to_anchor=(1.0, 1.0), fontsize=7.5)
    header(fig, 'Event timeline', when)
    save(fig, '30_event_timeline')

    # 34 compass heading against GPS course: course over fixes ~10 s apart, compass averaged over the same 10 s,
    # only while moving > 0.5 m/s (slower, GPS jitter swamps the course)
    if have_d:
        fig, A = cols(3, 11.5, ratios=[3, 1.6, 1.8], w=6.4, sharex=False)
        for L, (a1, a2, a3) in zip(logs, A):
            sog, cog, hdg = L.motion(5)
            m = (sog > 0.5) & np.isfinite(hdg)
            cog, hdg = cog[m], hdg[m]
            d = wrap180(hdg - cog)
            if d.size < 20:
                a1.text(0.5, 0.5, 'not enough time moving > 0.5 m/s', transform=a1.transAxes, ha='center', color=INK2)
                continue
            # Main peak: the most common 10° bin, refined to the median of everything within 30° of it. A peak more than
            # 90° out is the boat moving stern first (rolled backwards on land), so the bow-first one is 180° from it
            cnt, edges = np.histogram(d, bins=np.arange(-180, 181, 10))
            mode = edges[np.argmax(cnt)] + 5
            if abs(mode) > 90:
                mode = float(wrap180(mode + 180))
            near = np.abs(wrap180(d - mode)) <= 30
            offset = float(np.median(wrap180(d[near] - mode)) + mode)
            stern = np.abs(wrap180(d - offset - 180)) <= 30

            a1.scatter(cog, hdg, s=9, color=S[0], alpha=0.45, linewidths=0)
            g = np.array([0, 360])
            a1.plot(g, g, color=INK2, linewidth=0.9)
            a1.text(200, 200, 'compass = GPS course', fontsize=7.5, color=INK2, rotation=45, ha='center', va='bottom', path_effects=HALO)
            for lo in (-180, 180):
                a1.plot(g, g + lo, color=MUTED, linewidth=0.8)
            a1.text(270, 90, 'moving stern first', fontsize=7.5, color=INK2, rotation=45, ha='center', va='bottom', path_effects=HALO)
            a1.set_xlim(0, 360); a1.set_ylim(0, 360); a1.set_xticks(range(0, 361, 90)); a1.set_yticks(range(0, 361, 90))
            a1.set_aspect('equal'); a1.set_xlabel('GPS course over ground (deg)'); a1.set_ylabel('compass heading (deg)')

            a2.hist(d, bins=edges, color=S[0], edgecolor=SURFACE, linewidth=1.2)
            a2.axvline(offset, color=INK2, linewidth=0.9)
            a2.text(offset, a2.get_ylim()[1] * 0.95, f' {offset:+.0f}°', fontsize=8.5, color=INK, va='top', path_effects=HALO)
            a2.set_xlim(-180, 180); a2.set_xticks(range(-180, 181, 90))
            a2.set_xlabel('compass - GPS course (deg)'); a2.set_ylabel('samples')
            a2.set_title(f'{d.size} samples; {100 * near.mean():.0f}% within 30° of the bow-first peak'
                         + (f', {100 * stern.mean():.0f}% 180° from it (moving stern first)' if stern.mean() >= 0.05 else ''),
                         fontsize=8.5)

            a3.axhline(0, color=AXIS, linewidth=0.8); a3.axhline(offset, color=INK2, linewidth=0.9)
            a3.scatter(cog[near], d[near], s=9, color=S[0], alpha=0.45, linewidths=0)
            c, med = binned(cog[near], d[near], np.arange(0, 361, 30))
            a3.plot(c, med, color=S[1], linewidth=2, label='median per 30°')
            a3.set_xlim(0, 360); a3.set_xticks(range(0, 361, 90)); a3.set_ylim(offset - 45, offset + 45)
            a3.set_xlabel('GPS course over ground (deg)'); a3.set_ylabel('compass - GPS course (deg)')
            a3.set_title('error by direction near the peak; a slope or wave means calibrate the compass', fontsize=8.5)
            a3.legend(loc='upper right', fontsize=7.5)
            a1.set_title(f'{log_span(L)}: compass {offset:+.0f}° from GPS course', fontsize=9.5, weight='bold', color=INK)
        header(fig, 'Compass heading against GPS course',
               f'{when}. GPS course from the boat\'s fixes about 10 s apart while it moved faster than 0.5 m/s, against the '
               'compass averaged over the same 10 s. On land the difference is the compass error (if the boat moved bow first); '
               'on water it also holds leeway and current.')
        save(fig, '34_compass_vs_gps_course')
    return made


# ─── 31-33: comparing logs ─────────────────────────────────────────────

def expected_rssi(d, tx_dbm=17, gains_dbi=4, h1=1.0, h2=1.5, f_hz=915e6):
    """Free space up to the two-ray breakpoint, then the two-ray ground (water) reflection model."""
    lam = 3e8 / f_hz
    fspl = 20 * np.log10(4 * np.pi * d / lam)
    two_ray = 40 * np.log10(d) - 20 * np.log10(h1 * h2)
    return tx_dbm + gains_dbi - np.where(d < 4 * h1 * h2 / lam, fspl, np.maximum(fspl, two_ray))


def compare_graphs(logs, out):
    made = []
    with_d = [L for L in logs if L.n and not L.baseline and np.isfinite(L.dist).any()]
    labels = ', '.join(L.label for L in with_d)
    where = '' if with_d and with_d[0].ref_given else '. Distances are from each log\'s first GPS fix, so wrong once the ground station moved'

    def save(fig, name):
        path = os.path.join(out, f'{name}.png')
        fig.savefig(path)
        plt.close(fig)
        made.append(path)

    # 31 signal and SNR against distance
    if with_d:
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        dmax = max(np.nanmax(L.dist) for L in with_d)
        d = np.linspace(1, dmax, 300)
        a1.plot(d, expected_rssi(d), color=INK2, linewidth=1.2, label='healthy-hardware estimate (17 dBm, 2 dBi each end, 1 m and 1.5 m up)')
        for k, L in enumerate(with_d):
            a1.scatter(L.dist, L.rssi, s=6, color=S[k], alpha=0.25, linewidths=0)
            c, med = binned(L.dist, L.rssi, np.arange(0, dmax + 25, 25))
            a1.plot(c, med, color=S[k], linewidth=2, label=f'{L.label} (median per 25 m)')
            a2.scatter(L.dist, L.snr, s=6, color=S[k], alpha=0.25, linewidths=0)
            c, med = binned(L.dist, L.snr, np.arange(0, dmax + 25, 25))
            a2.plot(c, med, color=S[k], linewidth=2, label=L.label)
        a1.set_ylabel('signal at the boat (dBm)'); a1.legend(loc='upper right', fontsize=7.5)
        a2.axhline(0, color=AXIS, linewidth=0.8); hline(a2, SF8_SNR_LIMIT, f'SF8 decode limit {SF8_SNR_LIMIT} dB', va='top')
        a2.set_ylabel('SNR (dB)'); a2.set_xlabel('distance from the ground station (m)'); a2.set_xlim(left=0)
        a2.legend(loc='upper right', fontsize=7.5)
        header(fig, 'Ground station signal against distance', f'{labels}. The gap to the estimate is what the radio hardware is losing{where}')
        save(fig, '31_uplink_vs_distance')

    # 32 delivery in both directions against distance
    if with_d:
        fig, axs = plt.subplots(len(with_d), 1, figsize=(10, 3.4 * len(with_d)), squeeze=False)
        sent_by_log = {L.label: np.array([mdates.date2num(x['t']) for x in L.tx]) for L in with_d}
        edges = [0, 25, 50, 100, 150, 200, 250, 300, 400, 600, 1000]
        for ax, L in zip(axs[:, 0], with_d):
            sent = sent_by_log[L.label]
            ok = L.f('rx_ok')
            up_n, up_d, dn_n, dn_d = {}, {}, {}, {}
            for i in range(1, L.n):
                if L.reboot[i] or not np.isfinite(L.dist[i]) or L.ts[i] - L.ts[i - 1] > GAP_S:
                    continue
                b = next((k for k in range(len(edges) - 1) if edges[k] <= L.dist[i] < edges[k + 1]), None)
                if b is None:
                    continue
                n_sent = np.sum((sent > L.tn[i - 1]) & (sent <= L.tn[i]))
                # Summed per band, not capped per interval: a send can be logged in the interval before the one it counts in
                up_n[b] = up_n.get(b, 0) + (ok[i] - ok[i - 1]) % 65536
                up_d[b] = up_d.get(b, 0) + n_sent
                gap = (L.rx[i]['seq'] - L.rx[i - 1]['seq'] - 1) % 256
                dn_n[b] = dn_n.get(b, 0) + 1
                dn_d[b] = dn_d.get(b, 0) + 1 + gap
            bins = sorted(set(up_d) | set(dn_d))
            xs = np.arange(len(bins))
            up = [min(100.0, 100 * up_n.get(b, 0) / up_d[b]) if up_d.get(b) else np.nan for b in bins]
            dn = [100 * dn_n.get(b, 0) / dn_d[b] if dn_d.get(b) else np.nan for b in bins]
            ax.bar(xs - 0.21, up, width=0.4, color=S[0], label='ground station → boat (commands, heartbeats)')
            ax.bar(xs + 0.21, dn, width=0.4, color=S[1], label='boat → ground station (telemetry)')
            for k, (u, v) in enumerate(zip(up, dn)):
                if np.isfinite(u):
                    ax.text(k - 0.21, u, f'{u:.0f}', ha='center', va='bottom', fontsize=7, color=INK2)
                if np.isfinite(v):
                    ax.text(k + 0.21, v, f'{v:.0f}', ha='center', va='bottom', fontsize=7, color=INK2)
            ax.set_xticks(xs); ax.set_xticklabels([f'{edges[b]}-{edges[b + 1]}' for b in bins])
            ax.set_ylim(0, 112); ax.set_ylabel('% delivered'); ax.set_title(L.label, fontsize=9.5, weight='bold', color=INK)
            ax.grid(axis='x', visible=False); legend_right(ax, fontsize=7.5)
        axs[-1, 0].set_xlabel('distance from the ground station (m)')
        header(fig, 'Delivery against distance, both directions', f'{labels}. Share of packets that arrived, per distance band{where}')
        save(fig, '32_delivery_vs_distance')

    # 33 noise floor at the boat
    fig, ax = plt.subplots(figsize=(9, 4.2))
    bins = np.arange(-125, -55, 1)
    for k, L in enumerate([L for L in logs if L.n]):
        v = L.noise[np.isfinite(L.noise)]
        if v.size:
            ax.hist(v, bins=bins, histtype='step', linewidth=1.6, color=S[k], density=True,
                    label=f'{L.label}: median {np.median(v):.0f} dBm ({v.size} samples)')
    ax.axvline(-114, color=INK2, linewidth=0.9)
    ax.text(-114, ax.get_ylim()[1] * 0.97, ' thermal floor ~-114 dBm', fontsize=7.5, color=INK2, va='top')
    ax.set_xlabel('noise floor at the boat (dBm)'); ax.set_ylabel('share of samples'); ax.legend(loc='upper right')
    header(fig, 'Noise floor at the boat', f'{", ".join(L.label for L in logs if L.n)}. Each dB above the thermal floor costs commands a dB of range margin')
    save(fig, '33_noise_floor')
    return made


def main():
    # tight_layout warns about polar axes and colorbars, then lays them out fine
    warnings.filterwarnings('ignore', message='This figure includes Axes that are not compatible with tight_layout')
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('logs', nargs='+', metavar='LABEL=LOG', help='a log, optionally named: water=log/gs_log_....txt')
    ap.add_argument('--out', help='output folder (default: plots_<date of the first log>/ next to it)')
    ap.add_argument('--ref', help='ground station position LAT,LON (default: the boat\'s first GPS fix in each log)')
    ap.add_argument('--baseline', action='append', default=[], metavar='LABEL=LOG',
                    help='a log used only in the noise-floor comparison, e.g. a bench run (no per-log graphs, no distances)')
    args = ap.parse_args()
    ref = tuple(float(v) for v in args.ref.split(',')) if args.ref else None
    logs = []
    for item, baseline in [(a, False) for a in args.logs] + [(a, True) for a in args.baseline]:
        label, _, path = item.rpartition('=')
        label = label or re.sub(r'^gs_log_|\.txt$', '', os.path.basename(path))
        logs.append(Log(label, path, ref))
        logs[-1].baseline = baseline
        print(f'{label}: {logs[-1].n} telemetry packets, {len(logs[-1].tx)} sent' + (' (baseline)' if baseline else ''))
    m = re.search(r'(\d{8})_\d{6}', os.path.basename(logs[0].path))
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(logs[0].path)), f'plots_{m.group(1) if m else "log"}')
    os.makedirs(out, exist_ok=True)
    made = []
    runs = [L for L in logs if L.n and not L.baseline]
    for L in runs:
        run = L.auto_run()
        if run:
            made += ltp_graphs(L, run, out)
    if runs:
        made += per_log_graphs(runs, out)
    made += compare_graphs(logs, out)
    print(f'{len(made)} graphs in {out}')


if __name__ == '__main__':
    main()
