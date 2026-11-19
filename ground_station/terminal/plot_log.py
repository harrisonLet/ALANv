#!/usr/bin/env python3
"""
plot_log.py - turn a gs_term session log into CSVs, PNG plots and an HTML page

Decodes the raw RX-RAW bytes (not the decoded notes), so the plots show exactly what came
over the radio. Commands and their results come from the TX / RESULT lines.

Usage:   python3 plot_log.py log/gs_log_20261118_160241.txt [out_dir] [--no-map]
Output:  out_dir (default: next to the log, <log name>_plots/) with
           telemetry.csv, commands.csv, 01_heading.png ... 12_track_satellite.png, index.html
Needs:   numpy, matplotlib. The satellite map also needs contextily and internet; it is skipped without them.
"""

import csv
import html
import math
import os
import re
import struct
import sys
from datetime import datetime, timedelta

import matplotlib
matplotlib.use('Agg')
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# Packet layout, must match LoRaTelemetry_t in telemetry.h (73 bytes, packed, little-endian)
PKT_HDR = 5
TELEM_FMT = '<HHHHHB hHBB hB ddfBBBBBB BbB HhhB hh BHBhHIB'.replace(' ', '')
TELEM_FIELDS = ('battery_mv c1 c2 c3 c4 battery_valid sail_ang encoder_mag encoder_status encoder_valid '
                'wind_ang wind_valid lat lon alt sats fix utc_h utc_m utc_s gps_valid imu_cal imu_temp imu_valid '
                'heading roll pitch orientation_valid rudder_servo sail_servo mode desired_heading route_segment '
                'drift_m last_cmd_seq uptime_ms last_cmd_result').split()
assert struct.calcsize(TELEM_FMT) == 73

RESULTS = ['NONE', 'OK', 'UNSUPPORTED', 'BAD_LEN', 'BAD_VALUE', 'TOO_MANY', 'INCOMPLETE', 'BAD_CRC', 'DROPPED', 'WRONG_MODE']
OUTCOME = {'OK': 'OK', 'WRONG_MODE': 'Refused (wrong mode)',
           'not reported': 'Superseded before report', 'no report': 'Never acknowledged'}


def mode_names():
    """SYSTEM_MODE_* from commands.h, so new modes show up without editing this script"""
    names = {}
    try:
        for m in re.finditer(r'SYSTEM_MODE_(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)', open(os.path.join(HERE, 'commands.h')).read()):
            if m.group(1) != 'COUNT':
                names[int(m.group(2), 0)] = m.group(1)
    except OSError:
        pass
    return names


# ─── Decode ────────────────────────────────────────────────────────

def decode(log_path):
    date = re.search(r'(\d{4})(\d{2})(\d{2})_\d{6}', os.path.basename(log_path))
    day = f'{date.group(1)}-{date.group(2)}-{date.group(3)}' if date else '2000-01-01'
    modes = mode_names()

    stream, stamps, cmds = bytearray(), [], []
    for line in open(log_path, errors='replace'):
        p = line.split()
        if len(p) < 3:
            continue
        if p[1] == 'RX-RAW':
            b = bytes(int(x, 16) for x in p[3:])
            stream += b
            stamps += [p[0]] * len(b)
        elif p[1] == 'TX' and p[2] == 'seq' and p[4:5] != ['hb'] and 'WRITE FAILED' not in line:
            cmds.append(dict(datetime=f'{day}T{p[0]}', time=p[0], seq=int(p[3]),
                             command=' '.join(p[4:]), result='no report'))
        elif p[1] == 'RESULT':
            m = re.match(r'.*RESULT\s+seq\s+(\d+)\s+.*->\s+(.*)$', line.rstrip())
            if m:
                for c in reversed(cmds):
                    if c['seq'] == int(m.group(1)) and c['result'] == 'no report':
                        c['result'] = m.group(2)
                        break
    for c in cmds:
        c['outcome'] = OUTCOME.get(c['result'], c['result'])

    rows, i, prev_seq, prev_up = [], 0, None, None
    plen = PKT_HDR + 73
    while i + plen <= len(stream):
        if stream[i] == 0xAA and stream[i + 1] == 0xBB and stream[i + 3] == 74:
            r = dict(datetime=f'{day}T{stamps[i]}', time=stamps[i], seq=stream[i + 2], flags=stream[i + 4])
            r.update(zip(TELEM_FIELDS, struct.unpack_from(TELEM_FMT, stream, i + PKT_HDR)))
            # A restart shows as uptime going backwards; its seq jump is not lost packets
            r['reboot'] = int(prev_up is not None and r['uptime_ms'] < prev_up)
            r['lost'] = 0 if prev_seq is None or r['reboot'] else (r['seq'] - prev_seq - 1) % 256
            prev_seq, prev_up = r['seq'], r['uptime_ms']
            r['mode_name'] = modes.get(r['mode'], str(r['mode']))
            r['last_cmd_result_name'] = RESULTS[r['last_cmd_result']] if r['last_cmd_result'] < len(RESULTS) else str(r['last_cmd_result'])
            r['desired_heading'] = None if r['desired_heading'] == 0xFFFF else r['desired_heading']
            rows.append(r)
            i += plen
        else:
            i += 1
    return rows, cmds


def write_csv(path, rows):
    if not rows:
        return
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)


# ─── Plots ─────────────────────────────────────────────────────────

BLUE, ORANGE, GREEN, PURPLE = '#2a78d6', '#e8833a', '#2e9e6a', '#8a5cc9'
OK, WARN, BAD, MUTED, GRID = '#2e9e6a', '#e0a020', '#d64545', '#9a9893', '#e6e5e1'
MODE_COLORS = [BLUE, ORANGE, GREEN, PURPLE, '#c94f7c', '#5a9fb5', MUTED]


def plots(T, C, out, want_map):
    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': '#bbb', 'axes.grid': True, 'grid.color': GRID,
                         'axes.spines.top': False, 'axes.spines.right': False, 'figure.dpi': 150})
    made = []
    dt = datetime.fromisoformat
    t = np.array([dt(r['datetime']) for r in T])
    f = lambda k: np.array([np.nan if r[k] is None else float(r[k]) for r in T])

    rst = [dt(r['datetime']) for r in T if r['reboot']]
    last_ack = T[-1]['last_cmd_seq']
    ack = [dt(c['datetime']) for c in C if c['seq'] == last_ack][-1:]
    events = rst + ack
    note = 'dashed lines: ' + ', '.join([f"restart {e.strftime('%H:%M:%S')}" for e in rst] +
                                        [f"last acknowledged command {e.strftime('%H:%M:%S')}" for e in ack])

    def wrapbreak(v):
        v = v.copy()
        v[1:][np.abs(np.diff(v)) > 180] = np.nan   # no streak across the chart at 359 -> 0
        return v

    def timefig(title, ylabel, h=3.6):
        fig, ax = plt.subplots(figsize=(10, h))
        ax.set_title(title, loc='left', fontweight='bold')
        ax.set_ylabel(ylabel)
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        for e in events:
            ax.axvline(e, color=MUTED, ls='--', lw=1, zorder=0)
        if events:
            ax.text(1, 1.02, note, transform=ax.transAxes, ha='right', va='bottom', fontsize=8, color=MUTED)
        ax.set_xlim(t[0], t[-1])
        return fig, ax

    def save(fig, name, title):
        fig.tight_layout()
        fig.savefig(os.path.join(out, name))
        plt.close(fig)
        made.append((name, title))
        print('  ' + name)

    fig, ax = timefig('Heading vs heading target', 'degrees')
    ax.scatter(t, f('heading'), s=3, color=BLUE, label='fused heading')
    ax.step(t, wrapbreak(f('desired_heading')), where='post', color=ORANGE, lw=1.5, label='heading PID target')
    ax.set_ylim(0, 360)
    ax.set_yticks(range(0, 361, 90))
    ax.legend(loc='upper left', fontsize=8)
    save(fig, '01_heading.png', 'Heading vs heading target')

    fig, ax = timefig('Rudder servo', 'degrees')
    ax.plot(t, f('rudder_servo'), color=BLUE, lw=1.3, label='servo angle the boat drove')
    rs = [(dt(c['datetime']), float(c['command'].split()[1])) for c in C if c['command'].startswith('rsang ')]
    if rs:
        ax.scatter(*zip(*rs), s=18, color=ORANGE, edgecolor='white', zorder=3, label='rsang commands sent')
    ax.legend(loc='upper left', fontsize=8)
    save(fig, '02_rudder.png', 'Rudder servo')

    fig, ax = timefig('Main sail angle and apparent wind', 'degrees from bow (+ starboard)')
    ax.plot(t, f('sail_ang'), color=BLUE, lw=1.3, label='sail angle (encoder)')
    ax.plot(t, wrapbreak(f('wind_ang')), color=ORANGE, lw=1.0, label='apparent wind')
    ax.set_ylim(-180, 180)
    ax.set_yticks(range(-180, 181, 90))
    ax.legend(loc='upper left', fontsize=8)
    save(fig, '03_sail_wind.png', 'Main sail angle and apparent wind')

    fig, ax = timefig('Tail servo', 'degrees')
    ax.plot(t, f('sail_servo'), color=BLUE, lw=1.3)
    save(fig, '04_tail_servo.png', 'Tail servo')

    fig, ax = timefig('Roll and pitch', 'degrees')
    ax.plot(t, f('roll'), color=BLUE, lw=1.1, label='roll (+ starboard down)')
    ax.plot(t, f('pitch'), color=ORANGE, lw=1.1, label='pitch (+ bow up)')
    ax.legend(loc='upper left', fontsize=8)
    save(fig, '05_roll_pitch.png', 'Roll and pitch')

    # Validity 00 rows carry zeros, not readings
    bad = f('battery_valid') == 0
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for a in (a1, a2):
        for e in events:
            a.axvline(e, color=MUTED, ls='--', lw=1, zorder=0)
    pk = f('battery_mv') / 1000
    pk[bad | (pk == 0)] = np.nan
    a1.plot(t, pk, color=BLUE, lw=1.3)
    a1.set_title('Battery pack voltage', loc='left', fontweight='bold')
    a1.set_ylabel('V')
    for k, col in zip(['c1', 'c2', 'c3', 'c4'], [BLUE, ORANGE, GREEN, PURPLE]):
        v = f(k) / 1000
        v[bad | (v == 0)] = np.nan
        a2.plot(t, v, color=col, lw=1.1, label=f'cell {k[1]}')
    a2.set_title('Battery cells', loc='left', fontweight='bold')
    a2.set_ylabel('V')
    a2.legend(loc='lower left', ncol=4, fontsize=8)
    a2.set_xlim(t[0], t[-1])
    a2.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    save(fig, '06_battery.png', 'Battery pack and cells')

    fig, ax = timefig('GPS satellites and fix type', 'count', h=3)
    ax.step(t, f('sats'), where='post', color=BLUE, lw=1.3, label='satellites')
    ax.step(t, f('fix'), where='post', color=ORANGE, lw=1.3, label='fix type (1 GPS, 2 DGPS)')
    ax.set_ylim(0, 14)
    ax.legend(loc='lower right', fontsize=8)
    save(fig, '07_gps_quality.png', 'GPS satellites and fix type')

    sec = np.array([(x - t[0]).total_seconds() for x in t])
    b = (sec // 30).astype(int)
    rx, lost = np.bincount(b), np.bincount(b, weights=f('lost'))
    starts = [t[0] + timedelta(seconds=30 * i) for i in range(len(rx))]
    fig, ax = timefig('Radio link: telemetry packets per 30 s', 'packets', h=3)
    ax.bar(starts, rx, width=30 / 86400 * 0.9, align='edge', color=BLUE, label='received')
    ax.bar(starts, lost, width=30 / 86400 * 0.9, align='edge', bottom=rx, color=BAD, label='lost')
    ax.set_ylim(0, max(42, (rx + lost).max() * 1.3))
    ax.legend(loc='upper left', ncol=2, fontsize=8)
    save(fig, '08_link.png', 'Radio link')

    cats = ['Never acknowledged', 'Refused (wrong mode)', 'Superseded before report', 'OK']
    ccol = {'OK': OK, 'Refused (wrong mode)': BAD, 'Never acknowledged': WARN, 'Superseded before report': MUTED}
    fig, ax = timefig('Every command sent, by outcome', '', h=2.8)
    for c in C:
        y = cats.index(c['outcome']) if c['outcome'] in cats else 2
        ax.scatter(dt(c['datetime']), y, s=30, color=ccol.get(c['outcome'], MUTED), edgecolor='white', zorder=3)
    ax.set_yticks(range(len(cats)))
    ax.set_yticklabels(cats)
    ax.set_ylim(-0.6, len(cats) - 0.4)
    save(fig, '09_commands.png', 'Commands by outcome')

    fields = [('battery_valid', 'battery'), ('encoder_valid', 'sail encoder'), ('wind_valid', 'wind vane'),
              ('gps_valid', 'GPS'), ('imu_valid', 'IMU'), ('orientation_valid', 'orientation')]
    vcol = {3: OK, 2: WARN, 1: WARN, 0: BAD}
    fig, ax = plt.subplots(figsize=(10, 2.8))
    ax.grid(False)
    xs = mdates.date2num(t)
    dx = np.diff(xs, append=xs[-1] + (xs[-1] - xs[-2] if len(xs) > 1 else 1 / 86400))
    for i, (k, _) in enumerate(fields):
        ax.barh(len(fields) - 1 - i, dx, left=xs, height=0.7, color=[vcol.get(int(v), MUTED) for v in f(k)], linewidth=0)
    ax.set_yticks(range(len(fields)))
    ax.set_yticklabels([n for _, n in reversed(fields)])
    ax.xaxis.set_major_locator(mdates.AutoDateLocator())   # whole-minute ticks; raw floats would mislabel them
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    ax.set_xlim(xs[0], xs[-1] + dx[-1])
    ax.set_title('Sensor validity per packet   (green 11 good, amber 01/10 partial, red 00 failed)', loc='left', fontweight='bold')
    save(fig, '10_validity.png', 'Sensor validity per packet')

    # GPS track: plain, then on satellite imagery
    G = [r for r in T if r['fix'] and r['lat'] != 0]
    if len(G) < 2:
        print('  no GPS fixes, skipping track plots')
        return made
    lat = np.array([r['lat'] for r in G])
    lon = np.array([r['lon'] for r in G])
    modes = [r['mode_name'] for r in G]
    order = list(dict.fromkeys(modes))
    mcol = {m: MODE_COLORS[min(i, len(MODE_COLORS) - 1)] for i, m in enumerate(order)}

    def runs():
        s = 0
        for i in range(1, len(G) + 1):
            if i == len(G) or modes[i] != modes[s]:
                yield modes[s], slice(s, min(i + 1, len(G)))   # overlap one point so the line stays joined
                s = i

    def nearest(when):
        return min(range(len(G)), key=lambda i: abs((dt(G[i]['datetime']) - when).total_seconds()))

    def marks(X, Y, ax, ink, bg):
        pts = [(0, 'start', 8), (len(G) - 1, 'end', 8)] + [(nearest(e), 'restart', -14) for e in rst]
        for i, lbl, dy in pts:
            ax.scatter(X[i], Y[i], s=60, facecolor=bg, edgecolor=ink, lw=2, zorder=5)
            ax.annotate(f"{lbl} {G[i]['time'][:5]}", (X[i], Y[i]), xytext=(-8, dy), textcoords='offset points',
                        ha='right', fontsize=9, color=ink, zorder=6)
        for e in ack:
            i = nearest(e)
            ax.scatter(X[i], Y[i], s=60, marker='s', color=WARN, edgecolor=bg, lw=1.5, zorder=5)
            ax.annotate(f"last ack {e.strftime('%H:%M')}", (X[i], Y[i]), xytext=(8, -4), textcoords='offset points',
                        fontsize=9, color=ink, zorder=6)

    x = (lon - lon[0]) * 111320 * math.cos(math.radians(lat[0]))
    y = (lat - lat[0]) * 110540
    fig, ax = plt.subplots(figsize=(8, 8))
    for m, s in runs():
        ax.plot(x[s], y[s], color=mcol[m], lw=2.2)
    for m in order:
        ax.plot([], [], color=mcol[m], lw=3, label=m)
    marks(x, y, ax, 'black', 'white')
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend(loc='lower left', fontsize=8)
    ax.set_xlabel('metres east of start')
    ax.set_ylabel('metres north of start')
    ax.set_title('GPS track, colored by mode', loc='left', fontweight='bold')
    save(fig, '11_track.png', 'GPS track, colored by mode')

    if not want_map:
        return made
    try:
        import contextily as cx
        R = 6378137.0   # web mercator, what the tiles use
        mx = np.radians(lon) * R
        my = np.log(np.tan(np.pi / 4 + np.radians(lat) / 2)) * R
        fig, ax = plt.subplots(figsize=(9, 9))
        ax.grid(False)
        for m, s in runs():
            ax.plot(mx[s], my[s], color='white', lw=4.5, alpha=0.8, solid_capstyle='round')
        for m, s in runs():
            ax.plot(mx[s], my[s], color=mcol[m], lw=2.4, solid_capstyle='round')
        for m in order:
            ax.plot([], [], color=mcol[m], lw=3, label=m)
        marks(mx, my, ax, 'white', 'black')
        padx = (mx.max() - mx.min()) * 0.25 + 30
        pady = (my.max() - my.min()) * 0.25 + 30
        ax.set_xlim(mx.min() - padx, mx.max() + padx)
        ax.set_ylim(my.min() - pady, my.max() + pady)
        ax.set_aspect('equal')
        cx.add_basemap(ax, source=cx.providers.Esri.WorldImagery, zoom=18, attribution_size=6)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.legend(loc='lower left', fontsize=8, framealpha=0.85)
        ax.set_title(f'GPS track on satellite imagery   ({lat[0]:.5f}, {lon[0]:.5f})', loc='left', fontweight='bold')
        save(fig, '12_track_satellite.png', 'GPS track on satellite imagery')
    except Exception as e:   # no contextily, or offline
        plt.close('all')
        print(f'  skipped satellite map: {e}')
    return made


# ─── Summary and HTML ──────────────────────────────────────────────

def summary(T, C):
    sec = lambda s: sum(float(x) * m for x, m in zip(s.split(':'), (3600, 60, 1)))
    G = [r for r in T if r['fix'] and r['lat'] != 0]
    path = 0.0
    for a, b in zip(G, G[1:]):
        path += math.hypot((b['lon'] - a['lon']) * 111320 * math.cos(math.radians(a['lat'])), (b['lat'] - a['lat']) * 110540)
    lost = sum(r['lost'] for r in T)
    last_ack = T[-1]['last_cmd_seq']
    acked = [c for c in C if c['seq'] == last_ack]
    return [
        ('Session length', f"{(sec(T[-1]['time']) - sec(T[0]['time'])) / 60:.1f} min", f"{T[0]['time'][:8]} to {T[-1]['time'][:8]}"),
        ('Packets received', f'{len(T):,}', f'{lost} lost ({lost / (len(T) + lost) * 100:.1f}%)'),
        ('Boat restarts', str(sum(r['reboot'] for r in T)), ', '.join(r['time'][:8] for r in T if r['reboot']) or 'none'),
        ('Distance sailed', f'{path:.0f} m', 'sum of GPS fixes'),
        ('Commands confirmed OK', str(sum(c['result'] == 'OK' for c in C)), f'of {len(C)} sent, heartbeats not counted'),
        ('Last acknowledged command', f'#{last_ack}' if acked else 'none',
         f"sent {acked[-1]['time'][:8]}, {sum(c['seq'] > last_ack for c in C)} sent after it never acknowledged" if acked else ''),
    ]


def write_html(out, log_path, made, summ, C):
    e = html.escape
    tiles = ''.join(f'<div class="tile"><div class="k">{e(k)}</div><div class="v">{e(v)}</div><div class="n">{e(n)}</div></div>'
                    for k, v, n in summ)
    figs = ''.join(f'<figure id="{e(n[:-4])}"><a href="{e(n)}"><img src="{e(n)}" alt="{e(title)}" loading="lazy"></a>'
                   f'<figcaption>{e(title)}</figcaption></figure>' for n, title in made)
    rows = ''.join(f'<tr><td>{e(c["time"][:8])}</td><td>{c["seq"]}</td><td>{e(c["command"])}</td>'
                   f'<td class="o" data-o="{e(c["outcome"])}">{e(c["outcome"])}</td></tr>' for c in C)
    page = f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>ALAN log {e(os.path.basename(log_path))}</title>
<style>
  :root {{ --bg: #fbfbfa; --fg: #1f1f1e; --muted: #6b6a66; --line: #e3e2de; --panel: #ffffff;
          --ok: {OK}; --warn: {WARN}; --bad: {BAD}; }}
  @media (prefers-color-scheme: dark) {{ :root {{ --bg: #1a1a19; --fg: #ecebe8; --muted: #a3a29d; --line: #3a3a37; --panel: #232322; color-scheme: dark; }} }}
  body {{ margin: 0; background: var(--bg); color: var(--fg); font: 15px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif; }}
  main {{ max-width: 1100px; margin: 0 auto; padding: 28px max(16px, 3vw) 60px; display: grid; gap: 28px; }}
  h1 {{ margin: 0; font-size: 26px; }} h2 {{ margin: 0 0 8px; font-size: 18px; }}
  .sub {{ color: var(--muted); margin: 4px 0 0; font-size: 13px; }}
  .tiles {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 12px; }}
  .tile {{ background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 14px; }}
  .k, .n {{ color: var(--muted); font-size: 12px; }} .v {{ font-size: 24px; font-variant-numeric: tabular-nums; }}
  .figs {{ display: grid; gap: 22px; }}
  figure {{ margin: 0; background: #fff; border: 1px solid var(--line); border-radius: 8px; padding: 8px; }}
  figure img {{ width: 100%; height: auto; display: block; }}
  figcaption {{ color: #6b6a66; font-size: 12px; padding: 4px 4px 0; }}
  nav {{ display: flex; flex-wrap: wrap; gap: 6px 14px; font-size: 13px; }} nav a {{ color: var(--muted); }}
  .tw {{ overflow-x: auto; border: 1px solid var(--line); border-radius: 8px; }}
  table {{ border-collapse: collapse; width: 100%; font-size: 13px; font-variant-numeric: tabular-nums; }}
  th, td {{ text-align: left; padding: 5px 10px; border-bottom: 1px solid var(--line); white-space: nowrap; }}
  th {{ color: var(--muted); font-weight: 600; }}
  .o::before {{ content: ""; display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 6px; background: var(--muted); }}
  .o[data-o="OK"]::before {{ background: var(--ok); }} .o[data-o="Never acknowledged"]::before {{ background: var(--warn); }}
  .o[data-o="Refused (wrong mode)"]::before {{ background: var(--bad); }}
</style></head><body><main>
<header><h1>ALAN session {e(summ[0][2])}</h1>
<p class="sub">Decoded from the raw bytes in {e(os.path.basename(log_path))}. Times are ground station local time. Data: <a href="telemetry.csv">telemetry.csv</a>, <a href="commands.csv">commands.csv</a></p></header>
<section class="tiles">{tiles}</section>
<nav>{''.join(f'<a href="#{e(n[:-4])}">{e(title)}</a>' for n, title in made)}<a href="#commands">Command log</a></nav>
<section class="figs">{figs}</section>
<section id="commands"><h2>Command log</h2>
<p class="sub">The boat reports only its last command, once a second: "Superseded" ones were followed by another before a report and may still have run.</p>
<div class="tw"><table><thead><tr><th>Sent</th><th>Seq</th><th>Command</th><th>Outcome</th></tr></thead><tbody>{rows}</tbody></table></div></section>
</main></body></html>'''
    with open(os.path.join(out, 'index.html'), 'w') as fh:
        fh.write(page)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if not args:
        print(__doc__)
        sys.exit(1)
    log_path = args[0]
    out = args[1] if len(args) > 1 else os.path.splitext(log_path)[0] + '_plots'
    os.makedirs(out, exist_ok=True)

    T, C = decode(log_path)
    print(f'{len(T)} telemetry packets, {len(C)} commands')
    if not T:
        print('no telemetry packets in this log, nothing to plot')
        sys.exit(1)
    write_csv(os.path.join(out, 'telemetry.csv'), T)
    write_csv(os.path.join(out, 'commands.csv'), C)
    made = plots(T, C, out, '--no-map' not in sys.argv)
    write_html(out, log_path, made, summary(T, C), C)
    print(f'open {os.path.join(out, "index.html")}')


if __name__ == '__main__':
    main()
