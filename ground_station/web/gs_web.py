#!/usr/bin/env python3
"""
gs_web.py - browser ground station for ALANv (Python standard library only)

Reads the SAMD21 LoRa bridge over USB serial, parses LoRaTelemetry_t packets
from the boat and serves a live dashboard on http://localhost:8000: telemetry,
a satellite map with the boat and its track, a route planner (waypoints with a
corridor margin, sailed by LTP in AUTO), mode and heading commands, and the
manual rudder/sail keys.

Commands are real command packets (boatlink.py). They go out one at a time,
right after a telemetry packet arrives (that is when the boat's radio is
listening), and each one is resent with the same seq until the boat's
telemetry reports it in last_cmd_seq, or it gives up after a few tries.

The telemetry layout is read from ../terminal/telemetry.h and the command
numbers from ../terminal/commands.h at startup, so the C headers stay the
single source of truth.

Map tiles (Esri World Imagery) are fetched on demand and cached in ./tiles.
There is usually no internet at the lake, so fill the cache beforehand:
    python3 gs_web.py --prefetch 44.5646,-123.2620,800      (lat,lon,radius in m)

Run:   python3 gs_web.py [serial_port] [--http-port 8000] [--no-browser]
Test without hardware:  python3 sim_cm4.py   (prints a port), then  python3 gs_web.py <that port>
"""

import argparse
import collections
import glob
import itertools
import json
import math
import os
import queue
import re
import select
import struct
import termios
import threading
import time
import tty
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import boatlink as BL

HERE   = os.path.dirname(os.path.abspath(__file__))
HEADER = os.path.join(HERE, '..', 'terminal', 'telemetry.h')
INDEX  = os.path.join(HERE, 'index.html')
STATIC = {'/map.js': ('map.js', 'text/javascript; charset=utf-8')}
TILE_DIR = os.path.join(HERE, 'tiles')
TILE_URL = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'
TILE_MAX_ZOOM = 19

ADDR_GROUND = BL.ADDR_GROUND
ADDR_BOAT   = BL.ADDR_BOAT
HDR_LEN     = 5        # dest, src, seq, len, flags
HISTORY_LEN = 600      # samples kept for charts and track (~10 min at 1 Hz)

# Commands
ACK_TIMEOUT_S   = 2.5  # resend when the boat has not reported the command by then (it reports once a second)
MAX_TRIES       = 5    # sends of one command before giving up
SEND_WINDOW_S   = 0.4  # send within this long after a telemetry packet: the boat listens between its transmissions
BLIND_AFTER_S   = 2.0  # no telemetry for this long: send anyway, once a second
HB_PERIOD_S     = 1.0  # heartbeat when idle (the boat's link timeout counts them)
ROUTE_UPLOADS   = 3    # whole-route attempts before giving up

# Manual keys: step the rudder or the sail tail servo from the angle last asked for
COMMANDS = {
    'q': 'rudder +20', 'e': 'rudder -20', 'a': 'rudder +10',
    'd': 'rudder -10', 'z': 'rudder +5',  'c': 'rudder -5',
    'w': 'sail +20',   'r': 'sail -20',   's': 'sail +10',
    'f': 'sail -10',   'x': 'sail +5',    'v': 'sail -5',
}

# LoRaTelemetryValidity_e from Common/Inc/loraTelemetry.h on master:
# bit 0 = sensor metadata healthy, bit 1 = fresh telemetry reading
VALIDITY = {
    0x0: '(00) FAILED', 0x1: '(01) NO DATA', 0x2: '(10) SENSOR FAULT', 0x3: '(11) ALL GOOD',
}

# ─── Packet layout from the C header ─────────────────────────────────────────

C_TYPES = {'uint8_t': 'B', 'int8_t': 'b', 'uint16_t': 'H', 'int16_t': 'h',
           'uint32_t': 'I', 'int32_t': 'i', 'float': 'f', 'double': 'd'}


def load_layout(path):
    """Return (struct format, [(field, count)]) for LoRaTelemetry_t."""
    src = open(path).read()
    start = src.index('{', src.index('typedef struct'))
    body = src[start + 1:src.index('} LoRaTelemetry_t')]
    body = re.sub(r'/\*.*?\*/', '', body, flags=re.S)
    fmt, fields = '<', []
    for ctype, name, count in re.findall(r'(\w+)\s+(\w+)\s*(?:\[(\d+)\])?\s*;', body):
        n = int(count) if count else 1
        fmt += (str(n) if n > 1 else '') + C_TYPES[ctype]
        fields.append((name, n))
    return fmt, fields


def header_number(path, name, default):
    m = re.search(r'#define\s+' + name + r'\s+([0-9.]+)', open(path).read())
    return float(m.group(1)) if m else default


FMT, FIELDS = load_layout(HEADER)
TELEM_LEN = struct.calcsize(FMT)
PKT_LEN = HDR_LEN + TELEM_LEN
GPS_DEG_SCALE = header_number(HEADER, 'LORA_GPS_DEG_SCALE', 1e7)
GPS_ALT_SCALE = header_number(HEADER, 'LORA_GPS_ALT_SCALE', 10.0)
RX_SNR_SCALE = header_number(HEADER, 'LORA_RX_SNR_SCALE', 4.0)
NO_HEADING = 0xFFFF
# NavState_e (sharedNavigation.h), from the low bits of nav_info
NAV_STATES = {0: 'IDLE', 1: 'DIRECT', 2: 'UPWIND PORT', 3: 'UPWIND STBD', 4: 'TACKING', 5: 'ARRIVED', 6: 'FAULT',
              0x0F: 'NO LTP'}


def unpack(payload):
    values = struct.unpack(FMT, payload)
    out, i = {}, 0
    for name, n in FIELDS:
        out[name] = list(values[i:i + n]) if n > 1 else values[i]
        i += n
    # Integer encodings to plain units, so the page does not need to know them
    if isinstance(out.get('latitude'), int):
        out['latitude'] /= GPS_DEG_SCALE
        out['longitude'] /= GPS_DEG_SCALE
    if isinstance(out.get('altitude'), int):
        out['altitude'] /= GPS_ALT_SCALE
    if 'sats_fix' in out:
        out['satellites'] = out['sats_fix'] & 0x1F
        out['fix_valid'] = out['sats_fix'] >> 5
    if 'rx_last_snr' in out:
        out['rx_last_snr'] /= RX_SNR_SCALE
    if 'nav_info' in out:
        info = out['nav_info']
        out['nav_state'] = info & 0x0F
        out['nav_state_name'] = NAV_STATES.get(info & 0x0F, f'state {info & 0x0F}')
        out['nav_corridor'] = bool(info & 0x80)
        out['nav_tack_slow'] = bool(info & 0x40)
    out['mode_name'] = BL.MODE_NAMES.get(out.get('mode'), f"mode {out.get('mode')}")
    out['last_cmd_result_name'] = BL.RESULTS.get(out.get('last_cmd_result'), str(out.get('last_cmd_result')))
    return out


def gps_ok(t):
    return 0x01 <= t.get('gps_valid', 0) < 0x10 and (t['latitude'] or t['longitude'])

# ─── Shared state ────────────────────────────────────────────────────────────


class Cmd:
    _ids = itertools.count(1)

    def __init__(self, ctype, payload, label, group=None, route=None):
        self.id = next(Cmd._ids)
        self.type, self.payload, self.label = ctype, payload, label
        self.group = group          # a newer command of the same group replaces a queued one (servo jogs)
        self.route = route          # route upload this command belongs to
        self.seq = None
        self.tries = 0
        self.sent_t = None
        self.status = 'queued'      # queued, sent, ok, failed, gave up, cancelled
        self.result = None
        self.t = time.time()

    def view(self):
        return {'id': self.id, 'label': self.label, 'seq': self.seq, 'tries': self.tries,
                'status': self.status, 'result': self.result, 't': self.t}


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.wake = threading.Event()   # telemetry arrived, or something was queued
        self.port = None
        self.fd = None
        self.error = None
        self.telem = None
        self.raw = ''
        self.seq = None
        self.pkts = self.lost = self.junk = 0
        self.rx_time = None
        self.history = collections.deque(maxlen=HISTORY_LEN)
        self.clients = []
        # commands
        self.tx_seq = None              # picked after the first telemetry, past the boat's last_cmd_seq
        self.queue = collections.deque()
        self.inflight = None
        self.log = collections.deque(maxlen=60)
        self.cmds_sent = self.hbs_sent = 0
        self.last_send = 0.0
        self.heartbeat = True
        self.jog = {'rudder': 0, 'sail': 0}
        # route: what the boat holds (active) and the upload in progress
        self.route = {'active': None, 'upload': None}

    def link(self):
        return {'port': self.port, 'connected': self.fd is not None, 'error': self.error,
                'pkts': self.pkts, 'lost': self.lost, 'junk': self.junk, 'seq': self.seq,
                'rx_time': self.rx_time, 'cmds_sent': self.cmds_sent, 'hbs_sent': self.hbs_sent,
                'heartbeat': self.heartbeat}

    def cmd_view(self):
        route = {k: ({kk: vv for kk, vv in v.items() if not kk.startswith('_')} if v else v)
                 for k, v in self.route.items()}
        return {'log': [c.view() for c in self.log], 'route': route, 'jog': self.jog,
                'queued': len(self.queue) + (1 if self.inflight else 0)}

    def snapshot(self):
        return {'link': self.link(), 'telem': self.telem, 'raw': self.raw,
                'history': list(self.history), 'cmds': self.cmd_view(),
                'validity': VALIDITY, 'commands': COMMANDS,
                'modes': BL.MODES, 'heading_modes': sorted(BL.HEADING_MODES),
                'limits': {'servo': BL.SERVO_LIMIT, 'waypoints': BL.ROUTE_MAX_WAYPOINTS}}

    def broadcast(self, event, data):
        msg = f'event: {event}\ndata: {json.dumps(data)}\n\n'.encode()
        for q in list(self.clients):
            try:
                q.put_nowait(msg)
            except queue.Full:
                self.clients.remove(q)


S = State()


def broadcast_cmds():
    with S.lock:
        data = {'cmds': S.cmd_view(), 'link': S.link()}
    S.broadcast('cmd', data)

# ─── Serial ──────────────────────────────────────────────────────────────────


def open_serial(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def connect(path):
    disconnect()
    try:
        fd = open_serial(path)
    except OSError as e:
        with S.lock:
            S.port, S.error = path, str(e)
        S.broadcast('link', S.link())
        return False
    with S.lock:
        S.port, S.fd, S.error = path, fd, None
        S.pkts = S.lost = S.junk = 0
        S.seq = None
    threading.Thread(target=reader, args=(fd,), daemon=True).start()
    S.broadcast('link', S.link())
    return True


def disconnect():
    with S.lock:
        fd, S.fd = S.fd, None
    if fd is not None:
        try:
            os.close(fd)
        except OSError:
            pass
        S.broadcast('link', S.link())


def handle_packet(pkt):
    telem = unpack(pkt[HDR_LEN:])
    seq = pkt[2]
    now = time.time()
    with S.lock:
        if S.seq is not None:
            S.lost += (seq - S.seq - 1) & 0xFF
        S.seq = seq
        S.pkts += 1
        S.rx_time = now
        S.telem = telem
        S.raw = pkt.hex(' ')
        if S.tx_seq is None:
            # Past whatever the boat heard or executed last, so an old seq can't read as this session's ack
            # and a new command can't look like a resend of an old one
            heard = telem.get('rx_last_seq') if telem.get('rx_age_s', 255) != 255 else None
            S.tx_seq = ((heard if heard is not None else telem['last_cmd_seq']) + 1) & 0xFF
        sample = {'t': now, 'heading': telem['heading'], 'desired_heading': telem['desired_heading'],
                  'ang_wind': telem['ang_wind'], 'roll': telem['roll'], 'pitch': telem['pitch'],
                  'battery_voltage': telem['battery_voltage'], 'sail_ang_forward': telem['sail_ang_forward'],
                  'rudder_servo_angle': telem['rudder_servo_angle'], 'sail_servo_angle': telem['sail_servo_angle'],
                  'drift_distance': telem['drift_distance'], 'mode': telem['mode'],
                  'latitude': telem['latitude'], 'longitude': telem['longitude'],
                  'gps_ok': bool(gps_ok(telem))}
        S.history.append(sample)
        link = S.link()
    S.broadcast('telem', {'link': link, 'telem': telem, 'raw': S.raw, 'sample': sample})
    S.wake.set()


def reader(fd):
    buf = bytearray()
    while S.fd == fd:
        try:
            r, _, _ = select.select([fd], [], [], 0.5)
            if not r:
                continue
            data = os.read(fd, 1024)
        except OSError as e:
            with S.lock:
                S.error = str(e)
            disconnect()
            return
        if not data:
            continue
        buf += data
        i = 0
        while len(buf) - i >= PKT_LEN:
            if buf[i] == ADDR_GROUND and buf[i + 1] == ADDR_BOAT and buf[i + 3] == 1 + TELEM_LEN:
                handle_packet(bytes(buf[i:i + PKT_LEN]))
                i += PKT_LEN
            else:
                S.junk += 1
                i += 1
        del buf[:i]

# ─── Commands ────────────────────────────────────────────────────────────────


def enqueue(cmds):
    """Queue commands (caller does not hold the lock). A newer jog replaces a queued one of its group."""
    with S.lock:
        for c in cmds:
            if c.group:
                for old in [q for q in S.queue if q.group == c.group]:
                    S.queue.remove(old)
                    old.status = 'cancelled'
            S.queue.append(c)
            S.log.appendleft(c)
    S.wake.set()
    broadcast_cmds()


def write_packet(pkt):
    """Write one packet to the bridge (lock held). The bridge sends each write as one LoRa packet."""
    if S.fd is None:
        return 'not connected'
    try:
        os.write(S.fd, pkt)
    except OSError as e:
        return str(e)
    S.last_send = time.time()
    return None


def send(c):
    """(Re)send c with its seq (lock held)."""
    if c.seq is None:
        c.seq = S.tx_seq
        S.tx_seq = (S.tx_seq + 1) & 0xFF
    err = write_packet(BL.build_packet(c.seq, c.type, c.payload))
    c.tries += 1
    c.sent_t = time.time()
    c.status = 'sent' if err is None else 'sent (write failed)'
    S.cmds_sent += 1


def finish(c, status, result=None):
    """c is done (lock held): mark it and keep any route upload it belongs to moving."""
    c.status, c.result = status, result
    if S.inflight is c:
        S.inflight = None
    up = c.route
    if up is None or up is not S.route['upload']:
        return
    if c.type == BL.COMMAND_NVS_LOAD_STOP and status == 'ok':
        up['status'] = 'loaded'
        S.route['active'] = {'waypoints': up['waypoints'], 'arrive_m': up['arrive_m'],
                             'warning_m': up['warning_m'], 'crc': up['crc'], 't': time.time(),
                             'start': boat_position()}
        if not up['auto']:
            S.route['upload'] = None
        return
    if c.type == BL.MODE_COMMAND['AUTO'] and status == 'ok':
        if S.route['active']:
            S.route['active']['start'] = boat_position()   # LTP starts leg 1 where the boat is when AUTO takes the route
        S.route['upload'] = None
        return
    if status == 'ok':
        up['done'] += 1
        return
    # Something in the upload failed: drop the rest of it
    for q in [q for q in S.queue if q.route is up]:
        S.queue.remove(q)
        q.status = 'cancelled'
    # NONE: LTP was reading the route at that moment. The boat keeps the checked upload, so send the commit again
    if c.type == BL.COMMAND_NVS_LOAD_STOP and result == 'NONE' and up['attempt'] < ROUTE_UPLOADS + 2:
        up['attempt'] += 1
        up['status'] = 'boat busy, committing again'
        again = [Cmd(c.type, c.payload, c.label, route=up)]
        if up['auto']:
            again.append(Cmd(BL.MODE_COMMAND['AUTO'], b'', 'mode AUTO', route=up))
        for q in again:
            S.queue.append(q)
            S.log.appendleft(q)
        return
    # Any other failed commit clears the half-loaded route on the boat, so start over
    retry = (c.type == BL.COMMAND_NVS_LOAD_STOP and result == 'INCOMPLETE') or status == 'gave up'
    if retry and up['attempt'] < ROUTE_UPLOADS:
        up['attempt'] += 1
        up['done'] = 0
        up['status'] = f"retrying (attempt {up['attempt']}): {c.label} {result or status}"
        queue_route_cmds(up)
    else:
        up['status'] = f"failed: {c.label} {result or status}"


def boat_position():
    t = S.telem
    return {'lat': t['latitude'], 'lon': t['longitude']} if t and gps_ok(t) else None


def queue_route_cmds(up):
    """Append an upload's commands to the queue (lock held)."""
    cmds = [Cmd(ct, p, l, route=up) for ct, p, l in up['_commands']]
    if up['auto']:
        cmds.append(Cmd(BL.MODE_COMMAND['AUTO'], b'', 'mode AUTO', route=up))
    for c in cmds:
        S.queue.append(c)
        S.log.appendleft(c)


def sender():
    """One packet per telemetry cycle: the next command, a resend, or a heartbeat."""
    while True:
        S.wake.wait(0.25)
        S.wake.clear()
        changed = False
        with S.lock:
            now = time.time()
            t, c = S.telem, S.inflight
            # Acknowledged: the boat reports the seq it executed last and what came of it
            if c and t and S.rx_time and S.rx_time > c.sent_t and (t['last_cmd_seq'] & 0xFF) == c.seq:
                r = BL.RESULTS.get(t['last_cmd_result'], str(t['last_cmd_result']))
                finish(c, 'ok' if r == 'OK' else 'failed', r)
                changed = True
            if S.fd is not None:
                if S.tx_seq is None:
                    S.tx_seq = 0     # nothing heard from the boat yet: start anywhere
                fresh = S.rx_time and S.rx_time > S.last_send and now - S.rx_time < SEND_WINDOW_S
                blind = (not S.rx_time or now - S.rx_time > BLIND_AFTER_S) and now - S.last_send >= 1.0
                if fresh or blind:
                    c = S.inflight
                    if c:
                        if now - c.sent_t >= ACK_TIMEOUT_S:
                            if c.tries >= MAX_TRIES:
                                finish(c, 'gave up')
                            else:
                                send(c)
                            changed = True
                    elif S.queue:
                        S.inflight = S.queue.popleft()
                        send(S.inflight)
                        changed = True
                    elif S.heartbeat and now - S.last_send >= HB_PERIOD_S - SEND_WINDOW_S:
                        seq = S.tx_seq
                        S.tx_seq = (S.tx_seq + 1) & 0xFF
                        if write_packet(BL.build_packet(seq, BL.COMMAND_NONE)) is None:
                            S.hbs_sent += 1
        if changed:
            broadcast_cmds()


def cmd_key(key):
    if key not in COMMANDS:
        return 'unknown command'
    which, step = COMMANDS[key].split()
    with S.lock:
        S.jog[which] = max(-BL.SERVO_LIMIT, min(BL.SERVO_LIMIT, S.jog[which] + int(step)))
        deg = S.jog[which]
    ctype = BL.COMMAND_RUDDER_SERVO if which == 'rudder' else BL.COMMAND_SAIL_SERVO
    enqueue([Cmd(ctype, BL.angle_payload(deg), f'{which} {deg:+d}°', group=which)])
    return None


def cmd_servo(which, deg):
    if which not in ('rudder', 'sail'):
        return 'unknown servo'
    deg = int(deg)
    if abs(deg) > BL.SERVO_LIMIT:
        return f'servo angle must be within ±{BL.SERVO_LIMIT}°'
    with S.lock:
        S.jog[which] = deg
    ctype = BL.COMMAND_RUDDER_SERVO if which == 'rudder' else BL.COMMAND_SAIL_SERVO
    enqueue([Cmd(ctype, BL.angle_payload(deg), f'{which} {deg:+d}°', group=which)])
    return None


def cmd_mode(name):
    name = str(name).upper()
    if name not in BL.MODES:
        return 'unknown mode'
    enqueue([Cmd(BL.MODE_COMMAND[name], b'', f'mode {name}', group='mode')])
    return None


def cmd_heading(deg):
    deg = int(round(float(deg))) % 360
    enqueue([Cmd(BL.COMMAND_RUDDER_PID, BL.angle_payload(deg), f'heading {deg}°', group='heading')])
    return None


def cmd_route(waypoints, arrive_m, warning_m, auto):
    try:
        cmds, crc = BL.route_commands(waypoints, int(arrive_m), int(warning_m))
    except (ValueError, KeyError, TypeError) as e:
        return None, str(e)
    up = {'waypoints': [{'lat': float(w['lat']), 'lon': float(w['lon']), 'margin': int(round(w['margin']))}
                        for w in waypoints],
          'arrive_m': int(arrive_m), 'warning_m': int(warning_m), 'auto': bool(auto), 'crc': crc,
          '_commands': cmds, 'total': len(cmds) + (1 if auto else 0), 'done': 0, 'attempt': 1,
          'status': 'uploading', 't': time.time()}
    with S.lock:
        old = S.route['upload']
        if old:
            for q in [q for q in S.queue if q.route is old]:
                S.queue.remove(q)
                q.status = 'cancelled'
        S.route['upload'] = up
        queue_route_cmds(up)
    S.wake.set()
    broadcast_cmds()
    return {'crc': crc, 'commands': up['total']}, None


def cancel_queue():
    with S.lock:
        for q in S.queue:
            q.status = 'cancelled'
        S.queue.clear()
        if S.route['upload']:
            S.route['upload']['status'] = 'cancelled'
            S.route['upload'] = None
    broadcast_cmds()


def list_ports():
    pats = ['/dev/ttyACM*', '/dev/ttyUSB*', '/dev/tty.usbmodem*', '/dev/cu.usbmodem*']
    return sorted(p for pat in pats for p in glob.glob(pat))

# ─── Map tiles ───────────────────────────────────────────────────────────────

_tile_fail = {}          # url -> time of the last failed fetch, so an offline laptop doesn't retry every frame


def tile_path(z, x, y):
    return os.path.join(TILE_DIR, str(z), str(x), f'{y}.jpg')


def get_tile(z, x, y, fetch=True):
    """Tile bytes from the cache, fetched once from the imagery server if missing. None when unavailable."""
    if not (0 <= z <= TILE_MAX_ZOOM and 0 <= x < 2 ** z and 0 <= y < 2 ** z):
        return None
    path = tile_path(z, x, y)
    if os.path.exists(path):
        return open(path, 'rb').read()
    url = TILE_URL.format(z=z, x=x, y=y)
    if not fetch or time.time() - _tile_fail.get(url, 0) < 60:
        return None
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'ALANv-ground-station/1.0'})
        data = urllib.request.urlopen(req, timeout=6).read()
    except OSError:
        _tile_fail[url] = time.time()
        return None
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(data)
    return data


def tile_xy(lat, lon, z):
    n = 2 ** z
    x = (lon + 180) / 360 * n
    y = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n
    return x, y


def prefetch(spec, zmin=13, zmax=TILE_MAX_ZOOM):
    """Download every tile within radius_m of lat,lon so the map works offline."""
    parts = [float(v) for v in spec.split(',')]
    lat, lon = parts[0], parts[1]
    radius = parts[2] if len(parts) > 2 else 800.0
    dlat = radius / 111320
    dlon = radius / (111320 * max(0.01, math.cos(math.radians(lat))))
    todo = []
    for z in range(zmin, zmax + 1):
        x0, y0 = tile_xy(lat + dlat, lon - dlon, z)
        x1, y1 = tile_xy(lat - dlat, lon + dlon, z)
        todo += [(z, x, y) for x in range(int(x0), int(x1) + 1) for y in range(int(y0), int(y1) + 1)]
    have = sum(os.path.exists(tile_path(*t)) for t in todo)
    print(f'{len(todo)} tiles for {radius:.0f} m around {lat:.5f},{lon:.5f} (zoom {zmin}-{zmax}), {have} cached')
    got = failed = 0
    for i, t in enumerate(todo, 1):
        if os.path.exists(tile_path(*t)):
            continue
        if get_tile(*t) is None:
            failed += 1
        else:
            got += 1
        if i % 50 == 0:
            print(f'  {i}/{len(todo)}')
    print(f'done: {got} downloaded, {failed} failed, cache in {os.path.relpath(TILE_DIR)}')

# ─── HTTP ────────────────────────────────────────────────────────────────────


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send_json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_bytes(self, body, ctype, cache=None):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        if cache:
            self.send_header('Cache-Control', cache)
        self.end_headers()
        self.wfile.write(body)

    def read_json(self):
        n = int(self.headers.get('Content-Length', 0))
        try:
            return json.loads(self.rfile.read(n) or b'{}')
        except ValueError:
            return {}

    def do_GET(self):
        path = self.path.split('?')[0]
        if path in ('/', '/index.html'):
            self.send_bytes(open(INDEX, 'rb').read(), 'text/html; charset=utf-8', 'no-cache')
        elif path in STATIC:
            name, ctype = STATIC[path]
            self.send_bytes(open(os.path.join(HERE, name), 'rb').read(), ctype, 'no-cache')
        elif path.startswith('/tiles/'):
            m = re.fullmatch(r'/tiles/(\d+)/(\d+)/(\d+)', path)
            data = m and get_tile(*map(int, m.groups()))
            if data:
                self.send_bytes(data, 'image/jpeg', 'max-age=86400')
            else:
                self.send_error(404)
        elif path == '/ports':
            self.send_json({'ports': list_ports(), 'current': S.port})
        elif path == '/events':
            self.events()
        else:
            self.send_error(404)

    def do_POST(self):
        req = self.read_json()
        err, extra = None, {}
        try:
            if self.path == '/cmd':
                err = cmd_key(str(req.get('key', '')))
            elif self.path == '/servo':
                err = cmd_servo(str(req.get('which', '')), req.get('deg', 0))
            elif self.path == '/mode':
                err = cmd_mode(req.get('mode', ''))
            elif self.path == '/heading':
                err = cmd_heading(req.get('deg', 0))
            elif self.path == '/route':
                res, err = cmd_route(req.get('waypoints', []), req.get('arrive_m', 8), req.get('warning_m', 10),
                                     req.get('auto', False))
                extra = res or {}
            elif self.path == '/cancel':
                cancel_queue()
            elif self.path == '/heartbeat':
                with S.lock:
                    S.heartbeat = bool(req.get('on', True))
                broadcast_cmds()
            elif self.path == '/connect':
                ok = connect(str(req.get('port', '')))
                err = None if ok else (S.error or 'could not open port')
            elif self.path == '/disconnect':
                disconnect()
            else:
                return self.send_error(404)
        except (ValueError, TypeError) as e:
            err = str(e)
        self.send_json({'ok': err is None, 'error': err, **extra}, 200 if err is None else 400)

    def events(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        q = queue.Queue(maxsize=200)
        with S.lock:
            snap = S.snapshot()
        S.clients.append(q)
        try:
            self.wfile.write(f'event: snapshot\ndata: {json.dumps(snap)}\n\n'.encode())
            self.wfile.flush()
            while True:
                try:
                    msg = q.get(timeout=15)
                except queue.Empty:
                    msg = b': keepalive\n\n'
                self.wfile.write(msg)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            if q in S.clients:
                S.clients.remove(q)


def main():
    ap = argparse.ArgumentParser(description='ALANv browser ground station')
    ap.add_argument('port', nargs='?', help='serial port (default: first /dev/ttyACM*)')
    ap.add_argument('--http-port', type=int, default=8000)
    ap.add_argument('--no-browser', action='store_true')
    ap.add_argument('--prefetch', metavar='LAT,LON[,RADIUS_M]',
                    help='download map tiles around a place for offline use, then exit')
    args = ap.parse_args()

    if args.prefetch:
        prefetch(args.prefetch)
        return

    port = args.port or next(iter(list_ports()), None)
    if port:
        print(f'serial: {port}', 'connected' if connect(port) else f'FAILED ({S.error})')
    else:
        print('serial: no port found, pick one in the page')
    threading.Thread(target=sender, daemon=True).start()

    httpd = ThreadingHTTPServer(('127.0.0.1', args.http_port), Handler)
    httpd.daemon_threads = True
    url = f'http://localhost:{args.http_port}'
    print(f'ground station on {url}  ({TELEM_LEN}-byte telemetry from {os.path.relpath(HEADER)})')
    if not args.no_browser:
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        disconnect()


if __name__ == '__main__':
    main()
