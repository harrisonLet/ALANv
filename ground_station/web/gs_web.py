#!/usr/bin/env python3
"""
gs_web.py - browser ground station for ALANv (Python standard library only)

Reads the SAMD21 LoRa bridge over USB serial, parses LoRaTelemetry_t packets
from the boat and serves a live dashboard on http://localhost:8000. Command
keys from the page are written to the bridge as single bytes.

The packet layout is read from ../terminal/telemetry.h at startup, so the
C struct stays the single source of truth.

Run:   python3 gs_web.py [serial_port] [--http-port 8000] [--no-browser]
"""

import argparse
import collections
import glob
import json
import os
import queue
import re
import select
import struct
import termios
import threading
import time
import tty
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE   = os.path.dirname(os.path.abspath(__file__))
HEADER = os.path.join(HERE, '..', 'terminal', 'telemetry.h')
INDEX  = os.path.join(HERE, 'index.html')

ADDR_GROUND = 0xAA
ADDR_BOAT   = 0xBB
HDR_LEN     = 4
HISTORY_LEN = 600        # samples kept for charts and track (~10 min at 1 Hz)

COMMANDS = {
    'q': 'rudder +20', 'e': 'rudder -20', 'a': 'rudder +10',
    'd': 'rudder -10', 'z': 'rudder +5',  'c': 'rudder -5',
    'w': 'sail +20',   'r': 'sail -20',   's': 'sail +10',
    'f': 'sail -10',   'x': 'sail +5',    'v': 'sail -5',
}

# Validity_e codes from Common/Inc/sharedSensor.h on sensor_polling
VALIDITY = {
    0x00: 'UNKNOWN', 0x01: 'VALID', 0x02: 'CALIBRATING', 0x03: 'WARMING',
    0x04: 'WINDOW_FILLING', 0x05: 'DISABLED', 0x06: 'PARTIAL_DATA',
    0x10: 'UNTRUSTED', 0x11: 'OUT_OF_RANGE', 0x12: 'DEGRADED', 0x13: 'IMPLAUSIBLE',
    0x14: 'STATIC', 0x15: 'OVER_SATURATION', 0x16: 'UNDER_SATURATION',
    0x20: 'ERROR', 0x21: 'UNREACHABLE', 0x22: 'PARSE_FAIL', 0x23: 'ERROR_UNKNOWN',
    0x80: 'INVALID_BATTERY', 0x90: 'INVALID_ENCODER', 0xA0: 'INVALID_GPS',
    0xA1: 'GPS_STALE', 0xA2: 'GPS_NO_FIX', 0xA3: 'GPS_CHECKSUM',
    0xB0: 'INVALID_IMU', 0xC0: 'INVALID_WIND',
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


FMT, FIELDS = load_layout(HEADER)
TELEM_LEN = struct.calcsize(FMT)
PKT_LEN = HDR_LEN + TELEM_LEN


def unpack(payload):
    values = struct.unpack(FMT, payload)
    out, i = {}, 0
    for name, n in FIELDS:
        out[name] = list(values[i:i + n]) if n > 1 else values[i]
        i += n
    return out

# ─── Shared state ────────────────────────────────────────────────────────────


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.port = None
        self.fd = None
        self.error = None
        self.telem = None
        self.raw = ''
        self.seq = None
        self.pkts = self.lost = self.junk = 0
        self.rx_time = None
        self.history = collections.deque(maxlen=HISTORY_LEN)
        self.cmds_sent = 0
        self.cmd_log = collections.deque(maxlen=50)
        self.clients = []

    def link(self):
        return {'port': self.port, 'connected': self.fd is not None, 'error': self.error,
                'pkts': self.pkts, 'lost': self.lost, 'junk': self.junk, 'seq': self.seq,
                'rx_time': self.rx_time, 'cmds_sent': self.cmds_sent}

    def snapshot(self):
        return {'link': self.link(), 'telem': self.telem, 'raw': self.raw,
                'history': list(self.history), 'cmd_log': list(self.cmd_log),
                'validity': VALIDITY, 'commands': COMMANDS}

    def broadcast(self, event, data):
        msg = f'event: {event}\ndata: {json.dumps(data)}\n\n'.encode()
        for q in list(self.clients):
            try:
                q.put_nowait(msg)
            except queue.Full:
                self.clients.remove(q)


S = State()

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
        sample = {'t': now, 'heading': telem['heading'], 'desired_heading': telem['desired_heading'],
                  'ang_wind': telem['ang_wind'], 'roll': telem['roll'], 'pitch': telem['pitch'],
                  'battery_voltage': telem['battery_voltage'], 'sail_ang_forward': telem['sail_ang_forward'],
                  'rudder_servo_angle': telem['rudder_servo_angle'], 'sail_servo_angle': telem['sail_servo_angle'],
                  'drift_distance': telem['drift_distance'],
                  'latitude': telem['latitude'], 'longitude': telem['longitude'],
                  'gps_ok': 0x01 <= telem['gps_valid'] < 0x10}
        S.history.append(sample)
        link = S.link()
    S.broadcast('telem', {'link': link, 'telem': telem, 'raw': S.raw, 'sample': sample})


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
            if buf[i] == ADDR_GROUND and buf[i + 1] == ADDR_BOAT and buf[i + 3] == TELEM_LEN:
                handle_packet(bytes(buf[i:i + PKT_LEN]))
                i += PKT_LEN
            else:
                S.junk += 1
                i += 1
        del buf[:i]


def send_cmd(key):
    if key not in COMMANDS:
        return 'unknown command'
    with S.lock:
        fd = S.fd
    if fd is None:
        return 'not connected'
    try:
        os.write(fd, key.encode())
    except OSError as e:
        return str(e)
    with S.lock:
        S.cmds_sent += 1
        entry = {'t': time.time(), 'key': key, 'desc': COMMANDS[key], 'n': S.cmds_sent}
        S.cmd_log.appendleft(entry)
        link = S.link()
    S.broadcast('cmd', {'entry': entry, 'link': link})
    return None


def list_ports():
    pats = ['/dev/ttyACM*', '/dev/ttyUSB*', '/dev/tty.usbmodem*', '/dev/cu.usbmodem*']
    return sorted(p for pat in pats for p in glob.glob(pat))

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

    def read_json(self):
        n = int(self.headers.get('Content-Length', 0))
        try:
            return json.loads(self.rfile.read(n) or b'{}')
        except ValueError:
            return {}

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            body = open(INDEX, 'rb').read()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/ports':
            self.send_json({'ports': list_ports(), 'current': S.port})
        elif self.path == '/events':
            self.events()
        else:
            self.send_error(404)

    def do_POST(self):
        req = self.read_json()
        if self.path == '/cmd':
            err = send_cmd(str(req.get('key', '')))
            self.send_json({'ok': err is None, 'error': err}, 200 if err is None else 400)
        elif self.path == '/connect':
            ok = connect(str(req.get('port', '')))
            self.send_json({'ok': ok, 'error': S.error}, 200 if ok else 400)
        elif self.path == '/disconnect':
            disconnect()
            self.send_json({'ok': True})
        else:
            self.send_error(404)

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
    args = ap.parse_args()

    port = args.port or next(iter(list_ports()), None)
    if port:
        print(f'serial: {port}', 'connected' if connect(port) else f'FAILED ({S.error})')
    else:
        print('serial: no port found, pick one in the page')

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
