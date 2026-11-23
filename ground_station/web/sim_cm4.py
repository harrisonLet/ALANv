#!/usr/bin/env python3
"""
sim_cm4.py - fake boat + SAMD21 bridge for testing the ground station without hardware

Opens a pseudo-terminal that behaves like the bridge's USB serial port. It reads command packets the way
the boat's CM4 does, runs them through the same rules as the CM7 Commander (mode checks, route upload with
CRC, last_cmd_seq / last_cmd_result reporting), sails a simple version of LTP, and sends a LoRaTelemetry_t
packet once a second.

Not a physics model: the boat turns at a fixed rate and its speed depends only on the angle to the wind.
It is there to exercise the ground station: route upload, acks and resends, modes, the map.

Run:   python3 sim_cm4.py [--at LAT,LON] [--wind 270] [--drop 0.2] [--boot-mode STANDBY]
       python3 gs_web.py <port it prints>
"""
import argparse
import math
import os
import pty
import random
import select
import struct
import sys
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import boatlink as BL   # noqa: E402
import gs_web           # noqa: E402  reuse FMT/FIELDS parsed from telemetry.h

M = BL.MODES
RUDDER_MANUAL = {M['FULL_MAN'], M['AUTO_SAIL_MAN_RUDDER'], M['SEMI_SAIL_MAN_RUDDER']}
SAIL_MANUAL = {M['FULL_MAN'], M['AUTO_RUDDER_MAN_SAIL'], M['SEMI_RUDDER_MAN_SAIL']}
RESULT = {v: k for k, v in BL.RESULTS.items()}
HOLD_S = 10.0          # COMMANDER_RESULT_HOLD_MS: heartbeats leave a real command's result alone this long
NO_GO = 45.0           # NAV_NO_GO_DEG
TURN_RATE = 15.0       # deg/s
WP_SIZE = struct.calcsize('<HHii')


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Boat:
    def __init__(self, lat, lon, wind_from, speed, boot_mode):
        self.lat, self.lon = lat, lon
        self.heading = random.uniform(0, 360)
        self.wind_from = wind_from
        self.top_speed = speed
        self.speed = 0.0
        self.mode = M[boot_mode]
        self.rudder = self.sail = 0
        self.target = None              # heading PID target
        self.last_seq, self.last_result, self.result_t = 255, RESULT['NONE'], 0.0
        self.last_real = None           # (seq, type, payload) of the last real command, to spot resends
        self.last_real_result = RESULT['NONE']
        self.spare, self.rx_mask = {}, 0
        self.route, self.route_version = None, 0
        self.ltp_version = 0            # route version LTP has picked up
        self.leg, self.leg_start, self.tack_side, self.drift = 0, None, 0, 0
        self.ltp_target = None
        self.t0 = time.time()
        self.rx = dict(seq=0, type=0, t=None, ok=0, rejected=0)

    # ── Commander ──────────────────────────────────────────────────────────
    def command(self, seq, ctype, payload):
        # Like the Commander: a copy of the last real command (same seq, type, payload) is a ground station
        # resend whose report was lost. Report the first result again instead of running it twice
        key = (seq, ctype, payload.ljust(BL.PAYLOAD_MAX, b'\0'))
        if ctype != BL.COMMAND_NONE and key == self.last_real:
            self.last_seq, self.last_result, self.result_t = seq, self.last_real_result, time.time()
            print(f'seq {seq:3d} type 0x{ctype:02X} -> {BL.RESULTS[self.last_result]} (resend, not run again)', flush=True)
            return
        r = self.execute(ctype, payload)
        if ctype != BL.COMMAND_NONE:
            self.last_real, self.last_real_result = key, r
            self.last_seq, self.last_result, self.result_t = seq, r, time.time()
        elif time.time() - self.result_t > HOLD_S:
            self.last_seq, self.last_result = seq, RESULT['NONE']
        name = BL.RESULTS.get(r, r)
        if ctype != BL.COMMAND_NONE:
            print(f'seq {seq:3d} type 0x{ctype:02X} -> {name}, mode {BL.MODE_NAMES[self.mode]}', flush=True)

    def execute(self, ctype, payload):
        group = ctype >> 4
        if group == 0:
            return RESULT['NONE'] if ctype == BL.COMMAND_NONE else RESULT['UNSUPPORTED']
        if group == 1:
            mode = ctype & 0x0F
            if mode not in BL.MODE_NAMES:
                return RESULT['UNSUPPORTED']
            self.mode = mode
            if mode not in BL.HEADING_MODES and mode != M['AUTO']:
                self.target = None
            return RESULT['OK']
        if group == 2:
            if len(payload) < 2:
                return RESULT['BAD_LEN']
            deg = struct.unpack('<h', payload[:2])[0]
            if ctype == BL.COMMAND_RUDDER_SERVO:
                if self.mode not in RUDDER_MANUAL:
                    return RESULT['WRONG_MODE']
                if abs(deg) > BL.SERVO_LIMIT:
                    return RESULT['BAD_VALUE']
                self.rudder = deg
                return RESULT['OK']
            if ctype == BL.COMMAND_SAIL_SERVO:
                if self.mode not in SAIL_MANUAL:
                    return RESULT['WRONG_MODE']
                if abs(deg) > BL.SERVO_LIMIT:
                    return RESULT['BAD_VALUE']
                self.sail = deg
                return RESULT['OK']
            if ctype == BL.COMMAND_RUDDER_PID:
                if self.mode not in BL.HEADING_MODES:
                    return RESULT['WRONG_MODE']
                if not 0 <= deg <= BL.HEADING_MAX:
                    return RESULT['BAD_VALUE']
                self.target = deg
                return RESULT['OK']
            return RESULT['UNSUPPORTED']
        if group == 3:
            return self.route_cmd(ctype, payload)
        return RESULT['UNSUPPORTED']

    def route_cmd(self, ctype, payload):
        # Same checks as navigationRoute.c: index range, margin, then count / every waypoint / CRC on commit
        if ctype == BL.COMMAND_NVS_LOAD_START:
            return RESULT['OK']
        if ctype == BL.COMMAND_NVS_LOAD:
            index, margin, lat_e7, lon_e7 = struct.unpack('<HHii', payload[:WP_SIZE].ljust(WP_SIZE, b'\0'))
            if index >= BL.ROUTE_MAX_WAYPOINTS:
                return RESULT['TOO_MANY']
            if margin == 0 or not -900000000 <= lat_e7 <= 900000000 or not -1800000000 <= lon_e7 <= 1800000000:
                return RESULT['BAD_VALUE']
            self.spare[index] = payload[:WP_SIZE].ljust(WP_SIZE, b'\0')
            self.rx_mask |= 1 << index
            return RESULT['OK']
        if ctype == BL.COMMAND_NVS_LOAD_STOP:
            count, crc, arrive, warning, _ = struct.unpack('<HHBBH', payload[:8].ljust(8, b'\0'))
            required = (1 << count) - 1
            if count == 0 or count > BL.ROUTE_MAX_WAYPOINTS:
                r = RESULT['TOO_MANY']
            elif self.rx_mask & required != required:
                r = RESULT['INCOMPLETE']
            elif BL.crc16_ccitt(b''.join(self.spare[i] for i in range(count))) != crc:
                r = RESULT['BAD_CRC']
            else:
                wps = []
                for i in range(count):
                    _, margin, lat_e7, lon_e7 = struct.unpack('<HHii', self.spare[i])
                    wps.append({'lat': lat_e7 / 1e7, 'lon': lon_e7 / 1e7, 'margin': margin})
                self.route = {'wps': wps, 'arrive': arrive or 8, 'warning': warning}
                self.route_version += 1
                r = RESULT['OK']
                print(f'route loaded: {count} waypoints, arrive {arrive or 8} m, warning {warning} m', flush=True)
            self.spare, self.rx_mask = {}, 0   # every path: the next upload starts empty
            return r
        return RESULT['UNSUPPORTED']

    # ── LTP, simplified ────────────────────────────────────────────────────
    def en(self, lat, lon):
        """Local east/north metres of (lat, lon) from the boat."""
        return ((lon - self.lon) * 111320 * math.cos(math.radians(self.lat)), (lat - self.lat) * 111320)

    def ltp(self):
        if self.mode != M['AUTO'] or not self.route:
            return None
        if self.ltp_version != self.route_version:
            self.ltp_version, self.leg, self.tack_side = self.route_version, 0, 0
            self.leg_start = (self.lat, self.lon)
        wps = self.route['wps']
        wp = wps[self.leg]
        e, n = self.en(wp['lat'], wp['lon'])
        if math.hypot(e, n) <= self.route['arrive']:
            self.leg += 1
            self.leg_start = (wp['lat'], wp['lon'])
            if self.leg >= len(wps):
                print('route finished, requesting SAFE', flush=True)
                self.mode, self.route = M['SAFE'], None
                self.leg = 0
                return None
            return self.ltp()
        # cross-track from the leg line, + = starboard of the path
        se, sn = self.en(*self.leg_start)
        le, ln = e - se, n - sn
        length = math.hypot(le, ln) or 1.0
        self.drift = ((-se) * ln - (-sn) * le) / length
        if abs(self.drift) > wp['margin'] + self.route['warning']:
            print('outside the corridor, requesting SAFE', flush=True)
            self.mode = M['SAFE']
            return None
        bearing = math.degrees(math.atan2(e, n)) % 360
        rel = wrap180(bearing - self.wind_from)
        if abs(rel) >= NO_GO + 5:
            self.tack_side = 0
            return bearing
        # upwind: close-hauled on a tack, tack back at 80% of the margin
        if self.tack_side == 0:
            self.tack_side = 1 if rel >= 0 else -1
        if abs(self.drift) > 0.8 * wp['margin'] and (self.drift > 0) == (self.tack_side > 0):
            self.tack_side = -self.tack_side
        return (self.wind_from + self.tack_side * NO_GO) % 360

    # ── motion ─────────────────────────────────────────────────────────────
    def step(self, dt):
        want = None
        if self.mode == M['AUTO']:
            want = self.ltp_target = self.ltp()
            if want is None and self.mode == M['AUTO']:
                if self.target is None:
                    self.target = self.heading    # AUTO with no route holds the heading it has, like the heading PID
                want = self.target
        elif self.mode in BL.HEADING_MODES:
            want = self.target
        if want is not None:
            err = wrap180(want - self.heading)
            self.heading = (self.heading + max(-TURN_RATE * dt, min(TURN_RATE * dt, err))) % 360
            self.rudder = int(max(-35, min(35, err)))
        elif self.mode in RUDDER_MANUAL:
            self.heading = (self.heading + self.rudder * 0.3 * dt * (self.speed / max(0.1, self.top_speed))) % 360
        twa = abs(wrap180(self.heading - self.wind_from))
        sailing = self.mode not in (M['SAFE'], M['STANDBY'])
        v = self.top_speed * (0.1 if twa < 35 else min(1.0, 0.5 + twa / 180)) if sailing else 0.0
        self.speed += (v - self.speed) * min(1.0, dt / 3.0)
        go = math.radians(self.heading)
        drift = math.radians(self.wind_from + 180)
        de = self.speed * math.sin(go) * dt + (0.05 * math.sin(drift) * dt if not sailing else 0)
        dn = self.speed * math.cos(go) * dt + (0.05 * math.cos(drift) * dt if not sailing else 0)
        self.lat += dn / 111320
        self.lon += de / (111320 * math.cos(math.radians(self.lat)))

    def telemetry(self):
        t = time.time() - self.t0
        g = time.gmtime()
        awa = int(wrap180(self.wind_from - self.heading))
        target = self.ltp_target if self.mode == M['AUTO'] and self.route else self.target
        age = 255 if self.rx['t'] is None else min(254, int(time.time() - self.rx['t']))
        v = dict(battery_voltage=int(16400 - t / 10), cell_voltage=[4100, 4095, 4102, 4098], battery_valid=3,
                 sail_ang_forward=int(-awa / 2), sail_encoder_magnitude=2048, sail_encoder_status=0x20, encoder_valid=3,
                 ang_wind=awa, wind_valid=3,
                 latitude=int(round(self.lat * 1e7)), longitude=int(round(self.lon * 1e7)), altitude=12000,
                 sats_fix=(1 << 5) | 11, utc_hours=g.tm_hour, utc_minutes=g.tm_min, utc_seconds=g.tm_sec, gps_valid=3,
                 imu_cal=0xFF, imu_temp=24, imu_valid=3,
                 heading=int(self.heading) % 360, roll=int(5 * math.sin(t / 3)), pitch=0, orientation_valid=3,
                 rudder_servo_angle=self.rudder, sail_servo_angle=self.sail,
                 mode=self.mode, desired_heading=0xFFFF if target is None else int(target) % 360,
                 route_segment=self.leg, drift_distance=int(round(self.drift)) if self.route else 0,
                 last_cmd_seq=self.last_seq, uptime_ms=int(t * 1000) & 0xFFFFFFFF, last_cmd_result=self.last_result,
                 rx_last_seq=self.rx['seq'], rx_last_type=self.rx['type'], rx_last_rssi=-70, rx_last_snr=36,
                 rx_age_s=age, rx_noise=-110, rx_ok=self.rx['ok'] & 0xFFFF, rx_crc_err=0,
                 rx_rejected=self.rx['rejected'] & 0xFF, rx_dropped=0)
        vals = []
        for name, n in gs_web.FIELDS:
            vals += v[name] if n > 1 else [v.get(name, 0)]
        return struct.pack(gs_web.FMT, *vals)


def main():
    ap = argparse.ArgumentParser(description='fake ALANv boat for the ground station')
    ap.add_argument('--at', default='44.5646,-123.2620', help='start position LAT,LON')
    ap.add_argument('--wind', type=float, default=270.0, help='true wind direction, degrees it blows from')
    ap.add_argument('--speed', type=float, default=0.8, help='top speed, m/s')
    ap.add_argument('--drop', type=float, default=0.0, help='fraction of packets lost in each direction')
    ap.add_argument('--boot-mode', default='STANDBY', choices=sorted(M), help='mode at power-up')
    args = ap.parse_args()

    lat, lon = (float(v) for v in args.at.split(','))
    boat = Boat(lat, lon, args.wind, args.speed, args.boot_mode)
    master, slave = pty.openpty()
    tty.setraw(slave)
    print(f'sim serial port: {os.ttyname(slave)}', flush=True)

    seq, buf = 0, bytearray()
    next_tx = last = time.time()
    while True:
        r, _, _ = select.select([master], [], [], 0.1)
        now = time.time()
        boat.step(now - last)
        last = now
        if r:
            buf += os.read(master, 256)
            # The bridge sends each write as one LoRa packet; here the length byte splits them again
            while len(buf) >= 5:
                if buf[0] != BL.ADDR_BOAT or buf[1] != BL.ADDR_GROUND:
                    del buf[0]
                    boat.rx['rejected'] += 1
                    continue
                n = 4 + buf[3]
                if len(buf) < n:
                    break
                pkt, buf = bytes(buf[:n]), buf[n:]
                if random.random() < args.drop:
                    continue
                p = BL.parse_packet(pkt)
                if p is None or len(p[2]) > BL.PAYLOAD_MAX:
                    boat.rx['rejected'] += 1
                    continue
                boat.rx.update(seq=p[0], type=p[1], t=now, ok=boat.rx['ok'] + 1)
                boat.command(*p)
        if now >= next_tx:
            next_tx += 1.0
            payload = boat.telemetry()
            if random.random() >= args.drop:
                os.write(master, bytes([BL.ADDR_GROUND, BL.ADDR_BOAT, seq, 1 + len(payload), 0]) + payload)
            seq = (seq + 1) & 0xFF


if __name__ == '__main__':
    main()
