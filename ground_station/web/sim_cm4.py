#!/usr/bin/env python3
"""
sim_cm4.py - fake CM4 + SAMD21 bridge for testing the ground station without hardware

Opens a pseudo-terminal, sends a LoRaTelemetry_t packet once per second and
reacts to command keys (rudder/sail steps) written by the ground station.

Run:   python3 sim_cm4.py            (prints the port, e.g. /dev/ttys030)
       python3 gs_web.py <port>
"""
import math, os, pty, select, struct, sys, time, tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gs_web  # reuse FMT/FIELDS parsed from telemetry.h

STEP = {'q': ('r', 20), 'e': ('r', -20), 'a': ('r', 10), 'd': ('r', -10), 'z': ('r', 5), 'c': ('r', -5),
        'w': ('s', 20), 'r': ('s', -20), 's': ('s', 10), 'f': ('s', -10), 'x': ('s', 5), 'v': ('s', -5)}

master, slave = pty.openpty()
tty.setraw(slave)
print(f'sim serial port: {os.ttyname(slave)}', flush=True)

rudder = sail = 0
seq = cmd_seq = 0
t0 = time.time()
lat, lon, hdg = 44.5646, -123.2620, 90.0
next_tx = time.time()
while True:
    r, _, _ = select.select([master], [], [], max(0, next_tx - time.time()))
    if r:
        for ch in os.read(master, 64).decode(errors='ignore'):
            if ch in STEP:
                which, d = STEP[ch]
                if which == 'r': rudder = max(-135, min(135, rudder + d))
                else:            sail = max(-135, min(135, sail + d))
                cmd_seq = (cmd_seq + 1) & 0xFFFF
                print(f'cmd {ch!r}: rudder={rudder} sail={sail}', flush=True)
    if time.time() < next_tx:
        continue
    next_tx += 1.0
    t = time.time() - t0
    hdg = (hdg - rudder * 0.05) % 360
    lat += 2e-5 * math.cos(math.radians(hdg)); lon += 2e-5 * math.sin(math.radians(hdg))
    g = time.gmtime()
    v = dict(battery_voltage=int(16400 - t), cell_voltage=[4100, 4095, 4102, 4098], battery_valid=3,
             sail_ang_forward=sail, sail_encoder_magnitude=2048, sail_encoder_status=0x20, encoder_valid=3,
             ang_wind=int(45 + 15 * math.sin(t / 10)), wind_valid=3,
             latitude=lat, longitude=lon, altitude=70.0, satellites=9, fix_valid=1,
             utc_hours=g.tm_hour, utc_minutes=g.tm_min, utc_seconds=g.tm_sec, gps_valid=3,
             imu_cal=0xFF, imu_temp=24, imu_valid=3,
             heading=int(hdg) % 360, roll=int(10 * math.sin(t / 3)), pitch=int(3 * math.sin(t / 2)),
             orientation_valid=3, rudder_servo_angle=rudder, sail_servo_angle=sail,
             mode=1, desired_heading=90, route_segment=0,
             drift_distance=int(5 * math.sin(t / 20)), last_cmd_seq=cmd_seq, uptime_ms=int(t * 1000))
    vals = []
    for name, n in gs_web.FIELDS:
        vals += v[name] if n > 1 else [v[name]]
    payload = struct.pack(gs_web.FMT, *vals)
    os.write(master, bytes([gs_web.ADDR_GROUND, gs_web.ADDR_BOAT, seq, 1 + len(payload), 0]) + payload)
    seq = (seq + 1) & 0xFF
