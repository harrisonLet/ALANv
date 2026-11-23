"""
boatlink.py - what the ground station sends to the boat, byte for byte

Command packet (ground -> boat), one per LoRa packet:
    [0] dest 0xBB  [1] src 0xAA  [2] seq  [3] len = 1 + payload bytes  [4] command type  [5..] payload
Payloads are 0 to 16 bytes: none for heartbeats and modes, a little-endian int16 for angles and headings,
12 bytes for a route waypoint, 8 bytes for a route commit.

Command types, modes and results are read from ../terminal/commands.h at import, so the C header stays
the single source of truth on the ground station (it is a copy of Common/Inc/sharedCommand.h on the boat).

If the radio framing changes (error correction), only build_packet() needs to change.
"""
import os
import re
import struct

HERE = os.path.dirname(os.path.abspath(__file__))
COMMANDS_H = os.path.join(HERE, '..', 'terminal', 'commands.h')

ADDR_GROUND = 0xAA
ADDR_BOAT = 0xBB
PAYLOAD_MAX = 16               # IPC_CMD_DATA_SIZE on the boat
ROUTE_MAX_WAYPOINTS = 32       # NAV_ROUTE_MAX_WAYPOINTS on the boat
HEADING_MAX = 359



def _enums(path):
    """{NAME: value} for every `NAME = 0x..` line in the header."""
    src = re.sub(r'/\*.*?\*/', '', open(path).read(), flags=re.S)
    return {name: int(val, 0) for name, val in re.findall(r'\b([A-Z][A-Z0-9_]+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)', src)}


_E = _enums(COMMANDS_H)
COMMAND_NONE = _E['COMMAND_NONE']
COMMAND_SAIL_SERVO = _E['COMMAND_SAIL_SERVO']
COMMAND_RUDDER_SERVO = _E['COMMAND_RUDDER_SERVO']
COMMAND_SAIL_PID = _E['COMMAND_SAIL_PID']
COMMAND_RUDDER_PID = _E['COMMAND_RUDDER_PID']
COMMAND_NVS_LOAD_START = _E['COMMAND_NVS_LOAD_START']  # start of a route upload
COMMAND_NVS_LOAD = _E['COMMAND_NVS_LOAD']              # one waypoint
COMMAND_NVS_LOAD_STOP = _E['COMMAND_NVS_LOAD_STOP']    # commit: count, CRC, arrival radius, warning zone
SERVO_LIMIT = 90               # LIMIT_SERVO_DEG

# system_mode_t: {'AUTO': 0, 'FULL_MAN': 1, ...} and the command that switches to it (0x10 + mode)
MODES = {k[len('SYSTEM_MODE_'):]: v for k, v in _E.items() if k.startswith('SYSTEM_MODE_') and k != 'SYSTEM_MODE_COUNT'}
MODE_NAMES = {v: k for k, v in MODES.items()}
MODE_COMMAND = {k: _E['COMMAND_MODE_' + k] for k in MODES}
RESULTS = {v: k[len('CMD_RESULT_'):] for k, v in _E.items() if k.startswith('CMD_RESULT_') and k != 'CMD_RESULT_COUNT'}

# Modes in which the boat steers by the heading PID (rudder semi), so a heading command is accepted
HEADING_MODES = {MODES[m] for m in ('FULL_SEMI', 'SEMI_RUDDER_MAN_SAIL', 'SEMI_RUDDER_AUTO_SAIL')}


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE, same as crc16_ccitt() in CM7/App/Src/navigationRoute.c."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_packet(seq, ctype, payload=b''):
    """One command packet. The only function to change if the radio framing changes."""
    if len(payload) > PAYLOAD_MAX:
        raise ValueError(f'payload of {len(payload)} bytes, the boat takes at most {PAYLOAD_MAX}')
    return bytes([ADDR_BOAT, ADDR_GROUND, seq & 0xFF, 1 + len(payload), ctype & 0xFF]) + payload


def parse_packet(pkt):
    """(seq, type, payload) of a command packet, or None if it is not one. Used by the fake boat."""
    if len(pkt) < 5 or pkt[0] != ADDR_BOAT or pkt[1] != ADDR_GROUND or len(pkt) != 4 + pkt[3] or pkt[3] < 1:
        return None
    return pkt[2], pkt[4], bytes(pkt[5:])


def angle_payload(deg):
    return struct.pack('<h', int(deg))


def waypoint_payload(index, margin_m, lat, lon):
    """ipc_cmd_waypoint_t: index, margin_m, lat_e7, lon_e7 (little-endian, 12 bytes)."""
    return struct.pack('<HHii', index, margin_m, int(round(lat * 1e7)), int(round(lon * 1e7)))


def commit_payload(count, crc, arrive_m, warning_m):
    """ipc_cmd_route_commit_t: count, crc16, arrive_m, margin_warning_m, reserved (8 bytes)."""
    return struct.pack('<HHBBH', count, crc, arrive_m, warning_m, 0)


def route_commands(waypoints, arrive_m, warning_m):
    """[(type, payload, label)] to upload a route, and its CRC.

    waypoints: [{'lat', 'lon', 'margin'}]. The first leg starts where the boat is when AUTO picks the
    route up, so waypoints are only the destinations. Leg i is kept within waypoint i's margin.
    """
    if not 1 <= len(waypoints) <= ROUTE_MAX_WAYPOINTS:
        raise ValueError(f'a route needs 1 to {ROUTE_MAX_WAYPOINTS} waypoints')
    if not 0 <= arrive_m <= 255 or not 0 <= warning_m <= 255:
        raise ValueError('arrival radius and warning zone must be 0 to 255 m')
    wps = []
    for i, w in enumerate(waypoints):
        m = int(round(w['margin']))
        if not 1 <= m <= 255:
            raise ValueError(f'waypoint {i + 1}: margin must be 1 to 255 m')
        lat, lon = float(w['lat']), float(w['lon'])
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError(f'waypoint {i + 1}: position out of range')
        wps.append(waypoint_payload(i, m, lat, lon))
    crc = crc16_ccitt(b''.join(wps))
    cmds = [(COMMAND_NVS_LOAD_START, b'', 'route start')]
    cmds += [(COMMAND_NVS_LOAD, p, f'waypoint {i + 1}') for i, p in enumerate(wps)]
    cmds.append((COMMAND_NVS_LOAD_STOP, commit_payload(len(wps), crc, arrive_m, warning_m),
                 f'route commit ({len(wps)} wp, CRC {crc:04X})'))
    return cmds, crc
