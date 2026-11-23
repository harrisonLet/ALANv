"""Ground-station commands for the emulated firmware, and what its console says back.

The CM4 and its LoRa radio are not emulated, so the dashboard plays the ground station and the CM4 at once: it
builds the same ipc_cmd_t records the CM4 would push, and the plant bridge (sim/renode/models/AlanPlantBridge.cs)
writes them into the IPC command ring in SRAM4. From there they take the real path: Commander pops them, switches
modes, and hands waypoints to the Route Manager, which LTP sails.

Layouts come from the firmware's own headers:
  ring layout          Common/Inc/ipc.h (read by compiling a small helper, see ring_layout())
  command ids, modes   Common/Inc/sharedCommand.h
  waypoint, commit     ipc_cmd_waypoint_t / ipc_cmd_route_commit_t in Common/Inc/ipc.h
"""
import os
import re
import struct
import subprocess
import tempfile

COMMAND_NONE = 0x00
COMMAND_MODE_BASE = 0x10            # COMMAND_MODE_* = 0x10 + system_mode_t
COMMAND_NVS_LOAD_START = 0x31
COMMAND_NVS_LOAD = 0x32
COMMAND_NVS_LOAD_STOP = 0x33

SYSTEM_MODES = {"auto": 0x00, "full_man": 0x01, "safe": 0x02, "standby": 0x05, "full_semi": 0x06}
MODE_NAMES = {0x00: "AUTO", 0x01: "FULL_MAN", 0x02: "SAFE", 0x03: "AUTO_SAIL_MAN_RUDDER", 0x04: "AUTO_RUDDER_MAN_SAIL",
              0x05: "STANDBY", 0x06: "FULL_SEMI", 0x07: "SEMI_RUDDER_MAN_SAIL", 0x08: "SEMI_RUDDER_AUTO_SAIL",
              0x09: "SEMI_SAIL_MAN_RUDDER", 0x0A: "SEMI_SAIL_AUTO_RUDDER"}
RESULT_NAMES = {0: "NONE (try again)", 1: "OK", 2: "UNSUPPORTED", 3: "BAD_LEN", 4: "BAD_VALUE", 5: "TOO_MANY",
                6: "INCOMPLETE", 7: "BAD_CRC", 8: "DROPPED", 9: "WRONG_MODE"}
NAV_STATES = {0: "IDLE", 1: "DIRECT", 2: "UPWIND_PORT", 3: "UPWIND_STBD", 4: "TACKING", 5: "ARRIVED", 6: "FAULT"}

MAX_WAYPOINTS = 32                  # NAV_ROUTE_MAX_WAYPOINTS

# Layout of the current ipc.h, used only if no C compiler is found
FALLBACK_LAYOUT = {"base": 0x38000000, "head": 88, "tail": 92, "slots": 96, "slot_size": 18, "count": 64, "data": 16}

_LAYOUT_SRC = r"""
#include <stdio.h>
#include <stddef.h>
#include "ipc.h"
int main(void) {
    printf("%lu %zu %zu %zu %zu %u %u\n", (unsigned long)IPC_SHARED_BASE,
           offsetof(ipc_shared_t, cmd.head), offsetof(ipc_shared_t, cmd.tail), offsetof(ipc_shared_t, cmd.slot),
           sizeof(ipc_cmd_t), (unsigned)IPC_CMD_RING_SLOTS, (unsigned)IPC_CMD_DATA_SIZE);
    return 0;
}
"""


def ring_layout(repo):
    """Addresses of the command ring, measured from ipc.h. Returns (layout dict, note for the log)."""
    with tempfile.TemporaryDirectory() as d:
        src, exe = os.path.join(d, "layout.c"), os.path.join(d, "layout")
        with open(src, "w") as f:
            f.write(_LAYOUT_SRC)
        for compiler in ("cc", "gcc", "clang"):
            try:
                r = subprocess.run([compiler, "-I", os.path.join(repo, "Common", "Inc"), src, "-o", exe],
                                   capture_output=True, text=True, timeout=30)
            except (OSError, subprocess.TimeoutExpired):
                continue
            if r.returncode != 0:
                continue
            out = subprocess.run([exe], capture_output=True, text=True, timeout=10).stdout.split()
            keys = ("base", "head", "tail", "slots", "slot_size", "count", "data")
            return dict(zip(keys, map(int, out))), "command ring layout read from ipc.h"
    return dict(FALLBACK_LAYOUT), "no C compiler found: using the command ring layout of the current ipc.h"


def ring_spec(layout):
    """The bridge's ipc_ring field."""
    return "ipc_ring={:x},{},{},{},{},{}".format(layout["base"], layout["head"], layout["tail"], layout["slots"],
                                                 layout["slot_size"], layout["count"])


def crc16_ccitt(data):
    """Same as crc16_ccitt() in navigationRoute.c: CRC-16/CCITT-FALSE."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def command_hex(layout, ctype, seq, payload=b""):
    """One ipc_cmd_t: type, seq, then the raw payload padded with zeros."""
    data = layout["data"]
    if len(payload) > data:
        raise ValueError(f"payload of {len(payload)} bytes does not fit in {data}")
    rec = bytes([ctype & 0xFF, seq & 0xFF]) + payload + bytes(data - len(payload))
    if len(rec) != layout["slot_size"]:
        raise ValueError("ipc_cmd_t size does not match ipc.h")
    return rec.hex()


def waypoint_payload(index, margin_m, lat, lon):
    """ipc_cmd_waypoint_t: index, margin_m, lat_e7, lon_e7 (little-endian, 12 bytes)."""
    return struct.pack("<HHii", index, margin_m, int(round(lat * 1e7)), int(round(lon * 1e7)))


def route_commands(points, arrive_m, warning_m):
    """LOAD_START, one LOAD per waypoint, LOAD_STOP with the CRC. points: [(lat, lon, margin_m), ...]."""
    if not 1 <= len(points) <= MAX_WAYPOINTS:
        raise ValueError(f"a route needs 1 to {MAX_WAYPOINTS} waypoints")
    wps = [waypoint_payload(i, int(m), lat, lon) for i, (lat, lon, m) in enumerate(points)]
    crc = crc16_ccitt(b"".join(wps))
    commit = struct.pack("<HHBBH", len(points), crc, int(arrive_m), int(warning_m), 0)
    return [(COMMAND_NVS_LOAD_START, b"")] + [(COMMAND_NVS_LOAD, w) for w in wps] + [(COMMAND_NVS_LOAD_STOP, commit)], crc


# ---- what the firmware prints --------------------------------------------------------------------

LTP_LINE = re.compile(r"state (\d+) leg (\d+) xte (-?\d+) m \| wind apparent (-?\d+) estimated (-?\d+) \| "
                      r"heading (\d+) target (-?\d+) \| (-?\d+) cm/s(?: \| valid gps (\w+) imu (\w+) wind (\w+))?")
ROUTE_LINE = re.compile(r"route version (\d+), (\d+) waypoints")
RESULT_LINE = re.compile(r"seq (\d+) type 0x([0-9A-Fa-f]{2}) -> result (\d+), mode (\d+)")
SAFE_ASK_LINE = re.compile(r"(route finished|outside the corridor), requesting SAFE")   # LTP asks Commander
SAFE_DONE_LINE = re.compile(r"SAFE requested by another task")                          # Commander obeys
