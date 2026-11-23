# Browser ground station

`gs_web.py` talks to the SAMD21 LoRa bridge over USB and serves the ground station at http://localhost:8000.
Python standard library only.

```
python3 gs_web.py                  # first /dev/ttyACM* it finds, or pick the port in the page
python3 gs_web.py /dev/ttyACM0
```

## Before going to the lake: cache the map

The satellite map comes from Esri World Imagery and is cached in `tiles/`. There is usually no internet at the
lake, so download the area at home (latitude, longitude, radius in metres):

```
python3 gs_web.py --prefetch 44.5646,-123.2620,800
```

Without cached tiles the map still works, on a plain grid.

## Sending a route

1. Click the map to add waypoints, in order. Drag one to move it, right-click (or ✕) to remove it.
2. Set each waypoint's **margin**: how far LTP may let the boat drift off the line on the leg that ends at that
   waypoint. It tacks back at 80% of it. **Arrive** is how close counts as reaching a waypoint. **Warning** is extra
   room past the margin: beyond margin + warning the boat gives up and switches to SAFE.
3. **Send route + AUTO**. Leg 1 starts wherever the boat is when AUTO picks the route up (the dashed line).
   **Send only** loads the route without changing the mode, so you can switch to AUTO later.

The corridor turns green once the boat has confirmed the route, and the leg it is on is highlighted.
The plan is kept in the browser, so a reload keeps it.

## How commands go out

Every command is a real command packet (`boatlink.py`). The ground station sends one packet per telemetry cycle,
right after a telemetry packet arrives, because that is when the boat's radio is listening. It resends a command
with the same seq until the boat's telemetry reports that seq in `last_cmd_seq` (up to 5 tries), and shows the
result (OK, WRONG_MODE, ...) in the Commands list. A route upload is start, one command per waypoint, commit (with a
CRC), then AUTO if asked. If the commit fails it uploads the whole route again, up to 3 times.

The radio framing lives in `boatlink.build_packet()`, so error correction only needs to change that function.
Command numbers come from `../terminal/commands.h` and the telemetry layout from `../terminal/telemetry.h`.

## Testing without the boat

`sim_cm4.py` is a fake boat behind a pseudo serial port. It applies the same mode rules and route checks as the
CM7 Commander, sails a simple version of LTP, and can lose packets on purpose:

```
python3 sim_cm4.py --drop 0.2          # prints e.g. "sim serial port: /dev/pts/3"
python3 gs_web.py /dev/pts/3
```

Options: `--at LAT,LON` start position, `--wind DEG` true wind direction, `--boot-mode STANDBY|AUTO|...`.
