# ALAN simulator

Sail the real firmware on a virtual boat: the CM7 firmware runs on an emulated STM32 (Renode), a physics model
of the boat sails in simulated wind and current, and a browser dashboard shows it in 3D with live controls.

```
 Browser dashboard (3D view, wind/current/fault controls, charts, firmware console)
        │  HTTP + live stream
 sim/dashboard/server.py ── boat physics (sim/plant) ── lockstep link :5555 ──┐
        │ builds + starts                                                        │
 Renode: emulated STM32H755 (CM7) running your firmware ─ AlanPlantBridge ──────┘
        servo PWM (TIM1)  ──►  physics  ──►  simulated BNO055 / AS5600 / wind vane / GPS
```

The firmware is the same code you flash; only the sensor enable flags differ (passed as `-D` flags, so
`Common/Inc/shared_configuration.h` is untouched). Every sensor is emulated at the bus level (I2C registers,
Modbus over UART, NMEA over UART), so the real drivers run unmodified.

## Quick start

```bash
python3 sim/dashboard/server.py          # opens http://localhost:8080
```

In the dashboard:
1. Without Renode the boat runs in **physics-only** mode: steer it with the Manual servos sliders.
2. Pick sensors and click **Build & start**. The header switches to **FIRMWARE IN THE LOOP**: the boat now
   follows the servo PWM your firmware outputs, and the Firmware console shows its `printf` output.
3. Change wind, gusts, shifts and water current with the sliders; cut the GPS or wind vane under Sensor faults.
4. **Override firmware** lets you drive the servos by hand while the firmware keeps running.

Needs: Renode in `/Applications` (see `renode/README.md`), the Arm toolchain (`./build.sh` works), Python 3.
The 3D view loads three.js from a CDN, so the first load needs internet. Add `?no3d` to the URL on slow machines.

## Control the servos from your Mac terminal

The firmware has a serial command console (`CM7/App/Src/console.c`, enabled with `CM7_CONSOLE_ENABLE`; the
simulator build turns it on). It reads USART3, the same port as the ST-LINK USB serial on the real board.

```bash
python3 sim/dashboard/server.py --firmware     # dashboard + your firmware in Renode
python3 sim/tools/board.py --quiet             # in a second terminal: type commands to the virtual board
python3 sim/tools/board.py /dev/tty.usbmodem*  # the same tool, the real board over USB
```

Commands: `tail 30`, `rudder -10`, `center`, `sweep tail`, `sweep rudder`, `status`, `help`.
They go through your firmware's servo code, out as PWM, and move the simulated (or real) servos.
The Bench tab has the same console box, and a checkbox that sends its servo sliders through it.
(`nc localhost 3456` works too; it is the raw simulated serial port.)

## Dashboard tabs

| Tab | Shows |
|---|---|
| **Live** | 3D boat from your CAD, floating at its computed waterline, heeling with the wind; track, wind streaks, charts, firmware console |
| **Bench** | the boat on its stand with a fan: set a sail angle and see the tail angle it needs (with the math), test each servo, move the wind vane, firmware console |
| **Route** | click A and B, plan tacks/gybes inside a corridor, sail it with the sim autopilot in changing wind |
| **Stability** | where it floats, centre of gravity vs buoyancy at any heel, righting moment curve, heel for a given wind, editable mass list |
| **Points of sail** | the physics boat held at every heading: speed, wing angle, tail angle, heel; speed polar and best upwind VMG |
| **How it works** | what is real and what is simulated, and the edit → simulate → flash loop |

The banner under the header always says who is steering: you (sliders) or your firmware, with the branch and commit
it was built from (and whether it includes uncommitted edits).

## The boat model

Built from the Onshape exports by `sim/tools/prepare_cad.py` (see `dashboard/models/README.md`):
measured wing, tail, keel and rudder areas; mast, rudder post and vane positions; yaw inertia from Onshape;
buoyancy from a voxel model of the hull and keel; mass and CG from `dashboard/models/mass.json`.
Still estimated: hull drag, the tail's effect on the wing (`tail_to_aoa`), and how strongly the wing bends the flow
around the wind vane. Calibrate those from water tests.

## Pieces

| Folder | What it is |
|---|---|
| `dashboard/` | Web dashboard and its server (Python standard library only). `models/` takes your CAD STLs. |
| `plant/` | Boat physics: hull, keel, rudder, self-trimming wing + tail, heel from the hydrostatics, wind with gusts, current, wind-vane position effects, sensor noise. |
| `tools/prepare_cad.py` | Turns the Onshape STL exports into the simulator's boat model. |
| `plant/scenarios/` | Starting conditions and timed events (wind shifts, sensor dropouts). Add your own. |
| `renode/` | Emulated board, sensor models (`models/*.cs`), lockstep bridge, `run.sh` build/run script. |

Other ways to run it:

```bash
python3 sim/plant/run_plant.py --scenario sim/plant/scenarios/gusty_beam_reach.json   # terminal plant, type commands
sim/renode/run.sh --sensors wind,encoder,gps                                          # Renode in this terminal
python3 sim/plant/run_plant.py --standalone 120 --fast                                # physics only, no firmware
sim/.venv/bin/python sim/plant/viz.py                                                  # live plots of the latest run
```

Every run is logged to `sim/plant/logs/run-*.csv` (`latest.csv` points at the newest).
`viz.py` needs matplotlib: `python3 -m venv sim/.venv && sim/.venv/bin/pip install matplotlib`.

## Boat parameters

`sim/plant/alan_plant/boat.py` (`BoatParams`) holds the defaults; the CAD values override them at startup
(`python3 sim/plant/cad_check.py` prints what was measured), and a scenario can override anything under `"boat": {...}`.

Sign conventions: heading clockwise from true north; body x forward, y starboard; AWA positive = wind from
starboard; wing angle positive = leading edge to starboard; AoA = AWA - wing angle. If the firmware's rudder
or tail direction turns out opposite on the real boat, flip `rudder_sign` / `tail_sign`.

## Limits

- Only the CM7 core is emulated; CM4 and LoRa (SPI1) are not (see "LoRa" below).
- Renode runs the core at 24 MIPS (not 64 MHz) so the current busy-waiting firmware keeps up with real time; task
  timing is unaffected, only pure computation takes longer in virtual time (`sim/renode/alan_cm7.resc`).
- The Route tab's autopilot is a simulator stand-in, not the firmware's Live Tack Planner.

## LoRa (not simulated yet)

The radio code runs on the CM4, which Renode's STM32H7 model does not include. Simulating it needs: the CM4 as a
second emulated machine (booting on its own instead of waiting for the CM7's wake-up), SPI1 on that machine, a
register-level model of the RFM95W (SX1276) radio that sends and receives packets over a local socket, and a
ground-side stand-in for the SAMD21 bridge (a terminal tool or the ground station) on your Mac.
- Heel is quasi-static (it settles where the righting moment balances the wing), no pitching or waves in the physics
  (waves in the 3D view are visual only).
- Physics parameters are estimates until measured; use the sim to develop logic, the water to tune gains.
