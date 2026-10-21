# Virtual board (Renode)

Runs the **CM7 firmware** on an emulated NUCLEO-H755ZI-Q with simulated sensors, so you can develop without the board.
For the full simulator (boat physics, 3D dashboard) see `../README.md`; this page covers Renode on its own.

When a physics plant is listening on port 5555 (the dashboard, or `sim/plant/run_plant.py`), the
`plant` peripheral (`models/AlanPlantBridge.cs`) reads the firmware's servo PWM every 20 ms of virtual time and
drives the sensor models from the physics. Without a plant, the sensors keep the values you set by hand.

## Install (macOS, Apple Silicon)

1. Download `renode-<version>.osx-arm64-portable.dmg` from https://github.com/renode/renode/releases
2. Drag `Renode.app` into `/Applications`
3. Optional, so `renode` works in any terminal:
   ```bash
   echo 'export PATH="/Applications/Renode.app/Contents/MacOS:$PATH"' >> ~/.zshrc
   ```

## Run

From the repo root:

```bash
sim/renode/run.sh                            # build the sim firmware, open Renode (monitor + USART3 console)
sim/renode/run.sh --headless 20              # build, run 20 virtual seconds, print the console here
sim/renode/run.sh --sensors imu,encoder      # choose sensors (default: wind,imu,encoder,gps)
sim/renode/run.sh --no-build                 # reuse the last sim build
```

`run.sh` builds into `sim/renode/build/cm7` (git-ignored) and turns sensors on with `-D` flags,
so `Common/Inc/shared_configuration.h` is never modified.

In the Renode monitor: `start`, `pause`, `runMacro $reset` (reload the ELF after a rebuild), `quit`.

Expect the first sensor output about 6 virtual seconds in: the servo test sweeps run before the scheduler starts,
and the GPS task waits another 5 seconds before it polls.

## Change sensor inputs while it runs

Type these in the Renode monitor:

| Sensor | Command | Firmware sees |
|---|---|---|
| BNO055 heading | `i2c2.bno055 Heading 90` | Yaw/heading 90° (quaternion, Euler, magnetometer) |
| BNO055 turn rate | `i2c2.bno055 YawRate 5` | Gyro Z = 5 °/s |
| AS5600 wing angle | `i2c1.as5600 Angle 45` | Raw angle 512 (45°) |
| Wind vane | `windvane Angle 270.5` | Modbus reply 270.5° |
| GPS | `gps Latitude 41.26`, `gps Longitude -111.82`, `gps Satellites 8`, `gps Fix 1` | `$GPGGA` once per second at 9600 baud |
| User button | `gpioPortC.UserButton1 Press` / `Release` | PC13 |

The models are in `models/*.cs` (C#, compiled by Renode when the script loads).

## Debug with GDB

```bash
renode --disable-gui --console -e 'include @sim/renode/alan_cm7.resc; machine StartGdbServer 3333'
# in another terminal:
arm-none-eabi-gdb sim/renode/build/cm7/Autonomaus_Sailboat_CM7.elf -ex 'target remote :3333' -ex 'monitor start'
```

Breakpoints, stepping and variable inspection work as on the real board. Pausing in GDB freezes virtual time.
After a crash, `cpu PC` and ``sysbus FindSymbolAt `cpu PC` `` in the monitor show where the CPU stopped.

## What is and isn't emulated

| Works | Not emulated |
|---|---|
| CM7 core, FreeRTOS, HAL init, clocks, SysTick, `HAL_Delay` | The CM4 core (Renode's H7 models are single-core) |
| USART3 printf console | SPI1, so no LoRa |
| BNO055, AS5600, wind vane, GPS (simulated values you control) | Real sensor noise, magnetic interference, GPS error |
| TIM1 servo PWM registers, TIM6, GPIO/EXTI, LEDs, button, HSEM, SRAM4 | Physical servo motion and current draw |

Good for: control logic, state machines, RTOS tasks and their interaction, sensor parsing, console commands.
Still needs the real board: LoRa, CM4/cross-core behavior, real sensor behavior, tuning.

## Known firmware issues found in simulation (current `master`)

- **HardFault with the IMU enabled:** `sensorMagnetometerTask` is created with a 128-word stack
  (`CM7/App/Src/controller.c`), too small for its `printf` and math. It overflows, corrupts the
  FreeRTOS task list and the CPU ends in `HardFault_Handler` about 8 virtual seconds in.
  Raising it to 512 words fixes it in simulation.
- **Wind task never yields:** `sensorWind_handler` loops with no `vTaskDelay`, busy-waiting on UART4.
  It floods the console and takes most of the CPU from same-priority tasks.
- **`printf` from several tasks interleaves** mid-line (no mutex or logging task).
- **GPS longitude print:** negative values print as `-111.-842109` (the fraction keeps its sign); the stored value is correct.
