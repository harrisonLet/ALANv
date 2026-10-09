<!------------------------------------------------------------------------>

# GOALS

1) Manual Sailing: Control the rudder and tail sail manually over LoRa.

2) Autonomous Sailing
    1) Generate optimal sailing route from A to B using wind and map data on local computer.
    2) Send the optimal sailing route to the sailboat's control system.
    3) Adjust the sailboat's heading and sail positions in real-time based on sensor data and the optimal sailing route.
    4) Arrive at Point B safely.
    [Claude]: Define "arrive" (arrival radius around B, in meters) and "safely" (e.g. no tack in irons, battery above a floor, link-loss behaviour). What does the boat do after arriving: hold station, heave-to, or drop sails?

3) Semi-Autonomous Sailing:
    1) Control only the rudder manually over LoRa, while the sail is adjusted autonomously.
    2) Control only the sail manually over LoRa, while the rudder is adjusted autonomously.

4) Telemetry
    1) Collect sensor data from the sailboat.
    2) Save the collected telemetry data locally.
    3) Transmit the telemetry data to the ground station.
    4) Display the telemetry data on the ground station.

5) Ground Station GUI
    1) Display real-time telemetry data from the sailboat.
    2) Provide controls for manual sailing over LoRa.
    3) Visualize the optimal sailing route and current sailboat position.
    4) Log telemetry data for later analysis.

6) Hardening
    1) Waterproof boat
    2) Reinforced Sail & Sail Tail
    3) Waterproof Sail & Sail Tail

7) Vanity
    1) Painted Hull
    2) Painted Sail & Sail Tail
    3) Smooth finish on all surfaces
    4) Quick assembly and disassembly

<!------------------------------------------------------------------------>

# ASSIGNMENTS

[Claude]: Scheduling risk: every task depends on shared data types (WVData, GPSData, IMUData, EData, BMData, telemetry record, NVS, command packet) that nobody owns yet. Assign these to one person and finish them before the 10/19 pollers, otherwise each person will invent their own layout.
[Corbin]: I will handle this.

[Claude]: Jared's 11/02 arbiters and PIDs need the source-selection logic that lives in Harrison's Commander and Chief (due 11/09). Either move a minimal mode switch (manual heading / direct drive / auto) into Jared's scope, or have the arbiters take a simple "source" input that Commander and Chief fills in later.

## Corbin

Due 10/19/2026
* CM7: POLLING: Wind Vane
* CM7: POLLING: GPS
* CM7: POLLING: IMU
* CM7: POLLING: Encoder
* CM7: POLLING: Battery Monitor

## Connor

Due 10/26/2026
* CM7: PROCESS: Save Telemetry Data

## Jared

Due 11/02/2026, all functions are limited to manually provided headings
* CM7: PID: Heading Control
* CM7: ARBITER: Set Rudder Servo Angle
* CM7: PROCESS: Live Sail Planning (LSP)
* CM7: PID: Sail Control
* CM7: ARBITER: Set Sail Tail Servo Angle

Due ..., all functions are fully autonomous
* CM7: PID: Heading Control
* CM7: ARBITER: Set Rudder Servo Angle
* CM7: PROCESS: Live Sail Planning (LSP)
* CM7: PID: Sail Control
* CM7: ARBITER: Set Sail Tail Servo Angle

## Charbel

Due 10/26/2026, limited to a basic implementation plan, no actual code
* CM7: PROCESS: Live Tack Planning (LTP)

Due 11/09/2026, initial implementation
* CM7: PROCESS: Live Tack Planning (LTP)

## Harrison

Due 11/02/2026
* CM4: Receive LoRa Data
* CM4: Send LoRa Data

Due 11/09/2026
* CM7: PROCESS: Commander and Chief

## Unassigned

Due Someday
* LOCAL COMPUTER: Route Planning
* LOCAL COMPUTER: Ground Station GUI

<!------------------------------------------------------------------------>

# OPERATIONS

## LOCAL COMPUTER

<!-- Ground Station -->

### Route Planning

Given a starting point and a series of waypoints, dynamically generate the optimal sailing route considering general wind conditions and known land masses. The generated route will be a series of vectors describing the ideal path and the margin of deviation for each segment.

* Inputs:
    * Point A, starting point
    * Point B, destination point
    * Point ..., additional optional waypoints
* Stored:
    * Land/Water SVG Map
    * Standard Wind Map
* Outputs:
    * Nautical Vector Series (NVS), array of vectors and margins representing the ideal path for the boat to follow and the margin of deviation for each path segment.

[Claude]: Define the NVS wire format now, since it constrains CM4 and LTP: each segment as start/end lat-lon (or heading + length), plus margin as a lateral corridor half-width in meters. Keep each segment small and fixed size, with a count and CRC for the whole set, so it can be sent in chunks.

[Claude]: Plan for the NVS size limit given LoRa airtime, and decide whether the route is re-sent or re-planned when the boat leaves the corridor.

### Ground Station GUI

Receive and display boat telemetry alongside its position and planned route, provide LoRa controls for manual sailing, and record telemetry for later analysis.

* Inputs:
    * Real-time telemetry data from the sailboat
    * User Input
* Stored: 
    * Map spec
    * Boat spec
* Outputs:
    * Visual representation of critical info
        * Sailboat position
        * Sailboat orientation
        * NVS
        * Desired Heading (DHeading)
        * Rudder Servo Angle
        * Sensor Data
        * True Sail Angle
        * Desired Sail Angle
        * Sail Tail Servo Angle
    * Controls for manual sailing over LoRa
    * Logs of telemetry data for later analysis

[Claude]: The GUI should also show link health (age of last telemetry packet, RSSI/SNR), current mode, and command acknowledgement. Without an ACK/retry for mode and direct-drive commands, the operator cannot tell whether a command arrived. Direct drive should be sent as a repeated stream, not a one-shot.

[Claude]: The GUI list includes DHeading, Desired Sail Angle, Rudder Servo Angle and Sail Tail Servo Angle, but Save Telemetry Data on CM7 does not collect them. Add them to the telemetry record.

<!---->

## CM7

[Claude]: Every sensor output should carry a timestamp and a valid flag (and a sequence number), not only a value. The Stale field then has a concrete meaning, and consumers (PIDs, LTP, arbiters, telemetry) can fall back instead of acting on old data.
[Corbin]: I will add a timestamp, valid flag, and sequence to the sensor outputs, though I'm not sure we'll ever actually use the sequence number.

[Claude]: Define the frames once, in Common/Inc: heading in degrees 0-360 clockwise (true or magnetic, pick one), wind angle relative to the bow in -180..180 with positive meaning starboard, and whether WVData is apparent or true wind. The plan currently says only "measured wind angle", and LSP and LTP need different things (see their notes).
[Corbin]: I will define the frames in Common/Inc as suggested.

[Claude]: Each of the "PROCESS" and "PID" items should be a periodic RTOS task driven by a fixed rate (vTaskDelayUntil or a timer), not event-driven, so PID dt is known. See the suggested rate table in the AI section.

<!-- Control center -->

### PROCESS: Commander and Chief

Coordinate the boat's operating modes by enabling or disabling autonomous planning and control processes, while allowing direct rudder and sail-tail control.

* Inputs: None
* Stored: None
* Outputs: 
    * Toggle PROCESS: Live Tack Planning (LTP)
    * Toggle PID: Heading Control
    * Direct drive rudder
    * Toggle PROCESS: Live Sail Planning (LSP)
    * Toggle PID: Sail Control
    * Direct drive tail sail

[Claude]: "Inputs: None" is not right, since this process must receive mode commands from CM4 and status from the other tasks (battery low, link lost, sensor stale). List them.

[Claude]: Write this as an explicit state machine, e.g. MANUAL, AUTO, SEMI_RUDDER_MANUAL, SEMI_SAIL_MANUAL, FAILSAFE. For each state, list which tasks are enabled and which source feeds each arbiter. Also define precedence (Tack sequence > failsafe > direct drive > PID) and the boot default (servos neutral, autonomy off).

[Claude]: Failsafe triggers to define: LoRa silent for N seconds, GPS invalid while AUTO, IMU stale while AUTO, battery below a threshold, empty or invalid NVS when AUTO is requested. Direct drive should require a repeating command (dead-man timeout), so a lost link cannot leave a servo held at a manual value.

[Claude]: Specify what "Stop Autonomous" does (hold last servo positions, return to neutral, or go to a safe sail position). "Toggle" is also ambiguous; prefer explicit enable/disable so a repeated command is harmless.

<!---->
<!-- Sensors -->

### POLLING: Wind Vane

Read the wind vane at a regular interval and make the latest wind measurement available to the control processes.

* Inputs: None
* Stored: None
* Outputs:
    * Wind Vane Data (WVData)

* RTOS Priority:
* Frequency:
* Window:
* Stale:

[Claude]: Suggested meaning of the fields so all five pollers fill them the same way: Frequency = sample rate, Window = max allowed jitter/time between samples, Stale = age after which the data must be treated as invalid. Include a valid flag and timestamp in WVData.
[Corbin]: Discussed in earlier recommendations. The defenitions it uses are wrong though
    - Frequency = sample rate in Hz
    - Window = the number of samples to average
    - Stale = age in seconds after which the data must be treated as invalid

[Claude]: State whether the vane reports apparent wind relative to the bow (it will, physically), its zero offset, and its sign convention. Consider averaging or low-pass filtering, since vane readings are noisy and LSP/LTP should not react to gusts. Also note that a wind angle taken while the boat is turning is not the same as one taken in steady flight.
[Corbin]: Wind is relative to the boat in a -180 to 180 range, with 0 being straight ahead, positive to starboard, and negative to port. As for the filtering that is what the window is for.

### POLLING: GPS

Read the GPS receiver at a regular interval and make the boat's latest position available to navigation.

* Inputs: None
* Stored: None
* Outputs:
    * GPS Data (GPSData)

* RTOS Priority:
* Frequency:
* Window:
* Stale:

[Claude]: GPSData needs more than position: fix status/quality, satellite count or HDOP, speed over ground and course over ground. LTP needs the validity information to know when to trust the position, and course/speed over ground are useful as a check on heading and leeway.
[Corbin]: All this info will be in the struct...I think that a valid flag will be sufficent for the LTP and other functions though.

[Claude]: Pick a unit and datum for position (e.g. degrees * 1e7 as int32) to keep the telemetry record small and avoid float-precision surprises at lat/lon scale. Prefer UART + DMA with idle-line detection over per-byte polling.
[Corbin]: On chip we are going to store the float but in the telemetry we send we will probably optimize for size and use fixed-point representation.

### POLLING: IMU

Read the inertial measurement unit at a regular interval and make the boat's latest orientation data available to heading control.

* Inputs: None
* Stored: None
* Outputs:
    * IMU Data (IMUData)

* RTOS Priority:
* Frequency:
* Window:
* Stale:

[Claude]: Decide what Heading Control needs from IMUData: a fused heading (yaw) in the agreed frame, plus yaw rate if you want a D term. If the IMU only gives raw magnetometer/accelerometer data, the fusion and tilt compensation (the boat heels) must live somewhere, probably here.
[Corbin]: The IMU has a full internal fusion algorithm. We need the following data:
    - Yaw
    - Pitch
    - Roll
    - Global Heading Direction (i.e. relative to true north)

[Claude]: Magnetometers are disturbed by the servos and wiring (current draw). Plan a calibration step (hard/soft-iron) and mounting position away from the servos, and consider declination if using magnetic north. The old softwareplan.txt called this a magnetometer; confirm which sensor this actually is.
[Corbin]: Already discussed outside of this file.

### POLLING: Encoder

Read the sail encoder at a regular interval and make the latest sail-position measurement available to sail control.

* Inputs: None
* Stored: None
* Outputs:
    * Encoder Data (EData)

* RTOS Priority:
* Frequency:
* Window:
* Stale:

[Claude]: Define the encoder zero (sail aligned with the hull centerline), the sign (positive to starboard), and wraparound handling if it is absolute, or how it is homed on boot if it is incremental. State the resolution (the old plan said 2 bytes) and the angle it reports (sail relative to the hull, not to the wind).
[Corbin]: Sail angle is relative to the center line of the boat, with positive angles to starboard and negative angles to port (Same as wind). Precision will have to be int16 since there are 360 possible degrees of freedom. I think there is a way to get even hight precision from the wind sensor but I don't know that we need more then 1 degree of precision.

### POLLING: Battery Monitor

Read the battery monitor at a regular interval and make the latest battery status available to telemetry and monitoring.

* Inputs: None
* Stored: None
* Outputs:
    * Battery Monitor Data (BMData)

* RTOS Priority:
* Frequency:
* Window:
* Stale:

[Claude]: BMData feeds the failsafe logic, not just telemetry. Define low and critical thresholds (and their hysteresis) and who reacts to them (Commander and Chief). Servo stall and motor current draw will sag the battery voltage, so filter or use a debounce before triggering.
[Corbin]: Commander and Chief will handle putting the boat into a low power mode where we just hold the servos in their respective 0 positions. Beyond that the boat isn't actually going to have any more complex power management.

<!---->
<!-- Telemetry -->

### PROCESS: Save Telemetry Data

Collect the latest sensor readings into a telemetry record and save it in SRAM so the communications processor can transmit it.

* Inputs:
    * WVData
    * GPSData
    * IMUData
    * EData
    * BMData
* Stored: None
* Outputs:
    * SRAM Saved Telemetry Data

* RTOS Priority:

[Claude]: Define a fixed-size, packed, versioned telemetry struct in Common/Inc with a sequence counter, and a documented ownership rule for the SRAM region. CM7 writes, CM4 reads, so use a double buffer with an index swap, or an HSEM lock around the copy, to avoid CM4 reading a half-written record. Put the buffer in SRAM4 (the domain both cores can reach), and check the MPU/cache settings there, as D-cache on CM7 can hide writes from CM4.

[Claude]: The record should also include the control state: current mode, DHeading, DSail, RSAngle, STAngle, valid flags per sensor, and fault/failsafe flags. The GUI needs these (see the GUI note).

[Claude]: Goal 4.2 says "save locally", but this process only publishes to SRAM for CM4. If the boat itself must keep a log (SD card or flash) for later analysis, that needs its own task; otherwise clarify that "locally" means the ground station.

<!---->
<!-- Navigation -->

### PROCESS: Live Tack Planning (LTP)

Use the planned nautical vector series, current wind, and GPS position to determine the boat's desired heading along the route. Based on wind direction handle tacking/running and associated manuvers. Within Nautical Vectors the vector represents the ideal path and the margin represents the area in which the LTP can manuver for efficient sailing (i.e. tacking into the wind in a close hauled position).

* Inputs:
    * NVS
    * WVData
    * GPSData
* Stored: None
* Outputs:
    * Desired Heading (DHeading)

* RTOS Priority:

[Claude]: LTP also needs to output a tack request (direction) for the SEQUENCE: Tack, otherwise nothing triggers it. Add it as an output, and add the current mode or a "tack in progress" signal as an input so LTP does not issue a new heading while a tack is running.

[Claude]: Specify the no-go zone (angle off the wind where the boat cannot sail, roughly 40-50 degrees for a sail boat like this) and add hysteresis or a minimum time between tacks, to avoid chattering back and forth when the target bearing sits near the edge of the zone.

[Claude]: Define the waypoint-advance rule (arrival radius for each segment) and what LTP does when the boat leaves the margin corridor, when GPS is invalid, and when the NVS runs out. A margin defined as "area LTP can maneuver in" works, but LTP needs a decision rule: e.g. tack when cross-track error exceeds the margin.

[Claude]: NVS is written by CM4 over LoRa and read by LTP. Use a double buffer (receive into one, atomically switch on Close NVS) so LTP never sees a half-loaded route.

### PID: Heading Control

Compare the desired heading with the boat's measured orientation and adjust the rudder target to reduce the heading error.

* Inputs:
    * DHeading
    * IMUData
* Stored: None
* Outputs:
    * Rudder Servo Angle (RSAngle)

* RTOS Priority:
* Window:

[Claude]: The heading error must wrap into -180..180 before the P/I/D terms. Define anti-windup, output saturation (rudder limits), the derivative source (use the IMU yaw rate rather than differentiating a noisy heading), and reset the integrator whenever the task is enabled or the mode changes. On disable, the output should stop being published rather than keep the last value.

[Claude]: A rudder only steers when water flows over it. Consider a low-speed behaviour (limit integral growth, or reduce gain) so the PID does not wind the rudder hard over while the boat is nearly stopped, such as when it is in irons.

### ARBITER: Set Rudder Servo Angle

Apply the requested rudder angle by commanding the rudder servo to the corresponding position. Also handle safety limits, necessity limits, and smoothing transitions.

* Inputs:
    * RSAngle
* Stored: None
* Outputs:
    * Rudder Servo Position

[Claude]: The arbiter takes only RSAngle as input, but three sources can write the rudder: Heading Control, SEQUENCE: Tack and Direct Drive from Commander and Chief. Add the source inputs and a selected-source input from Commander and Chief, and state the priority. This is the single place that writes the PWM timer, so no other task should touch it.

[Claude]: Limits and smoothing need numbers: mechanical min/max in degrees (so the servo cannot stall against the linkage), a slew-rate limit in degrees per second, and the safe default position if every source is stale (probably neutral). Say whether the Tack sequence is allowed to bypass the slew limit, because a tack needs a fast rudder.

### SEQUENCE: Tack

Respond to a requested tack direction by performing a harsh turn through the wind to change the boat's global angle by at least 60 degrees (i.e -30 deg off wind turned into +30 deg).

* Inputs:
    * Direction of Tack
* Stored: None
* Outputs:
    * Rudder Servo Angle (RSAngle)
    * Sail Tail Servo Angle (STAngle)

[Claude]: A tack takes seconds, so this is a state machine run by a task (start turn, pass through the wind, settle on the new side), and needs more inputs than just the direction: WVData and IMUData to know when it is complete, plus the previous sail side. Define the exit condition as the wind having moved to the opposite side of the bow and heading having settled, not simply a 60-degree change; the heading change in a real tack is usually larger.

[Claude]: Define the abort/failure case: a boat that stalls in irons (no steerage, bow into the wind) needs a recovery behaviour or a timeout that hands control back. Also define what the sail tail does during the tack (release, center, or mirror to the new side).

[Claude]: While running, this sequence must be granted priority by the arbiters, and the PIDs should be paused and reset (see Heading Control). It also needs to report its state back to Commander and Chief.

<!---->
<!-- Sailing -->

### PROCESS: Live Sail Planning (LSP)

Use the measured wind angle to select a suitable sail angle for the current point of sail.

* Inputs:
    * WVData
* Stored:
    * Wind Angle Buckets: close hauled, beam reach, broad reach, running, etc.
* Outputs:
    * Desired Sail (DSail)

* RTOS Priority:

[Claude]: The inputs list only WVData, but a sail angle also depends on which side of the boat the wind is on (port or starboard), which the sign of the wind angle gives. State the output convention: DSail is relative to the hull or to the wind? softwareplan.txt had LTP output a desired sail/wind angle; now LSP derives it from wind alone. Confirm that this is intentional (the sail ignores the route).

[Claude]: Add hysteresis between the wind-angle buckets so the desired sail angle does not flip when the wind angle sits at a bucket boundary. Also decide whether to use apparent or true wind. Apparent wind from the vane is what the sail actually sees, which is usually the right input for sail trim.

### PID: Sail Control

Compare the desired sail angle with the encoder-measured sail position and adjust the sail-tail servo target to reduce the error.

* Inputs:
    * DSail
    * EData
* Stored: None
* Outputs:
    * Sail Tail Servo Angle (STAngle)

* RTOS Priority:
* Window:

[Claude]: This PID controls the tail, which only indirectly sets the sail angle (the tail sets the angle of attack and the sail then settles), so the plant is slow and nonlinear. Run it at a lower rate than Heading Control, add a deadband so the servo does not hunt, and tune with that lag in mind. The old plan mentioned saving servo current, which is another reason for the deadband.

[Claude]: The sign of the tail output flips with the side the wind is on (and during a tack), so the PID needs that as an input or the arbiter must handle the mirroring. Define which.

[Claude]: The same disable/reset, windup and saturation rules as Heading Control apply. If EData is stale, hold the last tail position rather than acting on old feedback.

### ARBITER: Set Sail Tail Servo Angle

Apply the requested sail-tail angle by commanding the sail-tail servo to the corresponding position. Also handle safety limits, necessity limits, and smoothing transitions.

* Inputs:
    * STAngle
* Stored: None
* Outputs:
    * Sail Tail Servo Position

[Claude]: The same source arbitration applies as for the rudder: Sail Control PID, SEQUENCE: Tack and Direct Drive from Commander and Chief can all write STAngle. Add the source inputs, priority and safe default. Add a current/stall consideration: limit continuous holding torque and rate so the servo does not overheat or brown out the supply during large moves.

<!---->

## CM4

[Claude]: CM4 does nothing but radio and the shared-memory interface, which is a good split. Keep it deterministic: do the LoRa transfer in a task and use ISRs only to set flags or hand off to DMA. Define a packet format with a start byte, type, length, sequence number and CRC, shared by CM4 and the ground station, and make sure CM4 does not need to wait on CM7 to stay responsive.

<!-- LoRa -->

### Receive LoRa Data

Receive ground-station commands over LoRa.

* Interupt on receive
* Commands
    * Start Autonomous
    * Stop Autonomous

    * Start Direct Drive Rudder
    * Direct Drive Rudder 
    * Stop Direct Drive Rudder

    * Start Direct Drive Tail Sail
    * Direct Drive Tail Sail
    * Stop Direct Drive Tail Sail

    * Load NVS
    * NVS
    * Close NVS

[Claude]: Keep the receive ISR minimal (read the radio IRQ, signal a task), and parse in a task. SPI transfers to the radio from an ISR can block other interrupts.

[Claude]: Missing commands: a periodic heartbeat/ping so CM7 can detect link loss, an emergency stop or return-to-safe command, and a status request. Mode commands should be acknowledged back to the ground station.

[Claude]: Direct Drive Rudder/Tail Sail should be repeated by the ground station and expire on CM7 if not refreshed (see Commander and Chief). "Start" and "Stop" messages alone can leave the servo stuck if "Stop" is lost.

[Claude]: NVS transfer: define chunk size, per-chunk index, total count and a CRC over the full set, and only commit the route on Close NVS once the CRC matches. Add a way to abort or restart. Reject Start Autonomous if no valid NVS is loaded.

[Claude]: Commands should reach CM7 through a small command queue in shared memory (type, payload, sequence) with a notification (HSEM or an interrupt) rather than by writing flags directly, so commands cannot be lost or half-read.

### Send LoRa Data

Transmit the latest telemetry record to the ground station over LoRa.

* Interupt on TIM
* Send telemetry data
    * Read SRAM saved Telemetry Data
    * Send Telemetry Data

[Claude]: Do not transmit from the timer ISR; have it set a flag and let a task send. LoRa transmission is slow (tens to hundreds of ms depending on spreading factor and payload size) and half-duplex, so the radio cannot receive commands while sending. Keep the telemetry record small and schedule TX around RX (e.g. fixed slots), or the 1 Hz telemetry will cause missed commands.

[Claude]: Check the regional duty-cycle limit for the LoRa band you use; at 1 Hz with a large payload you may exceed it, which would force a lower rate or a smaller record. Read the telemetry through the same double-buffer/HSEM scheme as the writer (see Save Telemetry Data), and detect an unchanged sequence number to skip stale records.

<!---->

<!------------------------------------------------------------------------>

# FAULT TABLE

LoRa Lost:
* Who detects?
* Response?

GPS Invalid:
* Detected by the GPS polling task
* After a set number of failed GPS readings, trigger a fault response 
* Fault response will be a notification sent via LoRa and a temporary return to home for the sail and rudder servos.

IMU Stale:
* Detected by the IMU polling task
* After a set number of failed IMU readings, trigger a fault response
* Fault response will be a notification sent via LoRa and a temporary return to home for the sail and rudder servos.

Wind Vane Stale:
* Detected by the wind vane polling task
* After a set number of failed wind vane readings, trigger a fault response
* Fault response will be a notification sent via LoRa and a temporary return to home for the sail and rudder servos.

Encoder Stale:
* Detected by the encoder polling task
* After a set number of failed encoder readings, trigger a fault response
* Fault response will be a notification sent via LoRa and a temporary fallback position for sail servo

Battery Low/Critical:
* Detected by the battery monitoring task
* Trigger a fault response when battery level falls below critical threshold
* Home sail and rudder servos then lock that position.

NVS Empty:

Tack Timeout:

<!------------------------------------------------------------------------>

# AI

[Claude]: Shared data layer. Put all cross-task and cross-core types in Common/Inc (WVData, GPSData, IMUData, EData, BMData, telemetry record, NVS segment, command packet), each with timestamp, valid flag and units documented in comments. Decide on fixed-point vs float up front.
[Corbin]: I will handle this for the sensor data structures and shared interpretation data such as what -10 deg refers to when talking about the sail or rudder.

[Claude]: Inter-core design. CM7 to CM4 is the telemetry buffer (SRAM4, double buffer or HSEM). CM4 to CM7 is the command queue (including NVS chunks). Specify the HSEM IDs, the memory map/linker placement for both cores, MPU and cache attributes for the shared region (non-cacheable or explicit clean/invalidate), and what each core does if the other has not booted. Check this against the dual-core boot sequence already in Common/Src.

[Claude]: Suggested starting rates (tune later): Heading PID 20 Hz, Sail PID 10 Hz, IMU 50 Hz, wind vane 10 Hz, encoder 20 Hz, GPS 1-5 Hz, battery 1 Hz, LSP 2-5 Hz, LTP 1-2 Hz, Save Telemetry 1-2 Hz. Give higher RTOS priority to the arbiters, Tack sequence and PIDs than to LTP, telemetry and polling of slow sensors. Make sure no task that holds a mutex or semaphore can block the arbiters.

[Claude]: Failsafe matrix. Write a table of fault vs response: LoRa lost, GPS invalid, IMU stale, wind vane stale, encoder stale, battery low/critical, NVS empty, Tack timeout. Each row names who detects it and what state Commander and Chief enters.
[Corbin]: I added a section above for this and filled out some of the fault cases, the list is not comprehensive yet.

[Claude]: Testing plan. Most of this cannot be tested on water until late. Add a simulation or hardware-in-the-loop mode (inject recorded or synthetic WVData/GPSData/IMUData and watch the outputs), unit-testable pure functions for LSP/LTP/PID, and a bench test of the servos with the arbiters and limits before the boat is put in the water. Do the first PID tuning with manual headings, as the schedule already suggests.

[Claude]: Housekeeping. Naming is inconsistent: "Tail Sail", "Sail Tail" and "Sail" are all used (e.g. "Direct Drive Tail Sail" vs "Sail Tail Servo Angle"), as are "Interupt" (CM4 sections) and "manuvers" (LTP). Choose one term per thing. The Set Rudder/Tail arbiters and the Tack sequence also need an owner for the interaction between them, which is not listed in the assignments.