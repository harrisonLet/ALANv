# Ground station ↔ boat link: what the ground station must implement

This is the boat firmware on `field_test_fixes`, as of commit `f11b8fd`. The ground station work (`gs_term` and the SAMD21 bridge) happens on its own branch and must match everything below.

## Why it changed
At the Nov 18 field test:
- **Commands were one-shot events with no retry.** A lost or corrupted packet was a lost command.
- **The boat transmitted on its own 1 s timer.** The GS had to guess when the boat was transmitting and couldn't hear commands.

Now:
- **The GS sends its whole desired state.** A lost packet is replaced by the next one.
- **The GS polls once a second and the boat answers immediately.** The two radios never transmit at the same time.
- **The link uses LoRa coding rate 4/7 with the radio's CRC.** 4/7 corrects errors; the CRC catches anything left over.

## 1. Radio settings (bridge firmware, `src/main.cpp`)
Both radios use the same settings:

| Setting | Value | Bridge call |
|---|---|---|
| Frequency | 915 MHz | `LoRa.begin(915E6)` |
| Spreading factor | SF8 | `LoRa.setSpreadingFactor(8)` |
| Bandwidth | 250 kHz | `LoRa.setSignalBandwidth(250E3)` |
| Coding rate | **4/7** | `LoRa.setCodingRate4(7)` **(new)** |
| Payload CRC | on | `LoRa.enableCrc()` |
| Header | explicit (the default) | |
| Preamble / sync word | 8 / 0x12 (the defaults) | |

- **The boat drops every received packet that has no CRC.** It counts them in `rx_crc_err`. A bridge without `enableCrc()` gets no commands through.
- **The coding rate is chosen per sender.** Each packet's header carries the rate, and the receiver follows it. A bridge left at 4/5 still works, but its commands get no error correction.

## 2. Uplink: GS → boat
Every uplink packet starts with a 5-byte header, then the payload. Everything is little-endian.

| Byte | Field | Value |
|---|---|---|
| 0 | dest | `0xBB` (boat) |
| 1 | src | `0xAA` (ground station) |
| 2 | seq | 0–255. Bump it on every packet. |
| 3 | len | 1 + payload length |
| 4 | type | command type (`CommandId_e`, `Common/Inc/sharedCommand.h`) |
| 5… | payload | 0–16 bytes |

The boat rejects a packet (counted in `rx_rejected`) if:
- the total length isn't `4 + len`, or
- dest isn't `0xBB`, or
- len is 0 or more than 17.

### `COMMAND_STATE` (type `0x40`): the GS's desired state
The payload is 10 bytes, `ipc_cmd_state_t` in `Common/Inc/ipc.h`:

| Payload byte | Field | Type | Meaning | KEEP value |
|---|---|---|---|---|
| 0 | flags | u8 | bit 0 = **POLL** (reply with telemetry); bits 4–7 = **mode request number** | |
| 1 | mode | u8 | `system_mode_t`, 0x00–0x0A | `0xFF` = no mode request |
| 2–3 | rudder_deg | i16 | rudder servo angle, used while the rudder is manual, −90…90 (+ = starboard) | `0x7FFF` |
| 4–5 | sail_deg | i16 | tail servo angle, used while the sail is manual, −90…90 | `0x7FFF` |
| 6–7 | heading | i16 | heading PID target, used while the rudder is semi, 0…359 | `0x7FFF` |
| 8–9 | sail_target | i16 | main sail PID target, used while the sail is semi, −180…180 | `0x7FFF` |

The whole packet is 15 bytes. Example: a poll carrying mode request #3 for FULL_MAN, with the rudder at +20 and everything else KEEP:
```
BB AA 2A 0B 40 31 01 14 00 FF 7F FF 7F FF 7F
               |  |  rudder=20 sail   heading sail_target
               |  mode FULL_MAN (0x01)
               flags 0x31 = request #3 (0x30) | POLL (0x01)
```

### How the boat applies a state packet
1. **It checks every field first.** If anything is out of range or the mode is invalid, the whole packet is rejected and nothing changes. The result is BAD_VALUE or UNSUPPORTED.
2. **It applies the mode once per mode request number, not on every packet.**
   - If the number differs from the last one applied, the boat switches to the mode.
   - Resends with the same number don't re-apply it. That way a SAFE the boat chose itself (LTP at the end of a route, or outside its corridor) isn't undone by the next resend.
   - A packet with mode `0xFF` makes the boat forget the last number, so the next request applies whatever its number is.
   - After a reboot, the first request applies too.
3. **It applies only the setpoints the boat's current mode uses:**
   - rudder manual → `rudder_deg`
   - rudder semi → `heading`
   - sail manual → `sail_deg`
   - sail semi → `sail_target`

   Everything else is ignored silently: no WRONG_MODE, and AUTO is never overridden. An unchanged value changes nothing: the servo doesn't move, and the PID target isn't set again, so the PID doesn't restart.
4. **The result goes into telemetry** as `last_cmd_seq` and `last_cmd_result`: OK, BAD_VALUE or UNSUPPORTED.

### Other command types
These still work, for bench use, `raw`, and routes:
- `0x00` heartbeat (no payload)
- `0x10–0x1A` one-shot mode commands
- `0x21–0x24` one-shot setpoints (i16)
- `0x31–0x33` route load

The GS shouldn't use the one-shot mode and setpoint commands any more; use `COMMAND_STATE`. Routes are unchanged; acknowledge-and-retry for them comes later.

## 3. Downlink: boat → GS
| Byte | Field | Value |
|---|---|---|
| 0 | dest | `0xAA` |
| 1 | src | `0xBB` |
| 2 | seq | the boat's telemetry counter, +1 on every send |
| 3 | len | `0x4D` (1 + 76) |
| 4 | flags | bit 0 STALE (the CM7 hasn't published new data; values are frozen), bit 1 BUSY_FALLBACK, bit 2 NO_IPC (the CM4 never attached to the CM7, so commands are dropped), **bit 3 UNSOLICITED (new)** |
| 5…80 | telemetry | `LoRaTelemetry_t`, 76 bytes |

- **The layout** is `Common/Inc/loraTelemetry.h`. It's identical to `ground_station/terminal/telemetry.h` on `origin/SAMD21_LoRa` (it has no `sog`/`cog`).
- **`rx_crc_err`** now counts packets with a bad *or missing* CRC.
- **UNSOLICITED** means the packet was sent on the boat's own 1 s timer because no poll arrived for 3 s. Without it, the packet is a reply to a poll.

## 4. Timing (`gs_term`)
| Rule | Value |
|---|---|
| Poll: a `COMMAND_STATE` with POLL set | every **1.0 s** |
| After a poll, hold all sends until the telemetry reply arrives, or this long | **0.35 s** |
| State changes (mode commands, `rsang`/`ssang`/`head`/`sang`, jog, `safe`): send immediately with POLL clear, unless inside that hold | right away |
| Minimum gap between any two sends (the bridge glues bytes written within 10 ms into one packet) | **0.15 s** |
| Jog send rate | at most 4 Hz (existing `JOG_PERIOD_S`) |

**Airtime at SF8 / 250 kHz / CR 4/7:**

| Packet | Time on air |
|---|---|
| state packet (15 B) | 56.6 ms |
| telemetry (81 B) | 171.3 ms |
| poll + boat turnaround (~5 ms) + reply | ≈ 235 ms |

- **What the operator sees:** a manual change is delayed by about 0.25 s at most, and only if it lands in the hold window (about a quarter of each second).
- **Merging updates:** a state change made during the hold just marks the state dirty, and the latest state goes out when the window closes, so no queue is needed.
- **When to guess the boat's timing:** use the old guess (`TELEM_DEAD_S`) only while the last telemetry was UNSOLICITED, because then the boat is on its own timer.
- **`hb on|off`** becomes "polling on/off". With polling off, the boat falls back to its 1 s timer after 3 s.

## 5. Desired state on the GS
- **Start with every field at KEEP and mode `0xFF`.** Starting or restarting `gs_term` must never move the boat. Polls still go out, so telemetry flows.
- **Every operator mode command bumps the mode request number** (mod 16), even a repeat of the current mode. That's how you re-ask for AUTO after the boat put itself in SAFE.
- **When a mode command switches an actuator between manual and semi, reset that actuator's setpoints to KEEP.** Otherwise an old manual angle would be re-applied the next time it goes manual. On entering manual, the boat already holds the current angle.
- **`safe`/`kill`** = a new mode request for SAFE, sent immediately.

## 6. Feedback and display
- **Uplink heard:** the telemetry's `rx_last_seq` is the seq of the last GS packet the boat received, polls included. A reply proves its poll arrived.
- **Applied:** `last_cmd_seq` / `last_cmd_result` belong to the last packet the CM7 executed. Because state packets go out at least once a second, this updates every second.
- **Desired vs actual:**
  - mode → `mode`
  - rudder → `rudder_servo_angle`
  - sail → `sail_servo_angle`
  - heading target → `desired_heading`

  Show these green when they match what was asked for.
- **Link quality**, both directions, as percentages:
  - **Uplink** = the change in `rx_ok` (packets the boat accepted) ÷ packets the GS sent, over the last 20 s.
  - **Downlink** = telemetry received ÷ the span of the boat's telemetry seq, over the last 100 packets.
- **Speed and course** come from the last two GPS fixes (lat/lon), with the time between them taken from the boat's `uptime_ms`, not GS receive times. Only show them when 0.5 s < dt ≤ 3 s and both packets have a fix. They lag by up to a second.
- **Flags:** show STALE, NO_IPC and UNSOLICITED on screen, not just in the log.

## 7. Bench checks once the GS side is done
- Poll TX → telemetry RX within about 250 ms in the `gs_term` log, with the reply not marked UNSOLICITED.
- A jog change goes out immediately when outside the hold window.
- Quit `gs_term`: after 3 s, telemetry arrives once a second marked UNSOLICITED.
- Restart `gs_term`: the boat's mode doesn't change until you send a mode command.
- Power-cycle the boat: your mode and setpoints are back within 1–2 s.
- A bridge built without `enableCrc()`: no command is applied and `rx_crc_err` climbs.
- Disconnect the bridge antenna: uplink link quality drops before downlink.
