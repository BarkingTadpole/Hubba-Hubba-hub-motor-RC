# Project Showcase Handoff

Last updated: 2026-08-26

This is the shared, concise source of truth for anyone building a project
showcase, portfolio page, demo script, or public summary for the RC car. Keep
it accurate as the firmware and physical validation progress. Detailed wiring,
equations, commands, and engineering reasoning belong in `README.md` and
`docs/powertrain_architecture_v1.md`, not here.

## Project Summary

This project is a custom ESP32 powertrain and vehicle-dynamics controller for
a 1/7 scale, four-motor RC car. Each wheel has an independent hub motor and
ESC output. The ESP32 reads the radio receiver, controls the steering-servo
signal, measures all four wheel speeds, reads a chassis IMU, receives Dragy
Lite GPS data, and can apply torque-vectoring corrections to the four motors.

The central project theme is turning a mechanically powerful RC platform into
a measured, configurable control system with strong safety fallbacks and
useful telemetry. Torque vectoring changes motor power only; it never changes
the driver's requested steering angle.

## Showcase-Ready Capabilities

- Four independent wheel-motor throttle and direction outputs.
- Selectable AWD, FWD, and RWD modes.
- Receiver-controlled throttle, steering, drive-enable state, and
  torque-vectoring mode.
- Calibrated steering relay with input-spike rejection, smoothing, trim, a
  measured servo-to-road-wheel curve, and speed-sensitive steering limits.
- Four hardware-counted RPM inputs with a confirmed seven rising edges per
  mechanical revolution on the tested motor/sensor combination.
- Chassis IMU with automatic stationary startup bias calibration.
- Straight-line and cornering torque-vectoring modes with equal-output sensor
  fallback.
- Dragy Lite GPS reception for position, speed, course, altitude, satellite
  count, and parser health.
- Read-only serial GPS diagnostics through `monitor gps` (`monitor dragy` is
  an alias), plus a bounded raw hex/ASCII capture for protocol diagnosis.
- Wi-Fi dashboard for live monitoring and guarded configuration through an
  ESP32-hosted SoftAP viewer. It deliberately exposes no remote driving or arming;
  its one-way maintenance disarm safely unlocks configuration and then requires
  a physical receiver STOP-to-RUN cycle before drive can rearm.
- Guided browser calibration for receiver throttle, steering, CH4, CH5, IMU
  bias, and guarded ESC endpoint/manual-relay workflows. It shows live steps
  and captured values while retaining the firmware's serial safety checks.
- Explicit NVS configuration reload, one-click AWD/FWD/RWD cycling, and a
  persistent choice between boot-latched and STOP-to-disarm receiver behavior.
- Offline, all-channel CSV recording in ESP32 flash, exportable after Wi-Fi
  reconnects.
- Browser-based post-run analysis for speed, RPM, receiver inputs, steering,
  yaw, ESC output, torque-vectoring correction, and acceleration.
- USB serial calibration, monitoring, diagnostics, and persistent NVS
  configuration.

## Hardware Snapshot

- Controller: ESP32 DOIT DevKit V1 / ESP32-WROOM-32.
- Vehicle: 1/7 scale RC chassis with four independently driven hub motors.
- Motors: four Hobbywing Skywalker 2820SL 550KV brushless motors.
- ESCs: four Hobbywing Skywalker 50A V2 units.
- Radio: Radiolink RC6GS V3 transmitter and R7FG receiver.
- Speed sensing: four Hobbywing phase-derived RPM sensors.
- Motion sensing: Adafruit ISM330DHCX IMU.
- GPS: Dragy Lite, using its u-blox MAX-M10S NMEA output.
- Steering: one servo controlled by a dedicated ESP32 PWM output and powered
  from a separate BEC.

The firmware uses all practical high-speed PWM channels and most available
GPIOs. The current wiring map is authoritative in `main/pin_config.h` and the
generated pin-map asset listed below.

## System Overview

```text
Radio receiver ──> validated PWM capture ──> steering + drive command
                                             │
Wheel RPM sensors ─┐                         v
Chassis IMU ───────┼─> vehicle state ──> torque vectoring ──> four ESCs
Dragy GPS ─────────┘          │
                              ├─> USB serial diagnostics
                              ├─> Wi-Fi live dashboard
                              └─> offline CSV log ──> browser analysis
```

The real-time powertrain loop runs separately from the low-priority Wi-Fi and
flash-logging work. Invalid or stale torque-vectoring sensors return the driven
wheels to balanced outputs instead of holding an old correction.
Receiver PWM edge capture remains available during internal-flash logging, so
flash-cache stalls cannot masquerade as plausible steering pulses. This fix is
firmware/build validated and still requires a traction-power-disconnected bench
check after flashing.

## Torque Vectoring in Plain Language

The controller compares the driver's steering request with measured yaw and
left-versus-right wheel speed. It can add power to one side and remove the
same amount from the other to help the car track straight or rotate through a
turn. Full cornering mode can also reduce both front-motor outputs as lateral
demand rises. Authority is deliberately limited and configurable.

Torque vectoring defaults to disabled. It requires current-boot IMU bias
calibration, valid wheel-speed data, receiver-mode calibration, and explicit
enablement. Reverse always uses balanced output.

## Steering and Vehicle Measurement

The steering model uses a measured 45-point table rather than assuming servo
angle equals road-wheel angle. Linear interpolation provides separate left and
right wheel angles for control and telemetry. A configurable NVS trim centers
the real vehicle. Receiver jitter is filtered across the full range, and the
maximum steering angle decreases with speed according to a lateral-acceleration
limit.

Wheel speed is measured independently at all four corners. The confirmed
conversion is seven rising edges per mechanical revolution for the tested
motor/sensor. The IMU supplies yaw rate and acceleration, while Dragy provides
an independent GPS speed and position source for logging and comparison.

## Dashboard and Data Story

The browser dashboard connects directly to the ESP32-hosted HTTP API and
reports fresh on-device telemetry. It shows receiver channels,
wheel RPM, steering geometry, IMU, GPS, torque vectoring, ESC output, logging
capacity, and non-secret configuration.

The ESP32 records a 128-column CSV at 1 Hz by default whether Wi-Fi is
available or not. Its dedicated raw flash partition is 2.75 MiB. Recording
stops when full instead of overwriting old runs. Representative retention is
roughly 60-80 minutes at 1 Hz; the dashboard calculates a live estimate from
the observed row size. Long or indefinite sessions will require microSD or
larger external storage.

The dashboard Configuration section can repopulate itself from the ESP32's
NVS-backed runtime configuration. It directly selects or cycles AWD/FWD/RWD,
controls the persistent arm-latch policy, and adjusts logging from 1-50 Hz.
Writes are DISARMED-only; readback is available while armed. Accepted settings
persist on the ESP32 and remain authoritative without Wi-Fi.

The Calibration console mirrors the project-specific serial setup flows rather
than exposing an arbitrary remote terminal. Receiver positions are averaged
for 1.5 seconds and saved only after a complete validated three-step workflow;
IMU retry is a two-second current-boot bias measurement. High-risk ESC endpoint
and bounded manual-relay actions have explicit confirmations and retain the
controller's neutral, CH4 STOP, signal-loss, safe-output, and timeout guards.

The recorder stops on a FAT write/flush fault and exposes the error instead of
continuing unreliable writes. Its browser export can salvage the committed CSV
prefix even when the writer's final flush fails. A header-only, zero-free-space
volume is identified as a storage condition rather than a malformed CSV, and
the DISARMED-only recovery action can rebuild the dedicated logging partition.
Destructive recovery remains separate and should happen only after export.

## Safety and Responsible Claims

The system establishes safe ESC PWM before higher-level initialization.
Receiver faults, invalid inputs, missed timing, and unavailable TV sensors are
designed to produce safe or balanced output. Direction changes pass through
minimum throttle, reverse power is limited by default, and Wi-Fi cannot send
drive, steering, arm, ordinary disarm, monitor, or arbitrary terminal commands.
It exposes only structured guarded calibration, plus a confirmed one-way
**Disarm for config** action that commands safe output, clears the armed state,
and still requires physical rearming. ESC calibration/manual relay is the only
browser path that deliberately changes motor signal output, and it retains the
same firmware prerequisites as serial.

The receiver drive-enable is a software control, not a physical emergency
stop. Permanent boot latching is the default but can be disabled persistently
so STOP and receiver safety faults fully disarm and require a fresh arm cycle.
Powered testing still requires immediately accessible traction-power
isolation, wheels clear of the ground for first-power tests, and a common
electrical ground.

Do not describe the project as autonomous, production-safe, road-legal, or
fully vehicle-validated. It is an advanced experimental RC control platform.

## Current Validation Status

Implemented and verified in code/build or simulation:

- Modular ESP-IDF firmware and custom flash partition table build successfully.
- Current firmware size is `0xfbc70`, leaving 21 percent free in the 1.25 MiB
  application partition.
- The ESP32 now creates its own WPA2 SoftAP and embeds/serves the complete
  dashboard and guarded HTTP API at `192.168.4.1`; no hotspot, upstream DHCP,
  UDP discovery, TCP telemetry bridge, or Python runtime is required.
- The expanded live telemetry schema formats into a dedicated static 8 KiB
  buffer outside the HTTP task stack.
- Host logic tests cover steering geometry/filtering, cornering limits,
  drivetrain selection, torque-vectoring behavior, command guarding, and GPS
  parsing.
- Browser/mock tests cover telemetry rendering, configuration and calibration
  command guarding, CSV export/loading, and analysis plots. Source invariants
  cover default SoftAP/HTTP hosting and the embedded assets.
- One direct oscilloscope check confirmed seven RPM rising edges per revolution.

Still requiring physical validation or tuning:

- Complete four-wheel RPM accuracy against an optical tachometer.
- On-car IMU sign, vibration behavior, and bias stability.
- Full receiver/servo/ESC signal validation on the current wiring.
- Guided browser calibration on the real receiver, IMU, and scoped ESC outputs,
  including cancellation and both workflow timeouts.
- Worst-case control timing, supply stability, reset behavior, and flash-log
  latency with Wi-Fi active.
- SoftAP join, viewer access, reconnect, and simultaneous access from several
  real client devices.
- Torque-vectoring tuning and repeatable road-test evidence.
- Long-run offline logging, full-partition behavior, and sudden-power-loss
  characterization.
- Dragy GPS protocol validation: live hardware receives UART bytes at the
  configured 9600 baud but currently decodes zero valid NMEA sentences. A
  bounded raw UART monitor is implemented for the next flash/bench session.
- Dragy compass heading is intentionally deferred; the compass protocol is not
  publicly documented and firmware currently performs device discovery only.
- Exact ESC SKU and permitted traction-battery voltage confirmation.

Keep code/build/simulation results distinct from physical bench or driving
evidence on the showcase.

## Media and Copy-Ready Assets

All paths below are on the shared development machine. Copy generated assets
into the showcase repository rather than linking the site directly to this
workspace.

| Asset | Copy-ready absolute path | Notes |
|---|---|---|
| Generated ESP32 pin map | `C:\Users\aeara\rc_car\build\esp32_pin_map.svg` | Primary wiring visual; generated from the live pin configuration. |
| Pin-map board background | `C:\Users\aeara\rc_car\tools\esp32_pin_map\esp32_board_minimal.png` | Source visual used by the generator; normally the SVG is the better showcase asset. |

Regenerate the pin map from the repository root with:

```powershell
.\tools\esp32_pin_map\open_pin_map.ps1
```

or as part of a normal `idf.py build`. The generator writes only into
`build/`; do not edit the SVG manually.

There are currently no checked-in vehicle photos, drive videos, oscilloscope
captures, dashboard screenshots, or analysis screenshots. Add every future
showcase-worthy media file to this table with a short description, date, and
whether it is measured hardware evidence or a simulation.

## Useful Source Paths for the Showcase Agent

- Project overview and setup: `C:\Users\aeara\rc_car\README.md`
- Detailed architecture and operating model:
  `C:\Users\aeara\rc_car\docs\powertrain_architecture_v1.md`
- Engineering review and evidence boundaries:
  `C:\Users\aeara\rc_car\docs\engineering_review_2026-08-10.md`
- Current physical pin assignments:
  `C:\Users\aeara\rc_car\main\pin_config.h`
- Dashboard implementation:
  `C:\Users\aeara\rc_car\tools\wifi_bridge\web`
- Dashboard/AP operating notes and legacy mock bridge:
  `C:\Users\aeara\rc_car\tools\wifi_bridge\README.md`

## Maintenance Rule

Update this handoff whenever a change affects public project facts, visible
features, hardware, validation status, performance/retention figures, safety
claims, or available media. Keep it concise and showcase-oriented. Do not add
Wi-Fi passwords, private identifiers, exhaustive pin tables, tuning dumps,
full command references, or claims that exceed the available evidence.
