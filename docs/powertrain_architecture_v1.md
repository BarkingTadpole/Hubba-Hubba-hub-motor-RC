# Independent-steering powertrain architecture

Updated 2026-10-23 for `experimental/independent-steering`. This describes
implemented firmware and planned wiring. Host tests and builds do not establish
physical validation of the new steering system.

## Firmware ownership

`powertrain_controller.c` owns motor state, receiver arming/failsafes, throttle
ramps, the direction interlock, initialization and periodic scheduling.
`powertrain_steering.c` owns CH1 filtering, held rear speed and servo commands.
`powertrain_calibration.c` owns serial/browser receiver captures and ESC
calibration/manual relay. `powertrain_settings.c` owns guarded setting changes
and save requests; `powertrain_diagnostics.c` owns serial status/help/monitors.
`powertrain_commands.c` dispatches parsed operations for both the serial and
HTTP interfaces. Both use `remote_command.c` for configuration parsing;
serial receiver calibration retains its interactive prompts. Shared helpers
define receiver calibration validation, PWM conversion and system state names.

Public callers continue to use `include/powertrain_controller.h`. The private
`powertrain_internal.h` exposes only shared context and cross-module helpers.
The recursive controller mutex guards mutable context and calibration/steering
state; timing counters use a separate short critical section. Blocking receiver
sampling releases the controller mutex while calibration guards remain active.
The 20 ms control loop still updates steering, checks browser calibration
timeout, handles CH4, updates armed drive outputs, then handles ESC calibration.
Torque vectoring has no module, runtime state, configuration or command path.

## Hardware and pin authority

Controller: ESP32 DOIT DevKit V1 / ESP32-WROOM-32, ESP-IDF 6.x (`esp32`).
Four Hobbywing Skywalker 2820SL 550KV motors (12N14P, seven pole pairs) and
four Skywalker 50A V2 ESCs remain. Confirm the exact ESC SKU before selecting
traction-battery voltage; regular and 6S variants may differ. Transmitter is
Radiolink RC6GS V3, receiver R7FG. Two HW86060041 phase RPM sensors remain on
the rear motors. Each front wheel now has its own PWM steering servo.

`main/pin_config.h` is authoritative. Update the wiring table below whenever
the pin assignments change.

| Connection | GPIO | Peripheral |
|---|---:|---|
| Conditioned R7FG CH1 | 16 | GPIO edge input |
| Conditioned R7FG CH2 | 25 | GPIO edge input |
| Conditioned R7FG CH4 | 33 | GPIO edge input |
| Left steering servo | 32 | Low-speed LEDC channel 0 |
| Right steering servo | 26 | Low-speed LEDC channel 1 |
| FL throttle / reverse | 13 / 2 | High-speed LEDC 0 / 4 |
| FR throttle / reverse | 14 / 27 | High-speed LEDC 1 / 5 |
| RL throttle / reverse | 21 / 19 | High-speed LEDC 2 / 6 |
| RR throttle / reverse | 18 / 17 | High-speed LEDC 3 / 7 |
| Rear-left RPM | 36 | PCNT rising-edge input |
| Rear-right RPM | 39 | PCNT rising-edge input |

The prior CH5 wire must be disconnected from GPIO26. Removed inputs include
front RPM GPIO34/35, I2C GPIO23/22, and UART1 GPIO4/5; firmware no longer
initializes those peripherals. GPIO33 remains PWM CH4, with no internal pull-up
or obsolete digital arm-switch interpretation. GPIO36/39 are input-only with
no internal pull-ups. GPIO2 remains an ESC output and boot strap, never a
status LED. Add no pull to GPIO2; investigate ESC-connected boot reliability.
All ten outputs use 50 Hz PWM. The two LEDC groups are separate hardware.
ESC initialization installs the IDF fade service before thread-safe updates.

Use common receiver, servo supply, ESP32, sensor, and ESC signal grounds.
GPIO inputs must not exceed 3.3 V. Condition CH1/CH2/CH4 and RPM outputs as
needed. Receiver DC-average measurements are not logic-high measurements;
verify pulses with a scope. GPIO16 only receives CH1; GPIO32/26 only drive
their respective servos. Verify servo 3.3 V logic compatibility or add buffers.
Power servos from a BEC or regulated supply, never GPIOs. Do not parallel ESC
BEC red wires without manufacturer approval. Keep the R7FG gyro disabled
initially so receiver correction does not obscure steering setup.

HW86060041 phase wires connect to any two motor phases. Planned sensor supply
is regulated 5 V; black goes to common ground, white to a conditioned GPIO.
The proposed divider is white → 1 kOhm → GPIO junction → 2 kOhm → ground,
about 3.33 V from exactly 5 V. Its opposite orientation is incorrect; a higher
rail can exceed GPIO margin. A 10 kOhm / 15 kOhm divider gives about 3.0 V at
5 V with less loading. Verify the actual waveform and rail. Sensor documents
conflict on minimum supply (3.3 V wiring text versus 3.5 V spec), so ESP32 3V3
is not the preferred final sensor supply. Rear-channel wiring/accuracy remain
bench pending.

## Steering signal path and geometry

CH1 capture accepts 800–2200 us, times out after 100 ms, and uses an IRAM-safe
priority-3 GPIO ISR on CPU0. It reads GPIO registers directly so internal-flash
cache stalls do not defer edges. Drive/steering updates run on CPU1 every
20 ms. Preserve this GPIO scheme; earlier MCPWM/RMT experiments were rejected
following channel starvation and an unsafe bench incident.

The distinct-frame median rejects isolated spikes; repeated reads of the same
timestamp cannot warm it. Three fresh frames are required after startup or
signal-loss reset. A first-order command filter follows, default 10 ms,
configurable 0–500 ms. CH1 loss, filter warmup, and a matching enabled throttle
failsafe center both servos immediately, bypassing smoothing. Normal steering
continues while drive is disarmed. CH4 inhibits motors, not steering.

CH1 calibration stores left/center/right/deadband independently of servo
endpoints. Defaults are 2050/1518/987 us and 30 us deadband. Both guided capture
and manual numeric settings are available. Manual pulses must bracket center
by at least 150 us within 800–2200 us. Deadband is 0–150 us and must be smaller
than both spans. Guided recalibration preserves the selected deadband. After
filtering, receiver travel is normalized by its direction-specific span to
[-1,1], positive right.

### Geometric Ackermann and calibration

Each servo stores left/center/right PWM in 800–2200 us. Center must be between
the endpoints, each at least 50 us away. Endpoint order handles reversal.
Each servo also stores positive measured road-wheel angles at its left/right
endpoints, 1–85 degrees at 0.1-degree precision. Center means straight ahead,
zero degrees; its PWM provides individual trim. Mapping angle to PWM is linear
on each side of center. Defaults 2000/1500/1000 us and 30/30 degrees are
unverified starting assumptions. The model does not compensate for nonlinear
linkage travel, compliance or tire slip; intermediate angles need measurement.
Displayed angle targets are not sensor feedback.

Geometry stores wheelbase/front track (50–2000 mm), rear-wheel diameter
(10–500 mm), and maximum requested left/right angles (1–85 degrees), all at
0.1-unit precision. Defaults are the user's earlier 445/320/107 mm dimensions
and 30/30-degree requests. A maximum request must not exceed the corresponding
calibrated endpoint angle on either servo. Invalid combinations are rejected,
so output clipping does not silently break the angle relationship. Lower a
request limit first when reducing calibrated travel. This static travel bound
is separate from the removed lateral-g/speed steering limiter.

For normalized CH1 magnitude `u`, the inside target `di` is `u * max_angle`
for the requested turn direction. Let `L` be wheelbase and `T` front track:

```text
di = abs(command) * direction_specific_max_angle
ideal_outside = atan2(L * sin(di), L * cos(di) + T * sin(di))
outside = di + (ideal_outside - di) * strength / 100
```

Angles in the trigonometric calculation use radians. At 100%,
`cot(outside) - cot(inside) = T/L`: both wheels imply the same rear-axle turn
center under the ideal rolling assumption. At 0%, targets are equal/parallel.
At intermediate strengths, blend the outside angle while holding the inside
at the driver's request. Geometry therefore responds to steering magnitude
as well as speed. Left turns select the left inside wheel; right turns select
the right. At center, both PWM centers are exact. Reverse uses the same mapping.
Neither an IMU nor torque vectoring is involved.

Reference: [SAE, Analysis of Ackermann Steering Geometry](https://doi.org/10.4271/2006-01-3638).
This defines a geometric target, not optimal high-speed tire behavior.

### Speed curve and persistent configuration

Store 2–8 `(speed_kmh, strength_percent)` points with 0.1-unit precision.
First speed is zero; subsequent speeds strictly increase up to 300 km/h.
Strength is 0–100%. Linear interpolation connects points; above the last
speed, hold the final strength. Defaults are (0,0)/(100,0), parallel throughout.
Saving replaces the entire curve in one operation. Negative reductions and
anti-Ackermann from the previous PWM model are no longer part of this curve.

RC Monitor exposes all steering parameters: CH1 calibration/deadband,
smoothing, both servo PWM/angle calibrations, dimensions, request limits,
RPM PPR, and point table/preview. All config remains DISARMED-only and blocked
during calibration. Invalid values are rejected by UI, parser, controller and
NVS validation. Draft points survive polling, navigation and device rotation.

NVS uses the validated 66-byte, fixed-width version-2 `cfg/dual_steer_v2` blob.
On a device with only the old 48-byte v1 blob, valid servo PWM endpoints are
preserved but the old reduction curve resets to 0% parallel. No other v1
values become geometry/angle calibration. New parameters start at defaults.
A corrupt v2 blob falls back to defaults, never resurrects stale v1 values.
Save failures preserve active RAM values. Receiver calibration and active
legacy drive keys remain compatible; no NVS erase is needed.

## Rear speed estimate

Motor speed equals wheel speed for these hub motors:

```text
mechanical_rpm = rising_edge_frequency_hz * 60 / PPR
vehicle_mps = mean(rear_left_rpm, rear_right_rpm) * pi * configured_wheel_diameter_m / 60
```

Wheel diameter defaults to 107 mm and is configurable in steering geometry.
Default PPR is seven, confirmed by the user's one-revolution scope edge count
on 2026-08-12. Rear channels still need waveform and optical-tachometer checks
across speed. Sampling stays 20 ms, with a rolling window of 5–25 samples
(about 100–500 ms), extending until four edges are available. A channel is
invalid after 250 ms without an edge. Snapshot age must also be <=100 ms.
Both rear channels must be valid and finite/nonnegative for a fresh estimate.
Counting quantization is about 85.7 RPM at 100 ms and 17.1 RPM at 500 ms.

While armed, retain the last valid speed if either channel becomes invalid
or the sensor task stops updating; publish `speed_valid=false, speed_held=true`.
This preserves the prior armed dropout policy. Disarmed missing RPM resets
speed to zero/unavailable. Before the first valid measurement, use the curve's
0 km/h point. Since no edges cannot distinguish standstill from sensor loss,
an armed stop after movement also holds the prior estimate until fresh RPM
returns or the car is disarmed. Do not label held speed as live measured speed.

## Motor control and safety

ESC `Brake Type = Reverse` is assumed, not Linear Reverse Brake. The local
`Skywalker_ESC_Manual.pdf`, when available, is the behavior authority. It is
not present in this checkout; no ESC interpretation is changed here.
Throttle is white-wire magnitude 1100–1940 us; yellow reverse is 1100 us
forward, 1940 us reverse. Safe means all eight outputs at 1100 us. Keep
valid safe PWM active while disarmed and yellow low during endpoint calibration.

Normal drive sends equal magnitude to selected motors: AWD all four, FWD only
front, RWD only rear. Inactive motors always receive 1100/1100 us, even in
reverse. Invalid runtime drivetrain mode safes all four outputs. Correct
physical motor directions by swapping any two phase wires on incorrect motors;
do not add per-wheel software inversion. Historical rear-left-only correct
direction is not proof of current phase wiring.

Throttle defaults full-forward/neutral/reverse 978/1514/2044 us, deadband 80 us,
quadratic response. Reverse limit defaults 100%: treat bench tests as full
reverse-authority tests. Drive smoothing 0–100%, default 100%, keeps 12 us
accel / 100 us decel steps per 20 ms; 50% uses 24/200 us; 0% bypasses ramps.
The existing 120 ms direction-change interlock is retained; direction changes
pass through throttle minimum before yellow changes.

Initial arming requires saved throttle and three-position CH4 calibration,
valid CH2/CH4, neutral throttle, learned RUN, 250 ms stable qualification, and
a healthy STOP observation since boot. CH4 defaults RUN=983, STOP1=2049,
STOP2=1515 us, with +/-125 us recognition windows. STOP acts immediately;
unrecognized valid-range CH4 requires 60 ms confirmation; missing input times
out at 100 ms. GPIO33 is not a hardware emergency stop.

Permanent arm latch defaults on. Once armed, CH4 STOP and receiver safety
faults temporarily inhibit outputs; valid input may automatically resume
through the acceleration ramp without another arm cycle. Operator must return
throttle to neutral before recovery. With latch off, these events fully disarm
and require healthy STOP-to-RUN. CLI `disarm` with latch on creates a boot-long
inhibit; with latch off it normally disarms. `disarm config` always disarms,
clears the latch, and requires a new physical STOP-to-RUN cycle. Boot never
arms or enters ESC calibration automatically.

The fixed transmitter-off throttle detector defaults on at 1565 +/-10 us,
based on the observed R7FG disconnected output. It immediately inhibits (or
disarms with latch off) and centers steering. Recommended receiver failsafes:
CH1 center, CH2 neutral, CH4 STOP. Always retain independent traction-power
isolation. Missed task deadlines increment counters, reset periodic timing,
and insert a one-tick block; do not weaken deadlines for networking.

## Commands, calibration, and network boundary

USB CLI remains 115200 baud:

```text
status | help | arm | disarm | disarm config
cal receiver | cal steering | cal arm
cal esc arm | cal esc max | cal esc min | cal manual | cal cancel
monitor throttle | monitor steering | monitor arm | monitor rpm
config drivetrain <awd|fwd|rwd>
config reverse <0-100>
config drive smoothing <0-100>
config failsafe off | <800-2200 pulse_us> <1-100 window_us>
config rpm ppr <1-120>
config rpm poles <even 2-60>
config steering smoothing <0-500>
config steering input <left_us> <center_us> <right_us> <deadband_us>
config steering servo <left|right> <wheel_left_us> <center_us> <wheel_right_us> <left_deg> <right_deg>
config steering geometry <wheelbase_mm> <track_mm> <wheel_diameter_mm> <max_left_deg> <max_right_deg>
config steering ackermann-strength <speed:percent> ...
config arm-latch <on|off>
```

Example only, not a physically validated tuning: `config steering ackermann-strength
0:60 20:30 50:0`. Serial receiver calibrations capture about 1.5 seconds per
prompt. Browser equivalents use start/capture/cancel state machines and a
120-second inactivity timeout. Monitor commands run 30 seconds. Configuration
is DISARMED-only and blocked during calibration. `cal steering` learns CH1;
servos continue following valid input, so restrain the chassis and verify
output endpoints first.

ESC endpoint procedure: lift and restrain, disconnect traction power, select
CH4 STOP and neutral, `cal esc arm`, then `cal esc max`. Power ESCs, wait for
maximum beeps, and send `cal esc min` within the manual's five-second window.
After acceptance/ready beeps, `cal cancel`. All four ESCs participate regardless
of drivetrain. Manual relay clamps receiver throttle to 1100–1940 us for at
most 30 seconds, yellow low. Leaving recognized STOP, required signal loss,
configured failsafe, cancellation, or 30-second inactivity restores safe PWM.

Default SoftAP SSID `RC-Car-ESP32`; configure credentials in menuconfig before
untrusted use. Dashboard at `http://192.168.4.1`, no hotspot or bridge needed.
HTTP exposes live telemetry, guarded settings/calibrations, and maintenance
disarm. It must never expose remote arm,
ordinary CLI disarm, arbitrary terminal, monitor, steering, or drive commands.
The obsolete Python TCP/UDP bridge and mock TCP peer have been removed.
Embedded monitor assets live in `main/web/`; browser tests simulate the same
direct HTTP API using a fixture checked against production JSON. Protocol 3
reports only CH1/CH2/CH4, rear RPM, two servo outputs,
Ackermann strength, signed model angle targets, speed live/held state, four ESC
outputs, and complete steering/input configuration.

The monitor uses a compact header, status strip and numeric tables. Desktop
Live presents receiver, steering, ESC and rear-RPM panels together; narrow
screens stack panels without reducing 44 px form/navigation targets. Units
remain explicit, and held speed has both a HELD label and warning color.
The header stays visible when scrolling, with scroll padding for focused
controls. Touch inputs use 16 px text in both orientations, and interactive
links have 44 px targets. Horizontal safe-area padding also applies in phone
landscape. Desktop and touch browser profiles are verified separately.
Lost telemetry dims every live panel and locks command buttons. Settings group
drive inputs and servo endpoints into compact rows; curve interpolation and
speed-loss details are expandable. Calibration instructions, ESC warnings,
confirmations and controller guards are retained. Calibration output states
remain identified in Live instead of appearing as disarmed motor outputs.

## Storage and verification

Telemetry recording and CSV import/export/analysis have been removed, including
the logger task, configuration, HTTP endpoints, FAT/wear-levelling dependency,
and logdata partition. RC Monitor keeps Live, Calibrate, and Settings views.
Only live telemetry is served. The old recording area (0x150000–0x3FFFFF) is
unallocated. NVS, PHY, and application partitions keep their offsets and sizes.
Old recording settings are ignored; there is no erase or format operation.
The pin-map generator, its assets/launcher, and its build target are removed.
Keep NVS writes out of the 20 ms drive loop.

2026-10-23: ESP-IDF 6.1 build and host/browser simulations are the available
validation categories. No flashing or new physical testing was performed.
Required bench progression:

1. Traction disconnected: scope eight ESC and two servo outputs through boot,
   CH1 motion/loss/failsafe, NVS saves, and network activity.
2. Establish each servo's physical direction, center and safe travel; verify
   measured endpoint/intermediate road-wheel angles, left/right inside-wheel
   selection, 0% parallel and 100% geometric targets, exact centers and
   interpolated strengths at several rear speeds.
3. Validate both rear RPM waveforms, 7 PPR and optical-tachometer accuracy.
   Disconnect each sensor while armed and verify the held-speed indication;
   verify disarm resets unavailable speed and restores the 0 km/h curve point.
4. Verify both CH4 STOP positions, signal loss, both latch policies, browser
   maintenance disarm/rearming and reverse transition at restrained low power.
5. Check supply integrity, GPIO2 boot behavior, network outages/recovery,
   timing overruns and independent traction isolation.
6. Test one motor before all four; then cautious low-speed vehicle tuning.

Unresolved historical risks include supply-related resets, flash-write receiver
capture timing and LEDC startup fixes validated only in code/build, and actual
AP multi-device/RF behavior. Never equate those with successful hardware tests.
