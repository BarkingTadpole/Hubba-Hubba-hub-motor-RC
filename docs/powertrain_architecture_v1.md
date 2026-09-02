# Four-Motor RC Car Powertrain Architecture

## 1. Scope

This firmware controls the powertrain of a 1/7 scale RC car:

- 4x Hobbywing Skywalker 2820SL 550KV motors
- 4x Hobbywing Skywalker 50A V2 ESCs
- Radiolink RC6GS V3 transmitter and R7FG receiver
- ESP32 DOIT DevKit WROOM-32
- Adafruit STEMMA QT ISM330DHCX IMU
- 4x Hobbywing HW86060041 RPM sensors

Receiver CH1 now enters the ESP32 on GPIO16. The firmware validates and
relays that command to the steering servo from GPIO32. A missing CH1 signal
or the configured transmitter-off throttle pulse centers the servo.

Normal drive can select AWD, FWD, or RWD. AWD is the default. With torque
vectoring disabled, the selected driven wheels receive identical throttle and
reverse commands while every inactive axle receives safe PWM. With torque
vectoring active, the driven left/right outputs may differ by a guarded
correction.

## 2. Operating Modes

- `DISARMED`: all throttle outputs are `1100 us`; all reverse outputs are
  `1100 us`.
- `DRIVE_ARMED`: receiver throttle is mapped to the ESC range and sent
  through the selected AWD/FWD/RWD drive and optional vectoring controller.
- `ESC_CAL_ARMED`: calibration prepared, safe outputs active.
- `ESC_CAL_MAX`: all throttle outputs `1940 us`, reverse outputs `1100 us`.
- `ESC_CAL_MIN`: all throttle and reverse outputs `1100 us`.
- `ESC_CAL_MANUAL`: receiver throttle PWM is clamped to `1100-1940 us` and
  relayed to all ESC throttle inputs; reverse stays low.

No drive or ESC calibration mode is entered automatically at boot. ESC LEDC
outputs are initialized to safe pulses before NVS, receiver, sensor, or CLI
initialization. A failure to create the required powertrain task leaves safe
outputs active and does not start the CLI. CLI commands and the periodic
powertrain update serialize state/output transitions with a recursive mutex,
so a stale drive iteration cannot overwrite a concurrent disarm or
calibration command.

After the IMU initializes, startup assumes the chassis is level and stationary
and blocks for five seconds while sampling its gyro yaw bias. Powertrain and
sensor tasks, CLI handling, and arming do not begin until this attempt ends;
the previously established ESC safe PWM continues in hardware. Success or
failure is printed over serial. Failure is non-fatal to equal-output driving
but keeps torque vectoring unavailable until a disarmed `cal imu` retry
succeeds. Beginning a retry invalidates the previous bias, so failure cannot
silently retain a stale calibration.

The DOIT DevKit's controllable onboard LED shares GPIO2 with the front-left
ESC reverse output. It cannot be flashed without corrupting the required ESC
direction PWM, and the board's power LED is hardwired. Therefore, startup
calibration does not use an onboard LED and the pin assignments remain
unchanged. A visual status indicator requires an external LED on a separately
assigned free GPIO.

ESP-IDF 6's thread-safe `ledc_set_duty_and_update()` API depends on the LEDC
fade service even for an immediate, non-fading update. After all eight ESC
channels are configured with safe initial duty, `esc_output_init()` installs
that service before calling the update API. Installation failure is returned
to `app_main`, which keeps the application in its non-driving startup error
loop. The service is a driver requirement, not a commanded throttle fade and
does not alter the 50 Hz pulses or pin map.

The persistent drivetrain selection affects normal `DRIVE_ARMED` output only:

- `AWD`: all four wheels are driven.
- `FWD`: only FL and FR are driven; RL and RR remain at `1100/1100 us`.
- `RWD`: only RL and RR are driven; FL and FR remain at `1100/1100 us`.

Inactive-wheel reverse/direction stays low even when the driven axle is in
reverse. ESC endpoint calibration and manual relay intentionally continue to
address all four ESCs so hardware can be configured without changing modes.
An invalid runtime mode masks every wheel to safe output.
Select the persistent mode before arming with:

```text
config drivetrain <awd|fwd|rwd>
```

## 3. Pin Assignment

All assignments live in `main/pin_config.h`.

### Receiver PWM Inputs

| Function | R7FG channel | ESP32 GPIO |
|---|---:|---:|
| Steering command | CH1 | GPIO16 |
| Throttle | CH2 | GPIO25 |
| Shutdown RUN/STOP | CH4 | GPIO33 |
| Torque-vectoring mode | CH5 | GPIO26 |

### Steering Servo Output

| Function | ESP32 GPIO |
|---|---:|
| 50 Hz steering-servo PWM | GPIO32 |

Recommended CH5 three-position mapping:

- Position 1: OFF
- Position 2: straight-line assist
- Position 3: full turn assist

### ESC Outputs

| Wheel | Throttle GPIO | Reverse GPIO |
|---|---:|---:|
| Front left | GPIO13 | GPIO2 |
| Front right | GPIO14 | GPIO27 |
| Rear left | GPIO21 | GPIO19 |
| Rear right | GPIO18 | GPIO17 |

GPIO2 is an ESP32 boot-strapping pin. The ESC signal input should be high
impedance, but do not add an external pull-up or pull-down to this line. If
boot reliability is poor with the ESC connected, relocate this signal to a
non-strapping output and update `pin_config.h`.

### RPM Inputs

| Wheel | Sensor white wire | ESP32 GPIO |
|---|---|---:|
| Front left | RPM signal | GPIO34 |
| Front right | RPM signal | GPIO35 |
| Rear left | RPM signal | GPIO36 |
| Rear right | RPM signal | GPIO39 |

GPIO34 through GPIO39 are input-only and have no internal pull-ups.

### ISM330DHCX

| IMU signal | ESP32 GPIO |
|---|---:|
| SDA | GPIO23 |
| SCL | GPIO22 |
| INT1, unused | Not connected |
| INT2, unused | Not connected |

### Dragy Lite GPS and Compass Harness

| Dragy wire | Function | ESP32 connection |
|---|---|---|
| Yellow | UART TX | GPIO4, UART1 RX |
| White | UART RX | GPIO5, UART1 TX |
| Blue | I2C SDA | GPIO23, shared with IMU |
| Green | I2C SCL | GPIO22, shared with IMU |
| Black | Ground | Common ground |
| Red | Power | Regulated 5-16 V rail, not a GPIO |

UART1 is routed through the ESP32 GPIO matrix so UART0 remains available for
USB flashing and the 115200-baud CLI. A receive-only installation needs only
Dragy yellow TX to GPIO4 and common ground. GPIO5 is a boot-strapping pin; do
not add a pull-up/down, and confirm that the Dragy RX input is high impedance
during ESP32 reset. If that connection causes unreliable boot, leave the
white wire disconnected until a non-strapping output can be freed.

GPIO23/GPIO22 remain a single 400 kHz bus rather than two independent I2C
controllers. The ISM330DHCX address is `0x6A`; the MAX-M10S address is `0x42`
if the Dragy exposes the GNSS receiver on its harness I2C bus. Measure both
UART and I2C logic-high voltages before connection. I2C pull-ups must reference
3.3 V, address conflicts are not allowed, and parallel pull-ups must not make
the total resistance too low.

The firmware polls the IMU over I2C at 200 Hz. Interrupt pins are not used
in this version. `CONFIG_FREERTOS_HZ=1000` is required so the 5 ms sensor
period converts to a nonzero FreeRTOS delay; the firmware also enforces this
constraint at compile time. Both periodic tasks reset their schedule and
block for at least one tick after a missed deadline so an overrun cannot turn
into a watchdog-starving catch-up loop. `status` reports both overrun counts,
maximum observed execution time, minimum free stack for both tasks, and
current/minimum/largest-block heap values. These are runtime observations and
must be recorded during worst-case hardware operation; a successful build
does not establish real-time headroom.

## 4. Electrical Wiring

### Common Ground

Receiver ground, ESP32 ground, IMU ground, RPM sensor grounds, and all ESC
signal grounds must be connected. Signal references are unreliable without
this common ground.

### Receiver and Steering Path

The servo signal is no longer connected directly to receiver CH1. Route the
command through the ESP32:

```text
R7FG CH1 signal ---- 3.3 V conditioning ---- GPIO16
GPIO32 -------------------------------------- steering servo signal
BEC/receiver servo rail --------------------- steering servo power
common ground ------------------------------- servo, receiver, and ESP32
```

Do not leave receiver CH1 connected to the servo signal after GPIO32 is
connected; two driven PWM outputs must not be tied together. Do not connect
servo power to an ESP32 GPIO. Condition every receiver PWM signal to a maximum
of 3.3 V if the R7FG output is above 3.3 V. GPIO32 produces 3.3 V logic; verify
that the servo accepts it, or use a proper 3.3 V-to-5 V logic buffer. Verify
input and output pulse widths with `monitor steering` and an oscilloscope or
signal tester, not with a multimeter's DC voltage mode.

The servo starts at the saved steering center, follows valid CH1 commands at
50 Hz subject to its speed-sensitive envelope, clamps commands to the
calibrated left/right endpoints, and applies
the steering deadband at center. A configurable first-order command filter
smooths valid requests; its default time constant is `10 ms` and `0 ms`
disables it. Before smoothing, a median filter uses the latest three distinct
receiver frames and rejects one isolated valid-looking pulse anywhere in the
steering range. A monotonic or stepped real command emerges one receiver frame
later, adding approximately `20 ms` of continuous, predictable latency.
Startup, missing CH1, the configured throttle failsafe, and a new steering
calibration clear its history and hold the saved center until three new frames
arrive, normally `40-60 ms`. Repeated powertrain-loop reads of the same
receiver timestamp do not advance the history. `status` and
`monitor steering` expose the raw pulse, current median, `warmup` or `active`
state, and a count of confirmed isolated spikes whose neighboring frames
returned within the steering deadband. The saved trim is applied after the
median filter and before the first-order command filter and is limited to
`+/-15 degrees` on a nominal `1000 us = 90 degrees` servo-command scale. This
is command-space trim, not a claim about measured road-wheel angle. Both trim
and filtered output remain clamped to the learned endpoints. A trim command is
rejected if its center would not remain strictly between those endpoints; a
persisted trim is reset to zero if a later steering calibration makes it
invalid.

Steering setup requires a verified trim entry in the persistent NVS drive
configuration (`cfg/st_trim10`); a stored zero is valid when the linkage is
already centered. While disarmed, this one-shot command validates and saves a
trim, waits one drive update for it to take effect, and then starts the normal
30-second steering monitor:

```text
monitor steering trim <-15..15>
```

If the NVS write fails, the controller restores the previous in-memory trim
and does not start the monitor. `config steering trim` remains available when
monitoring is not needed.

Missing CH1 returns the servo to the trimmed center immediately rather than
ramping through the filter. A matching CH2 transmitter-off pulse does the
same when the optional throttle-pulse detector is enabled. CH4 STOP/OFF does
not disable otherwise-valid steering control. It either temporarily inhibits
motor output or returns drive to `DISARMED` according to the saved arm policy.
Configuration changes are accepted only while `DISARMED`:

```text
config steering trim <-15..15>
config steering smoothing <0-500>
config steering speed-limit <on|off>
config steering lateral-g <0.2-3.0>
```

### Steering Curves

The corrected steering data contains 45 points from `-45` to `+45` servo
degrees. Most samples are two degrees apart, with three-degree intervals
between `40` and `43 degrees` in each direction. Source servo-positive steers
the vehicle left. The wheel-angle signs are local to each wheel: positive
means that wheel points outward, and negative means it points inward.

Firmware converts the supplied angles into the controller's common
positive-right heading frame:

```text
runtime_servo = -source_servo
runtime_LF    = -source_LF
runtime_RF    =  source_RF
```

Consequently, the supplied center values `LF=+1.64` and `RF=+1.64 degrees`
represent symmetric toe-out. They become runtime `LF=-1.64` and
`RF=+1.64 degrees`; their average is exactly zero and does not request yaw.

`steering_curve.c` stores all 45 rows verbatim and performs deterministic
linear interpolation between the two bounding servo samples. The applied,
smoothed GPIO32 pulse is converted to nominal servo-command degrees relative
to the trimmed center using `1000 us = 90 degrees`. Values beyond the measured
range saturate to the nearest endpoint. Interpolation produces separate LF/RF
headings, their common-frame mean, and a direction-specific normalized mean:
`+29.738 degrees` is full right and `-31.079 degrees` is full left. That
normalized mean is the steering input to the empirical torque-vectoring
target. The curve also provides the inverse lookup used by the speed-sensitive
steering envelope, so limiting is performed in measured average road-wheel
degrees rather than raw PWM percentage.

### Speed-Sensitive Steering

The measured geometry defaults are wheel diameter `107 mm`, wheelbase
`445 mm`, and track width `320 mm`. Average rear-wheel RPM estimates chassis
speed:

```text
speed_mps = average_rear_rpm * pi * 0.107 / 60
max_average_wheel_angle = atan(0.445 * lateral_accel_limit / speed_mps^2)
```

The limiter is enabled by default with a `1.0 g` ceiling. It uses the inverse
45-point steering curve to clamp the requested average wheel angle before the
existing first-order steering smoother. At `20 km/h` the initial limit is
approximately `8.05 degrees`; at `40 km/h` it is approximately `2.02 degrees`.
Low speed naturally permits the full measured steering range.

Both rear RPM channels must be valid before a new speed estimate is accepted.
While DRIVE_ARMED, the last valid estimate is retained through a temporary
RPM dropout so loss of speed telemetry cannot suddenly restore full steering.
The estimate resets when disarmed. If no valid speed has ever been observed,
the limiter cannot infer speed and leaves the requested steering unchanged.
This limitation is why RPM validation remains required before powered tests.

The earlier 91-point CSV and its cubic fits were superseded by this corrected
table and are not evaluated by the control path. Linear interpolation avoids
inventing values beyond the adjacent measurements and preserves the supplied
toe geometry and endpoints.

`monitor steering` reports applied nominal servo angle, LF/RF interpolated
angles, average, normalized value, and curve saturation. These angles are
provided calibration data, not new on-car measurements by this firmware.

Use the R7FG in standard PWM mode with its integrated gyro disabled. The
receiver gyro modifies steering before the ESP32 observes it and would add
another feedback loop that conflicts with the ISM330DHCX-based controller.
The [R7FG manual](https://radiolink.com.cn/r7fg_manual) describes its PWM and
gyro modes.

CH1, CH2, CH4, and CH5 pulse widths are measured with priority-3 GPIO edge
interrupts installed on CPU0. The powertrain task is pinned to CPU1 and drives
the eight ESC signals through the high-speed LEDC group. Steering uses the
separate low-speed LEDC group. This separation reduces interference between
receiver capture and output updates. Signal freshness expires after 100 ms
without a completed valid pulse.

The GPIO interrupt service is allocated with `ESP_INTR_FLAG_IRAM`, and the
IRAM handler reads the ESP32 GPIO input registers directly. This keeps both
edges of each receiver pulse observable while internal-flash logging disables
the flash cache; otherwise a delayed edge can still fall inside the accepted
`800-2200 us` range and appear to be a real but impossible steering command.

Earlier MCPWM and RMT capture experiments are not approved for use. MCPWM
allowed channels to become stale when throttle changed. The RMT version caused
an unsafe forward/pause cycle, servo jolts, and ignored CH4 shutdown during a
bench incident. The GPIO rollback must be validated with traction power
disconnected before powered testing resumes.

### ESC Signals and BECs

- ESP32 throttle output goes to each ESC white throttle signal wire.
- ESP32 reverse output goes to each ESC yellow reverse signal wire.
- ESP32 3.3 V PWM is a logic signal, not a 5 V power rail. The ESC input only
  needs a valid logic high plus a common ground.
- Keep the four ESC signal outputs independent. This avoids fan-out loading
  and permits per-wheel control.
- Use only one ESC BEC red wire, or use a dedicated regulator. Do not connect
  four BEC outputs in parallel unless the manufacturer explicitly supports
  it.

Confirm the exact ESC SKU before selecting the traction battery. Hobbywing
lists the 2820SL 550KV motor as a 6S model, but the normal Skywalker 50A V2
and the Skywalker 50A-6S V2 do not have the same battery rating. A motor's
6S rating does not make a 4S-only ESC safe on 6S.

### RPM Sensors

For each HW86060041:

- Sensor phase leads A and B connect to any two of that motor's three phase
  wires. Polarity does not matter.
- Red powers the sensor. Use a stable 5 V rail because the specification
  lists a 3.5 V minimum even though the wiring instructions also mention
  3.3 V.
- Black connects to common ground.
- White is the RPM pulse output. Condition it to 3.3 V before the ESP32.

The currently proposed `1 kOhm` series / `2 kOhm` shunt divider produces
`3.33 V` from an exact `5.00 V` input. At `5.40 V` it produces `3.60 V`, the
ESP32 data-sheet input maximum at a 3.3 V supply, so it has essentially no
overvoltage margin. A `10 kOhm` series / `15 kOhm` shunt divider produces
about `3.00 V` from 5.0 V and remains above the ESP32's approximately
`2.48 V` minimum high threshold. Neither divider substitutes for measuring
the actual rail, the white-wire high level, and the pulse shape. If white is
open-collector/open-drain, provide an external pull-up because GPIO34-39 have
no internal pulls; do not assume the divider alone creates a logic high.

The phase-tap wiring carries traction-battery switching voltage. Insulate and
strain-relieve these connections. The official
[Hobbywing sensor instructions](https://www.hobbywing.com/uploads/file/20220817/046abfc86910635cb6fa868ce1efe1b4.pdf)
cover the lead functions and voltage ranges, but do not specify the output
stage or pulse ratio.

### IMU

Power the STEMMA QT board from ESP32 `3V3` and `GND`, then connect SDA and
SCL. Mount it rigidly near the chassis centerline. In the current mounting,
sensor `+X` points toward the rear of the car and sensor `+Y` points toward
the right. Soft foam mounting adds delay and corrupts yaw-rate feedback. The
[Adafruit pinout guide](https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/pinouts)
documents the board connections.

## 5. ESC Signal Model

The reviewed local
[Skywalker ESC manual](../../rc_car_showcase/public/project-docs/Skywalker_ESC_Manual.pdf)
specifies:

- Default throttle range: `1100 us` to `1940 us`.
- White wire: throttle magnitude.
- Yellow wire: reverse/brake direction input.
- In Reverse Brake mode, yellow below the midpoint selects the default
  direction and above the midpoint selects reverse.
- Yellow must be low/default direction at startup and during throttle
  endpoint calibration.

Use ESC `Brake Type = Reverse`, not Linear Reverse Brake. Drive mapping is:

```text
forward:
    throttle = 1100 us + requested_magnitude
    reverse  = 1100 us

reverse:
    throttle = 1100 us + limited_reverse_magnitude
    reverse  = 1940 us
```

The user-requested default reverse limit is 100 percent. Persistent drive
smoothing ranges from 0 to 100 percent and defaults to 100 percent. At 100,
the smoothest established ramp uses 12 us acceleration and 100 us deceleration
steps per 20 ms update. Lower percentages shorten both ramp durations using
`ceil(smoothest_step * 100 / smoothing_percent)`; 50 percent therefore uses
24/200 us steps. Zero disables both ramps and applies the requested magnitude
on the next drive update. A direction change still reaches throttle minimum,
holds for 120 ms, changes the reverse outputs, and then applies the new
direction at every smoothing setting. A neutral receiver command retains the
current yellow-wire direction; it never flips the direction output while the
ESC throttle pulse is above minimum. The ESC output module clamps every write
to the configured endpoint ranges as a final boundary check.

## 6. RPM Conversion and Calibration

The official Skywalker 2820SL 550KV specification identifies the motor as
`12N14P`, so it has 14 magnetic poles and 7 pole pairs. See the
[Hobbywing motor specification](https://www.hobbywing.com/en/products/skywalker2814).

The firmware counts rising edges from each phase-derived RPM signal with an
ESP32 PCNT unit. Conversion now uses an explicit pulses-per-mechanical-
revolution value:

```text
mechanical_rpm = pulse_frequency_hz * 60 / pulses_per_revolution

mechanical_rpm = rising_edge_count * 60,000,000
                 --------------------------------
                 elapsed_time_us * pulses_per_revolution
```

On 2026-08-12, the user directly counted seven sensor-output rising edges in
one mechanical wheel/motor revolution. This confirms the compiled `7 PPR`
conversion for the tested hardware even though the sensor manual does not
specify the ratio. `config rpm ppr` remains the authoritative setting. The
legacy pole command is retained for stored-configuration and CLI
compatibility; with 14 poles it derives the same PPR:

```text
config rpm ppr 7
config rpm poles 14
```

Sampling occurs every `20 ms`. The estimator uses at least five samples
(`~100 ms`) and expands to at most 25 samples (`~500 ms`) until it collects
four rising edges. At the default 7 PPR, the single-count resolution is about
`85.7 RPM` over 100 ms and `17.1 RPM` over 500 ms. This explains an apparent
`~86 RPM` low-end step; it is counting quantization, not proof of real wheel
motion. A channel becomes invalid 250 ms after its last pulse, even if older
edges remain in the rolling window.

`monitor rpm` reports raw frequency, edge count, actual window duration, and
derived RPM for all wheels. Torque vectoring also requires every sensor's raw
frequency to be at least `16.7 Hz`, matching the sensor's documented minimum
for its 2-pole reference. Low-frequency readings can still be monitored for
diagnosis but cannot activate closed-loop correction.

The sensor documentation does not state output logic type, high voltage, duty
cycle, or pulse ratio. The direct count resolved the ratio for the tested
motor/sensor, but verify the electrical interface and counting accuracy on all
four installed channels before enabling vectoring:

1. Leave torque vectoring disabled and lift the car.
2. Send `monitor rpm`.
3. Spin one wheel at a time and confirm the matching FL/FR/RL/RR channel and
   that inactive channels do not count noise.
4. Scope sensor white before and after conditioning. Record sensor supply,
   low/high voltage, pulse width, frequency, and whether the output needs an
   external pull-up. GPIO34-39 have no internal pull-ups.
5. Compare displayed RPM at several stable speeds with an optical tachometer.
   Confirm that `frequency_hz * 60 / tach_rpm` remains near 7 on every wheel.
6. Any departure from 7, nonlinear error, or channel disagreement indicates
   missed/extra edges, noise, or a hardware difference and must not be hidden
   by controller tuning.

Because these are hub motors, motor RPM and wheel RPM are the same.

## 7. Receiver Calibration

The firmware stores calibrations in ESP32 NVS.

```text
cal receiver   neutral, full throttle, full reverse
cal steering   center, full left, full right
cal arm        CH4 RUN/ON, STOP/OFF 1, then STOP/OFF 2
cal tv         CH5 OFF, STRAIGHT, then FULL
```

Each command prompts for a position. Hold the requested position and press
Enter; the ESP32 averages 1.5 seconds of PWM measurements. Endpoints are
identified by their captured values, so transmitter channel reversal is
handled without hardcoded polarity.

Compiled receiver defaults are centralized in `main/default_config.h`:

| Control | Position | Default pulse |
|---|---|---:|
| Throttle | Neutral | `1514 us` |
| Throttle | Full throttle | `978 us` |
| Throttle | Full reverse | `2044 us` |
| Steering | Center | `1518 us` |
| Steering | Full left | `2050 us` |
| Steering | Full right | `987 us` |
| Torque vectoring | OFF | `2047 us` |
| Torque vectoring | STRAIGHT | `1513 us` |
| Torque vectoring | FULL | `981 us` |
| Arm/output inhibit | RUN/ON | `983 us` |
| Arm/output inhibit | STOP/OFF 1 | `2049 us` |
| Arm/output inhibit | STOP/OFF 2 | `1515 us` |

The default throttle deadband remains `80 us` and the steering deadband is
`30 us`. Valid NVS calibration overrides these compiled values. Compiled
defaults do not set `loaded_from_nvs`, so throttle and CH4 still require saved
calibration before drive can arm.

Drive arming requires saved throttle and CH4 calibrations. Steering and CH5
calibration are required only before enabling torque vectoring.

## 8. Receiver Arm Policy and Output Inhibits

CH4 replaces the former physical GPIO switch. It is a three-position switch
calibrated as one RUN/ON position and two STOP/OFF positions. The persistent
`cfg/arm_latch` option selects whether a successful arm remains latched for the
boot or returns to `DISARMED` on subsequent safety events. Permanent latching
is enabled by default for backward compatibility.

Normal arming:

1. Receiver is connected and CH4 is in either STOP/OFF position.
2. Throttle is neutral.
3. Move CH4 from STOP/OFF to RUN/ON.
4. Keep RUN/ON and neutral throttle valid continuously for 250 ms.

The 250 ms pre-arm qualification prevents a short receiver-wide PWM gap after
a switch transition from occurring just after drive entry. Any invalid CH4 or
throttle sample restarts the qualification timer.

With permanent latching enabled, `SYSTEM_DRIVE_ARMED` remains set until reset
or power loss. Either learned STOP/OFF position immediately applies safe ESC
outputs without changing that state. CH4 or throttle loss applies the same
inhibit after the 100 ms receiver timeout. When valid CH4 RUN and throttle
data return, the inhibits clear automatically and throttle restarts from
minimum through the acceleration ramp; no new arm cycle or neutral check is
required, so the operator must return throttle to neutral before recovery.

With permanent latching disabled, recognized STOP, CH4 or throttle loss, a
confirmed unrecognized CH4 position, and a matching configured throttle
failsafe return to `SYSTEM_DISARMED`. The controller then requires another
healthy STOP/OFF observation and STOP/OFF-to-RUN/ON transition, including the
250 ms RUN/neutral qualification. In both policies, an unrecognized pulse must
persist for 60 ms before action so one malformed frame is rejected.

CH4 is processed by the ESP32 and is not a hardware emergency stop. The CLI
`disarm` command creates a non-clearable safe-output inhibit for the rest of
the boot when permanent latching is enabled; when disabled it performs a
normal disarm. The exact `disarm config` maintenance command is the deliberate
exception: from `DRIVE_ARMED` it immediately commands safe output, returns to
`SYSTEM_DISARMED`, and requires a new physical CH4 STOP/OFF-to-RUN/ON cycle
before drive can rearm. It is available from serial and as the dashboard's
one-way **Disarm for config** action, but neither interface can rearm remotely.
Testing must retain direct traction-power isolation in either policy. A final
vehicle must independently remove ESC power or disable the ESCs even if the
ESP32 is stalled.

`config arm-latch <on|off>` changes the policy only in `DISARMED`, saves it to
NVS, and applies it to the next arm without a reboot. `status` reports the
selected policy, whether drive output is active or inhibited, the current
inhibit reason, a cumulative inhibit-event count, and the last reason.

Each learned switch position has a `+/-125 us` recognition window. A pulse
outside all three windows is unrecognized: after the 60 ms confirmation it
temporarily inhibits or fully disarms according to the selected policy.

Configure the R7FG failsafe so:

- CH1 commands steering center.
- CH2 commands neutral throttle.
- CH4 commands either learned STOP/OFF position.
- CH5 commands OFF.

The fixed throttle-pulse detector defaults enabled for the measured CH2
transmitter-off value of `1565 us` with a `10 us` window:

```text
config failsafe 1565 10
```

Disable the detector again with `config failsafe off`. When enabled, the
configured pulse immediately applies safe output. It is a recoverable inhibit
with permanent latching enabled and a full disarm when disabled. Complete PWM
loss is declared after 100 ms regardless of this setting.

## 9. Laptop ESC Calibration

Use the ESP32 USB serial connection at `115200` baud. CH4 must remain in a
learned STOP/OFF position,
throttle must remain neutral, the wheels must be lifted, and ESC traction
power starts disconnected.

1. Send `cal esc arm`.
2. Send `cal esc max`.
3. Confirm all four throttle outputs are `1940 us` and reverse outputs are
   `1100 us`.
4. Power the ESCs.
5. Wait for the maximum-endpoint beeps.
6. Send `cal esc min` within the ESC manual's five-second window.
7. Wait for endpoint acceptance and ready beeps.
8. Send `cal cancel`.

Any missing/non-STOP CH4 signal or a 30-second command timeout returns all
outputs to safe values.

`cal manual` is an alternate 30-second bench mode that relays CH2 PWM, clamped
to the ESC's `1100-1940 us` throttle range, to all four throttle outputs while
holding reverse low. It is unavailable until CH4 has a saved calibration and
is in either learned STOP position. CH4 leaving STOP, signal loss, timeout,
or `cal cancel` stops the relay. A configured throttle failsafe match also
stops it when that detector is enabled.

## 10. Torque-Vectoring Controller

Torque vectoring is configured enabled in the requested first-boot profile,
but runtime prerequisites and CH5 still gate activity. The baseline OFF mode
always sends the same throttle to all four ESCs.

Before `tv enable` is accepted during a boot:

- `cal steering` and `cal tv` must be stored.
- The five-second startup calibration or a later `cal imu` retry must succeed
  with the car level and stationary.
- Every selected driven-wheel RPM input must have produced valid pulses. Use
  `monitor rpm` and rotate the driven wheels. Validate all four channels
  before changing drivetrain modes.
- The sensor task must have started successfully.

IMU gyro bias is intentionally calibrated each boot because temperature and
mounting bias drift. If IMU or RPM data becomes invalid while driving, if an
input/configuration becomes non-finite, or if any selected driven RPM raw
frequency is below `16.7 Hz`, the controller immediately resets its integral
and falls back to equal throttle on the driven wheels. The inactive axle
remains safe.

`monitor imu` only reads the latest snapshot and reports whether its bias is
calibrated. It never invokes calibration. USB serial programs may toggle the
DOIT devkit DTR/RTS reset lines when attaching; that starts a new ESP32 boot
and therefore correctly starts another five-second calibration. Use a serial
terminal with DTR/RTS reset disabled to attach without rebooting.

### CH5 Modes

- OFF: equal commands on the selected driven wheels, with the inactive axle
  safe. A CH5 pulse outside all three learned `+/-125 us` windows is also
  treated as OFF.
- STRAIGHT: active only while steering is within 6 percent of center. Target
  yaw rate and left/right RPM difference are both zero.
- FULL: steering creates empirical target yaw rate and left/right RPM
  difference. It includes the straight controller around center.

Positive normalized steering means right. It is the direction-normalized mean
of the supplied LF/RF road-wheel curves evaluated at the applied servo output,
not raw receiver or servo travel. For a right-turn correction, the left pair
receives positive correction and the right pair receives equal negative
correction:

```text
FL = base + correction
FR = base - correction
RL = base + correction
RR = base - correction
```

The controller uses selected driven-wheel RPM: both axles in AWD, the front
pair in FWD, or the rear pair in RWD. It then applies correction only to the
selected driven wheels:

```text
curve_steering = average_road_wheel_angle / direction_specific_max_angle
target_yaw = curve_steering * turn_yaw_gain * base_throttle
desired_side_rpm_delta = curve_steering * turn_rpm_gain

yaw_error = target_yaw - measured_yaw
rpm_error = desired_side_rpm_delta
            - ((left_rpm - right_rpm) / average_rpm)

correction = yaw_kp * yaw_error
             + yaw_ki * integrated_yaw_error
             + rpm_kp * rpm_error
```

FULL mode also estimates lateral demand from RPM speed and applied average
road-wheel angle:

```text
predicted_lateral_accel = speed^2 * tan(abs(wheel_angle)) / wheelbase
lateral_demand = clamp(predicted_lateral_accel / configured_limit, 0, 1)
front_relief = min(front_relief_max * lateral_demand, base_throttle)

FL = base + correction - front_relief
FR = base - correction - front_relief
RL = base + correction
RR = base - correction
```

Front relief defaults to `20%` of full ESC span and only operates while FULL
mode is active in AWD or FWD. It deliberately reduces total requested power
instead of immediately transferring power rearward. RWD forces it to zero
because the front axle is already inactive. OFF and STRAIGHT do not apply it.
Configure it while `DISARMED` with `config tv front-relief <0-50>`.

The sign convention is internally consistent: positive steering and positive
yaw both mean a right turn, and positive correction adds left-side torque
while removing the same amount on the right. The integral is conditionally
held when the requested correction is saturated and additional integration
would drive farther into saturation. This is anti-windup for the existing
controller; it does not make the empirical yaw target a vehicle model.

Side correction defaults to at most `+/-25%` of full throttle span. Left and
right side corrections are equal and opposite. FULL-mode front relief is a
separate non-balanced reduction, so mean requested power intentionally falls
as lateral demand increases. Outputs remain clamped to the ESC range.

Vectoring is forward-only in this version. Reverse always uses equal throttle
on the selected driven axle, and the inactive axle remains safe.

Initial tuning commands:

```text
config tv authority 25
config tv front-relief 20
config tv gains 180 0.20 0.00025 0.00004 0.20
config imu yaw-sign -1
```

Use `monitor imu` and rotate the car right by hand. If reported yaw is
negative, use `config imu yaw-sign -1`. Use `monitor vector` to inspect
target yaw, errors, and four corrections. Tune with the car restrained or
at low speed, starting with authority at 2 to 5 percent.

## 11. Firmware Structure

- `app_main.c`: initializes and starts the application.
- `cli.c`: parses USB serial commands.
- `default_config.h`: owns compiled receiver defaults and the IMU mounting
  reference.
- `config_store.c`: owns defaults and NVS persistence.
- `cornering_control.c`: converts rear RPM to speed and calculates the
  steering envelope and lateral demand.
- `drivetrain_control.c`: validates AWD/FWD/RWD selection and masks inactive
  axle outputs to safe PWM.
- `rc_input.c`: captures CH1, CH2, CH4, and CH5 PWM with priority-3 GPIO edge
  interrupts on CPU0.
- `esc_output.c`: generates eight independent 50 Hz LEDC outputs.
- `servo_output.c`: generates the independent 50 Hz GPIO32 steering output.
- `steering_input_filter.c`: applies full-range three-frame median rejection
  using distinct receiver timestamps.
- `steering_curve.c`: interpolates the supplied 45-point LF/RF road-wheel
  curves and converts their sign convention for controller use.
- `rpm_sensor.c`: owns four hardware PCNT units and RPM conversion.
- `imu_sensor.c`: configures and reads the ISM330DHCX over I2C.
- `sensor_i2c_bus.c`: owns the shared 400 kHz I2C bus and serializes IMU,
  discovery, and future compass transactions.
- `dragy_nmea.c`: validates NMEA checksums and parses RMC, GGA, and VTG fields.
- `dragy_sensor.c`: owns UART1 GPS reception, stale-data state, and the
  startup shared-I2C address scan.
- `remote_command.c`: parses the configuration/TV subset, exact guided
  calibration operations, and the one-way `disarm config` maintenance action
  permitted over the network. It rejects drive, arm, ordinary disarm,
  monitoring, and arbitrary command text.
- `telemetry_log.c`: samples thread-safe controller/Dragy snapshots from a
  low-priority task and owns the wear-levelled FAT CSV file.
- `torque_vectoring.c`: contains the sensor controller and balanced wheel
  correction.
- `powertrain_controller.c`: owns system state, arming, failsafe, mapping,
  calibration flows, and task scheduling.
- `wifi_control.c`: owns the WPA2 SoftAP and embedded dashboard/guarded HTTP
  API server.
- `pin_config.h`: is the single source of truth for physical pin assignment.

## 12. ESP32 SoftAP and On-Device Viewer

Wi-Fi defaults enabled under the `RC car Wi-Fi access point` project menu.
After the powertrain, logger, and serial CLI start, the ESP32 creates a 2.4 GHz
WPA2 SoftAP instead of associating with an upstream WLAN. The tracked
development defaults are SSID `RC-Car-ESP32`, password `rc-car-viewer`, channel
1, four clients, and HTTP port 80. Normal access is `http://192.168.4.1`; the
ESP32 DHCP server supplies client addresses. There is no Python process,
station DHCP, UDP discovery, or separate TCP telemetry session in normal use.

The ESP-IDF HTTP server runs at priority 2 with an 8 KiB task stack. Receiver
edge interrupts remain priority 3 and the powertrain task remains pinned to
CPU1. The browser requests telemetry every 250 ms, and formatting occurs only
in the HTTP task; no network operation runs in the 20 ms drive loop. Wi-Fi or
HTTP initialization failure is non-fatal to the powertrain and leaves USB
serial operation available. The expanded state JSON uses a dedicated static
8 KiB buffer rather than the task stack.

The firmware embeds `index.html`, `app.js`, and `style.css` from
`tools/wifi_bridge/web`. Its direct HTTP surface is:

- `GET /`, `/index.html`, `/app.js`, `/style.css`: embedded viewer assets.
- `GET /api/state`: protocol-version-1 telemetry JSON containing controller
  state, receiver, steering, RPM, IMU, Dragy, vectoring, outputs, logging, and
  non-secret persistent configuration.
- `GET /api/config`: current configuration plus NVS/default source.
- `POST /api/config`: guarded configuration command JSON.
- `POST /api/calibration`: exact guarded calibration command JSON.
- `POST /api/disarm`: exact one-way `disarm config` maintenance action.
- `GET /api/log.csv`: a fixed-starting-size CSV snapshot streamed directly in
  8 KiB chunks.
- `POST /api/log/clear`: explicitly clear/recover logging storage; firmware
  rejects it unless the controller is `DISARMED`.

The accepted command subset matches the documented `config ...` commands,
`tv enable|disable`, the exact one-way maintenance action `disarm config`, and
these exact calibration operations: `cal receiver`, `cal steering`, `cal arm`,
`cal tv`, `cal imu`, `cal capture`, `cal esc arm|max|min`, `cal manual`, and
`cal cancel`. `remote_command_parse` has no arm, ordinary disarm, drive
throttle, steering output, `monitor ...`, or arbitrary terminal operation. The
maintenance action immediately applies safe ESC output and returns
`DRIVE_ARMED` to `DISARMED`; drive can rearm only after a physical CH4
STOP/OFF-to-RUN/ON cycle and its normal neutral qualification.

Browser receiver calibration remains a controller-owned state machine. Each
`cal capture` averages the requested channel for 1500 ms and only a complete,
validated three-position set replaces NVS. IMU capture uses the existing
two-second current-boot stationary bias retry. Configuration is rejected while
any calibration is active. Browser receiver/IMU workflows expire after 120
seconds, while ESC calibration/manual modes retain their 30-second timeout and
all neutral, CH4 STOP, receiver-loss, reverse-low, and failsafe guards.

The existing controller setters remain authoritative: configuration and
calibration start require `DISARMED`, TV enablement still requires live sensor
prerequisites, and NVS write success is required before HTTP reports success.
The dashboard's read-only configuration view remains available while armed.
Its confirmed **Disarm for config** action cannot arm and does not bypass the
physical rearm cycle.

There is no application-level login beyond WPA2 association. Treat the AP
password as a control credential and change the tracked development default
before operation around untrusted clients. `tools/wifi_bridge/bridge.py`
remains only for mock/frontend development and is not part of the runtime
architecture.

The Wi-Fi radio adds current bursts and RF/network tasks. Validate ESP32 supply
voltage and reset behavior with the SoftAP active and viewers polling before
powered driving; a successful compile or browser mock test is not
power-integrity or radio validation.

## 13. Dragy Lite GPS Integration

Enable `Dragy Lite GPS` at build time. UART1 uses `9600 8N1` by
default because that is the MAX-M10S reset configuration; select the actual
Dragy output rate if its firmware uses another value. The fixed-buffer parser
accepts checksum-valid RMC, GGA, and VTG sentences from any NMEA talker prefix.
It reports fix, latitude/longitude, ground speed, course, altitude, satellites,
HDOP, UTC/date, byte counts, and parser errors. GPS data becomes stale after
1.5 seconds without a recognized sentence. No GPS field participates in the
real-time drive or torque-vectoring path.

`monitor gps` provides a 30-second read-only serial diagnostic for UART bytes,
NMEA freshness, fix/position, ground speed, course, altitude, satellites/HDOP,
and parser errors. `monitor dragy` is an alias. GPS course over ground is not
magnetic compass heading.

`monitor gps raw` (`monitor dragy raw`) resets only a diagnostic RAM capture,
waits five seconds while the normal UART parser continues, and prints the most
recent 512 received bytes in hex and ASCII. Use it when byte counts increase
but valid NMEA sentence counts do not, to identify readable text, a baud-rate
mismatch, or a binary/proprietary stream without transmitting to the Dragy.

Compass integration is deferred. As retained discovery groundwork, after the IMU/shared bus initializes and before periodic sensor tasks start,
the firmware scans valid 7-bit I2C addresses once. It reports the address list
over serial and in Wi-Fi telemetry, recognizes `0x6A` as the known IMU, and
flags every other response as a compass candidate. Power the Dragy before the
ESP32 for this discovery pass. Although the MAX-M10S normally uses `0x42`, the
official Dragy harness graphic labels the exposed blue/green pair specifically
as the compass connection, so candidate detection does not exclude `0x42`.
Scanning is intentionally not repeated while driving because probing every
address could interfere with 200 Hz IMU service. An unavailable I2C bus logs a
warning but cannot prevent the independent UART GPS task from starting.

Dragy's public product material labels the compass output but does not publish
the compass device, address, register map, or heading protocol. Its official
developer page requires program registration for integration access.
Discovery is therefore implemented, but heading reads remain disabled until
the registered SDK/device documentation or a positively identified protocol
is available. Do not infer a common magnetometer model from an address alone.

Sources: [Dragy Lite](https://www.godragy.com/dragy-lite/) and
[Dragy developer registration](https://www.godragy.com/dragy-api/), plus the
[MAX-M10S data sheet](https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf).

## 14. Offline CSV Telemetry and Browser Analysis

The custom 4 MB flash layout is:

| Partition | Offset | Size | Purpose |
|---|---:|---:|---|
| NVS | `0x9000` | `0x6000` | calibration, drive configuration, Wi-Fi framework data |
| PHY init | `0xf000` | `0x1000` | radio initialization data |
| factory | `0x10000` | `0x140000` | 1.25 MiB firmware application |
| logdata | `0x150000` | `0x2b0000` | 2.75 MiB raw wear-levelled FAT CSV storage |

The logger is enabled through `RC car offline telemetry log` and defaults to a
1 Hz (1000 ms) sample interval. It runs at priority 1 on CPU0, below the HTTP
task and receiver interrupts. It obtains the public, mutex-protected
`powertrain_remote_snapshot_t` plus the Dragy snapshot. Flash formatting,
stdio, FAT calls, and file export never run in the priority drive task or the
receiver edge interrupt.

`config logging rate <1-50>` changes the sampling rate while `DISARMED`.
The live viewer exposes the same control in its Configuration section. The
validated value is stored as `cfg/log_hz` in NVS, applies without rebooting,
and remains authoritative when the ESP32 is disconnected from Wi-Fi.

The append-only `telemetry.csv` uses schema version 1 and 128 columns. It
records boot/sample identity, state/inhibit data, all receiver inputs,
steering/filter/road-wheel values, every RPM channel and raw measurement,
all IMU axes, TV internals, every ESC throttle/reverse output, Dragy GPS/parser
data, and the configuration context. Text fields are sanitized so a comma,
quote, or newline cannot change column alignment. A boot ID disambiguates the
monotonic uptime reset between power cycles.

Wi-Fi is not part of the record path. Disconnecting every AP client does not
stop or drop CSV samples; after reconnection the on-device HTTP server reads a
fixed starting size in 8 KiB chunks and exposes one browser download.
The writer is flushed before each read. The active file may continue growing,
but a download returns exactly the size captured at its start. Log clear is an
explicit destructive action, requires browser confirmation, and is accepted
only in `DISARMED`.

A CSV containing only the 1,956-byte schema header represents zero samples.
The viewer reports that condition without attempting to plot it. If the FAT
volume simultaneously reports a known capacity but no usable free clusters,
the UI changes the confirmed action to `Recover log storage`. Firmware first
deletes the CSV normally; if the 16 KiB reserve is still unavailable, it
formats only the dedicated `logdata` FAT volume, recreates the header, and
resumes logging. Firmware, NVS, and calibration partitions are outside that
operation.

If append or flush I/O fails, the logger latches a storage-fault flag, records
the underlying `errno`, and stops issuing further writes. Export first attempts
to flush but does not make that failure fatal to recovery: it obtains the
actual committed size with `stat()` and reads through a separate read-only
stream. This permits salvage of an intact prefix after a full or damaged FAT
filesystem. A known capacity with zero free bytes is treated as full. Clear or
reformat remains an explicit destructive recovery step after export.

Internal storage is bounded. The firmware maintains a 16 KiB free-space
reserve and stops recording rather than overwriting old data. It reports
mounted/recording/full/faulted state, the last storage `errno`, sample and error
counters, file size, actual FAT capacity and remaining bytes in live telemetry.
A representative 600-800 byte
row at the default 1 Hz yields about 60-80 minutes, but FAT overhead and numeric widths make
the real value variable; the dashboard derives a live estimate from observed
bytes per sample. A 50 Hz selection has roughly one fifth that retention.
Indefinite lossless logging requires microSD or larger external flash.

Buffered rows are flushed and `fsync`ed at least once per second. This protects
data across Wi-Fi outages, not arbitrary power removal: sudden controller
power loss can lose the final buffered second and can corrupt FAT. Absolute
power-loss durability requires hardware hold-up plus a journaled/transactional
record design. Mount failure currently allows reformatting the dedicated log
partition so a corrupt filesystem does not block the powertrain; that recovery
can erase damaged logs and must be treated separately from network-offline
retention.

Low task priority does not make internal-flash erase/program latency disappear:
ESP32 cache/flash critical sections can delay otherwise unrelated work. Before
powered driving, fill and flush the logger with traction disconnected while
watching powertrain deadline-overrun counters, minimum stack/heap, receiver
capture, PWM stability, supply voltage, and resets. If worst-case control
timing is not clean, disable internal logging or move the recorder to external
storage; do not weaken drive safety timing to accommodate logging.

The local browser analysis tool never uploads the CSV. It parses only complete
128-column rows, joins power-cycle segments into one elapsed-time view,
summarizes duration/peak speed/peak RPM, and plots unit-compatible project
groups: vehicle/GPS speed, wheel RPM, receiver PWM, steering geometry, yaw,
ESC output, TV correction, and IMU acceleration. Rendering may downsample
points for responsiveness; export preserves every recorded row.

## 15. Bench-Test Order

1. Test with traction power disconnected and inspect all nine PWM outputs.
2. Calibrate and monitor each receiver channel.
3. Verify the GPIO32 servo output follows CH1, respects both calibrated
   endpoints, and centers when CH1 disappears. Compare `monitor steering`
   curve angles with physical road-wheel measurements at center, intermediate
   points, and both endpoints; spin both rear wheels together and verify the
   reported speed and steering limit before motor power is connected. Abort if
   the wheel-local sign conversion,
   centered toe-out, or measured angles differ.
4. With permanent latching enabled, verify both CH4 STOP/OFF positions command
   safe ESC output while the armed state remains set, then return RUN with
   throttle neutral and verify ramped recovery. Repeat receiver-loss testing.
5. Disable permanent latching while `DISARMED`; verify STOP and receiver loss
   fully disarm, and confirm a fresh healthy STOP-to-RUN cycle is required.
6. Calibrate one ESC and motor first, then repeat for all four.
7. Confirm wheel direction. Swap any two phase wires on an incorrect motor.
8. Validate each RPM channel against an optical tachometer.
9. Validate IMU yaw sign and stationary bias.
10. With traction power disconnected, associate Wi-Fi, stream the dashboard,
    confirm CH1/CH2/CH4/CH5 and all sensor/output fields, and verify a guarded
    configuration write is accepted only in `DISARMED`. Verify `Load saved
    config`, the drivetrain cycle, and both arm-latch values. Confirm `arm`,
    ordinary `disarm`, `monitor ...`, malformed calibration text, and arbitrary
    commands are rejected over TCP. Exercise receiver/steering/CH4/CH5 capture
    with a scope and confirm NVS changes only after the third validated step;
    verify cancel and the 120-second timeout retain safe `DISARMED` output.
    Check IMU retry with the car stationary. Test browser ESC endpoint/manual
    actions only with the car lifted and restrained and traction power under
    immediate physical control; verify neutral/CH4 guards, reverse-low output,
    cancel, receiver-loss fallback, and the 30-second timeout.
    From `DRIVE_ARMED`, confirm **Disarm for config** immediately commands safe
    ESC outputs, reports `DISARMED`, and unlocks the forms. Confirm CH4 RUN alone
    cannot rearm until the operator physically cycles STOP/OFF-to-RUN/ON.
    Disconnect Wi-Fi for at least one minute, reconnect, export the CSV, and
    confirm timestamps/sample sequencing span the outage without missing
    intervals. Confirm clear is rejected after arming and accepted only in
    `DISARMED`. Remove controller power during a sacrificial run to characterize
    the documented final-buffer/corruption limit; do not use an important log.
11. With the Dragy powered and traction power disconnected, verify GPIO4 sees
    3.3 V UART data at the selected baud, the dashboard receives valid NMEA,
    and the startup I2C list is stable without making the IMU stale. Check that
    connecting GPIO5 does not change boot mode.
12. Drive with torque vectoring disabled and verify equal output behavior.
13. Enable straight assist at low authority and tune yaw response.
14. Enable full assist only after straight behavior is stable.

Keep the wheels clear of the ground for every calibration and first-power
test. Four independent motors can produce substantial force even at low
command.
