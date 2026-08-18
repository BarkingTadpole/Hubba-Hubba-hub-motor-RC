# AGENTS.md

## Purpose

This file is the durable engineering context and working agreement for coding
agents operating in this repository. Read `README.md`, this file, and the
relevant source before changing behavior. Use
`docs/powertrain_architecture_v1.md` for the detailed wiring and operating
procedure, and use `Skywalker_ESC_Manual.pdf` as the local authority for ESC
behavior.

Do not treat planned hardware, inferred signal behavior, or firmware-only
verification as completed physical testing. Keep those categories distinct in
code, documentation, and final reports.

## Project Goal

Build an ESP32-based controller for a 1/7 scale RC car with four independently
driven hub motors. The firmware owns motor power and the steering-servo signal.
Receiver CH1 is validated and relayed through the ESP32; torque vectoring still
adjusts motor power only and does not alter the requested steering angle.

The immediate torque-vectoring goals are:

- Keep the car tracking straight when steering input is centered.
- Improve cornering performance when steering input is present.
- Preserve a safe equal-output fallback whenever torque-vectoring sensors or
  calibrations are unavailable.
- Keep torque vectoring disabled by default until hardware inputs are
  validated and the user deliberately enables it.

## Hardware

- Controller: ESP32 DOIT DevKit V1 with ESP32-WROOM-32.
- Framework target: ESP-IDF 6.x, target `esp32`.
- Motors: 4x Hobbywing Skywalker 2820SL 550KV brushless motors.
- Motor construction: `12N14P`, meaning 14 magnetic poles and 7 pole pairs.
- ESCs: 4x Hobbywing Skywalker 50A V2 with separate throttle and yellow
  reverse/brake signal wires.
- Transmitter: Radiolink RC6GS V3.
- Receiver: Radiolink R7FG.
- IMU: Adafruit STEMMA QT ISM330DHCX.
- Speed sensors: 4x Hobbywing HW86060041 phase-derived RPM sensors, one per
  motor.

Confirm the exact ESC SKU before choosing traction-battery voltage. The
2820SL 550KV motor is listed as a 6S motor, but the regular Skywalker 50A V2
and Skywalker 50A-6S V2 do not necessarily have the same battery rating.

## Current Pin Plan

`main/pin_config.h` is the single source of truth. Update it and the
architecture document together whenever wiring changes.

### Radiolink Receiver Inputs

| Function | R7FG channel | ESP32 pin |
|---|---:|---:|
| Steering command | CH1 | GPIO16 |
| Throttle | CH2 | GPIO25 |
| Remote shutdown RUN/STOP | CH4 | GPIO33 |
| Torque-vectoring mode | CH5 | GPIO26 |

The former physical arm switch on GPIO33 has been replaced by receiver CH4
PWM. Do not reintroduce digital active-low switch logic or an internal pull-up
on GPIO33 unless the hardware architecture is deliberately changed again.

### Steering Servo Output

| Function | ESP32 pin |
|---|---:|
| 50 Hz servo PWM | GPIO32 |

GPIO16 is the conditioned receiver CH1 input. GPIO32 is the only signal
connection to the servo. The two driven signals must never be tied together.

### ESC Outputs

| Wheel | Throttle | Reverse/direction |
|---|---:|---:|
| Front left | GPIO13 | GPIO2 |
| Front right | GPIO14 | GPIO27 |
| Rear left | GPIO21 | GPIO19 |
| Rear right | GPIO18 | GPIO17 |

All eight high-speed LEDC channels are consumed by these outputs. The steering
servo uses low-speed LEDC channel 0, which is a separate hardware channel on
the ESP32. Keep the ESC signals independent even when they carry equal
commands, because per-wheel throttle control is required for torque vectoring.

GPIO2 is an ESP32 boot-strapping pin. Its ESC input should be high impedance,
but do not add a pull to this line. If ESC-connected boot reliability is poor,
relocate this signal to a non-strapping output.

### RPM Inputs

| Wheel | ESP32 pin |
|---|---:|
| Front left | GPIO34 |
| Front right | GPIO35 |
| Rear left | GPIO36 |
| Rear right | GPIO39 |

GPIO34 through GPIO39 are input-only and have no internal pull-ups.

### IMU

| Signal | ESP32 pin |
|---|---:|
| I2C SDA | GPIO23 |
| I2C SCL | GPIO22 |
| INT1, unused | Not connected |
| INT2, unused | Not connected |

## Electrical Assumptions And Wiring

- Receiver ground, ESP32 ground, RPM sensor grounds, IMU ground, and all ESC
  signal grounds must share a common reference.
- ESP32 GPIOs are 3.3 V inputs. Condition any receiver or sensor output that
  can exceed 3.3 V.
- The receiver throttle signal already has a 5 V to 3.3 V divider in the
  physical setup. Apply equivalent protection to steering, CH4, and CH5 if
  those receiver outputs exceed 3.3 V.
- Earlier multimeter readings of roughly 0.22 V to 0.385 V on receiver
  throttle were DC averages of a PWM waveform, not the waveform's logic-high
  voltage. Diagnose PWM with pulse capture or an oscilloscope, not DC voltage.
- Route conditioned receiver CH1 only to GPIO16 and route GPIO32 only to the
  servo signal. Power the servo from the BEC/receiver rail, never from an ESP32
  GPIO, and keep grounds common. Verify that the servo accepts 3.3 V logic or
  use a proper logic-level buffer.
- Keep the R7FG built-in gyro disabled initially. Its steering correction
  would form a second feedback loop and obscure ISM330DHCX torque-vectoring
  tuning.
- Do not parallel all four ESC BEC red wires unless the ESC manufacturer
  explicitly allows it. Use one BEC or a dedicated regulated supply while
  keeping grounds common.

### Planned RPM Sensor Wiring

For each HW86060041 sensor:

- Phase leads A and B connect to any two motor phase wires; polarity does not
  matter.
- Red is intended to use a regulated 5 V supply.
- Black connects to common ground.
- White is the RPM pulse output and must be conditioned before the ESP32.

The user's current divider plan is:

```text
sensor white --- 1 kOhm ---+--- ESP32 RPM GPIO
                            |
                           2 kOhm
                            |
                           GND
```

This orientation gives approximately 3.33 V from an exact 5.0 V input. The
opposite orientation gives only about 1.67 V and is incorrect. Before relying
on it, measure the actual 5 V rail and the divided waveform with an
oscilloscope. A rail above roughly 5.4 V leaves inadequate ESP32 input margin.
Higher-value alternatives such as 10 kOhm over 15 kOhm reduce loading and
produce about 3.0 V from 5.0 V.

Hobbywing documentation is internally awkward here: the wiring instructions
mention 3.3 V or 5 V sensor power, while the specification lists a 3.5 V
minimum. Supplying the sensors from ESP32 3V3 may work on a bench but is not
the preferred final arrangement because it has poor voltage and noise margin.

## Known Bench History

- Current measured receiver defaults are centralized in
  `main/default_config.h`:
  - Throttle: neutral `1514 us`, full `978 us`, reverse `2044 us`.
  - Steering: center `1518 us`, full left `2050 us`, full right `987 us`.
  - Torque vectoring: OFF `2047 us`, STRAIGHT `1513 us`, FULL `981 us`.
  - Shutdown: RUN/ON `983 us`, STOP/OFF 1 `2049 us`, STOP/OFF 2 `1515 us`.
- The receiver was observed to output approximately `1565 us` on throttle
  when the transmitter was off or disconnected.
- The optional transmitter-off pulse detector retains `1565 +/- 10 us` as
  its default values but is disabled by default.
- Reverse output is deliberately limited to 10 percent by default.
- Rear-left motor direction was previously observed correct while the other
  three were reversed. The preferred correction is swapping any two phase
  wires on each incorrect motor, not adding individual software inversion.
  Verify current wiring before assuming this observation is still true.
- Brake/reverse response was previously reported as delayed and sensitive.
  The ESC's reverse mode intentionally stops before reversing, and the
  firmware also provides a direction-change interlock. Do not remove either
  behavior casually.
- Earlier physical-switch monitoring problems came from mixing digital and
  ADC interpretations on GPIO33. That design is obsolete now that GPIO33 is
  receiver PWM CH4.
- On 2026-08-11, the user observed a repeatable boot panic at
  `esc_output.c:set_pin()` on the first `ledc_set_duty_and_update()` call.
  ESP-IDF 6 requires its LEDC fade service for that thread-safe API even when
  updates are immediate. Firmware now installs the service after configuring
  the eight safe ESC channels and before the first update. The correction has
  code/build validation only and must be reflashed and checked with traction
  power disconnected before motor power is restored. No pin changed.

## ESC Signal Behavior

Use the local ESC manual as authority. The current firmware assumes ESC
`Brake Type = Reverse`, not Linear Reverse Brake.

- Default throttle range is `1100 us` to `1940 us`.
- The white ESC wire receives throttle magnitude.
- The yellow wire receives direction/reverse state.
- Reverse low/default direction is `1100 us`.
- Reverse high/reverse direction is `1940 us`.
- Reverse must remain low during boot and throttle endpoint calibration.
- A missing throttle or reverse signal may trigger the ESC's signal-loss
  protection, so the ESP32 continues outputting valid safe PWM when disarmed.
- Changing direction must ramp to throttle minimum before changing yellow
  reverse state.

Safe output always means all four throttle outputs at `1100 us` and all four
reverse outputs at `1100 us`.

## Receiver Interpretation

Receiver throttle is centered, while the aircraft-style ESC throttle input
uses minimum as stopped and maximum as magnitude. Receiver calibration learns
neutral, full throttle, full reverse, and polarity.

- Input pulse acceptance range: `800 us` to `2200 us`.
- Receiver input timeout: 100 ms.
- Receiver pulse widths use four priority-3 GPIO edge interrupts installed on
  CPU0. The powertrain task is pinned to CPU1, where it updates the high-speed
  LEDC ESC outputs; the steering servo uses the separate low-speed LEDC group.
- MCPWM and RMT receiver-capture experiments were rejected. MCPWM starved
  channels when throttle changed, while RMT produced unsafe intermittent drive
  and ignored shutdown during a bench incident.
- Default throttle calibration: `978/1514/2044 us` for
  full-forward/neutral/full-reverse.
- Default throttle neutral deadband: `80 us`.
- Throttle magnitude uses a quadratic response curve.
- Default maximum reverse magnitude: 10 percent of ESC span.
- Steering valid-command smoothing is a first-order filter with a default
  `60 ms` time constant and a configurable `0-500 ms` range. Missing CH1 or a
  confirmed throttle failsafe centers immediately rather than filtering the
  safety response.
- While centered, steering rejects one isolated CH1 frame outside the saved
  deadband. Two distinct consecutive frames on the same side release the
  center guard, adding approximately 20 ms only when steering first begins.
  The first centered frame commands center and two centered frames relock it.
  Missing CH1, throttle failsafe, startup, and steering calibration recenter
  the guard. `status` and `monitor steering` expose its state and rejected
  neutral-spike count.
- Steering trim is stored in tenths of a degree over `+/-15 degrees`, using a
  nominal `1000 us = 90 degrees` command-space conversion. It is not a
  road-wheel angle calibration; trim and smoothing remain clamped to saved
  steering endpoints. Trim must leave the center strictly inside both
  endpoints and is reset if a new steering calibration makes it invalid.
- The corrected 45-point steering table covers `-45` to `+45` servo-command
  degrees at nonuniform intervals and provides separate LF/RF road-wheel
  angles. Source servo-positive steers left; each source wheel angle is
  wheel-local, with positive pointing outward and negative pointing inward.
  Runtime converts both angles to a common positive-right frame and uses
  linear interpolation between bounding samples. The centered source values
  `LF=+1.64` and `RF=+1.64 degrees` describe toe-out; runtime values are
  `LF=-1.64`, `RF=+1.64`, and zero mean. The curve shapes the torque-vectoring
  steering input only; it never changes servo pass-through.

CH4 is a three-position receiver shutdown control with one learned RUN/ON
position and two learned STOP/OFF positions. Arming requires:

- Saved receiver throttle calibration.
- Saved CH4 RUN/ON and two-position STOP/OFF calibration.
- Valid CH2 and CH4 PWM.
- Neutral throttle.
- CH4 in the learned RUN/ON position.
- Continuous valid RUN/ON and neutral throttle for 250 ms before drive entry.
- A healthy observation of either learned STOP/OFF position since boot or the
  last safety event.

Normal operation requires a deliberate CH4 STOP/OFF-to-RUN/ON transition.
Either learned STOP/OFF position disarms immediately. Missing CH4 PWM disarms
after the 100 ms receiver timeout. A pulse outside all three calibrated
positions must persist for 60 ms before disarming, which rejects a single
malformed PWM frame without delaying a recognized STOP command. After signal
loss, or after the optional throttle-pulse detector triggers, a healthy
OFF-to-ON cycle permits rearming without a laptop.

Each learned CH4 position has a `+/-125 us` recognition window. An
unrecognized pulse is safe-off but does not count as the healthy STOP/OFF
observation needed to unlock rearming.

Recommended R7FG failsafe positions:

- CH1: steering center.
- CH2: neutral throttle.
- CH4: either learned STOP/OFF position.
- CH5: OFF.

The fixed CH2 pulse detector is disabled by default. `config failsafe
<pulse_us> <window_us>` configures and enables it; `config failsafe off`
disables it. When enabled, a matching pulse applies safe output immediately
and disarms after a 60 ms confirmation. Complete PWM loss is declared after
100 ms regardless of this option.

## Drive Timing

- Main drive update interval: 20 ms.
- Pre-arm stable RUN/neutral qualification: 250 ms.
- Missed sensor or drive deadlines reset the periodic schedule and force a
  one-tick block; `status` exposes cumulative overrun counts, maximum observed
  execution time, minimum free task stack, and heap headroom.
- Acceleration ramp: 12 us of ESC pulse per update.
- Deceleration ramp: 100 us per update.
- Direction-change zero hold: 120 ms.
- ESC calibration inactivity timeout: 30 seconds.
- Monitor command duration: 30 seconds.

Do not weaken these values without documenting the reason and considering the
combined torque of four motors.

## RPM Interpretation

The HW86060041 observes voltage changes between two motor phases and emits an
RPM signal. The firmware counts rising edges with one ESP32 PCNT unit per
wheel.

The conversion is stored and configured explicitly as pulses per mechanical
revolution. On 2026-08-12, the user confirmed by direct oscilloscope edge
count that one mechanical wheel/motor revolution produces seven rising edges:

```text
pulses_per_revolution = 7
mechanical_rpm = pulse_frequency_hz * 60 / pulses_per_revolution

for the 14-pole motors:
mechanical_rpm = pulse_frequency_hz * 60 / 7
```

Implementation details:

- Default PPR: 7, confirmed for the tested motor/sensor by direct rising-edge
  count over one mechanical revolution.
- Rising edges only are counted.
- Sampling interval: 20 ms.
- Adaptive rolling window: at least five samples (approximately 100 ms), up
  to 25 samples (approximately 500 ms) until four edges are available.
- A speed channel becomes invalid if it sees no pulse for 250 ms.
- `monitor rpm` reports raw frequency, edge count, window duration, and RPM.
- `config rpm ppr <1-120>` changes the authoritative conversion.
- Legacy `config rpm poles <even 2-60>` also sets PPR to half the pole count.

The Hobbywing sensor documentation does not specify pulse count per mechanical
revolution, but the direct measurement confirms that the compiled `7 PPR`
conversion factor is correct for the tested hardware. All four channels still
need clean-waveform and optical-tachometer checks at several speeds before
torque vectoring is trusted; those tests detect missed/extra edges, noise, or
channel-specific faults rather than establish the already measured ratio.

At 7 PPR, count quantization is approximately 85.7 RPM over 100 ms and
17.1 RPM over 500 ms; the previously observed approximately 86 RPM low-end
step is consistent with this quantization. Torque vectoring requires at least
16.7 Hz raw sensor frequency, matching the sensor's documented minimum for
its 2-pole reference. Because the motors are hub motors, motor RPM and wheel
RPM are the same.

## IMU Behavior

- I2C address: `0x6A`.
- Expected `WHO_AM_I`: `0x6B`.
- I2C speed: 400 kHz.
- Accelerometer configuration: 208 Hz, +/-16 g.
- Gyroscope configuration: 208 Hz, +/-1000 degrees/second.
- Firmware polling interval: 5 ms, approximately 200 Hz.
- FreeRTOS tick rate: 1000 Hz so the 5 ms polling interval is representable.
- IMU data is stale after 100 ms without a successful read.
- Yaw-rate sign is configurable as `1` or `-1`.
- Startup assumes the car is level and completely stationary. After the IMU
  initializes, firmware automatically samples gyro yaw bias for five seconds
  before starting the powertrain and sensor tasks. Safe ESC PWM is already
  active and drive cannot arm during this blocking startup interval.
- `cal imu` remains available as a two-second disarmed retry. Starting any
  calibration invalidates the prior current-boot bias; a failed attempt leaves
  torque vectoring unavailable rather than retaining a stale bias.
- Torque vectoring requires valid IMU data and a successful current-boot bias
  calibration.
- The DOIT DevKit's controllable onboard LED is GPIO2, already assigned to the
  front-left ESC reverse signal. Firmware must not blink it because doing so
  would corrupt a safety-critical ESC command. The hardwired power LED is not
  software-controllable. Completion is reported over serial; an LED indicator
  requires a separate free GPIO and external LED or a deliberate pin-plan
  revision.

Mount the IMU rigidly near the chassis centerline. The current physical
orientation has sensor `+X` pointing toward the rear of the car and sensor
`+Y` pointing toward the right. Avoid soft foam that adds feedback delay.
This mounting reference does not replace the hand-rotation yaw-sign check.

## Torque Vectoring

Torque vectoring is implemented but defaults to disabled. CH5 has three
calibrated positions:

- OFF: four equal throttle commands.
- STRAIGHT: zero-yaw and equal-left/right-RPM correction while steering is
  within 6 percent of center.
- FULL: empirical steering-based yaw and side-RPM targets for turns, including
  straight correction around center.

Each CH5 mode requires a pulse inside its learned `+/-125 us` window. A pulse
outside all three windows selects OFF rather than the nearest active mode.

The controller operates only in forward drive. Reverse always uses equal
throttle. It requires valid RPM from all four wheels, valid and bias-calibrated
IMU data, sufficient speed, saved steering/CH5 calibration, and explicit
configuration enablement. Invalid data must produce equal outputs, not stale
corrections.

Positive steering means right. Positive side correction adds throttle to the
left pair and subtracts the same amount from the right pair:

```text
FL = base + correction
FR = base - correction
RL = base + correction
RR = base - correction
```

Core empirical model:

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

Default configuration:

- Authority: `+/-10%` of full ESC throttle span.
- Turn yaw gain: `180 deg/s`.
- Turn side-RPM gain: `0.20`.
- Yaw proportional gain: `0.00025`.
- Yaw integral gain: `0.00004`.
- Side-RPM proportional gain: `0.20`.
- IMU yaw sign: `1`.

Corrections are equal and opposite to preserve requested mean power. Available
authority naturally falls near zero and full throttle because both sides must
stay inside the ESC output limits. The yaw integrator is conditionally held
when saturation and error would wind it farther into the limit. Non-finite
configuration/input values, raw RPM below 16.7 Hz, or invalid/stale sensors
reset the controller and produce equal outputs.

`tv enable` is guarded. During the current boot, the automatic startup IMU
calibration or a manual `cal imu` retry must succeed, and every RPM channel
must have produced a valid pulse at least once. Use `monitor rpm` and rotate
each wheel before enabling. Even if enablement is persisted in NVS, the
runtime controller remains inactive after reboot until the IMU is recalibrated
and live sensor data is valid.

Tune at low speed and begin around 2 to 5 percent authority. Verify yaw sign
by rotating the car right by hand while running `monitor imu`; rightward yaw
should report positive. Use `config imu yaw-sign -1` if necessary.

## Serial Interface

USB serial runs through the ESP32 devkit programming connection at 115200
baud.

### General

```text
status
arm
disarm
help
```

The receiver CH4 transition is the normal arming method. The CLI `arm` command
still respects receiver calibration, CH4 RUN, neutral throttle, and the
STOP-cycle requirement.

### Calibration

```text
cal receiver
cal steering
cal arm
cal tv
cal imu
cal esc arm
cal esc max
cal esc min
cal manual
cal cancel
```

Receiver, steering, CH4, and CH5 calibration prompts the user to hold each
position, press Enter, and then averages approximately 1.5 seconds of PWM.
CH4 calibration captures RUN/ON, STOP/OFF 1, and STOP/OFF 2. Existing saved
two-position CH4 calibration is intentionally treated as uncalibrated until
`cal arm` is run again.
Values persist in NVS. IMU bias is intentionally current-boot state rather
than persistent calibration. The normal boot performs a five-second automatic
bias calibration; `cal imu` is a two-second retry if startup calibration fails
or the vehicle moved during startup.

Guided ESC endpoint calibration:

1. Lift and restrain the car.
2. Keep ESC traction power disconnected.
3. Keep CH4 in either learned STOP/OFF position and throttle neutral.
4. Send `cal esc arm`.
5. Send `cal esc max`; all throttle outputs become 1940 us and reverse stays
   1100 us.
6. Power the ESCs and wait for maximum-endpoint beeps.
7. Send `cal esc min` within the manual's five-second window.
8. Wait for acceptance and ready beeps.
9. Send `cal cancel`.

Leaving both CH4 STOP/OFF positions, losing a required receiver signal,
cancellation, or timeout must restore safe output. An enabled and matching
throttle-pulse detector does the same. `cal manual` directly relays receiver
throttle, clamped to the ESC's `1100-1940 us` range, to all four ESC throttle
outputs for at most 30 seconds while holding all reverse outputs low. ESC
endpoint calibration and manual relay both require saved CH4 calibration and
a currently recognized STOP/OFF position.

### Monitoring

```text
monitor throttle
monitor steering
monitor steering trim <-15..15>
monitor arm
monitor tv
monitor rpm
monitor imu
monitor vector
```

Monitoring blocks the serial CLI temporarily but does not stop the controller
or sensor tasks. Steering setup requires a verified trim value in the NVS
drive configuration; `0.0 degrees` is valid if no correction is needed.
`monitor steering trim <degrees>` is accepted only while disarmed, saves the
validated trim to NVS, waits for it to be applied, and then starts the normal
steering monitor. A failed NVS write rolls the in-memory value back.

### Configuration

```text
config reverse <0-100>
config failsafe off | <pulse_us> <window_us>
config rpm poles <even 2-60>
config rpm ppr <1-120>
config steering trim <-15..15>
config steering smoothing <0-500>
config tv authority <0-25>
config tv gains <yaw_gain_dps> <turn_rpm_gain> <yaw_kp> <yaw_ki> <rpm_kp>
config imu yaw-sign <-1|1>
tv enable
tv disable
```

Configuration changes are accepted only while disarmed and persist in NVS.

## Firmware Structure

- `main/app_main.c`: process I/O setup, controller initialization, and task
  startup only.
- `main/cli.c`: serial input editing, parsing, validation, and dispatch.
- `main/default_config.h`: compiled receiver defaults and IMU mounting
  reference.
- `main/config_store.c`: defaults, validation, backward-compatible NVS keys,
  and persistence.
- `main/rc_input.c`: four priority-3 GPIO edge-interrupt PWM inputs.
- `main/esc_output.c`: eight independent 50 Hz LEDC outputs.
- `main/servo_output.c`: independent 50 Hz steering PWM on GPIO32.
- `main/steering_center_guard.c`: distinct-frame neutral steering validation
  and rejected-spike telemetry.
- `main/steering_curve.c`: source-backed 45-point LF/RF road-wheel curve and
  deterministic piecewise-linear interpolation.
- `main/rpm_sensor.c`: four PCNT units, rolling windows, and RPM conversion.
- `main/imu_sensor.c`: minimal ISM330DHCX I2C driver and bias calibration.
- `main/torque_vectoring.c`: sensor feedback, controller state, authority
  limits, and balanced wheel corrections.
- `main/powertrain_controller.c`: system state, arming, failsafe, drive
  mapping, direction interlock, calibration workflows, monitoring, and task
  scheduling.
- `main/pin_config.h`: physical pin assignments only.
- `main/include/`: public module interfaces and shared powertrain types.

Do not collapse this structure back into one monolithic source file. Prefer
module APIs over reaching into another module's state. Add a new abstraction
only when it creates a real ownership boundary or removes meaningful
duplication.

## Persistent Configuration

NVS namespaces are `cal` for calibration and `cfg` for drive configuration.
Existing throttle and drive keys must remain backward compatible unless a
deliberate migration is implemented. Validate every loaded pulse width,
endpoint relationship, percentage, pole count, and gain before applying it.
Invalid or missing NVS data must fall back to safe defaults and must not make
the car arm unexpectedly.

## Safety Invariants

These are requirements, not suggestions:

- Safe ESC output is established before receiver, NVS, sensor, or CLI
  initialization can produce drive behavior.
- Boot never enters drive or ESC calibration automatically.
- No drive throttle is allowed without saved throttle/CH4 calibration, valid
  receiver signals, CH4 RUN, and neutral throttle at arming.
- Either CH4 STOP/OFF position immediately disarms. Missing CH4 disarms after
  the 100 ms input timeout; an unrecognized CH4 pulse disarms after 60 ms of
  continuous confirmation.
- CH4 is a software shutdown, not a hardware emergency stop. Powered testing
  requires independent, immediately accessible traction-power isolation.
- Receiver throttle loss or calibration timeout restores safe outputs. An
  enabled configured throttle-pulse detector does the same when it matches.
- The steering output starts centered, follows only valid CH1 pulses within
  calibrated endpoints after the two-frame neutral-release guard, and centers
  on CH1 loss. It also centers on a matching throttle pulse when the optional
  detector is enabled.
- Rearming after a safety event requires a healthy STOP/OFF-to-RUN/ON cycle and does
  not require a laptop.
- Reverse magnitude remains limited by configuration, default 10 percent.
- Direction changes pass through throttle minimum before reverse state
  changes.
- Reverse outputs remain low through ESC endpoint calibration.
- Torque vectoring defaults disabled and fails back to four equal commands on
  invalid sensor data.
- Do not add per-wheel software direction inversion as a substitute for
  correcting motor phase wiring.
- Do not test first-power behavior with wheels on the ground.

Safety-related changes require focused review, documentation updates, and
bench verification with traction power controlled.

## Build And Verification

Use an ESP-IDF 6.x terminal:

```powershell
idf.py build
idf.py flash monitor
```

Do not flash hardware unless the user explicitly requests it. The project has
`MINIMAL_BUILD` enabled in the root `CMakeLists.txt`. A first build after
component or configuration changes can rebuild hundreds of ESP-IDF objects;
subsequent incremental builds should be much faster.

The `esp32_pin_map` build target validates `main/pin_config.h` and generates a
self-contained `build/esp32_pin_map.svg`. It reads configuration and writes
only into `build/`; it is not compiled into or flashed with the firmware. The
standalone `tools/esp32_pin_map/open_pin_map.ps1` launcher regenerates and
opens the same artifact without building firmware.

The modular firmware was last confirmed to compile with ESP-IDF 6.0.1 for
`esp32` on 2026-08-12. The recorded application binary was `0x37720` bytes
with 78 percent of the 1 MiB application partition free. Treat size figures as
a historical baseline and report current build output after new changes.

Host tests cover balanced correction, the RPM frequency guard, non-finite
fallback, saturation anti-windup, steering-curve endpoints/signs,
interpolation, saturation, and invalid input. They are hardware-independent
logic tests, not ESP32 peripheral or vehicle tests. The
obsolete `pytest_hello_world.py` file was intentionally deleted. At minimum,
run the host logic test and a full firmware build after related code changes.
Scale bench testing with risk.

Required bench-test progression for major control changes:

1. Traction power disconnected; inspect all nine outputs with a scope or
   signal tester.
2. Calibrate and monitor all receiver inputs and verify the GPIO32 servo
   output follows CH1 and centers on signal loss.
3. Verify both CH4 STOP/OFF positions and receiver-loss behavior from every
   relevant state.
4. Calibrate and test one ESC/motor before all four.
5. Verify every wheel direction physically.
6. Validate each RPM channel and compare RPM against an optical tachometer.
7. Validate IMU sign, stationary bias, and stale-data fallback.
8. Drive with torque vectoring disabled.
9. Tune straight assist at low authority.
10. Enable full assist only after straight behavior is stable.

Never report a successful build as successful hardware validation.

## C And ESP-IDF Conventions

- Follow the formatting and naming style already present in nearby modules.
- Use fixed-width integer types for pulses, counters, timestamps, and NVS
  representations.
- Check and propagate ESP-IDF return values during initialization.
- Keep real-time drive paths deterministic; avoid dynamic allocation,
  blocking serial I/O, or NVS writes in the 20 ms control loop.
- Protect cross-task snapshots and mutable peripheral state appropriately.
- Use hardware peripherals such as PCNT, LEDC, and I2C drivers rather than
  timing-sensitive software loops.
- Keep comments focused on hardware assumptions, safety reasoning, and
  non-obvious controller behavior.
- Keep default values centralized with their owning module.
- Preserve ASCII unless an existing file clearly requires another encoding.

## Documentation Requirements

Update `README.md` and `docs/powertrain_architecture_v1.md` whenever changing:

- Pin assignments or electrical conditioning.
- Receiver channel assignments.
- Serial commands or calibration steps.
- ESC pulse limits or signal interpretation.
- Arming, shutdown, failsafe, or timeout behavior.
- RPM conversion assumptions.
- IMU orientation or calibration behavior.
- Torque-vectoring modes, equations, guards, defaults, or tuning ranges.

Keep the short startup path in the README and detailed engineering reasoning
in the architecture document. Do not duplicate stale procedures across files.

## Repository Hygiene

- The worktree may contain unrelated user changes, including deleted or
  modified mechanical model files. Never restore, remove, or rewrite them
  unless explicitly requested.
- Do not edit generated files in `build/`.
- Do not use destructive Git commands to clean the worktree.
- Keep changes scoped to the requested behavior.
- Do not commit secrets, personal serial-port identifiers, or credentials.
- Do not create placeholder tests or example files that are not used.

## Current Validation Gaps

The following must remain visible until physically resolved:

- New RPM sensor wiring and 1 kOhm/2 kOhm dividers have not been confirmed in
  firmware on all four wheels.
- A direct 2026-08-12 scope count confirmed `7 PPR` for the tested
  motor/sensor. RPM accuracy, clean counting, and the same ratio on all four
  installed channels still need optical-tachometer validation across speed;
  the adaptive window, raw telemetry, and 16.7 Hz control guard otherwise
  have only logic/build validation.
- ISM330DHCX orientation, yaw sign, bias behavior, and vibration performance
  need on-car validation.
- CH1 input, GPIO32 servo output, CH4 shutdown, and CH5 mode PWM calibrations
  need validation with the actual R7FG and steering servo. The new steering
  trim, smoothing, wheel-local curve signs, centered toe-out, and road-wheel
  angles also require scope and on-car validation. The supplied curve is input
  data, not physical validation observed by the firmware review.
- The CPU-isolated GPIO receiver capture and high-speed LEDC ESC output
  rollback and the 2026-08-11 LEDC fade-service startup fix compile but still
  require traction-disconnected validation while changing throttle and
  commanding CH4 STOP. Do not power the motors until this passes repeatedly
  without stale inputs, servo jolts, boot loops, or task resets.
- Torque vectoring has compiled but has not been tuned or proven on the car.
- Runtime `status` timing, stack, and heap instrumentation has compiled but no
  worst-case hardware data has been captured yet.
- Boot behavior with the ESC input connected to GPIO2 should be watched
  because it is a strapping pin.
- Exact ESC SKU and permitted traction-battery cell count must be confirmed
  before high-voltage operation.

Resolve these with measurements and update this file and the architecture
document when facts replace assumptions.
