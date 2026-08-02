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

Build an ESP32-based powertrain controller for a 1/7 scale RC car with four
independently driven hub motors. The firmware owns motor power only. The
steering servo remains connected directly to the RC receiver and is not
controlled by the ESP32 in this version.

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
| Steering observation | CH1 | GPIO32 |
| Throttle | CH2 | GPIO25 |
| Remote shutdown RUN/STOP | CH4 | GPIO33 |
| Torque-vectoring mode | CH5 | GPIO26 |

The former physical arm switch on GPIO33 has been replaced by receiver CH4
PWM. Do not reintroduce digital active-low switch logic or an internal pull-up
on GPIO33 unless the hardware architecture is deliberately changed again.

### ESC Outputs

| Wheel | Throttle | Reverse/direction |
|---|---:|---:|
| Front left | GPIO13 | GPIO2 |
| Front right | GPIO14 | GPIO27 |
| Rear left | GPIO21 | GPIO19 |
| Rear right | GPIO18 | GPIO17 |

All eight LEDC channels are currently consumed by these outputs. Keep the
signals independent even when they carry equal commands, because per-wheel
throttle control is required for torque vectoring.

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
- The steering input is a high-impedance Y tap from receiver CH1. The original
  CH1 signal continues directly to the servo. Never connect servo power to an
  ESP32 GPIO.
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

- Measured receiver throttle positions were approximately:
  - Full throttle: `1750 us`.
  - Neutral: `1250 us`.
  - Full reverse: `1000 us`.
- The receiver was observed to output approximately `1565 us` on throttle
  when the transmitter was off or disconnected.
- The configured default transmitter-off detector is therefore
  `1565 +/- 10 us`.
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
- Default throttle calibration: `1750/1250/1000 us` for
  full-forward/neutral/full-reverse.
- Default throttle neutral deadband: `80 us`.
- Throttle magnitude uses a quadratic response curve.
- Default maximum reverse magnitude: 10 percent of ESC span.

CH4 is a two-position receiver shutdown control. Arming requires:

- Saved receiver throttle calibration.
- Saved CH4 RUN/STOP calibration.
- Valid CH2 and CH4 PWM.
- Neutral throttle.
- CH4 in RUN.
- A healthy STOP observation since boot or the last safety event.

Normal operation requires a deliberate CH4 STOP-to-RUN transition. CH4 STOP
or missing CH4 PWM disarms immediately. After signal loss or the configured
throttle failsafe pulse, a healthy STOP-to-RUN cycle permits rearming without
a laptop. A receiver failsafe STOP pulse observed while CH2 is still at the
configured transmitter-off pulse must not silently unlock rearming.

Recommended R7FG failsafe positions:

- CH2: the known transmitter-off/safe pulse.
- CH4: STOP.
- CH5: OFF.

When CH2 matches the configured failsafe window, firmware applies safe output
immediately and disarms after a 60 ms confirmation. Complete PWM loss is
declared after 100 ms.

## Drive Timing

- Main drive update interval: 20 ms.
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

The current conversion assumes one pulse cycle per electrical revolution:

```text
pole_pairs = motor_poles / 2
mechanical_rpm = pulse_frequency_hz * 60 / pole_pairs

for the 14-pole motors:
mechanical_rpm = pulse_frequency_hz * 60 / 7
```

Implementation details:

- Default motor pole count: 14.
- Rising edges only are counted.
- Sampling interval: 20 ms.
- Rolling window: five samples, approximately 100 ms.
- A speed channel becomes invalid if it sees no pulse for 250 ms.
- `config rpm poles <even 2-60>` changes the conversion.

The Hobbywing sensor documentation does not clearly specify pulse count per
mechanical revolution. This formula is an informed assumption and must be
checked against an optical tachometer at several speeds before torque
vectoring is trusted. A constant factor error means the pulse/pole assumption
is wrong and must not be hidden by controller tuning.

Because the motors are hub motors, motor RPM and wheel RPM are the same.

## IMU Behavior

- I2C address: `0x6A`.
- Expected `WHO_AM_I`: `0x6B`.
- I2C speed: 400 kHz.
- Accelerometer configuration: 208 Hz, +/-16 g.
- Gyroscope configuration: 208 Hz, +/-1000 degrees/second.
- Firmware polling interval: 5 ms, approximately 200 Hz.
- IMU data is stale after 100 ms without a successful read.
- Yaw-rate sign is configurable as `1` or `-1`.
- Gyro yaw bias is calibrated for the current boot with `cal imu` while the
  car is level and completely still.
- Torque vectoring requires valid IMU data and a successful current-boot bias
  calibration.

Mount the IMU rigidly near the chassis centerline with axes aligned to the
car. Avoid soft foam that adds feedback delay.

## Torque Vectoring

Torque vectoring is implemented but defaults to disabled. CH5 has three
calibrated positions:

- OFF: four equal throttle commands.
- STRAIGHT: zero-yaw and equal-left/right-RPM correction while steering is
  within 6 percent of center.
- FULL: empirical steering-based yaw and side-RPM targets for turns, including
  straight correction around center.

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
target_yaw = steering * turn_yaw_gain * base_throttle
desired_side_rpm_delta = steering * turn_rpm_gain

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
stay inside the ESC output limits.

`tv enable` is guarded. During the current boot, `cal imu` must succeed and
every RPM channel must have produced a valid pulse at least once. Use
`monitor rpm` and rotate each wheel before enabling. Even if enablement is
persisted in NVS, the runtime controller remains inactive after reboot until
the IMU is recalibrated and live sensor data is valid.

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
Values persist in NVS. IMU bias is intentionally current-boot state rather
than persistent calibration.

Guided ESC endpoint calibration:

1. Lift and restrain the car.
2. Keep ESC traction power disconnected.
3. Keep CH4 STOP and throttle neutral.
4. Send `cal esc arm`.
5. Send `cal esc max`; all throttle outputs become 1940 us and reverse stays
   1100 us.
6. Power the ESCs and wait for maximum-endpoint beeps.
7. Send `cal esc min` within the manual's five-second window.
8. Wait for acceptance and ready beeps.
9. Send `cal cancel`.

Leaving CH4 STOP, losing a required receiver signal, matching the throttle
failsafe, cancellation, or timeout must restore safe output. `cal manual`
directly relays receiver throttle to all four ESC throttle outputs for at most
30 seconds while holding all reverse outputs low.

### Monitoring

```text
monitor throttle
monitor steering
monitor arm
monitor tv
monitor rpm
monitor imu
monitor vector
```

Monitoring blocks the serial CLI temporarily but does not stop the controller
or sensor tasks.

### Configuration

```text
config reverse <0-100>
config failsafe <pulse_us> <window_us>
config rpm poles <even 2-60>
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
- `main/config_store.c`: defaults, validation, backward-compatible NVS keys,
  and persistence.
- `main/rc_input.c`: four GPIO edge-interrupt PWM inputs.
- `main/esc_output.c`: eight independent 50 Hz LEDC outputs.
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
- CH4 STOP or missing CH4 immediately disarms.
- Receiver throttle loss, configured failsafe pulse, or calibration timeout
  restores safe outputs.
- Rearming after a safety event requires a healthy STOP-to-RUN cycle and does
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
`esp32`. The last recorded application binary was about `0x34700` bytes with
roughly 80 percent of the application partition free. Treat size figures as a
historical baseline and report current build output after new changes.

There is currently no automated host or hardware test suite. The obsolete
`pytest_hello_world.py` file was intentionally deleted. At minimum, run a full
firmware build after code changes. Scale bench testing with risk.

Required bench-test progression for major control changes:

1. Traction power disconnected; inspect all eight outputs with a scope or
   signal tester.
2. Calibrate and monitor all receiver inputs.
3. Verify CH4 STOP and receiver-loss behavior from every relevant state.
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
- RPM pulse-to-mechanical-RPM conversion still needs optical-tachometer
  validation.
- ISM330DHCX orientation, yaw sign, bias behavior, and vibration performance
  need on-car validation.
- CH1 steering tap, CH4 shutdown, and CH5 mode PWM calibrations need validation
  with the actual R7FG configuration.
- Torque vectoring has compiled but has not been tuned or proven on the car.
- Boot behavior with the ESC input connected to GPIO2 should be watched
  because it is a strapping pin.
- Exact ESC SKU and permitted traction-battery cell count must be confirmed
  before high-voltage operation.

Resolve these with measurements and update this file and the architecture
document when facts replace assumptions.
