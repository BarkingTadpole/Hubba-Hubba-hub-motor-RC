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
- Keep torque vectoring runtime-gated by current-boot IMU calibration, live
  selected-wheel RPM, saved calibration, and CH5 even though the user-requested
  first-boot configuration now stores it enabled.

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
| Arm/output inhibit RUN/STOP | CH4 | GPIO33 |
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

### Dragy Lite

| Signal | ESP32 pin |
|---|---:|
| Dragy yellow TX to ESP UART1 RX | GPIO4 |
| ESP UART1 TX to Dragy white RX | GPIO5 |
| Dragy blue SDA | GPIO23, shared with IMU |
| Dragy green SCL | GPIO22, shared with IMU |

UART1 is routed through the GPIO matrix so UART0 remains available for USB
flashing and the serial CLI. A receive-only installation may omit the white
wire/GPIO5 connection. GPIO5 is a boot-strapping pin: do not add an external
pull-up/down, and leave Dragy RX disconnected if it affects boot reliability.

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
- Dragy black must connect to common ground. Dragy red requires a suitable
  regulated 5-16 V supply and must not connect to an ESP32 GPIO. Confirm the
  yellow/white UART and blue/green I2C logic-high levels do not exceed 3.3 V.
- The Dragy compass shares the existing 400 kHz GPIO23/GPIO22 bus. Its pull-ups
  must reference 3.3 V, its address must not conflict with the IMU at `0x6A`,
  and parallel pull-ups must leave a valid total resistance.

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
  - Arm/output inhibit: RUN/ON `983 us`, STOP/OFF 1 `2049 us`, STOP/OFF 2 `1515 us`.
- The receiver was observed to output approximately `1565 us` on throttle
  when the transmitter was off or disconnected.
- The transmitter-off pulse detector defaults enabled at `1565 +/- 10 us` in
  the user-requested first-boot profile.
- Reverse output defaults to the user-requested 100 percent limit. This is a
  high-risk change from the former 10 percent default; preserve the direction
  interlock and validate it with the wheels lifted before powered use.
- Drive smoothing defaults to 100 percent, which preserves the established
  12/100 us acceleration/deceleration steps. Zero disables both ramps but
  never disables the 120 ms direction-change zero hold.
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
- On 2026-08-25, the browser bridge repeatedly established TCP but received
  zero bytes and no telemetry. The expanded fixed JSON schema was already
  4,240 bytes before formatted values, exceeding the firmware's former 4,096
  byte transmit buffer. The frame now uses a dedicated static 8 KiB buffer so
  it does not consume the Wi-Fi task stack, and both firmware and Python report
  actionable formatting/send/close/silence errors. Reflash is required for
  this correction; HTTP `/api/state` polling alone does not validate it.
- On 2026-08-26, live telemetry showed frequent valid-range but impossible CH1
  pulse widths after internal-flash telemetry logging was added. The steering
  median filter itself still rejected isolated pulses across the full range,
  but repeated GPIO edge delays could defeat a three-frame median; the neutral
  deadband merely hid more of the effect near center. Receiver capture now
  installs the GPIO service with `ESP_INTR_FLAG_IRAM` and reads GPIO input
  registers directly from the IRAM ISR, so flash-cache-disabled writes cannot
  defer receiver edges. This correction has code/build validation only and
  needs a sustained-logging bench check with traction power disconnected.
- On 2026-08-26, the user reported that after a bridge/ESP32 disconnect,
  Windows Mobile Hotspot could still show the ESP32 associated while the
  viewer could not reconnect until the hotspot was restarted. The desktop
  bridge had discarded the last working IP and depended entirely on a fresh
  UDP broadcast discovery. It now retains the last confirmed address, retries
  it directly after a drop, and falls back to UDP discovery only if direct TCP
  connection fails. A simulated one-shot-discovery/two-session test covers the
  recovery path; hotspot behavior still needs physical validation.
- On 2026-08-26, the browser gained guarded calibration coverage for receiver
  throttle, steering, CH4, CH5, IMU bias, ESC endpoints, and manual relay.
  Receiver/IMU work uses a structured prompt/capture state machine rather than
  arbitrary terminal forwarding; configuration and arming are blocked during
  an active workflow. The remote parser, simulated bridge/API, and firmware
  build pass, but every calibration path still requires scoped hardware bench
  validation before use with traction power.
- On 2026-08-23, the user explicitly requested boot-lifetime arm latching.
  After the initial valid STOP/OFF-to-RUN/ON arm transition, receiver faults
  and CH4 STOP/OFF now temporarily hold safe ESC output instead of clearing
  `SYSTEM_DRIVE_ARMED`; valid inputs can recover automatically through the
  acceleration ramp. The arm latch clears only on controller reset or power
  loss. The CLI `disarm` command creates a non-clearable safe-output inhibit
  for the rest of the boot. This behavior has code/build validation only and
  materially increases reliance on independent traction-power isolation.
- On 2026-08-25, permanent latching became an NVS-backed user option while
  remaining enabled by default for backward compatibility. With
  `cfg/arm_latch=0`, recognized STOP, receiver signal loss, confirmed
  unrecognized CH4, and the configured throttle failsafe fully disarm and
  require a fresh healthy STOP-to-RUN transition. The option is writable only
  in `DISARMED` and is exposed through serial and the browser viewer.
- On 2026-08-25, the user requested an explicit browser maintenance disarm so
  configuration does not require an ESP32 reset when permanent latching is on.
  `disarm config` is the only remote state-changing exception outside guarded
  configuration: it immediately returns `DRIVE_ARMED` to `DISARMED`, commands
  safe outputs, clears the latch, and requires a new physical CH4 STOP-to-RUN
  cycle. Remote arm remains prohibited.
- On 2026-08-25, live logger telemetry showed `samples=0`, a 1,956-byte
  header-only CSV, `capacity_bytes=2756608`, `free_bytes=0`, and `full=true`.
  The mounted FAT allocation map was exhausted or corrupt even though the
  visible file contained no rows. The viewer now reports this state directly.
  An explicitly confirmed `DISARMED` clear first unlinks normally, then formats
  only the dedicated `logdata` partition if usable space is still not
  reclaimed. This recovery has code/build/simulation validation only.
- On 2026-09-01, the user replaced the station/hotspot/Python-bridge path with
  an ESP32-hosted SoftAP viewer. The default build now creates WPA2 SSID
  `RC-Car-ESP32`, serves the embedded dashboard and guarded HTTP API at
  `http://192.168.4.1`, and requires no upstream hotspot or Python process.
  ESP-IDF 6.0.1 builds this image at `0xfbc70` bytes with 21 percent of the
  application partition free. This is code/build validation only; AP join,
  multi-device access, browser workflows, RF coexistence, supply integrity,
  and real hardware safety behavior remain bench pending.
- On 2026-09-01, drive acceleration/deceleration smoothing became a persistent
  `0-100%` setting. `100%` preserves the established smoothest `12 us` accel
  and `100 us` decel steps per 20 ms update; lower values shorten both ramps
  inversely, and `0%` applies requested throttle changes on the next update.
  The 120 ms zero-throttle direction-change hold is never disabled. This has
  code/build/host-test validation only and needs restrained bench validation.

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
  The GPIO service and handler are IRAM-safe, and the handler reads GPIO input
  registers directly, so internal-flash cache stalls do not delay PWM edges.
- MCPWM and RMT receiver-capture experiments were rejected. MCPWM starved
  channels when throttle changed, while RMT produced unsafe intermittent drive
  and ignored shutdown during a bench incident.
- Default throttle calibration: `978/1514/2044 us` for
  full-forward/neutral/full-reverse.
- Default throttle neutral deadband: `80 us`.
- Throttle magnitude uses a quadratic response curve.
- Default maximum reverse magnitude: 100 percent of ESC span, per the
  user-requested first-boot profile.
- Steering valid-command smoothing is a first-order filter with a default
  `10 ms` time constant and a configurable `0-500 ms` range. Missing CH1 or a
  confirmed throttle failsafe centers immediately rather than filtering the
  safety response.
- Steering applies a three-distinct-frame median before the first-order command
  filter. It rejects one isolated valid-looking CH1 pulse across the full
  range and adds approximately one 20 ms receiver frame of continuous latency.
  Startup, missing CH1, throttle failsafe, and steering calibration clear its
  history and hold center until three new frames arrive. Duplicate reads of
  one receiver timestamp do not advance it. `status` and `monitor steering`
  expose warmup/active state and a confirmed isolated-spike count.
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
  input and provides the inverse lookup for speed-sensitive steering limits.
- Speed-sensitive steering is enabled by default. It uses average rear RPM,
  a `107 mm` wheel diameter, `445 mm` wheelbase, and a configurable `1.0 g`
  default lateral-acceleration ceiling. The supplied `320 mm` track width is
  retained as vehicle geometry reference. A temporary rear-RPM dropout while
  armed retains the last valid speed instead of restoring full steering.

CH4 is a three-position receiver arm/output-inhibit control with one learned
RUN/ON position and two learned STOP/OFF positions. Initial arming requires:

- Saved receiver throttle calibration.
- Saved CH4 RUN/ON and two-position STOP/OFF calibration.
- Valid CH2 and CH4 PWM.
- Neutral throttle.
- CH4 in the learned RUN/ON position.
- Continuous valid RUN/ON and neutral throttle for 250 ms before drive entry.
- A healthy observation of either learned STOP/OFF position since boot.

Normal operation requires a deliberate CH4 STOP/OFF-to-RUN/ON transition.
`cfg/arm_latch` selects the post-arm policy and defaults to enabled. When
enabled, `SYSTEM_DRIVE_ARMED` remains latched until controller reset or power
loss; STOP and receiver safety faults apply temporary safe-output inhibits,
and valid inputs can recover without another arm cycle. When disabled, those
same safety events return the controller to `SYSTEM_DISARMED` and a fresh
healthy STOP/OFF-to-RUN/ON transition is required. In both modes, an
unrecognized CH4 pulse must persist for 60 ms before action, while a recognized
STOP acts immediately.

Each learned CH4 position has a `+/-125 us` recognition window. An
unrecognized pulse is safe-off after the 60 ms confirmation; it clears the
armed state only when permanent latching is disabled.

Recommended R7FG failsafe positions:

- CH1: steering center.
- CH2: neutral throttle.
- CH4: either learned STOP/OFF position.
- CH5: OFF.

The fixed CH2 pulse detector defaults enabled at `1565 +/- 10 us`. `config
failsafe <pulse_us> <window_us>` reconfigures it and `config failsafe off`
disables it. When enabled, a matching pulse applies safe output immediately.
It is a temporary inhibit when permanent latching is enabled and a full disarm
when disabled. Complete PWM loss is declared after 100 ms regardless of this
option.

## Drive Timing

- Main drive update interval: 20 ms.
- Pre-arm stable RUN/neutral qualification: 250 ms.
- Missed sensor or drive deadlines reset the periodic schedule and force a
  one-tick block; `status` exposes cumulative overrun counts, maximum observed
  execution time, minimum free task stack, and heap headroom.
- Persistent drive smoothing: `0-100%`, default `100%`.
- At `100%`, acceleration changes by 12 us of ESC pulse per update and
  deceleration changes by 100 us per update. Lower percentages shorten both
  ramp times; `50%` uses 24/200 us steps and `0%` disables both ramps.
- Direction-change zero hold: 120 ms.
- ESC calibration inactivity timeout: 30 seconds.
- Monitor command duration: 30 seconds.

Do not weaken these values without documenting the reason and considering the
combined torque of four motors.

## Drivetrain Modes

Normal drive defaults to AWD and supports persistent `AWD`, `FWD`, and `RWD`
selection with `config drivetrain <awd|fwd|rwd>` while DISARMED. The selected
mode affects normal drive only; ESC endpoint calibration and manual relay
still address all four ESCs.

- AWD drives all four wheels.
- FWD drives FL/FR and holds RL/RR at safe `1100/1100 us` PWM.
- RWD drives RL/RR and holds FL/FR at safe `1100/1100 us` PWM.

The inactive axle remains at throttle minimum and reverse low even when the
driven axle reverses. Invalid runtime mode data must safe all four outputs.
Torque vectoring uses only selected driven-wheel RPM and applies balanced
left/right correction only to that axle or pair of axles. Front relief applies
in AWD/FWD FULL mode and is forced to zero in RWD.

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

Torque vectoring is implemented and the requested first-boot profile stores it
enabled. Runtime prerequisites and CH5 still decide whether it becomes active;
CH5 has three calibrated positions:

- OFF: equal throttle on the selected driven wheels; inactive axle safe.
- STRAIGHT: zero-yaw and equal-left/right-RPM correction while steering is
  within 6 percent of center.
- FULL: empirical steering-based yaw and side-RPM targets for turns, including
  straight correction around center.

Each CH5 mode requires a pulse inside its learned `+/-125 us` window. A pulse
outside all three windows selects OFF rather than the nearest active mode.

The controller operates only in forward drive. Reverse always uses equal
throttle on the selected driven axle. It requires valid RPM from all selected
driven wheels, valid and bias-calibrated IMU data, sufficient speed, saved
steering/CH5 calibration, and explicit configuration enablement. Invalid data
must produce equal outputs on the driven axle, not stale corrections; inactive
axles remain safe.

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

- Authority: `+/-25%` of full ESC throttle span.
- Turn yaw gain: `180 deg/s`.
- Turn side-RPM gain: `0.20`.
- Yaw proportional gain: `0.00025`.
- Yaw integral gain: `0.00004`.
- Side-RPM proportional gain: `0.20`.
- IMU yaw sign: `-1`.

Left/right side corrections are equal and opposite across the selected driven
axle or axles. In FULL mode for AWD/FWD, front-axle relief additionally
subtracts up to 20 percent of full ESC span from both front motors in
proportion to predicted lateral demand. RWD forces relief to zero. That power
is not transferred rearward, so mean requested power intentionally falls while
relief is active. Available side authority naturally falls near zero and full
throttle because both sides must stay inside the ESC output limits. The yaw integrator is conditionally held
when saturation and error would wind it farther into the limit. Non-finite
configuration/input values, raw RPM below 16.7 Hz, or invalid/stale sensors
reset the controller and produce equal outputs.

`tv enable` is guarded. During the current boot, the automatic startup IMU
calibration or a manual `cal imu` retry must succeed, and every selected driven
RPM channel must have produced a valid pulse at least once. Use `monitor rpm`
and rotate the selected driven wheels before enabling. Validate all four
channels before changing between drive modes. Even if enablement is persisted in NVS, the
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
disarm config
help
```

The receiver CH4 transition is the normal arming method. The CLI `arm` command
still respects receiver calibration, CH4 RUN, neutral throttle, and the
initial STOP-cycle requirement. With permanent latching enabled, CLI `disarm`
holds safe ESC outputs until restart. With it disabled, CLI `disarm` returns
the controller to normal `DISARMED` state. `disarm config` always clears the
armed state for maintenance and forces a new physical CH4 arm cycle.

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
monitor gps
monitor gps raw
monitor vector
```

Monitoring blocks the serial CLI temporarily but does not stop the controller
or sensor tasks. `monitor imu` is read-only and never starts calibration.
`monitor gps` is read-only and reports UART/NMEA health plus current GPS data;
`monitor dragy` is an alias. GPS course over ground is not compass heading.
`monitor gps raw` (`monitor dragy raw`) captures five seconds into a bounded
512-byte RAM ring and prints hex/ASCII while normal parsing continues. It does
not transmit to or reconfigure the Dragy.
Attaching a serial monitor that toggles DTR/RTS can reset the devkit; the new
boot then correctly repeats the five-second startup calibration. Steering
setup requires a verified trim value in the NVS
drive configuration; `0.0 degrees` is valid if no correction is needed.
`monitor steering trim <degrees>` is accepted only while disarmed, saves the
validated trim to NVS, waits for it to be applied, and then starts the normal
steering monitor. A failed NVS write rolls the in-memory value back.

### Configuration

```text
config drivetrain <awd|fwd|rwd>
config arm-latch <on|off>
config reverse <0-100>
config drive smoothing <0-100>
config failsafe off | <pulse_us> <window_us>
config rpm poles <even 2-60>
config rpm ppr <1-120>
config steering trim <-15..15>
config steering smoothing <0-500>
config steering speed-limit <on|off>
config steering lateral-g <0.2-3.0>
config tv authority <0-25>
config tv front-relief <0-50>
config tv gains <yaw_gain_dps> <turn_rpm_gain> <yaw_kp> <yaw_ki> <rpm_kp>
config imu yaw-sign <-1|1>
config logging rate <1-50>
tv enable
tv disable
```

Configuration changes are accepted only in `DISARMED` and persist in NVS.
With permanent latching enabled, restart after arming to return to the
configurable state. With it disabled, a normal STOP/disarm returns there.

## ESP32-Hosted Wi-Fi Viewer

Wi-Fi monitoring/configuration defaults enabled under the `RC car Wi-Fi
access point` Kconfig menu. The ESP32 creates a 2.4 GHz WPA2 SoftAP instead of
joining an upstream WLAN. The tracked development defaults are SSID
`RC-Car-ESP32`, password `rc-car-viewer`, channel 1, up to four clients, and
HTTP port 80. Change the password before powered operation around untrusted
devices. Normal access is `http://192.168.4.1`; the ESP32 supplies client DHCP
addresses and no Python bridge, UDP discovery, station DHCP, or hotspot is in
the runtime path.

`main/wifi_control.c` starts an ESP-IDF HTTP server at priority 2 with an 8 KiB
stack. Static viewer assets are embedded from `tools/wifi_bridge/web`, and the
HTTP task formats state on demand into a dedicated static 8 KiB buffer. The
browser polls `GET /api/state` every 250 ms. Network work must remain outside
the drive loop, must not raise its priority above receiver capture, and must
stay non-fatal to powertrain/serial startup.

HTTP configuration and calibration handlers pass only the documented
`config ...` commands, `tv enable|disable`, exact `disarm config`, and the
exact structured calibration commands (`cal receiver|steering|arm|tv|imu`,
`cal capture`, `cal esc arm|max|min`, `cal manual`, and `cal cancel`) through
`remote_command_parse`. They must never expose arm, ordinary `disarm`, drive
throttle, steering output, `monitor ...`, or arbitrary terminal commands.
Existing controller guards and NVS writes remain authoritative. Configuration
and calibration start require `DISARMED`, and configuration remains locked
while a calibration is active. `GET /api/config` and telemetry remain
read-only and available in all states.

Browser receiver calibration is stateful: each `cal capture` averages the
requested channel for 1500 ms, advances one step, and saves only a complete
validated three-position set to NVS. IMU capture performs the existing
two-second current-boot bias retry. An unfinished browser receiver/IMU flow
times out after 120 seconds. ESC endpoint/manual modes retain their 30-second
timeout, saved-CH4 STOP and throttle guards, reverse-low output, and failsafe/
signal-loss cancellation. The UI requires explicit confirmations for
high-risk ESC actions, but firmware guards and physical traction-power
isolation remain mandatory.

The dashboard `Load saved config` button reads the configuration already
loaded from NVS into the controller and reported in fresh telemetry. It does
not issue an NVS write. The drivetrain UI supports direct selection and an
AWD -> FWD -> RWD cycle button; the arm-latch control uses the same guarded
`config arm-latch <on|off>` path as serial.
The separately confirmed **Disarm for config** button is one-way: it commands
safe outputs and clears the armed latch, after which configuration unlocks.
It cannot arm or bypass the physical CH4 STOP-to-RUN qualification.

`GET /api/log.csv` streams the current fixed-size CSV snapshot in 8 KiB chunks.
`POST /api/log/clear` retains the explicit confirmation/UI flow and firmware
`DISARMED` guard. HTTP has no login beyond WPA2 association, so the AP password
is a control credential and must not be shared with untrusted clients.
`tools/wifi_bridge/bridge.py` remains only as a legacy mock/frontend
development aid and is not required on the car.

Wi-Fi current bursts may worsen the known unexplained on-car reset problem,
so powered validation requires measuring the ESP32 rail with the SoftAP active
and one or more viewers polling.

## Offline CSV Telemetry Log

`partitions.csv` replaces the default single-app table. The 4 MB layout gives
the factory app `0x140000` bytes (1.25 MiB) and the wear-levelled FAT `logdata`
partition `0x2b0000` bytes (2.75 MiB raw). Do not revert to the default table
without either removing the logger cleanly or providing another persistent
storage design.

`telemetry_log.c` owns `/telemetry/telemetry.csv` and a priority-1 CPU0 task.
As of 2026-09-04, recording is explicitly manual and stopped on every boot.
`telemetry_log_init()` mounts/opens storage and starts an idle worker; only
`telemetry_log_set_recording(true)` starts sampling. Viewer Start/Stop actions
use dedicated `/api/log/start` and `/api/log/stop` POST routes and are allowed
while armed without changing drive state. Stop disables writes under the
logger mutex and flushes/fsyncs before acknowledging success; failures remain
stopped and faulted. A generation counter discards in-flight rows across
stop/start/clear. Start appends to the existing CSV. Clear/recovery always
leaves recording stopped. Never resume automatically on boot, reconnect,
arming, or a sample-rate change; recording state is not persisted in NVS.
This has host lifecycle, mock browser/API, and build validation only; actual
flash flush durability and on-car behavior remain bench pending.
The default rate is 1 Hz (1000 ms). The `cfg/log_hz` NVS key accepts `1-50 Hz`;
`config logging rate <hz>` and the live viewer configuration control update it
only while `DISARMED`, and the change takes effect without rebooting. It samples the public mutex-protected
powertrain snapshot and the Dragy snapshot, never performs file I/O in the
20 ms drive task, flushes/fsyncs at least once per second, and stops with a
16 KiB reserve rather than wrapping over old rows. Its schema version 1 has
128 columns covering controller state/inhibits, all receiver channels,
steering, RPM/raw counters, every IMU axis, TV internals, all ESC outputs,
Dragy GPS/parser fields, and drive configuration context.

Wi-Fi loss does not participate in the recording path. The on-device HTTP
server exports a fixed-size snapshot in 8 KiB response chunks after a client
reconnects. Clearing is explicitly confirmed in the UI and firmware accepts
it only while `DISARMED`. Keep download and clear out of the remote
configuration whitelist; they have separate HTTP handlers and guards.

A header-only CSV is zero samples, not an analysis-parser failure. If the
visible file has no samples but a known-capacity FAT volume reports no usable
space, the dashboard labels the action `Recover log storage`. The existing
confirmed clear operation then formats only `logdata` if unlinking the CSV did
not restore the 16 KiB reserve. Never format automatically from the record
path, and never apply this recovery to NVS or the application partition.

Writer or flush failures latch a storage fault, stop further log writes, and
publish the last `errno`. Export does not discard already-committed data merely
because the append stream cannot flush: it uses `stat()` plus a separate
read-only stream to salvage the readable prefix. Always export before clear or
reformat. A zero FAT free-space value is full when capacity is known; do not
treat it as an unknown value and continue writing.

This is bounded offline retention, not indefinite losslessness. A typical
600-800 byte row at the default 1 Hz is roughly 60-80 minutes; live telemetry/dashboard use
actual FAT free space and observed row size for the estimate. Faster sampling
reduces retention proportionally. Sudden power removal can lose the final
buffered second or corrupt FAT; mount recovery may reformat the dedicated log
partition. Indefinite or power-failure-durable capture requires added storage
and/or hold-up/journaling hardware. Keep these limitations explicit.

Internal-flash program/erase and cache critical sections can still delay work
on both cores even though the logger task is low priority. Hardware validation
must exercise worst-case allocation/flush while checking controller deadline
overruns, PWM, receiver capture, heap/stack, supply voltage, and resets. If it
affects deterministic control, disable it or move logs to external storage;
never relax powertrain safety deadlines for the logger.

## Dragy Lite GPS

The optional Dragy integration uses UART1 at `9600 8N1` by default, routed to
GPIO4 RX and GPIO5 TX. MAX-M10S NMEA RMC/GGA/VTG input is checksum-validated,
stored in fixed buffers, and considered stale after 1.5 seconds. The actual
Dragy UART rate may differ when configured for high-rate output, so verify it
on hardware and set `RC_DRAGY_UART_BAUD` accordingly.

GPS is the current integration scope. An I2C initialization or discovery
failure must not prevent the UART GPS task from starting. `monitor gps`
reports UART byte counts, NMEA freshness, fix, position, ground speed, course,
altitude, satellites/HDOP, and parser errors for 30 seconds; `monitor dragy`
is an alias. `monitor gps raw` and `monitor dragy raw` provide a bounded
five-second hex/ASCII UART capture without changing parser operation.

`sensor_i2c_bus.c` owns the 400 kHz GPIO23/GPIO22 controller and mutex for the
IMU and Dragy devices. `dragy_sensor.c` performs one boot-time address scan,
reports all responses over serial/Wi-Fi, recognizes the IMU at `0x6A`, and
treats every other response as a compass candidate. The MAX-M10S default I2C
address is `0x42`, but Dragy's official harness labels the exposed bus as the
compass connection, so do not exclude `0x42` from candidate detection. Power
the Dragy before boot if discovery is needed. Do not scan the entire address
space periodically while the 200 Hz IMU task is running.

Compass support is deferred. The public Dragy Lite page does not identify the compass chip, address,
register map, or heading format. The official developer page requires program
registration before integration access is provided. Do not guess a
magnetometer driver from an address. Heading support requires the registered
Dragy SDK/device documentation or a positively identified protocol. Public
references are:

- <https://www.godragy.com/dragy-lite/>
- <https://www.godragy.com/dragy-api/>
- <https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf>

## Firmware Structure

- `main/app_main.c`: process I/O setup, controller initialization, and task
  startup only.
- `main/cli.c`: serial input editing, parsing, validation, and dispatch.
- `main/default_config.h`: compiled receiver defaults and IMU mounting
  reference.
- `main/config_store.c`: defaults, validation, backward-compatible NVS keys,
  and persistence.
- `main/cornering_control.c`: RPM-derived speed, steering envelope, and
  predicted lateral demand.
- `main/drivetrain_control.c`: persistent AWD/FWD/RWD output selection and
  fail-safe inactive-axle masking.
- `main/rc_input.c`: four priority-3 GPIO edge-interrupt PWM inputs.
- `main/esc_output.c`: eight independent 50 Hz LEDC outputs.
- `main/servo_output.c`: independent 50 Hz steering PWM on GPIO32.
- `main/steering_input_filter.c`: full-range distinct-frame median rejection
  and isolated-spike telemetry.
- `main/steering_curve.c`: source-backed 45-point LF/RF road-wheel curve and
  deterministic piecewise-linear interpolation.
- `main/rpm_sensor.c`: four PCNT units, rolling windows, and RPM conversion.
- `main/imu_sensor.c`: minimal ISM330DHCX I2C driver and bias calibration.
- `main/sensor_i2c_bus.c`: shared 400 kHz IMU/Dragy bus ownership and locking.
- `main/dragy_nmea.c`: fixed-buffer, checksum-validated NMEA parser.
- `main/dragy_sensor.c`: UART1 Dragy GPS reception and startup I2C discovery.
- `main/remote_command.c`: network configuration/calibration whitelist and
  pure parser.
- `main/telemetry_log.c`: wear-levelled all-channel CSV recording, status,
  bounded export reads, and guarded clear primitive.
- `main/torque_vectoring.c`: sensor feedback, controller state, authority
  limits, and balanced wheel corrections.
- `main/wifi_control.c`: default WPA2 SoftAP, embedded viewer assets, direct
  telemetry/configuration HTTP API, CSV export, and guarded log clear.
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
- Permanent arm latching is enabled by default. In that policy, the valid
  initial arm transition latches until controller reset or power loss; CH4 and
  receiver safety events inhibit outputs without clearing it.
- When permanent latching is disabled, recognized CH4 STOP, CH4/throttle loss,
  a confirmed unrecognized CH4 pulse, and a matching configured failsafe all
  return to `DISARMED` and require a fresh healthy STOP-to-RUN cycle.
- CH4 is software safety logic, not a hardware emergency stop. Powered testing
  requires independent, immediately accessible traction-power isolation.
- Automatic recovery in permanent-latch mode passes through the acceleration
  ramp but does not require firmware-confirmed neutral; bench and operating
  procedure requires the operator to return throttle to neutral first.
- The steering output starts centered, follows the three-distinct-frame median
  of valid CH1 pulses within calibrated endpoints and the speed-sensitive
  steering envelope, and centers on CH1 loss. It also centers on a matching
  throttle pulse when the optional detector is enabled.
- Receiver safety events require rearming only when permanent latching is
  disabled.
- CLI `disarm` is a boot-long inhibit when permanent latching is enabled and a
  normal disarm when it is disabled.
- `disarm config`, including the confirmed browser action, always clears the
  armed state, commands safe outputs, and requires physical CH4 rearming.
- Reverse magnitude remains limited by configuration, default 100 percent in
  the user-requested profile. Treat reverse testing as full-authority testing.
- Direction changes pass through throttle minimum before reverse state
  changes.
- Reverse outputs remain low through ESC endpoint calibration.
- Torque vectoring configuration defaults enabled but remains runtime-gated
  and fails back to equal commands on the selected driven wheels on invalid
  sensor data; inactive axles stay safe.
- Wi-Fi exposes telemetry, guarded persistent configuration, exact guided
  calibration workflows, and the one-way `disarm config` maintenance action.
  It must not provide remote arm, ordinary disarm/inhibit, drive throttle,
  steering output, `monitor ...`, or arbitrary terminal paths. Calibration may
  change output only through the existing guarded ESC endpoint/manual states.
- Offline logging must remain independent of Wi-Fi and outside the powertrain
  loop after an explicit Start. Boot and clear must leave recording stopped.
  Full storage stops recording rather than overwriting existing rows;
  browser clear remains a deliberate `DISARMED`-only operation.
- Invalid drivetrain selection commands are rejected, invalid stored values
  fall back to AWD, and invalid runtime values safe all four ESC outputs.
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
`esp32` on 2026-09-01 with the SoftAP, embedded HTTP viewer/API, browser
calibration console, Dragy UART1 GPS, shared-I2C discovery, UART0 serial
console, FAT wear levelling, and offline CSV logger enabled. The application
binary was `0xfbc70` bytes with `0x44390` bytes (21 percent) free in the
1.25 MiB application partition. The dedicated raw log partition is 2.75 MiB.

Host tests cover balanced correction, the RPM frequency guard, non-finite
fallback, saturation anti-windup, steering-curve endpoints/signs,
interpolation, saturation, invalid input, the remote command whitelist,
checksum-validated Dragy NMEA parsing, legacy bridge/mock behavior, embedded
viewer assets, and direct SoftAP/HTTP source invariants. Browser validation
covers live state rendering, guarded calibration submission, direct device-log
loading, and project chart groups against the mock. These are not ESP32
flash-peripheral, Wi-Fi/GPS hardware, power-loss, browser-to-real-car,
calibration-on-real-hardware, or vehicle tests. The
obsolete `pytest_hello_world.py` file was intentionally deleted. At minimum,
run the host logic test and a full firmware build after related code changes.
Scale bench testing with risk.

Required bench-test progression for major control changes:

1. Traction power disconnected; inspect all nine outputs with a scope or
   signal tester.
2. Calibrate and monitor all receiver inputs; verify the GPIO32 servo output
   follows CH1, centers on signal loss, and respects the RPM-derived steering
   limit while both rear wheels are spun together.
3. Verify both CH4 STOP/OFF positions and receiver-loss behavior under both
   persisted arm policies. With latching on, confirm safe output, controlled
   recovery with throttle neutral, and clearing only after restart. With it
   off, confirm full disarm and a required fresh STOP-to-RUN cycle.
   With latching on, also use browser **Disarm for config** and confirm all ESC
   outputs go safe, configuration unlocks, and RUN alone cannot rearm until a
   physical STOP-to-RUN cycle occurs.
4. With traction power disconnected, verify Dragy UART logic levels/baud,
   GPS telemetry, the shared-I2C address list, IMU freshness, and reliable boot
   with GPIO5 connected.
5. Leave Wi-Fi disconnected while the logger runs, reconnect, export the CSV,
   and verify complete timing across the outage. Fill a sacrificial log to
   confirm it stops without wrapping, and characterize sudden-power loss on a
   disposable run. Confirm clear is rejected outside `DISARMED`.
6. Calibrate and test one ESC/motor before all four.
7. Verify every wheel direction physically.
8. Validate each RPM channel and compare RPM against an optical tachometer.
9. Validate IMU sign, stationary bias, and stale-data fallback.
10. Drive with torque vectoring disabled.
11. Tune straight assist at low authority.
12. Enable full assist only after straight behavior is stable.

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
- Wi-Fi association, protocol, remote-command access, dashboard operation, or
  security assumptions.

Keep the short startup path in the README and detailed engineering reasoning
in the architecture document. Do not duplicate stale procedures across files.

`docs/project_showcase_handoff.md` is the shared source of truth for the
separate showcase-site agent. Update it whenever a change affects public
project facts, visible capabilities, hardware, validation status,
performance/retention figures, safety claims, or available media. Keep it
concise and presentation-oriented rather than copying detailed engineering
procedures. Add the absolute copy-ready path and a short evidence description
for every newly generated photo, diagram, screenshot, video, plot, or other
showcase asset. Never put credentials or private identifiers in it.

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
- CH1 input, GPIO32 servo output, CH4 arm/output-inhibit, and CH5 mode PWM
  calibrations
  need validation with the actual R7FG and steering servo. The new steering
  trim, smoothing, wheel-local curve signs, centered toe-out, and road-wheel
  angles also require scope and on-car validation. The supplied curve is input
  data, not physical validation observed by the firmware review.
- Speed-sensitive steering and FULL-mode front torque relief have host-test
  and build validation only. With traction power disconnected, spin both rear
  wheels together and verify reported speed, decreasing steering limit, servo
  output, and front-relief telemetry before conservative low-speed road tests.
  These controls reduce demand but cannot correct a mechanical Ackermann or
  toe error that makes the front tires scrub at a given steering angle.
- The CPU-isolated GPIO receiver capture and high-speed LEDC ESC output
  rollback and the 2026-08-11 LEDC fade-service startup fix compile but still
  require traction-disconnected validation while changing throttle and
  commanding CH4 STOP. Verify the new boot-latched state, safe-output inhibits,
  automatic receiver recovery, and restart-only latch clearing. Do not power
  the motors until this passes repeatedly
  without stale inputs, servo jolts, boot loops, or task resets.
- Torque vectoring has compiled but has not been tuned or proven on the car.
- Runtime `status` timing, stack, and heap instrumentation has compiled but no
  worst-case hardware data has been captured yet.
- Wi-Fi association, reconnect behavior, UDP discovery, TCP telemetry, supply
  margin, and timing have only build/simulated-bridge validation. Verify them
  on the ESP32 and watch the 5 V rail because radio current bursts may worsen
  the unexplained driving resets.
- Boot behavior with the ESC input connected to GPIO2 should be watched
  because it is a strapping pin.
- Exact ESC SKU and permitted traction-battery cell count must be confirmed
  before high-voltage operation.

Resolve these with measurements and update this file and the architecture
document when facts replace assumptions.
