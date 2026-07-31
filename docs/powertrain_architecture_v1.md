# Four-Motor RC Car Powertrain Architecture

## 1. Scope

This firmware controls the powertrain of a 1/7 scale RC car:

- 4x Hobbywing Skywalker 2820SL 550KV motors
- 4x Hobbywing Skywalker 50A V2 ESCs
- Radiolink RC6GS V3 transmitter and R7FG receiver
- ESP32 DOIT DevKit WROOM-32
- Adafruit STEMMA QT ISM330DHCX IMU
- 4x Hobbywing HW86060041 RPM sensors

The steering servo remains connected directly to receiver CH1. The ESP32
only observes a high-impedance copy of CH1 PWM. V1 does not generate servo
commands.

With torque vectoring disabled, all four ESCs receive identical throttle and
reverse commands. With torque vectoring active, reverse/direction remains
identical but the four throttle outputs may differ by a guarded correction.

## 2. Operating Modes

- `DISARMED`: all throttle outputs are `1100 us`; all reverse outputs are
  `1100 us`.
- `DRIVE_ARMED`: receiver throttle is mapped to the ESC range and sent
  through the drive and optional vectoring controller.
- `ESC_CAL_ARMED`: calibration prepared, safe outputs active.
- `ESC_CAL_MAX`: all throttle outputs `1940 us`, reverse outputs `1100 us`.
- `ESC_CAL_MIN`: all throttle and reverse outputs `1100 us`.
- `ESC_CAL_MANUAL`: receiver throttle PWM is directly relayed to all ESC
  throttle inputs; reverse stays low.

No drive or ESC calibration mode is entered automatically at boot.

## 3. Pin Assignment

All assignments live in `main/pin_config.h`.

### Receiver PWM Inputs

| Function | R7FG channel | ESP32 GPIO |
|---|---:|---:|
| Steering observation | CH1 | GPIO32 |
| Throttle | CH2 | GPIO25 |
| Shutdown RUN/STOP | CH4 | GPIO33 |
| Torque-vectoring mode | CH5 | GPIO26 |

Recommended CH5 three-position mapping:

- Position 1: OFF
- Position 2: straight-line assist
- Position 3: full turn assist

### ESC Outputs

| Wheel | Throttle GPIO | Reverse GPIO |
|---|---:|---:|
| Front left | GPIO15 | GPIO2 |
| Front right | GPIO13 | GPIO12 |
| Rear left | GPIO21 | GPIO19 |
| Rear right | GPIO27 | GPIO14 |

GPIO2, GPIO12, and GPIO15 are ESP32 boot-strapping pins. The ESC signal
inputs should be high impedance, but do not add external pull-ups or
pull-downs to these lines. If boot reliability is poor with the ESCs
connected, relocate these three signals to GPIO16, GPIO17, and GPIO18 and
update `pin_config.h`. The currently unused IMU interrupt assignments can be
released for that purpose.

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
| INT1, reserved but unused | GPIO18 |
| INT2, reserved but unused | GPIO17 |

The firmware polls the IMU over I2C at 200 Hz. Interrupt pins are not used
in this version.

## 4. Electrical Wiring

### Common Ground

Receiver ground, ESP32 ground, IMU ground, RPM sensor grounds, and all ESC
signal grounds must be connected. Signal references are unreliable without
this common ground.

### Receiver and Steering Tap

Splicing the steering PWM signal is a good approach because the ESP32 input
is high impedance and does not take control away from the servo. Make the
splice as a Y connection:

```text
R7FG CH1 signal ----+---- steering servo signal
                    |
                    +---- 3.3 V conditioning ---- GPIO32
```

Do not connect servo power to an ESP32 GPIO. Condition every receiver PWM
signal to a maximum of 3.3 V if the R7FG output is above 3.3 V. A resistor
divider or a proper logic-level buffer is suitable. Verify pulse widths with
`monitor steering`, not with a multimeter's DC voltage mode.

Use the R7FG in standard PWM mode with its integrated gyro disabled. The
receiver gyro modifies steering before the ESP32 observes it and would add
another feedback loop that conflicts with the ISM330DHCX-based controller.
The [R7FG manual](https://radiolink.com.cn/r7fg_manual) describes its PWM and
gyro modes.

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

The phase-tap wiring carries traction-battery switching voltage. Insulate and
strain-relieve these connections. The
[Hobbywing RPM sensor page](https://www.hobbywingdirect.com/products/rpm-sensor)
and its linked manual cover the lead functions and voltage ranges.

### IMU

Power the STEMMA QT board from ESP32 `3V3` and `GND`, then connect SDA and
SCL. Mount it rigidly near the chassis centerline with its axes aligned to
the car. Soft foam mounting adds delay and corrupts yaw-rate feedback. The
[Adafruit pinout guide](https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/pinouts)
documents the board connections.

## 5. ESC Signal Model

The local [Skywalker ESC manual](../Skywalker_ESC_Manual.pdf) specifies:

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

The default reverse limit is 10 percent. A direction change first ramps
throttle to minimum, holds for 120 ms, changes the reverse outputs, and then
ramps back up.

## 6. RPM Conversion and Calibration

The official Skywalker 2820SL 550KV specification identifies the motor as
`12N14P`, so it has 14 magnetic poles and 7 pole pairs. See the
[Hobbywing motor specification](https://www.hobbywing.com/en/products/skywalker2814).

The firmware counts rising edges from each phase-derived RPM signal with an
ESP32 PCNT unit and interprets one pulse cycle as one electrical revolution:

```text
pole_pairs = motor_poles / 2 = 7
mechanical_rpm = pulse_frequency_hz * 60 / pole_pairs

mechanical_rpm = rising_edge_count * 60,000,000
                 --------------------------------
                 elapsed_time_us * pole_pairs
```

RPM uses a rolling window of five 20 ms samples. A channel becomes invalid
if no pulse is seen for 250 ms. The default is 14 poles and can be changed:

```text
config rpm poles 14
```

The sensor manual does not explicitly state the pulse count per mechanical
revolution, so verify this conversion before enabling vectoring:

1. Leave torque vectoring disabled and lift the car.
2. Send `monitor rpm`.
3. Spin one wheel at a time and confirm the matching FL/FR/RL/RR channel.
4. Compare displayed RPM at several speeds with an optical tachometer.
5. A constant factor error indicates an incorrect poles/pulses assumption,
   not a gain error. Do not tune the vectoring controller around it.

Because these are hub motors, motor RPM and wheel RPM are the same.

## 7. Receiver Calibration

The firmware stores calibrations in ESP32 NVS.

```text
cal receiver   neutral, full throttle, full reverse
cal steering   center, full left, full right
cal arm        CH4 STOP, then CH4 RUN
cal tv         CH5 OFF, STRAIGHT, then FULL
```

Each command prompts for a position. Hold the requested position and press
Enter; the ESP32 averages 1.5 seconds of PWM measurements. Endpoints are
identified by their captured values, so transmitter channel reversal is
handled without hardcoded polarity.

The currently measured throttle defaults remain:

- Full throttle: `1750 us`
- Neutral: `1250 us`
- Full reverse: `1000 us`
- Neutral deadband: `80 us`

Drive arming requires saved throttle and CH4 calibrations. Steering and CH5
calibration are required only before enabling torque vectoring.

## 8. Receiver Shutdown and Failsafe

CH4 replaces the former physical GPIO switch.

Normal arming:

1. Receiver is connected and CH4 is in STOP.
2. Throttle is neutral.
3. Move CH4 from STOP to RUN.

The controller disarms when CH4 requests STOP or its PWM disappears. On
boot, after signal loss, or after the configured throttle failsafe pulse, it
requires a healthy STOP observation followed by RUN. The receiver's own
failsafe STOP output cannot clear this lock while throttle is at the
transmitter-off failsafe value. This prevents automatic re-arming when the
transmitter reconnects.

Configure the R7FG failsafe so:

- CH2 commands the known safe/failsafe pulse.
- CH4 commands STOP.
- CH5 commands OFF.

The legacy measured CH2 transmitter-off value is `1565 us`. Configure its
detection window with:

```text
config failsafe 1565 10
```

When that pulse is detected, safe ESC outputs are applied immediately and
the drive state disarms after a 60 ms confirmation. Complete PWM loss is
declared after 100 ms.

## 9. Laptop ESC Calibration

Use the ESP32 USB serial connection at `115200` baud. CH4 must remain STOP,
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

`cal manual` is an alternate 30-second bench mode that directly relays CH2
PWM to all four throttle outputs while holding reverse low. A throttle
failsafe match, CH4 leaving STOP, signal loss, timeout, or `cal cancel`
stops the relay.

## 10. Torque-Vectoring Controller

Torque vectoring defaults to disabled. The baseline OFF mode always sends
the same throttle to all four ESCs.

Before `tv enable` is accepted during a boot:

- `cal steering` and `cal tv` must be stored.
- `cal imu` must succeed with the car level and stationary.
- Every RPM input must have produced valid pulses. Use `monitor rpm` and
  rotate each wheel.

IMU gyro bias is intentionally calibrated each boot because temperature and
mounting bias drift. If IMU or RPM data becomes invalid while driving, the
controller immediately falls back to equal throttle outputs.

### CH5 Modes

- OFF: equal four-wheel commands.
- STRAIGHT: active only while steering is within 6 percent of center. Target
  yaw rate and left/right RPM difference are both zero.
- FULL: steering creates empirical target yaw rate and left/right RPM
  difference. It includes the straight controller around center.

Positive normalized steering means right. For a right-turn correction, the
left pair receives positive correction and the right pair receives equal
negative correction:

```text
FL = base + correction
FR = base - correction
RL = base + correction
RR = base - correction
```

The controller uses:

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

Correction defaults to at most `+/-10%` of full throttle span. Left and
right corrections are equal and opposite, so requested mean power is
preserved. Correction naturally reduces near zero and full throttle because
one side cannot go below minimum or above maximum.

Vectoring is forward-only in this version. Reverse always uses equal
throttle.

Initial tuning commands:

```text
config tv authority 10
config tv gains 180 0.20 0.00025 0.00004 0.20
config imu yaw-sign 1
```

Use `monitor imu` and rotate the car right by hand. If reported yaw is
negative, use `config imu yaw-sign -1`. Use `monitor vector` to inspect
target yaw, errors, and four corrections. Tune with the car restrained or
at low speed, starting with authority at 2 to 5 percent.

## 11. Firmware Structure

- `app_main.c`: initializes and starts the application.
- `cli.c`: parses USB serial commands.
- `config_store.c`: owns defaults and NVS persistence.
- `rc_input.c`: captures CH1, CH2, CH4, and CH5 PWM using GPIO edge
  interrupts.
- `esc_output.c`: generates eight independent 50 Hz LEDC outputs.
- `rpm_sensor.c`: owns four hardware PCNT units and RPM conversion.
- `imu_sensor.c`: configures and reads the ISM330DHCX over I2C.
- `torque_vectoring.c`: contains the sensor controller and balanced wheel
  correction.
- `powertrain_controller.c`: owns system state, arming, failsafe, mapping,
  calibration flows, and task scheduling.
- `pin_config.h`: is the single source of truth for physical pin assignment.

## 12. Bench-Test Order

1. Test with traction power disconnected and inspect all eight PWM outputs.
2. Calibrate and monitor each receiver channel.
3. Verify CH4 STOP disarms from every drive state.
4. Turn the transmitter off and verify immediate safe outputs, then confirm
   STOP-to-RUN is required after reconnection.
5. Calibrate one ESC and motor first, then repeat for all four.
6. Confirm wheel direction. Swap any two phase wires on an incorrect motor.
7. Validate each RPM channel against an optical tachometer.
8. Validate IMU yaw sign and stationary bias.
9. Drive with torque vectoring disabled and verify equal output behavior.
10. Enable straight assist at low authority and tune yaw response.
11. Enable full assist only after straight behavior is stable.

Keep the wheels clear of the ground for every calibration and first-power
test. Four independent motors can produce substantial force even at low
command.
