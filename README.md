# RC Car Powertrain Controller

ESP-IDF firmware for a four-motor 1/7 scale RC car using an ESP32 DOIT
DevKit, four Hobbywing Skywalker 50A V2 ESCs, a Radiolink RC6GS V3/R7FG
radio system, four Hobbywing HW86060041 RPM sensors, and an Adafruit
ISM330DHCX IMU.

The normal drive path supports forward/reverse, receiver calibration,
ESP32 steering-servo pass-through, remote shutdown, receiver-loss protection,
and per-wheel ESC outputs.
Torque vectoring is implemented but defaults to disabled until its sensors
are validated and it is explicitly enabled.

## Project Layout

```text
main/
  app_main.c                 Startup only
  default_config.h           Receiver defaults and IMU mounting reference
  cli.c                      USB serial command parser
  config_store.c             NVS calibration and configuration
  powertrain_controller.c    Drive state, safety, and calibration workflows
  rc_input.c                 Four priority-3 GPIO PWM inputs
  servo_output.c             Validated 50 Hz steering-servo output
  esc_output.c               Four throttle and four reverse PWM outputs
  rpm_sensor.c               Four PCNT-based motor-speed inputs
  imu_sensor.c               ISM330DHCX I2C driver
  steering_input_filter.c    Full-range steering spike rejection
  steering_curve.c           Servo-to-road-wheel curve interpolation
  torque_vectoring.c         Straight and turn-assist controller
  pin_config.h               All external pin assignments
  include/                   Module interfaces and shared types
docs/
  powertrain_architecture_v1.md
```

## First Setup

Use the ESP32 USB serial monitor at `115200` baud. Keep the wheels off the
ground and the ESC traction battery disconnected while configuring inputs.

1. Set the R7FG to standard PWM mode with its built-in gyro disabled.
2. Configure receiver failsafe: CH1 centered, CH2 neutral, CH4 at either
   STOP/OFF position, and CH5 OFF.
3. Send `cal receiver`.
4. Send `cal steering`.
5. Send `cal arm` and capture RUN/ON, STOP/OFF 1, and STOP/OFF 2.
6. Send `cal tv`.
7. Leave CH4 in either STOP/OFF position and calibrate the four ESC endpoints.
8. Keep the car level and completely still for the first five seconds of every
   ESP32 boot while the firmware automatically calibrates the IMU yaw bias.
   Use `cal imu` only to retry a failed calibration.
9. Use `monitor rpm` and rotate each wheel to validate all four speed inputs.
10. Keep torque vectoring disabled for initial equal-output drive testing.

Receiver CH1 now connects only to the conditioned GPIO16 input. Connect the
servo signal to GPIO32, power the servo from its BEC/receiver rail, and keep
all grounds common. Do not power the servo from the ESP32.

The measured receiver defaults are centralized in `main/default_config.h`:

| Control | Positions in microseconds |
|---|---|
| Throttle | neutral 1514, full 978, reverse 2044 |
| Steering | center 1518, left 2050, right 987 |
| Torque vectoring | off 2047, straight 1513, full 981 |
| Shutdown | run/on 983, stop/off 1 2049, stop/off 2 1515 |

Saved calibration in NVS overrides these values. Run the calibration commands
after flashing; compiled defaults do not by themselves authorize arming.

The IMU mounting reference is `+X` toward the rear and `+Y` toward the right
side of the car. Confirm yaw polarity with `monitor imu` before enabling torque
vectoring.

To drive, put CH4 in either STOP/OFF position once after boot, hold throttle
neutral, then move CH4 to RUN/ON. RUN and neutral throttle must remain valid
for 250 ms before drive arms. Either OFF position disarms the car. After a
receiver-loss or safety event, repeat that OFF-to-ON cycle; no laptop command
is needed.

The optional fixed throttle-pulse failsafe detector is disabled by default.
CH4 shutdown and actual receiver PWM-loss detection remain active. Use
`config failsafe <pulse_us> <window_us>` to enable the detector for a known
receiver loss pulse, or `config failsafe off` to disable it again.

CH4 is a software shutdown command, not an independent emergency stop. A
separate physical control that removes ESC traction power or hardware enable
is required to guarantee shutdown if the ESP32 or firmware stops responding.

## Serial Commands

```text
status
arm
disarm

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

monitor throttle
monitor steering
monitor steering trim <-15..15>
monitor arm
monitor tv
monitor rpm
monitor imu
monitor vector

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

`tv enable` is guarded. The automatic five-second startup IMU calibration (or
a later successful `cal imu` retry) must have succeeded during the current
boot, and every RPM input must have produced valid pulses. CH5 then selects
OFF, straight-line assist, or full turn assist.

The user confirmed that one mechanical revolution produces seven rising edges,
so the compiled `7 PPR` conversion is correct for the tested motor/sensor.
Use `config rpm ppr 7` to restore that value if configuration changes. The
legacy `config rpm poles 14` command remains backward compatible and derives
the same value. `monitor rpm` prints raw frequency, edge count,
measurement-window duration, and derived RPM. Check all four installed
channels against an optical tachometer across speed to detect noise or
missed/extra edges.

Steering setup requires a verified trim value in the persistent NVS drive
configuration. A stored `0.0 degrees` is valid when no correction is needed.
While disarmed, `monitor steering trim <degrees>` validates and saves the
trim, waits for the drive loop to apply it, and then monitors the resulting
center. `config steering trim <degrees>` saves the same value without opening
the monitor. If the NVS write fails, the in-memory change is rolled back.
Trim is a command-space adjustment based on the conventional
`1000 us = 90 degrees` servo scale; it is not a measured road-wheel angle.
Either command is rejected if the trimmed center would not remain strictly
inside both calibrated endpoints.
Smoothing is a first-order time constant in milliseconds and defaults to
`60 ms`. A missing CH1 signal still centers the servo immediately. Set
`config steering smoothing 0` to disable smoothing while troubleshooting.
A three-distinct-frame median filter rejects an isolated valid-looking CH1
pulse anywhere in the steering range. Once active, it adds one receiver frame,
approximately `20 ms`, of predictable steering latency. Startup and signal
recovery hold the saved center until three new frames are available, normally
about `40-60 ms`. `status` and `monitor steering` report the raw input, median,
filter state, and a confirmed isolated-spike count. Repeated control-loop reads
of one receiver frame do not advance the filter.

The corrected 45-point steering table maps the applied, smoothed servo command
from `-45` to `+45 degrees` into separate left-front and right-front road-wheel
angles. Firmware performs linear interpolation between the nonuniform sample
points and saturates outside the supplied range. Source servo-positive steers
left, while each wheel angle is positive when that wheel points outward. The
controller converts both wheel-local signs into its common positive-right
frame. Thus, the supplied centered `LF=+1.64` and `RF=+1.64 degrees` toe-out
becomes runtime `LF=-1.64` and `RF=+1.64 degrees`, with zero mean steering.
`monitor steering` shows the converted, interpolated angles. Torque vectoring
uses their direction-normalized average instead of raw normalized servo
travel, but its yaw target remains empirical until wheelbase, front track,
effective tire radius, and a defensible vehicle-speed estimate are available.

`status` includes maximum observed sensor/powertrain execution time, deadline
overruns, minimum free task stack, and current/minimum heap. Those values are
runtime instrumentation: collect them on the car during worst-case driving
before treating timing or memory headroom as validated.

## Build

Open an ESP-IDF 6.x terminal in the project directory:

```powershell
idf.py build
idf.py flash monitor
```

The first build after changing ESP-IDF components is long. Later builds are
incremental. This project enables ESP-IDF's minimal-build option to avoid
building unrelated framework components.

ESP-IDF 6 requires the LEDC fade service before its thread-safe immediate-duty
API can be used. Startup installs that service after establishing all eight
ESC channels at safe pulses and before the first update; an installation
failure leaves the controller unavailable. This requirement does not consume
or change any GPIO assignment.

Each build also validates `main/pin_config.h` and generates the self-contained
USB-up wiring diagram at `build/esp32_pin_map.svg`. Generate and open only the
diagram with:

```powershell
.\tools\esp32_pin_map\open_pin_map.ps1
```

See [the architecture document](docs/powertrain_architecture_v1.md) for the
complete pin table, electrical notes, RPM conversion, safety behavior, and
bench-test order. The dated
[engineering review](docs/engineering_review_2026-08-10.md) records the source
research, code findings, torque-vectoring assessment, RPM/tachometer validation
package, performance budget, and unresolved physical evidence.
