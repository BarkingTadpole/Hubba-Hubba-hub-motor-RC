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
8. Send `cal imu` with the car level and still after every ESP32 boot.
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
monitor arm
monitor tv
monitor rpm
monitor imu
monitor vector

config reverse <0-100>
config failsafe off | <pulse_us> <window_us>
config rpm poles <even 2-60>
config tv authority <0-25>
config tv gains <yaw_gain_dps> <turn_rpm_gain> <yaw_kp> <yaw_ki> <rpm_kp>
config imu yaw-sign <-1|1>

tv enable
tv disable
```

`tv enable` is guarded. During the current boot, `cal imu` must have
succeeded and every RPM input must have produced valid pulses. CH5 then
selects OFF, straight-line assist, or full turn assist.

## Build

Open an ESP-IDF 6.x terminal in the project directory:

```powershell
idf.py build
idf.py flash monitor
```

The first build after changing ESP-IDF components is long. Later builds are
incremental. This project enables ESP-IDF's minimal-build option to avoid
building unrelated framework components.

Each build also validates `main/pin_config.h` and generates the self-contained
USB-up wiring diagram at `build/esp32_pin_map.svg`. Generate and open only the
diagram with:

```powershell
.\tools\esp32_pin_map\open_pin_map.ps1
```

See [the architecture document](docs/powertrain_architecture_v1.md) for the
complete pin table, electrical notes, RPM conversion, safety behavior, and
bench-test order.
