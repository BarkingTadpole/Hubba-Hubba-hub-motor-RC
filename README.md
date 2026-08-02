# RC Car Powertrain Controller

ESP-IDF firmware for a four-motor 1/7 scale RC car using an ESP32 DOIT
DevKit, four Hobbywing Skywalker 50A V2 ESCs, a Radiolink RC6GS V3/R7FG
radio system, four Hobbywing HW86060041 RPM sensors, and an Adafruit
ISM330DHCX IMU.

The normal drive path supports forward/reverse, receiver calibration,
remote shutdown, receiver-loss protection, and per-wheel ESC outputs.
Torque vectoring is implemented but defaults to disabled until its sensors
are validated and it is explicitly enabled.

## Project Layout

```text
main/
  app_main.c                 Startup only
  cli.c                      USB serial command parser
  config_store.c             NVS calibration and configuration
  powertrain_controller.c    Drive state, safety, and calibration workflows
  rc_input.c                 Four receiver PWM inputs
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
2. Configure receiver failsafe: CH2 throttle at the known failsafe position
   and CH4 at STOP.
3. Send `cal receiver`.
4. Send `cal steering`.
5. Send `cal arm`.
6. Send `cal tv`.
7. Leave CH4 in STOP and calibrate the four ESC endpoints.
8. Send `cal imu` with the car level and still after every ESP32 boot.
9. Use `monitor rpm` and rotate each wheel to validate all four speed inputs.
10. Keep torque vectoring disabled for initial equal-output drive testing.

To drive, put CH4 in STOP once after boot, hold throttle neutral, then move
CH4 to RUN. After a receiver-loss or failsafe event, repeat that STOP-to-RUN
cycle; no laptop command is needed.

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
config failsafe <pulse_us> <window_us>
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
