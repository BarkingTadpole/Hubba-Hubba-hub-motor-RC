# RC Car Powertrain Controller

ESP-IDF firmware for a 1/7 scale RC car powertrain controller using an ESP32 DOIT DevKit and four Hobbywing Skywalker 50A V2 ESCs.

V1 mirrors one logical drive command across four throttle PWM outputs and four reverse PWM outputs. Calibration is controlled from a laptop over the ESP32 USB serial console at `115200` baud.

## Project Layout

```text
CMakeLists.txt
README.md
Skywalker_ESC_Manual.pdf
docs/
  powertrain_architecture_v1.md
main/
  CMakeLists.txt
  pin_config.h
  rc_car_main.c
```

## Serial Commands

- `status`: print controller state, receiver pulse, calibration, and output pulse data.
- `arm`: enter drive mode; requires the arm switch on plus valid neutral receiver throttle.
- `disarm`: leave drive mode and restore safe outputs; turning the arm switch off also disarms.
- `config reverse <0-100>`: set maximum reverse throttle as a percentage of full ESC throttle.
- `config failsafe <pulse_us> <window_us>`: set receiver transmitter-off failsafe pulse detection.
- `monitor throttle`: print receiver throttle pulse width for 30 seconds.
- `monitor arm`: print arm switch digital state for 30 seconds.
- `cal receiver`: capture receiver neutral, full throttle, and full reverse.
- `cal esc arm`: prepare ESC calibration with ESC battery disconnected.
- `cal esc max`: output throttle max and reverse low.
- `cal esc min`: output throttle min and reverse low.
- `cal manual`: relay receiver throttle directly to all ESC throttle outputs for 30 seconds.
- `cal cancel`: restore disarmed safe outputs.
- `help`: print the command list.

## Notes

- Normal boot immediately commands throttle minimum and reverse low on all ESC outputs.
- Arm switch wiring is `GPIO33 -> switch -> GND`; the firmware enables the ESP32 internal pull-up, so closed means arm requested.
- `monitor arm` is a bench-test helper for the arm switch input. Open should read digital HIGH/OFF; closed should read digital LOW/ON.
- Drive mode maps the calibrated receiver throttle positions into ESC throttle magnitude and reverse-wire direction.
- Default receiver calibration is full throttle `1750 us`, neutral `1250 us`, full reverse `1000 us`.
- Default reverse throttle limit is `10%`; at full reverse transmitter input the ESC throttle output is only 10% of full ESC throttle.
- Default receiver transmitter-off failsafe detection is `1565 +/- 10 us`; if that pulse persists while driving, the controller disarms and restores safe outputs.
- After receiver signal loss or transmitter-off failsafe, the arm switch must be cycled OFF then ON before drive mode can arm again.
- Receiver signal loss while driving immediately disarms and restores safe outputs.
- ESC calibration is never entered automatically at boot.
- See `docs/powertrain_architecture_v1.md` for wiring and calibration details.
