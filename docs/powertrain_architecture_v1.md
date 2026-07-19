# 4 Hub Motor RC Car Powertrain Architecture - V1

## Project Goal

Build an ESP32-based powertrain controller for a 1/7 scale RC car using four identical hub motors and four Hobbywing Skywalker 50A V2 ESCs. V1 only controls motor drive. Steering servo control stays on the RC system directly.

All four ESCs receive the same throttle and reverse commands, so all motors run at the same requested speed.

## Components

- Controller: ESP32 DOIT DevKit WROOM-32
- Motors: 4x Hobbywing Skywalker 2820 550KV brushless motors
- ESCs: 4x Hobbywing Skywalker 50A V2
- Input: RC receiver throttle channel and a physical arm switch
- Power: traction battery connected to each ESC power input

## Confirmed ESC Signal Behavior

From `Skywalker_ESC_Manual.pdf`:

- The tri-color throttle cable contains:
  - White: throttle signal input
  - Red: BEC 5V output
  - Black: ground
- The default throttle range is 1100 us to 1940 us.
- The ESC should be throttle-calibrated whenever used with a new transmitter/controller.
- The yellow reverse brake signal wire is a separate RC-style input.
- For Reverse Brake mode:
  - The yellow wire's signal range must match the throttle range.
  - 0-50% on the reverse channel means default motor direction.
  - 50-100% on the reverse channel commands reverse direction.
  - On first ESC power-up, the reverse channel should be in the 0-50% range, preferably 0%.
  - When reverse is activated, the ESC stops the motor first, then spins in reverse and increases to the speed commanded by throttle.
  - Loss of either throttle signal or reverse signal can trigger ESC throttle signal loss protection.
- For Linear Reverse Brake mode:
  - The yellow wire is also used, but motor speed in reverse is controlled by the linear reverse channel.
  - The manual says the linear reverse stroke is fixed to 1.34 ms to 1.79 ms.
  - This mode is less suitable for V1 because it splits forward and reverse speed control across different channels.

Recommended V1 ESC setting: `Brake Type = Reverse`, not `Linear Reverse`.

## High-Level Architecture

```text
RC Receiver Throttle PWM
        |
        v
ESP32 input capture
        |
        v
Signal validation, calibration, deadband, ramp limiting, failsafe
        |
        +--------------------+
        |                    |
        v                    v
Throttle PWM output      Reverse PWM output
to all four ESCs         to all four ESC yellow wires
```

## Wiring Concept

### Receiver to ESP32

- Receiver throttle signal -> ESP32 pulse input GPIO
- Receiver ground -> ESP32 ground
- Arm switch -> ESP32 `GPIO33` and ground, using the ESP32 internal pull-up
- Optional receiver aux/reverse channel -> ESP32 pulse input GPIO

The ESP32 GPIO is 3.3V logic. Many receivers output 3.3V-compatible PWM, but verify this before direct connection. If the receiver signal is 5V, use a level shifter or resistor divider into the ESP32 input.

Arm switch wiring:

```text
GPIO33 ---- switch ---- GND
```

The firmware enables the ESP32 internal pull-up. Switch open reads high and means arm not requested. Switch closed reads low and means arm requested.
Use `monitor arm` over USB serial during bench testing to confirm the digital input changes correctly. Open should read digital HIGH/OFF; closed should read digital LOW/ON. Keep this pin digital-only in firmware; if voltage measurement is needed, use a multimeter or a separate ADC sense pin.

### ESP32 to ESCs

- ESP32 throttle PWM output -> all four ESC white throttle signal wires
- ESP32 reverse PWM output -> all four ESC yellow reverse signal wires
- ESP32 ground -> all ESC signal grounds

Use one shared signal ground across receiver, ESP32, and ESC signal grounds.

### ESC BEC Red Wires

Do not tie all four ESC BEC 5V red wires together casually. For V1:

- Use one ESC BEC, or a dedicated 5V regulator, to power the ESP32 through a suitable 5V input.
- Remove or isolate the red BEC wires from the other ESC throttle connectors.
- Keep all black/brown grounds connected.

The ESP32 can have current spikes from Wi-Fi/Bluetooth even if unused, so a dedicated regulator is often cleaner than relying on one ESC BEC.

## Control Model

Use one normalized command internally:

```text
drive_command = -1.0 to +1.0

-1.0 = full reverse
 0.0 = neutral / stopped
+1.0 = full forward
```

For ESC Reverse Brake mode:

- Throttle output always represents requested speed magnitude.
- Reverse output represents direction.

```text
if drive_command >= 0:
    throttle_pwm = map(abs(drive_command), 0..1, throttle_min..throttle_max)
    reverse_pwm = reverse_low
else:
    throttle_pwm = map(abs(drive_command), 0..1, throttle_min..throttle_max)
    reverse_pwm = reverse_high
```

Suggested defaults after calibration:

- `throttle_min`: calibrated minimum, initially 1100 us
- `throttle_neutral`: same as minimum for aircraft-style ESC throttle
- `throttle_max`: calibrated maximum, initially 1940 us
- `reverse_low`: calibrated minimum or a conservative low value such as 1100 us
- `reverse_high`: calibrated maximum or a conservative high value such as 1940 us
- `reverse_threshold`: midpoint between reverse low and high

## Receiver Interpretation

Most RC car throttle channels center at neutral and use one side for forward and the other side for braking/reverse. The Skywalker ESC throttle input is aircraft-style, where low throttle means stop and high throttle means speed.

The ESP32 should therefore translate the receiver throttle channel into the normalized `drive_command`:

- Receiver center pulse -> `0.0`
- Receiver forward endpoint -> `+1.0`
- Receiver reverse endpoint -> `-1.0`

The exact pulse directions should be learned during calibration, because transmitter channel direction may be reversed.

## Calibration Modes

### 1. Receiver Input Calibration

Purpose: learn the user's transmitter/receiver range.

Store:

- Receiver minimum pulse
- Receiver neutral/center pulse
- Receiver maximum pulse
- Direction polarity
- Deadband around neutral

Current V1 receiver defaults:

- Full throttle: `1750 us`
- Neutral: `1250 us`
- Full reverse: `1000 us`
- Deadband: `80 us`

Suggested process:

1. User sends `cal receiver` over the USB serial CLI.
2. Firmware prompts the user to hold neutral throttle and press Enter.
3. ESP32 records neutral throttle.
4. Firmware prompts the user to hold full throttle and press Enter.
5. ESP32 records full throttle.
6. Firmware prompts the user to hold full reverse and press Enter.
7. ESP32 records full reverse.
8. ESP32 validates that full throttle and full reverse are distinct and on opposite sides of neutral.
9. Values are saved to ESP32 NVS flash.

### 2. ESC Output Calibration

Purpose: teach all four ESCs the ESP32 throttle output range.

Manual sequence:

1. Disconnect drive wheels from the ground.
2. ESP32 outputs max throttle PWM to all ESC throttle inputs.
3. Power ESCs.
4. ESCs beep twice to accept maximum endpoint.
5. Within 5 seconds, ESP32 outputs minimum throttle PWM.
6. ESCs accept minimum endpoint and finish calibration after cell-count beeps and long ready beep.

Important: during ESC calibration, keep the reverse output in the low/default-direction state.

For firmware, implement this as a deliberate USB serial "ESC calibration mode" that cannot be entered accidentally at boot.

## Laptop Serial Calibration

V1 calibration is entered from a laptop over the ESP32 devkit USB serial port at `115200` baud. Physical calibration buttons can be added later, but are not required for V1.

Supported commands:

- `status`: print current state, receiver pulse, arming state, saved calibration, and output pulse values.
- `arm`: enter drive mode after verifying the arm switch is on, receiver signal is valid, and throttle is neutral.
- `disarm`: leave drive mode and restore safe outputs. Turning the physical arm switch off does the same.
- `config reverse <0-100>`: set the maximum reverse throttle as a percentage of full ESC throttle. Default is `10%`.
- `config failsafe <pulse_us> <window_us>`: set receiver transmitter-off failsafe pulse detection. Default is `1565 +/- 10 us`.
- `monitor throttle`: print the receiver throttle pulse width for 30 seconds to verify the ESP32 is receiving the signal.
- `monitor arm`: print the arm switch digital state for 30 seconds.
- `cal receiver`: start receiver input calibration and save neutral, full throttle, and full reverse.
- `cal esc arm`: prepare ESC calibration mode while ESC battery power is still disconnected.
- `cal esc max`: output throttle max to all four ESC throttle outputs and reverse low to all four reverse outputs.
- `cal esc min`: output throttle min to all four ESC throttle outputs and reverse low.
- `cal manual`: relay the receiver throttle pulse directly to all four ESC throttle outputs for 30 seconds, with all reverse outputs held low.
- `cal cancel`: return to disarmed safe output.

Receiver calibration requires the controller to be disarmed and a valid receiver throttle signal to be present. Guided ESC calibration commands also require the receiver throttle to match the saved neutral value. Manual ESC calibration relay requires a valid receiver throttle signal but does not require neutral, because the transmitter position is intentionally passed through.

Drive mode:

1. User turns the arm switch on, or sends `arm` while the arm switch is on, receiver signal is valid, and the saved neutral throttle is present.
2. ESP32 maps calibrated receiver full-throttle/neutral/full-reverse values into one normalized drive command.
3. All four ESC throttle outputs receive the same mapped throttle magnitude.
4. All four ESC reverse outputs receive `reverse_low` for forward and `reverse_high` for reverse.
5. Reverse throttle magnitude is limited by the configured reverse limit, defaulting to `10%`, so full reverse transmitter input only commands 10% of full ESC throttle.
6. If requested direction changes, throttle is ramped to minimum and held briefly before the reverse output changes.
7. User opens the arm switch, sends `disarm`, or sends `stop` to return to safe output.
8. Receiver signal loss immediately returns to disarmed safe output.
9. If the receiver outputs the configured transmitter-off failsafe pulse, default `1565 +/- 10 us`, for 300 ms, drive mode disarms and restores safe output.
10. After receiver signal loss or transmitter-off failsafe, the arm switch must be cycled off then on before drive mode can arm again.

Guided laptop ESC calibration flow:

1. Connect the laptop USB cable and open the ESP32 serial monitor at `115200`.
2. Keep ESC battery power disconnected.
3. Send `cal esc arm`.
4. Confirm the firmware reports ESC calibration armed.
5. Send `cal esc max`.
6. Power the ESCs.
7. Wait for the ESC max-endpoint beeps.
8. Send `cal esc min` within the ESC manual's 5-second calibration window.
9. Wait for the ESCs to accept minimum throttle and finish their ready beeps.
10. Send `cal cancel`, or let the 30-second calibration timeout return outputs to safe stop.

During the entire ESC calibration procedure, all four reverse outputs stay at `reverse_low`.

Manual laptop ESC calibration relay:

1. Keep the car safely lifted and restrained.
2. Hold the transmitter throttle at the position the ESC should see.
3. Send `cal manual`.
4. The ESP32 copies the live receiver throttle pulse directly to all four ESC throttle outputs.
5. All four reverse outputs remain at `reverse_low`.
6. Send `cal cancel` to stop early, or let the 30-second timeout restore safe output.

## Safety and Failsafe Requirements

Minimum V1 safety behavior:

- Do not output drive throttle until a valid receiver signal has been seen.
- On receiver signal loss for more than 100 ms, command stop immediately.
- Treat the receiver's transmitter-off failsafe pulse as invalid. The current measured value is `1565 us`, using a tight `+/- 10 us` detection window and a 300 ms hold before disarming.
- Continue sending valid stop throttle and default reverse PWM so the ESC does not see signal loss unless the ESP32 itself fails.
- Require arm switch on and neutral throttle before arming.
- Require the arm switch to be cycled off then on after receiver signal loss or transmitter-off failsafe before re-arming.
- Apply a deadband around receiver neutral to prevent creeping.
- Apply acceleration ramp limiting to avoid sudden torque spikes across four hub motors.
- Apply direction-change interlock:
  - If command crosses from forward to reverse or reverse to forward, first ramp throttle to zero.
  - Hold zero throttle briefly.
  - Switch reverse output.
  - Ramp throttle back up.
- Use the physical arm switch as the V1 kill/disarm input; a receiver aux kill channel can be added later.
- Log or blink fault codes for signal loss, calibration invalid, not-neutral-on-boot, and output fault.

## Suggested ESP32 Firmware Modules

- `rc_input`: captures receiver PWM pulse width using RMT or MCPWM capture.
- `calibration`: manages receiver calibration and NVS persistence.
- `drive_mapper`: converts receiver pulse to normalized drive command.
- `safety`: arming, failsafe, neutral check, direction-change interlock.
- `pwm_output`: generates ESC throttle and reverse PWM using LEDC or MCPWM.
- `diagnostics`: serial logging and LED/status output.

## Timing Targets

- RC input expected range: roughly 1000-2000 us
- ESC default range: 1100-1940 us
- PWM frame rate: start with standard servo rate around 50 Hz unless testing confirms the ESC accepts higher rates
- Failsafe timeout: 100-250 ms
- Neutral deadband: start around 30-50 us, then tune
- Direction-change zero hold: start around 250-500 ms, then tune

## Useful Additions

- Use guarded USB serial commands for V1 calibration; an external arming/calibration button can be added later.
- Add a status LED:
  - Fast blink: no receiver signal
  - Slow blink: valid signal, disarmed
  - Solid: armed
  - Pattern blink: calibration/fault
- Add current or voltage telemetry later if available from separate sensors.
- Consider per-wheel ESC outputs in V2 even if they initially mirror each other. That makes future traction control, diagnostics, or trim possible without rewiring.
- Use waterproof/strain-relieved signal harnessing. Four ESC signal lines in a high-current RC car will be noise-prone.

## Open Hardware Checks

- Confirm receiver PWM voltage level before connecting to ESP32 GPIO.
- Confirm whether all four ESC yellow reverse wires can be driven from one ESP32 output without buffering. If uncertain, use a small logic buffer or distribute through separate ESP32 outputs.
- Confirm motor direction wiring for each hub motor. Swap any two motor phase wires on a wheel if its physical direction is wrong relative to the others.
- Confirm battery voltage/cell count matches the Skywalker 50A V2 rating. The regular Skywalker 50A V2 is listed as 3-4S LiPo, while the 50A-6S V2 is 3-6S.
- Bench-test Reverse Brake mode with one ESC and motor before wiring all four.

## Initial Bench Test Plan

1. Program one ESC to `Brake Type = Reverse`.
2. Connect one motor and one ESC to the ESP32.
3. Send throttle minimum and reverse low at boot.
4. Verify ESC arms.
5. Sweep throttle in forward direction.
6. Return throttle to zero.
7. Set reverse high.
8. Sweep throttle again and verify reverse spin.
9. Repeat with all four ESCs, unloaded.
10. Test receiver loss and kill/disarm behavior before wheels touch the ground.
