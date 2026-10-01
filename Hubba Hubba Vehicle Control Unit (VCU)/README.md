# Hubba Hubba Vehicle Control Unit

This folder is the hardware-reference package for the custom Hubba Hubba VCU. It intentionally contains no firmware source code.

## Selected direction

The VCU uses one `ESP32-S3-WROOM-1U-N16R8` vehicle supervisor and four identical programmable motor-control nodes. Each node uses one `STSPIN32G4` and six external 100 V MOSFETs to drive one Hobbywing Skywalker 2820SL 550KV, 12N14P motor.

ST does **not** offer an exposed gull-wing-lead STSPIN32G4. The tray and tape-and-reel order codes are all the same 9 x 9 mm, 0.5 mm-pitch VFQFPN-64 package. Per the project decision to use QFN when no same-family leaded option exists, the selected controller is:

- ST `STSPIN32G4`
- LCSC `C7434864`
- VFQFPN-64, 9 x 9 mm, exposed center pad

Use JLCPCB assembly for this part. Audit or create the EasyEDA footprint from ST's recommended land pattern, use a segmented paste aperture and reviewed thermal-via field, and include inspection/rework access. The part is not intended to be treated as a hand-solderable leaded device.

## What is programmable

There are five processors on the VCU:

- one ESP32-S3 running the vehicle-level firmware; and
- four STM32G431 motor controllers embedded inside the four STSPIN32G4 devices, all running the same versioned motor-node firmware with a different wheel identity.

The ESP32 owns receiver capture, arming, steering, drive-mode and torque-vectoring decisions, GNSS, IMU, microSD logging, configuration, UI and global safety supervision. Each ST motor node owns its high-rate PWM generation, three-shunt current sampling, sensorless FOC observer, current control, gate-driver configuration, local protection, braking/reversal state machine and speed estimate.

The custom ESCs therefore **do need programming**. `CODE_HANDOFF.md` is the contract for the future coding agent. Code must be created elsewhere in the repository, never in this reference folder.

## Electrical scope

| Function | Selected implementation |
|---|---|
| Traction battery | User-selected 3S, 4S, 5S or 6S LiPo; 9.0 V provisional cold/load floor and 25.2 V full-charge maximum |
| Central logic feed | Separately fused upstream of the traction-branch contactor so the supervisor can boot without energizing the four inverter branches |
| Motors | 4 x Hobbywing Skywalker 2820SL 550KV, 14 poles / 7 pole pairs |
| Motor control | 4 x STSPIN32G4, sensorless three-shunt FOC, one external six-MOSFET bridge per wheel |
| Vehicle supervisor | ESP32-S3-WROOM-1U-N16R8; normal R8-module ambient ceiling is +65 °C |
| GNSS | MAX-M10S-00B; Dragy is not connected or supported |
| IMU | ASM330LHHTR automotive six-axis IMU |
| Storage | Onboard microSD in SPI mode with switched 3.3 V power |
| Fast phase-current sensing | Three 0.5 mOhm Kelvin shunts per motor node into the STSPIN32G4 internal op-amps/ADCs and comparators; gain and thresholds remain release-gated |
| Branch telemetry | One separate high-side 0.5 mOhm shunt and INA238 per motor branch; not used for the FOC loop or fast trip |
| Speed feedback | Each motor node reports signed observer speed through diagnostics; its validated, externally buffered 7-pulse-per-mechanical-revolution FG output gives speed magnitude only |
| Thermal monitoring | Two 10 kOhm NTCs per motor cell through two ADS1115 devices |
| Safety shutdown | Per-node hardware watchdog, independently supervised fail-low reset release, local TIM1 break/VDS protection, global fault latch/permit, branch fuses and an independent external contactor/E-stop |

The STSPIN32G4 accepts 5.5-75 V at `VM`, so the controller IC covers 3S-6S directly. That does not prove the complete inverter is safe across the range: MOSFET avalanche margin, DC-link overshoot, capacitor bias, BMS limits, cold 3S startup and regenerative braking still require measured validation.

The selected ESP32-S3 `N16R8` module is normally specified only to +65 °C ambient. Espressif documents an up-to-85 °C option when PSRAM ECC is enabled, at the cost of one-sixteenth of usable PSRAM, but that configuration and its memory impact remain release-gated. Keep the supervisor/RF zone within the qualified module limit and thermally isolated from the four inverter zones.

## Current rating

“80 A ESC” is a transient/headroom design target, not a promised continuous rating. The motor's published test point is 40.9 A for 46 seconds and 910.2 W for 46 seconds. The initial engineering envelope is:

- 40 A battery-side operating target per motor branch;
- 60 A provisional cycle-by-cycle phase-current limit; and
- 70-80 A provisional short fault/transient region.

Phase current, battery current and MOSFET current are not interchangeable. The final continuous and transient ratings are set by measured motor current, switching and conduction losses, shunts, copper/busbars, terminals, capacitors, cooling, battery/BMS capability and protection response. Nothing in this folder certifies an 80 A continuous output.

## Motor-node interface

CAN/TWAI is not used in this revision. All motor nodes are on the same PCB, so the selected boundary is:

- four independent hardware-timed signed command PWM inputs, one per node;
- four dedicated externally buffered FG pulse paths to the ESP32, one per node;
- a muxed external I2C2 diagnostics/configuration path on each node's exposed `PA8/PA9` pins, one isolated channel per node, with the node address distinct from its INA238 on that channel;
- one common hardware permit and fault latch, with a separate watchdog/reset sink at every node; and
- individual SWD pads for manufacturing, recovery and debug.

GPIO21 carries a windowed supervisor permit heartbeat/WDI, not a steady enable level. Its timing and startup behavior are schematic-dependent release parameters. The heartbeat may be active while the vehicle is disarmed so healthy nodes can remain out of reset for gate-disabled self-test and diagnostics; it is not torque permission. Normal startup first energizes each traction branch while that node's `NRST` remains asserted by a fail-low circuit, then releases reset only for gate-disabled self-test. Torque permission is a later, separate state requiring healthy nodes, valid heartbeats and the deliberate vehicle arm sequence.

The command PWM is a requested q-axis current/torque direction, not commutation PWM. Both extreme duty regions are invalid/OFF, a center band is neutral, and every node must remove gate drive after a stale or malformed command. External I2C2 is for version/status/configuration and must never be required for a real-time drive command or emergency shutdown. It is separate from the STSPIN32G4 gate driver's internal I2C3 connection on `PC8/PC9`; internal `READY`, `nFAULT` and I2C3 are not package pins.

## Integrated RPM limitations

There are no external HW86060041 sensors, phase-tap connectors or alternate RPM inputs. The speed source is integrated into each STSPIN32G4 motor node. Firmware converts observer electrical angle to mechanical speed using seven pole pairs and synthesizes a 7-PPR output for the ESP32.

This is phase-derived sensorless speed, not independent zero-speed sensing. It is invalid before observer lock and below the qualified BEMF range. Reverse requires zero request, near-zero measured current, a valid stopped state and a bench-qualified coast timeout of at least 120 ms; missing FG pulses alone never proves that the rotor has stopped.

## Folder contents

- `AGENTS.md` — durable hardware and safety rules for future agents.
- `BOM.xlsx` — formula-driven LCSC planning BOM, sizing checks, architecture map, release gates and external system items.
- `CODE_HANDOFF.md` — required behavior for the ESP32 and common STM32 motor-node firmware.
- `DATASHEETS.md` — LCSC procurement links and manufacturer documentation.
- `HARDWARE_ARCHITECTURE.md` — detailed electrical architecture, interfaces, power stages and validation plan.
- `PINOUTS.md` — consolidated provisional GPIO, peripheral, microSD/ESD, diagnostic-bus, motor-node and service-pin assignments.

## Release status

This is a researched design baseline, not a fabrication-ready schematic or a tested 80 A controller. Do not order the traction-power PCB until the open release gates in `BOM.xlsx` are closed, every schematic-dependent value has an exact LCSC part, the EasyEDA footprint/pin audit is complete, ERC/DRC and independent power-electronics review pass, and one motor cell has been qualified at restricted energy before four-cell testing.
