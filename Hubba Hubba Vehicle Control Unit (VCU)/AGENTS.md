# AGENTS.md

## Purpose and scope

This file is the durable engineering contract for agents working in the `Hubba Hubba Vehicle Control Unit (VCU)` folder. Read this file, `README.md`, `HARDWARE_ARCHITECTURE.md`, `CODE_HANDOFF.md`, `DATASHEETS.md` and every applicable manufacturer document before changing the design.

This folder is documentation-only. Do not add firmware, scripts, generated source, build systems, binary images or test programs here. Firmware belongs elsewhere in the `rc_car` repository. The only generated artifact allowed here is the user-facing `BOM.xlsx`.

Never present planned hardware, simulation, formula output, firmware tests or a single-cell bench result as completed four-motor physical validation.

## Frozen project decisions

- The VCU supports explicitly selected 3S, 4S, 5S or 6S LiPo operation.
- The vehicle supervisor is `ESP32-S3-WROOM-1U-N16R8`.
- Each wheel has an independent programmable ST motor node using `STSPIN32G4`, LCSC `C7434864`, and six external MOSFETs.
- ST offers no exposed gull-wing-lead STSPIN32G4. The authorized fallback is its 9 x 9 mm VFQFPN-64 package, not the slower leaded STSPIN32F0 family.
- Four STM32 motor-node instances must be programmed. The release contains exactly two firmware images: one ESP32 supervisor image and one common, reproducible STM32 image deployed to all four nodes with explicit per-wheel identity/configuration.
- Motor control is sensorless three-shunt FOC unless a reviewed revision deliberately changes it.
- Dragy is removed. MAX-M10S is the sole GNSS source.
- External HW86060041 RPM sensors and their connectors/conditioning are removed.
- RPM is integrated: every motor node derives speed from its phase-current/BEMF observer and produces a validated 7-PPR FG signal.
- CAN/TWAI is not fitted. Direct PWM commands, dedicated buffered FG paths, muxed I2C diagnostics and hardwired shutdown are the selected interfaces.
- The 80 A figure is transient/headroom, not continuous capability. The operating target is 40 A battery-side per branch.

Changing any frozen decision requires updating all five reference documents and every affected workbook sheet together.

## Hardware identity

- Motors: 4 x Hobbywing Skywalker 2820SL 550KV, `12N14P`, seven pole pairs.
- Published motor test point: 40.9 A and 910.2 W for 46 seconds; do not call this continuous.
- Motor controllers: 4 x `STSPIN32G4`, embedded STM32G431, 5.5-75 V `VM`, 1 A gate source/sink capability.
- Supervisor temperature: the selected ESP32-S3-WROOM-1U-N16R8 is normally limited to +65 °C ambient. The data-sheet option to reach +85 °C requires PSRAM ECC and reduces usable PSRAM by one-sixteenth; it is not assumed enabled until the build configuration, memory budget and thermal test are released.
- Bridges: 24 external 100 V N-channel MOSFETs, provisionally one per switch position.
- Current control: three Kelvin low-side phase shunts per node into PWM-synchronous local ADC/op-amp/comparator paths.
- Branch telemetry: separate high-side shunt plus INA238 per node; supervisory only.
- Sensors: ASM330LHHTR IMU and MAX-M10S GNSS.
- Storage: switched onboard microSD.
- Thermal: two NTCs per motor cell; all eight channels must be monitored and fault checked.
- Power sequencing: the separately fused central-logic feed is upstream of the traction-branch contactor. It may power the supervisor while all four inverter branches remain de-energized and must not provide a bypass into them.

The controller IC voltage and gate-current ratings do not establish ESC current capacity. MOSFET losses, shunt pulse energy, PCB/busbar geometry, terminals, DC-link ripple, cooling, fuse coordination and measured motor behavior determine the rating.

## Firmware ownership boundary

### ESP32-S3 owns

- Receiver CH1/CH2/CH4/CH5 capture and failsafe interpretation.
- Boot-lifetime arm policy, vehicle safe state and global drive permit.
- Steering relay, filtering, calibration and speed-sensitive limit.
- AWD/FWD/RWD selection and per-wheel torque-vectoring requests.
- IMU, GNSS, microSD logging, USB, Wi-Fi UI and persistent vehicle configuration.
- Four signed motor-command PWM outputs and four FG captures.
- Motor-node identity/version/health checks over muxed I2C.
- Global fault logging and contactor/E-stop status.

### Each STSPIN32G4 owns

- TIM1 complementary gate PWM, dead time and break handling.
- PWM-synchronous three-shunt acquisition and current-offset calibration.
- Sensorless FOC observer, q-axis current loop and speed validity.
- Gate-driver register setup, lock/readback and local VDS/UVLO/thermal handling.
- Command decoding, plausibility, rate limiting and local timeout.
- Current-limited forward drive, braking, coast and reversal sequencing.
- Signed observer-speed reporting through diagnostics and 7-PPR open-drain FG magnitude synthesis only while observer data is valid.
- Local watchdog service, fault output, diagnostic registers and safe reset behavior.

Do not make the ESP32 run commutation loops. Do not call a q-axis current request exact wheel torque until motor torque constant, current gain/offset, magnetic saturation and any field-weakening behavior have been measured.

## Supervisor pin contract

The provisional ESP32-S3 pin plan remains:

| Function | GPIO |
|---|---:|
| MAX-M10S UART RX / TX | 1 / 2 |
| Receiver CH1 / CH2 / CH4 / CH5 | 4 / 5 / 6 / 7 |
| Steering-servo output | 8 |
| Shared SPI SCLK / MOSI / MISO | 9 / 10 / 11 |
| ASM330LHH CS / INT1 | 12 / 13 |
| microSD CS / card detect | 14 / 15 |
| MAX-M10S TIMEPULSE | 16 |
| Internal supervisory I2C SDA / SCL | 17 / 18 |
| USB D- / D+ | 19 / 20 |
| Supervisor permit heartbeat / watchdog WDI | 21 |
| Motor signed-command PWM FL / FR / RL / RR | 38 / 39 / 40 / 41 |
| Motor FG FL / FR / RL / RR | 42 / 43 / 44 / 47 |
| TCA9548A active-low RESET | 48 |

GPIO35-37 are unavailable on the octal-PSRAM WROOM-1U variant. Re-run the complete module strapping, USB, boot and reserved-pin audit before schematic release.

## Motor-command and diagnostic interfaces

- Use one independent hardware-timed PWM command per motor node.
- Encode signed q-axis current request around a neutral center band. Reserve both duty-cycle rails as invalid/OFF regions.
- The node command-loss deadline is provisional at no more than 50 ms and must be bench measured.
- Every malformed, stale, out-of-range or implausibly changing command forces zero current and gate-off/coast through the local state machine.
- Keep four commands coherent at the ESP32's 20 ms vehicle-control cadence; local FOC continues at its much faster timer rate.
- Keep I2C off the real-time torque and shutdown paths.
- GPIO21 carries a windowed supervisor permit heartbeat/WDI, not a steady enable level. Its valid frequency, window, startup grace, polarity and timeout come from the final watchdog/latch schematic and remain release-gated. It may run in a healthy disarmed, reset-released self-test/diagnostic state; it does not grant torque. Firmware suppresses it on supervisor failure, a designated hard inhibit or shutdown.
- TCA9548A channels 0-3 each isolate one motor node's external diagnostic I2C2 slave on exposed `PA8/SDA` and `PA9/SCL`, plus its INA238. The node and INA238 on the same mux channel must have different frozen addresses; node addresses may repeat only across isolated channels. Channels 4-5 serve the two ADS1115 devices. Channels 6-7 remain reserved and are not routed off-board.
- Do not confuse that external diagnostic slave with the STSPIN32G4's internal gate-driver I2C3 connection on `PC8/PC9`. The internal bus, `READY` on `PE14`, and `nFAULT` on `PE15` are die-to-die connections inside the SiP and cannot be routed to the ESP32 or a board-level wire-OR net.
- TCA9534A may control only slow, non-critical functions such as microSD power, fans, indicators and latch-status inputs. It must not own gate enable, emergency shutdown, commutation or a braking request.
- CAN/TWAI must not be added casually. A future CAN revision requires a pin, transceiver, termination, protocol, EMC and failure-mode redesign.

## Integrated RPM contract

Motor speed is not supplied by a discrete external sensor. Each node estimates signed electrical angle/speed from its FOC observer and reports the signed result through diagnostics. The node generates one FG pulse for each full electrical revolution of absolute accumulated observer travel, in either direction, equal to seven rising edges per mechanical revolution.

Do not pull an unpowered motor-node MCU pin up from the central rail. Pass each FG source through a partial-power-down-safe open-drain buffer powered from the central logic domain, give its input a defined no-node state and pull its output up to clean central 3.3 V. A static level in either direction is not a valid speed measurement.

The one-wire FG frequency conveys speed magnitude only; it does not encode direction. The node must suppress/mark FG invalid during alignment, forced/open-loop acceleration, observer unlock, stale current sampling, local fault and low-speed operation below the qualified BEMF range. A quiet FG wire does not prove zero speed. The ESP32 must combine FG freshness with signed diagnostic state, current, command history and a coast timer. Validate every wheel against an optical tachometer through startup, handoff, steady speed, coast, braking and reversal.

## Safety invariants

1. Power-up, reset, brownout, unprogrammed flash and debugger attachment must leave all six gate outputs low.
2. Normal node startup energizes the protected traction branches while every STSPIN32G4 `NRST` remains asserted by a passive fail-low or independently supervised circuit. A node cannot boot while its VM branch is de-energized.
3. Supervised reset release permits gate-disabled boot and self-test only. Torque permit is a later, separate state requiring four healthy node instances, valid heartbeats, valid vehicle prerequisites and a deliberate arm transition.
4. Each reset path must remain asserted when the central control rail, its release logic or the node rail is absent or invalid. Use an independently powered suitable reset supervisor, a passive `NRST` pull-down with qualified active release, or an equivalently proven fail-low topology. `SN74LVC1G07` `Ioff` prevents backfeed but does not pull reset low when the buffer itself is unpowered and is not a fail-low solution by itself.
5. Fit one independent external watchdog per motor node and one for the ESP32. A missed heartbeat pulls that node reset low and reports into the global fault latch.
6. Configure gate-driver `nFAULT`/`READY` internal connections to TIM1 break inputs. Hardware VDS, interlock and minimum-dead-time protections remain enabled and are locked after verified setup.
7. Export a fail-safe node-health/fault signal. Local hard protection must disable its own bridge without ESP32 or I2C help; a node fault normally causes the global latch to reset all four nodes.
8. Hardware shutdown is coast/gates-off. Controlled braking is allowed only while the responsible motor MCU, phase-current sensing, bus measurement and gate driver are healthy.
9. The physical E-stop independently opens a DC-rated traction contactor. Closing the contactor or energizing VM is not torque permission. The reset/permit network is not certified safe torque off.
10. Never parallel four regulator/BEC outputs.
11. A cell-aware BMS or monitor is mandatory. STSPIN32G4 UVLO and total-pack INA238 readings do not protect individual LiPo cells.
12. Keep branch and main fuses, precharge/anti-spark, contactor, busbars, harness and cooling in the system design; they are not optional PCB afterthoughts.
13. Reverse requires zero command, near-zero phase and branch current, valid stopped state, no recent valid FG and a qualified coast delay of at least 120 ms.
14. Do not enable regenerative or active spin-down braking until full-6S bus rise, BMS charge acceptance and a possible brake chopper have been tested.

## Current-sense rules

- Use three 0.5 mOhm Kelvin phase shunts per motor node and a separate high-side 0.5 mOhm branch shunt.
- Never merge the Kelvin phase-sense returns with the INA238 measurement domain.
- INA238 is too slow for FOC and fast overcurrent protection.
- Select op-amp gain, bias, anti-alias filter, comparator threshold, blanking and ADC timing together from the released phase-current envelope.
- Treat 40 A branch DC, phase RMS, q-axis current, MOSFET current and 60/70/80 A protection values as different quantities.
- A 0.5 mOhm shunt dissipates 0.8 W at 40 A and 3.2 W at 80 A under ideal DC conditions; PWM duty and phase waveform change actual per-shunt heating.
- Use injected-fault tests to prove the local fast trip independent of normal firmware scheduling.

## QFN and power-layout rules

- Verify the STSPIN32G4 symbol, exposed pad, orientation and 64-pin mapping against the current ST data sheet, not only the EasyEDA library.
- Use the ST recommended land pattern as the starting point. Review paste segmentation, voiding target, thermal-via diameter/pitch, copper spreading, assembly class and rework method with JLCPCB.
- Keep each motor cell physically symmetric and independently probeable.
- Minimize each bootstrap, gate-drive and high-current commutation loop. Keep switch nodes away from current sense, crystals, I2C, USB, GNSS, IMU, receiver and reset nets.
- Kelvin-route every phase and branch shunt directly to its sensing device.
- Place the STSPIN buck diode, 18 uH inductor, VCC capacitor, VM bypass and three bootstrap capacitors exactly as the reference/current data sheet requires.
- For provisional 8 V gate drive, use the IPT015N10N5 data-sheet 2.0 mOhm maximum at 6 V as the conservative initial conduction bound. The 1.6 mOhm value is typical at 6 V, not a guaranteed maximum.
- Provide differential probe points for every gate-source pair, switch node, phase shunt, branch shunt and local DC link, plus reset, fault, FG and supply rails.
- Use an external metal baseplate/heatsink and reviewed isolation/interface material. Do not claim 40 A continuous from copper area alone.
- Keep the ESP32 module's local ambient within its released +65 °C limit, or explicitly release and verify the documented PSRAM-ECC 85 °C configuration and reduced memory budget.

## Programming and service access

- Provide four clearly labelled five-signal SWD pad groups: `SWDIO`, `SWCLK`, `NRST`, local `3V3` reference and `GND`.
- Define BOOT0 behavior and prevent floating boot configuration.
- Provide a production fixture plan and a way to identify FL/FR/RL/RR without compiling four unrelated images.
- Firmware release records must include toolchain, STM32Cube/MCSDK versions, build hash, protocol version, calibration schema and all four readbacks.
- Never permit arbitrary motor-node flashing from the browser or while traction power is connected.

## BOM and sourcing discipline

- Every production component must have an exact manufacturer part number and an LCSC listing before schematic release.
- Inventory is a dated snapshot. Recheck stock, lifecycle, JLCPCB assembly eligibility and minimum quantity immediately before ordering.
- Never substitute a controller, MOSFET, shunt, capacitor, inductor, TVS, fuse, regulator, sensor or connector on package similarity alone.
- `Selected` means the exact part has been chosen; it does not mean the circuit is electrically released.
- `Provisional` means an exact stocked candidate exists but a calculation or test can still change it.
- `Schematic dependent` means the value/quantity is not frozen. No required zero-quantity placeholder may remain in the production BOM.
- Keep formulas live in `BOM.xlsx`; do not replace calculated cells with typed results.

## Documentation synchronization

Any change to controller, interface, pin, current limit, command encoding, speed source, cell count, power tree or safety state must be reflected in:

1. `README.md`;
2. `HARDWARE_ARCHITECTURE.md`;
3. `CODE_HANDOFF.md`;
4. `DATASHEETS.md`; and
5. all affected `BOM.xlsx` sheets.

Before handoff, search the entire folder for stale controller names or obsolete assumptions, inspect workbook formulas for errors, render every workbook sheet, and confirm the folder still contains no code.
