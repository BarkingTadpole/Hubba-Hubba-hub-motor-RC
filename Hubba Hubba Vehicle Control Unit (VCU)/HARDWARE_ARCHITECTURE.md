# Hubba Hubba VCU Hardware Architecture

## 1. Design status

Revision: programmable ST motor-control baseline, 2026-09-03.

This document defines the proposed Hubba Hubba Vehicle Control Unit: one ESP32-S3 supervisor and four independent, onboard, programmable three-phase inverter nodes. Each inverter uses an STSPIN32G4 system-in-package and external MOSFETs. This is a reference architecture, not a released schematic, a continuous-current certification, or permission to connect a traction battery. Values marked provisional must be recalculated from the final schematic and proven on hardware.

The selected controller is **STSPIN32G4, LCSC C7434864**, in the only package offered for this family: a 9 mm x 9 mm, 0.5 mm-pitch VFQFPN-64 with a central exposed pad. There is no exposed-gull-wing or other leaded STSPIN32G4 variant. The user has accepted this QFN fallback. Solder-paste design, exposed-pad voiding, via construction, assembly process control and inspection are therefore release requirements, not manufacturing details to defer.

The STSPIN32G4 contains a programmable STM32G431 motor-control MCU. All four nodes must be programmed and verified before the VCU can operate. The VCU reference folder remains documentation-only; the separate motor-node and ESP32 firmware projects are defined by `CODE_HANDOFF.md`.

## 2. Battery range and inverter sizing

The traction input is user-selectable 3S-6S LiPo. Design endpoints are a cold/discharged 3S installation at the low end and 25.2 V from a fully charged 6S pack at the high end. Use 9.0 V only as a provisional cold/load design point. Final warning, derating and cutoff thresholds must come from the selected cells and BMS.

Pack voltage alone cannot safely identify cell count because operating ranges overlap. Store an explicit `3..6` cell-count setting and use total-pack voltage only as a plausibility check. A cell-aware BMS or low-voltage monitor must enforce per-cell limits independently. The STSPIN32G4 supply protections protect electronics; they are not per-cell LiPo discharge protection.

| Pack | Nominal | Fully charged | Ideal no-load speed at 550 KV | Electrical frequency at 7 pole pairs |
|---:|---:|---:|---:|---:|
| 3S | 11.1 V | 12.6 V | 6,930 rpm | 808.5 Hz |
| 4S | 14.8 V | 16.8 V | 9,240 rpm | 1,078 Hz |
| 5S | 18.5 V | 21.0 V | 11,550 rpm | 1,347.5 Hz |
| 6S | 22.2 V | 25.2 V | 13,860 rpm | 1,617 Hz |

The 6S ideal no-load point is an important control-bandwidth case. A 20 kHz FOC/PWM rate gives only about 12.4 updates per electrical cycle at 1.617 kHz; 25 kHz gives about 15.5 and 30 kHz about 18.6. Select the final PWM rate only after verifying STM32G431 execution margin, current-sampling windows, observer bandwidth, switching loss, acoustic behavior and gate-driver temperature. Loaded speed is lower, but overspeed and field behavior still need explicit limits.

Hobbywing publishes the 2820SL 550KV as a 6S, 12N14P motor with 60.8 mOhm resistance, a `40.9 A / 46 s` operating point and `910.2 W / 46 s`, and recommends an 80 A commercial ESC. This supports an 80 A **class** as useful transient/fault headroom. It does not establish an indefinite 80 A custom-board rating.

| Current domain | Planning value | Meaning |
|---|---:|---|
| Branch DC operating target | 40 A | Battery-side average per inverter; thermally derated |
| Local phase-current ceiling | Provisional 60 A | Fast FOC/comparator limit; phase current is not the same as battery current |
| Lock/hard-fault region | Provisional 70-80 A | Threshold, tolerance, blanking and permitted duration remain open |
| Four-motor published-point aggregate | 163.6 A / 3.64 kW | External pack, BMS, PDU, contactor, harness, fuse and cooling problem |

At 25.2 V and 60.8 mOhm, a simplistic cold locked-rotor calculation is about 414 A before battery, cable, inverter and inductive effects. It is not a predicted operating current; it demonstrates why cycle-by-cycle current protection and independent hardware shutdown are mandatory. Phase current can greatly exceed branch battery current at low PWM duty.

## 3. Controller decision and package constraint

| Candidate | Result | Reason |
|---|---|---|
| STSPIN32G4, LCSC C7434864 | Selected | STM32G431, three-phase 1 A source/sink gate driver, three internal op amps, comparators, fast ADCs, motor timers, VDS monitoring and 5.5-75 V motor-supply range |
| Leaded member of the same ST family | Unavailable | ST offers this part only in VFQFPN-64; no exposed-lead equivalent preserves this integration and capability |
| Four commercial 3-6S ESCs | System fallback | Use if custom high-current power-stage validation, QFN process control or motor-control firmware cannot be completed safely |

The selected approach is programmable. It enables true phase-current control, tunable sensorless FOC, integrated speed estimation and deterministic local protection, but it also creates four safety-critical programmed motor-node instances. There are two firmware images in a vehicle release: one ESP32 supervisor image and one common STM32 image deployed to all four nodes. A board is not ready because it powers up or spins a motor; firmware identity, current-offset calibration, observer performance, watchdog behavior and fault response are release evidence.

Primary references are the [STSPIN32G4 data sheet](https://www.st.com/resource/en/datasheet/stspin32g4.pdf), the STM32G431 documentation, ST motor-control references and the exact external-MOSFET data sheet.

## 4. Functional partition and interfaces

| Owner | Responsibilities |
|---|---|
| ESP32-S3 supervisor | Receiver capture, steering, arming, drive profiles, torque-vectoring requests, drivetrain mode, IMU, GNSS, logging, user interfaces, system plausibility, external-watchdog service and four independent signed motor commands |
| Each STSPIN32G4 node | Sensorless three-shunt FOC, current-loop execution, startup/observer handoff, signed torque request, direction interlock, braking policy, phase-current protection, VDS/thermal protection handling, local watchdog and synthesized speed output |
| Independent hardware | Branch/main fusing, E-stop/contactor, precharge, default reset/inhibit, external watchdog and fault latch, DC-link transient control and thermal supervision |

Normal control is:

`receiver/sensors -> ESP32 -> four signed PWM commands -> four STSPIN32G4 FOC nodes -> four MOSFET bridges -> four motors`

High-rate commutation and current control stay local to each motor node. The ESP32 sends a bounded signed torque/current request every 20 ms and never generates phase PWM.

### Why CAN/TWAI is absent

CAN/TWAI is intentionally not used. All four motor nodes are on the same PCB and need only one bounded command, one speed return and hardware fault/reset behavior per wheel. Dedicated command PWM, synthesized FG, four node-watchdog outputs combined by a global latch, a hardware reset path, and low-rate muxed I2C diagnostics are simpler and easier to fault-inject than four transceivers plus a safety-relevant network protocol. This decision does not prevent a future networked revision.

Each node exposes a bounded diagnostic I2C2 slave through package pins `PA8/SDA` and `PA9/SCL`. It reports identity, version, state, observer validity, current limits and fault history, but is never the torque-command or emergency-shutdown path. Per-node recovery and deep service still use individual SWD pads and optional isolated service-UART test pads while traction is safely isolated.

## 5. ESP32-S3 supervisor and retained provisional pin plan

Use ESP32-S3-WROOM-1U-N16R8. The `1U` module requires an external 2.4 GHz antenna. GPIO35-37 are unavailable because the N16R8 module uses octal PSRAM. GPIO0, GPIO3, GPIO45 and GPIO46 remain unloaded boot straps. Native USB uses GPIO19/20.

Espressif specifies the selected R8 module for a normal maximum ambient of +65 °C. The documented option to reach +85 °C requires PSRAM ECC and reduces usable PSRAM by one-sixteenth. Do not assume that option: freeze it in the ESP-IDF build and release manifest, verify the reduced memory budget, and thermally qualify the supervisor/RF zone apart from the inverter hot zones.

The following allocation is retained. Confirm matrix functions, boot states, pulls, drive levels and timing in the final schematic and an ESP-IDF target build.

| Function | GPIO | Notes |
|---|---:|---|
| MAX-M10S UART RX/TX | 1 / 2 | ESP RX receives GNSS TX |
| Receiver CH1/CH2/CH4/CH5 | 4 / 5 / 6 / 7 | Through 5 V-tolerant input buffers and connector ESD |
| Steering-servo output | 8 | Through 5 V AHCT buffer and dedicated low-capacitance ESD; servo power external |
| Shared SPI SCLK/MOSI/MISO | 9 / 10 / 11 | IMU and microSD have separate chip selects |
| IMU CS / INT1 | 12 / 13 | High-priority inertial acquisition |
| microSD CS / card detect | 14 / 15 | SD power switch controlled by TCA9534A |
| MAX-M10S TIMEPULSE | 16 | Input only; remain high-impedance during GNSS startup because this module pin is shared internally with SAFEBOOT_N |
| Internal supervisory I2C SDA/SCL | 17 / 18 | TCA9548A and TCA9534A; no off-board branch |
| USB D- / D+ | 19 / 20 | Matched pair and ESD |
| Supervisor permit heartbeat / watchdog WDI | 21 | Windowed heartbeat, not a steady enable; timing remains schematic-dependent and release-gated |
| Motor command FL/FR/RL/RR | 38 / 39 / 40 / 41 | Four independent center-neutral signed PWM inputs to the motor nodes |
| Motor FG FL/FR/RL/RR | 42 / 43 / 44 / 47 | Four dedicated, externally buffered 7-PPR observer-derived returns |
| TCA9548A active-low RESET | 48 | Pull up passively; assert before I2C initialization and for stuck-branch recovery |
| BOOT | 0 | Button/test pad only |
| Reserved straps | 3 / 45 / 46 | No functional load |

There is no spare direct aggregate-fault GPIO in this plan. The hardware fault latch independently asserts all four node resets, and its non-safety status is made available through a protected low-speed input. The STSPIN32G4 `READY` and `nFAULT` signals are internal die-to-die connections and cannot be routed to the ESP32 or a board-level wire-OR net. If a later build needs a direct fault interrupt, a listed function must be deliberately removed or multiplexed after a timing and boot review.

### Low-speed supervisory I/O

TCA9534A owns only non-time-critical functions such as microSD rail enable, indicators or fan request, and fault-latch/status readback. Every output has a passive safe default. It must never own phase PWM, motor command timing, receiver capture, steering timing, FG capture, node reset or emergency torque removal. A mismatch or bus failure blocks drive permit but cannot itself be the mechanism that removes torque.

TCA9548A channels 0-3 each contain one INA238 branch monitor and the corresponding node's external diagnostic I2C2 slave on `PA8/PA9`. Within one mux channel, the INA238 and motor-node slave must use different frozen addresses and both identities must be read back. Channels 4-5 each contain one ADS1115 thermal ADC. Channels 6-7 are reserved, deselected by default and not routed off-board. GPIO48 directly resets the mux. Before initialization, and after any timeout or stuck-low condition, keep all motor nodes inhibited, reset the mux, restart with every channel deselected and health-check every required device. Recovery requires deliberate rearming.

The external diagnostic I2C2 slave must not be confused with the STSPIN32G4 gate-driver I2C3 connection, which is internal to its embedded MCU (`PC8/I2C3_SCL` and `PC9/I2C3_SDA`). The local node firmware is the sole owner of those internal configuration/status registers; the gate driver itself is not a device on the ESP32 mux. The four external node-slave addresses may be identical across isolated mux channels, but a node address must never collide with the INA238 address on its own channel.

## 6. GNSS, IMU, storage, USB and power trees

- MAX-M10S-00B is the sole GNSS source and uses UART, TIMEPULSE, reset/backup support, U.FL, controlled-impedance RF routing and a very-low-capacitance RF ESD device. Keep the ESP32 TIMEPULSE input high-impedance during module startup, include protection and matching footprints in the 50 ohm model, and validate the populated path with a VNA.
- ASM330LHHTR uses SPI. Mount it rigidly near the vehicle centerline, mark +X/+Y/+Z and preserve the current +X rear/+Y right convention or document a transform.
- The microSD socket shares SPI with the IMU but has its own chip select, detect input and switched 3.3 V. Give IMU acquisition priority; logging must tolerate bounded stalls, removal and power loss. Never autoformat without explicit consent.
- USB-C is logic/debug power and data. Fit CC pull-downs, ESD and source isolation. USB must not energize an inverter or backfeed traction power.
- Do not fit or route a Dragy UART, I2C, power or connector interface. MAX-M10S replaces it.

The central logic tree uses a separately fused raw 3S-6S feed upstream of the traction-branch contactor: 60 V TPS54360B buck -> regulated 5 V, safely muxed with USB VBUS, followed by a 3.3 V buck. This lets the supervisor boot while the four motor branches remain de-energized; the logic feed must not bypass the contactor into any inverter power path. Low-noise LDO branches feed GNSS and IMU. The TPS54360B compensation, UVLO and power-stage values in the BOM are reference-design starting points; prove 9.0 V cold/load startup, dropout, compensation, EMI, load transient and temperature with the actual system load.

Each motor node is powered from its local protected VM branch. Its STSPIN32G4 internal buck generates gate-drive VCC and its regulator chain generates 3.3 V for the embedded MCU. An 8 V VCC setting is the provisional starting point because a step-down regulator cannot maintain 10, 12 or 15 V from a 9.0 V 3S endpoint. Prove buck dropout and startup margin and characterize the selected MOSFET at the actual gate voltage. If 8 V cannot meet conduction/switching requirements across tolerance and temperature, implement the data-sheet-supported alternate supply topology; do not quietly abandon 3S compatibility.

Use the ST reference topology for VM bypass, external buck inductor/Schottky/output capacitance, VCC, REGIN/REG3V3, VDD/VDDA/VREF and all local decoupling. Check sequencing and backfeed between traction-powered nodes, central 3.3 V, USB and every interface. The preferred production architecture does not keep a node MCU alive from USB after its VM branch is dead.

## 7. One programmable motor-control node, repeated four times

### 7.1 Device, firmware and command contract

Use one STSPIN32G4 (C7434864) per motor. Its embedded STM32G431 runs three-shunt sensorless FOC. Use the same reviewed binary on all four nodes; select wheel identity through documented passive straps or a provisioned, read-back identifier. Firmware version/hash, identity and calibration schema are verified and recorded through each node's SWD service port during provisioning, then rechecked by the ESP32 through the muxed external I2C2 diagnostic slave at every safe startup. The diagnostic readback is evidence of identity and health, not a safety-rated enable or shutdown mechanism.

Each ESP32 command pin carries one center-neutral, signed forward/reverse PWM request captured by a hardware timer on the local STM32. Exact frame rate, pulse/duty endpoints, neutral band, CRC-equivalent plausibility strategy and tolerances are release-gated. The interface must meet all of these rules:

- Both stuck-low and stuck-high are invalid, not maximum command.
- Missing frames, impossible period, impossible pulse width, excessive jitter or stale command request zero q-axis current and then gate-safe coast according to the local fault policy.
- Each node applies its own magnitude, slew, current, speed, temperature and bus-voltage limits; the ESP32 cannot command around them.
- Forward/reverse crossing requires zero command and the qualified stop sequence in Section 7.9.
- The ESP32 updates all four requests coherently on its 20 ms drive cadence. The node current loop remains local and much faster.

Motor-node firmware must not enable PWM until clocks, ADCs, current offsets, power rails, driver registers, `READY`, `nFAULT`, watchdogs, command input and all limit data pass self-test. Invalid flash/configuration or an unexpected firmware identity leaves the bridge disabled.

### 7.2 Three-shunt sensorless FOC

Use one Kelvin-sensed low-side shunt in each phase leg: three phase shunts per motor node. Route the three differential sense pairs directly into the STSPIN32G4/STM32G4 internal op-amp and ADC network shown by the ST reference design. Trigger both fast ADCs synchronously from TIM1 at qualified sampling points and use the internal comparators/TIM1 break path for hardware-fast phase overcurrent.

The 0.5 mOhm BVN shunt is a provisional starting candidate, not a frozen value. It produces 20 mV at 40 A, 30 mV at 60 A and 40 mV at 80 A. An individual shunt carrying 40 A RMS dissipates 0.8 W; at 80 A RMS it would dissipate 3.2 W. Select resistance, power rating, amplifier gain, blanking, RC filtering, common-mode behavior, offset budget and ADC range from the final definition of peak phase current. Calibrate all three offsets at every safe startup and cross-check their reconstructed-current sum.

Use sensorless FOC with a BEMF/flux observer appropriate to the measured 12N14P motor parameters. Standstill has no BEMF, so alignment or initial-position strategy, forced/open-loop acceleration and observer handoff are mandatory. Loaded hub-motor launch, crawl, hill hold, moving restart, wheel lift, stall and reversal are major validation gates. Do not claim accurate wheel torque until phase-current gain, motor parameters and q-axis torque mapping are calibrated.

### 7.3 Gate drive and bridge

Use one IPT015N10N5ATMA1 100 V MOSFET at each of six switch positions: six per node, 24 total. With 211 nC maximum gate charge, the conservative all-six average switched charge is:

`I_GATE(avg) = 6 x 211 nC x f_PWM`

- 15 kHz: approximately 19.0 mA.
- 20 kHz: approximately 25.3 mA.

The STSPIN32G4 gate driver has up to 1 A source and sink capability, but that peak rating does not establish allowable average regulator current, switching loss or package temperature. Derive the VCC-buck load from measured gate charge at the selected 8 V provisional gate rail and final PWM rate. Use double-pulse results to set source/sink resistance, dead time, dV/dt and any snubber.

The IPT015N10N5 data sheet guarantees 1.5 mOhm maximum at 10 V and 2.0 mOhm maximum at 6 V; 1.6 mOhm is only the typical 6 V value, and no maximum is guaranteed specifically at the provisional 8 V gate rail. Use the documented 2.0 mOhm maximum at 6 V as the conservative initial bound. One high-side plus one low-side device then gives an idealized 6.4 W at 40 A and 25.6 W at 80 A before hot RDS(on), switching, commutation, shunt and copper losses. Recalculate from measured behavior at the actual gate voltage and junction temperature. Give every FET a short gate path, resistor footprint, gate-source pull-down, Kelvin-source driver return and defined bottom-side thermal path. Provide DNP snubber footprints and populate them only from measured ringing.

### 7.4 STSPIN32G4 support and protection network

Follow the STSPIN32G4 data sheet and a matching ST evaluation/reference schematic for the external buck network, supply bypass, three bootstrap capacitors, gate resistors, pull-downs, SCREF network and MCU analog decoupling. Do not copy values from a different FET, gate rail or switching frequency without recalculation.

The driver monitors VDS across all six external MOSFETs and shuts all gate outputs off after the configured threshold/deglitch response. Set SCREF from worst-case hot MOSFET RDS(on), peak current, switching transient and tolerance. The available threshold and deglitch choices are secondary protection, not a substitute for shunt comparators or fuses. Validate false-trip immunity and real fault clearing at cold and hot.

Configure the internally connected `READY` and `nFAULT` signals on PE14/TIM1_BKIN2 and PE15/TIM1_BKIN as ST specifies. These are internal SiP connections, not package pins, so they cannot feed the board fault latch directly. The VDS/UVLO/thermal mechanisms and TIM1 break stop the affected bridge locally without waiting for an ESP32 or software handler. When a hard driver fault is observed, the node firmware stops servicing its external window watchdog; that watchdog then asserts the global fault latch within a measured bound.

### 7.5 SWD, provisioning and service access

Give each node its own labeled pads for SWDIO, SWCLK, NRST, GND and VTREF; optional SWO and a service UART may also have pads. Do not hard-parallel four SWD targets. Use four connectors, a qualified selector, or separate pogo access so one target cannot drive another.

Program and verify each node with the traction battery disconnected. Power only the target from a current-limited, defined service supply, keep the wheel lifted/restrained, hold gate outputs inhibited and prevent the debugger from backfeeding an unpowered rail. Production records must include firmware hash, option bytes/readout-protection policy, wheel identity, calibration schema and readback result for every node.

The central exposed pad is both the primary control-ground/thermal connection and a manufacturing risk. Use the ST land pattern as the starting point, then define paste-window coverage, solder-mask strategy and a symmetric ground/thermal via array that does not wick excessive solder. Require first-article X-ray or equivalent evidence for voiding, shorts and opens; ordinary visual inspection cannot see the central joint.

### 7.6 Branch current and bus-voltage supervision

In addition to the three phase shunts, each node has one high-side branch shunt and INA238 for battery-side current, bus voltage and power. This monitor is supervisory. It does not sample phase currents and cannot replace the comparator/TIM1-break or VDS shutdown paths.

A 0.5 mOhm high-side shunt produces 20 mV at 40 A and dissipates 0.8 W; at 80 A it produces 40 mV and dissipates 3.2 W. Confirm the exact INA238 input range/configuration, shunt tolerance, temperature coefficient, power rating, common-mode transient behavior, Kelvin routing and calibration. Use its measurements for operating envelopes, logging, plausibility checks and braking/reversal qualification.

### 7.7 Thermal acquisition

Fit two 10 kOhm NTCs per node: one at the MOSFET/heatsink hot spot and one at the branch-shunt/terminal hot spot. Each uses a 10 kOhm 1% bias resistor and local 100 nF filter footprint. Two ADS1115 ADCs provide eight single-ended channels on TCA9548A channels 4 and 5.

Poll at a bounded 10-20 Hz or faster validated rate. Convert with the exact populated NTC curve, bias tolerance, ADC gain and board calibration. Open, shorted, implausible or stale channels block initial permit and cause a controlled global inhibit while driving. These NTCs are supervisory; correlate thresholds with thermocouples during current soak and retain the STSPIN32G4's independent internal thermal shutdown.

### 7.8 DC link and branch protection

Planning baseline per node:

- Four fitted 470 uF/63 V low-ESR electrolytics and four optional footprints; 1.88 mF fitted is a starting population only.
- Local 1 uF/100 V and smaller 100 V ceramics in the minimum bridge loop.
- One SM8S30A across the local link, subject to measured clamp voltage and pulse-energy suitability.
- One provisional 60 A, 58 V branch fuse in the external PDU.
- Separate high-current branch positive/negative and phase U/V/W terminals.
- Two NTCs connected to the thermal-acquisition ADCs.

The TVS handles brief switching/harness overshoot, not sustained braking energy. A branch fuse protects wiring and fire risk; it is too slow to save MOSFETs from a switching fault. Verify fuse time-current behavior, ambient derating, interruption rating, pack fault current, cable ampacity and main-fuse coordination.

### 7.9 Integrated RPM output, braking and reverse

Each node computes signed mechanical speed from its qualified sensorless observer and exposes that signed value through diagnostics:

`mechanical_rpm = electrical_omega_rad_s x 60 / (2 x pi x 7)`

The one-wire FG signal conveys speed magnitude, not direction. The node synthesizes it from absolute accumulated valid electrical-angle travel in either direction, with exactly one rising edge per full electrical revolution and therefore seven rising edges per mechanical revolution for the 14-pole motor. Route each source through a central-logic-powered, partial-power-down-safe open-drain buffer with a defined input default and a 5.1 kOhm pull-up to clean central 3.3 V, then to ESP32 GPIO42/43/44/47. This prevents a central pull-up from back-powering an unpowered node.

FG transitions are allowed only after closed-loop observer handoff and while the observer is valid. Hold the buffered output static in its defined idle state during reset, alignment, forced/open-loop startup, stopped state, observer loss, node power loss and node fault. Preserve the ESP32 20 ms sampling cadence, adaptive approximately 100-500 ms window and 250 ms stale timeout unless measured evidence justifies a documented change. Seven PPR yields approximately 85.7 rpm quantization over 100 ms and 17.1 rpm over 500 ms.

No external phase RPM module, phase-sense connector or external RPM conditioning path is fitted. Validate every synthesized FG magnitude against the absolute value of the node's internal signed speed and against an independent optical tachometer through startup, handoff, steady speed, coast and braking; validate diagnostic direction separately in both directions. Because FG and the speed estimate come from the same observer, they are not independent evidence. Missing pulses are never, by themselves, proof of zero speed.

The node firmware implements controlled regenerative and/or dynamic braking through FOC. The reset/fault state is all gate outputs low and high-impedance coast. Do not use a low-side phase short as a default stop; qualify any such mode at restricted energy with measured current. Regeneration can raise the DC bus and is permitted only after full-6S tests prove pack/BMS charge acceptance and bus overshoot. Add a separately controlled, thermally sized brake chopper/resistor if the battery cannot accept the required stop energy.

Before changing direction, require zero command, near-zero phase current, near-zero INA238 branch current, a valid local stopped state, no recent qualified FG and a conservative coast timer of at least the existing 120 ms. The local node and ESP32 both enforce the transition. If sensorless evidence cannot qualify safe reversal, the next hardware revision requires true motor-integrated Hall/encoder feedback.

## 8. Hardware reset, watchdog and system shutdown

GPIO21 is the supervisor permit heartbeat/window-watchdog WDI, not a static enable level and not the sole safety control. The valid frequency, watchdog window, startup grace, polarity and timeout must be calculated from the final watchdog/latch schematic and are release-gated. The ESP32 emits it only after a proved-alive supervision iteration and suppresses it on supervisor failure, a designated hard inhibit or shutdown. It may remain active while the vehicle is disarmed so healthy nodes can stay reset-released for gate-disabled self-test and diagnostics; the heartbeat itself is not torque permission.

A central reset supervisor and reset/fault latch must default to asserting all four node `NRST` inputs low. Each reset release must genuinely fail low when central control power, release-logic power or node power is missing or invalid. Implement this with an independently powered suitable reset supervisor, a passive `NRST` pull-down plus qualified active release, or an equivalently proven topology. Use four isolated reset stages so node domains cannot backfeed each other. An `SN74LVC1G07` with `Ioff` can prevent backfeed, but its output becomes high impedance when its own supply is absent; it is not a fail-low reset solution by itself.

Each motor node also gets a dedicated external window watchdog. Its WDI pin is serviced only by a proved-alive iteration of that node's safety/control loop, never by a free-running timer. A missing, stuck or out-of-window heartbeat asserts that node's NRST and the common fault latch. The internal STM32 independent watchdog provides another local reset path. A detected internal `nFAULT`, `READY` loss, comparator/TIM1 break, current fault, invalid command or irrecoverable observer fault stops the heartbeat after the bridge has been driven safe, causing bounded all-wheel inhibition. This exported heartbeat consumes a node-local MCU GPIO, not an ESP32 pin.

`READY`, `nFAULT` and the gate-driver I2C bus are internal to each STSPIN32G4. They must not be drawn as external package pins or claimed as board-level wire-OR signals. The local gate driver itself forces its six outputs low for its documented VDS, thermal and supply faults; the external node watchdog propagates a persistent local failure to the global latch.

The final hardware must implement a reset-aware power-on/release sequence:

1. With the traction contactor open, let the central supervisor boot, hold all four command outputs in their invalid/off state, and keep the reset-release and torque-permit requests false. No motor node is expected to boot while its VM branch is de-energized.
2. After deliberate activation and the defined precharge sequence, energize all protected traction branches while the fail-low hardware continues to assert every node `NRST`. Closing the contactor or applying VM is not torque permission.
3. After node rails meet their validated thresholds and the central supervisor produces its valid GPIO21 heartbeat, the supervised reset-release circuit may deassert each `NRST` for gate-disabled boot and self-test only. Startup qualification in each per-node watchdog must allow this bounded boot interval, then require a valid node heartbeat; all exact timing remains schematic-dependent and release-gated. The GPIO21 heartbeat and reset release do not grant torque.
4. Require all four nodes to complete self-test, calibrate current offsets, report approved identity/configuration and produce valid heartbeats while torque permit remains false. The global fault latch must already be capable of reasserting all four resets.
5. Only after all hardware and software prerequisites pass may a deliberate vehicle arm transition grant the separate torque-permit state and allow node firmware to enter a torque-producing state. GPIO21 continues as supervision evidence but is not the arm command.
6. Any subsequent central or node watchdog fault latches all four resets. Clear only through an explicit disarmed recovery or power cycle.

The release circuit must not mask an active VDS, thermal or supply fault. Its timing and truth table require schematic review and injected-fault proof. NRST is a robust system inhibit only after bench proof that reset, brownout, malformed clocks and partial power always force the internally connected PWM signals and all external gates low. It is not certified safe torque off. The physical E-stop independently opens a DC-rated traction contactor.

I2C is not in the emergency stop path. Loss of INA238/ADS1115/mux diagnostics blocks or withdraws drive permit through the supervisor/watchdog policy, but the external latch, node watchdog, comparator break and VDS shutdown remain effective with the bus dead.

## 9. External power distribution

Do not carry the 160 A-class aggregate traction current through one ordinary PCB input trunk. Use an external star PDU or busbar:

| External item | Requirement |
|---|---|
| Battery/BMS | 3S-6S with documented discharge/fault current, per-cell cutoff and regenerative charge acceptance |
| Main fuse | Close to the battery and coordinated with four branch fuses |
| Contactor/manual disconnect | DC-rated for voltage, load and fault interruption |
| E-stop | Direct fail-safe contactor control, independent of software |
| Precharge/anti-spark | Sized for all four local DC links and reconnection interval |
| Four branch feeds | Individually fused, short paired conductors sized by temperature rise and voltage drop |
| Returns | Paired/star paths with a deliberately defined quiet logic reference |
| Cooling | Mechanically defined baseplate/heatsink, interface pressure, airflow and temperature sensors |

Never parallel legacy ESC BEC outputs. Steering-servo power remains external with a common signal reference.

## 10. Existing-function mapping

| Existing function | New VCU implementation |
|---|---|
| Receiver CH1/CH2/CH4/CH5 | Conditioned capture on ESP32-S3 |
| Steering relay/filter/trim/speed limit | ESP32-S3 plus 5 V AHCT output buffer |
| Four independent ESC commands | Four direct signed PWM torque requests to four STSPIN32G4 nodes |
| AWD/FWD/RWD | Zero signed request to the inactive axle; node gates remain controlled and safe |
| Four wheel speeds | Signed observer speed over diagnostics plus four onboard 7-PPR FG magnitude outputs; valid only in qualified closed-loop operation |
| IMU/current-boot bias | ASM330LHH SPI driver and preserved policy |
| Torque vectoring | ESP32 computes balanced per-wheel q-axis current/torque requests within node and system envelopes |
| Reverse/braking | Dual-layer zero-current/direction interlock plus qualified local FOC braking |
| GNSS | MAX-M10S UBX/NMEA over UART; no Dragy interface |
| Logging | Removable SPI microSD with card/write-fault handling |
| Serial/browser UI | Native USB and ESP32 SoftAP/web service |
| ESC endpoint calibration | Replaced by motor-parameter commissioning, phase-current offset/gain calibration, observer tuning and signed-command verification |

All current Hubba Hubba safety behaviors remain requirements unless this architecture explicitly strengthens them: valid receiver calibration, neutral/start qualification, deliberate arm transition, selected-wheel RPM validity before torque-vectoring activation, current-boot IMU calibration, equal-output fallback, 20 ms supervisor cadence, 250 ms RPM stale rule, and at least 120 ms zero-command direction hold. Basic drive may launch without prior FG because the observer cannot be qualified before rotation; torque vectoring may not.

## 11. PCB and mechanical constraints

- Start with at least six layers and 2-4 oz outer copper, four separated inverter zones and a quiet supervisor/RF zone.
- Use busbars or soldered copper reinforcement for high-current paths. Heavy copper alone is not a current rating.
- Put each node's ceramics directly across its bridge loop, with bulk and TVS immediately behind them.
- Minimize switch-node copper and separate it from command PWM, FG, SWD, NRST, I2C, USB, GNSS, IMU, receiver and current-sense nets.
- Kelvin-route all three phase shunts and the high-side branch shunt. Do not share noisy high-current vias with sense returns.
- Treat the VFQFPN exposed pad as the STSPIN32G4 ground/thermal reference; use the data-sheet land pattern and controlled via/paste process.
- Provide differential probe points for all three current senses, gate-source, switch node, bus, branch shunt, NRST, node-watchdog heartbeat/output, FG and supply rails. Inspect internal `nFAULT`/`READY` through debug firmware and SWD because they are not package pins.
- Keep every SWD pad out of switch-node electric fields and make production pogo access possible after assembly.
- Mechanically support electrolytics, terminals, busbars, fuses and heatsink against vehicle shock and vibration.
- Validate lug/screw torque, plating, land, solder volume, creepage, clearance and temperature rise, not just catalog current.

## 12. Mandatory release sequence

1. Freeze pack/BMS, cell-count workflow, vehicle mass/speed, cooling, current domains, braking energy and acceptable stopping behavior.
2. Measure motor phase resistance/inductance, KV/BEMF, no-load current, pole pairs, inertia, phase-versus-DC current and hub-load temperature.
3. Complete the STSPIN32G4 schematic against its data sheet and relevant ST reference boards. Independently review the internal pin reservations, 3S gate-rail solution, three-shunt analog front end, SCREF/VDS settings, reset latch and QFN land pattern; run ERC.
4. Freeze STM32Cube/MCSDK/toolchain versions, node pin mapping, command/FG protocol, motor parameters, protections, option bytes, firmware image hash and per-node provisioning record.
5. Assemble and X-ray or equivalently inspect one VFQFPN motor node before building four. Reject central-pad voiding/short/open criteria defined with the assembler.
6. Program one node through its own SWD pads using a current-limited service supply, traction isolated and gate outputs inhibited. Prove blank/invalid firmware cannot create torque.
7. Prove power-up, brownout, NRST, internal MCU watchdog, every external node watchdog, central watchdog, internal `READY`/`nFAULT`, neutral/stuck/missing command and no-torque-on-boot behavior before fitting a motor.
8. Double-pulse and scope gate voltage, dead time, dV/dt, Miller behavior, shoot-through, ringing, bus overshoot and switching loss at cold/hot and at the selected 8 V provisional gate rail.
9. Calibrate all three phase shunts and the INA238. Prove comparator/TIM1-break, current-loop ceiling, VDS shutdown and global latch with injected thresholds, stall, phase short/open, brownout and reset.
10. Tune and validate three-shunt sensorless FOC startup, minimum sustainable speed, observer handoff, moving restart, coast, stall, controlled braking and both direction transitions at 3S and 6S.
11. Verify synthesized 7-PPR FG magnitude against the absolute internal speed and an optical tachometer at multiple speeds and through open-loop start, handoff, coast, braking and faults; verify the separate signed diagnostic direction in forward and reverse. Establish the lowest trustworthy speed and reversal timeout.
12. Calibrate and fault-inject all eight NTC channels, then thermal-soak first at 20-25 A and progress toward 40 A. Attempt 70-80 A only with a defined short duration/duty while instrumenting every hotspot and the motor.
13. Test full-6S regenerative braking, BMS rejection, contactor/fuse behavior and harness inductance. Size and validate a chopper if required.
14. Prove all-wheel torque removal within measured bounds for ESP32 lockup, each command stuck low/high, node firmware lockup, each node heartbeat watchdog, the central watchdog, NRST, every internal `nFAULT`, dead central logic, dead node rail, I2C/mux failure and broken permit wiring.
15. Verify that each motor-node diagnostic address differs from the INA238 address on the same mux channel and that channel/identity readback detects swaps or collisions.
16. Qualify the ESP32-S3-WROOM-1U-N16R8 supervisor zone to its normal +65 °C ambient limit, or explicitly enable PSRAM ECC, account for the one-sixteenth memory reduction and prove the documented +85 °C operating configuration.
17. Build and provision all four nodes, then test them together for PDU drop/ripple, thermal coupling, cross-node fault containment/global shutdown, EMI, GNSS, IMU, receiver capture, Wi-Fi, USB and microSD.

No road test precedes these stages or the repository's restrained, wheels-lifted safety checks. Firmware-only, current-limited bench, one-node power, four-node power and moving-vehicle validation must remain separately reported.
