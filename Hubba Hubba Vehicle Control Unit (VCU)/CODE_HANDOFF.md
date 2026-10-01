# Hubba Hubba VCU Code Handoff

## Purpose and status

This is the maintained interface and behavior contract for the agents that will implement the Hubba Hubba VCU firmware. The hardware is a researched prototype architecture. It is not schematic-release ready, fabrication approved, physically tested, or qualified for a continuous 80 A motor current.

The VCU reference folder is documentation-only. Do not place source code, generated code, build files, binaries, linker scripts, board-support packages, test programs, or programming images in this folder. Put both firmware projects elsewhere in the repository and link back to this handoff.

Do not mechanically port the current ESP32-WROOM-32 GPIO, LEDC, PCNT, aircraft-ESC, or external-RPM-sensor implementation. Preserve the vehicle behavior identified below while replacing the hardware-specific layers.

## Exactly two firmware targets

The VCU requires two source/build targets:

1. **ESP32 supervisor firmware**
   - Target: ESP32-S3-WROOM-1U-N16R8 using ESP-IDF for target esp32s3.
   - One image owns receiver input, steering, vehicle state, per-wheel requests, torque vectoring, GNSS, IMU, storage, user interfaces, and system supervision.
2. **Common motor-node firmware**
   - Target: the embedded STM32G431 in STSPIN32G4.
   - Build one common image and install that same approved image in all four STSPIN32G4 QFN motor nodes: front-left, front-right, rear-left, and rear-right.
   - Do not create four drifting wheel-specific forks. Physical port assignment, MCU unique ID, a reviewed node-identity record, and configuration data distinguish instances. Wheel direction or wiring corrections must not require a different executable.

The two targets have separate build systems, unit tests, release artifacts, version numbers, and programming procedures. A vehicle release is one compatibility-tested bundle containing one supervisor image, one common motor-node image, and the matching parameter/protocol manifest.

There is no CAN bus and no TWAI dependency. Real-time wheel commands use four direct signed-PWM links. Four synthesized FG links return speed pulses. A muxed I2C service bus carries bounded diagnostics and commissioning data only; it is not the torque-command path and is not required for immediate torque removal. Each node's external diagnostic slave uses the exposed STM32 `PA8/I2C2_SDA` and `PA9/I2C2_SCL` pins. This is separate from the STSPIN32G4 gate driver's internal, non-package I2C3 connection on `PC8/PC9`.

## Functional partition

### ESP32 supervisor owns

- Receiver CH1/CH2/CH4/CH5 capture, validation, calibration, failsafe detection, and arming policy.
- Steering median filtering, smoothing, trim, speed-sensitive limit, and buffered servo output.
- Global vehicle state, permanent-arm-latch option, drivetrain selection, drive smoothing, and the system direction-change interlock.
- Four signed wheel requests and all torque-vectoring calculations.
- Four hardware-timed signed-PWM command outputs and four FG capture inputs.
- Version/compatibility checks and bounded node health reads through TCA9548A.
- Four INA238 branch current/voltage/power monitors and all eight NTC channels through two ADS1115 devices.
- ASM330LHH, MAX-M10S, microSD, persistent configuration, native-USB CLI, and the WPA2 SoftAP dashboard/API.
- The GPIO21 windowed supervisor permit heartbeat/WDI. The ESP32 produces it only from a proved-alive control iteration; it may run while disarmed to support reset-released self-test and diagnostics, but it is not torque permission and cannot defeat a latched hardware fault.

### Each STSPIN32G4 motor node owns

- Three-shunt phase-current acquisition, offset calibration, plausibility checking, and saturation detection.
- High-rate sensorless field-oriented control, including Clarke/Park transforms, current regulators, modulation, ADC synchronization, dead-time handling, and signed torque-producing current control.
- Sensorless alignment/open-loop launch, observer acquisition, closed-loop handoff, moving-start policy, loss-of-lock handling, and local current/speed/thermal envelopes.
- Six external MOSFET gate commands and the immediate local response to gate-driver, timer-break, overcurrent, brownout, deadline, command-timeout, and watchdog faults.
- Signed-PWM input capture, validation, deadband mapping, slew limiting, and conversion to a bounded local q-axis current request.
- Signed observer-derived mechanical-speed estimation for diagnostics and generation of one qualified 7-PPR FG magnitude output.
- A versioned, CRC-protected diagnostic snapshot exposed on its isolated I2C segment.
- Its own internal watchdog, independent reset state, fault output, and heartbeat used by its independent hardware supervision path.

The gate-driver `READY` (`PE14`), `nFAULT` (`PE15`), and I2C3 (`PC8/PC9`) signals are internal SiP connections. Motor-node firmware must use them locally, including TIM1 break handling, but no schematic or firmware may treat them as external package pins or direct ESP32 signals.

The initial control contract is sensorless three-shunt FOC with q-axis current as the local torque-producing variable and a default d-axis request of zero. Field weakening, maximum-torque-per-amp extensions, high-frequency injection, and automatic regeneration are out of scope until individually designed and bench-qualified. Never describe branch battery current as phase current or q-axis current.

### Independent hardware owns

- Passive fail-low power-stage inhibition during reset, boot, brownout, missing logic power, and unprogrammed GPIO states.
- A separate watchdog/heartbeat path for each motor node. Failure of one node heartbeat must inhibit that node without waiting for the ESP32 or I2C.
- A separate reset net for each motor node. Do not tie the four reset nets together. Each reset net needs a defined passive state and access at that node's programming pads.
- A common fault latch triggered by any motor-node fault, any per-node watchdog expiry, the supervisor watchdog, or another designated hard fault. The latch fans out through four partial-power-down-safe open-drain stages that assert the four separate node `NRST` nets; it must never electrically join the powered node domains.
- A supervised reset-release path that allows an energized node to boot into gate-disabled self-test without granting torque permission. Each path must remain asserted when its controlling supply is absent, using an independently powered suitable reset supervisor, a passive `NRST` pull-down with qualified active release, or an equivalently proven circuit. `SN74LVC1G07` `Ioff` is anti-backfeed behavior only; an unpowered device has a high-impedance output and is not fail-low by itself.
- Branch and main fusing, contactor/E-stop, precharge, DC-link protection, and the analog fast-overcurrent path.

The global latch clears only through a deliberate, disarmed recovery with neutral commands, acceptable current and bus state, valid versions, and all fault sources released. Resetting the ESP32 or any one of the four STM32 motor-node instances must never re-enable drive automatically. A hard motor-node fault normally stops all four wheels.

## Provisional ESP32-S3 pin contract

| Function | GPIO |
|---|---:|
| MAX-M10S UART RX/TX | 1 / 2 |
| Receiver CH1/CH2/CH4/CH5 | 4 / 5 / 6 / 7 |
| Steering PWM | 8 |
| Shared SPI SCLK/MOSI/MISO | 9 / 10 / 11 |
| IMU CS/INT | 12 / 13 |
| microSD CS/detect | 14 / 15 |
| MAX-M10S TIMEPULSE | 16 |
| Internal service I2C SDA/SCL | 17 / 18 |
| Native USB D−/D+ | 19 / 20 |
| Supervisor permit heartbeat / watchdog WDI | 21 |
| Signed command PWM FL/FR/RL/RR | 38 / 39 / 40 / 41 |
| Qualified FG FL/FR/RL/RR | 42 / 43 / 44 / 47 |
| TCA9548A active-low reset | 48 |

GPIO0 is BOOT only. GPIO3, GPIO45, and GPIO46 remain unloaded straps. GPIO35–37 are unavailable on the N16R8 module. Treat this map as provisional until the final schematic, boot-state review, and an ESP-IDF target build confirm every assignment. Individual motor-node reset lines, fault-latch status, and any retained non-critical I/O expander allocation must be frozen in the schematic before code freeze; do not infer missing nets from this table.

GPIO21 carries a windowed permit heartbeat, not a static enable level. Its valid frequency, watchdog window, startup grace, polarity and timeout are generated from the final watchdog/latch schematic and remain release-gated. Service it only after a proved-alive supervisor iteration and suppress it on supervisor failure, a designated hard inhibit or shutdown. It may remain active while disarmed so healthy nodes stay reset-released for gate-disabled self-test and diagnostics; do not treat it as the torque-arm command.

## Direct signed-PWM command contract

Each wheel has a dedicated PWM wire from the ESP32 to the corresponding STM32 input-capture timer. This command represents a **signed normalized torque/current request**, not inverter duty cycle and not measured torque.

- Nominal 50% duty represents zero request.
- A validated band above neutral maps monotonically to forward positive q-axis current.
- A validated band below neutral maps monotonically to reverse negative q-axis current.
- Narrow neutral hysteresis prevents sign chatter.
- Guard regions at both rails are invalid/off. Continuous low, continuous high, missing edges, out-of-range frequency, malformed periods, excessive jitter, non-finite conversion data, or an expired command age produces zero request and a local fault/inhibit response.
- Exact carrier frequency, valid duty bands, neutral width, filtering, polarity, input timer clock, timeout, and unit scaling are protocol fields that remain release-gated. Both firmware targets must compile from the same reviewed values.
- The ESP32 updates all four requests coherently on the 20 ms vehicle-control cadence. Each node captures every carrier period and runs its faster local current loop independently between updates.
- The node applies its own phase-current, bus-voltage, temperature, acceleration, and observer-state limits even when the received command is otherwise valid.
- An inactive drivetrain axle receives a zero request and remains inhibited unless a separately qualified zero-torque observer state is deliberately approved.

A frozen valid nonzero PWM duty is a credible failure. The command-age check alone cannot detect it. The supervisor window watchdog, per-node heartbeat watchdogs, and global hardware latch must remove gate permission when the ESP32 or any one of the four STM32 processors stops making progress.

## Motor-node FOC and startup contract

The common STM32 image must use all three Kelvin-routed low-side phase-current channels provided by the final schematic. Current samples must be synchronized to the inverter PWM and checked for offset, gain, clipping, impossible sum, timing-window loss, and ADC/DMA overrun.

Required local states are at least:

- reset/inhibited;
- self-test and current-offset calibration;
- ready with gates disabled;
- rotor alignment or other approved open-loop acquisition;
- open-loop acceleration;
- observer qualification;
- sensorless closed-loop FOC;
- controlled deceleration;
- coast/zero-current wait;
- fault latched.

At boot, calibrate current offsets only while the bridge is inhibited and prove the samples are within a released band. Do not enter a switching state with stale or implausible calibration. The fast analog overcurrent/timer-break route must act without waiting for the control ISR.

The first prototype current controller must:

- use explicit ampere units for phase current and q-axis request;
- clamp d/q magnitude before modulation;
- include integrator anti-windup;
- handle voltage saturation and minimum current-sampling windows;
- fail safe on missed control deadlines or corrupt DMA sequencing;
- limit startup current independently of the ESP32 request;
- disable or tightly gate any feature that can return energy to the DC link;
- publish requested, limited, and measured currents separately.

Motor resistance, inductance, flux linkage/back-EMF constant, pole pairs, current-sense gain, dead time, PWM frequency, observer gains, startup ramp, lock criteria, and current limits are released parameters. Do not copy demonstration-project defaults and do not silently identify them at normal boot.

## Sensorless limitations

The STSPIN32G4 does not create an independent shaft sensor. Rotor angle and speed are estimates derived from phase measurements and the motor model.

- At standstill and very low electrical speed, back-EMF information is weak or absent.
- Alignment and open-loop launch can rotate the motor before the observer is trustworthy.
- Heavy load, abrupt reversal, current saturation, coast, aggressive deceleration, wiring errors, or poor motor parameters can cause loss of lock.
- An observer that is unlocked, stale, numerically implausible, or disagreeing with current/state checks is not a valid speed source.
- Missing FG edges never proves that the wheel is stopped.
- Basic drive may arm before any wheel has produced FG pulses, because requiring closed-loop speed before launch would deadlock startup.
- Torque vectoring requires every selected driven wheel to be in a qualified observer state with current, plausible FG. Speed-sensitive steering must use its documented conservative fallback if the needed rear-wheel speed is unavailable; it must not energize an inactive axle solely to obtain speed.

Loaded hub-motor launch, crawl, hill start, moving restart, minimum sustainable speed, observer reacquisition, and both direction transitions must be validated at 3S and 6S. If reliable low-speed or zero-speed knowledge is required beyond what the observer can prove, the hardware needs real Hall or encoder feedback in a later revision.

## Integrated 7-PPR FG contract

Each motor node synthesizes a local 3.3 V-compatible FG signal from **validated observer electrical angle**, not from requested PWM, commutation commands, or elapsed time. The internal estimate and diagnostic field are signed; the single pulse wire carries speed magnitude only and does not encode direction.

The MCU source passes through the hardware-defined, central-powered, partial-power-down-safe open-drain FG buffer before reaching the ESP32. Firmware must use the released source polarity and idle level; it must not assume that a static electrical level distinguishes zero speed from reset, node power loss, observer invalidity, or a broken wire.

- The 14-pole motor has seven pole pairs. One qualified rising edge per observer electrical revolution therefore yields exactly seven rising edges per mechanical revolution.
- Generate a bounded-width pulse or 50% phase signal from absolute accumulated electrical-angle travel. Count a full revolution in either direction and use an unwrapped phase accumulator or equivalent logic that cannot miss reverse travel or double-count a noisy wrap.
- Hold the output in its defined idle state during reset, open-loop startup, observer qualification, loss of lock, invalid angle/speed, node fault, and any state for which the speed estimate has not been validated.
- On requalification, align the pulse generator before enabling output so state transitions cannot create a false edge.
- Publish a separate observer-valid/FG-valid bit, sequence counter, electrical speed, mechanical speed, and reason for invalidation through diagnostics. The pulse wire alone is insufficient to establish validity.
- Use the same explicit conversion on the ESP32:

  mechanical rpm = rising-edge frequency in hertz × 60 / 7

- Preserve the 20 ms sampling cadence, adaptive rolling window, and 250 ms stale timeout unless bench evidence supports a documented change. At 7 PPR, count resolution is about 85.7 rpm over 100 ms and 17.1 rpm over 500 ms.
- Compare every FG magnitude against the absolute internal speed and an optical tachometer through launch, observer handoff, steady operation, coast, deceleration, and faults. Validate the separate signed diagnostic direction in forward and reverse. Record the lowest trustworthy speed and false/missed-edge rate.

There are no external HW86060041 modules, phase-tap RPM circuits, source-selection links, or alternate runtime speed inputs in this VCU revision.

## Muxed I2C diagnostics

TCA9548A isolates repeated addresses across branches and bounds a failed branch:

- Channels 0–3 each contain one motor-node service endpoint and that branch's INA238 monitor. Those two devices on the same channel must have different frozen addresses and both identities must be read back; a node address may repeat only on another isolated mux channel.
- Channels 4–5 contain the two ADS1115 devices used for eight NTC channels.
- Channels 6–7 remain reserved, deselected by default, and are not routed off-board.
- GPIO48 directly resets the mux. Start with all downstream channels deselected and select only one channel at a time.

Each node exposes a coherent, CRC-protected diagnostic snapshot containing at least:

- protocol and firmware versions, build ID, feature bitmap, MCU unique ID, hardware revision, and parameter digest;
- local state, reset cause, uptime, control-loop counter, watchdog status, and fault history;
- captured command duty/frequency/age and decoded signed request;
- requested/limited/measured d/q current and three raw/offset-corrected phase currents;
- bus voltage, observer electrical angle/speed, mechanical rpm, observer confidence/state, and FG-valid state;
- gate-driver and analog protection status, temperature inputs available to the node, and saturation/overrun counters.

Runtime reads are bounded, low priority, and read-only from the dashboard. Never use I2C to stream the real-time command. Never expose arbitrary node memory or peripheral writes over Wi-Fi. Any service write requires DISARMED, recognized CH4 STOP, global power-stage inhibit, explicit local confirmation, a version check, and readback. Programming or recovery requires traction isolation.

On a timeout or stuck-low bus, remove global permit, reset the mux, reinitialize with every channel deselected, and revalidate all required devices. Recovery never restores torque automatically.

Any retained GPIO expander is limited to non-critical functions such as microSD power/status or indicators. It must not own a motor command, gate permission, reset safety response, fault latch, receiver capture, steering timing, or FG capture.

## Programming, SWD, and common-image rules

Provide separate programming pads for every motor node:

- SWDIO;
- SWCLK;
- node-specific reset;
- target reference voltage;
- ground;
- optional UART/service pin only if the schematic explicitly allocates it.

Pads must be reachable after assembly and clearly labeled FL, FR, RL, and RR. They must not expose phase or traction voltage to the programmer. Program and recover nodes with traction power isolated and the inverter hardware inhibited. Boot-strap pulls and debug-pin states must keep all gates off before, during, and after a programming session.

The release process builds the common motor-node image once, records its cryptographic hash/build ID, flashes that artifact to all four nodes, reads back identity/version, and verifies the four hashes match. Calibration records may contain node-specific measured offsets or identifiers, but they must use one common schema and must not fork the executable.

## Version and compatibility contract

Define a versioned supervisor-to-node contract before implementation. At minimum it contains:

- protocol magic and major/minor version;
- signed-PWM carrier, polarity, duty mapping, deadband, timeout, and normalized-unit definition;
- diagnostic register-map/schema version and byte order;
- FOC feature bitmap and safety-feature bitmap;
- motor pole-pair count and FG PPR;
- motor/control parameter schema and digest;
- hardware-revision compatibility range;
- motor-node semantic version, build ID, and image hash;
- supervisor semantic version, build ID, and approved-node compatibility table.

At startup the ESP32 queries all four nodes through their physical mux channels and verifies that:

- all four run the approved common-image major version and compatible minor version;
- all four report the required safety features and identical command scaling;
- each physical wheel channel maps to the recorded MCU unique ID;
- hardware revision and parameter digests are approved;
- pole pairs and synthesized FG are both configured as seven;
- no node reports an unsupported or partially migrated state.

Unknown major versions, undeclared feature combinations, mixed images, wrong wheel identity, inconsistent units, corrupt data, or a missing node inhibits the entire drivetrain. Minor-version compatibility is allowed only when explicitly listed and tested in the compatibility matrix; never assume it.

Firmware update is maintenance-only: DISARMED, CH4 STOP, contactor open or traction otherwise physically isolated, all gates inhibited, one node at a time, verified artifact hash, readback, reboot, self-test, and final four-node compatibility check. Do not implement an over-the-air motor-node update for the first prototype.

## Startup and shutdown contract

Power-up must be torque-free before application code executes:

1. With the traction contactor open, the separately fused upstream logic feed powers the ESP32, which initializes brownout handling, native USB/console, watchdog logic and safe command generation while initially keeping GPIO21 permit heartbeat suppressed. All four command lines remain in their invalid/off state. Motor nodes are not expected to boot while their VM branches are de-energized.
2. After deliberate activation and the released precharge sequence, the hardware energizes the protected traction branches while passive fail-low circuitry continues to assert every node `NRST`. Closing the contactor or applying VM is not torque permission.
3. Once the node rails satisfy the released thresholds and the proved-alive ESP32 begins the valid GPIO21 heartbeat, supervised reset release may deassert each `NRST` solely for gate-disabled boot and self-test. The per-node watchdog startup-grace behavior must allow this bounded boot interval, then require a valid heartbeat; all timing is schematic-dependent and must not be invented in firmware. Neither GPIO21 nor reset release grants torque.
4. Each STM32 starts independently, configures clock/brownout/watchdogs, keeps outputs inactive, validates program/config integrity, initializes the gate driver, calibrates all three current channels, and publishes a distinct `node_ready` diagnostic status only after self-test. This must not be confused with the internally connected gate-driver `READY` signal on PE14.
5. Each node starts its internal watchdog and its independent heartbeat. A missed FOC deadline, invalid command, failed calibration, observer fault outside an approved transition, gate-driver fault, or analog protection trip removes local gate permission and asserts its fault output.
6. The ESP32 asserts GPIO48 reset for the required interval, releases it, confirms all mux channels are deselected, and then identifies one downstream segment at a time.
7. Verify all four node identities/versions/configuration digests, all four INA238 devices, both ADS1115 devices, and all eight NTC channels. Reject a node/INA address collision on any channel and reject missing, duplicate, stale, implausible or wrong-revision devices.
8. Load persistent data through a versioned migration path. Require explicit battery cell count 3, 4, 5, or 6 and compare total pack voltage with conservative limits for that selection. Never infer a different count automatically.
9. Initialize receiver, steering, FG capture, IMU, GNSS, microSD, SoftAP/API, and telemetry without allowing optional services to block a drive deadline.
10. Run the five-second stationary IMU yaw-bias calibration before starting normal drive control.
11. Confirm all four signed-PWM decoders see neutral/valid timing, all node resets and watchdogs are healthy, FG lines are electrically idle without false edges, observer validity is false while stationary, branch current is near zero, all hard faults are clear, and the global latch can reassert all four resets. Torque permit remains false throughout reset release and self-test.
12. Only after a deliberate valid arm sequence and all prerequisites pass may the separate torque-permit state allow node firmware to enter alignment/startup. GPIO21 continues as supervision evidence; neither it nor reset release grants torque.

An ESP32 reset, any node reset, watchdog expiry, brownout, incompatible version, mux/bus fault, current/thermal sensor fault, overcurrent/overvoltage, observer/control failure, gate-driver fault, or global-latch fault removes drive permission. The normal uncontrolled-fault response is immediate gate inhibition and high-impedance coast, backed by independent traction isolation. This design is not certified safe torque off.

## Reverse and braking safeguards

Direction reversal is a coordinated state transition, never just a sign-bit change. Preserve 120 ms as the absolute minimum zero-command hold, but use a longer bench-qualified value whenever required.

Before either the ESP32 or a motor node accepts the opposite sign:

- the supervisor request has ramped to zero;
- the node's requested and limited q-axis current are zero;
- measured phase current and INA238 branch current are within released zero-current bands;
- observer speed is below a validated threshold while still valid;
- no recent qualified FG edges exist;
- the node is not aligning, launching, reacquiring, actively decelerating, saturated, or faulted;
- the conservative coast timer has completed.

If the observer becomes invalid, absence of FG is not proof of zero. Keep the inverter inhibited and require the conservative worst-case coast timeout or a stronger physical stop indication. A disagreement or timeout keeps the wheel at zero and reports a fault.

Service deceleration may use controlled negative q-axis current only within proven phase-current, bus-voltage, MOSFET, thermal, and battery charge-acceptance limits. Low-side phase shorting, active spin-down, regenerative energy return, and field-weakening deceleration remain disabled by default until each mode passes restricted-energy testing. At full 6S charge, a battery/BMS that cannot accept returned energy must force coast or use a separately sized dump path. A TVS and local DC-link capacitors are transient controls, not sustained braking-energy sinks.

Emergency and hard-fault behavior is gate inhibition/high-impedance coast plus the external contactor/E-stop. Software-controlled deceleration is never the sole emergency stop.

## 3S–6S battery and current behavior

- Persist battery_cells with allowed values 3, 4, 5, or 6. The default is unset.
- Allow cell-count changes only while DISARMED, CH4 is in a recognized STOP position, and traction is inhibited.
- Treat 25.2 V as the maximum normal full-charge voltage for 6S. Treat 9.0 V only as a provisional cold/load 3S logic-power test point until the actual cells and BMS are selected.
- A separate cell-aware BMS or low-voltage monitor is mandatory. Total pack voltage and device undervoltage lockout do not protect individual LiPo cells.
- Reject pack voltage inconsistent with the selected count using conservative BMS-derived limits; never guess or switch cell count from voltage alone.
- Derate current and torque by validated pack voltage, cell temperature, branch temperature, motor temperature assumptions, and BMS capability.
- Disable energy-return deceleration until charge acceptance and full-charge bus overshoot are measured.

The planning target remains 40 A battery-side average per motor branch, subject to duty-cycle and thermal validation. A provisional 60 A fast bridge/phase-current ceiling and a 70–80 A brief fault/transient region may be used only as starting hypotheses for protection analysis. They are not continuous ratings and must be replaced by the final three-shunt gain/tolerance analysis, MOSFET safe-operating data, motor limits, PCB/busbar thermal results, and fault-injection evidence. Battery current, DC-link current, phase RMS current, phase peak current, and q-axis current must be logged and specified separately.

Hobbywing's published 40.9 A for 46 seconds and recommendation of an 80 A commercial controller support substantial headroom, not an 80 A continuous custom-board claim.

## Existing vehicle behavior to preserve

### Receiver and arming

- CH1 steering, CH2 centered throttle, CH4 three-position RUN/STOP, and CH5 OFF/STRAIGHT/FULL torque-vectoring mode.
- Accepted receiver pulse range 800–2200 microseconds and 100 ms signal-loss timeout.
- Default throttle calibration: 978/1514/2044 microseconds for full forward/neutral/full reverse.
- Default steering calibration: 2050/1518/987 microseconds for full left/center/full right.
- Default CH5 calibration: 2047/1513/981 microseconds for OFF/STRAIGHT/FULL.
- Default CH4 calibration: RUN 983 microseconds and STOP positions 2049/1515 microseconds.
- Fixed transmitter-off throttle detector enabled by default at 1565 ± 10 microseconds.
- Initial arming requires saved throttle and CH4 calibration, valid CH2/CH4, neutral throttle, a healthy STOP observation since boot, and 250 ms of continuous valid RUN plus neutral.
- Preserve the persistent arm_latch option and its current enabled default. With latching enabled, recognized receiver STOP or a recoverable receiver fault temporarily inhibits output while the logical arm state remains latched. With it disabled, the same event fully disarms and requires a fresh STOP-to-RUN transition.
- Hardware, motor-node, power, version, thermal, or global-latch faults always require deliberate disarmed recovery regardless of arm_latch.
- Preserve CLI disarm and maintenance disarm config semantics. Remote arm remains prohibited.

### Steering

- Three-distinct-frame median filtering, isolated-spike count, startup warmup, and immediate centering on missing CH1 or confirmed failsafe.
- Configurable first-order command smoothing from 0–500 ms, default 10 ms, plus stored trim over ±15 degrees.
- Corrected 45-point left/right road-wheel table and inverse speed-limit lookup.
- Speed-sensitive steering based on valid rear-wheel speed, 107 mm wheel diameter, 445 mm wheelbase, stored 320 mm track reference, and configurable 1.0 g default lateral-acceleration ceiling.
- A temporary rear-speed dropout while armed may retain the last valid speed only for the existing bounded policy. Stale or unqualified observer/FG data must ultimately select the conservative steering limit.

### Drive

- 20 ms supervisor update interval.
- Persistent AWD, FWD, and RWD selection. Inactive wheels receive zero signed request.
- Centered receiver throttle, neutral deadband, quadratic magnitude response, and configured forward/reverse limits.
- Persistent 0–100% drive smoothing. At 100%, preserve the timing represented by the existing 12-microsecond acceleration and 100-microsecond deceleration steps per 20 ms update. Translate it into normalized signed-current-command increments; do not copy old pulse endpoints.
- Zero smoothing may remove the normal ramp but never removes the direction-change state machine or 120 ms minimum hold.
- Missed drive or sensor deadlines force at least one safe update and increment exposed overrun counters.
- A local node limit always has authority to reduce or reject the supervisor request.

### IMU and torque vectoring

- ASM330LHH over SPI at a validated clock, configured for 208 Hz, ±16 g accelerometer, and ±1000 degrees/second gyroscope.
- Approximately 5 ms acquisition cadence and 100 ms stale timeout.
- Five-second stationary boot yaw-bias calibration plus a guarded two-second retry. Torque vectoring requires a successful current-boot bias.
- Preserve configurable yaw sign, with the current default of −1 pending physical orientation validation.
- CH5 OFF, STRAIGHT, and FULL modes; saved CH5/steering calibration; live qualified selected-wheel speed; and valid IMU prerequisites.
- STRAIGHT corrects yaw and side-speed mismatch near steering center. FULL retains empirical steering-based yaw and side-speed targets plus straight correction.
- Reverse always uses equal left/right requests. Invalid or stale torque-vectoring inputs fall back to equal requests on the selected driven axle, never stale correction. Inactive wheels remain at zero.
- Preserve balanced left/right correction, anti-windup, ±25% initial authority limit, 180 degrees/second turn-yaw gain, 0.20 turn side-speed gain, yaw proportional/integral starting values 0.00025/0.00004, side-speed proportional starting value 0.20, and up to 20% front relief in AWD/FWD. RWD forces front relief to zero.
- These gains are migration starting points only. Revalidate every value because local phase-current control is materially different from the former aircraft-controller pulse-to-thrust response.

### GNSS, logging, and interfaces

- MAX-M10S UBX/NMEA over UART is the sole GNSS source.
- Capture TIMEPULSE on GPIO16. Keep the pin high impedance with no pull during GNSS startup because it is shared internally with SAFEBOOT_N.
- Do not port a Dragy UART/I2C driver, parser, configuration path, monitor command, power control, connector assumption, or Dragy-specific telemetry field. Use generic GNSS naming.
- Native USB replaces the development-board UART bridge for flashing and CLI.
- Keep the ESP32-hosted WPA2 SoftAP and guarded HTTP dashboard/API.
- microSD replaces the internal FAT logging partition. Use bounded queues and a dedicated low-priority writer; never block receiver, FOC supervision, steering, or drive timing, and never autoformat.
- Servo power remains external. The VCU provides only the protected/buffered command and common signal reference.

## Calibration and service UI

Retain guarded receiver, steering, CH4, CH5, and IMU calibration workflows. Replace legacy motor endpoint calibration with a motor-node commissioning workflow that reports, per physical wheel:

- MCU unique ID, hardware revision, node firmware version/build/hash, protocol version, and compatibility result;
- common parameter schema/digest and any node-specific measured calibration record;
- three current-channel raw values, offsets, gains, saturation margins, and sum plausibility;
- motor phase order/direction, resistance, inductance, flux/back-EMF data, pole pairs, and approved source of each parameter;
- current-loop rate/bandwidth evidence, PWM/dead-time configuration, fast protection thresholds, and gate-driver fault state;
- launch ramp, current ceiling, observer-lock/loss criteria, minimum qualified speed, and moving-start policy;
- signed-PWM decoded duty/frequency/age, neutral/rail fault checks, and watchdog behavior;
- observer angle/speed and synthesized 7-PPR FG validation against an optical tachometer;
- INA238 zero/gain checks, thermal-channel plausibility, and restricted low-energy spin/identify results.

Do not expose arbitrary memory, register, gate-control, current-command, reset, or programming operations through the browser. Developer service operations require a local interface, explicit confirmation, traction isolation, global gate inhibition, and an audit log.

## Scheduling and concurrency

### STM32 motor-node priority

1. Hardware timer break and gate-driver fault response.
2. PWM/ADC/DMA FOC interrupt and control-deadline supervision.
3. Observer, launch/reacquisition, and current/voltage/thermal limit state machines.
4. Signed-PWM input capture, command watchdog, per-node heartbeat, and qualified FG generation.
5. Coherent diagnostic snapshot and bounded I2C service.

No diagnostic or service transaction may delay the FOC ISR. The internal watchdog is serviced only after verified progress of the safety-critical control path, not from an unrelated idle task.

### ESP32 supervisor priority

1. Receiver capture and hardware-fault response.
2. 20 ms drive/steering update, coherent four-wheel command update, and supervisor watchdog service.
3. 5 ms IMU acquisition.
4. FG capture/speed estimation and bounded current/node/thermal health reads.
5. GNSS, USB, Wi-Fi/API, UI, and microSD logging.

No microSD, Wi-Fi, USB, GNSS, or I2C activity may block receiver, steering, command, watchdog, or fault deadlines. Every transaction has a timeout and stale-data policy.

## Required verification

### Build and protocol evidence

- Reproducible ESP32 supervisor build for esp32s3 with unit/host tests for parsers, state machines, persistence migration, signed-PWM encoding, arming, direction interlock, speed conversion, torque vectoring, and timeouts.
- Reproducible common STM32G431 node build with unit/target tests for three-shunt transforms, regulators, saturation, observer state, signed-PWM decoding, FG synthesis, watchdogs, reset causes, diagnostic CRC/snapshots, and every fault transition.
- Compatibility tests covering approved and rejected major/minor versions, mixed four-node images, wrong unit scaling, wrong feature bits, corrupt parameter digest, swapped node identity, and partial updates.
- Release manifest proving the same motor-node image hash is installed on all four nodes.

### Bench evidence

- No-torque boot and reset tests for the ESP32 and each motor node independently.
- Fault-inject continuous-low, continuous-high, frozen-valid, malformed-frequency, jittered, missing, and mid-neutral command inputs.
- Measure each local watchdog, reset, gate-inhibit, node fault output, supervisor watchdog, and global-latch response time. Prove any one hard fault disables all four inverters and cannot self-rearm.
- Validate the three current channels per node for offset, gain, bandwidth, clipping, ADC timing, phase-sum plausibility, shunt heating, and fast-trip behavior.
- Scope gate voltage, dead time, Miller behavior, shoot-through, ringing, bus overshoot, and timer-break response before high-current operation.
- Test current limiting, stall, lock loss, phase open/short, moving restart, control-deadline miss, brownout, and thermal faults.
- Compare all four synthesized 7-PPR FG magnitudes with absolute internal speed and an optical tachometer through open-loop launch, observer acquisition, steady speed, coast, controlled deceleration, and injected observer loss; verify signed diagnostics separately in forward and reverse.
- Prove the full zero-command/current/speed/coast reversal sequence and the never-shorter-than-120-ms hold in both directions.
- Test any enabled service-deceleration mode for phase current, DC-link rise, BMS rejection, and MOSFET/motor temperature. Keep energy return disabled until this passes at full-charge 6S.
- Test receiver latency and control deadlines while Wi-Fi, microSD, GNSS, USB, all node diagnostics, and thermal/current polling are active.
- Calibrate and fault-inject all eight NTC channels and four INA238 branches.
- Validate cold/load 3S and full-charge 6S boundaries, with intermediate pack counts spot-checked.
- Progress from one inverter on a current-limited source to four-channel aggregate power, EMI, PDU-drop, thermal-coupling, GNSS, IMU, and RF testing.
- Hold the ESP32-S3-WROOM-1U-N16R8 supervisor zone within its normal +65 °C ambient limit, or explicitly enable PSRAM ECC, budget the one-sixteenth capacity reduction and qualify the documented +85 °C configuration.

Always distinguish unit/build/static evidence from physical bench evidence. No road test precedes restrained, wheels-lifted, restricted-current validation and an independent power-electronics safety review.

## Inputs still blocking code freeze

- Final schematics, exact net names, per-node reset/watchdog/heartbeat wiring, global-fault-latch truth table, gate-enable polarity, and ESP32 pin confirmation.
- Exact STSPIN32G4 ordering/package revision, gate-driver supply implementation, external MOSFET/gate network, and one-node power-stage review.
- SWD pad layout, programmer voltage/isolation procedure, boot configuration, production flashing flow, and node-identity record.
- Frozen signed-PWM frequency/duty/deadband/timeout/polarity specification and diagnostic I2C register map.
- Frozen GPIO21 permit-heartbeat/watchdog timing derived from the final latch schematic, including startup grace, loss response and reset-release versus torque-permit states.
- Approved toolchain/library versions for both targets and the supervisor/node compatibility matrix.
- Measured motor resistance, inductance, flux/back-EMF constant, no-load current, loaded temperature, phase order, and sensorless launch/restart envelope.
- Final three-shunt values, amplifier gains, ADC references, offset/gain tolerances, sample timing, fast overcurrent thresholds, and phase RMS/peak limits.
- Selected battery/BMS, exact 3S–6S warning/cutoff limits, discharge capability, and regenerative charge-acceptance rules.
- Branch continuous-current, inverter phase-current, transient duration/duty, fuse, terminal, busbar, PCB, cooling, and motor thermal limits.
- Observer qualification/loss thresholds, lowest trustworthy FG speed, coast timeout, moving-start policy, and optical-tach evidence.
- Braking/deceleration policy, bus-energy analysis, and any required dump path.
- Final current/thermal trip thresholds, latch/clear classifications, INA238/NTC calibration, and connector pinouts.
- ESP32-S3 R8-module ambient-temperature decision, including PSRAM-ECC configuration and memory budget if operation above +65 °C is required.

Do not invent missing values in either firmware target. Keep traction inhibited until the required inputs are supplied, reviewed, and validated.
