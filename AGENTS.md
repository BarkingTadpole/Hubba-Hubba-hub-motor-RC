# AGENTS.md

NEVER EDIT THE README.md without EXPLICIT PERMISSION.

## Purpose and branch

Durable engineering context for this repository. Read this file, `README.md`,
and relevant source before changing behavior. Detailed wiring and procedure
are in `docs/powertrain_architecture_v1.md`. Use `Skywalker_ESC_Manual.pdf`
as ESC authority when present; it is absent in this checkout. Do not invent
ESC behavior. This session stays on `experimental/independent-steering`.

Planned wiring, inferred signal behavior, host tests, firmware builds, and
physical bench testing are separate evidence categories. Never report a
successful build or simulation as successful hardware validation.

## Current architecture

ESP32 DOIT DevKit V1 / ESP32-WROOM-32, ESP-IDF 6.x, target `esp32`. Four
Hobbywing Skywalker 2820SL 550KV 12N14P motors, four Skywalker 50A V2 ESCs,
Radiolink RC6GS V3 and R7FG. Confirm exact ESC SKU before traction-battery
selection; regular and 6S ESC variants may have different voltage ratings.

There is one independent PWM steering servo on each front wheel. Receiver
CH1 is validated, filtered, normalized, and mapped to independent servo
endpoints. A user-defined speed/Ackermann-strength curve blends parallel and ideal
geometric wheel-angle targets before per-servo angle-to-PWM conversion. Normal drive supplies equal commands to the selected AWD/FWD/RWD
motors. Rear-wheel speed estimation is retained.

Removed on this branch: torque vectoring, front RPM capture, IMU, Dragy,
shared I2C, UART1 GPS, CH5 mode input, the mechanical steering table, and
its dependent lateral-g steering limiter. Telemetry recording, CSV analysis/
export, and the pin-map generator are also removed. Do not reintroduce their
config, calibration, telemetry, or UI paths. Historical NVS values for those
features are ignored. They do not need erasure.

## Pins and electrical constraints

`main/pin_config.h` is authoritative; update it and architecture docs together.

| Signal | GPIO |
|---|---:|
| Conditioned CH1 steering | 16 |
| Conditioned CH2 throttle | 25 |
| Conditioned CH4 RUN/STOP | 33 |
| Left servo PWM | 32 |
| Right servo PWM | 26 |
| FL throttle / reverse | 13 / 2 |
| FR throttle / reverse | 14 / 27 |
| RL throttle / reverse | 21 / 19 |
| RR throttle / reverse | 18 / 17 |
| Rear-left RPM | 36 |
| Rear-right RPM | 39 |

The user approved GPIO32 left and GPIO26 right. Disconnect old CH5 from
GPIO26 before driving it. GPIO34/35, GPIO23/22, GPIO4/5 are now unassigned.
Never connect driven receiver and servo signals together. Each servo has
its own signal wire and a suitable BEC/regulated power source with common
ground. Verify 3.3 V servo logic acceptance or use a buffer.

All eight high-speed LEDC channels serve ESCs. Two separate low-speed LEDC
channels (0/1) serve steering. All outputs are 50 Hz. Keep four ESC output
pairs independent for drivetrain masking. ESP-IDF 6's thread-safe
`ledc_set_duty_and_update()` requires the fade service, installed by ESC
initialization before the first update.

GPIO2 is a boot strap and the onboard controllable LED; never blink it or
add a pull while it carries FL reverse. Investigate boot reliability with
ESC connected. GPIO33 is CH4 PWM, not a digital active-low switch; do not
add an internal pull-up. GPIO36/39 are input-only with no internal pull-ups.

Receiver, ESP32, RPM sensors, servo rail and ESC signal grounds must share
a reference. Condition signals exceeding 3.3 V. The physical CH2 divider
already exists; verify equivalent CH1/CH4 protection. DC-average readings
of PWM (historically ~0.22–0.385 V) do not establish logic-high voltage.
Use a scope. Do not parallel ESC BEC red wires without manufacturer approval.
Keep R7FG gyro disabled initially during independent-steering setup.

HW86060041 phase leads go to any two motor phases. Planned sensor power is
regulated 5 V, black common ground, white conditioned signal. The planned
1 kOhm series / 2 kOhm ground divider gives ~3.33 V from exactly 5 V; the
opposite orientation is wrong. Measure actual rail and divided waveform;
above ~5.4 V leaves inadequate margin. 10 kOhm / 15 kOhm gives ~3.0 V at
5 V with less loading. Sensor documentation conflicts between 3.3 V wiring
instructions and a 3.5 V minimum spec; ESP32 3V3 is not preferred final power.

## Drive safety invariants

- Establish safe ESC PWM before NVS, receiver, sensor or CLI behavior.
- Safe is four throttle outputs at 1100 us and four reverse at 1100 us.
  Continue valid PWM when disarmed; signal absence can trigger ESC protection.
- Current ESC assumption is Brake Type = Reverse, not Linear Reverse Brake.
  White receives magnitude 1100–1940 us; yellow receives direction 1100/1940.
  Yellow remains low throughout boot and endpoint calibration.
- Never auto-arm or auto-enter ESC calibration at boot.
- Initial arming requires saved throttle and three-position CH4 calibration,
  valid CH2/CH4, neutral throttle, learned RUN, healthy STOP seen since boot,
  then 250 ms continuous RUN/neutral qualification.
- CH4 recognition is +/-125 us at each learned position. Recognized STOP
  acts immediately; unrecognized valid-range pulses require 60 ms confirmation.
  Receiver timeout is 100 ms. Obsolete two-position CH4 data is uncalibrated.
- Permanent arm latch defaults enabled. After initial arming, STOP and receiver
  faults inhibit safe outputs without clearing DRIVE_ARMED. Recovery can
  resume through acceleration ramp without another arm cycle or firmware
  neutral qualification. Operator must return throttle neutral first.
- With arm latch disabled, STOP/loss/confirmed unknown CH4/configured failsafe
  fully disarm and require a fresh healthy STOP-to-RUN transition.
- CLI `disarm` is boot-long inhibit with latch enabled, ordinary disarm with
  it disabled. `disarm config` always clears latch, safes ESCs, and requires
  physical STOP-to-RUN. Remote arm is prohibited.
- CH4 is software protection, not a hardware emergency stop. Powered testing
  needs independent immediately accessible traction-power isolation.
- Throttle is quadratic. Reverse default remains user-requested 100%.
- Drive smoothing default 100% is 12/100 us accel/decel per 20 ms. At 50%,
  steps are 24/200 us. At 0%, ramps bypass; the 120 ms direction interlock
  remains. Direction changes pass through throttle minimum first.
- AWD drives all motors, FWD front only, RWD rear only. Inactive motors stay
  1100/1100 even during reverse. Invalid runtime drivetrain selection safes
  all motors; invalid saved mode falls back AWD. Equal selected-wheel commands
  are intentional. No per-wheel software motor direction inversion.
- ESC calibration/manual relay still addresses all four motors. Saved CH4,
  recognized STOP, and receiver guards remain required. CH4 exit, receiver
  loss, enabled failsafe, cancellation or 30-second timeout restores safe PWM.
- Do not test first-power behavior with wheels on the ground.

## Steering and rear speed

Measured receiver defaults in `main/default_config.h`:
CH2 neutral/full/reverse 1514/978/2044 us, deadband 80 us; CH1 center/left/right
1518/2050/987 us, deadband 30 us; CH4 RUN/STOP1/STOP2 983/2049/1515 us.
Transmitter-off CH2 was observed at 1565 us; its detector defaults enabled
at 1565 +/-10 us. This centers steering and inhibits/disarms motors per policy.

Three priority-3 IRAM-safe GPIO edge inputs on CPU0 feed the powertrain task
on CPU1. Direct GPIO register reads preserve receiver timing during flash
cache-disabled periods. Earlier MCPWM and RMT experiments were rejected;
RMT caused unsafe intermittent drive in a bench incident. Do not revert.

CH1 uses a median over three distinct frames, then first-order smoothing
(default 10 ms, configurable 0–500 ms). Duplicate timestamps do not advance
warmup. Missing CH1, warmup, or matching throttle failsafe commands both
centers immediately, bypassing smoothing. Fresh samples rebuild median history.
Steering continues while disarmed; CH4 controls motors. CH1 calibration stays
separate from servo output endpoint settings.

Each servo has wheel-left, center, wheel-right PWM in 800–2200 us; center must
be bracketed by endpoints at least 50 us away. Swap endpoint order to reverse
mounting; centers trim straight ahead. Left/right endpoint road-wheel angle
magnitudes (1–85 degrees, 0.1-degree precision) map linearly to PWM on each side
of center. Default PWM 2000/1500/1000 and endpoint angles 30/30 degrees are
unmeasured starting assumptions. Live degree values are model targets, not
measured wheel angles. Check nonlinear linkage response at intermediate angles.

Geometry is adjustable: wheelbase/front track 50–2000 mm, rear-wheel diameter
10–500 mm, maximum requested left/right angles 1–85 degrees, all step 0.1.
Defaults are the user's earlier 445/320/107 mm dimensions and 30/30 degrees.
Requested maxima must fit both servos' corresponding endpoint angles; reject
inconsistent settings instead of clipping away the geometric relationship.
CH1 left/center/right/deadband are also editable; calibration endpoints bracket
center by >=150 us within 800–2200 us. Deadband 0–150 us must be less than each
span. Guided recalibration preserves the configured deadband.

Ackermann strength curve: 2–8 points; first speed zero, strictly increasing
speeds 0–300 km/h, strength 0–100%, one decimal. Linear interpolation, final
value held above the last point; default (0,0)/(100,0). The inside wheel angle
is normalized CH1 magnitude times the direction-specific maximum. Ideal outside
angle is atan2(L*sin(inside), L*cos(inside)+T*sin(inside)), in radians. Outside
blends from inside angle at 0% to ideal outside at 100%; inside stays fixed.
Both centers remain exact at zero input. Sign selects the inside automatically;
one curve covers both directions and reverse. No dynamic steering limiter,
IMU, torque vectoring or slip model participates. This replaces the old
percentage-of-PWM reduction semantics, including its negative-value behavior.

Rear RPM uses two PCNT units, rising edges only, 7 PPR (user scope-count
confirmed on 2026-08-12 for the tested motor/sensor). Keep 20 ms sampling,
5–25-sample adaptive windows until four edges, 250 ms no-edge invalidation.
Both rear RPM channels must be valid/finite/nonnegative; snapshots expire
at 100 ms. Speed is mean rear RPM * pi * configured diameter in meters / 60 m/s. Motor and wheel
RPM are equal for hub motors. PPR remains configurable 1–120; legacy even
motor-pole input sets PPR to half poles. Quantization is ~85.7 RPM at 100 ms
and ~17.1 RPM at 500 ms.

Armed RPM dropout retains last valid speed and reports held, not fresh.
Disarmed missing RPM resets unavailable speed to zero. Before any valid
estimate use the 0 km/h curve point. An armed stop after motion is also a
no-edge condition and holds speed; do not silently treat absent pulses as
confirmed standstill. Both rear channels still need clean-waveform and
optical-tachometer checks.

## Configuration, network and storage

Keep `cal` throttle/CH1/CH4 and active `cfg` keys backward compatible.
New independent steering uses a validated 66-byte, versioned fixed-width
`cfg/dual_steer_v2` blob. Validate length/version, PWM/angle calibration, geometry,
travel limits and every point. A v1-only device preserves valid servo PWM
endpoints but resets the old reduction curve to 0% parallel; new geometry/angles
start at defaults. Never reinterpret the old curve. Invalid v2 uses defaults
without falling back to stale v1. Keep migration read-only until a config save.
Do not migrate the removed steering table/trim into unmeasured dual servos.
New curve/endpoints saves validate before apply and restore RAM on failure.
Configuration remains DISARMED-only and unavailable during calibration.

USB CLI 115200. Commands and calibration procedure are in README/architecture.
Monitor duration 30 seconds. Browser receiver captures use structured
start/capture/cancel with 120-second inactivity timeout, not arbitrary terminal
forwarding. Endpoint/manual output operations preserve their existing guards.

SoftAP `RC-Car-ESP32` hosts embedded RC Monitor at `http://192.168.4.1`.
The tracked development credential is configured in Kconfig; change it before
untrusted operation. No hotspot/Python bridge required. Remote paths only
expose live telemetry, guarded config/calibrations, and one-way maintenance
disarm. Never add remote arm, arbitrary CLI, drive,
steering, ordinary disarm, or monitor commands. The legacy TCP/UDP bridge and
mock TCP peer are removed. Embedded assets live in main/web; browser tests
use a synthetic direct HTTP device and a fixture checked against firmware JSON.

Live telemetry protocol 3 removes retired channels and reports rear speed,
live/held validity, both servo outputs, Ackermann strength, commanded model angles and full config.
RC Monitor curve drafts survive polling/navigation. Use clear units and
inside/outside definitions in UI. Do not claim servo PWM is road-wheel angle.

No telemetry recorder, CSV endpoints, Logs page, FAT/wear-levelling dependency,
or logdata partition remains. The former 2.75 MiB recording region is unallocated;
NVS, PHY, and application partition offsets/sizes stay unchanged. Old log_hz
NVS values are ignored. This source change does not erase physical flash.
Keep NVS writes/allocation out of real-time drive loops. Check deadlines,
heap/stack, PWM and supply under worst-case configuration/network activity.

## Engineering workflow and validation

Keep module ownership: powertrain_controller handles motor state, arming,
failsafes and scheduling; powertrain_steering owns CH1 processing, speed hold
and servo mixing; powertrain_calibration owns receiver and ESC workflows;
powertrain_settings owns guarded configuration changes; powertrain_diagnostics
owns serial status/help/monitors. The private powertrain_internal interface
shares context under the controller mutex; public callers use
include/powertrain_controller.h. Pure independent_steering handles geometry
and validation, servo_output owns PWM, rpm_sensor owns PCNT,
cornering_control retains rear-speed conversion, config_store owns NVS,
remote_command parses, and wifi_control serializes/serves live telemetry.
powertrain_commands dispatches shared serial/HTTP operations through the same
guards. Keep PWM conversion in pwm_output.h, receiver calibration validation
in calibration_validation.h, and system state names in powertrain_types.h.
Do not collapse into a monolith. Use deterministic fixed-size real-time paths,
fixed-width persisted types, guarded cross-task state, and checked init errors.
Follow nearby C style, ASCII, concise hardware/safety comments.

Use ESP-IDF 6.x `idf.py build`. Wiring is documented in `main/pin_config.h`
and the architecture guide; no pin-map generation runs during the build.
Do not manually edit generated build files. Do not flash unless explicitly
requested. Run `python3 tests/host/run_tests.py` for native logic/controller/
NVS/wire-format tests and the browser suite in `tests/host/README.md`.
No test here substitutes for ESP32 peripheral, RF, flash/power-loss, servo,
receiver, ESC or vehicle validation.

Preserve unrelated user changes (including mechanical models and editor
settings). Do not destructive-clean Git or restore user files. Do not commit
secrets, personal serial identifiers or credentials. Update README and detailed
architecture when changing pins, commands, calibration, safety, timing, RPM,
steering or network behavior. Keep `docs/project_showcase_handoff.md` current
for public project facts, validation status and generated media; include
absolute copy-ready asset paths and describe simulated versus real evidence.

Current physical gaps remain: new dual-servo directions/centers/endpoints,
road-wheel angle calibration, geometric mixing and held-speed behavior; rear sensor wiring/accuracy;
CH1/CH4 calibration and both latch policies; GPIO2 boot reliability;
CPU-isolated/IRAM receiver capture and LEDC startup fixes during NVS writes;
supply-related resets, RF/AP multi-device behavior, worst-case deadlines and
exact ESC voltage rating. Follow the traction-disconnected then restrained
low-power bench progression in the architecture document before road tuning.
