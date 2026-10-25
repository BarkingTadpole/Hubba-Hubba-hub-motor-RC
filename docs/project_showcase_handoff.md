# Showcase facts: independent steering branch

Updated 2026-10-24. Four hub motors and four ESC output pairs remain, with
AWD/FWD/RWD selection and equal power commands across the selected drivetrain.
Tank mode adds rotation in place: CH1 selects the spin direction and either
direction of CH2 supplies power. All four motors participate; left turns
reverse the left side and right turns reverse the right. A separate tank
throttle cap defaults to 25%, configurable 0-100% independently of normal
reverse limiting. Zero disables tank power. This is not closed-loop speed control.
Tank keeps the front wheels at their configured toe alignment and bypasses
Ackermann. Mode selection remains disarmed-only; CH1 must be valid and centered
for initial arming. CH1 loss applies the existing latch/disarm policy. Tank
reversals decelerate both sides, hold minimum throttle for 120 ms, and switch
direction at minimum before accelerating.
Each front wheel has its own PWM steering servo (GPIO32 left, GPIO26 right).
The user identified ANNIMOS BLS3325 brushless servos, 25 kg / 180-degree variant,
model DSBL3325180 ([ASIN B0DQPSVCTR](https://www.amazon.ca/dp/B0DQPSVCTR)).
The selected listing was checked on 2026-10-24 and matches the supplied
control/electrical specs; no control values needed changing. The listing's
"25 kg" name and 30 kg.cm at 8.4 V torque table refer to the same variant.
Positional-servo specs: 500–2500 us PWM, 1500 us
nominal neutral, 50–330 Hz supported; firmware uses 50 Hz. The servo shaft's
180-degree rating is separate from measured road-wheel angle calibration.
The stated mounts use CCW/higher PWM on both servos for left turns and
CW/lower PWM on both for right turns. Compiled defaults match; the monitor
flags saved/draft endpoint orders opposite that mounting. Existing calibrated
endpoints are preserved; initial defaults remain 2000/1500/1000 us pending
linkage measurements. Servo power is rated 4.8–8.4 V, with 6.2 A stall per servo
at 8.4 V (12.4 A summed for two); these are ratings, not measured consumption.
Rear-wheel RPM still estimates vehicle speed using 7 PPR and 107 mm wheels.

RC Monitor exposes both servo PWM/angle calibrations, separate signed neutral
trims in microseconds with previews, shared toe alignment, inside/outside linkage travel limits, CH1 input calibration and
deadband, smoothing, dimensions, request-angle limits, RPM PPR, and a 2–8-point
Ackermann-strength versus speed editor. Before toe, 0% commands parallel wheel angles;
100% computes ideal geometric Ackermann using current steering input,
wheelbase and track; intermediate values blend the outside angle. Defaults
are 445/320/107 mm and unmeasured 30-degree endpoint/request assumptions.
Targets use piecewise interpolation through measured per-wheel PWM/angle pairs,
or the unmeasured endpoint approximation until calibration. The same Ackermann curve
handles both turn directions automatically. Per-servo trim moves neutral while
holding calibrated endpoint PWM fixed, with at least 50 us clearance from each
endpoint. Safe linkage travel defaults to 90 degrees inside and 68 degrees
outside, configurable 0–90 in 0.1-degree steps. These are servo-shaft rotations
from trimmed neutral, separate from road-wheel angles. Firmware translates
them to PWM limits using the stated 180-degree / 2000-us range and respects
narrower calibrated endpoints. When either cap binds, both wheel targets
reduce while retaining their chosen Ackermann relationship before adding toe.
With zero toe, a zero limit holds both wheels straight. These commanded limits
still need physical validation.
Toe defaults to zero and accepts the full calibrated/linkage travel range in
road-wheel degrees per wheel, in 0.1-degree steps, positive toe-in/negative
toe-out. There is no separate fixed toe cap. Toe-in uses both outside envelopes;
toe-out uses both inside envelopes. Firmware derives the smaller available
travel from both wheels and reports the asymmetric range to the monitor, which
refreshes bounds without replacing drafts. Toe is an offset after steering
mixing and consumes the same travel envelope. Zero inside travel permits no
toe-out; zero outside travel permits no toe-in.
Startup and signal-loss centering use the configured toe alignment; zero toe
gives exact trimmed neutral. Saves remain
DISARMED-only and blocked during calibration; failed saves restore active values.
RC Monitor includes a guided per-wheel angle measurement workflow. Physical CH1
positions the selected servo within its saved PWM/shaft limits while motors stay
inhibited. Users capture commanded PWM and enter actual signed road-wheel angles,
or enter known measurement pairs manually. Each wheel stores 3-9 points including
straight ahead; piecewise interpolation models nonlinear linkage travel. The save
preview shows endpoint updates and reductions in requested steering maxima.
Unmeasured defaults and migrated values are clearly labeled uncalibrated; live
degree values remain model targets. Configuration saves require finishing the
measurement session. Signal loss, throttle movement, leaving CH4 STOP, cancellation
or 120 seconds without capture exits the mode; no remote steering-position command
exists. Incompatible trim/endpoint edits require clearing and remeasuring that wheel.
NVS v6 stores these curves alongside toe, trims and linkage limits. Valid v5 data
retains all values with zero measured points; valid v4 settings retain all fields
with zero toe. Valid v3 settings retain trims and add
default 90/68-degree limits. Valid v2 settings retain all fields with zero trims
and default linkage limits. Migration is read-only until a configuration save;
corrupt present blobs use defaults instead of falling back to older versions.
NVS v1 PWM endpoints migrate; its old reduction curve resets to parallel
instead of changing meaning.
Live monitor shows PWM, signed model angle targets, strength, rear RPM and
live/held speed. In tank mode the RPM conversion is labeled wheel surface speed,
not vehicle speed. Targets are not measured wheel angles, and this is not a
proven handling improvement.
RC Monitor now uses a compact engineering layout: one header, a drive status
strip, numeric receiver/ESC tables, and side-by-side steering/rear-RPM panels.
Repeated headings, slogans, decorative speed graphics and empty command
banners are removed. Settings use compact rows and horizontal endpoint fields;
longer curve notes expand on demand. LIVE/HELD status, stale-data indication,
configuration locks and calibration warnings remain visible. Browser checks
verify all four Live panels fit at 1280x720 and 1024x768, and receiver/steering
panels fit in the first 390x844 phone screen, with 44 px control targets.
Desktop and touch profiles are tested separately, covering desktop widths up
to 2560 px and phone portrait/landscape layouts. Navigation remains available
while scrolling; touch input text stays at 16 px in either orientation, and
layout margins respect phone safe areas. These are browser simulations,
not tests on a physical iPhone.

Removed capabilities: torque vectoring, front-wheel speed capture, IMU/yaw
feedback, Dragy GPS/compass, CH5 modes, old mechanical steering lookup table,
its lateral-g steering limiter, telemetry recording/CSV analysis and export,
and the pin-map generator. Retain no public claim that those are active on
this branch. Live monitoring and the curve editor remain available.

The former controller monolith is split into motor control/scheduling,
steering, calibration, settings and serial diagnostics. Host regression tests
link these production modules and step the control task with simulated inputs.
Serial and HTTP configuration share one parser/dispatcher; calibration
validation, PWM conversion and state names use shared helpers. Unused accessors,
wrappers, task handles, styles and the obsolete TCP/UDP bridge have been removed.
The monitor assets now live in `main/web/`; tests simulate direct HTTP responses.

Validation (2026-10-24): ESP-IDF 6.1 builds (24% application partition free).
Native host/controller/NVS/wire-format checks pass, including signed trims,
fixed calibration endpoints, inside/outside linkage caps across trims, mounting
directions and geometry, preserved Ackermann blends, boot/failsafe centering,
full toe ranges beyond +/-10 degrees, exact asymmetric boundaries, zero role
limits and sign crossings within calibrated/shaft travel, read-only
v1-v5 migration, v6 persistence and rollback. Measured-wheel tests cover nonlinear
and mirrored interpolation, inverse travel bounds, invalid point rejection,
neutral/endpoint consistency, guarded saves and capture sessions, motor inhibition,
signal/STOP/throttle exits, cancellation and timeout. Tank checks cover both
rotation directions, throttle halves, independent limits, center stops, CH1
warmup/loss, arming, both latch policies and 120 ms holds with/without smoothing.
Direction tests cover the user's inside/outside mapping. A production driver
test checks 500–2500 us output, 50 Hz LEDC, quantization and clamps using
simulated peripherals.
Simulated Chrome/WebKit monitor checks pass all 18 configuration forms,
covering toe and tank mode/limit drafts, validation, save/reload and live labeling,
firmware-derived toe bounds changing with calibration/trims/linkage while retaining drafts,
linkage defaults/drafts/bounds/save/reload, servo range and mounting hints,
trim previews/drafts/validation/save/reload
and existing safety locks. The wheel-measurement workflow also passes capture
and manual entry, per-wheel drafts, save preview, rejection/retry, reload/clear,
offline/armed locks and phone layout checks. These are distinct from
physical tests. No hardware was flashed or physically validated during this
change. Tank rotation, toe alignment, servo/linkage
limits, sensor waveforms, control behavior, supply margin and RF behavior still
require the documented bench progression. Do not carry forward old logging
retention or handling-performance claims.

Simulated RC Monitor screenshots (2026-10-24); updated settings, toe, tank
and wheel-calibration views were visually reviewed. All displayed
readings and the example curve are test data, not hardware measurements,
firmware defaults or a recommended driving setup:

- Desktop Live: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-desktop.png`
- Phone Live: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-iphone.png`
- Phone landscape Live: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-iphone-landscape.png`
- Wide desktop Live: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-desktop-wide.png`
- Settings: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-settings.png`
- Phone Settings: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-settings-phone.png`
- Phone linkage travel: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-linkage-phone.png`
- Phone servo trims: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-servo-trim-phone.png`
- Calibration: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-calibration.png`
- Phone wheel-angle calibration (synthetic measurements): `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-wheel-calibration-phone.png`
- Desktop curve editor: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-ackermann.png`
- Phone curve editor: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-ackermann-phone.png`

- Phone toe alignment: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-toe-phone.png`
- Phone tank settings: `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-tank-settings-phone.png`
- Phone tank Live (synthetic disarmed state): `/Users/ahmadarab/.codex/visualizations/2026/10/25/01a1705b-d42f-7401-954a-90e684d5a2fb/monitor-tank-live-phone.png`
