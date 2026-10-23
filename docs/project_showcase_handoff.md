# Showcase facts: independent steering branch

Updated 2026-10-23. Four hub motors and four ESC output pairs remain, with
AWD/FWD/RWD selection and equal power commands across the selected drivetrain.
Each front wheel has its own PWM steering servo (GPIO32 left, GPIO26 right).
Rear-wheel RPM still estimates vehicle speed using 7 PPR and 107 mm wheels.

RC Monitor exposes both servo PWM/angle calibrations, CH1 input calibration and
deadband, smoothing, dimensions, request-angle limits, RPM PPR, and a 2–8-point
Ackermann-strength versus speed editor. 0% commands parallel wheel angles;
100% computes ideal geometric Ackermann using current steering input,
wheelbase and track; intermediate values blend the outside angle. Defaults
are 445/320/107 mm and unmeasured 30-degree endpoint/request assumptions.
Targets map linearly through per-wheel calibration to PWM. The same curve
handles both turn directions automatically. NVS v1 PWM endpoints migrate;
its old reduction curve resets to parallel instead of changing meaning.
Live monitor shows PWM, signed model angle targets, strength, rear RPM and
live/held speed. Targets are not measured wheel angles, and this is not a
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

Validation: ESP-IDF 6.1 builds (27% application partition free). Native
host/controller/NVS/wire-format checks,
and simulated Chrome/WebKit monitor checks pass. These are distinct from
physical tests. No hardware was flashed or physically validated during this
change. Servo/linkage
limits, sensor waveforms, control behavior, supply margin and RF behavior still
require the documented bench progression. Do not carry forward old logging
retention or handling-performance claims.

Visually reviewed simulated RC Monitor screenshots (2026-10-23). All displayed
readings and the example curve are test data, not hardware measurements,
firmware defaults or a recommended driving setup:

- Desktop Live: `/Users/ahmadarab/.codex/visualizations/2026/10/23/01a16abd-dcf4-7451-a026-ae6c9995833a/monitor-desktop.png`
- Phone Live: `/Users/ahmadarab/.codex/visualizations/2026/10/23/01a16abd-dcf4-7451-a026-ae6c9995833a/monitor-iphone.png`
- Phone landscape Live: `/Users/ahmadarab/.codex/visualizations/2026/10/23/01a16abd-dcf4-7451-a026-ae6c9995833a/monitor-iphone-landscape.png`
- Wide desktop Live: `/Users/ahmadarab/.codex/visualizations/2026/10/23/01a16abd-dcf4-7451-a026-ae6c9995833a/monitor-desktop-wide.png`
- Settings: `/Users/ahmadarab/.codex/visualizations/2026/10/23/01a16abd-dcf4-7451-a026-ae6c9995833a/monitor-settings.png`
- Phone Settings: `/Users/ahmadarab/.codex/visualizations/2026/10/23/01a16abd-dcf4-7451-a026-ae6c9995833a/monitor-settings-phone.png`
- Calibration: `/Users/ahmadarab/.codex/visualizations/2026/10/23/01a16abd-dcf4-7451-a026-ae6c9995833a/monitor-calibration.png`
- Desktop curve editor: `/Users/ahmadarab/.codex/visualizations/2026/10/23/01a16abd-dcf4-7451-a026-ae6c9995833a/monitor-ackermann.png`
- Phone curve editor: `/Users/ahmadarab/.codex/visualizations/2026/10/23/01a16abd-dcf4-7451-a026-ae6c9995833a/monitor-ackermann-phone.png`
