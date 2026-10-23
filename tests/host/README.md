# Host verification

Run native C logic and Python harnesses (C11 compiler required; `CC` selects it):

```sh
python3 tests/host/run_tests.py
```

The runner uses temporary output directories. It covers Ackermann interpolation,
Ackermann turn-center identity at 100%, parallel angles at 0%, intermediate
blending, geometry changes, asymmetric/mirrored angle-to-PWM calibration and
endpoint bounds;
rear speed conversion; drivetrain masks/ramps; median filter; command parser;
production controller modules with simulated inputs and PWM; legacy NVS
compatibility, v1 endpoint migration with curve reset, geometry/angle/input
round trips, invalid new blobs and failed commits; actual live JSON formatting,
complete curves, schema and buffer sizes.

The controller harness links the actual calibration, settings, steering and
diagnostics modules and steps the production control task. It checks CH1
filtering/centering, speed hold, config guards/rollback, equal AWD/FWD/RWD
outputs, STOP-to-RUN qualification, both arm-latch policies, receiver failsafes,
throttle ramps, the 120 ms direction hold, receiver/ESC calibration, cancellation,
timeouts and snapshots. It uses simulated time and locks, not a FreeRTOS scheduler.
Serial and HTTP configuration paths must produce identical settings and retain
their state guards. Wire-format tests also check strict HTTP command-body parsing
and the monitor fixture against the production telemetry schema.

Old removed algorithm/steering-table/GPS parser tests were retired with their
implementations. Recorder lifecycle tests were retired with the recorder.

Browser regression runs the actual embedded `main/web` assets against a synthetic
HTTP ESP32 with Chrome and Playwright WebKit. It needs local loopback server and
browser permissions. Install Playwright outside the repo,
for example in a temporary directory, and set `NODE_PATH` to its node_modules.
Chrome must be installed; run `playwright install webkit` for WebKit.

```sh
NODE_PATH=/tmp/rc-independent-browser/node_modules node tests/host/test_monitor_ui.cjs
```

Tests cover separate desktop (640–2560 px) and touch browser profiles,
phone portrait (320–430 px), landscape (568–932 px), and reduced-height
viewports. They verify 44 px touch targets, 16 px touch input text, persistent
navigation while scrolling, desktop keyboard focus, all 13 configuration forms,
curve draft persistence/validation/preview/submission, held-speed labeling,
remaining calibration commands, drive/config locks, reconnect and unsupported
protocol rejection. Density checks require all four Live panels to fit in
1280x720 and 1024x768 viewports, and receiver/steering panels to fit in the
first 390x844 phone screen. Calibration output states must remain explicit;
held speed retains warning color. `MONITOR_SCREENSHOTS` selects
an optional output directory for simulated monitor screenshots. Browser tests
accept commands against a mock; controller and parser guards have separate
host coverage.
The obsolete TCP/UDP bridge, TCP mock and bridge-only tests are removed.

Always also run a full ESP-IDF 6.x `idf.py build`. These checks do not validate
physical PWM, actual servos, receiver safety behavior, NVS durability, ESP32
flash peripherals, real scheduling, Wi-Fi/RF, sudden power loss, or the car.
