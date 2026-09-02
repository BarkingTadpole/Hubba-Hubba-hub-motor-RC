# RC Car Powertrain Controller

ESP-IDF firmware for a four-motor 1/7 scale RC car using an ESP32 DOIT
DevKit, four Hobbywing Skywalker 50A V2 ESCs, a Radiolink RC6GS V3/R7FG
radio system, four Hobbywing HW86060041 RPM sensors, and an Adafruit
ISM330DHCX IMU.

The normal drive path supports forward/reverse, receiver calibration,
validated speed-limited steering output, configurable permanent arming,
receiver-loss safety handling, selectable AWD/FWD/RWD operation, and
per-wheel ESC outputs.
Torque vectoring is compiled enabled in the requested first-boot profile, but
it remains inactive until its IMU/RPM/calibration prerequisites are valid and
CH5 requests an active mode.

## Project Layout

```text
main/
  app_main.c                 Startup only
  default_config.h           Receiver defaults and IMU mounting reference
  cli.c                      USB serial command parser
  config_store.c             NVS calibration and configuration
  cornering_control.c        Speed, steering envelope, and lateral demand
  drivetrain_control.c       AWD/FWD/RWD selection and inactive-axle masking
  powertrain_controller.c    Drive state, safety, and calibration workflows
  rc_input.c                 Four priority-3 GPIO PWM inputs
  servo_output.c             Validated 50 Hz steering-servo output
  esc_output.c               Four throttle and four reverse PWM outputs
  rpm_sensor.c               Four PCNT-based motor-speed inputs
  imu_sensor.c               ISM330DHCX I2C driver
  sensor_i2c_bus.c            Shared 400 kHz IMU/Dragy I2C bus
  dragy_nmea.c               Checksum-validated GPS NMEA parser
  dragy_sensor.c             Dragy UART GPS and I2C device discovery
  remote_command.c           Wi-Fi configuration/calibration whitelist/parser
  telemetry_log.c            Offline wear-levelled all-channel CSV recorder
  steering_input_filter.c    Full-range steering spike rejection
  steering_curve.c           Servo-to-road-wheel curve interpolation
  torque_vectoring.c         Straight and turn-assist controller
  wifi_control.c             SoftAP and on-device dashboard/API server
  pin_config.h               All external pin assignments
  include/                   Module interfaces and shared types
docs/
  powertrain_architecture_v1.md
  project_showcase_handoff.md Shared showcase facts, status, and media paths
tools/wifi_bridge/
  bridge.py                  Legacy/mock dashboard development bridge
  web/                       Dashboard assets embedded into the firmware
```

## First Setup

Use the ESP32 USB serial monitor at `115200` baud. Keep the wheels off the
ground and the ESC traction battery disconnected while configuring inputs.

1. Set the R7FG to standard PWM mode with its built-in gyro disabled.
2. Configure receiver failsafe: CH1 centered, CH2 neutral, CH4 at either
   STOP/OFF position, and CH5 OFF.
3. Send `cal receiver`.
4. Send `cal steering`.
5. Send `cal arm` and capture RUN/ON, STOP/OFF 1, and STOP/OFF 2.
6. Send `cal tv`.
7. Leave CH4 in either STOP/OFF position and calibrate the four ESC endpoints.
8. Keep the car level and completely still for the first five seconds of every
   ESP32 boot while the firmware automatically calibrates the IMU yaw bias.
   Use `cal imu` only to retry a failed calibration.
9. Use `monitor rpm` and rotate each wheel to validate all four speed inputs.
10. Keep CH5 at OFF for initial equal-output drive testing.

Receiver CH1 now connects only to the conditioned GPIO16 input. Connect the
servo signal to GPIO32, power the servo from its BEC/receiver rail, and keep
all grounds common. Do not power the servo from the ESP32.

The optional Dragy Lite interface uses UART1 through the ESP32 GPIO matrix,
leaving UART0 available for USB flashing and the serial CLI:

| Dragy Lite wire | ESP32 connection |
|---|---|
| Yellow TX | GPIO4, UART1 RX |
| White RX | GPIO5, UART1 TX |
| Blue SDA | GPIO23, shared with the IMU |
| Green SCL | GPIO22, shared with the IMU |
| Black GND | Common ground |
| Red power | Suitable regulated 5-16 V rail; not an ESP32 GPIO |

GPS receive operation requires only yellow TX to GPIO4 and common ground.
GPIO5 is an ESP32 boot-strapping pin; the Dragy RX input must be high
impedance and no external pull-up/down may be added. Measure the UART and I2C
logic-high levels before connection and level-shift any signal above 3.3 V.
The shared I2C bus already runs at 400 kHz. Confirm that the Dragy bus pull-ups
reference 3.3 V and that the combined pull-up resistance is suitable before
joining it to the IMU bus.

The measured receiver defaults are centralized in `main/default_config.h`:

| Control | Positions in microseconds |
|---|---|
| Throttle | neutral 1514, full 978, reverse 2044 |
| Steering | center 1518, left 2050, right 987 |
| Torque vectoring | off 2047, straight 1513, full 981 |
| Arm/output inhibit | run/on 983, stop/off 1 2049, stop/off 2 1515 |

Saved calibration in NVS overrides these values. Run the calibration commands
after flashing; compiled defaults do not by themselves authorize arming.

The requested first-boot drive profile is also compiled in: AWD, permanent arm
latch on, 100% reverse limit, 100% drive smoothing, the `1565 +/- 10 us`
throttle failsafe detector enabled, 14 poles/7 PPR, zero steering trim,
10 ms steering smoothing, the
1.0 g speed steering limit enabled, torque vectoring configured on at 25%
authority, 20% front relief, IMU yaw sign `-1`, 1 Hz logging, and TV gains
`180, 0.20, 0.00025, 0.00004, 0.20`. Existing valid NVS values still override
these first-boot defaults after an ordinary firmware update.

The IMU mounting reference is `+X` toward the rear and `+Y` toward the right
side of the car. Confirm yaw polarity with `monitor imu` before enabling torque
vectoring.

To drive, put CH4 in either STOP/OFF position once after boot, hold throttle
neutral, then move CH4 to RUN/ON. RUN and neutral throttle must remain valid
for 250 ms before drive arms. Permanent arm latching is enabled by default:
after that first transition, CH4 STOP/OFF and receiver safety faults command
safe ESC output without clearing the armed state. Valid RUN and throttle input
can then restore control through the configured acceleration ramp. At 0%
drive smoothing that recovery is immediate, so return throttle to neutral
before restoring a lost receiver signal or CH4 RUN.

Use `config arm-latch off` while `DISARMED` to select conventional behavior.
In that mode, CH4 STOP/OFF, CH4 or throttle loss, a confirmed unrecognized CH4
position, and the configured throttle failsafe return the controller to
`DISARMED`; another healthy STOP/OFF-to-RUN/ON transition is required. Use
`config arm-latch on` to restore the default. The choice persists in NVS.

The fixed throttle-pulse failsafe detector defaults to the measured
`1565 +/- 10 us` transmitter-off pulse. CH4 safe-output inhibition and actual
receiver PWM-loss detection remain active independently. Use
`config failsafe <pulse_us> <window_us>` to change the detector, or
`config failsafe off` to disable it.

CH4 remains a software control, not an independent emergency stop. With the
permanent latch enabled, CLI `disarm` holds safe outputs for the rest of the
boot without clearing the latch; reset or power loss clears it. With the latch
disabled, CLI `disarm` returns normally to `DISARMED`. In either mode, retain
an independent, immediately accessible control that removes ESC traction
power or hardware enable.

The explicit `disarm config` maintenance command is an exception to the
permanent policy. It immediately restores safe outputs, clears the armed state,
and unlocks configuration. The browser exposes the same action as **Disarm for
config**. It never rearms remotely: another physical CH4 STOP/OFF-to-RUN/ON
cycle and the normal neutral/250 ms qualification are required.

## Serial Commands

```text
status
arm
disarm
disarm config

cal receiver
cal steering
cal arm
cal tv
cal imu
cal esc arm
cal esc max
cal esc min
cal manual
cal cancel

monitor throttle
monitor steering
monitor steering trim <-15..15>
monitor arm
monitor tv
monitor rpm
monitor imu
monitor gps
monitor gps raw
monitor vector

config drivetrain <awd|fwd|rwd>
config arm-latch <on|off>
config reverse <0-100>
config drive smoothing <0-100>
config failsafe off | <pulse_us> <window_us>
config rpm poles <even 2-60>
config rpm ppr <1-120>
config steering trim <-15..15>
config steering smoothing <0-500>
config steering speed-limit <on|off>
config steering lateral-g <0.2-3.0>
config tv authority <0-25>
config tv front-relief <0-50>
config tv gains <yaw_gain_dps> <turn_rpm_gain> <yaw_kp> <yaw_ki> <rpm_kp>
config imu yaw-sign <-1|1>
config logging rate <1-50>

tv enable
tv disable
```

Drivetrain mode defaults to AWD and persists in NVS. Change it only while
DISARMED with `config drivetrain awd`, `config drivetrain fwd`, or
`config drivetrain rwd`. During normal drive, an inactive axle receives safe
ESC commands: `1100 us` throttle and `1100 us` reverse/direction. ESC endpoint
calibration and manual ESC relay continue to address all four ESCs regardless
of the selected drive mode. Configuration requires `DISARMED`; with the
permanent arm latch enabled, that means configuring before the first arm or
restarting the controller.

`config drive smoothing <0-100>` controls both normal acceleration and
deceleration ramps and persists in NVS. `100` is the smoothest setting and
retains the established `12 us` acceleration and `100 us` deceleration step
per 20 ms drive update. Lower percentages shorten both ramp times; `50` uses
`24/200 us` steps, and `0` disables both ramps so requested throttle changes
apply on the next drive update. The mandatory 120 ms zero-throttle hold before
changing the ESC direction wire remains active at every setting.

`tv enable` is guarded. The automatic five-second startup IMU calibration (or
a later successful `cal imu` retry) must have succeeded during the current
boot, and every RPM input must have produced valid pulses. CH5 then selects
OFF, straight-line assist, or full turn assist.

The user confirmed that one mechanical revolution produces seven rising edges,
so the compiled `7 PPR` conversion is correct for the tested motor/sensor.
Use `config rpm ppr 7` to restore that value if configuration changes. The
legacy `config rpm poles 14` command remains backward compatible and derives
the same value. `monitor rpm` prints raw frequency, edge count,
measurement-window duration, and derived RPM. Check all four installed
channels against an optical tachometer across speed to detect noise or
missed/extra edges.

Torque vectoring uses and requires live RPM only from the selected driven
axle: all four wheels in AWD, the front pair in FWD, and the rear pair in RWD.
FWD/RWD corrections remain equal-and-opposite across the driven axle. Normal
reverse remains equal-output on the selected driven axle.

Steering setup requires a verified trim value in the persistent NVS drive
configuration. A stored `0.0 degrees` is valid when no correction is needed.
While disarmed, `monitor steering trim <degrees>` validates and saves the
trim, waits for the drive loop to apply it, and then monitors the resulting
center. `config steering trim <degrees>` saves the same value without opening
the monitor. If the NVS write fails, the in-memory change is rolled back.
Trim is a command-space adjustment based on the conventional
`1000 us = 90 degrees` servo scale; it is not a measured road-wheel angle.
Either command is rejected if the trimmed center would not remain strictly
inside both calibrated endpoints.
Smoothing is a first-order time constant in milliseconds and defaults to
`10 ms`. A missing CH1 signal still centers the servo immediately. Set
`config steering smoothing 0` to disable smoothing while troubleshooting.
A three-distinct-frame median filter rejects an isolated valid-looking CH1
pulse anywhere in the steering range. Once active, it adds one receiver frame,
approximately `20 ms`, of predictable steering latency. Startup and signal
recovery hold the saved center until three new frames are available, normally
about `40-60 ms`. `status` and `monitor steering` report the raw input, median,
filter state, and a confirmed isolated-spike count. Repeated control-loop reads
of one receiver frame do not advance the filter.

Speed-sensitive steering is enabled by default using the measured `107 mm`
wheel diameter and `445 mm` wheelbase. Average rear-wheel RPM estimates
vehicle speed, and the measured steering curve limits average road-wheel angle
to a configurable lateral-acceleration ceiling, initially `1.0 g`. At
`20 km/h` the initial limit is approximately `8.05 degrees`; at `40 km/h` it
is approximately `2.02 degrees`. A valid speed held during DRIVE_ARMED is not
discarded merely because both rear RPM channels become temporarily invalid.
Use `status` or `monitor steering` to see speed, requested angle, maximum
angle, and whether limiting is active.

The corrected 45-point steering table maps the applied, smoothed servo command
from `-45` to `+45 degrees` into separate left-front and right-front road-wheel
angles. Firmware performs linear interpolation between the nonuniform sample
points and saturates outside the supplied range. Source servo-positive steers
left, while each wheel angle is positive when that wheel points outward. The
controller converts both wheel-local signs into its common positive-right
frame. Thus, the supplied centered `LF=+1.64` and `RF=+1.64 degrees` toe-out
becomes runtime `LF=-1.64` and `RF=+1.64 degrees`, with zero mean steering.
`monitor steering` shows the converted, interpolated angles. Torque vectoring
uses their direction-normalized average instead of raw normalized servo
travel. Its yaw target remains empirical; RPM-derived speed currently drives
the steering envelope and front-axle relief rather than replacing that target.

In active FULL mode, predicted lateral demand can remove up to `20%` of full
ESC span from both front motors by default. This applies in AWD and FWD and
deliberately reduces total power rather than transferring it immediately to
the rear. It is automatically zero in RWD because the front axle is already
inactive. OFF and STRAIGHT retain their existing axle behavior. Tune the
feature while disarmed with `config tv front-relief <0-50>`.

`monitor imu` is read-only and never starts calibration. The five-second bias
sample runs once inside firmware initialization on every ESP32 boot. Opening
some USB serial monitors toggles the devkit reset lines; reconnecting such a
monitor causes a new boot and therefore another legitimate startup
calibration. Use a terminal configured not to toggle DTR/RTS when attaching
without resetting the controller.

`status` includes maximum observed sensor/powertrain execution time, deadline
overruns, minimum free task stack, and current/minimum heap. Those values are
runtime instrumentation: collect them on the car during worst-case driving
before treating timing or memory headroom as validated.
It also reports the selected arm policy, current safe-output inhibit,
cumulative inhibit count, and last inhibit reason so recovered receiver faults
remain visible.

## Wi-Fi Browser Dashboard

The default build creates a 2.4 GHz WPA2 access point and hosts the complete
viewer/API on the ESP32. It does not join a phone/Windows hotspot, obtain an
upstream DHCP address, or require Python.

1. Connect a phone, tablet, or laptop to SSID `RC-Car-ESP32`.
2. Enter the default WPA2 password `rc-car-viewer`.
3. Open `http://192.168.4.1`.

The ESP32 supplies client addresses by DHCP. The AP normally has no internet
access, so a device may report “connected without internet”; remain connected
to use the viewer. Change the tracked development credentials before powered
operation under `RC car Wi-Fi access point` in `idf.py menuconfig`. The HTTP
port, channel, and maximum of one to four clients are configurable there too.

The dashboard polls at 4 Hz and shows CH1/CH2/CH4/CH5 input pulses, steering
filter and road-wheel geometry, all RPM sensors, IMU yaw, Dragy GPS fix/speed/
position and UART parser health, shared-I2C discovery, torque-vectoring state,
and all ESC output pulses. `Load saved config` explicitly reloads the current
NVS-backed device configuration into the controls. The drivetrain control can
save a direct AWD/FWD/RWD selection or cycle to the next mode, and the arm
latch control selects permanent or STOP-to-disarm behavior. Firmware still
requires `DISARMED` for writes, validates every range and prerequisite, and
saves accepted values to NVS. Reading the configuration remains available
while armed. The one-way **Disarm for config** control can clear an armed latch
and command safe outputs so the forms unlock; it requires confirmation and a
later physical CH4 rearm cycle. Wi-Fi still cannot arm the car, command
throttle or steering, or use the ordinary boot-long `disarm`.

The dashboard Calibration console provides the same guarded receiver,
steering, CH4, CH5, IMU, ESC-endpoint, and manual-relay workflows as the USB
serial CLI. Receiver workflows show one live instruction at a time and sample
each selected position for 1.5 seconds; values are written to NVS only after
all three positions validate. IMU retry samples for two seconds and remains a
current-boot calibration. ESC endpoint and manual-relay buttons use the
existing controller prerequisites and add explicit browser confirmations;
keep the car lifted and restrained and follow the traction-power sequence.
Configuration remains locked while any calibration is active. An unfinished
browser receiver/IMU workflow times out after 120 seconds, while ESC output
modes retain their existing 30-second timeout. Loss of the dashboard does not
create a remote drive path, and **Cancel / safe output** restores the guarded
DISARMED state when the connection is available.

The page polls the on-device `GET /api/state` endpoint every 250 ms. Static
HTML/CSS/JavaScript, guarded configuration/calibration POST endpoints, and CSV
download/clear all run in the ESP32 HTTP task outside the 20 ms drive loop. A
dedicated static 8 KiB buffer holds the expanded telemetry JSON.

WPA2 access is the only network authentication; the HTTP API has no separate
login. Treat the AP password as a control credential, do not share it with
untrusted devices, and do not add an internet-facing route to this network.
Wi-Fi transmit bursts also increase ESP32 supply demand; given the car's prior
unexplained driving resets, verify the 5 V rail with the AP active before
powered road testing. The legacy Python bridge remains only for mock/frontend
development and is not part of normal operation.

Enable `Dragy Lite GPS` in `idf.py menuconfig` and select the actual
Dragy UART rate. The MAX-M10S default is `9600 8N1`; Dragy may configure a
different rate, especially for higher navigation update rates. Firmware
accepts checksum-valid NMEA RMC, GGA, and VTG sentences and declares them
stale after 1.5 seconds without a recognized sentence. UART GPS startup is
independent of the deferred compass discovery path, so an unavailable I2C bus
does not prevent GPS telemetry.

Use `monitor gps` for a 30-second serial view of UART bytes, NMEA freshness,
fix/position, ground speed, course, altitude, satellite/HDOP quality, and
parser errors. `monitor dragy` is accepted as an alias. Course is GPS course
over ground, not magnetic compass heading.

Use `monitor gps raw` (`monitor dragy raw`) when bytes arrive but no valid NMEA
sentences decode. It clears only a diagnostic RAM buffer, captures five seconds
while the normal parser keeps running, and prints up to the latest 512 bytes as
hex plus ASCII. This distinguishes readable NMEA from a wrong baud rate or a
different binary/proprietary protocol without changing the Dragy output.

Compass support is deferred. The public Dragy Lite material does not identify the compass chip, I2C
address, register map, or heading format; the official developer page requires
program registration before providing integration access. Firmware therefore
reports safe device discovery only and does not guess a compass driver. It
treats every responding address other than the ISM330DHCX at `0x6A` as a
compass candidate and prints the address list at startup. Compass heading
support requires the registered Dragy SDK/device documentation or a positively
identified chip and register protocol. The MAX-M10S itself normally uses I2C
address `0x42`, but Dragy's official harness graphic labels the exposed
blue/green pair specifically as the compass connection, so `0x42` is not
excluded from candidate detection.

References: [Dragy Lite product page](https://www.godragy.com/dragy-lite/) and
[Dragy developer registration](https://www.godragy.com/dragy-api/), plus the
[u-blox MAX-M10S data sheet](https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf).

## Offline CSV Logging and Analysis

The tracked custom 4 MB partition table allocates `0x140000` bytes (1.25 MiB)
to the factory application and `0x2b0000` bytes (2.75 MiB raw) to a
wear-levelled FAT data partition. With `RC car offline telemetry log` enabled,
`telemetry_log.c` samples at `1 Hz` (`1000 ms`) by default in a priority-1 CPU0 task.
It reads a thread-safe controller/Dragy snapshot and performs no filesystem
work in the 20 ms powertrain loop.

The rate is adjustable from `1-50 Hz` with `config logging rate <hz>` or the
Logging rate control in the live viewer's Configuration section. Changes are
accepted only while `DISARMED`, take effect without a reboot, and persist in
NVS for offline logging on later boots.

`telemetry.csv` contains 128 columns: boot/sample identity; controller and
inhibit state; all four receiver inputs; applied steering/filter/road-wheel
geometry; per-wheel RPM, frequency, edge count and window; all IMU axes; TV
targets/errors/corrections; all eight ESC outputs; Dragy GPS/parser values;
and the drive configuration that produced the row. Recording is independent
of Wi-Fi, continues through network outages, and resumes being exportable
as soon as a device reconnects to the ESP32 AP. It stops at the flash reserve instead of
overwriting any old rows. Export and the explicitly confirmed clear action are
available in the dashboard; clear is accepted only while `DISARMED`. If clear
removes the CSV but FAT still reports no usable clusters, that same explicitly
confirmed action reformats only the dedicated `logdata` partition and reopens
a fresh header. It does not touch firmware, NVS, or receiver calibration.

A FAT write or flush error now latches a storage-fault state and stops further
writes instead of repeatedly hammering the damaged/full filesystem. Export
uses the file's actual readable size and can salvage already-committed CSV
bytes even when the active writer cannot flush its final buffer. The viewer
shows the stored `errno`. A header-only file is reported as zero samples rather
than a CSV parsing failure. Export any real samples before using the destructive
clear/recovery action.

This satisfies lossless logging across a Wi-Fi outage only while bounded
internal capacity remains. Indefinite lossless storage is not feasible on the
onboard 4 MB flash. CSV text is deliberately convenient but relatively large:
at the default 1 Hz, a representative 600-800 byte row gives roughly 60-80
minutes before filesystem overhead and the safety reserve. The dashboard uses
the observed row size and actual FAT free-space value to show a live retention
estimate. A 50 Hz log would retain only about one fifth as long; use a microSD
card or larger external flash for long sessions or indefinite capture.

Rows are flushed and synchronized at least once per second. Wi-Fi loss does
not affect them, but sudden controller power loss can discard approximately
the final second and can still corrupt a flash filesystem. Absolute
power-failure durability would require energy hold-up/journaling or a storage
device designed for it. Export and verify important runs before clearing.

Internal-flash writes can also introduce cache/flash critical-section latency
despite the logger's low task priority. Receiver PWM edge capture is registered
as an IRAM-safe priority-3 interrupt and reads GPIO input registers directly so
cache stalls do not turn delayed edges into valid-looking steering pulses.
With traction power disconnected, exercise sustained logging and file growth
while checking the existing deadline-overrun telemetry, PWM stability,
receiver capture, heap/stack, supply voltage, and reset behavior. Disable the
internal logger or move it to external storage if deterministic control is
affected.

The browser analysis panel loads the current device log or any previously
exported project CSV without uploading it anywhere. It summarizes duration,
peak speed and peak wheel RPM, and plots aligned project-specific groups for
speed, four wheel RPMs, four receiver channels, steering geometry, yaw control,
ESC output, TV correction, and IMU acceleration. Plot rendering is downsampled
for responsiveness; the CSV itself remains complete.

## Build

Open an ESP-IDF 6.x terminal in the project directory:

```powershell
idf.py build
idf.py flash monitor
```

The first build after changing ESP-IDF components is long. Later builds are
incremental. This project enables ESP-IDF's minimal-build option to avoid
building unrelated framework components.

The 2026-09-01 ESP-IDF 6.0.1 SoftAP/on-device-viewer build is `0xfbc70`
bytes, leaving `0x44390` bytes (21 percent) free in the `0x140000` application
partition.
This is code/build evidence only; the firmware and new partition table have
not been flashed or validated on the car in this work session.

ESP-IDF 6 requires the LEDC fade service before its thread-safe immediate-duty
API can be used. Startup installs that service after establishing all eight
ESC channels at safe pulses and before the first update; an installation
failure leaves the controller unavailable. This requirement does not consume
or change any GPIO assignment.

Each build also validates `main/pin_config.h` and generates the self-contained
USB-up wiring diagram at `build/esp32_pin_map.svg`. Generate and open only the
diagram with:

```powershell
.\tools\esp32_pin_map\open_pin_map.ps1
```

See [the architecture document](docs/powertrain_architecture_v1.md) for the
complete pin table, electrical notes, RPM conversion, safety behavior, and
bench-test order. The dated
[engineering review](docs/engineering_review_2026-08-10.md) records the source
research, code findings, torque-vectoring assessment, RPM/tachometer validation
package, performance budget, and unresolved physical evidence.

For a concise, regularly maintained summary intended for the separate project
showcase, use [the showcase handoff](docs/project_showcase_handoff.md). It also
lists copy-ready paths for generated media such as the ESP32 pin map.
