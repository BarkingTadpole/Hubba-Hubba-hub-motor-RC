# Independent-steering powertrain architecture

Updated 2026-10-24 for `experimental/independent-steering`. This describes
implemented firmware and planned wiring. Host tests and builds do not establish
physical validation of the steering or tank-drive system.

## Firmware ownership

`powertrain_controller.c` owns motor state, receiver arming/failsafes, throttle
ramps, the direction interlock, initialization and periodic scheduling.
`powertrain_steering.c` owns CH1 filtering, held rear speed and servo commands.
`powertrain_calibration.c` owns serial/browser receiver captures and ESC
calibration/manual relay. `powertrain_settings.c` owns guarded setting changes
and save requests; `powertrain_diagnostics.c` owns serial status/help/monitors.
`powertrain_commands.c` dispatches parsed operations for both the serial and
HTTP interfaces. Both use `remote_command.c` for configuration parsing;
serial receiver calibration retains its interactive prompts. Shared helpers
define receiver calibration validation, PWM conversion and system state names.
`drivetrain_control.c` handles normal drivetrain masks and tank per-side output
mixing; controller scheduling retains all motor safety guards.

Public callers continue to use `include/powertrain_controller.h`. The private
`powertrain_internal.h` exposes only shared context and cross-module helpers.
The recursive controller mutex guards mutable context and calibration/steering
state; timing counters use a separate short critical section. Blocking receiver
sampling releases the controller mutex while calibration guards remain active.
The 20 ms control loop still updates steering, checks browser calibration
timeout, handles CH4, updates armed drive outputs, then handles ESC calibration.
Torque vectoring has no module, runtime state, configuration or command path.

## Hardware and pin authority

Controller: ESP32 DOIT DevKit V1 / ESP32-WROOM-32, ESP-IDF 6.x (`esp32`).
Four Hobbywing Skywalker 2820SL 550KV motors (12N14P, seven pole pairs) and
four Skywalker 50A V2 ESCs remain. Confirm the exact ESC SKU before selecting
traction-battery voltage; regular and 6S variants may differ. Transmitter is
Radiolink RC6GS V3, receiver R7FG. Two HW86060041 phase RPM sensors remain on
the rear motors. Each front wheel now has its own PWM steering servo.

`main/pin_config.h` is authoritative. Update the wiring table below whenever
the pin assignments change.

| Connection | GPIO | Peripheral |
|---|---:|---|
| Conditioned R7FG CH1 | 16 | GPIO edge input |
| Conditioned R7FG CH2 | 25 | GPIO edge input |
| Conditioned R7FG CH4 | 33 | GPIO edge input |
| Left steering servo | 32 | Low-speed LEDC channel 0 |
| Right steering servo | 26 | Low-speed LEDC channel 1 |
| FL throttle / reverse | 13 / 2 | High-speed LEDC 0 / 4 |
| FR throttle / reverse | 14 / 27 | High-speed LEDC 1 / 5 |
| RL throttle / reverse | 21 / 19 | High-speed LEDC 2 / 6 |
| RR throttle / reverse | 18 / 17 | High-speed LEDC 3 / 7 |
| Rear-left RPM | 36 | PCNT rising-edge input |
| Rear-right RPM | 39 | PCNT rising-edge input |

The prior CH5 wire must be disconnected from GPIO26. Removed inputs include
front RPM GPIO34/35, I2C GPIO23/22, and UART1 GPIO4/5; firmware no longer
initializes those peripherals. GPIO33 remains PWM CH4, with no internal pull-up
or obsolete digital arm-switch interpretation. GPIO36/39 are input-only with
no internal pull-ups. GPIO2 remains an ESC output and boot strap, never a
status LED. Add no pull to GPIO2; investigate ESC-connected boot reliability.
All ten outputs use 50 Hz PWM. The two LEDC groups are separate hardware.
ESC initialization installs the IDF fade service before thread-safe updates.

Use common receiver, servo supply, ESP32, sensor, and ESC signal grounds.
GPIO inputs must not exceed 3.3 V. Condition CH1/CH2/CH4 and RPM outputs as
needed. Receiver DC-average measurements are not logic-high measurements;
verify pulses with a scope. GPIO16 only receives CH1; GPIO32/26 only drive
their respective servos. Verify servo 3.3 V logic compatibility or add buffers.
Power servos from a BEC or regulated supply, never GPIOs. Do not parallel ESC
BEC red wires without manufacturer approval. Keep the R7FG gyro disabled
initially so receiver correction does not obscure steering setup.

### Steering servo specification (ANNIMOS BLS3325, 25 kg / 180-degree version)

The user identified the ANNIMOS BLS3325 brushless servo, model DSBL3325180,
[Amazon Canada ASIN B0DQPSVCTR](https://www.amazon.ca/dp/B0DQPSVCTR), selecting
25 kg / 180 degrees. On 2026-10-24 the selected listing's control and electrical
specifications were checked against the supplied ratings below; they match.
The listing calls it "25 kg" but specifies 30 kg.cm stall torque at 8.4 V.
These are seller-listed ratings, not bench measurements. Do not apply the
270-degree option's angle scale or a different ANNIMOS 25 kg model's ratings.
This is a positional servo with 180 degrees of nominal shaft travel across
500–2500 us, nominal center 1500 us, 2 us internal deadband and a supported
50–330 Hz update rate. The firmware uses 50 Hz (20 ms frames), within that
range. Integer-microsecond commands and 16-bit LEDC duty conversion provide
nominal pulse quantization below the 2 us deadband; physical pulse quality
still needs a scope check. The servo deadband is separate from CH1 receiver
deadband and input filtering.

The implementation audit covers startup centering, live CH1 control, receiver
calibration/recentering, smoothing, signal-loss/failsafe centering, independent
trims, endpoint validation, linkage caps and Ackermann mixing. All servo writes
use the shared output driver, and all non-neutral targets pass through the
same limited mixer. Monitor/serial settings share parser, controller and NVS
validation. No control-value changes were required for this exact model.

| Supply | No-load speed (s/60 degrees) | Stall torque (kg.cm) | Stall current per servo (A) | Two-servo stall sum (A) |
|---|---:|---:|---:|---:|
| 4.8 V | 0.080 | 18 | 3.2 | 6.4 |
| 6.0 V | 0.065 | 22 | 4.0 | 8.0 |
| 7.4 V | 0.055 | 26 | 5.2 | 10.4 |
| 8.4 V | 0.050 | 30 | 6.2 | 12.4 |

Rated operating supply is 4.8–8.4 V; idle current is 5 mA per servo at each
listed voltage. The pair-current column is arithmetic from the supplied stall
ratings, not a measured vehicle current. Account for simultaneous servo loads
and transients when validating the regulated rail, wiring and connectors.
The firmware does not set or measure servo voltage/current and does not model
load-dependent speed or torque from these ratings. Supply voltage does not
establish 3.3 V signal compatibility; the selected listing does not specify
signal-high thresholds, so that remains to be checked.

Increasing PWM rotates the shaft counterclockwise (CCW); decreasing PWM
rotates it clockwise (CW), using the user's rotation convention. The user's
mounting directions imply:

| Vehicle turn | Left wheel/servo | Right wheel/servo |
|---|---|---|
| Left | Inside, CCW, higher PWM | Outside, CCW, higher PWM |
| Right | Outside, CW, lower PWM | Inside, CW, lower PWM |

Both compiled servo mappings therefore use wheel-left PWM above center and
wheel-right PWM below center. At nonzero Ackermann strength, the inside wheel
gets the larger road-wheel angle; outside/inside does not reverse a servo's
rotation. The monitor flags endpoint order opposite the stated mounting.
Saved calibration is retained and requires an explicit configuration save to
change; inspect the displayed mapping on an already configured device.

### Rear RPM sensor wiring

HW86060041 phase wires connect to any two motor phases. Planned sensor supply
is regulated 5 V; black goes to common ground, white to a conditioned GPIO.
The proposed divider is white → 1 kOhm → GPIO junction → 2 kOhm → ground,
about 3.33 V from exactly 5 V. Its opposite orientation is incorrect; a higher
rail can exceed GPIO margin. A 10 kOhm / 15 kOhm divider gives about 3.0 V at
5 V with less loading. Verify the actual waveform and rail. Sensor documents
conflict on minimum supply (3.3 V wiring text versus 3.5 V spec), so ESP32 3V3
is not the preferred final sensor supply. Rear-channel wiring/accuracy remain
bench pending.

## Steering signal path and geometry

CH1 capture accepts 800–2200 us, times out after 100 ms, and uses an IRAM-safe
priority-3 GPIO ISR on CPU0. It reads GPIO registers directly so internal-flash
cache stalls do not defer edges. Drive/steering updates run on CPU1 every
20 ms. Preserve this GPIO scheme; earlier MCPWM/RMT experiments were rejected
following channel starvation and an unsafe bench incident.

The distinct-frame median rejects isolated spikes; repeated reads of the same
timestamp cannot warm it. Three fresh frames are required after startup or
signal-loss reset. A first-order command filter follows, default 10 ms,
configurable 0–500 ms. CH1 loss, filter warmup, and a matching enabled throttle
failsafe center both servos immediately, bypassing smoothing. Normal steering
continues while drive is disarmed. CH4 inhibits motors, not steering.

CH1 calibration stores left/center/right/deadband independently of servo
endpoints. Defaults are 2050/1518/987 us and 30 us deadband. Both guided capture
and manual numeric settings are available. Manual pulses must bracket center
by at least 150 us within 800–2200 us. Deadband is 0–150 us and must be smaller
than both spans. Guided recalibration preserves the selected deadband. After
filtering, receiver travel is normalized by its direction-specific span to
[-1,1], positive right.

### Geometric Ackermann and calibration

Each servo stores left/base-center/right PWM in 500–2500 us and an independent
signed neutral trim in integer microseconds, default zero. Neutral PWM is
`center_us + trim_us`; positive trim increases PWM regardless of mounting
direction. Base center and trimmed neutral must both remain between the
endpoints, at least 50 us from each. Invalid combinations are rejected.
The absolute trim bound is +/-1900 us; actual limits follow the saved
calibration (defaults allow -450 through +450 us). Endpoint order handles reversal.
Each servo also stores positive measured road-wheel angles at its left/right
endpoints, 1–85 degrees at 0.1-degree precision. Trimmed neutral means straight
ahead, zero degrees. Mapping angle to PWM uses piecewise interpolation through
saved measured wheel points when available; otherwise it uses a linear
approximation from trimmed neutral to each endpoint. Trim leaves endpoint pulses and the Ackermann
relationship unchanged; moving neutral can change the available road-wheel
travel under the linkage limits below. Defaults 2000/1500/1000 us and 30/30 degrees are
unverified starting assumptions. Intermediate measurements can model nonlinear
linkage travel; compliance, hysteresis and tire slip are not modeled.
Displayed angle targets are not sensor feedback.
The 180-degree rating is servo-shaft travel, not a 180-degree road-wheel turn.
At the nominal linear shaft scale, 1000/2000 us corresponds to about +/-45
shaft degrees, but no fixed shaft-to-road-wheel ratio is assumed. Keep the
initial 1000–2000 us travel until safe linkage endpoints are established;
500–2500 us is an allowed servo range, not a verified mechanical travel limit.
The shared `steering_servo_limits.h` supplies model and output-driver bounds.

RC Monitor has separate left/right trim saves and neutral previews based on
saved endpoint calibration. Endpoint edits preserve existing trims and reject
conflicts. Setting a trim to zero restores its base center. A successful save
takes effect at the next steering update; failed saves restore the prior RAM
value. Startup, CH1 loss/filter warmup, receiver failsafe and explicit recenter
all use the configured toe alignment immediately through the existing centering paths
(exact trimmed neutral when toe is zero).
CH1 input calibration remains independent of these output trims.

### Measuring wheel angles in RC Monitor

Open **Calibrate > Wheel angle calibration**. Left and right front wheels have
separate saved measurements and separate browser drafts. Until measurements are
saved, the monitor labels the wheel uncalibrated. Existing endpoint values and
all older NVS migrations remain unmeasured; software validation is not a physical
calibration. The displayed live wheel degrees are still targets after calibration.

1. Disconnect traction power and power the servos. Set toe to zero. With CH1
   centered, adjust trims until each wheel points straight ahead. Keep CH2
   neutral and calibrated CH4 in a recognized STOP position.
2. Select a wheel and **Start measurement**. Motor PWM remains 1100/1100 for
   all four ESCs and arming is inhibited. Physical CH1 positions only that servo
   by interpolating PWM from its trimmed neutral to each saved endpoint, bounded
   by the inside/outside shaft limits. The other servo stays at trimmed neutral.
   This mode bypasses the assumed road-wheel conversion, Ackermann, toe and tank
   steering. No browser command specifies a servo output position.
3. Hold each position still and **Capture PWM**, then enter the physically measured
   road-wheel angle: negative left, positive right, zero straight ahead. Captured
   PWM is the commanded pulse, not feedback from the servo or an angle measurement.
   Record both sides and neutral, plus intermediate positions for nonlinear motion.
   Manual entry of previously measured PWM/angle pairs is also supported.
4. **Finish measurement**, review the proposed endpoints and requested steering
   maxima, then **Save measured calibration**. Repeat for the other wheel. Drafts
   survive polling, navigation and connection loss; page reload retrieves saved
   measurements. Save failures retain the draft and prior firmware configuration.

Use 3-9 points per wheel with angles in 0.1-degree steps, within -85..85 road-wheel
degrees and 500..2500 us. The firmware requires strictly increasing angles and
strictly monotonic PWM in the configured mounting direction. Include 0 degrees
at exactly `center_us + trim_us`. Outer points must represent at least 1 degree
in each direction and bracket base center and trimmed neutral by at least 50 us.
If the neutral is not physically straight ahead, finish and adjust trim first.
Capturing the same PWM twice does not add a duplicate browser point.

Saving replaces the selected wheel's endpoint PWM/angle values with the outer
measurements. The monitor previews any reduction of shared requested maxima to
fit both wheels; those values are explicitly included in the same guarded atomic
save. Other dimensions, toe, base centers, trims and the other wheel stay intact.
Toe and linkage bounds use the inverse measured mapping. No extrapolation beyond
measured endpoints occurs. Incompatible toe/geometry or nonmonotonic data is rejected.
Subsequent trim/endpoint edits are rejected if they invalidate the measured points.
**Clear saved measurements** removes that wheel's curve and retains its endpoints
as an unmeasured approximation, allowing trim/endpoint adjustment before recalibration.

Measurement start requires valid centered CH1/CH2, saved CH4 STOP, no configured
failsafe match and zero toe. CH1/CH2 loss, CH2 leaving neutral, failsafe, CH4 leaving
recognized STOP, cancellation or 120 seconds without a capture ends measurement.
The arm cycle resets and normal steering resumes. Configuration cannot be saved
during measurement; finish first. These guards do not replace traction isolation.

### Toe and linkage travel

**Toe alignment** is one shared signed road-wheel angle, default 0 degrees,
configurable over the full calibrated/linkage range in 0.1-degree steps per wheel.
There is no separate fixed toe angle cap. Positive toe-in
points the fronts toward each other: add +toe to the left wheel angle and -toe
to the right, with rightward wheel angles positive. Negative toe is toe-out.
This is an intentional angle offset after the geometric blend, separate from
the trims that calibrate straight-ahead. PWM endpoints and trims remain fixed.
Startup, input loss, receiver failsafe and tank mode all hold this toe alignment.

Static toe must fit both calibrated/linkage envelopes; reject edits to toe,
trim, endpoints or linkage caps that violate this. Toe-in is bounded by the
left wheel's rightward and right wheel's leftward outside travel; toe-out by
their opposite inside travel. Convert each available PWM span through its
calibrated road-wheel angle, take the smaller of the two wheels, and round
inward to 0.1 degree. These bounds can differ and change with saved endpoints,
trims and linkage limits. The monitor displays the firmware-derived bounds
and refreshes them without replacing unsaved toe input. A zero inside cap
permits no toe-out; a zero outside cap permits no toe-in.
Deduct each toe offset from the travel available in the
requested direction, then reduce the common steering request as necessary.
The Ackermann relationship is maintained for the steering components before
toe, not the resulting total wheel angles. Displayed model targets include toe.

RC Monitor's **Safe linkage travel** form sets two servo-shaft limits, measured
from each servo's trimmed straight-ahead position. Inside defaults to 90 degrees;
outside defaults to 68 degrees. Each accepts 0–90 degrees in 0.1-degree steps.
Left turns apply inside to the left wheel and outside to the right; right turns
swap the roles. With toe, each total wheel angle selects its physical inside or
outside travel envelope, including near-center sign crossings. With zero toe,
either limit at zero holds both wheels straight to preserve the angle relationship.
These values are separate from road-wheel angle calibration.

The nominal 180-degree / 2000-us shaft scale converts each limit to a maximum
PWM displacement: `floor(shaft_degrees * 2000 / 180)` microseconds from trimmed
neutral. Thus 90 degrees permits up to 1000 us and 68 degrees up to 755 us
(67.95 nominal shaft degrees). Calibrated endpoint spans and absolute 500–2500 us
bounds can only narrow this envelope. The initial 1000–2000 us endpoints remain
in place; setting a wider linkage limit does not extend them. Limits do not
rewrite any endpoint, trim or road-wheel calibration. They constrain commanded
rotation using the supplied servo scale; actual linkage clearance and angular
accuracy remain physical checks.

Geometry stores wheelbase/front track (50–2000 mm), rear-wheel diameter
(10–500 mm), and maximum requested left/right angles (1–85 degrees), all at
0.1-unit precision. Defaults are the user's earlier 445/320/107 mm dimensions
and 30/30-degree requests. A maximum request must not exceed the corresponding
calibrated endpoint angle on either servo. Invalid combinations are rejected,
so output clipping does not silently break the angle relationship. Lower a
request limit first when reducing calibrated travel. This static travel bound
is separate from the removed lateral-g/speed steering limiter.

For normalized CH1 magnitude `u`, the requested inside target `di` starts at
`u * max_angle` for that turn direction. Let `L` be wheelbase and `T` front track:

```text
di = abs(command) * direction_specific_max_angle
ideal_outside = atan2(L * sin(di), L * cos(di) + T * sin(di))
outside = di + (ideal_outside - di) * strength / 100
```

Angles in the trigonometric calculation use radians. At 100%,
`cot(outside) - cot(inside) = T/L`: both wheels imply the same rear-axle turn
center under the ideal rolling assumption before toe. At 0%, the steering
components are equal/parallel before toe is added.
At intermediate strengths, blend the outside angle while holding the inside
at the driver's request when travel allows. Convert each role's permitted PWM
span through its wheel's calibration to a road-wheel cap. Clamp the inside
target to its cap, then compute the outside blend. If the outside exceeds its
cap, reduce the inside target and recompute the blend (a bounded 20-iteration
monotonic search; parallel and zero-travel cases need no search). This preserves
the selected geometric relationship instead of clipping one wheel alone.
For example, full 500/2500 us endpoint calibration with 30-degree wheel angles
and default 90/68 shaft limits allows only 22.65-degree parallel targets; at
100% Ackermann the default geometry can reach the 30-degree inside target
because the smaller outside angle fits. Reported model angles are the final
limited targets. Geometry therefore responds to steering magnitude
as well as speed. Left turns select the left inside wheel; right turns select
the right. At center, both trimmed PWM neutrals are exact when toe is zero;
otherwise the calibrated toe angles apply. Normal reverse uses the
same mapping.
Neither an IMU nor torque vectoring is involved.

Reference: [SAE, Analysis of Ackermann Steering Geometry](https://doi.org/10.4271/2006-01-3638).
This defines a geometric target, not optimal high-speed tire behavior.

### Speed curve and persistent configuration

Store 2–8 `(speed_kmh, strength_percent)` points with 0.1-unit precision.
First speed is zero; subsequent speeds strictly increase up to 300 km/h.
Strength is 0–100%. Linear interpolation connects points; above the last
speed, hold the final strength. Defaults are (0,0)/(100,0), parallel throughout.
Saving replaces the entire curve in one operation. Negative reductions and
anti-Ackermann from the previous PWM model are no longer part of this curve.

RC Monitor exposes all steering parameters: CH1 calibration/deadband,
smoothing, both servo PWM/angle calibrations and neutral trims, shared toe, linkage travel, dimensions,
request limits, RPM PPR, and point table/preview. All config remains DISARMED-only and blocked
during calibration. Invalid values are rejected by UI, parser, controller and
NVS validation. Draft points survive polling, navigation and device rotation.

NVS uses the validated 152-byte, fixed-width version-6 `cfg/dual_steer_v6` blob.
The first 66 bytes retain the v2 layout, followed by two signed 16-bit trims
and two unsigned 16-bit linkage limits, then a signed 16-bit toe angle in
tenths of degrees (v3 prefix: 70 bytes, v4 prefix: 74 bytes, v5 prefix: 76 bytes).
Two 38-byte wheel calibrations follow: uint16_t count and nine pairs of
uint16_t PWM / int16_t signed angle in tenths. A valid v5 retains all fields
and adds zero measured points. Every older migration also starts unmeasured.
A valid `cfg/dual_steer_v4` retains all calibration, trims, linkage limits and
curve values with zero toe. Every older version also starts with zero toe.
A valid `cfg/dual_steer_v3` retains every field including signed trims, adding
only the default 90/68-degree linkage limits.
A valid `cfg/dual_steer_v2` migrates with every calibration, geometry and curve
field preserved, both trims zero and default linkage limits. Loading migration
never writes NVS; the next configuration save writes v6. On a device with only the old 48-byte
v1 blob, valid servo PWM endpoints are preserved but the old reduction curve
resets to 0% parallel. No other v1
values become geometry/angle calibration. New parameters start at defaults.
A present but corrupt blob uses defaults without falling back to an older
version. A missing key permits the next check in v6/v5/v4/v3/v2/v1 order. Removed single-servo trim
keys remain ignored.
The 500–2500 us servo range and shaft limits do not alter the separate
800–2200 us receiver calibration rules.
Save failures preserve active RAM values. Receiver calibration and active
legacy drive keys remain compatible; no NVS erase is needed.

### Steering limit versus speed

The independent steering-limit graph scales the complete normalized CH1 input
before geometric Ackermann mixing. It is separate from Ackermann strength,
measured wheel-angle calibration, neutral trims, toe and linkage travel limits.
This is a user-defined input range, not the removed mechanical-table/lateral-g
limiter or a measured tire-grip model. At each 20 ms steering update:

```text
range_percent = interpolate(steering_limit_points, live_rear_speed_kmh)
limited_command = filtered_normalized_CH1 * range_percent / 100
inside_request = abs(limited_command) * direction_specific_max_angle
```

The normal mixer then computes the outside angle, applies existing coupled
travel constraints, adds toe and converts both wheel targets to calibrated PWM.
At a 60% graph value, full CH1 requests 60% of the configured angle and half CH1
requests 30%. It scales input rather than clipping everything above a threshold.
The same percentage applies to left and right turns and to forward/reverse.
100% preserves full input range; 0% holds the configured toe alignment. Endpoint
PWM, measured wheel points, trims and toe are not rewritten. Tank and physical
wheel measurement bypass the limiter.

Use 2-8 points, first speed 0, strictly increasing speeds 0-300 km/h and steering
percentages 0-100%, with one decimal place. Percentages may increase or decrease.
Interpolate linearly between points and hold the final value above the last
speed. Defaults are (0,100)/(100,100), preserving prior steering until a curve
is explicitly saved. No percentages are supplied as validated handling limits.

Per the user's requested policy, any unavailable, invalid or stale rear-speed
estimate immediately disables this limiter and restores 100% range. Both rear
RPM channels must meet the existing validity/finite/nonnegative requirements,
with snapshot age at most 100 ms. A held estimate is not live: Ackermann can
continue using held speed while this limiter is bypassed. Fresh valid readings
resume limiting. These transitions can change output with CH1 held stationary.
Rear wheel surface speed during slip is not independent ground-speed feedback.

`config steering limit <speed_kmh:range_percent>...` saves the whole curve
through the shared serial/HTTP parser and normal DISARMED/outside-calibration
guards. Failed saves restore the previous RAM curve. RC Monitor Settings has a
separate graph, add/remove point controls, validation, save and persistent drafts.
Live shows input versus scaled command and the applied steering range, including
100% speed-unavailable bypass, tank bypass and wheel-measurement bypass.

The curve uses a separate 36-byte `cfg/steer_limit_v1` NVS blob: uint16 version
1, uint16 point count and eight {uint16 speed_tenths_kmh, uint16
limit_tenths_percent} pairs. Missing or invalid length/version/points use flat
100% defaults. Existing 152-byte `dual_steer_v6` and its measured calibration
remain unchanged, including legacy migrations. No new NVS work occurs in the
drive loop. Protocol 3 adds `config.steering_limit_points` and steering
`limited_command`/`limit_percent`; config/telemetry buffer limits are host tested.

Validation added on 2026-10-29 covers interpolation, bounds, proportional input
scaling, left/right targets, Ackermann preservation, guards/rollback, NVS
round trips and measured-calibration preservation, stale/invalid RPM bypass and
fresh-speed recovery. These are host/synthetic checks. No firmware is flashed by
these checks. Verify the commanded PWM with traction disconnected, then follow
the restrained low-power progression before evaluating any handling setting.
The user's reported outside-front hopping near full lock above approximately
40 km/h is an observation, not a validated speed threshold or a diagnosed cause.

## Rear speed estimate

Motor speed equals wheel speed for these hub motors:

```text
mechanical_rpm = rising_edge_frequency_hz * 60 / PPR
vehicle_mps = mean(rear_left_rpm, rear_right_rpm) * pi * configured_wheel_diameter_m / 60
```

Wheel diameter defaults to 107 mm and is configurable in steering geometry.
Default PPR is seven, confirmed by the user's one-revolution scope edge count
on 2026-08-12. Rear channels still need waveform and optical-tachometer checks
across speed. Sampling stays 20 ms, with a rolling window of 5–25 samples
(about 100–500 ms), extending until four edges are available. A channel is
invalid after 250 ms without an edge. Snapshot age must also be <=100 ms.
Both rear channels must be valid and finite/nonnegative for a fresh estimate.
Counting quantization is about 85.7 RPM at 100 ms and 17.1 RPM at 500 ms.

While armed, retain the last valid speed if either channel becomes invalid
or the sensor task stops updating; publish `speed_valid=false, speed_held=true`.
This preserves the prior armed dropout policy. Disarmed missing RPM resets
speed to zero/unavailable. Before the first valid measurement, use the curve's
0 km/h point. Since no edges cannot distinguish standstill from sensor loss,
an armed stop after movement also holds the prior estimate until fresh RPM
returns or the car is disarmed. Do not label held speed as live measured speed.

## Motor control and safety

ESC `Brake Type = Reverse` is assumed, not Linear Reverse Brake. The local
`Skywalker_ESC_Manual.pdf`, when available, is the behavior authority. It is
not present in this checkout; no ESC interpretation is changed here.
Throttle is white-wire magnitude 1100–1940 us; yellow reverse is 1100 us
forward, 1940 us reverse. Safe means all eight outputs at 1100 us. Keep
valid safe PWM active while disarmed and yellow low during endpoint calibration.

Normal drive sends equal magnitude to selected motors: AWD all four, FWD only
front, RWD only rear. Inactive motors always receive 1100/1100 us, even in
reverse. Invalid runtime drivetrain mode safes all four outputs. Correct
physical motor directions by swapping any two phase wires on incorrect motors;
do not use tank mode to compensate for incorrect phase wiring. Historical rear-left-only correct
direction is not proof of current phase wiring.

Throttle defaults full-forward/neutral/reverse 978/1514/2044 us, deadband 80 us,
quadratic response. Reverse limit defaults 100%: treat bench tests as full
reverse-authority tests. Drive smoothing 0–100%, default 100%, keeps 12 us
accel / 100 us decel steps per 20 ms; 50% uses 24/200 us; 0% bypasses ramps.
The existing 120 ms direction-change interlock is retained; direction changes
pass through throttle minimum before yellow changes.

### Tank rotation

Tank is the fourth drivetrain selection, configured only while DISARMED and
outside calibration. It uses all four motors to rotate in place, with no
forward/reverse translation request. CH1 left reverses FL/RL and drives FR/RR
forward; CH1 right reverses FR/RR and drives FL/RL forward. CH1 deflection
magnitude does not set power. Either side of CH2 supplies its calibrated
absolute magnitude with the existing quadratic response. Center CH1 or CH2
to decelerate to minimum. The front servos hold the configured toe alignment,
and the speed/Ackermann curve is bypassed.

The independent **Tank speed limit** is a 0-100% throttle cap, default 25%:
`1100 + round(840 * normalized_CH2^2 * tank_limit / 100)` microseconds, equally
on both sides. The normal reverse limit does not apply in tank. Zero disables
tank power. It is not a speed-feedback loop or an angular-speed target. Since
rear RPM is unsigned, tank-mode telemetry labels the converted speed as wheel
surface speed rather than vehicle translation. Live/held validity still applies.

Changing spin direction ramps all four throttles to 1100 us, holds them there
for the full 120 ms, then publishes the new reverse signals at minimum for one
control cycle before accelerating. The hold remains with drive smoothing off;
canceling or changing the requested direction restarts its qualification.
The CH1 median filter remains active, but steering smoothing is bypassed in
tank to avoid a stale spin direction. Raw CH1 center or a raw direction crossing
suppresses the old command immediately; the median must confirm a new direction.

Tank additionally requires valid centered CH1 throughout the initial 250 ms
RUN/neutral qualification. CH1 loss or filter warmup immediately sets every
ESC output to 1100 us, with a dedicated inhibit bit (16) while latched; without
the latch it fully disarms and requires a new STOP-to-RUN cycle. CH2/CH4 loss,
configured failsafe, CLI holds, calibration guards and maintenance disarm keep
their existing policies. Recovery with latch enabled can resume through the
tank direction hold and acceleration ramp, so return controls to neutral first.
ESC calibration/manual relay still addresses all motors with reverse low.

Persistence retains the existing `cfg/drive_mode` values AWD=0, FWD=1, RWD=2
and adds TANK=3. `cfg/tank_limit` is an unsigned byte; missing/invalid values
default to 25%. Failed mode or tank-limit saves restore the previous RAM values.

Initial arming requires saved throttle and three-position CH4 calibration,
valid CH2/CH4, neutral throttle, learned RUN, 250 ms stable qualification, and
a healthy STOP observation since boot. CH4 defaults RUN=983, STOP1=2049,
STOP2=1515 us, with +/-125 us recognition windows. STOP acts immediately;
unrecognized valid-range CH4 requires 60 ms confirmation; missing input times
out at 100 ms. GPIO33 is not a hardware emergency stop.

Permanent arm latch defaults on. Once armed, CH4 STOP and receiver safety
faults temporarily inhibit outputs; valid input may automatically resume
through the acceleration ramp without another arm cycle. Operator must return
throttle to neutral before recovery. With latch off, these events fully disarm
and require healthy STOP-to-RUN. CLI `disarm` with latch on creates a boot-long
inhibit; with latch off it normally disarms. `disarm config` always disarms,
clears the latch, and requires a new physical STOP-to-RUN cycle. Boot never
arms or enters ESC calibration automatically.

The fixed transmitter-off throttle detector defaults on at 1565 +/-10 us,
based on the observed R7FG disconnected output. It immediately inhibits (or
disarms with latch off) and centers steering. Recommended receiver failsafes:
CH1 center, CH2 neutral, CH4 STOP. Always retain independent traction-power
isolation. Missed task deadlines increment counters, reset periodic timing,
and insert a one-tick block; do not weaken deadlines for networking.

## Commands, calibration, and network boundary

USB CLI remains 115200 baud:

```text
status | help | arm | disarm | disarm config
cal receiver | cal steering | cal arm
cal esc arm | cal esc max | cal esc min | cal manual | cal cancel
monitor throttle | monitor steering | monitor arm | monitor rpm
config drivetrain <awd|fwd|rwd|tank>
config reverse <0-100>
config tank limit <0-100>
config drive smoothing <0-100>
config failsafe off | <800-2200 pulse_us> <1-100 window_us>
config rpm ppr <1-120>
config rpm poles <even 2-60>
config steering smoothing <0-500>
config steering input <left_us> <center_us> <right_us> <deadband_us>
config steering servo <left|right> <wheel_left_us> <center_us> <wheel_right_us> <left_deg> <right_deg>
config steering trim <left|right> <signed_us>
config steering toe <signed degrees per wheel within calibrated/linkage range>
cal wheel <left|right>
cal capture | cal cancel
config steering wheel <left|right> <max_left_deg> <max_right_deg> <pwm:signed_wheel_deg>... (3-9 points)
config steering wheel <left|right> clear
config steering linkage <inside_deg> <outside_deg>
config steering geometry <wheelbase_mm> <track_mm> <wheel_diameter_mm> <max_left_deg> <max_right_deg>
config steering ackermann-strength <speed:percent> ...
config arm-latch <on|off>
```

Example only, not a physically validated tuning: `config steering ackermann-strength
0:60 20:30 50:0`. Serial receiver calibrations capture about 1.5 seconds per
prompt. Browser equivalents use start/capture/cancel state machines and a
120-second inactivity timeout. Monitor commands run 30 seconds. Configuration
is DISARMED-only and blocked during calibration. For example, `config steering
trim left 25` adds 25 us to the left servo's neutral; `config steering trim
right -35` subtracts 35 us from the right servo's neutral. These are examples,
not measured alignment settings. `config steering linkage 90 68` restores the
user-requested shaft travel limits; both values accept 0–90 with one decimal.
`config steering toe 1.5` requests 1.5 degrees toe-in per front wheel;
`config steering toe -1.5` requests toe-out. `config tank limit 25` caps tank
throttle at 25%; select it using `config drivetrain tank`. These settings are
saved while disarmed and movement remains controlled only by the transmitter.
`cal steering` learns CH1;
servos continue following valid input, so restrain the chassis and verify
output endpoints first.

ESC endpoint procedure: lift and restrain, disconnect traction power, select
CH4 STOP and neutral, `cal esc arm`, then `cal esc max`. Power ESCs, wait for
maximum beeps, and send `cal esc min` within the manual's five-second window.
After acceptance/ready beeps, `cal cancel`. All four ESCs participate regardless
of drivetrain. Manual relay clamps receiver throttle to 1100–1940 us for at
most 30 seconds, yellow low. Leaving recognized STOP, required signal loss,
configured failsafe, cancellation, or 30-second inactivity restores safe PWM.

Default SoftAP SSID `RC-Car-ESP32`; configure credentials in menuconfig before
untrusted use. Dashboard at `http://192.168.4.1`, no hotspot or bridge needed.
HTTP exposes live telemetry, guarded settings/calibrations, and maintenance
disarm. It must never expose remote arm,
ordinary CLI disarm, arbitrary terminal, monitor, steering, or drive commands.
The obsolete Python TCP/UDP bridge and mock TCP peer have been removed.
Embedded monitor assets live in `main/web/`; browser tests simulate the same
direct HTTP API using a fixture checked against production JSON. Protocol 3
reports only CH1/CH2/CH4, rear RPM, two servo outputs,
Ackermann strength, signed model angle targets, speed live/held state, four ESC
outputs, and complete steering/input configuration. Each config servo object
adds signed `trim_us` and `wheel_points: [{pwm_us, angle_deg}, ...]` (empty means
uncalibrated); `center_us` remains the base center. Calibration kinds include
`wheel-left` and `wheel-right`; `wheel_capture_sequence` identifies new commanded-PWM
captures in `values_us[0]`. During measurement the monitor hides model wheel angles.
Config also includes
`steering_linkage: {inside_deg, outside_deg}`, signed `steering_toe_deg`,
derived `steering_toe_range: {min_deg, max_deg}` and
`tank_limit_percent`. `drive_mode` can now be `TANK`. Protocol stays 3 with these
additive fields; live servo PWM and model targets include toe and the applied limits.

The monitor uses a compact header, status strip and numeric tables. Desktop
Live presents receiver, steering, ESC and rear-RPM panels together; narrow
screens stack panels without reducing 44 px form/navigation targets. Units
remain explicit, and held speed has both a HELD label and warning color.
The header stays visible when scrolling, with scroll padding for focused
controls. Touch inputs use 16 px text in both orientations, and interactive
links have 44 px targets. Horizontal safe-area padding also applies in phone
landscape. Desktop and touch browser profiles are verified separately.
Lost telemetry dims every live panel and locks command buttons. Settings group
drive inputs and servo endpoints into compact rows; curve interpolation and
speed-loss details are expandable. Calibration instructions, ESC warnings,
confirmations and controller guards are retained. Calibration output states
remain identified in Live instead of appearing as disarmed motor outputs.

### Configuration backup and restore

RC Monitor Settings > Configuration backup transfers the complete saved vehicle
setup to another ESP32 running compatible firmware. Export reads saved NVS,
including defaults for absent values and the original saved/uncalibrated flags;
it does not export unsaved browser drafts or transient controller state.
Both export and import require DISARMED and no calibration in progress.

The `.rcconfig` file includes CH2 throttle endpoints/neutral/deadband, CH1 input
endpoints/center/deadband, three-position CH4 calibration and all three saved
flags; drivetrain, reverse/tank limits, drive smoothing, throttle failsafe,
motor poles, RPM PPR, steering smoothing and arm-latch policy; both independent
servo endpoint/angle calibrations, measured wheel points, trims, linkage/toe,
geometry and Ackermann points; and the steering-limit curve. Arm-latch policy
is a setting; the active arm latch and armed state are never restored. Firmware
GPIO assignments, Wi-Fi credentials, live PWM/telemetry and retired keys are
excluded. Match the receiver, wiring and mechanical setup before powered use.

Export downloads `rc-car-configuration.rcconfig`. Selecting a file sends it for
validation and previews drivetrain, saved receiver calibrations, measured wheel
point counts and both curve counts. The user then confirms replacement of the
complete saved setup. Servo alignment can change after import. Malformed,
truncated, oversized, unsupported-version, checksum-failing or semantically
invalid files are rejected without applying any fields. Normal live connection,
DISARMED and calibration locks remain in force; the firmware rechecks guards
when applying, independently of the preview or browser controls.

The portable file is exactly 250 bytes, with an eight-byte `RCVCUCFG` magic,
uint16 version 1, uint16 payload size 234 and uint32 CRC32 of the payload.
All integers are little endian. The payload has a uint16 three-bit saved-calibration
mask and eleven receiver-calibration uint16 values, eleven drive scalar uint16
values, 76 words describing the v6 independent-steering fields and 18 words for
the v1 steering-limit fields. Signed trim/toe/wheel-angle fields use 16-bit
two's complement. No native bool, enum size, padding or pointers are serialized.
Unused point slots are zero-filled. Numeric values and booleans are validated
before narrowing to RAM fields; valid CRC alone is insufficient.

Import first saves the entire checked file in one authoritative
`cfg/profile_v1` blob. RAM applies only after that record save succeeds. Then
compatibility mirrors update the existing `cal` and `cfg` keys without changing
their types or the `dual_steer_v6`/`steer_limit_v1` formats. Unsaved receiver
calibrations remove corresponding legacy keys instead of promoting defaults to
calibrated data. A mirror failure is reported as a warning: the already-saved
complete profile remains authoritative. Subsequent ordinary configuration or
calibration saves preserve the rest of this profile and update the changed
component plus its compatibility keys. Missing profile retains existing legacy
loading/migrations. A present corrupt/invalid profile uses uncalibrated defaults
and cannot silently combine old receiver calibration with new steering settings.
Startup loads the complete bundle once. No blanket NVS/flash erasure is added.

During import all ESC outputs stay safe. The controller retains DISARMED,
clears the arm-cycle qualification and pending RUN timer, updates RPM PPR and
resets steering median/smoothing/speed history before applying the new neutral
and toe alignment. Physical CH4 STOP-to-RUN is required to arm afterward. The
saved-calibration flags remain intact; importing an uncalibrated source does
not bypass required receiver calibration.

HTTP boundaries are GET `/api/config/backup`, POST `/api/config/preview` and
POST `/api/config/import`, with `application/octet-stream` upload/download.
Uploads have an exact 250-byte limit and bounded receive timeout retries. Preview
does not write NVS or control outputs. These endpoints accept this schema only,
not arbitrary NVS keys, terminal text, firmware images or remote arm commands.

2026-10-29 native verification includes an independently packed Python/zlib
golden file, every-byte corruption, truncated/oversized files, full calibration
round trips, semantic rejection with valid recalculated CRC, commit failure,
subsequent edits, corrupt-profile fallback, arming/calibration guards and actual
HTTP handler upload bounds/timeouts. Chrome and WebKit tests cover file download,
preview, malformed files, cancellation, rejection/retry, successful import,
settings refresh and armed/calibration locks. These use synthetic inputs/NVS/HTTP,
not hardware or physical flash/power-loss testing. The documented bench and
power-loss validation requirements still apply. No ESP32 was flashed.

## Storage and verification

Telemetry recording and CSV import/export/analysis have been removed, including
the logger task, configuration, HTTP endpoints, FAT/wear-levelling dependency,
and logdata partition. RC Monitor keeps Live, Calibrate, and Settings views.
Only live telemetry is served. The old recording area (0x150000–0x3FFFFF) is
unallocated. NVS, PHY, and application partitions keep their offsets and sizes.
Old recording settings are ignored; there is no erase or format operation.
The pin-map generator, its assets/launcher, and its build target are removed.
Keep NVS writes out of the 20 ms drive loop.

2026-10-24: ESP-IDF 6.1 build passes with 24% application-partition space free.
Host tests pass, including independent/mirrored trims, fixed endpoint calibration,
inside/outside shaft limits across trims, turn directions, geometry and curve
strengths, preserved Ackermann relationships, boot/failsafe centering,
full shared toe ranges beyond +/-10 degrees, exact asymmetric boundaries, zero
role limits and sign crossings within those envelopes, guarded saves and rollback,
read-only v1-v5 migration, v6 persistence and wire formats. Wheel-calibration
checks cover nonlinear/mirrored interpolation, inverse travel/toe bounds,
neutral/monotonicity validation, guarded saves, rollback, stale trim/endpoint
rejection, physical-CH1 measurement in tank mode, PWM captures, motor inhibition,
CH4/throttle/receiver exits, cancellation and timeout. Tank tests cover
left/right side direction, both throttle halves, quadratic scaling, independent
0/25/100% limits, centered controls, CH1 warmup/loss, arming, latch policies and
full 120 ms holds after deceleration, including smoothing disabled. The production servo-output driver is also tested with simulated LEDC
for the complete 500–2500 us range, clamping, 50 Hz configuration, duty
quantization and initialization failures. Chrome and WebKit simulations pass
all 18 configuration forms, covering toe and tank mode/limit drafts, signed input,
save/reload/rejection, firmware-derived toe bounds updating without losing drafts,
tank speed labeling, linkage defaults/drafts/bounds/save/reload,
expanded servo range/reference/mapping hints, trim drafts/previews, validation,
calibration/configuration locks and responsive layouts. The wheel-measurement
workflow also passes capture/manual entry, per-wheel drafts, save preview,
rejection/retry, reload, clear, offline/armed locks and phone layout checks.
Toe, tank settings, tank Live and wheel-calibration captures were visually
reviewed. No flashing or new physical testing was performed.
Required bench progression:

1. Traction disconnected: scope eight ESC and two servo outputs through boot,
   CH1 motion/loss/failsafe, NVS saves, and network activity.
2. Establish each servo's physical direction, center and safe travel. Check
   the stated CCW/CW mapping with small travel before extending PWM endpoints.
   Verify measured endpoint/intermediate road-wheel angles, left/right inside-wheel
   selection, 0% parallel and 100% geometric targets, trimmed neutral at boot
   and input loss, retained endpoint calibration after trim saves, effective
   inside/outside linkage stops and interpolated strengths at several rear speeds.
   Measure toe-in/out at center and during turns, including trim changes,
   startup/input loss and both limits binding; use zero toe for the initial setup.
   Check worst-case steering deadlines when the bounded linkage search runs
   during concurrent configuration/network activity.
3. Validate both rear RPM waveforms, 7 PPR and optical-tachometer accuracy.
   Disconnect each sensor while armed and verify the held-speed indication;
   verify disarm resets unavailable speed and restores the 0 km/h curve point.
4. Verify both CH4 STOP positions, signal loss, both latch policies, browser
   maintenance disarm/rearming and reverse transition at restrained low power.
   With wheels raised, scope tank left/right side direction, neutral stops,
   CH1 loss and holds after full deceleration; verify the independent throttle cap.
   Confirm ESC direction-switch behavior before any surface skid-turn test.
   Surface grip, tire scrub, linkage loads and current during tank rotation
   are unmeasured; a passing host test does not establish a physical pivot.
5. Check supply integrity, GPIO2 boot behavior, network outages/recovery,
   timing overruns and independent traction isolation.
6. Test one motor before all four; then cautious low-speed vehicle tuning.

Unresolved historical risks include supply-related resets, flash-write receiver
capture timing and LEDC startup fixes validated only in code/build, and actual
AP multi-device/RF behavior. Never equate those with successful hardware tests.
