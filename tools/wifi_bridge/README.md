# Legacy Wi-Fi browser bridge (development only)

Normal firmware operation no longer uses this Python process. The ESP32 now
creates SSID `RC-Car-ESP32` and serves the embedded viewer directly at
`http://192.168.4.1`; connect with the configured WPA2 password (tracked
development default: `rc-car-viewer`). The files in `web/` remain the source
assets embedded into each firmware build.

The bridge below is retained only for frontend/mock development and tests.

The ESP32 connects to the configured 2.4 GHz WLAN and exposes a small TCP
protocol on port 3333. This Python program discovers it over UDP port 3334,
maintains the TCP connection, and serves the dashboard to a local browser.
It uses only the Python standard library.

Run from the repository root:

```powershell
python tools\wifi_bridge\bridge.py
```

Then open `http://127.0.0.1:8080`.

The repeating `GET /api/state ... 200` terminal entries are expected browser
polls. They confirm browser-to-Python HTTP only, not ESP32 telemetry. A healthy
end-to-end stream also prints:

```text
esp: TCP connected to 192.168.x.x:3333
esp: first valid telemetry frame received
```

The page shows browser/bridge, TCP, and telemetry freshness separately. If TCP
is absent, use the ESP32's serial DHCP address with `--esp`. If TCP exists but
no first frame arrives, flash the matching firmware and inspect the page's
protocol/JSON error counters. The frontend treats missing fields from older
firmware as unavailable instead of aborting the whole live display. A favicon
request is now answered with `204`; its earlier `404` was harmless.

If UDP broadcast discovery is blocked, use the IP printed by the ESP32:

```powershell
python tools\wifi_bridge\bridge.py --esp 192.168.1.50
```

Automatic mode remembers the last address that completed a TCP connection.
After a link drop it tries that address directly before broadcasting another
discovery request. This lets the viewer recover when Windows Mobile Hotspot
still lists the ESP32 as connected but temporarily stops forwarding UDP
broadcasts. If the ESP32 receives a different DHCP address, the failed direct
attempt falls back to normal discovery. The dashboard shows the remembered
reconnect target while the TCP session is down.

If the console repeatedly reports a connection but `rx_bytes` remains zero,
the ESP32 has not delivered even its protocol greeting. Firmware predating the
8 KiB telemetry-buffer correction will do this because its expanded JSON frame
does not fit the old 4 KiB buffer. Flash the matching current firmware. Current
bridge messages distinguish a five-second silent socket, an ESP32 peer close,
a receive error, and a protocol-buffer overflow.

The HTTP server binds only to localhost by default. To open it to other
devices on the same trusted LAN:

```powershell
python tools\wifi_bridge\bridge.py --listen 0.0.0.0
```

There is no application-level authentication. Do not expose either port to
the internet or run it on an untrusted WLAN. The ESP32 protocol accepts only
the documented configuration commands, exact guided calibration commands, and
the one-way `disarm config` maintenance action. It cannot arm, command drive
throttle or steering, or perform an ordinary remote disarm. Firmware state and
validation remain authoritative.

The Calibration console covers receiver throttle, steering, CH4, CH5, IMU
bias, ESC endpoints, and the bounded manual ESC relay. Start a receiver or IMU
workflow, follow its live instruction, and use **Capture current position**;
receiver PWM is averaged for 1.5 seconds per step and saved to NVS only after
the complete set validates. The IMU retry samples for two seconds and is valid
only for the current boot. Configuration is locked during calibration.
Receiver/IMU workflows time out after 120 seconds without progress, and ESC
output modes time out after 30 seconds. ESC endpoint and manual-relay actions
require their on-page confirmations, the same neutral/CH4/controller guards as
serial, a lifted and restrained car, and immediately accessible traction-power
isolation. **Cancel / safe output** restores safe DISARMED output; it is not a
general remote disarm command.

The ESP32 records its all-channel CSV on internal flash without relying on
this program or Wi-Fi. `Export device CSV` proxies the file in 8 KiB binary
chunks, while the analysis panel can load it directly or open a previously
saved CSV. `Clear after export` requires confirmation in the page and is
rejected by firmware unless the controller is `DISARMED`. The log stops when
full and never silently wraps over old data.

A header-only device CSV is shown as zero recorded samples, not as a parser
failure. If the device also reports a full FAT allocation map with no usable
free space, the button becomes `Recover log storage`. After confirmation, the
updated firmware attempts a normal clear and reformats only the dedicated
telemetry partition if the lost space is not reclaimed. Restart this Python
bridge and flash the matching firmware before using that recovery path.

`Load saved config` repopulates the controls from the fresh NVS-backed
configuration reported by the ESP32. This read is available while armed;
configuration writes still require `DISARMED`. The drivetrain control supports
direct AWD/FWD/RWD selection plus an AWD -> FWD -> RWD cycle button. The
permanent arm-latch control persists whether post-arm safety events retain the
boot latch or fully disarm and require another STOP-to-RUN cycle. The section
also includes a `1-50 Hz` logging-rate control, defaulting to `1 Hz`; accepted
changes apply without reboot and persist for later offline sessions.

While fresh telemetry reports `DRIVE_ARMED`, **Disarm for config** is enabled.
After confirmation it immediately commands safe motor outputs and returns the
controller to `DISARMED`, so the forms can be changed even with permanent
latching enabled. It never rearms remotely: the operator must physically move
CH4 through STOP/OFF and back to RUN/ON, with the normal neutral qualification.

For dashboard development without hardware, run the mock in one terminal and
point the bridge at it in another:

```powershell
python tools\wifi_bridge\mock_esp.py
python tools\wifi_bridge\bridge.py --esp 127.0.0.1
```
