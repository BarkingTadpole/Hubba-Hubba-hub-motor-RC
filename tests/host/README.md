# Torque-vectoring host logic test

The tests are freestanding so Espressif's bundled RV32 Clang can build them
without linking ESP-IDF. Failed checks execute a trap; a normal return from
`main` means every check passed. The runner requires the `pyelftools` and
`unicorn` Python packages.

From an ESP-IDF 6.x PowerShell environment:

```powershell
New-Item -ItemType Directory -Force build\host_tests | Out-Null
$rv = @("-march=rv32imac_zicsr_zifencei", "-mabi=ilp32")
$libm = Get-ChildItem "$env:IDF_TOOLS_PATH\riscv32-esp-elf" `
  -Recurse -Filter libm.a | Where-Object FullName -Match `
  'rv32imac_zicsr_zifencei_zaamo_zalrsc\\ilp32\\libm.a$' | Select-Object -First 1

clang @rv -std=c11 -Wall -Wextra -Werror -I main\include `
  main\cornering_control.c main\drivetrain_control.c main\torque_vectoring.c `
  tests\host\test_torque_vectoring.c `
  -L $libm.DirectoryName -lm `
  -o build\host_tests\test_torque_vectoring.elf
python tests\host\run_rv32_unicorn.py `
  build\host_tests\test_torque_vectoring.elf

clang @rv -std=c11 -Wall -Wextra -Werror -I main\include `
  main\steering_curve.c tests\host\test_steering_curve.c `
  -o build\host_tests\test_steering_curve.elf
python tests\host\run_rv32_unicorn.py `
  build\host_tests\test_steering_curve.elf

clang @rv -std=c11 -Wall -Wextra -Werror -I main\include `
  main\steering_input_filter.c tests\host\test_steering_input_filter.c `
  -o build\host_tests\test_steering_input_filter.elf
python tests\host\run_rv32_unicorn.py `
  build\host_tests\test_steering_input_filter.elf

clang @rv -std=c11 -Wall -Wextra -Werror -I main\include `
  main\cornering_control.c tests\host\test_cornering_control.c `
  -L $libm.DirectoryName -lm `
  -o build\host_tests\test_cornering_control.elf
python tests\host\run_rv32_unicorn.py `
  build\host_tests\test_cornering_control.elf

clang @rv -std=c11 -Wall -Wextra -Werror -I main\include `
  main\drivetrain_control.c tests\host\test_drivetrain_control.c `
  -o build\host_tests\test_drivetrain_control.elf
python tests\host\run_rv32_unicorn.py `
  build\host_tests\test_drivetrain_control.elf

clang @rv -std=c11 -Wall -Wextra -Werror -I main\include `
  main\remote_command.c tests\host\test_remote_command.c `
  -o build\host_tests\test_remote_command.elf
python tests\host\run_rv32_unicorn.py `
  build\host_tests\test_remote_command.elf

clang @rv -std=c11 -Wall -Wextra -Werror -I main\include `
  main\dragy_nmea.c tests\host\test_dragy_nmea.c `
  -L $libm.DirectoryName -lm `
  -o build\host_tests\test_dragy_nmea.elf
python tests\host\run_rv32_unicorn.py `
  build\host_tests\test_dragy_nmea.elf

python tests\host\test_wifi_bridge.py -v
```

Keep Python dependencies outside the source tree or under ignored `build/`.
The C tests cover pure control, drive-ramp percentage scaling, interpolation,
full-range steering-filter logic, remote-command validation, and
checksum-validated Dragy NMEA parsing.
The Python test checks the default SoftAP/direct-HTTP source path, embedded
viewer assets, the requested first-boot configuration profile, and telemetry
JSON buffer headroom. It also retains legacy bridge/mock coverage for telemetry
freshness, NVS-backed HTTP configuration readback, guarded configuration and
calibration commands, the one-way configuration-disarm endpoint, binary CSV
chunk export, HTTP CSV download, recovery diagnostics, log clear, and remote
command rejection. These tests do not
exercise ESP32 flash/FAT peripherals, FreeRTOS
scheduling, receiver safety behavior, Wi-Fi or GPS hardware, power-loss
recovery, physical calibration workflows, or the car.

## Monitor browser regression checks

`test_monitor_ui.cjs` runs the actual dashboard against a synthetic local HTTP
device in Chrome and Playwright WebKit. It checks layouts at 320, 375, 390,
430, 844, and 1440 px, 44 px touch targets, all 17 configuration forms,
calibration commands/captures/cancel, maintenance disarm, CSV export/import,
eight chart groups, touch/keyboard inspection, and connection-loss recovery.
Graph coverage includes button/wheel/keyboard zoom, drag/button/keyboard pan,
window bounds, sample inspection while zoomed, state preservation across views
and groups, reset on CSV load, and native Chromium multi-touch pinch input.
The mock accepts commands for UI verification; firmware guards are covered
separately by the existing host tests. These are not physical iPhone tests.

Install browser-test dependencies outside the repository, for example:

```powershell
npm install --prefix "$env:TEMP/rc-monitor-ui" playwright
& "$env:TEMP/rc-monitor-ui/node_modules/.bin/playwright.cmd" install webkit
$env:NODE_PATH = "$env:TEMP/rc-monitor-ui/node_modules"
node tests/host/test_monitor_ui.cjs
```

Chrome must also be installed. `PYTHON` can select the Python executable used
to read the existing mock telemetry fixture. Set `MONITOR_SCREENSHOTS` to an
output directory to save browser previews with simulated data.

## Logger lifecycle host checks

`python tests/host/test_telemetry_log.py` compiles the production logger's
file lifecycle and worker with host filesystem I/O and deterministic RTOS
stubs. It covers idle boot/reboot with existing data, explicit start/stop,
append/export, flush failure, clear/recovery, storage guards, and in-flight
row rejection across stop/start/clear. CSV sensor formatting is stubbed.
This does not test ESP32 FAT, physical flash durability, or real scheduling.

Use a native C compiler via `CC`, or set `ZIG` to a Zig executable (the runner
uses `zig cc`). For example, keep the dependency outside the repository:

```powershell
python -m pip install --target "$env:TEMP/rc-monitor-logger" ziglang
$env:ZIG = "$env:TEMP/rc-monitor-logger/ziglang/zig.exe"
python tests/host/test_telemetry_log.py
```

Browser tests also exercise manual recording controls in disarmed/armed
states and ensure they lock on unavailable, full, faulted, or older firmware.
