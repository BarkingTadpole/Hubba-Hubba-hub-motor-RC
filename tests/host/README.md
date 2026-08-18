# Torque-vectoring host logic test

The tests are freestanding so Espressif's bundled RV32 Clang can build them
without linking ESP-IDF. Failed checks execute a trap; a normal return from
`main` means every check passed. The runner requires the `pyelftools` and
`unicorn` Python packages.

From an ESP-IDF 6.x PowerShell environment:

```powershell
New-Item -ItemType Directory -Force build\host_tests | Out-Null
clang -std=c11 -Wall -Wextra -Werror -I main\include `
  main\torque_vectoring.c tests\host\test_torque_vectoring.c `
  -o build\host_tests\test_torque_vectoring.elf
python tests\host\run_rv32_unicorn.py `
  build\host_tests\test_torque_vectoring.elf

clang -std=c11 -Wall -Wextra -Werror -I main\include `
  main\steering_curve.c tests\host\test_steering_curve.c `
  -o build\host_tests\test_steering_curve.elf
python tests\host\run_rv32_unicorn.py `
  build\host_tests\test_steering_curve.elf

clang -std=c11 -Wall -Wextra -Werror -I main\include `
  main\steering_input_filter.c tests\host\test_steering_input_filter.c `
  -o build\host_tests\test_steering_input_filter.elf
python tests\host\run_rv32_unicorn.py `
  build\host_tests\test_steering_input_filter.elf
```

Keep Python dependencies outside the source tree or under ignored `build/`.
These tests cover pure control, interpolation, and full-range steering-filter
logic only; they do not exercise ESP32 peripherals, FreeRTOS scheduling,
receiver safety behavior, or the car.
