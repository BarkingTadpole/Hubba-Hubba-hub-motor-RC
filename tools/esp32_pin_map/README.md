# ESP32 Pin Map

This tool generates a self-contained SVG wiring reference from
`main/pin_config.h`. It uses no server and does not modify or become part of
the ESP32 firmware binary.

The normal ESP-IDF build regenerates the map whenever the pin configuration,
generator, or board artwork changes:

```powershell
idf.py build
```

The output is `build/esp32_pin_map.svg`.

To regenerate and open the map without building the firmware:

```powershell
.\tools\esp32_pin_map\open_pin_map.ps1
```

Generation fails safely when it finds duplicate active assignments, an ESC
output on an input-only GPIO, or a GPIO that is not present on the mapped DOIT
DevKit header. GPIO strapping assignments are shown as notices in the SVG.
