$ErrorActionPreference = "Stop"

$repository = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$generator = Join-Path $PSScriptRoot "generate_pin_map.py"
$configuration = Join-Path $repository "main\pin_config.h"
$background = Join-Path $PSScriptRoot "esp32_board_minimal.png"
$output = Join-Path $repository "build\esp32_pin_map.svg"
$python = Get-Command python -ErrorAction Stop

& $python.Source $generator `
    --config $configuration `
    --background $background `
    --output $output

if ($LASTEXITCODE -ne 0) {
    throw "Pin-map generation failed."
}

Invoke-Item $output
