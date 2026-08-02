"""Generate a self-contained SVG pin map from main/pin_config.h."""

from __future__ import annotations

import argparse
import base64
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from xml.sax.saxutils import escape


WIDTH = 1680
HEIGHT = 945
LEFT_X = 663
RIGHT_X = 1008
FIRST_ROW_Y = 175
LAST_ROW_Y = 810
ROW_COUNT = 15

# USB-up order. UART and sensor-only pins include their GPIO aliases so the
# silkscreen remains useful when reading pin_config.h.
LEFT_SILKSCREEN = (
    "3V3",
    "GND",
    "D15",
    "D2",
    "D4",
    "RX2/D16",
    "TX2/D17",
    "D5",
    "D18",
    "D19",
    "D21",
    "RX0/D3",
    "TX0/D1",
    "D22",
    "D23",
)
RIGHT_SILKSCREEN = (
    "VIN",
    "GND",
    "D13",
    "D12",
    "D14",
    "D27",
    "D26",
    "D25",
    "D33",
    "D32",
    "D35",
    "D34",
    "VN/D39",
    "VP/D36",
    "EN",
)

PIN_DEFINE = re.compile(
    r"^\s*#define\s+(PIN_[A-Z0-9_]+)\s+GPIO_NUM_([A-Z0-9_]+)\s*(?://.*)?$",
    re.MULTILINE,
)

GPIO_LAYOUT = {
    15: ("left", 2),
    2: ("left", 3),
    4: ("left", 4),
    16: ("left", 5),
    17: ("left", 6),
    5: ("left", 7),
    18: ("left", 8),
    19: ("left", 9),
    21: ("left", 10),
    3: ("left", 11),
    1: ("left", 12),
    22: ("left", 13),
    23: ("left", 14),
    13: ("right", 2),
    12: ("right", 3),
    14: ("right", 4),
    27: ("right", 5),
    26: ("right", 6),
    25: ("right", 7),
    33: ("right", 8),
    32: ("right", 9),
    35: ("right", 10),
    34: ("right", 11),
    39: ("right", 12),
    36: ("right", 13),
}

SIGNAL_METADATA = {
    "PIN_RC_THROTTLE_INPUT": ("Receiver throttle (CH2)", "rc"),
    "PIN_RC_STEERING_INPUT": ("Steering observation (CH1)", "rc"),
    "PIN_RC_ARM_INPUT": ("Shutdown RUN/STOP (CH4)", "rc"),
    "PIN_RC_TV_MODE_INPUT": ("Torque-vectoring mode (CH5)", "rc"),
    "PIN_ESC_FL_THROTTLE": ("Front-left throttle", "throttle"),
    "PIN_ESC_FR_THROTTLE": ("Front-right throttle", "throttle"),
    "PIN_ESC_RL_THROTTLE": ("Rear-left throttle", "throttle"),
    "PIN_ESC_RR_THROTTLE": ("Rear-right throttle", "throttle"),
    "PIN_ESC_FL_REVERSE": ("Front-left reverse", "reverse"),
    "PIN_ESC_FR_REVERSE": ("Front-right reverse", "reverse"),
    "PIN_ESC_RL_REVERSE": ("Rear-left reverse", "reverse"),
    "PIN_ESC_RR_REVERSE": ("Rear-right reverse", "reverse"),
    "PIN_RPM_FL_INPUT": ("Front-left RPM", "rpm"),
    "PIN_RPM_FR_INPUT": ("Front-right RPM", "rpm"),
    "PIN_RPM_RL_INPUT": ("Rear-left RPM", "rpm"),
    "PIN_RPM_RR_INPUT": ("Rear-right RPM", "rpm"),
    "PIN_IMU_I2C_SDA": ("IMU I2C SDA", "imu"),
    "PIN_IMU_I2C_SCL": ("IMU I2C SCL", "imu"),
}

COLORS = {
    "rc": "#087ea4",
    "throttle": "#c84036",
    "reverse": "#cc710c",
    "rpm": "#267849",
    "imu": "#7055a3",
    "other": "#59656b",
}

INPUT_ONLY_GPIOS = {34, 35, 36, 39}
STRAPPING_GPIOS = {0, 2, 5, 12, 15}


class PinMapError(ValueError):
    """Raised when pin_config.h cannot produce a trustworthy map."""


@dataclass(frozen=True)
class Assignment:
    macro: str
    gpio: int | None
    label: str
    category: str


def row_y(row: int) -> float:
    return FIRST_ROW_Y + row * (LAST_ROW_Y - FIRST_ROW_Y) / (ROW_COUNT - 1)


def fallback_label(macro: str) -> str:
    words = macro.removeprefix("PIN_").replace("_", " ").lower()
    return words[:1].upper() + words[1:]


def parse_assignments(config_path: Path) -> list[Assignment]:
    try:
        source = config_path.read_text(encoding="ascii")
    except (OSError, UnicodeError) as error:
        raise PinMapError(f"cannot read {config_path}: {error}") from error

    matches = PIN_DEFINE.findall(source)
    if not matches:
        raise PinMapError(f"no GPIO pin definitions found in {config_path}")

    assignments: list[Assignment] = []
    gpio_owners: dict[int, list[str]] = {}

    for macro, raw_gpio in matches:
        if raw_gpio == "NC":
            gpio = None
        elif raw_gpio.isdigit():
            gpio = int(raw_gpio)
            gpio_owners.setdefault(gpio, []).append(macro)
        else:
            raise PinMapError(f"{macro} has unsupported value GPIO_NUM_{raw_gpio}")

        label, category = SIGNAL_METADATA.get(
            macro,
            (fallback_label(macro), "other"),
        )
        assignments.append(Assignment(macro, gpio, label, category))

    duplicates = {
        gpio: owners for gpio, owners in gpio_owners.items() if len(owners) > 1
    }
    if duplicates:
        details = "; ".join(
            f"GPIO{gpio}: {', '.join(owners)}"
            for gpio, owners in sorted(duplicates.items())
        )
        raise PinMapError(f"duplicate active GPIO assignments: {details}")

    for assignment in assignments:
        if assignment.gpio is None:
            continue
        if assignment.gpio not in GPIO_LAYOUT:
            raise PinMapError(
                f"{assignment.macro} uses GPIO{assignment.gpio}, which is not "
                "available on this DOIT DevKit header map"
            )
        if (
            assignment.macro.startswith("PIN_ESC_")
            and assignment.gpio in INPUT_ONLY_GPIOS
        ):
            raise PinMapError(
                f"{assignment.macro} uses input-only GPIO{assignment.gpio}"
            )

    return assignments


def embedded_png(background_path: Path) -> str:
    try:
        image = background_path.read_bytes()
    except OSError as error:
        raise PinMapError(f"cannot read {background_path}: {error}") from error
    if not image.startswith(b"\x89PNG\r\n\x1a\n"):
        raise PinMapError(f"{background_path} is not a PNG image")
    return base64.b64encode(image).decode("ascii")


def signal_svg(assignment: Assignment) -> str:
    assert assignment.gpio is not None
    side, row = GPIO_LAYOUT[assignment.gpio]
    x = LEFT_X if side == "left" else RIGHT_X
    y = row_y(row)
    color = COLORS[assignment.category]
    gpio_text = f"GPIO{assignment.gpio}"
    label = escape(assignment.label)
    macro = escape(assignment.macro)

    if side == "left":
        line = f'<line x1="{x - 12}" y1="{y:.1f}" x2="548" y2="{y:.1f}"/>'
        text = (
            f'<text x="528" y="{y + 5:.1f}" text-anchor="end">'
            f'<tspan class="signal">{label}</tspan>'
            f'<tspan class="gpio" dx="9">{gpio_text}</tspan></text>'
        )
    else:
        line = f'<line x1="{x + 12}" y1="{y:.1f}" x2="1132" y2="{y:.1f}"/>'
        text = (
            f'<text x="1152" y="{y + 5:.1f}" text-anchor="start">'
            f'<tspan class="gpio">{gpio_text}</tspan>'
            f'<tspan class="signal" dx="9">{label}</tspan></text>'
        )

    return (
        f'<g class="callout {assignment.category}" data-macro="{macro}" '
        f'style="--signal-color:{color}">'
        f"{line}"
        f'<circle class="marker-ring" cx="{x}" cy="{y:.1f}" r="10"/>'
        f'<circle class="marker" cx="{x}" cy="{y:.1f}" r="6"/>'
        f"{text}</g>"
    )


def generate_svg(
    assignments: list[Assignment], background_data: str, config_label: str
) -> str:
    if len(LEFT_SILKSCREEN) != ROW_COUNT or len(RIGHT_SILKSCREEN) != ROW_COUNT:
        raise PinMapError("silkscreen labels must match the 15-row headers")

    active = [assignment for assignment in assignments if assignment.gpio is not None]
    strapping = sorted(
        assignment.gpio
        for assignment in active
        if assignment.gpio in STRAPPING_GPIOS
    )

    holes = []
    for x in (LEFT_X, RIGHT_X):
        for row in range(ROW_COUNT):
            holes.append(
                f'<circle class="header-hole" cx="{x}" cy="{row_y(row):.1f}" r="10"/>'
            )

    silkscreen = []
    for row, label in enumerate(LEFT_SILKSCREEN):
        silkscreen.append(
            f'<text class="silkscreen" x="688" y="{row_y(row) + 4:.1f}" '
            f'text-anchor="start">{label}</text>'
        )
    for row, label in enumerate(RIGHT_SILKSCREEN):
        silkscreen.append(
            f'<text class="silkscreen" x="983" y="{row_y(row) + 4:.1f}" '
            f'text-anchor="end">{label}</text>'
        )

    callouts = [signal_svg(assignment) for assignment in active]
    strap_note = ""
    if strapping:
        pins = ", ".join(f"GPIO{gpio}" for gpio in strapping)
        strap_note = f'<text class="strap-note" x="1625" y="913" text-anchor="end">Boot-strapping: {pins}</text>'

    return f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}" role="img" aria-labelledby="title description">
  <title id="title">ESP32 powertrain pin map</title>
  <desc id="description">USB-up ESP32 DOIT DevKit V1 with pin assignments generated from {escape(config_label)}.</desc>
  <style>
    text {{ font-family: "Segoe UI", Arial, sans-serif; letter-spacing: 0; }}
    .title {{ fill: #182126; font-size: 27px; font-weight: 680; }}
    .subtitle, .footer {{ fill: #6b777d; font-size: 13px; font-weight: 600; }}
    .legend {{ fill: #46545a; font-size: 12px; font-weight: 650; }}
    .rail {{ fill: #222323; }}
    .header-hole {{ fill: #f4f2e9; stroke: #bca36d; stroke-width: 5; }}
    .silkscreen {{
      fill: #fff;
      stroke: #171919;
      stroke-width: 4px;
      stroke-linejoin: round;
      paint-order: stroke fill;
      font-family: Consolas, monospace;
      font-size: 13px;
      font-weight: 800;
    }}
    .callout line {{ stroke: var(--signal-color); stroke-width: 1.5; }}
    .marker-ring {{ fill: #fff; stroke: var(--signal-color); stroke-width: 3; }}
    .marker {{ fill: var(--signal-color); }}
    .signal {{ fill: #182126; font-size: 15px; font-weight: 650; }}
    .gpio {{ fill: var(--signal-color); font-family: Consolas, monospace; font-size: 14px; font-weight: 800; }}
    .strap-note {{ fill: #8a5a17; font-size: 13px; font-weight: 700; }}
  </style>
  <rect width="1680" height="945" fill="#fff"/>
  <image width="1680" height="945" href="data:image/png;base64,{background_data}"/>

  <text class="title" x="48" y="48">ESP32 powertrain pin map</text>
  <text class="subtitle" x="48" y="72">USB-up | generated from {escape(config_label)}</text>

  <g class="legend">
    <circle cx="54" cy="106" r="5" fill="{COLORS['rc']}"/><text x="67" y="110">Receiver</text>
    <circle cx="163" cy="106" r="5" fill="{COLORS['throttle']}"/><text x="176" y="110">ESC throttle</text>
    <circle cx="292" cy="106" r="5" fill="{COLORS['reverse']}"/><text x="305" y="110">ESC reverse</text>
    <circle cx="411" cy="106" r="5" fill="{COLORS['rpm']}"/><text x="424" y="110">RPM</text>
    <circle cx="476" cy="106" r="5" fill="{COLORS['imu']}"/><text x="489" y="110">IMU</text>
  </g>

  <rect class="rail" x="643" y="143" width="40" height="699"/>
  <rect class="rail" x="988" y="143" width="40" height="699"/>
  {''.join(holes)}
  {''.join(silkscreen)}
  {''.join(callouts)}

  <line x1="48" y1="886" x2="1632" y2="886" stroke="#d7dddf"/>
  <text class="footer" x="48" y="913">{len(active)} active signals | IMU interrupts not connected</text>
  {strap_note}
</svg>
'''


def write_if_changed(output_path: Path, content: str) -> None:
    encoded = content.encode("utf-8")
    try:
        if output_path.exists() and output_path.read_bytes() == encoded:
            return
        output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = output_path.with_suffix(output_path.suffix + ".tmp")
        temporary.write_bytes(encoded)
        temporary.replace(output_path)
    except OSError as error:
        raise PinMapError(f"cannot write {output_path}: {error}") from error


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--background", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    try:
        assignments = parse_assignments(args.config)
        background_data = embedded_png(args.background)
        config_label = f"{args.config.parent.name}/{args.config.name}"
        svg = generate_svg(assignments, background_data, config_label)
        write_if_changed(args.output, svg)
    except PinMapError as error:
        print(f"pin map error: {error}", file=sys.stderr)
        return 1

    active_count = sum(assignment.gpio is not None for assignment in assignments)
    print(f"Generated {args.output} with {active_count} active signals")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
