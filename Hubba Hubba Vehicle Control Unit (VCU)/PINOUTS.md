# Hubba Hubba VCU Pinouts

Revision: initial consolidated pinout register, 2026-09-03.

This document is the working pin and net-assignment reference for the Hubba Hubba VCU. It is not yet a released wiring diagram. The ESP32 assignments below are provisional until schematic review, boot-state review, ERC/DRC, and an ESP-IDF target build pass. Physical connector part numbers and contact numbering must be added before fabrication; do not infer connector order from the order of rows in a table.

## Current signal-routing progress

| Device | Signal assignment status | Work intentionally deferred |
|---|---|---|
| ASM330LHHTR IMU | Complete at the logical and package-pin level: SPI, chip select and INT1 are assigned | Power-rail implementation, decoupling part selection/check, placement and final layout review |
| microSD | Complete at the logical and socket-contact level: SPI, chip select, card detect and six-channel signal ESD are assigned | Power-rail implementation, decoupling and final socket/ESD layout review |
| MAX-M10S-00B GNSS | Complete at the logical and package-pin level: UART and TIMEPULSE are assigned; unused digital pins are defined | Power-rail implementation, antenna choice, active/passive bias decision, and final 50-ohm RF geometry from the selected PCB stack-up |

“Complete” here means the intended signal mapping is documented. It does not mean the schematic, power design, controlled-impedance layout, ERC/DRC, firmware pin audit, or physical validation has passed.

## Status legend

- **Assigned**: selected for the present design and may be used as the schematic starting point.
- **Provisional**: selected, but still requires schematic, electrical, timing, or firmware validation.
- **TBD**: no pin or contact may be assigned by assumption.
- **Internal**: contained inside the STSPIN32G4 SiP and not available as a PCB net.

## 1. ESP32-S3 supervisor GPIO map

Target module: `ESP32-S3-WROOM-1U-N16R8`.

| GPIO | Net / function | Direction at ESP32 | Status | Connection and notes |
|---:|---|---|---|---|
| 0 | `ESP_BOOT` | Input | Assigned | Boot button/test pad only. Apply the Espressif-recommended default pull; no operational load. |
| 1 | `GNSS_UART_RX` | Input | Provisional | Connects to MAX-M10S `TXD`. |
| 2 | `GNSS_UART_TX` | Output | Provisional | Connects to MAX-M10S `RXD`. |
| 3 | Reserved strap | Input at boot | Assigned reserved | Leave free of functional loads. |
| 4 | `RC_CH1_STEERING` | Input | Provisional | R7FG CH1 through 5 V-tolerant conditioning and connector ESD. |
| 5 | `RC_CH2_THROTTLE` | Input | Provisional | R7FG CH2 through 5 V-tolerant conditioning and connector ESD. |
| 6 | `RC_CH4_RUN_STOP` | Input | Provisional | R7FG CH4 through 5 V-tolerant conditioning and connector ESD. |
| 7 | `RC_CH5_TV_MODE` | Input | Provisional | R7FG CH5 through 5 V-tolerant conditioning and connector ESD. |
| 8 | `STEERING_SERVO_PWM` | Output | Provisional | Through a 5 V AHCT buffer and dedicated low-capacitance ESD. Servo power is external. |
| 9 | `SPI_SCLK` | Output | Provisional | Shared by ASM330LHH and microSD. |
| 10 | `SPI_MOSI` | Output | Provisional | Shared by ASM330LHH and microSD. |
| 11 | `SPI_MISO` | Input | Provisional | Shared by ASM330LHH and microSD. |
| 12 | `IMU_CS_N` | Output | Provisional | ASM330LHH chip select; provide a passive inactive-high default. |
| 13 | `IMU_INT1` | Input | Provisional | ASM330LHH interrupt 1. |
| 14 | `SD_CS_N` | Output | Provisional | microSD SPI chip select; pull up to 3.3 V so the card is deselected during reset. |
| 15 | `SD_DETECT_N` | Input | Provisional | Socket detect switch; pull up to 3.3 V, with an inserted card closing the switch to ground if supported by the selected socket. Confirm actual switch polarity. |
| 16 | `GNSS_TIMEPULSE` | Input | Provisional | MAX-M10S `TIMEPULSE`. Keep high impedance with no pull during GNSS startup because the module pad is shared internally with `SAFEBOOT_N`. |
| 17 | `SUP_I2C_SDA` | Bidirectional open-drain | Provisional | Internal board-only bus to TCA9548A and TCA9534A. |
| 18 | `SUP_I2C_SCL` | Output/open-drain | Provisional | Internal board-only bus to TCA9548A and TCA9534A. |
| 19 | `USB_D_N` | Bidirectional | Assigned by device | Native USB D-. Route as a matched differential pair with USB ESD. |
| 20 | `USB_D_P` | Bidirectional | Assigned by device | Native USB D+. Route as a matched differential pair with USB ESD. |
| 21 | `SUPERVISOR_WDI` | Output | Provisional | Windowed proved-alive heartbeat into the supervisor/reset safety circuit. This is not a static enable and not torque permission. Timing and polarity are TBD from the final circuit. |
| 22-34 | Unallocated | TBD | TBD | Do not use until the final peripheral, strap, boot, and timing audit. |
| 35-37 | Not available | - | Assigned unavailable | Occupied internally by octal PSRAM on the N16R8 module. Do not route. |
| 38 | `MOTOR_CMD_FL` | Output | Provisional | Center-neutral signed current/torque request to the front-left STSPIN32G4 node. Exact carrier and duty contract are TBD. |
| 39 | `MOTOR_CMD_FR` | Output | Provisional | Center-neutral signed request to the front-right node. |
| 40 | `MOTOR_CMD_RL` | Output | Provisional | Center-neutral signed request to the rear-left node. |
| 41 | `MOTOR_CMD_RR` | Output | Provisional | Center-neutral signed request to the rear-right node. |
| 42 | `MOTOR_FG_FL` | Input | Provisional | Buffered, open-drain, observer-derived 7-PPR speed magnitude from the front-left node. |
| 43 | `MOTOR_FG_FR` | Input | Provisional | Buffered 7-PPR speed magnitude from the front-right node. |
| 44 | `MOTOR_FG_RL` | Input | Provisional | Buffered 7-PPR speed magnitude from the rear-left node. |
| 45 | Reserved strap | Input at boot | Assigned reserved | Leave free of functional loads. |
| 46 | Reserved strap | Input at boot | Assigned reserved | Leave free of functional loads. |
| 47 | `MOTOR_FG_RR` | Input | Provisional | Buffered 7-PPR speed magnitude from the rear-right node. |
| 48 | `I2C_MUX_RESET_N` | Output | Provisional | TCA9548A active-low reset. Use a passive pull-up; firmware asserts it during initialization and stuck-bus recovery. |

The current plan has no spare direct ESP32 aggregate-fault interrupt. The hardware fault latch must remove drive independently and expose noncritical status through the low-speed supervisory path. Any decision to allocate another direct GPIO requires an updated boot/timing review.

## 2. Shared SPI bus

| Signal | ESP32 | ASM330LHH | microSD socket | Notes |
|---|---:|---|---|---|
| Clock | GPIO9 | `SPC` | Pin 5 `CLK` | Route clock carefully; add source termination only after signal-integrity review. |
| Controller-to-device data | GPIO10 | `SDI` | Pin 3 `CMD` / MOSI | Shared output from ESP32. |
| Device-to-controller data | GPIO11 | `SDO` | Pin 7 `DAT0` / MISO | Shared input; each device must release it while deselected. |
| IMU select | GPIO12 | `CS` | - | Pull inactive high. |
| microSD select | GPIO14 | - | Pin 2 `DAT3` / CS | Pull inactive high. |
| IMU interrupt | GPIO13 | `INT1` | - | `INT2` is presently TBD/unconnected. |
| Card detect | GPIO15 | - | Socket detect switch | This is a mechanical switch contact, not a microSD card contact. |

The IMU and card may share SCLK/MOSI/MISO but never share chip select. IMU acquisition has higher scheduling priority than logging.

## 3. microSD card and ESD mapping

The selected card interface is SPI mode. TPD6E05U06 protection channels are independent single-ended shunt channels; the `+` and `-` suffixes in TI's channel names do not mean that the microSD signals form differential pairs.

| microSD contact | SD name | SPI use | ESP32 / rail | TPD6E05U06 RVZ pin | Status |
|---:|---|---|---|---:|---|
| 1 | `DAT2` | Unused in SPI mode | No GPIO; keep per SD guidance | 14 (`D1+`) | Provisional protected spare data contact |
| 2 | `DAT3` | `CS_N` | GPIO14 | 13 (`D1-`) | Provisional |
| 3 | `CMD` | MOSI | GPIO10 | 12 (`D2+`) | Provisional |
| 4 | `VDD` | Card supply | Regulated 3.3 V with local decoupling | None | Assigned rail; do not route through a signal ESD channel |
| 5 | `CLK` | SPI clock | GPIO9 | 11 (`D2-`) | Provisional |
| 6 | `VSS` | Ground | Ground | None | Assigned ground |
| 7 | `DAT0` | MISO | GPIO11 | 9 (`D3+`) | Provisional |
| 8 | `DAT1` | Unused in SPI mode | No GPIO; keep per SD guidance | 8 (`D3-`) | Provisional protected spare data contact |

TPD6E05U06 RVZ ground pins 5 and 10 connect directly to the ground plane with the shortest practical path and nearby vias. Pins 1, 2, 3, 4, 6, and 7 are `NC`; leave them unconnected unless the final TI land-pattern guidance explicitly permits a grounded pad strategy. Place the array beside the socket so a discharge reaches the protection ground before the traces enter the board.

The selected socket's shell/shield and mechanical mounting tabs connect to the chassis/PCB shielding strategy defined by the final EMC review. Do not assume they are card `VSS` contacts. The load switch remains documented in the current design; its TCA9534A control bit is still TBD.

## 4. ASM330LHHTR IMU physical pinout

Package: LGA-14L, 2.5 mm x 3.0 mm. The manufacturer pin drawing is a **bottom view**; verify pin-1 orientation against the top-side package mark and the EasyEDA footprint before placement. The VCU uses four-wire SPI at 3.3 V.

| IMU pin | Pin name | VCU connection | Status and notes |
|---:|---|---|---|
| 1 | `SDO/SA0` | ESP32 GPIO11 `SPI_MISO` | SPI serial-data output. `SA0` applies only in I2C mode. |
| 2 | `RES` | Ground | ST allows connection to either VDDIO or ground; use ground consistently and do not leave floating. |
| 3 | `RES` | Ground | ST allows connection to either VDDIO or ground; use ground consistently and do not leave floating. |
| 4 | `INT1` | ESP32 GPIO13 `IMU_INT1` | Data-ready interrupt. The ESP32 pin must remain high impedance during IMU power-on; ST requires INT1 low or unconnected during power-on. Do not add a pull-up. |
| 5 | `VDDIO` | Low-noise 3.3 V IMU rail | Place a 100 nF ceramic bypass capacitor from this pin to ground close to the package. |
| 6 | `GND` | Ground plane | Short return to the local IMU ground region. |
| 7 | `GND` | Ground plane | Short return to the local IMU ground region. |
| 8 | `VDD` | Low-noise 3.3 V IMU rail | Place 100 nF ceramic plus 10 uF bulk decoupling from this pin/rail to ground close to the package. |
| 9 | `INT2/DEN` | No connection | Leave unconnected unless a later reviewed feature needs INT2 or DEN. |
| 10 | `NC` | No connection | Must remain unconnected. Do not create a test pad. |
| 11 | `NC` | No connection | Must remain unconnected. Do not create a test pad. |
| 12 | `CS` | ESP32 GPIO12 `IMU_CS_N` | Active-low SPI chip select. Pull up to VDDIO so the device remains deselected through ESP32 reset. |
| 13 | `SCL/SPC` | ESP32 GPIO9 `SPI_SCLK` | SPI clock input. |
| 14 | `SDA/SDI/SDO` | ESP32 GPIO10 `SPI_MOSI` | SPI serial-data input in four-wire mode. |

Both `VDD` and `VDDIO` operate from the selected low-noise 3.3 V rail; the device's allowed operating-supply range is 2.0-3.6 V. Keep the decoupling loop small, keep the IMU and its SPI traces away from all switch nodes, inductors, motor phases, high-current copper, and the antenna feed, and provide an unambiguous PCB axis/orientation mark. The retained vehicle convention is sensor `+X` toward the rear and sensor `+Y` toward the right.

## 5. MAX-M10S-00B GNSS physical pinout

Package: 18-pin LCC, approximately 9.7 mm x 10.1 mm. The u-blox assignment drawing is a **top view**. The VCU uses the 3.3 V supply option, UART for configuration/data, and TIMEPULSE for precise timing. The initial antenna path assumes a passive GNSS antenna connected through U.FL; any change to an active antenna requires the bias network to be reviewed.

| GNSS pin | Pin name | VCU connection | Status and notes |
|---:|---|---|---|
| 1 | `GND` | Ground plane | Use a short ground connection with stitching vias. |
| 2 | `TXD` | ESP32 GPIO1 `GNSS_UART_RX` | GNSS UART output to ESP32 input. Default UART is 9600 baud, 8 data bits, no parity, one stop bit. |
| 3 | `RXD` | ESP32 GPIO2 `GNSS_UART_TX` | GNSS UART input from ESP32 output. |
| 4 | `TIMEPULSE` | ESP32 GPIO16 `GNSS_TIMEPULSE` | Timing output. ESP32 GPIO16 must be high impedance with no pull during GNSS startup because this signal is internally connected to pin 18 through 1 kOhm. |
| 5 | `EXTINT` | No connection | Leave open unless a future reviewed external time-mark/wakeup feature needs it. |
| 6 | `V_BCKP` | Optional backup supply; otherwise no connection | u-blox permits this pin to be left open when no backup supply is fitted. A later supercapacitor/cell option must remain within 1.65-3.6 V and requires a charging/isolation review. |
| 7 | `V_IO` | Low-noise 3.3 V GNSS rail | Required I/O supply; do not leave floating. Add close local bypassing. |
| 8 | `VCC` | Low-noise 3.3 V GNSS rail | Main supply. Add close local bypassing and sufficient local bulk capacitance; the source must tolerate up to 100 mA startup inrush. |
| 9 | `RESET_N` | No connection or protected test pad | Active-low hardware reset; internal pull-up is provided. If controlled later, hold low for at least 1 ms and never drive it while the GNSS I/O supply is absent. |
| 10 | `GND` | RF ground plane | Place ground vias close to the module and antenna feed. |
| 11 | `RF_IN` | 50-ohm matching/ESD path to U.FL center contact | Internally DC blocked. Keep the trace short, controlled impedance, and isolated from ESP32 RF, clocks, switch nodes, motor phases, and power inductors. |
| 12 | `GND` | RF ground plane | Ground closely beside `RF_IN` with stitching vias. |
| 13 | `LNA_EN` | No connection for the initial passive-antenna build | Output for controlling an external LNA or active antenna. Do not repurpose as a GPIO. |
| 14 | `VCC_RF` | No connection for the initial passive-antenna build | RF-section supply output. An active-antenna revision may use it only through the u-blox bias/reference network; do not connect it directly to `RF_IN`. |
| 15 | `VIO_SEL` | No connection | **Leave open for 3.3 V V_IO operation.** Connecting it to ground selects the 1.8 V I/O mode. |
| 16 | `SDA` | No connection | I2C is not used. Leave open. |
| 17 | `SCL` | No connection | I2C is not used. Leave open. |
| 18 | `SAFEBOOT_N` | No connection | Active-low safe-boot input. Leave open. It is internally connected to pin 4 `TIMEPULSE` through 1 kOhm, which is why GPIO16 must not pull the line low during startup. |

Place the selected very-low-capacitance RF ESD device directly beside the U.FL connector, with an extremely short ground return. Its capacitance and footprint are part of the 50-ohm RF model. The U.FL shell contacts connect to the RF ground plane; the center contact connects only to the protected/matched `RF_IN` trace.

For the default 3.3 V design, connect pins 7 `V_IO` and 8 `VCC` to the clean GNSS 3.3 V rail and leave pin 15 `VIO_SEL` open. The exact bypass and bulk capacitor values remain governed by the released power schematic, but both supply pins need close high-frequency decoupling and the rail must remain stable during the specified startup inrush.

## 6. Receiver and steering interfaces

Physical connector contact order is TBD. Label every contact by function on the PCB silkscreen and never depend on cable color alone.

| External function | VCU signal | ESP32 | Required interface |
|---|---|---:|---|
| R7FG CH1 steering | `RC_CH1_STEERING` | GPIO4 | 5 V-tolerant input buffer/conditioning plus ESD |
| R7FG CH2 throttle | `RC_CH2_THROTTLE` | GPIO5 | 5 V-tolerant input buffer/conditioning plus ESD |
| R7FG CH4 run/stop | `RC_CH4_RUN_STOP` | GPIO6 | 5 V-tolerant input buffer/conditioning plus ESD |
| R7FG CH5 TV mode | `RC_CH5_TV_MODE` | GPIO7 | 5 V-tolerant input buffer/conditioning plus ESD |
| Receiver reference | `RC_GND` | Ground | Common signal reference |
| Optional receiver supply | `RC_PWR` | External regulated rail | Voltage and current are TBD; never source from an ESP32 GPIO |
| Steering command | `STEERING_SERVO_PWM_5V` | GPIO8 through AHCT buffer | Protected signal only |
| Steering reference | `SERVO_GND` | Ground | Common signal reference |
| Steering power | `SERVO_PWR_EXT` | External supply | Must not come from the ESP32 3.3 V rail or a GPIO |

## 7. Internal supervisory I2C routing

| Device / segment | SDA/SCL route | Mux channel | Status / notes |
|---|---|---:|---|
| TCA9548A upstream | ESP32 GPIO17/GPIO18 | Upstream | Provisional; internal board-only bus |
| TCA9534A | ESP32 GPIO17/GPIO18 | Upstream | Provisional; slow noncritical I/O only |
| Front-left node diagnostic I2C2 + FL INA238 | TCA9548A downstream | 0 | Node uses exposed `PA8/SDA`, `PA9/SCL`; node and INA238 addresses must differ |
| Front-right node diagnostic I2C2 + FR INA238 | TCA9548A downstream | 1 | Same address rule |
| Rear-left node diagnostic I2C2 + RL INA238 | TCA9548A downstream | 2 | Same address rule |
| Rear-right node diagnostic I2C2 + RR INA238 | TCA9548A downstream | 3 | Same address rule |
| Thermal ADS1115 A | TCA9548A downstream | 4 | Four single-ended NTC channels |
| Thermal ADS1115 B | TCA9548A downstream | 5 | Four single-ended NTC channels |
| Reserved | Not routed off-board | 6 | Keep deselected |
| Reserved | Not routed off-board | 7 | Keep deselected |

TCA9548A `RESET_N` connects to ESP32 GPIO48 and a passive pull-up. The exact TCA9548A, TCA9534A, node-slave, INA238, and ADS1115 addresses and strap pins are TBD and must be recorded here before code freeze.

## 8. Repeated motor-node logical pin contract

There are four identical logical instances: `FL`, `FR`, `RL`, and `RR`. This table defines their PCB boundary, not the final STM32 alternate-function allocation.

| Node net | Source / destination | Direction at motor node | Status / notes |
|---|---|---|---|
| `MOTOR_CMD_xx` | ESP32 GPIO38/39/40/41 by wheel | Input | Signed center-neutral PWM request; exact timer-capable MCU pin is TBD |
| `MOTOR_FG_xx_RAW` | Local MCU to central powered open-drain buffer | Output | Observer-derived magnitude; exactly 7 rising edges/mechanical revolution while valid; exact MCU pin TBD |
| `NODE_I2C2_SDA` | TCA9548A channel | Bidirectional | Exposed STSPIN32G4 STM32 `PA8` |
| `NODE_I2C2_SCL` | TCA9548A channel | Bidirectional | Exposed STSPIN32G4 STM32 `PA9` |
| `NODE_WDI` | Local MCU to that node's external window watchdog | Output | Proved-alive heartbeat; exact MCU pin/timing TBD |
| `NODE_NRST` | Fail-low reset/fault-latch network and SWD pad | Input | Must remain asserted through unsafe power/reset states |
| `SWDIO` | Programming pad | Bidirectional | Dedicated pad group per node; exact package pin must be verified from ST data |
| `SWCLK` | Programming pad | Input | Dedicated pad group per node |
| `VTREF_3V3` | Programming pad | Power reference | Reference only; programming-power policy TBD |
| `GND` | Programming pad and node ground | Power | Keep programmer isolated from traction power as required |
| `PHASE_U/V/W` | MOSFET bridge to motor connector | Power bidirectional | Physical connector contact numbering TBD; never infer phase order from wire color |
| `VM_xx` / `PGND_xx` | Fused branch DC link | Power | One isolated branch domain per node; high-current connector/busbar definition TBD |
| Three phase-shunt sense pairs | Kelvin shunts to local analog front end | Analog input | Exact OPAMP/ADC/comparator pin mapping and polarity TBD |
| Branch INA238 shunt inputs | High-side branch shunt | Analog input to INA238 | Separate from all phase-shunt Kelvin nets |
| Two NTC nodes | MOSFET and terminal/shunt hot spots | Analog to ADS1115 | Exact ADC channel allocation TBD |

`READY` on `PE14`, `nFAULT` on `PE15`, and gate-driver I2C3 on `PC8/PC9` are **internal SiP connections**. They must appear in motor-node firmware configuration, but must not be drawn as external STSPIN32G4 package pins or connected directly to the ESP32, mux, or board-level fault latch.

The final STSPIN32G4 pin table must also record all six gate outputs, three switch/phase sense inputs, bootstrap connections, supply pins, regulator components, OPAMP/ADC inputs, comparator paths, TIM1 channels/break inputs, BOOT behavior, and every no-connect. Copy physical pin numbers only after auditing the current ST data sheet and EasyEDA symbol pin-for-pin.

## 9. Per-node SWD service pad order

Provide four separately labeled pad groups. The required signals are fixed, but the physical pad order is TBD:

| Required pad | Signal | Notes |
|---|---|---|
| TBD | `VTREF_3V3` | Programmer voltage reference; do not accidentally back-power an unpowered traction domain |
| TBD | `SWDIO` | Dedicated to one node only |
| TBD | `SWCLK` | Dedicated to one node only |
| TBD | `NODE_NRST` | Must not defeat the fail-low safety network |
| TBD | `GND` | Local node reference |

Do not hard-parallel the four SWD targets. Programming and recovery occur with traction power isolated and all gates inhibited.

## 10. USB-C logical pinout

| USB-C receptacle function | VCU connection | Notes |
|---|---|---|
| A6/B6 `D+` | ESP32 GPIO20 through USB ESD | Join the receptacle's duplicated USB 2.0 contacts per USB-C guidance |
| A7/B7 `D-` | ESP32 GPIO19 through USB ESD | Route D+/D- as a matched pair |
| A5 `CC1` | 5.1 kOhm `Rd` to ground | Device/UFP indication |
| B5 `CC2` | 5.1 kOhm `Rd` to ground | Device/UFP indication |
| VBUS contacts | Protected 5 V USB input and source-isolation path | Must not energize an inverter or backfeed traction power |
| Ground contacts | Ground | Provide short ESD return and shield strategy |
| SuperSpeed contacts | Not connected | USB 2.0-only design |

Verify the exact receptacle symbol pin numbering against its manufacturer drawing; contact designators above are USB-C standard names, not a substitute for that audit.

## 11. Unassigned pinout work before schematic release

- Freeze every connector manufacturer part number, orientation, contact numbering, mating cable, keying, current rating, and silkscreen label.
- Audit all ESP32 boot straps, reset defaults, pulls, peripheral matrix choices, and input-only/output requirements.
- Audit the completed MAX-M10S and ASM330LHH physical pin tables against the exact EasyEDA symbols and footprints, including view orientation and pin-1 marks.
- Complete the 64-pin STSPIN32G4 package audit and the node TIM1, ADC, OPAMP, comparator, signed-command capture, FG, watchdog, SWD, and identity-strap assignments.
- Freeze supervisory-I2C addresses, address straps, pull-ups, voltage domains, mux reset behavior, and the TCA9534A bit allocation.
- Freeze motor, traction, logic-power, receiver, servo, antenna, E-stop, contactor, and programming connector pinouts.
- Record ESD-array channel mapping and placement for every external signal connector, not only microSD.
- Update this document, the schematic, `HARDWARE_ARCHITECTURE.md`, `CODE_HANDOFF.md`, and the `BOM.xlsx` Architecture/Release Gates sheets together when an assignment changes.
