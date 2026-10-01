# Hubba Hubba VCU Datasheets and Procurement Links

Checked 2026-09-03. LCSC inventory changes continuously; these links establish catalog identity, not future availability or JLCPCB assembly eligibility. Match the exact suffix, package, voltage, tolerance and EasyEDA footprint in `BOM.xlsx` before ordering.

## Motor, battery and engineering references

| Item | Primary documents |
|---|---|
| Hobbywing Skywalker 2820SL 550KV | [Official motor page](https://www.hobbywing.com/en/products/skywalker2814.html) · [2820 load chart](https://www.hobbywing.com/en/uploads/file/20231121/8d16fd52280806a44638a568c6b58985.pdf) · [mechanical drawing](https://www.hobbywing.com/en/uploads/file/20231121/6ce36297af7f04e8e0c41c3b28a36dbd.pdf) |
| Commercial 3–6S fallback/benchmark | [Hobbywing Skywalker V2 series](https://www.hobbywing.com/en/products/skywalker-v2-series) · [current official manual](https://www.hobbywing.com/en/uploads/file/20250930/64b726be7a56c9f415385f77683cdc46.pdf) |
| Existing aircraft-ESC behavior | Repository `Skywalker_ESC_Manual.pdf` |
| Braking and DC-bus risks | [TI SLVAF66](https://www.ti.com/lit/an/slvaf66/slvaf66.pdf) |
| MOSFET gate-drive switching | [TI SLUA618A](https://www.ti.com/lit/an/slua618a/slua618a.pdf) |

The Hobbywing motor's published 40.9 A/46 s and 910.2 W/46 s data are finite propeller-test values. They are not a continuous vehicle rating.

## Supervisor, navigation, IMU and storage

| Exact part | LCSC | Manufacturer documents |
|---|---|---|
| ESP32-S3-WROOM-1U-N16R8 | [C3013946](https://www.lcsc.com/product-detail/C3013946.html) | [Module data sheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) · [hardware guidelines](https://www.espressif.com/sites/default/files/documentation/esp32-s3_hardware_design_guidelines_en.pdf) |
| MAX-M10S-00B | [C4153167](https://www.lcsc.com/product-detail/C4153167.html) | [Data sheet](https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf) · [integration manual](https://content.u-blox.com/sites/default/files/MAX-M10S_IntegrationManual_UBX-20053088.pdf) · [interface description](https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf) |
| ASM330LHHTR | [C459826](https://www.lcsc.com/product-detail/C459826.html) | [Data sheet](https://www.st.com/resource/en/datasheet/asm330lhh.pdf) · [AN5259](https://www.st.com/resource/en/application_note/an5259-asm330lhh-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf) |
| Molex 475710001 microSD socket | [C19807215](https://www.lcsc.com/product-detail/C19807215.html) | [Product/drawing](https://www.molex.com/en-us/products/part-detail/475710001) |
| TPS22918DBVR microSD load switch | [C131941](https://www.lcsc.com/product-detail/C131941.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/tps22918.pdf) |
| TPD6E05U06RVZR microSD ESD | [C962978](https://www.lcsc.com/product-detail/C962978.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/tpd6e05u06.pdf) |

The ESP32-S3-WROOM-1U-N16R8 R8 module is normally specified for -40 °C to +65 °C ambient. Espressif documents a maximum-ambient increase to +85 °C when PSRAM ECC is enabled, with usable PSRAM reduced by one-sixteenth. That option is not automatic: record the exact build configuration, memory budget and thermal qualification if it is used.

## Programmable ST motor control

| Exact part | LCSC | Manufacturer documents |
|---|---|---|
| STSPIN32G4 | [C7434864](https://www.lcsc.com/product-detail/C7434864.html) | [Product page](https://www.st.com/en/motor-drivers/stspin32g4.html) · [data sheet](https://www.st.com/resource/en/datasheet/stspin32g4.pdf) |
| EVSPIN32G4 reference | — | [Evaluation board](https://www.st.com/en/evaluation-tools/evspin32g4.html) · [user manual](https://www.st.com/resource/en/user_manual/um2850-getting-started-with-the-evspin32g4-evspin32g4nh-stmicroelectronics.pdf) · [schematic](https://www.st.com/resource/en/schematic_pack/evspin32g4_schematics.pdf) · [BOM](https://www.st.com/resource/en/bill_of_materials/evspin32g4-bom.pdf) |
| STM32G431 MCU core | Integrated | [STM32G431VB data sheet](https://www.st.com/resource/en/datasheet/stm32g431vb.pdf) · [RM0440 reference manual](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) |
| STM32 motor-control software | — | [X-CUBE-MCSDK product page](https://www.st.com/en/embedded-software/x-cube-mcsdk.html) · [Motor Control Workbench manual](https://www.st.com/resource/en/user_manual/um2380-stm32-motor-control-sdk-st-motor-control-workbench-stmicroelectronics.pdf) |
| STM32 programming/debug | — | [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) · [ST-LINK/V3SET](https://www.st.com/en/development-tools/stlink-v3set.html) |
| TCA9548APWR 8-channel I2C switch | [C130026](https://www.lcsc.com/product-detail/C130026.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/tca9548a.pdf) |
| TCA9534APWR I2C GPIO expander | [C206010](https://www.lcsc.com/product-detail/C206010.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/tca9534a.pdf) |
| ADS1115IDGSR 16-bit 4-channel ADC | [C37593](https://www.lcsc.com/product-detail/C37593.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf) |

ST offers no exposed gull-wing version of STSPIN32G4: every current ordering code in the data sheet is VFQFPN-64, 9 x 9 mm, 0.5 mm pitch. The QFN is selected under the project's explicit fallback. Its embedded STM32G431 must be programmed on all four motor nodes.

## Power bridge, current sensing and local protection

| Exact part | LCSC | Manufacturer documents |
|---|---|---|
| IPT015N10N5ATMA1 100 V MOSFET | [C108964](https://www.lcsc.com/product-detail/C108964.html) | [Infineon data sheet](https://www.infineon.com/assets/row/public/documents/24/49/infineon-ipt015n10n5-datasheet-en.pdf) · [LCSC mirror](https://datasheet.lcsc.com/szlcsc/Infineon-Technologies-IPT015N10N5ATMA1_C108964.pdf) |
| BVN-Z-R0005-1.0 0.5 mΩ Kelvin shunt | [C2688874](https://www.lcsc.com/product-detail/C2688874.html) | [BVN data sheet](https://www.isabellenhuette.com/hubfs/Files/Data-sheets/BVN.pdf) |
| INA238AIDGSR 85 V monitor | [C2868250](https://www.lcsc.com/product-detail/C2868250.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/ina238.pdf) |
| SM8S30A TVS | [C41991001](https://www.lcsc.com/product-detail/C41991001.html) | [SM8S data sheet](https://www.littelfuse.com/assetdocs/tvs-diode-sm8s-datasheet?assetguid=59636cd8-2d53-4eb3-b51b-1f5d4bf30c3e) |
| EEUFR1J471 470 µF/63 V | [C407954](https://www.lcsc.com/product-detail/C407954.html) | [Panasonic FR series](https://industrial.panasonic.com/cdbs/www-data/pdf/RDF0000/ABA0000C1215.pdf) |
| CL32B105KCJNNNE 1 µF/100 V | [C444847](https://www.lcsc.com/product-detail/C444847.html) | [Samsung product](https://product.samsungsem.com/mlcc/CL32B105KCJNNNE.do) |
| CL21B224KCFSFNE 220 nF/100 V | [C307542](https://www.lcsc.com/product-detail/C307542.html) | [Samsung MLCC catalog](https://product.samsungsem.com/mlcc/) |
| NTCGS163JF103FT8 10 kΩ NTC | [C22432708](https://www.lcsc.com/product-detail/C22432708.html) | [TDK NTC catalog](https://product.tdk.com/system/files/dam/doc/product/sensor/ntc/chip-ntc-thermistor/catalog/ntc_commercial_general_en.pdf) |

Each cell uses four separate BVN shunts: three low-side phase shunts for PWM-synchronous FOC/current protection and one high-side branch shunt for INA238 DC telemetry. INA238 is not fast enough for the local current loop or hardware trip.

For the IPT015N10N5, Infineon specifies 1.5 mOhm maximum at 10 V gate drive and 2.0 mOhm maximum at 6 V. The 1.6 mOhm figure is typical at 6 V, not a guaranteed maximum, and the data sheet gives no separate maximum at the proposed 8 V rail. Use 2.0 mOhm as the conservative initial bound until the populated bridge is characterized at its actual gate voltage and temperature.

## STSPIN32G4 support network

| Exact part | Use | LCSC | Manufacturer documents |
|---|---|---|---|
| STPS1H100A | Integrated-buck catch diode, ST typical application | [C165701](https://www.lcsc.com/product-detail/C165701.html) | [Data sheet](https://www.st.com/resource/en/datasheet/stps1h100.pdf) |
| FNR5040S180MT 18 uH | Integrated-buck inductor; 1.65 A rated, 2.0 A saturation | [C167970](https://www.lcsc.com/product-detail/C167970.html) | [LCSC data sheet](https://datasheet.lcsc.com/lcsc/1811011712_cjiang--Changjiang-Microelectronics-Tech-FNR5040S180MT_C167970.pdf) |
| CL32B106KBJNNNE 10 uF/50 V | VCC buck output capacitor | [C138687](https://www.lcsc.com/product-detail/C138687.html) | [Samsung MLCC catalog](https://product.samsungsem.com/mlcc/) |
| CL21B224KCFSFNE 220 nF/100 V | VM bypass and three bootstrap capacitors | [C307542](https://www.lcsc.com/product-detail/C307542.html) | [Samsung MLCC catalog](https://product.samsungsem.com/mlcc/) |
| CL10B105KA8NNNC 1 uF/25 V | VREF+/VDDA support | [C29936](https://www.lcsc.com/product-detail/C29936.html) | [Samsung MLCC catalog](https://product.samsungsem.com/mlcc/) |
| CL05B104KB5NNNC 100 nF/50 V | VBAT, VREF+, VDDA, REG3V3 and reset support | [C960916](https://www.lcsc.com/product-detail/C960916.html) | [Samsung MLCC catalog](https://product.samsungsem.com/mlcc/) |
| CL21B106KOQNNNE 10 uF/16 V | REG3V3 output storage | [C95841](https://www.lcsc.com/product-detail/C95841.html) | [Samsung MLCC catalog](https://product.samsungsem.com/mlcc/) |
| ABM8W-24.0000MHZ-7-D1X-T3 24 MHz/7 pF crystal | HSE candidate matching the ST reference electrical values; larger 3225 package | [C1986867](https://www.lcsc.com/product-detail/C1986867.html) | [Abracon ABM8W data sheet](https://abracon.com/Resonators/ABM8W.pdf) |
| CL05C6R8CB5NNNC 6.8 pF C0G | Provisional HSE load capacitors matching the ST evaluation BOM | [C318598](https://www.lcsc.com/product-detail/C318598.html) | [Samsung exact-part data](https://product.samsungsem.com/mlcc/CL05C6R8CB5NNN.do) |
| CL05B333KB5VPNC 33 nF/50 V | Bus-voltage and NTC ADC filters | [C472437](https://www.lcsc.com/product-detail/C472437.html) | [Samsung MLCC catalog](https://product.samsungsem.com/mlcc/) |
| BAT30KFILM | Phase/bus ADC clamp candidate used by EVSPIN32G4 | [C2926449](https://www.lcsc.com/product-detail/C2926449.html) | [Data sheet](https://www.st.com/resource/en/datasheet/bat30.pdf) |

The ST data sheet's typical application calls for 18 uH/1 A, STPS1H100A, 10 uF/25 V VCC output, four 220 nF/100 V high-voltage capacitors, two 1 uF and five 100 nF local capacitors. Crystal load, current-amplifier gain, ADC filters, comparator thresholds and phase/bus dividers remain schematic calculations. Capacitance must be checked at actual DC bias, temperature and tolerance.

## Logic power, watchdog and buffering

| Exact part | LCSC | Manufacturer documents |
|---|---|---|
| TPS54360BDDAR raw 3S–6S buck | [C524806](https://www.lcsc.com/product-detail/C524806.html) | [TPS54360B data sheet](https://www.ti.com/lit/ds/symlink/tps54360b.pdf) |
| FXL1040-8R2-M 8.2 µH | [C167232](https://www.lcsc.com/product-detail/C167232.html) | [LCSC exact-part documents](https://www.lcsc.com/product-detail/C167232.html) |
| B560C-13-F Schottky | [C85100](https://www.lcsc.com/product-detail/C85100.html) | [Data sheet](https://www.diodes.com/assets/Datasheets/ds13004.pdf) |
| GRM32ER72A225KA35L 2.2 µF/100 V | [C86054](https://www.lcsc.com/product-detail/C86054.html) | [Murata SimSurfing exact-part model](https://ds.murata.com/simsurfing/mlcc.html?oripartnumbers=%5B%22GRM32ER72A225KA35L%22%5D) |
| LMK325B7476KM-PR 47 µF/10 V | [C20486249](https://www.lcsc.com/product-detail/C20486249.html) | [Taiyo Yuden MLCC catalog](https://www.yuden.co.jp/productdata/catalog/en/mlcc_all_e.pdf) |
| RC0402FR-07162KL TPS54360B RT | [C185462](https://www.lcsc.com/product-detail/C185462.html) | [TPS54360B 5 V example](https://www.ti.com/lit/ds/symlink/tps54360b.pdf) |
| ERJ2RKF5233X TPS54360B EN/UVLO upper | [C416558](https://www.lcsc.com/product-detail/C416558.html) | [TPS54360B 5 V example](https://www.ti.com/lit/ds/symlink/tps54360b.pdf) |
| 0402WGF8452TCE TPS54360B EN/UVLO lower | [C26988](https://www.lcsc.com/product-detail/C26988.html) | [TPS54360B 5 V example](https://www.ti.com/lit/ds/symlink/tps54360b.pdf) |
| 0402WGF5362TCE TPS54360B feedback upper | [C53398](https://www.lcsc.com/product-detail/C53398.html) | [TPS54360B 5 V example](https://www.ti.com/lit/ds/symlink/tps54360b.pdf) |
| RC0402FR-0710K2L TPS54360B feedback lower | [C138069](https://www.lcsc.com/product-detail/C138069.html) | [TPS54360B 5 V example](https://www.ti.com/lit/ds/symlink/tps54360b.pdf) |
| RC0402FR-0713KL TPS54360B COMP resistor | [C138057](https://www.lcsc.com/product-detail/C138057.html) | [TPS54360B 5 V example](https://www.ti.com/lit/ds/symlink/tps54360b.pdf) |
| CL05B682KB5NNNC TPS54360B COMP capacitor | [C318580](https://www.lcsc.com/product-detail/C318580.html) | [TPS54360B 5 V example](https://www.ti.com/lit/ds/symlink/tps54360b.pdf) |
| 0402CG390J500NT TPS54360B COMP pole | [C1563](https://www.lcsc.com/product-detail/C1563.html) | [TPS54360B 5 V example](https://www.ti.com/lit/ds/symlink/tps54360b.pdf) |
| TPS2121RUXR 5 V power mux | [C485916](https://www.lcsc.com/product-detail/C485916.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/tps2121.pdf) |
| TPS62132RGTR 3.3 V buck | [C81563](https://www.lcsc.com/product-detail/C81563.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/tps62130.pdf) |
| XAL4020-222MEC 2.2 µH | [C3151182](https://www.lcsc.com/product-detail/C3151182.html) | [Coilcraft product](https://www.coilcraft.com/en-us/products/power/shielded-inductors/molded-inductor/xal/xal4020/) |
| CL21B106KOQNNNE 10 µF/16 V | [C95841](https://www.lcsc.com/product-detail/C95841.html) | [Samsung MLCC catalog](https://product.samsungsem.com/mlcc/) |
| CL21A226MPQNNNE 22 µF/10 V | [C29277](https://www.lcsc.com/product-detail/C29277.html) | [Samsung MLCC catalog](https://product.samsungsem.com/mlcc/) |
| CC0402KRX7R9BB332 TPS62132 soft-start | [C107028](https://www.lcsc.com/product-detail/C107028.html) | [Yageo MLCC catalog](https://www.yageo.com/upload/media/product/productsearch/datasheet/mlcc/PYu-CC_16V-to-50V_13.pdf) |
| RC0402FR-07100KL optional TPS62132 PG pull-up | [C60491](https://www.lcsc.com/product-detail/C60491.html) | [Yageo RC series](https://www.yageo.com/upload/media/product/productsearch/datasheet/rchip/PYu-RC_Group_51_RoHS_L_13.pdf) |
| TPS7A2033PDBVR low-noise LDO | [C2862740](https://www.lcsc.com/product-detail/C2862740.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/tps7a20.pdf) |
| TPS3430WDRCR window watchdog | [C2870545](https://www.lcsc.com/product-detail/C2870545.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/tps3430.pdf) |
| SN74LVC2G02DCTR latch logic | [C94600](https://www.lcsc.com/product-detail/C94600.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/sn74lvc2g02.pdf) |
| SN74LVC1G07DBVR open-drain FG isolation buffer | [C7829](https://www.lcsc.com/product-detail/C7829.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/sn74lvc1g07.pdf) |
| SN74LVC125APWR receiver buffer | [C7813](https://www.lcsc.com/product-detail/C7813.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/sn74lvc125a.pdf) |
| SN74AHCT1G125DBVR servo buffer | [C7484](https://www.lcsc.com/product-detail/C7484.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/sn74ahct1g125.pdf) |
| 0451002.MRL 2 A logic fuse | [C99547](https://www.lcsc.com/product-detail/C99547.html) | [Littelfuse 451/453 series](https://www.littelfuse.com/assetdocs/nano2-451-453-series-datasheet?assetguid=e3580b49-d9ac-477f-ab50-f88262262fc2) |
| STPS2H100A reverse-blocking diode | [C81548](https://www.lcsc.com/product-detail/C81548.html) | [Data sheet](https://www.st.com/resource/en/datasheet/stps2h100.pdf) |
| MF-MSMF150/24X-2 USB PPTC | [C78695](https://www.lcsc.com/product-detail/C78695.html) | [Bourns MF-MSMF data sheet](https://www.bourns.com/docs/product-datasheets/mf-msmf.pdf) |
| SMBJ5.0A USB VBUS TVS | [C83333](https://www.lcsc.com/product-detail/C83333.html) | [Littelfuse SMBJ data sheet](https://www.littelfuse.com/assetdocs/tvs-diode-smbj-datasheet?assetguid=bed67040-284a-430a-afc1-01414fb9f46a) |
| SMBJ30A optional logic-buck input TVS | [C83851](https://www.lcsc.com/product-detail/C83851.html) | [Littelfuse SMBJ data sheet](https://www.littelfuse.com/assetdocs/tvs-diode-smbj-datasheet?assetguid=bed67040-284a-430a-afc1-01414fb9f46a) |

TPS2121 sees only regulated 5 V inputs, never raw traction voltage. The separately fused raw-battery logic feed is upstream of the traction-branch contactor and must not energize an inverter through any other path. Plain USB-C pull-downs do not negotiate high power, so USB remains a flashing/debug supply.

`SN74LVC1G07` `Ioff` prevents backfeed when the buffer is unpowered; it does not actively pull its output low in that state. It is therefore not, by itself, a fail-low `NRST` implementation. The released schematic must add an independently powered suitable reset supervisor, a passive reset pull-down with qualified active release, or an equivalently proven circuit. Every additional fitted reset component must have an exact LCSC listing before release.

## Connectors and interface protection

| Exact part | LCSC | Manufacturer documents |
|---|---|---|
| USB4085-GF-A USB-C | [C7095263](https://www.lcsc.com/product-detail/C7095263.html) | [GCT drawing](https://gct.co/files/drawings/usb4085.pdf) |
| TPD4E05U06DQAR USB/CC ESD | [C138714](https://www.lcsc.com/product-detail/C138714.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/tpd4e05u06.pdf) |
| SP0504BAHTG receiver-input ESD | [C10675](https://www.lcsc.com/product-detail/C10675.html) | [Data sheet](https://www.littelfuse.com/assetdocs/tvs-diode-array-sp05-datasheet?assetguid=aa5cda8e-3825-4a58-b143-0b97fb0def03) |
| TPD1E05U06DPYR servo-signal ESD | [C436349](https://www.lcsc.com/product-detail/C436349.html) | [Data sheet](https://www.ti.com/lit/ds/symlink/tpd1e05u06.pdf) |
| CPDQC5V0SC-HF 0.2 pF GNSS RF ESD | [C2443470](https://www.lcsc.com/product-detail/C2443470.html) | [Data sheet](https://datasheet.lcsc.com/datasheet/pdf/607846473be111c6c809ed6a92ded7c7.pdf?productCode=C2443470) |
| U.FL-R-SMT-1(80) | [C88374](https://www.lcsc.com/product-detail/C88374.html) | [Hirose U.FL catalog](https://www.hirose.com/en/product/series/U.FL) |
| Molex 430450600 Micro-Fit 6-pin receiver header | [C127367](https://www.lcsc.com/product-detail/C127367.html) | [Molex product](https://www.molex.com/en-us/products/part-detail/430450600) |
| JST B3P-VH(LF)(SN) servo header | [C160316](https://www.lcsc.com/product-detail/C160316.html) | [JST VH catalog](https://www.jst-mfg.com/product/pdf/eng/eVH.pdf) |
| XFCN TM56145145-3300 M5 terminal | [C52097684](https://www.lcsc.com/product-detail/C52097684.html) | [Exact LCSC page/drawing](https://www.lcsc.com/product-detail/C52097684.html) |
| Littelfuse 142.5631.5602 60 A/58 V BF1 fuse | [C3662332](https://www.lcsc.com/product-detail/C3662332.html) | [BF1 series](https://www.littelfuse.com/products/fuses/automotive-passenger-car/high-current-fuses/bf1-58v.aspx) |
| Littelfuse 04980917ZXT 58 V holder | [C3204665](https://www.lcsc.com/product-detail/C3204665.html) | [Holder guide](https://www.littelfuse.com/assetdocs/automotive-fuse-holder-selection-guide?assetguid=f50b8dc0-4605-4936-bbd2-7106ee81af3f) |

Harness-side parts are system items, not fitted PCB parts:

| Exact part | LCSC | Manufacturer documents |
|---|---|---|
| Molex 430250600 Micro-Fit housing | [C293525](https://www.lcsc.com/product-detail/C293525.html) | [Molex product](https://www.molex.com/en-us/products/part-detail/430250600) |
| Molex 430300007 crimp contact | [C293530](https://www.lcsc.com/product-detail/C293530.html) | [Molex product](https://www.molex.com/en-us/products/part-detail/430300007) |
| JST VHR-3N VH housing | [C157899](https://www.lcsc.com/product-detail/C157899.html) | [JST VH catalog](https://www.jst-mfg.com/product/pdf/eng/eVH.pdf) |
| JST SVH-21T-P1.1 VH contact | [C160349](https://www.lcsc.com/product-detail/C160349.html) | [JST VH catalog](https://www.jst-mfg.com/product/pdf/eng/eVH.pdf) |

Catalog current ratings do not validate PCB copper, busbars, solder, terminals, bolt torque, lugs, vibration or temperature rise.

## Small passives

| Family / exact examples | LCSC | Data sheet |
|---|---|---|
| Yageo RC0402FR 10 Ω gate resistor | [RC0402FR-0710RL, C138066](https://www.lcsc.com/product-detail/C138066.html) | [RC series](https://www.yageo.com/upload/media/product/productsearch/datasheet/rchip/PYu-RC_Group_51_RoHS_L_13.pdf) |
| 10 kΩ and 100 kΩ pulls | [RC0402FR-0710KL, C60490](https://www.lcsc.com/product-detail/C60490.html) · [RC0402FR-07100KL, C60491](https://www.lcsc.com/product-detail/C60491.html) | [RC series](https://www.yageo.com/upload/media/product/productsearch/datasheet/rchip/PYu-RC_Group_51_RoHS_L_13.pdf) |
| 5.1 kΩ pull-up/USB CC | [RC0402FR-075K1L, C105872](https://www.lcsc.com/product-detail/C105872.html) | [RC series](https://www.yageo.com/upload/media/product/productsearch/datasheet/rchip/PYu-RC_Group_51_RoHS_L_13.pdf) |
| 22 Ω interface series | [RC0402FR-0722RL, C114765](https://www.lcsc.com/product-detail/C114765.html) | [RC series](https://www.yageo.com/upload/media/product/productsearch/datasheet/rchip/PYu-RC_Group_51_RoHS_L_13.pdf) |
| 100 nF/50 V general bypass | [CL05B104KB5NNNC, C960916](https://www.lcsc.com/product-detail/C960916.html) | [Samsung MLCC catalog](https://product.samsungsem.com/mlcc/) |
| 6.8 pF/50 V C0G HSE load | [CL05C6R8CB5NNNC, C318598](https://www.lcsc.com/product-detail/C318598.html) | [Samsung exact-part data](https://product.samsungsem.com/mlcc/CL05C6R8CB5NNN.do) |
| 22 kOhm ST reference current-sense resistor | [RC0402FR-0722KL, C82868](https://www.lcsc.com/product-detail/C82868.html) | [Yageo RC series](https://www.yageo.com/upload/media/product/productsearch/datasheet/rchip/PYu-RC_Group_51_RoHS_L_13.pdf) |
| 1.5 kOhm ST reference current-sense resistor | [CR0402-FX-1501GLF, C2077086](https://www.lcsc.com/product-detail/C2077086.html) | [Bourns CR series](https://www.bourns.com/docs/product-datasheets/cr.pdf) |
| 11 kOhm ST reference current-sense resistor | [RMCF0402FT11K0, C2484254](https://www.lcsc.com/product-detail/C2484254.html) | [Stackpole RMCF series](https://www.seielect.com/catalog/sei-rmcf_rmcp.pdf) |

The 1.5 kOhm/22 kOhm/11 kOhm values above reproduce the EVSPIN32G4 current-amplifier reference only; they are **not released** for the selected 0.5 mOhm shunts. Regulator feedback/compensation, current gain and offset, signal filtering, bus/phase dividers, I2C address straps, fault-latch timing and snubbers remain schematic calculations. On each TCA9548A branch, the motor-node slave address must differ from its INA238 address; repeating a node address is acceptable only across isolated mux channels. Do not fabricate until every fitted value has an exact LCSC line in the released BOM.
