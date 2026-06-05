# GPS and WiFi Connection Map

Generated from the KiCad project on 2026-06-05.

Primary source files:

- `Card_CRAN_SEGULA.kicad_pcb`
- `Navigation_Peripherique.kicad_sch`
- `Communication_Peripherique.kicad_sch`
- `MCU_Navigation.kicad_sch`
- `MCU_Communication.kicad_sch`
- Reference PDF: `PDF/Card_CRAN_SEGULA.pdf`

Notes for AI parsing:

- "GPS" in this document is the same hardware block as "GNSS" in the KiCad net names.
- The main shared 3.3 V rail is named `WIFI_BUCVBATS` in the KiCad PCB. This net also powers the GPS module and both MCUs.
- Module pin numbers are physical package pads from the PCB footprint.
- MCU pin numbers are physical package pads plus GPIO names from the PCB footprint.

## Machine-Readable Summary

```yaml
gps:
  module: "U3 NEO-M9N-00B"
  sheet: "Navigation_Peripherique"
  host_mcu: "IC1 STM32F405RGT6"
  host_mcu_sheet: "MCU_Navigation"
  host_interface: "SPI2-style GPS/GNSS interface"
  power_net: "WIFI_BUCVBATS"
  ground_net: "GND"
  connections:
    - {net: "GNSS_CS",        gps_pad: "18 SDA/SPI_CS_N", gps_direction: "input/bidirectional", mcu_pad: "33 PB12", mcu_signal: "SPI2_NSS / GPS_CS", notes: "R45 10k pull-up to WIFI_BUCVBATS"}
    - {net: "GNSS_CLK",       gps_pad: "19 SCL/SPI_CLK",  gps_direction: "input",               mcu_pad: "29 PB10", mcu_signal: "SPI2_SCK"}
    - {net: "GNSS_SDO",       gps_pad: "20 TXD/SPI_MISO", gps_direction: "GPS to MCU",           mcu_pad: "10 PC2",  mcu_signal: "SPI2_MISO"}
    - {net: "GNSS_SDI",       gps_pad: "21 RXD/SPI_MOSI", gps_direction: "MCU to GPS",           mcu_pad: "11 PC3",  mcu_signal: "SPI2_MOSI"}
    - {net: "GNSS_PPS",       gps_pad: "3 TIMEPULSE",     gps_direction: "GPS to MCU",           mcu_pad: "53 PC12", mcu_signal: "PPS input"}
    - {net: "GNSS_NRST",      gps_pad: "8 RESET_N",       gps_direction: "MCU to GPS",           mcu_pad: "3 PC14",  mcu_signal: "GPS reset; R2 10k pull-up to WIFI_BUCVBATS"}
    - {net: "GNSS_LNA_EN",    gps_pad: "14 LNA_EN",       gps_direction: "GPS to MCU",           mcu_pad: "2 PC13",  mcu_signal: "LNA enable monitor/control net"}
    - {net: "GNSS_BAT_BACKUP", gps_pad: "22 V_BCKP",      gps_direction: "power",                mcu_pad: null,      mcu_signal: null, notes: "to JP6 pad 2/C"}

wifi:
  module: "U6 NRF7002-QFAA-R"
  sheet: "Communication_Peripherique"
  host_mcu: "IC2 STM32H753VIT6"
  host_mcu_sheet: "MCU_Communication"
  host_interface: "SPI4 / NRF7002 QSPI-named pins used as 4-wire SPI"
  power_net: "WIFI_BUCVBATS"
  ground_net: "GND"
  connections:
    - {net: "WIFI_CS",     wifi_pad: "36 QSPI_SS",    wifi_direction: "MCU to WiFi", mcu_pad: "41 PE11", mcu_signal: "SPI4_NSS / WIFI_NSS", notes: "R42 10k pull-up to WIFI_BUCVBATS"}
    - {net: "WIFI_CLK",    wifi_pad: "35 QSPI_CLK",   wifi_direction: "MCU to WiFi", mcu_pad: "42 PE12", mcu_signal: "SPI4_SCK / WIFI_SCK"}
    - {net: "WIFI_MISO",   wifi_pad: "38 QSPI_DATA1", wifi_direction: "WiFi to MCU", mcu_pad: "43 PE13", mcu_signal: "SPI4_MISO / WIFI_MISO"}
    - {net: "WIFI_MOSI",   wifi_pad: "37 QSPI_DATA0", wifi_direction: "MCU to WiFi", mcu_pad: "44 PE14", mcu_signal: "SPI4_MOSI / WIFI_MOSI"}
    - {net: "WIFI_INT",    wifi_pad: "46 HOST_IRQ",   wifi_direction: "WiFi to MCU", mcu_pad: "33 PC5",  mcu_signal: "GPIO input; CubeMX label WIFI_INIT"}
    - {net: "WIFI_BUCKEN", wifi_pad: "30 BUCKEN",     wifi_direction: "MCU to WiFi", mcu_pad: "63 PC6",  mcu_signal: "GPIO output / WIFI_BUCKEN"}
```

## GPS / GNSS Module

GPS module:

- Reference: `U3`
- Part: `NEO-M9N-00B`
- Sheet: `Navigation_Peripherique`
- Host MCU: `IC1 STM32F405RGT6`

### GPS Host Interface

| Net | GPS module pin | GPS pad | Connected MCU pin | MCU pad | Direction | Notes |
|---|---:|---|---|---:|---|---|
| `GNSS_CS` | `SDA/SPI_CS_N` | `U3.18` | `PB12` on `IC1` | `33` | MCU to GPS | GPS chip select. `R45` 10k pulls this net up to `WIFI_BUCVBATS`. |
| `GNSS_CLK` | `SCL/SPI_CLK` | `U3.19` | `PB10` on `IC1` | `29` | MCU to GPS | SPI clock. |
| `GNSS_SDO` | `TXD/SPI_MISO` | `U3.20` | `PC2` on `IC1` | `10` | GPS to MCU | GPS data output, MCU MISO. |
| `GNSS_SDI` | `RXD/SPI_MOSI` | `U3.21` | `PC3` on `IC1` | `11` | MCU to GPS | MCU MOSI, GPS data input. |
| `GNSS_PPS` | `TIMEPULSE` | `U3.3` | `PC12` on `IC1` | `53` | GPS to MCU | PPS/timepulse signal. |
| `GNSS_NRST` | `RESET_N` | `U3.8` | `PC14` on `IC1` | `3` | MCU to GPS | GPS reset. `R2` 10k pulls this net up to `WIFI_BUCVBATS`. |
| `GNSS_LNA_EN` | `LNA_EN` | `U3.14` | `PC13` on `IC1` | `2` | GPS to MCU | LNA enable net. |

Firmware cross-check:

- `nav_f405_test/nav_f405_test/Core/Src/main.c` configures GPS over `SPI2`.
- The comments in that file map GPS pins as `PB10 = SPI2_SCK`, `PC2 = SPI2_MISO`, `PC3 = SPI2_MOSI`, and `PB12 = CS`.

### GPS Power, Boot, and Backup

| Net | GPS module pin | GPS pad | Other endpoint | Direction / type | Notes |
|---|---:|---|---|---|---|
| `WIFI_BUCVBATS` | `VCC` | `U3.23` | Shared 3.3 V rail | Power input | Same rail also powers `IC1`, `IC2`, and `U6`. |
| `GND` | `GND`, `GND__1`, `GND__2`, `GND__3` | `U3.10`, `U3.12`, `U3.13`, `U3.24` | Board ground | Ground | GPS ground pins. |
| `GND` | `D_SEL` | `U3.2` | Board ground | Strap input | Board is wired for the SPI-style interface used by the firmware. |
| `GND` | `V_USB` | `U3.7` | Board ground | Power/strap | USB is not used on this GPS module. |
| `GNSS_BAT_BACKUP` | `V_BCKP` | `U3.22` | `JP6.2` / pad `C` | Backup power | GPS backup supply net. |
| `Net-(JP1-C)` | `SAFEBOOT_N` | `U3.1` | `JP1.2` / pad `C` | Boot strap | `JP1.1` / pad `A` is `WIFI_BUCVBATS`; `JP1.3` / pad `B` is `GND`. |

`JP6` backup jumper:

| JP6 pad | Pin name | Net | Meaning |
|---:|---|---|---|
| `1` | `A` | `WIFI_BUCVBATS` | Shared 3.3 V rail. |
| `2` | `C` | `GNSS_BAT_BACKUP` | GPS backup supply to `U3.22`. |
| `3` | `B` | `Net-(JP6-B)` | Alternate backup source side. |

### GPS RF / Antenna Path

| Segment | Net | Endpoint A | Endpoint B | Notes |
|---|---|---|---|---|
| GPS RF input | `Net-(U3-RF_IN)` | `U3.11 RF_IN` | `C85.2` | GPS RF input side. |
| Series RF capacitor | `C85 22pF` | `C85.2` on `Net-(U3-RF_IN)` | `C85.1` on `Net-(ANT1-SIGNAL)` | Series coupling/matching part. |
| GPS antenna signal | `Net-(ANT1-SIGNAL)` | `C85.1` | `ANT1.1 SIGNAL` | Main GPS antenna signal net. |
| GPS antenna ground | `GND` | `ANT1.2 GND` | `ANT1.3 GND` | Antenna ground pads. |
| Shunt capacitor | `Net-(ANT1-SIGNAL)` to `GND` | `C9.2` | `C9.1` | `C9` is 10nF to ground. |
| Bias/matching inductor | `Net-(ANT1-SIGNAL)` to `Net-(L1-Pad2)` | `L1.1` | `L1.2` | `L1` is 27nH. |
| RF bias feed | `Net-(L1-Pad2)` to `Net-(U3-VCC_RF)` | `R1.1` | `R1.2` | `R1` is 10R. |
| GPS RF bias pin | `Net-(U3-VCC_RF)` | `R1.2` | `U3.9 VCC_RF` | GPS antenna/RF bias supply pin. |

### GPS No-Connect Pins

| GPS pad | Pin name | Net |
|---:|---|---|
| `U3.4` | `EXTINT` | `unconnected-(U3-EXTINT-Pad4)` |
| `U3.5` | `USB_DM` | `unconnected-(U3-USB_DM-Pad5)` |
| `U3.6` | `USB_DP` | `unconnected-(U3-USB_DP-Pad6)` |
| `U3.15` | `RESERVED` | `unconnected-(U3-RESERVED-Pad15)` |
| `U3.16` | `RESERVED__1` | `unconnected-(U3-RESERVED__1-Pad16)` |
| `U3.17` | `RESERVED__2` | `unconnected-(U3-RESERVED__2-Pad17)` |

## WiFi Module

WiFi module:

- Reference: `U6`
- Part: `NRF7002-QFAA-R`
- Sheet: `Communication_Peripherique`
- Host MCU: `IC2 STM32H753VIT6`

### WiFi Host Interface

| Net | WiFi module pin | WiFi pad | Connected MCU pin | MCU pad | Direction | Notes |
|---|---:|---|---|---:|---|---|
| `WIFI_CS` | `QSPI_SS` | `U6.36` | `PE11` on `IC2` | `41` | MCU to WiFi | CubeMX label `WIFI_NSS`, SPI4 NSS. `R42` 10k pulls this net up to `WIFI_BUCVBATS`. |
| `WIFI_CLK` | `QSPI_CLK` | `U6.35` | `PE12` on `IC2` | `42` | MCU to WiFi | CubeMX label `WIFI_SCK`, SPI4 SCK. |
| `WIFI_MISO` | `QSPI_DATA1` | `U6.38` | `PE13` on `IC2` | `43` | WiFi to MCU | CubeMX label `WIFI_MISO`, SPI4 MISO. |
| `WIFI_MOSI` | `QSPI_DATA0` | `U6.37` | `PE14` on `IC2` | `44` | MCU to WiFi | CubeMX label `WIFI_MOSI`, SPI4 MOSI. |
| `WIFI_INT` | `HOST_IRQ` | `U6.46` | `PC5` on `IC2` | `33` | WiFi to MCU | Interrupt input to MCU. CubeMX currently labels this pin `WIFI_INIT`. |
| `WIFI_BUCKEN` | `BUCKEN` | `U6.30` | `PC6` on `IC2` | `63` | MCU to WiFi | Buck regulator enable. CubeMX label `WIFI_BUCKEN`. |

Firmware/CubeMX cross-check:

- `com_h753/com_H753VIT/com_H753VIT.ioc` maps `PE11/PE12/PE13/PE14` to `SPI4_NSS/SPI4_SCK/SPI4_MISO/SPI4_MOSI`.
- The same `.ioc` maps `PC5` as a GPIO input and `PC6` as a GPIO output.

### WiFi Power and Local Rails

| Net | WiFi module pins | Support parts / endpoints | Notes |
|---|---|---|---|
| `WIFI_BUCVBATS` | `U6.13 VBAT`, `U6.20 AFEVBAT`, `U6.25 BUCKVBAT`, `U6.31 BUCVBATS`, `U6.48 IOVDD` | Shared 3.3 V rail. Decoupling includes `C11`, `C12`, `C25`, `C26`, `C27`, `C29`, `C30`. | Same net also powers GPS `U3.23 VCC`, `IC1` VDD/VBAT pins, and `IC2` VDD/VBAT pins. |
| `GND` | `U6.27 BUCKVSS`, `U6.28 BUCKVSS`, `U6.47 VSS`, `U6.49 VSS` | Board ground | WiFi ground pins. |
| `Net-(U6-BUCKOUT)` | `U6.26 BUCKOUT` | `L2.1` | Internal buck output before inductor. |
| `WIFI_RFBUCKVDD` | `U6.15 RFBUCKVDD`, `U6.32 PWRBUCKVDD` | `L2.2`, `C23`, `C24`, `C28` | Buck-derived RF/power rail after `L2 3.3uH`. |
| `WIFI_PWRIOVDD` | `U6.1 OTPVDD`, `U6.34 PWRIOVDD` | `C20 100nF`, `C21 2.2uF` | Local PWR/IO supply decoupling. |
| `WIFI_DIGVDD` | `U6.33 DIGVDD` | `C22 1uF` | Digital supply decoupling. |
| `WIFI_RFVDD` | `U6.14 RFVDD` | `C17 22nF` | RF supply decoupling. |
| `WIFI_SXLDO` | `U6.11 SXLDO` | `C13 0.47uF` | LDO capacitor net. |
| `Net-(U6-PALDO)` | `U6.8 PAVDD<1>`, `U6.10 PAVDD<0>`, `U6.12 PALDO` | `C14 4.7uF`, `C15 100nF` | PA LDO rail. |
| `WIFI_XOLDO` | `U6.16 XOLDO` | `C16 2.2uF` | Crystal oscillator LDO net. |
| `WIFI_AFELDO` | `U6.19 AFELDO` | `C18 1uF` | AFE LDO net. |
| `WIFI_BUCKVMID` | `U6.29 BUCKVMID` | `C19 10nF` | Buck midpoint capacitor net. |

### WiFi RF / Antenna Path

| Segment | Net | Endpoint A | Endpoint B | Notes |
|---|---|---|---|---|
| 2.4 GHz RF | `WIFI_2.4GH` | `U6.9 TXRF<0>` | `U8.1` | Input to diplexer `U8`. |
| 5 GHz RF | `WIFI_5GH` | `U6.7 TXRF<1>` | `U8.3` | Input to diplexer `U8`. |
| Combined antenna RF | `Net-(ANT2-SIGNAL)` | `U8.5` | `ANT2.1 SIGNAL` | Output of diplexer to antenna connector. |
| RF grounds | `GND` | `U8.2`, `U8.4`, `U8.6` | `ANT2.2`, `ANT2.3` | Diplexer and antenna grounds. |

### WiFi Crystal / Oscillator

| Net | WiFi module pin | WiFi pad | Other endpoint |
|---|---:|---|---|
| `WIFI_XOP` | `XOP` | `U6.17` | `X1.1` |
| `WIFI_XON` | `XON` | `U6.18` | `X1.3` |
| `GND` | Ground pins | - | `X1.2`, `X1.4` |

`X1` value is `NX2016SA-40M-STD-CZS-3`.

### WiFi Other Control Nets

| Net | WiFi module pin | WiFi pad | Other endpoint | Notes |
|---|---:|---|---|---|
| `Net-(U6-SW_CTRL0)` | `SW_CTRL0` | `U6.44` | `R5.1` | `R5` is 100R. The other side of `R5` is `Net-(D1-A)`. |

### WiFi No-Connect Pins

| WiFi pad | Pin name | Net |
|---:|---|---|
| `U6.2` | `NC` | `unconnected-(U6-NC-Pad2)` |
| `U6.3` | `NC` | `unconnected-(U6-NC-Pad3)` |
| `U6.4` | `NC` | `unconnected-(U6-NC-Pad4)` |
| `U6.5` | `NC` | `unconnected-(U6-NC-Pad5)` |
| `U6.6` | `NC` | `unconnected-(U6-NC-Pad6)` |
| `U6.21` | `NC` | `unconnected-(U6-NC-Pad21)` |
| `U6.22` | `NC` | `unconnected-(U6-NC-Pad22)` |
| `U6.23` | `NC` | `unconnected-(U6-NC-Pad23)` |
| `U6.24` | `NC` | `unconnected-(U6-NC-Pad24)` |
| `U6.39` | `QSPI_DATA2` | `unconnected-(U6-QSPI_DATA2-Pad39)` |
| `U6.40` | `QSPI_DATA3` | `unconnected-(U6-QSPI_DATA3-Pad40)` |
| `U6.41` | `COEX_STATUS0` | `unconnected-(U6-COEX_STATUS0-Pad41)` |
| `U6.42` | `COEX_REQ` | `unconnected-(U6-COEX_REQ-Pad42)` |
| `U6.43` | `COEX_GRANT` | `unconnected-(U6-COEX_GRANT-Pad43)` |
| `U6.45` | `SW_CTRL1` | `unconnected-(U6-SW_CTRL1-Pad45)` |

