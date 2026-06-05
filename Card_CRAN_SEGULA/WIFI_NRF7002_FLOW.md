# nRF7002 WiFi Connection and Bring-Up Flow

This note explains how the WiFi chip on the CRAN SEGULA board is connected and what has to happen in firmware to make it work.

## Main Idea

The WiFi chip is `U6`, an `NRF7002-QFAA-R`.

It is not a standalone WiFi module like an ESP8266/ESP32 running AT commands. It is a WiFi companion IC. That means:

- The `STM32H753VIT6` communication MCU runs the application.
- The STM32 also runs the TCP/IP stack and the WiFi driver.
- The nRF7002 handles the WiFi radio and low-level WiFi MAC/PHY work.
- The STM32 talks to the nRF7002 over SPI.

Simple mental model:

```text
Your application
    |
TCP/IP stack on STM32
    |
nRF7002 WiFi driver
    |
SPI4 bus
    |
nRF7002 WiFi radio chip
    |
Diplexer and antenna
    |
WiFi router / access point
```

## Hardware Connections

The WiFi chip is connected to the communication MCU `IC2 STM32H753VIT6`.

| Function | Board net | STM32H753 pin | nRF7002 pin | Direction |
|---|---|---|---|---|
| Chip select | `WIFI_CS` | `PE11 / SPI4_NSS` | `QSPI_SS` | STM32 to WiFi |
| SPI clock | `WIFI_CLK` | `PE12 / SPI4_SCK` | `QSPI_CLK` | STM32 to WiFi |
| MISO | `WIFI_MISO` | `PE13 / SPI4_MISO` | `QSPI_DATA1` | WiFi to STM32 |
| MOSI | `WIFI_MOSI` | `PE14 / SPI4_MOSI` | `QSPI_DATA0` | STM32 to WiFi |
| Interrupt | `WIFI_INT` | `PC5` | `HOST_IRQ` | WiFi to STM32 |
| Power enable | `WIFI_BUCKEN` | `PC6` | `BUCKEN` | STM32 to WiFi |

The nRF7002 pins are named `QSPI_*`, but on this board only `DATA0` and `DATA1` are connected. `QSPI_DATA2` and `QSPI_DATA3` are not connected.

So conceptually this board uses the nRF7002 in 4-wire SPI mode:

```text
CS   = WIFI_CS
SCK  = WIFI_CLK
MOSI = WIFI_MOSI
MISO = WIFI_MISO
```

## Power Path

The main WiFi supply net is:

```text
WIFI_BUCVBATS
```

This rail feeds the nRF7002 power inputs:

- `VBAT`
- `AFEVBAT`
- `BUCKVBAT`
- `BUCVBATS`
- `IOVDD`

The nRF7002 then generates/uses local internal rails with the external support components:

- Buck inductor `L2`
- Decoupling capacitors around `WIFI_RFBUCKVDD`
- LDO capacitor nets such as `WIFI_SXLDO`, `WIFI_XOLDO`, `WIFI_AFELDO`, `WIFI_DIGVDD`, and `WIFI_PWRIOVDD`

The important firmware-controlled power signal is:

```text
PC6 -> WIFI_BUCKEN -> nRF7002 BUCKEN
```

The STM32 must drive `WIFI_BUCKEN` to the correct active level to enable the nRF7002 internal buck/power system.

## RF Path

The nRF7002 has separate RF outputs for 2.4 GHz and 5 GHz:

| Band | Board net | nRF7002 pin | Goes to |
|---|---|---|---|
| 2.4 GHz | `WIFI_2.4GH` | `TXRF<0>` | Diplexer `U8` |
| 5 GHz | `WIFI_5GH` | `TXRF<1>` | Diplexer `U8` |

The diplexer combines the two RF paths into one antenna connector:

```text
nRF7002 2.4 GHz path ----\
                          -> U8 diplexer -> ANT2
nRF7002 5 GHz path ------/
```

## Boot and Communication Flow

At startup, the STM32 should do this:

1. Configure GPIO clocks and pins.
2. Configure `WIFI_BUCKEN` as GPIO output.
3. Configure `WIFI_INT` as interrupt input, normally EXTI on `PC5`.
4. Configure `SPI4` as SPI master.
5. Keep `WIFI_CS` inactive when idle.
6. Enable the nRF7002 by driving `WIFI_BUCKEN`.
7. Wait for the nRF7002 power/clock system to become ready.
8. Start the nRF7002 driver.
9. Driver initializes the chip over SPI.
10. Driver loads/configures required WiFi firmware data.
11. Application asks the WiFi stack to scan or connect.
12. nRF7002 raises `HOST_IRQ` when it has events or received data.
13. STM32 interrupt handler wakes the WiFi driver.
14. Driver reads event/data packets from nRF7002 over SPI.

Conceptual sequence:

```text
STM32 boots
    |
Configure SPI4 and GPIOs
    |
Set WIFI_BUCKEN active
    |
nRF7002 powers up
    |
STM32 driver initializes nRF7002 over SPI4
    |
WiFi firmware/driver state starts
    |
Scan SSIDs or connect to configured SSID
    |
Exchange IP packets through TCP/IP stack
```

## Runtime Data Flow

### Sending Data

Example: the STM32 wants to send telemetry to a server.

```text
Application creates data
    |
TCP/UDP socket sends packet
    |
TCP/IP stack builds IP packet
    |
nRF7002 driver wraps it for WiFi
    |
STM32 sends packet over SPI4
    |
nRF7002 transmits over antenna
    |
WiFi access point receives packet
```

### Receiving Data

Example: a server sends data back to the board.

```text
WiFi access point transmits packet
    |
ANT2 receives RF signal
    |
nRF7002 receives WiFi frame
    |
nRF7002 asserts HOST_IRQ on WIFI_INT / PC5
    |
STM32 EXTI handler notifies WiFi driver
    |
STM32 reads packet over SPI4
    |
TCP/IP stack processes packet
    |
Application receives data
```

## What Is Already Present In This Project

The board connection map says the hardware wiring exists.

The CubeMX `.ioc` file also maps:

- `PE11` to `SPI4_NSS / WIFI_NSS`
- `PE12` to `SPI4_SCK / WIFI_SCK`
- `PE13` to `SPI4_MISO / WIFI_MISO`
- `PE14` to `SPI4_MOSI / WIFI_MOSI`
- `PC5` to GPIO input labelled `WIFI_INIT`
- `PC6` to GPIO output labelled `WIFI_BUCKEN`

However, the current active communication firmware in `com_h753/com_H753VIT/Core/Src/main.c` is mainly using `SPI3` for the NAV/COM link. I did not find active application code that:

- Creates `hspi4`
- Calls `MX_SPI4_Init()`
- Drives `WIFI_BUCKEN`
- Handles `WIFI_INT` as an EXTI interrupt
- Runs an nRF7002 WiFi driver
- Runs a TCP/IP stack for WiFi

So the board appears wired for WiFi, but the firmware still needs a real WiFi software stack.

## Minimal Bring-Up Checklist

Before trying full WiFi connection, bring it up in layers.

### 1. Hardware Power Check

Check with a multimeter or oscilloscope:

- `WIFI_BUCVBATS` is around 3.3 V.
- `WIFI_BUCKEN` changes when STM32 toggles `PC6`.
- Local nRF7002 rails look valid after enabling `BUCKEN`.
- The 40 MHz crystal path is assembled correctly.

### 2. SPI Signal Check

Write a small firmware test that only toggles SPI4:

- `PE11` chip select moves.
- `PE12` clock toggles.
- `PE14` MOSI toggles.
- `PE13` MISO is not stuck because of solder/pin issues.

Use a logic analyzer if available.

### 3. Interrupt Check

Configure `PC5` as EXTI input.

The nRF7002 driver should be notified when `HOST_IRQ` changes. Polling may be useful for first debug, but the normal design should use the interrupt.

### 4. Driver Integration

You need the nRF70/nRF7002 driver stack.

There are two realistic paths:

1. Use Zephyr/nRF Connect SDK style integration.
2. Port Nordic's nRF70 driver transport layer into the STM32Cube project.

For this board, the STM32Cube path means you must provide:

- SPI transfer function using `HAL_SPI_TransmitReceive()` on `SPI4`
- Chip-select control if using software CS
- Delay/timing functions
- Interrupt callback for `WIFI_INT`
- Memory allocation/buffers expected by the driver
- Firmware blob handling required by the nRF70 driver
- TCP/IP integration, likely LwIP

### 5. Network Test

After driver initialization works:

1. Scan for SSIDs.
2. Connect to a known 2.4 GHz network first.
3. Get an IP address with DHCP.
4. Ping the gateway.
5. Send a UDP test packet.
6. Then test TCP/MQTT/HTTP depending on the final application.

## Important Firmware Shape

Very simplified pseudocode:

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI4_Init();

    HAL_GPIO_WritePin(WIFI_BUCKEN_GPIO_Port, WIFI_BUCKEN_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    nrf7002_platform_init(&hspi4);
    nrf7002_driver_init();

    wifi_scan();
    wifi_connect("SSID", "PASSWORD");

    while (1)
    {
        wifi_driver_process();
        application_process();
    }
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(WIFI_INT_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == WIFI_INT_Pin)
    {
        nrf7002_driver_irq();
    }
}
```

This is only conceptual. The real function names depend on the driver stack you choose.

## Key Point

The hardware connection is:

```text
STM32H753 SPI4 + GPIO control
        |
        v
nRF7002 WiFi companion IC
        |
        v
Diplexer + antenna
```

To make it work, the missing piece is not just "send AT commands". The missing piece is a real nRF7002 host driver plus a TCP/IP network stack running on the STM32.
