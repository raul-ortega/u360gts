# Virtual serial ports

u360gts supports two types of virtual serial ports in addition to hardware UARTs: software serial (SoftSerial) and USB Virtual COM Port (VCP).

## Software Serial (SoftSerial)

SoftSerial provides up to two additional serial ports using bit-banging on arbitrary GPIO pins. They share a hardware timer (TIM3) with the PWM inputs.

### Enabling SoftSerial

```
feature SOFTSERIAL
```

### Pin assignment

SoftSerial pins are defined per target:

| Port | Pin | Timer hardware |
|---|---|---|
| SOFTSERIAL1 RX | PWM 5 | TIM3_CH4 |
| SOFTSERIAL1 TX | PWM 6 | TIM3_CH5 |
| SOFTSERIAL2 RX | PWM 7 | TIM3_CH6 |
| SOFTSERIAL2 TX | PWM 8 | TIM3_CH7 |

On targets without PWM outputs 5-8, SoftSerial may not be available. Check your target's `target.h` for `USE_SOFTSERIAL1` / `USE_SOFTSERIAL2` defines.

### Limitations

- Baud rates are less stable than hardware UART
- Shares timer resources with PWM input
- Not available when SoftSerial feature is disabled
- Only available on targets that define `USE_SOFTSERIAL1` / `USE_SOFTSERIAL2`

## USB Virtual COM Port (VCP)

On boards with USB support (CC3D, SPARKY, SPRACINGF3, etc.), the USB port can act as a virtual serial port.

### VCP identifiers

| Identifier | Value |
|---|---|
| USB_VCP | 20 |

### VCP availability by target

| Target | VCP available |
|---|---|
| CC3D | Yes |
| SPARKY | Yes |
| SPRACINGF3 | Yes |
| RMDO | Yes |
| ALIENWIIF3 | Yes |
| COLIBRI_RACE | Yes |
| MOTOLAB | Yes |
| CHEBUZZF3 | Yes |
| STM32F3DISCOVERY | Yes |
| NAZE32PRO | Yes |
| NAZE / BLUEPILL | No |

## Configuring serial ports

Use the `serial` command in CLI to configure port functions.

Each serial port can be assigned one or more functions via a function bitmask:

| Function | Bit | Value |
|---|---|---|
| MSP | 0 | 1 |
| GPS | 1 | 2 |
| TELEMETRY_FRSKY | 2 | 4 |
| TELEMETRY_HOTT | 3 | 8 |
| TELEMETRY_MSP | 4 | 16 |
| TELEMETRY_SMARTPORT | 5 | 32 |
| RX_SERIAL | 6 | 64 |
| BLACKBOX | 7 | 128 |
| TELEMETRY_MFD | 8 | 256 |
| TELEMETRY_MAVLINK | 9 | 512 |
| TELEMETRY_NMEA | 10 | 1024 |
| TELEMETRY_LTM | 11 | 2048 |
| TELEMETRY_POSEST | 12 | 4096 |
| TELEMETRY_FORWARD | 13 | 8192 |

### Default configuration

| Port | Function | Baud |
|---|---|---|
| USART1 (Serial 0) | MSP | 115200 |
| USART2 (Serial 1) | GPS | 9600 |

## Connections

```
USB VCP ──> Configurator / MSP
UART1   ──> Configurator / MSP (when no USB)
UART2   ──> GPS module
SoftSerial 1/2 ──> Telemetry output
```
