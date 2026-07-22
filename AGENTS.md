# AI Contribution Guidelines for u360gts

This document provides guidelines for AI assistants (ChatGPT, Claude, Copilot, Gemini, or any LLM-based tool) contributing code to the u360gts project. These rules supplement the existing [CONTRIBUTING.md](CONTRIBUTING.md) — which also applies fully to AI-assisted contributions.

u360gts is firmware for DIY 360° antenna trackers that control real servos and hardware. Every change must be correct, tested, and reviewable by human maintainers.

---

## Table of Contents

- [1. Code of Conduct & Ethics](#1-code-of-conduct--ethics)
- [2. Repository Structure](#2-repository-structure)
- [3. Coding Style](#3-coding-style)
- [4. Build System](#4-build-system)
- [5. Testing](#5-testing)
- [6. Documentation](#6-documentation)
- [7. Commit Messages](#7-commit-messages)
- [8. Pull Request Guidelines](#8-pull-request-guidelines)
- [9. Interacting with Maintainers & Developers](#9-interacting-with-maintainers--developers)
- [10. What AI Should NOT Do](#10-what-ai-should-not-do)

---

## 1. Code of Conduct & Ethics

- **Always** read and follow the existing [CONTRIBUTING.md](CONTRIBUTING.md).
- **Never** fabricate test results, log data, or claim testing that was not actually performed.
- **Always** disclose that a contribution was AI-assisted. The human submitting the PR bears full responsibility.
- This firmware controls physical hardware (servos, motors). Do not introduce changes that could cause unsafe hardware behavior.

---

## 2. Repository Structure

Understand the layout before making changes:

```text
src/main/                    # Main firmware source tree
  main.c                     # Entry point (init + tracker_setup + tracker_loop)
  platform.h                 # Platform definitions (STM32F10X / STM32F303xC)
  mw.c / mw.h                # Main wing (Cleanflight core)
  build_config.c / .h        # Build configuration
  version.c / .h             # Firmware version
  debug.c / .h               # Debug helpers

  tracker/                   # Antenna tracker logic (the core of u360gts)
    main.c                   # tracker_setup() and tracker_loop()
    Arduino.c / .h           # Arduino-compatible wrappers (millis, micros, delay)
    math.c / .h              # Math helpers (distance_between, course_to, ...)
    TinyGPS.c / .h           # NMEA GPS parser
    telemetry.c / .h         # Telemetry protocol state machine
    compass.c / .h            # Compass handling
    mavlink.c                # MAVLink parser
    frskyd.c                 # FrSky D protocol
    frskyx.c                 # FrSky X (SmartPort) protocol
    ltm.c                    # LTM protocol
    mfd.c                    # MFD protocol
    rvosd.c                  # RVOSD protocol
    pitlab.c                 # PitLab protocol
    crossfire.c              # Crossfire protocol
    protocol_detection.c/.h  # Auto-detection of telemetry protocols
    gps.c                    # Local GPS handling
    gps_estimation.c / .h    # Position estimation system (EPS)
    interpolation.c / .h     # Lagrange interpolation for EPS
    defines.h                # Core types (positionVector_t, epsVectorGain_t)
    config.h                 # Tracker-specific config defines
    servos.h                 # Servo pin definitions
    hott.cpp                 # HoTT telemetry (C++)

  config/
    config.c / .h            # EEPROM config read/write
    config_master.h          # master_t struct (all config parameters)
    config_profile.h         # Profile struct
    runtime_config.c / .h    # Runtime state (feature flags, arming)

  common/                    # Shared utilities
    maths.c / .h             # Math primitives
    printf.c / .h            # Lightweight printf
    filter.c / .h            # Digital filters
    encoding.c / .h          # Base64/Varint encoding
    typeconversion.c / .h    # Type casting helpers
    atomic.h                 # Atomic operations
    axis.h                   # Axis definitions
    color.h / colorconversion.c/.h
    utils.h                  # Utility macros

  drivers/                   # Hardware drivers (MCU peripherals & sensors)
    serial.c / .h            # Serial port abstraction
    serial_uart.c / .h       # UART driver
    serial_uart_stm32f10x.c  # F1 UART
    serial_uart_stm32f30x.c  # F3 UART
    serial_softserial.c      # Software serial bit-bang
    serial_usb_vcp.c / .h    # USB VCP
    system.c / .h            # System init
    system_stm32f10x.c / f30x.c
    gpio.h / gpio_stm32f10x.c / f30x.c
    timer.c / .h             # Timer abstraction
    timer_stm32f10x.c / f30x.c
    pwm_mapping.c / .h       # PWM pin mapping
    pwm_output.c / .h        # PWM servo output
    pwm_rx.c / .h            # PWM receiver input
    adc.c / .h / adc_impl.h  # ADC driver
    adc_stm32f10x.c / f30x.c
    bus_i2c.h / bus_i2c_soft.c / i2c_stm32f10x.c / f30x.c
    bus_spi.c / .h           # SPI driver
    accgyro_*.c / .h         # Accelerometer/gyro drivers
    barometer_*.c / .h       # Barometer drivers
    compass_*.c / .h         # Magnetometer drivers
    display_ug2864hsweg01.c/.h # OLED display (SSD1306)
    sound_beeper.c / .h      # Beeper abstraction
    sound_beeper_stm32f10x.c / f30x.c
    light_led.c / .h         # LED abstraction
    light_led_stm32f10x.c / f30x.c
    light_ws2811strip.c / .h # WS2811 LED strip
    flash_m25p16.c / .h      # SPI flash
    sonar_hcsr04.c / .h      # Sonar driver
    inverter.c / .h          # Serial inverter
    sensor.h                 # Sensor type definitions

  flight/                    # Flight control algorithms (adapted from Cleanflight)
    pid.c / .h               # PID controller
    imu.c / .h               # IMU fusion
    mixer.c / .h             # Motor/servo mixer
    failsafe.c / .h          # Failsafe logic
    navigation.c / .h        # Navigation
    gps_conversion.c / .h    # GPS coordinate conversion
    altitudehold.c / .h      # Altitude hold
    lowpass.c / .h           # Low-pass filter
    gtune.c / .h             # G-tune

  io/                        # I/O subsystems
    serial.c / .h            # Serial port management
    serial_cli.c / .h        # CLI (command-line interface)
    serial_msp.c / .h        # MultiWii Serial Protocol
    serial_1wire.c / .h      # 1-wire serial
    gps.c / .h               # GPS thread
    display.c / .h           # Display management
    beeper.c / .h            # Beeper patterns
    rc_controls.c / .h       # RC channel processing
    rc_curves.c / .h         # RC curve mapping
    ledstrip.c / .h          # LED strip control
    flashfs.c / .h           # Flash filesystem
    statusindicator.c / .h   # Status LED patterns
    gimbal.h                 # Gimbal definitions
    escservo.h               # ESC/Servo definitions

  rx/                        # Receiver protocols
    rx.c / .h                # Receiver abstraction
    pwm.c / .h               # PWM/PPM input
    sbus.c / .h              # SBUS
    spektrum.c / .h          # Spektrum satellite
    sumd.c / .h              # SUMD
    sumh.c / .h              # SUMH
    xbus.c / .h              # XBUS
    msp.c / .h               # MSP RC

  sensors/                   # Sensor abstraction layer
    sensors.h                # Sensor detection order
    acceleration.c / .h      # Accelerometer abstraction
    gyro.c / .h              # Gyroscope abstraction
    compass.c / .h           # Compass abstraction
    barometer.c / .h         # Barometer abstraction
    battery.c / .h           # Battery monitoring
    sonar.c / .h             # Sonar abstraction
    boardalignment.c / .h    # Board alignment
    initialisation.c / .h    # Sensor autodetection

  telemetry/                 # Outgoing telemetry
    telemetry.c / .h         # Telemetry scheduler
    frsky.c / .h             # FrSky output
    hott.c / .h              # HoTT output
    mavlink.c / .h           # MAVLink output
    ltm.c / .h               # LTM output
    mfd.c / .h               # MFD output
    smartport.c / .h         # SmartPort output
    nmea.c / .h              # NMEA output
    msp.c / .h               # MSP telemetry
    forward.c / .h           # Protocol forwarding
    position_estimation_log.c/.h

    mavlink/                 # MAVLink C library headers (v1, generated)
    blackbox/                # Blackbox logging

  startup/                   # MCU startup assembly (stm32f10x/f30x)
  target/                    # Board-specific configs (NAZE, BLUEPILL, CC3D, ...)
    <TARGET>/target.h        # Pin mappings, feature definitions
    <TARGET>/hardware_revision.c/.h
    <TARGET>/system_stm32f30x.c/.h
    stm32_flash_f103_*.ld    # Linker scripts for F103
    stm32_flash_f303_*.ld    # Linker scripts for F303

  vcp/                       # USB Virtual COM Port (STM32 USB-FS device driver)

src/test/                    # Unit tests (Google Test)
  unit/                      # Test cases (*_unittest.cc)
  Makefile                   # Test build system

lib/                         # Third-party libraries
  main/                      # Build dependencies
    CMSIS/                   # CMSIS core headers
    STM32F10x_StdPeriph_Driver/
    STM32F30x_StdPeriph_Driver/
    STM32_USB-FS-Device_Driver/
  test/                      # Test dependencies
    gtest/                   # Google Test framework

docs/                        # Documentation
  README.md                  # Documentation index
  overview.md                # Feature overview
  hardware-*.md              # Hardware guides
  install-*.md               # Installation guides
  configuration-*.md         # Configuration guides
  display-*.md               # Display guides
  assets/images/             # Images for docs

Makefile                     # Root build system
Vagrantfile                  # Vagrant dev environment (Ubuntu Trusty + GCC ARM)
```

### Key conventions

- The firmware is a fork of **Cleanflight** with antenna tracker logic added in `src/main/tracker/`.
- The entry point is `src/main/main.c` → calls `init()` (Cleanflight init), then `tracker_setup()`, then loops `tracker_loop()`.
- Each target board has a directory under `src/main/target/<TARGET>/` with `target.h` for pin mappings and feature definitions.
- All configuration parameters are in `master_t` struct in `config/config_master.h`, persisted to EEPROM.
- Features are bitmask flags in `features_e` enum in `config/config.h`.
- Protocol types are defined as `TP_*` constants in `tracker/config.h`.

---

## 3. Coding Style

### C

u360gts inherits Cleanflight style. The key rules:

| Rule | Convention |
|---|---|
| **Indentation** | 4 spaces, no tabs |
| **Brace style** | Linux/K&R — opening brace on same line for functions, after `)` for control flow |
| **Line endings** | LF only (no CRLF) |
| **Header guards** | Mix of `#pragma once` (Cleanflight files) and `#ifndef` guards (tracker files) — follow file's existing style |
| **Single-line blocks** | Braces are optional but preferred for clarity; most existing code uses braces |
| **Comments** | Block comments `/* */` for file headers, `//` for inline (inconsistent — follow existing pattern in each file) |

#### Naming conventions

| Element | Convention | Example |
|---|---|---|
| Types | `_t` suffix, snake_case | `positionVector_t`, `master_t` |
| Functions | `snake_case` | `tracker_setup()`, `calculatePID()` |
| Global variables | `camelCase` or `snake_case` | `masterConfig`, `targetPosition`, `homeSet` |
| Constants/defines | `UPPER_SNAKE_CASE` | `FEATURE_GPS`, `TP_MAVLINK` |
| Enum values | `UPPER_SNAKE_CASE` | `TP_SERVOTEST`, `OP_EXIT` |

#### Other conventions

- Include driver headers directly, not through HAL layers.
- Use `extern` for globals shared across files.
- Prefer `uint8_t`, `uint16_t`, `int32_t`, etc. for portability.
- Use `millis()` / `micros()` wrappers from `tracker/Arduino.c` for time (not `AP_HAL::millis()` like ArduPilot).
- Use `serialPrint()` / `printf()` for debug output (not `GCS_SEND_TEXT()`).
- Feature test with `feature(FEATURE_*)` / `featureSet()` / `featureClear()` macros.
- Protocol test with `PROTOCOL(TP_*)` / `ENABLE_PROTOCOL()` / `DISABLE_PROTOCOL()` macros.
- Use `STATE()` / `ENABLE_STATE()` / `DISABLE_STATE()` for runtime state flags.
- Hardware abstraction: driver files prefixed by platform (`_stm32f10x` / `_stm32f30x`).
- Always check `target.h` for board-specific pin mappings before adding hardware code.

---

## 4. Build System

u360gts uses a **Makefile**-based build system (inherited from Cleanflight). Key commands:

```sh
# Clone and build
git clone https://github.com/raul-ortega/u360gts.git
cd u360gts

# Build for a specific target (default: NAZE)
make TARGET=NAZE

# Build for other boards
make TARGET=BLUEPILL
make TARGET=SPRACINGF3
make TARGET=CC3D

# Compile-time options
make TARGET=NAZE OPTIONS="GPS TELEMETRY"

# List valid targets
make help

# Build unit tests
make test

# Clean
make clean

# Flash via UART (stm32flash)
make flash TARGET=NAZE SERIAL_DEVICE=/dev/ttyUSB0

# Flash via ST-Link (st-flash)
make st-flash TARGET=NAZE

# Static analysis
make cppcheck
```

### Supported targets

```
ALIENWIIF1 BLUEPILL ALIENWIIF3 CC3D CHEBUZZF3 CJMCU COLIBRI_RACE
EUSTM32F103RC MOTOLAB NAZE NAZE32PRO OLIMEXINO PORT103R RMDO
SPARKY SPRACINGF3 STM32F3DISCOVERY
```

**Important:** Never run `make` with `sudo`. Always call from the repository root.

---

## 5. Testing

All changes must be tested. u360gts has unit tests using **Google Test**.

### 5.1 Unit Tests (GTest)

Located in `src/test/unit/`. Tests are C++ files (`*_unittest.cc`) using Google Test.

```sh
# Run all tests
make test

# Run a specific test binary directly
./obj/test/<test_name>
```

Test structure:
```cpp
#include <gtest/gtest.h>
extern "C" {
#include "path/to/source.h"
}
TEST(TestSuite, TestName) {
    EXPECT_EQ(expected, actual);
}
```

### 5.2 What to test

- **Core tracker math**: `maths.c`, `gps_estimation.c`, `interpolation.c`
- **Protocol parsers**: frsky, mavlink, ltm, etc.
- **Config persistence**: EEPROM read/write
- **PID / NOPID control logic**

### 5.3 Manual testing

- Build for SITL (not available natively — test on real hardware or with a serial simulator).
- Verify with `u360gts-configurator` (cross-platform GUI) where possible.
- Test protocol auto-detection with known telemetry streams.

---

## 6. Documentation

- User documentation lives in `docs/`. When adding features, update or add relevant `.md` files.
- Configuration parameters are stored in `config/config_master.h` (the `master_t` struct) and should be documented in the CLI help (`io/serial_cli.c`).
- The `u360gts-configurator` GUI tool at https://github.com/raul-ortega/u360gts-configurator is the primary configuration interface — consider its parameter schema when adding new config fields.

---

## 7. Commit Messages

Follow conventional commit format:

```text
Subsystem: short description of the change

Optional longer description explaining the motivation,
what was changed, and why.
```

### Subsystem prefixes

Use the directory or module name as the prefix:

| Prefix | Area |
|---|---|
| `tracker/` | Core antenna tracker logic (`tracker/main.c`, protocol parsers, EPS) |
| `drivers/` | Hardware drivers (UART, I2C, SPI, PWM, ADC, sensors) |
| `config/` | Configuration, EEPROM, profiles |
| `flight/` | PID, IMU, navigation, mixer, failsafe |
| `io/` | Serial, CLI, display, GPS thread, beeper |
| `rx/` | Receiver protocols (SBUS, Spektrum, SUMD, etc.) |
| `sensors/` | Sensor abstraction (gyro, accel, compass, baro, battery) |
| `telemetry/` | Outgoing telemetry (FrSky, HoTT, MAVLink, NMEA, LTM) |
| `common/` | Shared utilities (maths, filter, encoding, printf) |
| `target/` | Board-specific config and linker scripts |
| `docs/` | Documentation |
| `Makefile` | Build system |
| `test/` | Unit tests |

### Rules

- Keep the first line under ~72 characters.
- **No merge commits** — always rebase.
- **No `fixup!` commits** — squash them.
- One logical change per commit. Split unrelated changes into separate commits.

### Examples

```text
tracker: add FrSky X SmartPort CRC fix option
drivers: fix UART2 baud rate on STM32F103
telemetry: forward MAVLink attitude data to NMEA output
config: persist easing_last_tilt across reboots
```

---

## 8. Pull Request Guidelines

### Before opening a PR

1. **Fork and branch**: Work on a feature branch in your fork, not on `master`.
2. **Rebase on master**: Ensure your branch is up to date.
3. **Build**: `make TARGET=<board>` — at least for the affected target(s).
4. **Run tests**: `make test` — all unit tests must pass.
5. **Check formatting**: Match the style of surrounding code (no blanket reformatting).
6. **Verify commit messages**: Every commit must follow the `Subsystem: description` format.

### PR description

- Clearly describe **what** the change does and **why**.
- Reference related issues (e.g., `Fixes #123`).
- Describe how it was tested (board, protocol, hardware setup).
- **Explicitly state that the contribution was AI-assisted** and describe the level of AI involvement.
- If the change affects configuration parameters, note them and any migration considerations.

### Review process

- u360gts is maintained by volunteers. Be patient.
- Respond to all review comments. If a reviewer asks for changes, make them and push updated commits.
- CI must pass before merge. There is no automated CI currently — testing evidence is the contributor's responsibility.
- Maintainers may ask you to split large PRs into smaller, focused ones.

---

## 9. Interacting with Maintainers & Developers

### Communication channels

| Channel | Purpose |
|---|---|
| [GitHub Issues](https://github.com/raul-ortega/u360gts/issues) | Bug reports and feature requests |
| [GitHub Pull Requests](https://github.com/raul-ortega/u360gts/pulls) | Code review and submission |
| [Facebook Group](https://www.facebook.com/groups/u360gts/) | User support and discussions |
| [RC Groups Thread](https://www.rcgroups.com/forums/showthread.php?2964122-u360gts-360%C2%B0-antenna-tracker) | Community discussions |

### Etiquette for AI-assisted contributions

1. **Transparency**: The human author must clearly state that AI tools were used in the PR description.
2. **Accountability**: The human submitting the PR is fully responsible for the code.
3. **Understanding**: The human author must understand every line of the submitted code and be able to explain it during review.
4. **Quality**: Do not submit low-quality AI-generated PRs. Quality over quantity.
5. **Respect expertise**: Maintainers have deep domain knowledge in embedded systems, STM32, and antenna tracking. Defer to their judgment.
6. **No AI in discussions**: Do not paste raw AI-generated responses into discussion threads or review comments.

---

## 10. What AI Should NOT Do

- **Do not create a PR that provides no real benefit** (e.g., resume padding, educational exercises that don't improve the codebase).
- **Do not fabricate**: Never invent APIs, registers, sensor IDs, or hardware interfaces that don't exist in the codebase. Always verify against actual source code.
- **Do not guess at hardware behavior**: If unsure about STM32 peripheral configuration, servo timing, or protocol parsing, flag it for human review.
- **Do not remove compile-time guards** (`#ifdef` blocks) — they exist for a reason (flash/RAM constraints).
- **Do not introduce platform-specific code** in shared files. Use the `_stm32f10x` / `_stm32f30x` suffixed driver pattern.
- **Do not modify third-party libraries** in `lib/` (CMSIS, STM32 StdPeriph, USB-FS driver, Google Test) — those are upstream dependencies.
- **Do not change parameter indices** in EEPROM structs. Existing indices are baked into user configurations.
- **Do not add unnecessary dependencies**: u360gts runs on constrained microcontrollers (64-256 KB flash). Every byte matters.
- **Do not generate large speculative refactors**: Focus on minimal, targeted, well-tested changes.
- **Do not remove or weaken existing tests** unless there is a clear, documented reason.
- **Do not move functions around without a goal**: Keep the original structure as much as possible.
- **Do not auto-generate commit messages**: Write meaningful messages that reflect the actual change.
- **Do not add comments on all functions/lines**: Document only what was changed and useful for future reading.
- **Do not duplicate PRs**: Check if a PR was already opened for a feature/bugfix before creating one.
