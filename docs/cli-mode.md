# CLI mode

u360gts includes a command-line interface accessible over the serial port (UART1) for configuring all settings.

## Entering CLI mode

Send three consecutive uppercase `R` characters (`RRR`) over the serial connection. The tracker will respond with:

```
Entering CLI Mode, type 'exit' to return, or 'help'
#
```

The `#` prompt indicates you are in CLI mode.

## Available commands

| Command | Description |
|---|---|
| `help` | Show available commands |
| `exit` | Leave CLI mode (unsaved changes are lost) |
| `save` | Save settings to EEPROM and reboot |
| `defaults` | Reset all settings to factory defaults and reboot |
| `set` | List all parameters with current values |
| `set *` | List all parameters with min/max ranges |
| `set <name>=<value>` | Change a parameter value |
| `get <name>` | Search for a parameter by name |
| `feature` | List currently enabled features |
| `feature list` | List all available features |
| `feature <NAME>` | Enable a feature |
| `feature -<NAME>` | Disable a feature |
| `dump` | Show full configuration |
| `serial` | Configure serial ports |
| `status` | Show system status |
| `version` | Show firmware version |
| `calibrate mag` | Start magnetometer calibration |
| `calibrate pan` | Start pan servo calibration |
| `heading <deg>` | Move pan servo to a specific angle (0-360) |
| `tilt <deg>` | Move tilt servo to a specific angle (0-90) |
| `boot mode` | Reboot into bootloader for firmware flashing |
| `rssi` | Show RSSI status |

## Saving settings

Use `save` to write all changes to EEPROM and reboot the tracker. Changes are **not** saved automatically.

## Restoring defaults

Use `defaults` to reset the EEPROM to factory defaults and reboot. All custom settings will be lost.

## Features

Enable or disable features with the `feature` command.

| Feature | Bit | Description |
|---|---|---|
| `VBAT` | 0 | Battery voltage monitoring |
| `SERVO_TILT` | 1 | Tilt servo enabled |
| `SOFTSERIAL` | 2 | Software serial ports |
| `GPS` | 3 | Local GPS module |
| `SONAR` | 4 | Sonar sensor |
| `TELEMETRY` | 5 | Telemetry output |
| `CURRENT_METER` | 6 | Current sensor |
| `DISPLAY` | 7 | OLED display |
| `BLACKBOX` | 8 | Blackbox logging |
| `EASING` | 9 | Servo easing curves |
| `NOPID` | 10 | NoPID control instead of PID |
| `DEBUG` | 11 | Debug mode |
| `EPS` | 12 | Estimated Position System |
| `RSSI_ADC` | 13 | RSSI via ADC |
| `AUTODETECT` | 14 | Auto-detect telemetry protocol |

Usage: `feature GPS` enables GPS, `feature -GPS` disables it.

## Parameters

Use `set` to list all parameters or `set <name>=<value>` to change one. Partial names work as a search (e.g. `set pan` lists all parameters containing "pan").

### Tracking

| Parameter | Range | Default | Description |
|---|---|---|---|
| `offset` | -360 - 360 | 0 | Pan offset in degrees |
| `offset_trim` | -20 - 20 | 0 | Fine trim for pan offset |
| `start_tracking_distance` | 0 - 100 | 0 | Min distance (m) to start tracking |
| `start_tracking_altitude` | 0 - 100 | 0 | Min altitude (m) to start tracking |
| `min_pan_speed` | 0 - 100 | 0 | Minimum pan servo speed |
| `looptime` | 0 - 9000 | 3500 | Main loop time in microseconds |

### Pan servo

| Parameter | Range | Default | Description |
|---|---|---|---|
| `pan_pin` | 0 - 7 | 0 | PWM pin for pan servo |
| `pan0` | 0 - 3000 | 1500 | Pan pulse at 0 degrees |
| `pan0_calibrated` | 0 - 1 | 0 | Whether pan is calibrated |
| `pan_calibration_pulse` | 0 - 3000 | 0 | Calibration pulse width |
| `pan_inverted` | OFF / ON | OFF | Invert pan direction |

### Tilt servo

| Parameter | Range | Default | Description |
|---|---|---|---|
| `tilt_pin` | 0 - 7 | 0 | PWM pin for tilt servo |
| `tilt0` | 0 - 3000 | 1500 | Tilt pulse at horizon |
| `tilt90` | 0 - 3000 | 2000 | Tilt pulse at 90 degrees |
| `tilt_max_angle` | 0 - 90 | 90 | Maximum tilt angle |

### PID controller

| Parameter | Range | Default | Description |
|---|---|---|---|
| `p` | 0 - 50000 | 40 | Proportional gain |
| `i` | 0 - 50000 | 10 | Integral gain |
| `d` | 0 - 50000 | 30 | Derivative gain |
| `max_pid_error` | 0 - 100 | 0 | Max error for integral |
| `max_pid_accumulator` | 0 - 50000 | 0 | Max integral accumulation |
| `max_pid_gain` | 0 - 5000 | 0 | Max output gain |
| `pid_divider` | 0 - 255 | 0 | PID frequency divider |

### NoPID control

| Parameter | Range | Default | Description |
|---|---|---|---|
| `nopid_min_delta` | 0 - 100 | 0 | Minimum angle delta |
| `nopid_map_angle` | 0 - 180 | 0 | Angle mapping range |
| `nopid_max_speed` | 0 - 500 | 0 | Maximum servo speed |

### Telemetry

| Parameter | Range | Default | Description |
|---|---|---|---|
| `telemetry_protocol` | 1 - 1024 | 1 | Protocol bitmask |
| `telemetry_baud` | list | 115200 | Telemetry baud rate |
| `telemetry_provider` | NONE / DIY_GPS / INAV / APM10 | NONE | Telemetry data provider |
| `telemetry_home` | DEFAULT / AUTO | DEFAULT | Home position source |
| `telemetry_min_sats` | 0 - 20 | 6 | Min satellites for telemetry |
| `telemetry_inversion` | OFF / ON | OFF | Invert telemetry signal |

### GPS

| Parameter | Range | Default | Description |
|---|---|---|---|
| `gps_baudrate` | list | 9600 | GPS serial baud rate |
| `gps_provider` | NMEA / UBLOX | NMEA | GPS module type |
| `gps_sbas_mode` | list | AUTO | SBAS mode |
| `gps_auto_config` | OFF / ON | ON | Auto-configure GPS |
| `gps_auto_baud` | OFF / ON | ON | Auto-detect GPS baud |
| `gps_min_sats` | 4 - 20 | 6 | Min sats for auto home |
| `update_home_by_local_gps` | OFF / ON | OFF | Update home from local GPS |

### Battery

| Parameter | Range | Default | Description |
|---|---|---|---|
| `battery_capacity` | 0 - 20000 | 0 | Battery capacity in mAh (0 = disabled) |
| `vbat_scale` | 0 - 255 | 110 | Voltage divider scale |
| `vbat_max_cell_voltage` | 10 - 50 | 43 | Max voltage per cell (0.1V) |
| `vbat_min_cell_voltage` | 10 - 50 | 33 | Min voltage per cell (0.1V) |
| `vbat_warning_cell_voltage` | 10 - 50 | 35 | Warning voltage per cell (0.1V) |

### Magnetometer

| Parameter | Range | Default | Description |
|---|---|---|---|
| `mag_hardware` | 0 - MAG_MAX | 0 | Magnetometer type |
| `mag_declination` | -18000 - 18000 | 0 | Magnetic declination (degrees * 100) |
| `mag_calibrated` | 0 - 1 | 0 | Whether magnetometer is calibrated |

### Easing

| Parameter | Range | Default | Description |
|---|---|---|---|
| `easing` | 1 - 4 | 1 | Easing curve type |
| `easing_steps` | 0 - 100 | 5 | Easing steps |
| `easing_min_angle` | 1 - 10 | 1 | Min angle for easing |
| `easing_millis` | 1 - 100 | 10 | Easing update interval |

### EPS (Estimated Position System)

| Parameter | Range | Default | Description |
|---|---|---|---|
| `eps` | 1 - 3 | 1 | EPS mode |
| `eps_distance_gain` | 1 - 1000 | 200 | Distance estimation gain |
| `eps_heading_gain` | 1 - 1000 | 100 | Heading estimation gain |
| `eps_speed_gain` | 1 - 1000 | 50 | Speed estimation gain |
| `eps_frequency` | 100 - 1000 | 300 | EPS update frequency |
| `eps_interpolation` | OFF / ON | OFF | Enable position interpolation |
| `eps_interpolation_points` | 2 - 10 | 2 | Interpolation sample count |
| `max_speed_filter` | 0 - 255 | 0 | Speed filter threshold |

### Other

| Parameter | Range | Default | Description |
|---|---|---|---|
| `emf_avoidance` | OFF / ON | OFF | EMF interference avoidance |
| `min_logic_level` | 0 - 255 | 50 | Button debounce threshold |
| `servo_pwm_rate` | 50 - 498 | 50 | Servo PWM frequency |
| `init_servos` | 0 - 1 | 0 | Initialize servos at startup |
| `rssi_channel` | 0 - 8 | 0 | RSSI channel |
| `rssi_scale` | 0 - 255 | 0 | RSSI scale |
| `rssi_zoom` | 1 - 100 | 0 | RSSI zoom |
| `align_gyro` | list | DEFAULT | Gyro alignment |
| `align_mag` | list | DEFAULT | Magnetometer alignment |
| `align_board_roll` | -180 - 360 | 0 | Board roll alignment |
| `align_board_pitch` | -180 - 360 | 0 | Board pitch alignment |
| `align_board_yaw` | -180 - 360 | 0 | Board yaw alignment |
| `gyro_lpf` | 0 - 256 | 0 | Gyro low-pass filter |

## Telemetry protocol values

Set `telemetry_protocol` to the sum of desired protocol bit values:

| Protocol | Bit | Value |
|---|---|---|
| SERVOTEST | 0 | 1 |
| CALIBRATING_MAG | 1 | 2 |
| MFD | 2 | 4 |
| GPS_TELEMETRY | 3 | 8 |
| MAVLINK | 4 | 16 |
| RVOSD | 5 | 32 |
| FRSKY_D | 6 | 64 |
| FRSKY_X | 7 | 128 |
| LTM | 8 | 256 |
| PITLAB | 9 | 512 |
| CROSSFIRE | 10 | 1024 |
