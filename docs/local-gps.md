# Local GPS - Installation and Settings

u360gts can use a local GPS module connected to a serial port to determine the tracker's own position and set the HOME point automatically.

## Installation

Connect a GPS module to USART2 (Serial port 2) on the flight controller:

| GPS Pin | Connection |
|---|---|
| VCC | 3.3V or 5V (depending on module) |
| GND | Ground |
| TX | RX pin of USART2 |
| RX | TX pin of USART2 (optional, needed for uBlox config) |

Supported GPS protocols:
- **NMEA** - works with most standard GPS modules
- **uBlox** - binary protocol with automatic configuration

## Enabling GPS

1. Enable the GPS feature in CLI:
   ```
   feature GPS
   ```
2. Configure the GPS provider:
   ```
   set gps_provider = NMEA
   ```
   or
   ```
   set gps_provider = UBLOX
   ```
3. Set the baud rate:
   ```
   set gps_baudrate = 115200
   ```
4. Save and reboot:
   ```
   save
   ```

## Settings

| Parameter | Default | Description |
|---|---|---|
| `gps_provider` | NMEA | NMEA or UBLOX protocol |
| `gps_baudrate` | 9600 | Serial baud rate for GPS |
| `gps_sbas_mode` | AUTO | SBAS mode (AUTO, EGNOS, WAAS, MSAS, GAGAN) |
| `gps_auto_config` | ON | Auto-configure uBlox GPS on startup |
| `gps_auto_baud` | ON | Auto-detect GPS baud rate |
| `gps_min_sats` | 6 | Minimum satellites to auto-set home |
| `update_home_by_local_gps` | OFF | Continuously update home from GPS |
| `gps_home_beeper` | ON | Beep when home is set |

### `update_home_by_local_gps`

When enabled, the tracker will automatically update the HOME position from the local GPS as the tracker moves. This is useful for mobile trackers (e.g., car-mounted or boat-mounted).

The home position will only update when:
- GPS has a 3D fix
- At least `gps_min_sats` satellites are visible

## Auto home setting

The local GPS can automatically set the HOME position:
- With at least `gps_min_sats` satellites (default 6) when `update_home_by_local_gps` is active
- With at least 4 satellites when the HOME button is pressed

When home is set by local GPS, the tracker stores the position and can restore it after power loss if `restore_last_home` is enabled.
