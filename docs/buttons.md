# MENU and HOME buttons

The tracker has two physical buttons for on-screen menu navigation and setting the HOME position.

## Button connections

| Button | GPIO | Pin | Active |
|---|---|---|---|
| MENU | GPIOB | Pin 8 | Low (pressed = 0) |
| HOME | GPIOB | Pin 9 | Low (pressed = 0) |

Buttons have internal debouncing via firmware. The debounce threshold is configured with `min_logic_level` (default 50).

## MENU button

The MENU button controls the on-screen display menu and page navigation.

### Normal mode (tracking active)

| Action | Result |
|---|---|
| Short press | Cycle through display pages |
| Long press (> 1.5s) | Enter menu mode |
| Long press (in menu) | Exit menu mode |

### Menu mode

When the display shows the menu:

| Action | Result |
|---|---|
| Short press | Move cursor to next option |
| Long press (> 1.5s) | Exit menu mode |

### Offset trim mode

When offset trim is active:

| Action | Result |
|---|---|
| Short press | Decrease offset trim by 1 unit |

## HOME button

The HOME button sets or resets the tracker's home position.

### Normal mode

| Action | Result |
|---|---|
| Short press | Set HOME position (telemetry GPS or local GPS) |
| Long press (> 2s) | Reset HOME position |

### Menu mode

| Action | Result |
|---|---|
| Short press | Select / enter highlighted menu option |

### Offset trim mode

When offset trim is active:

| Action | Result |
|---|---|
| Short press | Increase offset trim by 1 unit |
| Long press (> 2s) | Disable offset trim mode |

## Setting HOME position

When the HOME button is pressed:
1. If telemetry data with GPS coordinates is available, home is set from the aircraft's reported position
2. If no telemetry GPS is available but a local GPS is fitted, home is set from the local GPS position
3. The home position is stored in EEPROM (`restore_last_home` must be enabled for persistence across reboots)

## Menu structure

The on-screen menu provides access to:

| Menu option | Description |
|---|---|
| Calibrate | Magnetometer calibration and pan calibration |
| Battery | Battery monitoring settings (feature toggle) |
| GPS | Local GPS settings |
| Telemetry | Telemetry protocol and baud rate selection |
| EPS | Estimated Position System settings (mode, gains, frequency) |
| Easing | Servo easing curve settings |
| Exit | Return to normal operation |

## Configuration

| Parameter | Default | Description |
|---|---|---|
| `min_logic_level` | 50 | Button debounce threshold (higher = longer press needed) |
