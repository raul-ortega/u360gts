# Battery monitoring

u360gts can monitor battery voltage and current using the on-board ADC.

## Enabling battery monitoring

- **Voltage monitoring**: Enable `feature VBAT` (enabled by default on most targets)
- **Current monitoring**: Enable `feature CURRENT_METER`

## Voltage monitoring

The ADC measures the battery voltage through a voltage divider connected to the flight controller.

### ADC pins by target

| Target | GPIO | Pin | ADC Channel |
|---|---|---|---|
| NAZE / BLUEPILL / PORT103R / EUSTM32F103RC | PA4 | A4 | ADC1_IN4 |
| ALIENWIIF3 / SPARKY / SPRACINGF3 / RMDO | PA4 | A4 | ADC1_IN1 |
| CC3D | PA0 | A0 | ADC1_IN0 |
| MOTOLAB | PA5 | A5 | ADC1_IN2 |
| CHEBUZZF3 / COLIBRI_RACE / STM32F3DISCOVERY | PC0 | A0 | ADC1_IN6 |

### Settings

| Parameter | Default | Range | Description |
|---|---|---|---|
| `vbat_scale` | 110 | 0 - 255 | Voltage divider scale factor |
| `vbat_max_cell_voltage` | 43 | 10 - 50 (0.1V) | Max voltage per cell (used for cell count detection) |
| `vbat_min_cell_voltage` | 33 | 10 - 50 (0.1V) | Critical voltage per cell |
| `vbat_warning_cell_voltage` | 35 | 10 - 50 (0.1V) | Warning voltage per cell |

### Cell count detection

On battery connection, the cell count is auto-detected:
```
cells = (measured_voltage / vbat_max_cell_voltage) + 1
```
Detection supports 1S to 8S packs. The warning and critical thresholds are calculated as `cells * per_cell_voltage`.

### Voltage states

| State | Condition | Behavior |
|---|---|---|
| BATTERY_OK | Above warning threshold | Normal operation |
| BATTERY_WARNING | Below `vbat_warning_cell_voltage` per cell | Low battery beeper pattern |
| BATTERY_CRITICAL | Below `vbat_min_cell_voltage` per cell | Critical battery beeper pattern |
| BATTERY_NOT_PRESENT | Below 10mV | No battery detected |

100mV of hysteresis prevents rapid state toggling.

### Voltage percentage

A linear percentage is calculated from the voltage range:
```
percentage = (vbat - min_voltage) * 100 / (max_voltage - min_voltage)
```

## Current monitoring

| Type | Description |
|---|---|
| ADC | Reads from a current sensor via ADC |
| Virtual | Estimates current from throttle position |

### Settings

| Parameter | Default | Range | Description |
|---|---|---|---|
| `battery_capacity` | 0 | 0 - 20000 mAh | Battery capacity (0 = capacity tracking disabled) |

When `battery_capacity` is set, the tracker tracks mAh consumed and calculates remaining capacity percentage.

## Display

When a display is connected, battery information appears on the battery page:
- Voltage per cell and total voltage
- Cell count
- Voltage percentage bar
- Current draw (amperage)
- mAh consumed
- RSSI

## Telemetry output

Battery data is transmitted over supported telemetry protocols:

| Protocol | Data sent |
|---|---|
| FrSky | Per-cell voltage, total voltage, capacity % as RPM |
| SmartPort | Total voltage as VFAS |
| HoTT | Main voltage, battery 1 voltage, alarm flags |
| LTM | Voltage (mV), current (mA), mAh drawn |
