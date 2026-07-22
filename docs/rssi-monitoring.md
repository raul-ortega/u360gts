# RSSI monitoring

u360gts can monitor the received signal strength (RSSI) from your video or RC receiver and display it on the OLED screen. RSSI can be read via an ADC pin or from an RC channel.

## Enabling RSSI

RSSI reading is available through two mutually exclusive methods:

| Method | Configuration | Priority |
|---|---|---|
| RC Channel (PWM/PPM) | Set `rssi_channel` to a value 1-18 | Higher (takes precedence) |
| ADC | Enable `feature RSSI_ADC` | Lower (only used when `rssi_channel = 0`) |

If `rssi_channel > 0`, the ADC method is ignored. If both are unconfigured, RSSI is not read.

## RSSI via RC Channel

When `rssi_channel` is set to a value between 1 and 18, RSSI is read from the corresponding RC receiver channel:

```
set rssi_channel = 8
save
```

The channel value (1000-2000 µs) is mapped to a 0-1023 range:
- 1000 µs = 0 (minimum RSSI)
- 2000 µs = 1023 (maximum RSSI)

Optionally, the channel can be inverted with `rssi_ppm_invert` (though this parameter is not directly settable via CLI — configure it through the configurator GUI if needed).

## RSSI via ADC

When `feature RSSI_ADC` is enabled and `rssi_channel = 0`, RSSI is read from the RSSI ADC pin:

```
feature RSSI_ADC
save
```

The raw ADC value (12-bit, 0-4095) is divided by `rssi_scale` to obtain a percentage, then averaged over 16 samples for noise reduction.

### ADC RSSI Target Pins

The RSSI ADC pin varies by target. Check your target's `target.h` for the specific pin assignment.

## Parameters

| Parameter | Default | Range | Description |
|---|---|---|---|
| `rssi_channel` | 0 | 0 - 18 | RC channel for RSSI (0 = disabled, uses ADC instead) |
| `rssi_scale` | 30 | 0 - 255 | ADC RSSI divisor (raw ADC / scale = percentage) |
| `rssi_zoom` | 35 | 1 - 100 | Display zoom threshold for low-RSSI detail |

### `rssi_channel`

1-based index of the RC channel (from your receiver) that carries the RSSI signal. Set to 0 to use the ADC method instead.

### `rssi_scale`

Only used in ADC mode. The raw ADC reading is divided by this value to obtain a percentage. With the default scale of 30, a raw ADC reading of 3000 gives 100% RSSI.

- Higher `rssi_scale` = lower reported RSSI for the same input voltage
- Lower `rssi_scale` = higher reported RSSI

### `rssi_zoom`

Controls a "zoomed" RSSI bar on the display that provides higher resolution at low signal strengths. When the RSSI percentage is below `rssi_zoom`, a second percentage bar is drawn with the range `[0, rssi_zoom]` mapped to `[0, 100]`. This makes small changes at weak signal levels more visible.

**Example with `rssi_zoom = 35`:**

| Actual RSSI | Normal bar | Zoomed bar |
|---|---|---|
| 10% | 10% filled | 28.6% filled |
| 20% | 20% filled | 57.1% filled |
| 35% | 35% filled | 100% filled |
| 70% | 70% filled | 100% filled |

Above the zoom threshold, the zoomed bar stays at 100% and only the normal bar provides useful information.

## Display

RSSI appears on the **Battery page** (PAGE_BATTERY) of the OLED display when either `rssi_channel > 0` or `feature RSSI_ADC` is enabled:

- Percentage text: `Value: XX %`
- Bar 1: Full range 0-100% horizontal bar
- Bar 2: Zoomed bar (detail below `rssi_zoom` threshold)

The battery page automatically appears in the page cycle when RSSI is configured.

## CLI Command

The `rssi` command shows the current raw RSSI value and percentage:

```
# rssi
rssi: 512 50 %
```

- First number: raw RSSI value (0-1023)
- Second number: calculated percentage (0-100)

## RSSI Percentage

The percentage is calculated as a simple linear mapping:

```
percentage = (rssi * 100) / 1023
```

[<< Go back](README.md)
