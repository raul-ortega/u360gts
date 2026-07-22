# EPS (Estimated Position System)

> **DEPRECATED** — This feature is deprecated and will be removed in future versions. Use the [NOPID control system](configuration-nopid-system.md) instead, which provides simpler and more robust tracking without needing position estimation.

## Overview

The Estimated Position System (EPS) is a dead-reckoning system that estimates the aircraft's current position between telemetry updates. When telemetry arrives at a low rate or is temporarily lost, EPS extrapolates from the last known position, speed, heading, and elapsed time using great-circle navigation.

EPS is controlled by `FEATURE_EPS` and is **disabled by default**.

## EPS Modes

| Mode | Display Name | Runs on telemetry | Runs periodically | Uses interpolation | Behavior |
|---|---|---|---|---|---|
| 0 | DISABLED | - | - | - | EPS off |
| 1 | MODE 1 | Yes | No | No | Simple distance scaling from last known position |
| 2 | MODE 2 | No | Yes | Yes | Full dead reckoning at `eps_frequency` interval |
| 3 | MODE 1+2 | Yes | Yes | Yes | Both: updates on telemetry AND periodically |

- **Mode 1:** Takes the last known distance to target and scales it by the distance gain. Does not estimate between telemetry packets. Updates only when new telemetry arrives.
- **Mode 2:** Full dead-reckoning using speed × time to estimate distance traveled. Updates at the configured `eps_frequency`. Uses Lagrange interpolation to estimate heading and speed changes if enabled.
- **Mode 3:** Combines both approaches — runs estimation immediately on each new telemetry packet AND periodically between packets for the most responsive tracking.

## How EPS Works

EPS processes telemetry data in the main tracking loop:

1. When a new telemetry packet arrives, the position, speed and heading are loaded into `targetCurrent`
2. If `max_speed_filter` is set and the reported speed exceeds the threshold, the packet is discarded (prevents GPS glitches from causing wild movements)
3. If EPS is enabled (modes 1 or 3), `calcEstimatedPosition()` runs immediately
4. In the main loop (modes 2 or 3), `calcEstimatedPosition()` runs at the configured `eps_frequency` interval
5. The estimated position is computed via great-circle navigation from the last known position using the estimated distance and heading
6. `targetPosition.distance` and `targetPosition.heading` are recalculated from the estimated coordinates for use by the servo control

### Position Estimation Algorithm

The estimated position is calculated using the **direct great-circle problem**: given a starting point (last known lat/lon), a distance, and a bearing, compute the destination point.

```
angularDistance = estimatedDistance / earthRadius
estimatedLat = asin(sin(lat) * cos(angularDistance) + cos(lat) * sin(angularDistance) * cos(heading))
estimatedLon = lon + atan2(sin(heading) * sin(angularDistance) * cos(lat), cos(angularDistance) - sin(lat) * sin(estimatedLat))
```

### Lagrange Interpolation

When interpolation is enabled (`eps_interpolation = ON`), EPS uses a Lagrange polynomial to predict heading and speed changes based on recent telemetry trends.

The system stores the last `eps_interpolation_points` (2-10) telemetry deltas (change in heading and speed between consecutive packets). At each estimation cycle, it evaluates the Lagrange polynomial at the current time to get the expected delta, which is scaled by the heading and speed gains and applied to the current values.

Interpolation only runs when enough points have been collected to fill the buffer.

## Parameters

| Parameter | Default | Range | Description |
|---|---|---|---|
| `eps` | 1 | 1 - 3 | EPS mode selection |
| `eps_distance_gain` | 100 | 1 - 1000 | Distance estimation gain (%) |
| `eps_heading_gain` | 50 | 1 - 1000 | Heading interpolation gain (%) |
| `eps_speed_gain` | 60 | 1 - 1000 | Speed interpolation gain (%) |
| `eps_frequency` | 250 | 100 - 1000 | Estimation update interval (ms) |
| `eps_interpolation` | OFF | OFF / ON | Enable Lagrange interpolation |
| `eps_interpolation_points` | 3 | 2 - 10 | Number of points for interpolation |
| `max_speed_filter` | 0 | 0 - 255 | Speed threshold filter (0 = disabled) |

### Gain Tuning

The gains act as confidence multipliers on the estimation:

- **`eps_distance_gain`**: In mode 1, scales the last known distance. In modes 2/3, scales the dead-reckoned distance (speed × time). Default 100 (100%).
- **`eps_heading_gain`**: Scales the interpolated heading delta. Only used when interpolation is enabled. Lower values make the estimated heading more conservative. Default 50 (50%).
- **`eps_speed_gain`**: Scales the interpolated speed delta. Only used when interpolation is enabled. Default 60 (60%).

### `max_speed_filter`

When set to a non-zero value, any telemetry packet reporting a speed equal to or greater than the threshold is discarded. This prevents spurious GPS readings (e.g., position jumps that imply impossible speeds) from corrupting the estimation. Default 0 (disabled).

### `eps_frequency`

Controls how often the dead-reckoning estimation runs in modes 2 and 3. Lower values = more frequent updates but higher CPU load. Default 250 ms (4 Hz).

## Configuration

### Enable EPS

```
feature EPS
set eps = 2
save
```

### Disable EPS

```
feature -EPS
save
```

[<< Go back](README.md)
