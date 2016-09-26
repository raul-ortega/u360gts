#include "rx/rx.h"

#ifndef TELEMETRY_MAVLINK_
#define TELEMETRY_MAVLINK_

void handleMAVLINKTelemetry(void);
void checkMAVLINKTelemetryState(void);

void initMAVLINKTelemetry(telemetryConfig_t *telemetryConfig);
void configureMAVLINKTelemetryPort(void);
void freeMAVLINKTelemetryPort(void);

#endif /* TELEMETRY_MAVLINK_ */
