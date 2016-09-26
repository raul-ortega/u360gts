
#include "rx/rx.h"

#ifndef TELEMETRY_NMEA_
#define TELEMETRY_NMEA_

void handleNMEATelemetry(void);
void checkNMEATelemetryState(void);

void initNMEATelemetry(telemetryConfig_t *telemetryConfig);
void configureNMEATelemetryPort(void);
void freeNMEATelemetryPort(void);

#endif /* TELEMETRY_NMEA_ */
