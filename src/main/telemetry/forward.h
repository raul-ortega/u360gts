#include "rx/rx.h"

#ifndef TELEMETRY_FORWARD_
#define TELEMETRY_FORWARD_

void handleForwardTelemetryvoid(rxConfig_t *rxConfig, uint16_t deadband3d_throttle);
void checkForwardTelemetryState(void);

void initForwardTelemetry(telemetryConfig_t *telemetryConfig);
void configureForwardTelemetryPort(void);
void freeForwardTelemetryPort(void);
bool forwardEnabled(void);

#endif /* TELEMETRY_FORWARD_ */
