
//#include "rx/rx.h"

#ifndef TELEMETRY_POSEST_
#define TELEMETRY_POSEST_

void handlePOSESTTelemetry(void);
void checkPOSESTTelemetryState(void);

void initPOSESTTelemetry(telemetryConfig_t *telemetryConfig);
void configurePOSESTTelemetryPort(void);
void freePOSESTTelemetryPort(void);

#endif /* TELEMETRY_POSEST_ */
