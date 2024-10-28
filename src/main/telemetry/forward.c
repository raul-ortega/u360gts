#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef TELEMETRY

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"


#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"

#include "io/serial.h"
#include "io/rc_controls.h"
#include "io/gps.h"

#include "rx/rx.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "telemetry/telemetry.h"
#include "telemetry/forward.h"

static serialPort_t *forwardPort = NULL;
static serialPortConfig_t *portConfig;

#define FORWARD_BAUDRATE 57600
#define FORWARD_INITIAL_PORT_MODE MODE_RXTX

static telemetryConfig_t *telemetryConfig;
static bool forwardTelemetryEnabled =  false;
static portSharing_e forwardPortSharing;

#define CYCLETIME             125

static uint32_t lastCycleTime = 0;
static uint8_t cycleNum = 0;

void initForwardTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_FORWARD);
    forwardPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_FORWARD);
}

void freeForwardTelemetryPort(void)
{
    closeSerialPort(forwardPort);
    forwardPort = NULL;
    forwardTelemetryEnabled = false;
}

void configureForwardTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    forwardPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_FORWARD, NULL, FORWARD_BAUDRATE, FORWARD_INITIAL_PORT_MODE, telemetryConfig->telemetry_inversion ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
    if (!forwardPort) {
        return;
    }

    forwardTelemetryEnabled = true;
}

void checkForwardTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(forwardPortSharing);

    if (newTelemetryEnabledValue == forwardTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureForwardTelemetryPort();
    else
        freeForwardTelemetryPort();
}

void handleForwardTelemetry(rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
    if (!forwardTelemetryEnabled) {
        return;
    }

    while (serialRxBytesWaiting(forwardPort) > 0) {
            uint8_t c = serialRead(forwardPort);
            evaluateOtherData(forwardPort,c);
    }


}

void forwardTelemetry(uint8_t c){
    serialWrite(forwardPort, c);
}

bool forwardEnabled(void){
    return(forwardTelemetryEnabled);
}


#endif

