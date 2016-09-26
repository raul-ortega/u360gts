/*
 * MFD AAT Control Protocol


(1) Serial protocol parameter:
TTL level, baudrate 19200 bps (default) / 38400bps
8 data bits, 1 stop bit, no parity

(2) control command

1) Test command
Send char 'N' to point north with elevation angle of 0 degrees
Send char 'E' to point deast with elevation 30 degrees
Send char 'S' to point south with elevation angle of 0 degrees
Send char 'W' to point west with elevation angle of 60 degrees

2) Standby command
Send char '#' to tell the AAT that coordinates are not set. This allows you manual rotation of the AAT.

3) Coordinates set command
Send char 'X' to tell the AAT that coordinates are set. Tracking will start if object is more than 10m away from AAT.

4) Link failure, enter offline mode
Send char '@' to tell the AAT that there is a connection failure. This allows you manual rotation of the AAT.

5) direction and elevation control commands
'D' + distance + 'H' + height + 'A' + target azimuth + '*' + checksum +

D: distance in meters - positive values only INTEGER
H: height relative to start point - positive/negative height possible, but minimal antenna angle is limited to 0° INTEGER
A: azimuth in degrees 0 - 359*
Checksum: unsigned int 8bit, Sum up all chars of the "D***H***A*" string. Intermediate overflow discarded.
: Newline \n, ASCII code 0x0A

Example: D1001H120A265*??\n
distance: 1001m
height: 120m
azimuth: 265°
 */
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
#include "telemetry/mfd.h"
#include "tracker/defines.h"

static serialPort_t *mfdPort = NULL;
static serialPortConfig_t *portConfig;

#define MFD_BAUDRATE 9600
#define MFD_INITIAL_PORT_MODE MODE_TX

static telemetryConfig_t *telemetryConfig;
static bool mfdTelemetryEnabled =  false;
static portSharing_e mfdPortSharing;

bool TELEMETRY_FIX = false;
extern bool trackingStarted;
extern bool homeSet;
extern positionVector_t targetPosition;

uint16_t distance = 0;
uint16_t altitude = 0;
uint16_t azimuth = 0;

#define CYCLETIME            200

// Official data IDs
#define ID_DISTANCE			  68
#define ID_HEIGHT	 		  72
#define ID_AZIMUTH			  65
#define ID_NOFIX	          35
#define ID_FIX			      88
#define ID_CHECKSUM			  42

uint8_t mfd_checksum=0;

static uint32_t lastCycleTime = 0;
static uint8_t cycleNum = 0;


static void sendTelemetryTail(void);
static void serializeHeader(uint8_t data);
static void serialize16(int16_t data,bool sum);
void sendNoFix();
void sendFix();
void sendData();
void sendDistance();
void sendHeigth();
void sendAzimuth();
void sendChecksum();


static void sendTelemetryTail(void)
{
    serialWrite(mfdPort, '\n');
}

static void serializeHeader(uint8_t data)
{
	serialWrite(mfdPort, data);
	if(data!='*') mfd_checksum+=data;
}

static void serialize16(int16_t data,bool sum){
	char temp[5];
	int i=0;
	sprintf(temp, "%u\0", data);
	while(temp[i]!='\0')
	{
		if(sum) mfd_checksum += temp[i];
		serialWrite(mfdPort, temp[i++]);
	}
}

void sendNoFix()
{
	serialWrite(mfdPort,ID_NOFIX);
}
void sendFix()
{
	serialWrite(mfdPort,ID_FIX);
}

void sendData()
{
    sendDistance();
    sendHeigth();
    sendAzimuth();
}

void sendDistance()
{
    serializeHeader(ID_DISTANCE);
    serialize16(distance,true);
}

void sendHeigth()
{
    serializeHeader(ID_HEIGHT);
    serialize16(altitude,true);
}
void sendAzimuth()
{
    serializeHeader(ID_AZIMUTH);
    serialize16(azimuth,true);
}
void sendChecksum()
{
	serializeHeader(ID_CHECKSUM);
	serialize16(mfd_checksum,false);
}

void initMFDTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MFD);
    mfdPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_MFD);
}

void freeMFDTelemetryPort(void)
{
    closeSerialPort(mfdPort);
    mfdPort = NULL;
    mfdTelemetryEnabled = false;
}

void configureMFDTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    mfdPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MFD, NULL, baudRates[portConfig->telemetry_baudrateIndex], MFD_INITIAL_PORT_MODE, telemetryConfig->telemetry_inversion ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
    if (!mfdPort) {
        return;
    }

    mfdTelemetryEnabled = true;
}

bool hasEnoughTimeLapsedSinceLastTelemetryTransmission_mfd(uint32_t currentMillis)
{
    return currentMillis - lastCycleTime >= CYCLETIME;
}

void checkMFDTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(mfdPortSharing);

    if (newTelemetryEnabledValue == mfdTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureMFDTelemetryPort();
    else
        freeMFDTelemetryPort();
}

void handleMFDTelemetry(void)
{
    if (!mfdTelemetryEnabled) {
        return;
    }

    uint32_t now = millis();

    if (!hasEnoughTimeLapsedSinceLastTelemetryTransmission_mfd(now)) {
        return;
    }

	lastCycleTime = now;

	cycleNum++;

    if(!homeSet && !trackingStarted) {
    	TELEMETRY_FIX=false;
    	sendNoFix();
    	return;
    }

    if(homeSet && !trackingStarted && !TELEMETRY_FIX) {
    	sendFix();
    	TELEMETRY_FIX=true;
    	return;
    }

	if(TELEMETRY_FIX) {
		// Sent every 125ms
		mfd_checksum=0;

		distance = targetPosition.distance;
		altitude = targetPosition.alt;
		azimuth = targetPosition.heading/10;

		sendData();
		sendChecksum();
		sendTelemetryTail();
    }

}

#endif
