/*
 * This file is part of u360gts, aka amv-open360tracker 32bits:
 * https://github.com/raul-ortega/amv-open360tracker-32bits
 *
 * u360gts is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * u360gts is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with u360gts.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef TELEMETRY

#include "common/printf.h"
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
#include "telemetry/position_estimation_log.h"

#include "tracker/defines.h"
#include "tracker/telemetry.h"
#include "tracker/TinyGPS.h"
#include "tracker/gps_estimation.h"

static serialPort_t *POSESTPort = NULL;
static serialPortConfig_t *portConfig;

//#define POSEST_BAUDRATE 9600
#define POSEST_INITIAL_PORT_MODE MODE_TX

static telemetryConfig_t *telemetryConfig;
static bool POSESTTelemetryEnabled =  false;
static portSharing_e POSESTPortSharing;

bool POSEST_TELEMETRY_FIX = false;

extern epsVector_t targetLast;
extern epsVector_t targetCurrent;
extern epsVector_t targetEstimated;
extern positionVector_t trackerPosition;

uint32_t prevLastTime = 0;
uint16_t prevEstimateIndex = 0;

#define CYCLETIME 200

uint8_t POSESTIdx=0;
// Official data IDs

uint8_t POSEST_checksum=0;

static uint32_t lastCycleTime = 0;
static uint8_t cycleNum = 0;

#define POSEST_MAX_BUFFER_LENGTH 255
uint8_t POSEST_BUFFER[POSEST_MAX_BUFFER_LENGTH];
uint8_t POSESTIndex=0;

void addPOSEST_char(char c);
void addPOSEST_int8(uint8_t data);
void addPOSEST_int32(int32_t data);
void addPOSEST_coma(void);
void addPOSEST_char(char c);
void addPOSEST_serialize(char* data);
void addPOSEST_trackerData(void);
void addPOSEST_TinyGPS_stats(void);

void addPOSEST_serialize(char* data){
	uint8_t i = 0;
	while(data[i]!='\0')
	{
		POSEST_BUFFER[POSESTIndex++]=data[i++];
	}
}

void addPOSEST_int8(uint8_t data){
	char temp[5];
	tfp_sprintf(temp, "%d\0", data);
	addPOSEST_serialize(data);
}

void addPOSEST_coma(void){
	POSEST_checksum ^= ',';
	POSEST_BUFFER[POSESTIndex++]=',';
}
void addPOSEST_int32(int32_t data){
	char temp[10];
	tfp_sprintf(temp, "%d\0", data);
	addPOSEST_serialize(temp);
}

void addPOSEST_vector(pvQElement_t *epsVector){

	char temp[50];

	tfp_sprintf(temp,"%d,%d,",epsVector->type,epsVector->index);
	addPOSEST_serialize(temp);

	tfp_sprintf(temp,"%d.%06d,",epsVector->lat_sgn*epsVector->lat_a,epsVector->lat_b);
	addPOSEST_serialize(temp);

	tfp_sprintf(temp,"%d.%06d,",epsVector->lon_sgn*epsVector->lon_a,epsVector->lon_b);
	addPOSEST_serialize(temp);

	tfp_sprintf(temp,"%d,%d,%d,%d",epsVector->time,(uint16_t)epsVector->distance,(uint16_t)epsVector->heading,(uint16_t)epsVector->speed);
	addPOSEST_serialize(temp);

	tfp_sprintf(temp,",%d",(uint16_t) telemetry_course);
	addPOSEST_serialize(temp);


}
void addPOSEST_trackerData(void){
	char temp[20];

	tfp_sprintf(temp,"%d,%d,",trackerPosition.distance,trackerPosition.heading);
	addPOSEST_serialize(temp);

}

/*void addPOSEST_TinyGPS_stats(void){
	char temp[20];
	uint16_t chars;
	uint8_t sentences;
	uint8_t failed_cs;
	TinyGPS_stats(&chars,&sentences,&failed_cs);
	tfp_sprintf(temp,"%d,%d,%d",chars,sentences,failed_cs);
	addPOSEST_serialize(temp);
}*/

void addPOSEST_char(char c){
	POSEST_BUFFER[POSESTIndex++]=c;
}

void sendPOSESTPacket(void){
	for(uint8_t i=0;i<POSESTIndex;i++){
		serialWrite(POSESTPort,POSEST_BUFFER[i]);
	}
	serialWrite(POSESTPort, '\n');
}

/*void sendPOSESTSentence(void)
{

	if(targetEstimated.index != prevEstimateIndex) {

		POSESTIndex = 0;
		//addPOSEST_vector(&targetLast);
		//addPOSEST_coma();
		addPOSEST_vector(&targetCurrent);
		addPOSEST_coma();
		addPOSEST_vector(&targetEstimated);
		addPOSEST_coma();
		addPOSEST_trackerData();

		if(PROTOCOL(TP_GPS_TELEMETRY)){
			addPOSEST_TinyGPS_stats();
		}

		sendPOSESTPacket();

		prevEstimateIndex = targetEstimated.index;
	}
}*/
void sendPOSESTSentence(void)
{

	if(!pvEmpty()) {

		POSESTIndex = 0;
		pvQElement_t pvector = pvGet();
		addPOSEST_vector(&pvector);
		/*if(PROTOCOL(TP_GPS_TELEMETRY)){
			addPOSEST_TinyGPS_stats();
		}*/

		sendPOSESTPacket();

		prevEstimateIndex = targetEstimated.index;
	}
}

void initPOSESTTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_POSEST);
    POSESTPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_POSEST);
    POSESTIdx=0;
}

void freePOSESTTelemetryPort(void)
{
    closeSerialPort(POSESTPort);
    POSESTPort = NULL;
    POSESTTelemetryEnabled = false;
}

void configurePOSESTTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    POSESTPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_POSEST, NULL, baudRates[portConfig->telemetry_baudrateIndex], POSEST_INITIAL_PORT_MODE, telemetryConfig->telemetry_inversion ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
    if (!POSESTPort) {
        return;
    }

    POSESTTelemetryEnabled = true;
}

bool hasEnoughTimeLapsedSinceLastTelemetryTransmission_POSEST(uint32_t currentMillis)
{
    return currentMillis - lastCycleTime >= CYCLETIME;
}

void checkPOSESTTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(POSESTPortSharing);

    if (newTelemetryEnabledValue == POSESTTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configurePOSESTTelemetryPort();
    else
        freePOSESTTelemetryPort();
}

void handlePOSESTTelemetry(void)
{
    if (!POSESTTelemetryEnabled) {
        return;
    }

    uint32_t now = millis();

    if (!hasEnoughTimeLapsedSinceLastTelemetryTransmission_POSEST(now)) {
        return;
    }

	lastCycleTime = now;

	cycleNum++;

    /*if(!homeSet && !trackingStarted) {
    	POSEST_TELEMETRY_FIX=false;
    	//sendNoFix();
    	return;
    }

    if(homeSet && !trackingStarted && !POSEST_TELEMETRY_FIX) {
    	//sendFix();
    	POSEST_TELEMETRY_FIX=true;
    	return;
    }

	if(POSEST_TELEMETRY_FIX) {
		sendPOSESTSentence();
    }*/

	sendPOSESTSentence();
}

#endif
