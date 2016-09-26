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
#include "telemetry/nmea.h"

#include "tracker/defines.h"
#include "tracker/telemetry.h"

static serialPort_t *nmeaPort = NULL;
static serialPortConfig_t *portConfig;

//#define NMEA_BAUDRATE 9600
#define NMEA_INITIAL_PORT_MODE MODE_TX

static telemetryConfig_t *telemetryConfig;
static bool nmeaTelemetryEnabled =  false;
static portSharing_e nmeaPortSharing;

bool NMEA_TELEMETRY_FIX = false;
extern bool trackingStarted;
extern bool homeSet;
extern positionVector_t targetPosition;

#define CYCLETIME 10

uint8_t nmeaIdx=0;
// Official data IDs

uint8_t nmea_checksum=0;

static uint32_t lastCycleTime = 0;
static uint8_t cycleNum = 0;

#define NMEA_MAX_BUFFER_LENGTH 255
uint8_t NMEA_BUFFER[NMEA_MAX_BUFFER_LENGTH];
uint8_t nmeaIndex=0;

void addNMEA_char(char c);
void addNMEA_int8(uint8_t data);
void addNMEA_int32(int32_t data);
void addNMEA_sats(void);
void addNMEA_coma(void);
void addNMEA_hordilution(void);
void addNMEA_altitude(void);
void addNMEA_latitude(void);
void addNMEA_longitude(void);
void addNMEA_char(char c);
void addNMEA_time(void);
void addNMEA_checksum(void);
void addNMEA_header(char * header);

void sendNMEA_status(void);
void addNMEA_speed(void);
void addNMEA_angle(void);
void addNMEA_date(void);
void addNMEA_declination(void);

void addNMEA_GPGGA_header(void);
void sendNMEA_GPGGA_Sentence(void);

void sendNMEA_GPRMC_Sentence(void);
void addNMEA_GPRMC_header(void);


void addNMEA_serialize(char* data);

void addNMEA_serialize(char* data){
	uint8_t i = 0;
	while(data[i]!='\0')
	{
		NMEA_BUFFER[nmeaIndex++]=data[i++];
	}
}

void addNMEA_int8(uint8_t data){
	char temp[5];
	tfp_sprintf(temp, "%d\0", data);
	addNMEA_serialize(data);
}
void addNMEA_sats(void){
	char temp[5];
	tfp_sprintf(temp, "%02d\0", getSats());
	addNMEA_serialize(temp);
}
void addNMEA_coma(void){
	nmea_checksum ^= ',';
	NMEA_BUFFER[nmeaIndex++]=',';
}
void addNMEA_int32(int32_t data){
	char temp[10];
	tfp_sprintf(temp, "%d\0", data);
	addNMEA_serialize(temp);
}

void addNMEA_hordilution(void){
	char temp[5];
	uint8_t temp_a = (uint8_t) telemetry_hdop;
	uint8_t temp_b = (uint8_t) (telemetry_hdop*10 - temp_a*10);
	tfp_sprintf(temp, "%01d.%1d\0",temp_a,temp_b);
	addNMEA_serialize(temp);
}

void addNMEA_declination(void){
	char temp[10];
	tfp_sprintf(temp, "%03d.%1d\0",0,0);
	addNMEA_serialize(temp);
	addNMEA_coma();
	addNMEA_char('E');
}

void addNMEA_altitude(void){
	char temp[10];
	uint16_t temp_a = (uint16_t) targetPosition.alt;
	uint16_t temp_b = (uint16_t) targetPosition.alt - temp_a;
	tfp_sprintf(temp, "%03d.%1d\0",temp_a,temp_b);
	addNMEA_serialize(temp);
	addNMEA_coma();
	addNMEA_char('M');
}

void addNMEA_speed(void){
	char temp[10];
	uint16_t temp_a = (uint16_t) telemetry_speed;
	uint16_t temp_b = (uint16_t) telemetry_speed*10 - temp_a*10;
	tfp_sprintf(temp, "%03d.%1d\0",temp_a,temp_b);
	addNMEA_serialize(temp);
}

void addNMEA_angle(void){
	char temp[10];
	uint16_t temp_a = (uint16_t) telemetry_course;
	uint16_t temp_b = (uint16_t) telemetry_course*10 - temp_a*10;
	tfp_sprintf(temp, "%03d.%1d\0",temp_a,temp_b);
	addNMEA_serialize(temp);
}

void addNMEA_latitude(void){

	char temp[20];

	/*int16_t lat_a=abs(targetPosition.lat)/1000;
	uint32_t lat_b=abs(targetPosition.lat)-lat_a;*/

	int32_t lat_a = abs(targetPosition.lat)/1000000;
	int32_t lat_b = abs(targetPosition.lat)%1000000;
	float decimals = (lat_b/1000000.0f)*60.0f;
	lat_a = lat_a * 100 + decimals;
	lat_b = (decimals - ((int32_t) decimals))*1000000;
	
	tfp_sprintf(temp, "%04d.%06d\0",lat_a,lat_b);

	addNMEA_serialize(temp);

	char nS=(targetPosition.lat<0)?'S':'N';

	addNMEA_coma();
	addNMEA_char(nS);
}
void addNMEA_longitude(void){
	char temp[20];

	/*int16_t lon_a = abs(targetPosition.lon)/1000;
	uint32_t lon_b = abs(targetPosition.lon)-lon_a;*/

	int32_t lon_a = abs(targetPosition.lon)/1000000;
	int32_t lon_b = abs(targetPosition.lon)%1000000;
	float decimals = (lon_b/1000000.0f)*60.0f;
	lon_a = lon_a * 100 + decimals;
	lon_b = (decimals - ((int32_t) decimals))*1000000;

	tfp_sprintf(temp, "%05d.%06d\0",lon_a,lon_b);

	addNMEA_serialize(temp);

	char eW=(targetPosition.lon<0)?'W':'E';

	addNMEA_coma();
	addNMEA_char(eW);
}
void addNMEA_char(char c){
	NMEA_BUFFER[nmeaIndex++]=c;
}
void addNMEA_GPGGA_header(void){
	char temp[]="$GPGGA\0";
	addNMEA_header(temp);
}

void addNMEA_GPRMC_header(void){
	char temp[]="$GPRMC\0";
	addNMEA_header(temp);
}

void addNMEA_header(char * header){
	addNMEA_serialize(header);
}

void addNMEA_time(void){
	addNMEA_int32(telemetry_time);
}

void addNMEA_date(void){
	addNMEA_int32(telemetry_date);
}

void addNMEA_checksum(void){
	char temp[5];
	nmea_checksum = 0;
	for(uint8_t i=1;i<nmeaIndex;i++)
		nmea_checksum ^= NMEA_BUFFER[i];

	tfp_sprintf(temp, "%02X\0", nmea_checksum);

	addNMEA_char('*');
	addNMEA_serialize(temp);
}

void sendNMEAPacket(void){
	for(uint8_t i=0;i<nmeaIndex;i++){
		serialWrite(nmeaPort,NMEA_BUFFER[i]);
	}
	serialWrite(nmeaPort, '\n');
}

void sendNMEA_status(void){

}


void sendNMEA_GPGGA_Sentence(void)
{
	/*$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

	Where:
	     GGA          Global Positioning System Fix Data
	     123519       Fix taken at 12:35:19 UTC
	     4807.038,N   Latitude 48 deg 07.038' N
	     01131.000,E  Longitude 11 deg 31.000' E
	     1            Fix quality: 0 = invalid
	                               1 = GPS fix (SPS)
	                               2 = DGPS fix
	                               3 = PPS fix
				       4 = Real Time Kinematic
				       5 = Float RTK
	                               6 = estimated (dead reckoning) (2.3 feature)
				       7 = Manual input mode
				       8 = Simulation mode
	     08           Number of satellites being tracked
	     0.9          Horizontal dilution of position
	     545.4,M      Altitude, Meters, above mean sea level
	     46.9,M       Height of geoid (mean sea level) above WGS84
	                      ellipsoid
	     (empty field) time in seconds since last DGPS update
	     (empty field) DGPS station ID number
	     *47          the checksum data, always begins with * */
	nmeaIndex = 0;
	addNMEA_GPGGA_header();
	addNMEA_coma();
	addNMEA_time();
	addNMEA_coma();
	addNMEA_latitude();
	addNMEA_coma();
	addNMEA_longitude();
	addNMEA_coma();
	addNMEA_char('1');
	addNMEA_coma();
	addNMEA_sats();
	addNMEA_coma();
	addNMEA_hordilution();
	addNMEA_coma();
	addNMEA_altitude();
	addNMEA_coma();
	addNMEA_altitude();
	addNMEA_coma();
	addNMEA_coma();
	addNMEA_checksum();

	sendNMEAPacket();
}

void sendNMEA_GPRMC_Sentence(void)
{
/*  $GPRMC,123519,  A,4807.038,N,01131.000,E,022.4, 084.4,230394,003.1,W*6A
 *  $GPRMC,23572383,A,4740.010,N,00858.621,E,037.0,9000.0,250216, 00.0,E,*27
 *
	RMC          Recommended Minimum sentence C
	123519       Fix taken at 12:35:19 UTC
	A            Status A=active or V=Void.
	4807.038,N   Latitude 48 deg 07.038' N
	01131.000,E  Longitude 11 deg 31.000' E
	022.4        Speed over the ground in knots
	084.4        Track angle in degrees True
	230394       Date - 23rd of March 1994
	003.1,W      Magnetic Variation
	*6A          The checksum data, always begins with */
	nmeaIndex = 0;
	addNMEA_GPRMC_header();
	addNMEA_coma();
	addNMEA_time();
	addNMEA_coma();
	addNMEA_char('A');
	addNMEA_coma();
	addNMEA_latitude();
	addNMEA_coma();
	addNMEA_longitude();
	addNMEA_coma();
	addNMEA_speed();
	addNMEA_coma();
	addNMEA_angle();
	addNMEA_coma();
	addNMEA_date();
	addNMEA_coma();
	addNMEA_declination();
	addNMEA_checksum();

	sendNMEAPacket();
}

void initNMEATelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_NMEA);
    nmeaPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_NMEA);
    nmeaIdx=0;
}

void freeNMEATelemetryPort(void)
{
    closeSerialPort(nmeaPort);
    nmeaPort = NULL;
    nmeaTelemetryEnabled = false;
}

void configureNMEATelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    nmeaPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_NMEA, NULL, baudRates[portConfig->telemetry_baudrateIndex], NMEA_INITIAL_PORT_MODE, telemetryConfig->telemetry_inversion ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
    if (!nmeaPort) {
        return;
    }

    nmeaTelemetryEnabled = true;
}

bool hasEnoughTimeLapsedSinceLastTelemetryTransmission_nmea(uint32_t currentMillis)
{
    return currentMillis - lastCycleTime >= CYCLETIME;
}

void checkNMEATelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(nmeaPortSharing);

    if (newTelemetryEnabledValue == nmeaTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureNMEATelemetryPort();
    else
        freeNMEATelemetryPort();
}
/*void forwardNMEATelemetry(char c) {
	if (!nmeaTelemetryEnabled) {
	        return;
	}
	serialWrite(nmeaPort, c);
}*/
void handleNMEATelemetry(void)
{
    if (!nmeaTelemetryEnabled) {
        return;
    }

    uint32_t now = millis();

    if (!hasEnoughTimeLapsedSinceLastTelemetryTransmission_nmea(now)) {
        return;
    }

	lastCycleTime = now;

	cycleNum++;

    if(!homeSet && !trackingStarted) {
    	NMEA_TELEMETRY_FIX=false;
    	//sendNoFix();
    	return;
    }

    if(homeSet && !trackingStarted && !NMEA_TELEMETRY_FIX) {
    	//sendFix();
    	NMEA_TELEMETRY_FIX=true;
    	return;
    }

	if(NMEA_TELEMETRY_FIX) {
		nmeaIdx++;

		if ((nmeaIdx % 30) == 0)
			sendNMEA_GPGGA_Sentence();

		if ((nmeaIdx % 30) == 15)
			sendNMEA_GPRMC_Sentence();
    }

}

#endif
