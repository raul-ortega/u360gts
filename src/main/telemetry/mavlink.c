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
#include "telemetry/mavlink/common/mavlink.h"
#include "tracker/defines.h"
#include "tracker/telemetry.h"

static serialPort_t *mavlinkPort = NULL;
static serialPortConfig_t *portConfig;

#define MAVLINK_BAUDRATE 9600
#define MAVLINK_INITIAL_PORT_MODE MODE_TX

static telemetryConfig_t *telemetryConfig;
static bool mavlinkTelemetryEnabled =  false;
static portSharing_e mavlinkPortSharing;

bool MAVLIN_TELEMETRY_FIX=false;
uint8_t mavlinkParamIdx=0;

extern bool trackingStarted;
extern bool homeSet;
extern positionVector_t targetPosition;
extern int16_t telemetry_sats;
extern float telemetry_roll;
extern float telemetry_pitch;
extern float telemetry_yaw;

#define CYCLETIME            10//150

static uint32_t lastCycleTime = 0;
static uint8_t cycleNum = 0;

mavlink_message_t msg;
uint8_t mavlink_buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;

uint16_t gpsStatus=0;
uint16_t apmMode=0;
float vgnd=1000.0;    //[cm/s ]

float mavlink_timer=0;

static void serializeMAVLINK(void){
	for(int i=0;i<len;i++) {
		serialWrite(mavlinkPort, mavlink_buf[i]);
	}
}

void sendMavlinkAttitude() {
//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed
  //mavlink_msg_attitude_pack (100, 200, &msg, 0, telemetry_roll, telemetry_pitch, telemetry_yaw, 0.0, 0.0, 0.0);
  mavlink_msg_attitude_pack (100, 200, &msg, 0, telemetry_roll, telemetry_pitch, radians(telemetry_course), 0.0, 0.0, 0.0);
  len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
}
void sendMavlinkGpsCoord() {
	uint16_t sats;
	if(telemetry_sats == 0)
		sats = 8;
	else
		sats = telemetry_sats;

	//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible
	mavlink_msg_gps_raw_int_pack (100, 200, &msg, 0, gpsStatus, targetPosition.lat*10, targetPosition.lon*10, ((int32_t)targetPosition.alt)*1000, 0, 0, vgnd, 0, (uint8_t)sats);
	//mavlink_msg_gps_raw_int_pack (100, 200, &msg, 0, gpsStatus, 471234500, 81234500, 1000, 0, 0, vgnd, 0, telemetry_sats);
	len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
}
void sendMavlinkGpsCoord2() {
	//lon = 81234500;
	//lat = 471234500;
	//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg
	mavlink_msg_global_position_int_pack(100, 200, &msg, 0, targetPosition.lat*10, targetPosition.lon*10, ((int32_t)targetPosition.alt)*1000, ((int32_t)targetPosition.alt)*1000, 0, 0, 0, (uint16_t)telemetry_course);
	//mavlink_msg_global_position_int_pack (100, 200, &msg, 0, 471234500, 81234500, 1000, targetPosition.alt, 0, 0, 0, 0);
	len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
}

void sendMavlinkHeartBeat() {
	//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status
	mavlink_msg_heartbeat_pack (100, 200, &msg, 1, 1, 1, apmMode, 1);
	len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
}
void sendMavlinkParam() {
	//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index
	mavlink_msg_param_value_pack (100, 200, &msg, "TKOFF_FLAP_PCNT", 0.0, 2, 1, 1);
	len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
}

/*void sendBatteryVoltage() {
  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load,
//uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1,
//uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
  mavlink_msg_sys_status_pack(100, 200, &msg, 0, 0, 0, 0, vcc, 0, rssi, 0, 0, 0, 0, 0, 0);  //I am sending rssi here because of DroidPlanners location of remaining battery % is exactly the place where I want RSSI to show

  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    //Serial.write(buf, len);
  #endif
}*/

/*void sendRssi() {
//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed
  mavlink_msg_radio_pack (100, 200, &msg, rssi, rssi, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    //Serial.write(buf, len);
  #endif
}*/

void sendMavlinkAltitude() {
  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
  mavlink_msg_vfr_hud_pack (100, 200, &msg, vgnd/100, vgnd/100, (uint16_t)telemetry_course, 0, ((int32_t)targetPosition.alt)*1000, 0);
  len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
}


void initMAVLINKTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);
    mavlinkPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_MAVLINK);
    mavlinkParamIdx=0;
}

void freeMAVLINKTelemetryPort(void)
{
    closeSerialPort(mavlinkPort);
    mavlinkPort = NULL;
    mavlinkTelemetryEnabled = false;
}

void configureMAVLINKTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, NULL, baudRates[portConfig->telemetry_baudrateIndex], MAVLINK_INITIAL_PORT_MODE, telemetryConfig->telemetry_inversion ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
    if (!mavlinkPort) {
        return;
    }

    mavlinkTelemetryEnabled = true;
}

bool hasEnoughTimeLapsedSinceLastTelemetryTransmission_mavlink(uint32_t currentMillis)
{
    return currentMillis - lastCycleTime >= CYCLETIME;
}

void checkMAVLINKTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(mavlinkPortSharing);

    if (newTelemetryEnabledValue == mavlinkTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureMAVLINKTelemetryPort();
    else
        freeMAVLINKTelemetryPort();
}
/*void forwardMAVLINKTelemetry(char c) {
	if (!mavlinkTelemetryEnabled) {
	        return;
	}
	serialWrite(mavlinkPort, c);
}*/

void handleMAVLINKTelemetry(void)
{

    if (!mavlinkTelemetryEnabled) {
        return;
    }

    uint32_t now = millis();

    if (!hasEnoughTimeLapsedSinceLastTelemetryTransmission_mavlink(now)) {
        return;
    }

	lastCycleTime = now;

	cycleNum++;

    if(!homeSet && !trackingStarted) {
    	MAVLIN_TELEMETRY_FIX=false;
    	return;
    }

    if(homeSet && !trackingStarted && !MAVLIN_TELEMETRY_FIX) {
    	MAVLIN_TELEMETRY_FIX=true;
    	return;
    }


	if(MAVLIN_TELEMETRY_FIX) {
		mavlinkParamIdx++;
		// Sent every 100 ms and stops sending at 1000 ms
		if((mavlinkParamIdx % 5) == 0) {
			sendMavlinkHeartBeat();
			serializeMAVLINK();
		}
		if(((mavlinkParamIdx % 10) == 0) && (mavlinkParamIdx <= 100)) {
			sendMavlinkParam();
			serializeMAVLINK();
		}
		if ((mavlinkParamIdx % 30) == 0) {

			sendMavlinkGpsCoord();
			serializeMAVLINK();

			sendMavlinkGpsCoord2();
			serializeMAVLINK();
		}

		if ((mavlinkParamIdx % 30) == 15) {

			sendMavlinkAltitude();
			serializeMAVLINK();

			sendMavlinkAttitude();
			serializeMAVLINK();
		}



    }

}

#endif
