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
#include <string.h>
#include "config/runtime_config.h"

#define TELEMETRY_LATLON_DIVIDER_F 1000000.0f
#define TELEMETRY_LATLON_DIVIDER_I 1000000

extern bool mfdTestMode;
extern bool gotAlt;
extern bool gotFix;
extern bool settingHome;

extern int32_t telemetry_lat;
extern int32_t telemetry_lon;
extern int32_t telemetry_alt;
extern uint8_t telemetry_sats;
extern uint32_t telemetry_time;
extern uint32_t telemetry_date;
extern uint16_t telemetry_age;    // replace with uint16_t
extern uint16_t telemetry_volt;  //
extern uint16_t telemetry_amp;
extern float telemetry_course;
extern float telemetry_speed;        // to replace with telemetry_gnd_speed  and  telemetry_air_speed
extern uint8_t telemetry_gnd_speed;
extern uint8_t telemetry_air_speed;
extern float telemetry_declination;  // replace with uint8_t
extern float telemetry_hdop;        // replace with uint8_t
extern float telemetry_pitch;       // replace with int16_t
extern float telemetry_roll;			// replace with int16_t
extern float telemetry_yaw;			// replace with int16_t
extern uint8_t telemetry_failed_cs;
/////////////////////////////////////////////////////////////////////     new 
extern uint8_t telemetry_gnd_speed;
extern uint8_t telemetry_air_speed; 
extern uint8_t telemetry_provider;
extern uint8_t telemetry_fixtype;
extern uint8_t telemetry_volts;
extern uint8_t telemetry_amps;
extern uint8_t telemetry_rssi;
extern uint8_t telemetry_flightmode;
extern uint8_t telemetry_uav_type;
extern bool telemetry_armed;
extern bool telemetry_failsafe;

int32_t getTargetAlt(int16_t home_alt);
void encodeTargetData(uint8_t c);
int32_t getTargetLat();
int32_t getTargetLon();
uint16_t getSats();
uint16_t getDistance();
uint16_t getAzimuth();
void enableProtocolDetection(void);
void disableProtocolDetection(void);

