/*
 * This file is part of u360gts, aka amv-open360tracker 32bits:
 * https://github.com/raul-ortega/amv-open360tracker-32bits
 *
 * The code below is an adaptation by Raúl Ortega of the original code written by Samuel Brucksch:
 * https://github.com/SamuelBrucksch/open360tracker
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

#include "config.h"
#include "telemetry.h"
#include "TinyGPS.h"
#include "protocol_detection.h"

int32_t telemetry_lat = 0;
int32_t telemetry_lon = 0;
int16_t telemetry_alt = 0;
int16_t telemetry_sats = 0;
int32_t telemetry_time = 0;
int32_t telemetry_date = 0;
int16_t telemetry_age = 0;

uint8_t telemetry_failed_cs = 0;

float telemetry_course = 0.0f;
float telemetry_speed = 0.0f;
float telemetry_declination = 0.0f;
float telemetry_hdop = 0.0f;

float telemetry_pitch = 0.0f;
float telemetry_roll = 0.0f;
float telemetry_yaw = 0.0f;

uint8_t telemetry_diy_gps = 0;

uint8_t a;

uint8_t LOCAL_GPS;

uint16_t chars = 0;

uint8_t sentences = 0;

int32_t getTargetLat() {
  return telemetry_lat;
}

int32_t getTargetLon() {
  return telemetry_lon;
}

int16_t getTargetAlt() {
  return telemetry_alt;
}

uint16_t getSats() {
  return telemetry_sats;
}

void encodeTargetData(uint8_t c) {

	uint16_t chars = 0;
	uint8_t sentences = 0;

	protocolDetectionParser(c);

	if(PROTOCOL(TP_MFD))
		mfd_encodeTargetData(c);
	else if(PROTOCOL(TP_GPS_TELEMETRY))
		gps_encodeTargetData(c);
	else if(PROTOCOL(TP_MAVLINK))
		mavlink_encodeTargetData(c);
	else if(PROTOCOL(TP_RVOSD))
		rvosd_encodeTargetData(c);
	else if(PROTOCOL(TP_FRSKY_D))
		frskyd_encodeTargetData(c);
	else if(PROTOCOL(TP_FRSKY_X))
		frskyx_encodeTargetData(c);
	else if(PROTOCOL(TP_LTM))
		ltm_encodeTargetData(c);
	else if(PROTOCOL(TP_LTM_FRSKYD))
		frskyd_encodeTargetData(c);
}

void gps_encodeTargetData(uint8_t c) {
  //if(PROTOCOL(TP_GPS_TELEMETRY)){
	TinyGPS_stats(&chars,&sentences,&telemetry_failed_cs);
  //}
  if (TinyGPS_encode(c)) {
    unsigned long fix_age;
    get_position(&telemetry_lat, &telemetry_lon, &fix_age);
    if (fix_age == GPS_INVALID_AGE) {
      gotFix = false;
    } else if (fix_age > 5000) {
      gotFix = false;
    } else {
      //TODO valid data

      if (altitude() != GPS_INVALID_ALTITUDE) {
        telemetry_alt = (int16_t)(altitude() / 100);
        gotAlt = true;
      }
      //date and time
      get_datetime(&telemetry_date, &telemetry_time, &telemetry_age);
      //course (angle)
      telemetry_course = f_course();
      //speed
      telemetry_speed = f_speed_knots();
      //hdop
      telemetry_hdop = f_hdop();

      if(!LOCAL_GPS){
        if (satellites() != GPS_INVALID_SATELLITES) {
        	telemetry_sats = (int16_t)satellites();
        }
      }
      if (get_sentence_type() != _GPS_SENTENCE_GPRMC)
    	  gotFix = true;
    }
  }
}

//TODO check if this is the right conversion
int32_t gpsToLong(int8_t neg, uint16_t bp, uint16_t ap) {
  // we want convert from frsky to millionth of degrees
  // 0302.7846 -> 03 + (02.7846/60) = 3.04641 -> 3046410
  // first the upper part
  uint32_t first = ((uint32_t)bp / 100) * 100000;
  uint32_t second = ((((uint32_t)bp % 100) * 100000) + ((uint32_t)ap * 10)) / 60;

  // take sign into account
  return ((int32_t)(first + second) * (uint32_t)neg);
}


