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
#include "telemetry/mavlink/common/mavlink.h"
#include "Arduino.h"

#define MAVLINK_MAX_PAYLOAD_LEN 36

void mavlink_handleMessage(mavlink_message_t* msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      telemetry_lat = mavlink_msg_gps_raw_int_get_lat(msg) / 10;
      telemetry_lon = mavlink_msg_gps_raw_int_get_lon(msg) / 10;
      telemetry_sats = mavlink_msg_gps_raw_int_get_satellites_visible(msg);
      telemetry_alt = (int16_t)(mavlink_msg_gps_raw_int_get_alt(msg) / 1000);
      telemetry_fixtype = mavlink_msg_gps_raw_int_get_fix_type(msg);
      telemetry_speed = mavlink_msg_gps_raw_int_get_vel(msg) / 100;
      gotAlt = true;

      // fix_type: GPS lock 0-1=no fix, 2=2D, 3=3D
      if (telemetry_fixtype > 1) {
        gotFix = true;
      } else {
        gotFix = false;
      }
      break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      telemetry_lat = mavlink_msg_global_position_int_get_lat(msg) / 10;
      telemetry_lon = mavlink_msg_global_position_int_get_lon(msg) / 10;
      telemetry_alt = (int16_t)(mavlink_msg_global_position_int_get_alt(msg) / 1000);
      gotAlt = true;

      if (telemetry_lat != 0 || telemetry_lon != 0) {
        gotFix = true;
      } else {
        gotFix = false;
      }
      break;
    case MAVLINK_MSG_ID_ATTITUDE:
		{
			telemetry_pitch = mavlink_msg_attitude_get_pitch(msg);
			telemetry_roll = mavlink_msg_attitude_get_roll(msg);
			telemetry_yaw = mavlink_msg_attitude_get_yaw(msg);
			telemetry_course = degrees(telemetry_yaw);
		}
		break;
  }
}

void mavlink_encodeTargetData(uint8_t c) {
  mavlink_status_t status;
  mavlink_message_t msg;
  if (mavlink_parse_char(0, c, &msg, &status))
    mavlink_handleMessage(&msg);
  else
	telemetry_failed_cs += status.packet_rx_drop_count;
}

