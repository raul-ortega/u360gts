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

/*
This code is written by Samuel Brucksch, and improved by Raul Ortega
 
 it will decode the RVOSD telemetry protocol and will convert it into values the open360tracker can understand
 
 $1,1,00040291,00.000000,N,000.000000,W,+00000,+00000,000,0000,0000,000,+000,000,089,089,089,089,1160,0000,00004,0004,00,000,0026,*00
 $1,1,00194723,41.572332,N,001.973692,E,+00312,+00312,254,0021,0000,005,+000,220,000,000,000,000,1156,0002,00050,0161,06,000,0033,0,*01
 
 It is sent 25 times/S, 115200, 8N1.
 
 $
 Validity (1 valid, 0 invalid)
 Units (1 metric, 0 imperial)
 HHMMSSmm (hour|minutes|seconds|tenth of seconds)
 ddmm.mmmm, 
 N/S (Latitude)
 dddmm.mmmm,
 E/W (Longitude)
 +/- altitude (relative)
 +/- altitude (absolute)
 Roll (0 to 255)
 Pitch (0 to 4096)
 Airspeed
 GroundSpeed
 +/- Variometer
 Heading (0 to 360)
 Rudder
 Elevator
 Aileron
 Throttle (mS / 100)
 Main battery voltage
 Aux battery voltage (V * 100)
 Current
 mAh
 Sats
 RSSI
 Temp
 *checksum
 
 
 Checksum is everything XORed from "$" to "temperature".
 */
#include "config.h"
#include "telemetry.h"

uint8_t dot_found = 0;

uint16_t checksum_calculation = 0;
uint16_t checksum_read = 0;

//data needed to buffer
int16_t alttemp;
int16_t alt;
int8_t altsign;

uint16_t rvosd_headingtemp;
uint16_t rvosd_heading=0;

int8_t latsign;
int8_t lonsign;
uint32_t rvosd_lat_bp;
uint32_t rvosd_lon_bp;
uint32_t rvosd_lat_ap;
uint32_t rvosd_lon_ap;
uint8_t satstemp;
int32_t lat = 0;
int32_t lon = 0;
uint8_t sats = 0;

uint8_t rvosd_data_index = 0;
uint8_t frame_started = 0;
uint8_t checksum_started = 0;
uint8_t metric = 0;
uint8_t checksum_index = 0;

void rvosd_encodeTargetData(uint8_t c) {
  if (c == '$' && !frame_started){
    frame_started = true;
    checksum_started = false;
    checksum_calculation = 0;
    checksum_read = 0;
    rvosd_lat_bp = 0;
    rvosd_lat_ap = 0;
    rvosd_lon_bp = 0;
    rvosd_lon_ap = 0;
    latsign = 0;
    lonsign = 0;
    alttemp = 0;
    satstemp = 0;
    altsign = 0;
    checksum_index = 0;
    rvosd_data_index=0;
    dot_found=false;
    rvosd_headingtemp=0;
    return;
  } 
  else if (c == 'R' && frame_started){
    checksum_calculation ^= c;
    return;
  } 
  else if (c == 'V' && frame_started){
    checksum_calculation ^= c;
    return;
  }
  else if (c == '*' && frame_started){
    checksum_started = true;
    return;
  }
  else if (checksum_index == 2){
    if (checksum_read == checksum_calculation){
      telemetry_lat = ((int32_t)rvosd_lat_bp*100000+(int32_t)(rvosd_lat_ap/10))*(int32_t)latsign;
      telemetry_lon = ((int32_t)rvosd_lon_bp*100000+(int32_t)(rvosd_lon_ap/10))*(int32_t)lonsign;
      telemetry_sats=satstemp;
      telemetry_alt=altsign*alttemp;
      rvosd_heading=rvosd_headingtemp;
      // data is ready
      gotFix = true;
      gotAlt = true;
    }
    else{
      //needed?
    	telemetry_failed_cs++;
    }
    frame_started = false;
  }

  if (frame_started){
    if (checksum_started){
      checksum_index++;
      checksum_read *= 16;
      if (c >= '0' && c <= '9'){
        checksum_read += (c - '0');
      }
      else{
        checksum_read += (c-'A'+10);
      }
    }
    else{
      //we get data
      checksum_calculation ^= c;
    }
  }
  else{
    return;
  }

  if (c == ','){
    rvosd_data_index++;
    dot_found=false;
    return;
  }

  switch (rvosd_data_index) {
  case 0:
    //frame valid?
    if ((c - '0') == 0){
      //frame invalid, skip...
      frame_started = false;
      return;
    }
    else if ((c - '0') == 1){
      //frame valid, do nothing
    }
    break;
  case 1:
    //units, 0 = imperial, 1 = metric
    if (c - '0' == 0){
      metric = false;
    }
    else if (c - '0' == 1){
      metric = true;
    }
    break;
  case 3: 
    //lat
    if (c == '.'){
      dot_found = true;
      return;
    }
    if (!dot_found){
      rvosd_lat_bp *= 10;
      rvosd_lat_bp += c-'0';
    }
    else{
      rvosd_lat_ap *= 10;
      rvosd_lat_ap += c-'0';
    }
    break;
  case 4:
    //N/S
    if (c == 'N'){
      latsign = 1;
    }
    else if (c == 'S'){
      latsign = -1;
    }
    break;
  case 5:
    //lon
    if (c == '.'){
      dot_found = true;
      return;
    }
    if (!dot_found){
      rvosd_lon_bp *= 10;
      rvosd_lon_bp += c-'0';
    }
    else{
      rvosd_lon_ap *= 10;
      rvosd_lon_ap += c-'0';
    }
    break;
  case 6:
    //E/W
    if (c == 'E'){
      lonsign = 1;
    }
    else if (c == 'W'){
      lonsign = -1;
    }
    break;
  case 8:
    //absolute alt;
    if (c == '.'){
      dot_found = true;
      return;
    }
    else if (c == '-'){
      altsign = -1;
      return;
    }
    else if (c == '+'){
      altsign = 1;
      return;
    }
    if (!dot_found){
      alttemp *= 10;
      alttemp += c- '0';
    }
    else{
      //no after comma i think
    }
    break;
    case 14:
      //Heading;
      rvosd_headingtemp *= 10;
      rvosd_headingtemp += c- '0';
    break;
  case 23:
    satstemp *= 10;
    satstemp += c - '0';
    break;
  default:
    break;
  }
}



