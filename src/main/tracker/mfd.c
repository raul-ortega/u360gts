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


int8_t alt_neg = 1;
int16_t alt;
uint16_t mfd_distance;
uint16_t azimuth;

uint8_t read_checksum = 125;
uint8_t calc_checksum = 255;

uint16_t getDistance() {
  return mfd_distance;
}

uint16_t getAzimuth() {
  return azimuth;
}

uint8_t state = 0;

void mfd_encodeTargetData(uint8_t c) {
#ifdef TRACKER_DEBUG
  //Serial.write(c);
#endif
  if (c == 'D') {
    //distance
    state = 1;
    mfd_distance = 0;
    read_checksum = 0;
    calc_checksum = c;
    alt_neg = 1;
    mfdTestMode = false;
    return;
  } else if (c == 'H') {
    //altitude
    state = 2;
    alt = 0;
    calc_checksum += c;
    return;
  } else if (c == 'A') {
    //azimuth
    state = 3;
    azimuth = 0;
    calc_checksum += c;
    return;
  } else if (c == '*') {
    //checksum
    state = 4;
    return;
  } else if (c == '\n') {
    //newline
    if (calc_checksum == read_checksum) {
      gotFix = true;
      telemetry_alt = alt_neg * alt;
    }
    state = 0;
    return;
  } else if (c == 'X') {
    //home set
    mfdTestMode = false;
    settingHome = true;
    return;
  } else if (c == '#') {
    //no coordinates
    mfdTestMode = false;
    return;
  } else if (c == '@') {
    //connection failure
    mfdTestMode = false;
    return;
  } else if (c == 'N') {
    azimuth = 0;
    mfd_distance = 1000;
    telemetry_alt = 0;
    mfdTestMode = true;
    return;
  } else if (c == 'E') {
    azimuth = 90;
    mfd_distance = 1000;
    telemetry_alt = 1000;
    mfdTestMode = true;
    return;
  } else if (c == 'S') {
    azimuth = 180;
    mfd_distance = 0;
    telemetry_alt = 1000;
    mfdTestMode = true;
    return;
  } else if (c == 'W') {
    azimuth = 270;
    mfd_distance = 1000;
    telemetry_alt = 0;
    mfdTestMode = true;
    return;
  }

  switch (state) {
    case 1:
      mfd_distance = (mfd_distance * 10) + (c - 48);
      calc_checksum += c;
      break;
    case 2:
      calc_checksum += c;
      if (c == '-') {
        alt_neg = -1;
        break;
      }
      alt = (alt * 10) + (c - 48);
      break;
    case 3:
      azimuth = (azimuth * 10) + (c - 48);
      calc_checksum += c;
      break;
    case 4:
      read_checksum = (read_checksum * 10) + (c - 48);
      break;
  }
}
