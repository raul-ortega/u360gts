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

#ifndef DEFINES_H
#define DEFINES_H

#include "Arduino.h"
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

/*
 * Lat/Lon in units of millionths of a degree
 * Altitude from -32.768m to 32.767m
 * Heading from 0 to - 3599
 * Distance from 0 ... 64000 meters
 */
typedef struct {
  int32_t lat;
  int32_t lon;
  int16_t alt;
  uint16_t heading;
  uint32_t distance;
  int16_t home_alt;
}
positionVector_t;

typedef struct {
	uint16_t distance;
	uint16_t heading;
	uint16_t speed;
} epsVectorGain_t;

#define toRad(val) val * PI/180.0f
#define toDeg(val) val * 180.0f/PI

#define meter2feet(value) value * 3.2808399
#define feet2meter(value) value * 0.3048

#endif
