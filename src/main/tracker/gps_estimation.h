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

#include <stdint.h>
#include <stdbool.h>

#include "tracker/defines.h"

typedef struct {
	float heading;
	float speed;
	float distance;
	int16_t lat_a;
	uint32_t lat_b;
	int8_t lat_sgn;
	int16_t lon_a;
	uint32_t lon_b;
	int8_t lon_sgn;
	uint32_t time;
	uint16_t index;
	uint8_t type;
	uint16_t frequency;
} pvQElement_t;

typedef struct {
	float heading;
	float speed;
	float distance;
	int32_t lat;
	int32_t lon;
	int16_t lat_a;
	uint32_t lat_b;
	int8_t lat_sgn;
	int16_t lon_a;
	uint32_t lon_b;
	int8_t lon_sgn;
	uint32_t time;
	uint16_t index;
	uint16_t frequency;
} epsVector_t;



epsVector_t targetLast;
epsVector_t targetCurrent;
epsVector_t targetEstimated;

void pvInit(void);
bool pvFull();
bool pvEmpty();
bool pvPut(epsVector_t *pvector, uint8_t vectorType);
pvQElement_t pvGet(void);
void epsVectorsInit(epsVector_t *last, epsVector_t *current, epsVector_t *esttimated, uint8_t interpolation,uint8_t interpolation_points);
uint16_t epsVectorEstimate(epsVector_t *last, epsVector_t *current, epsVector_t *estimated,epsVectorGain_t gain, bool hasFix);
void epsVectorLoad(epsVector_t *current,int32_t lat,int32_t lon,float distance, uint32_t last_time, uint32_t currentTime,epsVectorGain_t gain);
void epsVectorCurrentToLast(epsVector_t *current,epsVector_t *last);
void epsVectorDecompose(epsVector_t *pvector);
float epsVectorSpeed(uint32_t last_time,uint32_t currentTime, float distance);
uint16_t getPositionVectorIndex(void);
