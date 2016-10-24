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

#include "gps_estimation.h"

#include <stdint.h>
#include <stdbool.h>
#include "Arduino.h"
#include "telemetry.h"
#include "interpolation.h"

#define earthRadius 6378137.0f

float speed;
uint16_t vartime;
uint32_t estimatedTime;
float estimatedDistance;
float estimatedAccDistance;
float estimatedHeading;
float estimatedSpeed;
uint16_t positionIndex=0;

bool interpolationOn;

#define PVQ_ELEMENTS 20
pvQElement_t epsVectors[PVQ_ELEMENTS];
int PVQ_SIZE;
uint8_t pvQIn;
uint8_t pvQOut;

void pvInit(void)
{
	PVQ_SIZE = PVQ_ELEMENTS;
	pvQIn = 0;
	pvQOut = 0;
}

bool pvFull(){
	return (pvQIn == (( pvQOut - 1 + PVQ_SIZE) % PVQ_SIZE));
}
bool pvEmpty(){
	return (pvQIn == pvQOut);
}
bool pvPut(epsVector_t *pvector, uint8_t vectorType){
    if(pvFull())
    {
         return true;
    }

    epsVectors[pvQIn].heading = pvector->heading;
    epsVectors[pvQIn].distance = pvector->distance;
    epsVectors[pvQIn].speed = pvector->speed;
    epsVectors[pvQIn].lat_a = pvector->lat_a;
    epsVectors[pvQIn].lat_b = pvector->lat_b;
    epsVectors[pvQIn].lon_a = pvector->lon_a;
    epsVectors[pvQIn].lon_b = pvector->lon_b;
    epsVectors[pvQIn].lat_sgn = pvector->lat_sgn;
    epsVectors[pvQIn].lon_sgn = pvector->lon_sgn;
    epsVectors[pvQIn].time = pvector->time;
    epsVectors[pvQIn].index = pvector->index;
    epsVectors[pvQIn].type = vectorType;

    pvQIn = (pvQIn + 1) % PVQ_SIZE;

    return false;
}

pvQElement_t pvGet(void){

	pvQElement_t pvector = {0,0,0,0,0,0,0,0};

    if(pvEmpty())
    {
        return pvector;
    }

    pvector = epsVectors[pvQOut];

    pvQOut = (pvQOut + 1) % PVQ_SIZE;

    return pvector;
}

float epsVectorSpeed(uint32_t last_time,uint32_t currentTime, float distance){
	if(last_time == currentTime || distance == 0)
		return 0;
	uint16_t vartime;
	float speed;
	vartime = currentTime - last_time;
	speed = distance / (vartime / 1000.0f);
	return speed;
}

void epsVectorAddPoint(epsVector_t *last, epsVector_t *current){

		iPoint_t delta;


		delta.speed = current->speed - last->speed;
		delta.heading = current->heading - last->heading;

		if(current->heading < last->heading)
			delta.heading = delta.heading + 360;

		if (delta.heading > 180)
			delta.heading -= 360.0f;
		else if (delta.heading < -180)
			delta.heading += 360.0f;

		iPutPoint(current->time,delta.heading,delta.speed);
}

void epsVectorEstimate(epsVector_t *last, epsVector_t *current, epsVector_t *estimated,epsVectorGain_t gain, uint32_t eps_frequency){
	float angularDistance;
	float headingRadians;
	iPoint_t delta;
	float lat2;
	float lon2;
	float x;
	float y;

	estimatedHeading = current->heading;
	estimatedSpeed = current->speed;
	estimatedTime = millis();
	vartime = estimatedTime - current->time;
	estimatedDistance = current->speed * (vartime / 1000.0f) *(gain.distance / 100.0f);

	if(interpolationOn) {
		delta.heading = 0;
		delta.speed = 0;
		if(iFull()) {
			delta = iEval(estimatedTime);
			estimatedHeading = current->heading + delta.heading * (gain.heading / 100.0f);
			estimatedHeading = fmod(estimatedHeading,360.0f);
			estimatedSpeed = current->speed + delta.speed * (gain.speed / 100.0f);
			estimatedDistance = estimatedSpeed * (vartime / 1000.0f);
		}
	}

	estimatedAccDistance += estimatedDistance;

	angularDistance = estimatedDistance / earthRadius;
	//
	headingRadians = radians(estimatedHeading);
	//
	lat2 = asin(sin(radians(current->lat/TELEMETRY_LATLON_DIVIDER_F))*cos(angularDistance) + cos(radians(current->lat/TELEMETRY_LATLON_DIVIDER_F))*sin(angularDistance)*cos(headingRadians));
	x = cos(angularDistance) - sin(radians(current->lat/TELEMETRY_LATLON_DIVIDER_F)) * sin(lat2);
	y = sin(headingRadians) * sin(angularDistance) * cos(radians(current->lat/TELEMETRY_LATLON_DIVIDER_F));
	lon2 = radians(current->lon/TELEMETRY_LATLON_DIVIDER_F) + atan2(y, x);
	lat2 = degrees(lat2);
	lon2 = fmod(degrees(lon2) + 540,360)-180;
	//
	estimated->lat = lat2 * TELEMETRY_LATLON_DIVIDER_I;
	estimated->lon = lon2 * TELEMETRY_LATLON_DIVIDER_I;
	estimated->lat_a = abs(estimated->lat / TELEMETRY_LATLON_DIVIDER_I);
	estimated->lat_b = abs(estimated->lat % TELEMETRY_LATLON_DIVIDER_I);
	estimated->lat_sgn = (estimated->lat < 0) ? -1 : 1;
	estimated->lon_a = abs(estimated->lon / TELEMETRY_LATLON_DIVIDER_I);
	estimated->lon_b = abs(estimated->lon % TELEMETRY_LATLON_DIVIDER_I);
	estimated->lon_sgn = (estimated->lon < 0) ? -1 : 1;
	estimated->time = estimatedTime;
	estimated->heading = estimatedHeading;
	estimated->speed = estimatedSpeed;
	estimated->distance = estimatedDistance;
	positionIndex++;
	estimated->index = positionIndex;

	if(!pvFull())
		pvPut(estimated,2);
}

void epsVectorsInit(epsVector_t *last, epsVector_t *current, epsVector_t *estimated, uint8_t interpolation,uint8_t points){

	positionIndex=0;

	last->heading = 0;
	last->speed = 0;
	last->distance = 0;
	last->speed = 0;
	last->lat = 0;
	last->lon = 0;
	last->time = 0;
	last->index = 0;

	current->heading = 0;
	current->speed = 0;
	current->distance = 0;
	current->speed = 0;
	current->lat = 0;
	current->lon = 0;
	current->time = 0;
	current->index = 0;

	estimated->heading = 0;
	estimated->speed = 0;
	estimated->distance = 0;
	estimated->speed = 0;
	estimated->lat = 0;
	estimated->lon = 0;
	estimated->time = 0;
	estimated->index = 0;

	pvInit();

	if(interpolation == 1) {
		iInit(points);
		interpolationOn = true;
	}
}

void epsVectorCurrentToLast(epsVector_t *current,epsVector_t *last){
	last->heading = current->heading;
	last->speed = current->speed;
	last->distance = current->distance;
	last->speed = current->speed;
	last->lat = current->lat;
	last->lon = current->lon;
	last->time = current->time;
	last->lat_a = current->lat_a;
	last->lat_b = current->lat_b;
	last->lat_sgn = current->lat_sgn;
	last->lon_a = current->lon_a;
	last->lon_b = current->lon_b;
	last->lon_sgn = current->lon_sgn;
}

void epsVectorLoad(epsVector_t *current,int32_t lat,int32_t lon,float distance, uint32_t last_time, uint32_t currentTime){

	current->lat = lat;
	current->lon = lon;
	current->time = currentTime;
	current->distance = distance;
	current->speed = epsVectorSpeed(last_time,currentTime,distance);
	current->lat_a = abs(current->lat / TELEMETRY_LATLON_DIVIDER_I);
	current->lat_b = abs(current->lat % TELEMETRY_LATLON_DIVIDER_I);
	current->lat_sgn = (current->lat < 0) ? -1 : 1;
	current->lon_a = abs(current->lon / TELEMETRY_LATLON_DIVIDER_I);
	current->lon_b = abs(current->lon % TELEMETRY_LATLON_DIVIDER_I);
	current->lon_sgn = (current->lon < 0) ? -1 : 1;
	positionIndex++;
	current->index = positionIndex;

}

uint16_t getPositionVectorIndex(void){
	return positionIndex++;
}
