#ifndef MATH_H
#define MATH_H

#include <stdint.h> //#include "inttypes.h"

#include "defines.h"
//#include "Arduino.h"

void calcTargetDistanceAndHeading(positionVector_t *tracker, positionVector_t *target);
void setHome(positionVector_t *tracker, positionVector_t *target);

//Easing functions
float easeTilt(float t, float b, float c, float d);
float easeOutQuart(float t, float b, float c, float d);
float easeOutCirc(float t, float b, float c, float d);
float easeOutExpo(float t, float b, float c, float d);
float easeOutCubic(float t, float b, float c, float d);
float map(long x, long in_min, long in_max, long out_min, long out_max);
#endif
