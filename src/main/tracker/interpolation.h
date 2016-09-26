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

#ifndef SRC_MAIN_TRACKER_LAGRANGE_H_
#define SRC_MAIN_TRACKER_LAGRANGE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

typedef struct {
	uint32_t time;
	float heading;
	float speed;
} iPoint_t;

void iInit(uint8_t n);
bool iPutPoint(uint32_t time,float heading,float speed);
bool iPut(iPoint_t point);
bool iGet();
bool iFull();
iPoint_t iEval(float A);

#endif /* SRC_MAIN_TRACKER_LAGRANGE_H_ */
