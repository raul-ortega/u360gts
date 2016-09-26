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

#include "interpolation.h"

#define INT_ELEMENTS	10

iPoint_t iPoints[INT_ELEMENTS];

uint8_t INT_QSIZE;
uint8_t iQIn;
uint8_t iQOut;

void iInit(uint8_t n)
{
	INT_QSIZE = n;
	iQIn = 0;
	iQOut = 0;
}

bool iPutPoint(uint32_t time,float heading,float speed){
	iPoint_t point;
	point.time = time;
	point.heading = heading;
	point.speed = speed;
	if(iPut(point)) {
		iGet();
		iPut(point);
	}
	return iFull();
}

bool iFull(){
	return (iQIn == (( iQOut - 1 + INT_QSIZE) % INT_QSIZE));
}

bool iPut(iPoint_t point){
    if(iFull())
    {
         return true; /* Full */
    }

    iPoints[iQIn].time = point.time;
    iPoints[iQIn].heading = point.heading;
    iPoints[iQIn].speed = point.speed;

    iQIn = (iQIn + 1) % INT_QSIZE;

    return false; // No errors
}

bool iGet(){
    if(iQIn == iQOut)
    {
        return true; /* Queue Empty - nothing to get*/
    }

    iQOut = (iQOut + 1) % INT_QSIZE;

    return false; // No errors
}

iPoint_t iEval(float A){
	int i;
	int j;
	int index_i = iQIn;
	int index_j = index_i;
	float dif_Aj;
	float dif_ij;
	iPoint_t S;
	iPoint_t T;
	iPoint_t K;
	K.heading = 0.0f;
	K.speed = 0.0f;
	for(i=0; i<INT_QSIZE - 1; i++)
	{
		index_i = (index_i + 1) % INT_QSIZE;
		index_j=iQIn;
		S.heading = 1.0f;
		T.heading = 1.0f;
		S.speed = 1.0f;
		T.speed = 1.0f;
		for(j=0; j<INT_QSIZE - 1; j++)
		{
			index_j = (index_j + 1) % INT_QSIZE;
			if(index_j!=index_i)
			{
				dif_Aj = (A - iPoints[index_j].time);
				dif_ij = (iPoints[index_i].time - iPoints[index_j].time);
				S.heading = S.heading * dif_Aj;
				T.heading = T.heading * dif_ij;
				S.speed = S.speed * dif_Aj;
				T.speed = T.speed * dif_ij;
			}
		}
		K.time = A;
		K.heading = K.heading +((S.heading / T.heading) * iPoints[index_i].heading);
		K.speed = K.speed + ((S.speed/T.speed) * iPoints[index_i].speed);
	}
	return K;
}
