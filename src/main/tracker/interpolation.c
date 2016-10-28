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
	int index_i = iQIn - 1;
	int index_j = index_i;
	iPoint_t l;
	iPoint_t v;
	v.heading = 0;
	v.speed = 0;
	for(i=1; i<INT_QSIZE + 1; i++)
	{
		index_i = (index_i + 1) % INT_QSIZE;
		index_j = iQIn - 1;;
		l.heading = iPoints[index_i].heading;
		l.speed = iPoints[index_i].speed;
		for(j=1; j<INT_QSIZE + 1; j++)
		{
			index_j = (index_j + 1) % INT_QSIZE;

			if(index_j!=index_i)
			{
				l.heading = (l.heading * (A - iPoints[index_j].time * 1.0f))/(iPoints[index_i].time * 1.0f - iPoints[index_j].time * 1.0f);
				l.speed = (l.speed * (A - iPoints[index_j].time))/(iPoints[index_i].time - iPoints[index_j].time);
				//printf("%d %d %d %d %.2f %d %d %d\n",i,j,index_i,index_j,A,iPoints[index_i].time,iPoints[index_j].time);
			}
		}
		v.heading = v.heading + l.heading;
		v.speed = v.speed + l.speed;
	}
	v.time = A;
	return v;
}
