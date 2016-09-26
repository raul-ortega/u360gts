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

#ifndef SERVOS_H
#define SERVOS_H

#include "config.h"
#include "drivers/pwm_output.h"

void servosInit(void);
void pwmWritePanServo(int16_t A);
void pwmWriteTiltServo(int16_t A);

void servosInit(void)
{
	pwmWritePanServo(masterConfig.pan0);
	pwmWriteTiltServo(masterConfig.tilt0);
}
void pwmWritePanServo(int16_t A){
	pwmWriteServo(panServo, A);
}

void stopPanServo(void){
	pwmWritePanServo(masterConfig.pan0);
}

void pwmWriteTiltServo(int16_t A){
	pwmWriteServo(TILT_SERVO, A);
}
#endif
