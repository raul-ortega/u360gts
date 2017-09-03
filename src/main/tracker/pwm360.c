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
#include "math.h"
#include "pwm360.h"

uint16_t pwm360_pan;
uint16_t pwm360_tilt;

uint8_t pwm360_read_checksum = 0;
uint8_t pwm360_calc_checksum = 0;
uint8_t pwm360_checksum_index = 0;

uint16_t master_pwm_pan0;
uint16_t master_pwm_pan90;
uint16_t master_pwm_pan180;
uint16_t master_pwm_pan270;
uint16_t master_pwm_pan360;
uint16_t master_pwm_tilt0;
uint16_t master_pwm_tilt90;

void pwm360_setMasterPulses(uint16_t pan0,uint16_t pan90,uint16_t pan180,uint16_t pan270,uint16_t pan360,uint16_t tilt0,uint16_t tilt90){
	master_pwm_pan0 = pan0;
	master_pwm_pan90 = pan90;
	master_pwm_pan180 = pan180;
	master_pwm_pan270 = pan270;
	master_pwm_pan360 = pan360;
	master_pwm_tilt0 = tilt0;
	master_pwm_tilt90 = tilt90;
}

uint16_t pwm360_getTilt(void) {
  return map(pwm360_tilt, master_pwm_tilt0, master_pwm_tilt90,0, 90);
}

uint16_t pwm360_getheading(void) {
  uint16_t pwm1;
  uint16_t pwm2;
  if(pwm360_pan >= master_pwm_pan0 && pwm360_pan <= master_pwm_pan360) {
	  if(pwm360_pan >= master_pwm_pan0 && pwm360_pan < master_pwm_pan90) {
		  pwm1 = master_pwm_pan0;
		  pwm2 = master_pwm_pan90 - 1;
		  return (uint16_t)map(pwm360_pan, pwm1, pwm2,0, 89);
	  } else if(pwm360_pan >= master_pwm_pan90 && pwm360_pan < master_pwm_pan180) {
		  pwm1 = master_pwm_pan90;
		  pwm2 = master_pwm_pan180 - 1;
		  return (uint16_t)map(pwm360_pan, pwm1, pwm2,90, 179);
	  } else if(pwm360_pan >= master_pwm_pan180 && pwm360_pan < master_pwm_pan270) {
		  pwm1 = master_pwm_pan180;
		  pwm2 = master_pwm_pan270 - 1;
		return (uint16_t)map(pwm360_pan, pwm1, pwm2,180,269);
	  } else if(pwm360_pan >= master_pwm_pan270 && pwm360_pan <= master_pwm_pan360) {
		  pwm1 = master_pwm_pan270;
		  pwm2 = master_pwm_pan360;
		return (uint16_t)map(pwm360_pan, pwm1, pwm2,270, 360);
	  }
  } else
	  return 0;
}

uint8_t pwm360_state = 0;

void pwm360_encodeTargetData(uint8_t c) {
#ifdef TRACKER_DEBUG
  //Serial.write(c);
#endif
  if (c == 'P') {
    //PAN
	pwm360_pan = 0;
    pwm360_state = 1;
    pwm360_read_checksum = 0;
    pwm360_calc_checksum = c;
    pwm360_checksum_index = 0;
    return;
  } else if (c == 'T') {
    //TILT
    pwm360_state = 2;
    pwm360_calc_checksum ^= c;
    return;
  } else if (c == '*') {
    //checksum
    pwm360_checksum_index = 0;
    pwm360_state = 3;
    return;
  } else if (pwm360_checksum_index == 2) {
    //newline
    if (pwm360_calc_checksum == pwm360_read_checksum) {
      gotFix = true;
    }
    pwm360_state = 0;
    return;
  } else if (c == 'K') {
    //home set
    settingHome = true;
    return;
  }

  switch (pwm360_state) {
    case 1:
      pwm360_pan = (pwm360_pan * 10) + (c - 48);
      pwm360_calc_checksum ^= c;
      break;
    case 2:
      pwm360_tilt = (pwm360_tilt * 10) + (c - 48);
      pwm360_calc_checksum ^= c;
      break;
    case 3:
      pwm360_read_checksum *= 16;
      if (c >= '0' && c <= '9'){
        pwm360_read_checksum += (c - '0');
      }
      else{
        pwm360_read_checksum += (c-'A'+10);
      }
      pwm360_checksum_index++;
      break;
  }
}
