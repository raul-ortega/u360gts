/*
 * This file is part of u360gts, aka amv-open360tracker 32bits:
 * https://github.com/raul-ortega/u360gts
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

void pwm360_setMasterPulses(uint16_t pan0,uint16_t pan90,uint16_t pan180,uint16_t pan270,uint16_t pan360,uint16_t tilt0,uint16_t tilt90);
uint16_t pwm360_getTilt(void);
uint16_t pwm360_getheading(void);
void pwm360_encodeTargetData(uint8_t c);


