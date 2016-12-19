/*
 * This file is part of u360gts, aka amv-open360tracker 32bits:
 * https://github.com/raul-ortega/amv-open360tracker-32bits
 *
 * The code bellow uses Easing Ecuations by Robert Penner http://www.gizma.com/easing/
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

#include "math.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

///////
#include "platform.h"
#include "version.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/color.h"
#include "common/typeconversion.h"

#include "drivers/system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"


#include "io/escservo.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/rc_controls.h"
#include "io/serial.h"
#include "io/serial_1wire.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/beeper.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"

#include "tracker/gps_estimation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

extern master_t masterConfig;

float lonScale = 1.0f;

enum {
	EASE_OUT_QRT = 1,
	EASE_OUT_CIRC,
	EASE_OUT_EXPO,
	EASE_OUT_CUBIC,
};

float easeTilt(float t, float b, float c, float d) {
  if(masterConfig.easing == EASE_OUT_QRT)
	  return easeOutQuart(t, b, c, d);
  else if(masterConfig.easing == EASE_OUT_CIRC)
	  return easeOutCirc(t, b, c, d);
  else if(masterConfig.easing == EASE_OUT_EXPO)
 	  return easeOutExpo(t, b, c, d);
  else if(masterConfig.easing == EASE_INOUT_CUBIC)
 	  return easeInOutCubic(t, b, c, d);
  else if(masterConfig.easing == EASE_OUT_CUBIC)
 	  return easeOutCubic(t, b, c, d);
  else
	  return easeOutQuart(t, b, c, d);
}

float easeOutQuart(float t, float b, float c, float d) {
	return -c * ((t=t/d-1)*t*t*t - 1) + b;
}

float easeOutCirc(float t, float b, float c, float d) {
	t /= d/2;
	if (t < 1) return -c/2 * (sqrt(1 - t*t) - 1) + b;
	t -= 2;
	return c/2 * (sqrt(1 - t*t) + 1) + b;
}

float easeOutExpo(float t, float b, float c, float d) {
	return (t==d) ? b+c : c * (-pow(2, -10 * t/d) + 1) + b;
}

float easeInOutCubic(float t, float b, float c, float d) {
	t /= d/2;
	if (t < 1) return c/2*t*t*t + b;
	t -= 2;
	return c/2*(t*t*t + 2) + b;
}

float easeOutCubic(float t, float b, float c, float d) {
	t /= d;
	t--;
	return c*(t*t*t + 1) + b;
}

