/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// Inertial Measurement Unit (IMU)

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/maths.h"

#include "platform.h"
#include "debug.h"

#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "config/runtime_config.h"

#include "tracker/config.h"
#include "tracker/defines.h"

int16_t accSmooth[XYZ_AXIS_COUNT];
int32_t accSum[XYZ_AXIS_COUNT];

uint32_t accTimeSum = 0;        // keep track for integration of acc
int accSumCount = 0;
float accVelScale;

int16_t smallAngle = 0;

float throttleAngleScale;
float fc_acc;

float magneticDeclination = 0.0f;       // calculated at startup from config
float gyroScaleRad;


rollAndPitchInclination_t inclination = { { 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
float anglerad[ANGLE_INDEX_COUNT] = { 0.0f, 0.0f };    // absolute angle inclination in radians

static imuRuntimeConfig_t *imuRuntimeConfig;
static pidProfile_t *pidProfile;
static accDeadband_t *accDeadband;

extern int16_t OFFSET;
extern float DECLINATION;

void imuConfigure(
    imuRuntimeConfig_t *initialImuRuntimeConfig,
    pidProfile_t *initialPidProfile,
    accDeadband_t *initialAccDeadband,
    float accz_lpf_cutoff,
    uint16_t throttle_correction_angle
)
{
    imuRuntimeConfig = initialImuRuntimeConfig;
    pidProfile = initialPidProfile;
    accDeadband = initialAccDeadband;
    //fc_acc = calculateAccZLowPassFilterRCTimeConstant(accz_lpf_cutoff);
    //throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);
}

void imuInit(void)
{
    smallAngle = lrintf(acc_1G * cos_approx(degreesToRadians(imuRuntimeConfig->small_angle)));
    accVelScale = 9.80665f / acc_1G / 10000.0f;
    gyroScaleRad = gyro.scale * (M_PIf / 180.0f) * 0.000001f;
}



/*
* Baseflight calculation by Luggi09 originates from arducopter
* ============================================================
* This function rotates magnetic vector to cancel actual yaw and
* pitch of craft. Then it computes it's direction in X/Y plane.
* This value is returned as compass heading, value is 0-360 degrees.
*
* Note that Earth's magnetic field is not parallel with ground unless
* you are near equator. Its inclination is considerable, >60 degrees
* towards ground in most of Europe.
*
* First we consider it in 2D:
*
* An example, the vector <1, 1> would be turned into the heading
* 45 degrees, representing it's angle clockwise from north.
*
*      ***************** *
*      *       |   <1,1> *
*      *       |  /      *
*      *       | /       *
*      *       |/        *
*      *       *         *
*      *                 *
*      *                 *
*      *                 *
*      *                 *
*      *******************
*
* //TODO: Add explanation for how it uses the Z dimension.
*/


int16_t imuCalculateHeading(t_fp_vector *vec)
{
    int16_t head;

    float cosineRoll = cos_approx(anglerad[AI_ROLL]);
    float sineRoll = sin_approx(anglerad[AI_ROLL]);
    float cosinePitch = cos_approx(anglerad[AI_PITCH]);
    float sinePitch = sin_approx(anglerad[AI_PITCH]);
    float Xh = vec->A[X] * cosinePitch + vec->A[Y] * sineRoll * sinePitch + vec->A[Z] * sinePitch * cosineRoll;
    float Yh = vec->A[Y] * cosineRoll - vec->A[Z] * sineRoll;
    //TODO: Replace this comment with an explanation of why Yh and Xh can never simultanoeusly be zero,
    // or handle the case in which they are and (atan2f(0, 0) is undefined.
    //float hd = (atan2f(Yh, Xh) * 1800.0f / M_PIf + magneticDeclination) / 1.0f; //10.0f;
    float hd = atan2f(Yh, Xh);
    //head = lrintf(hd);

    if (hd < 0)
    	hd += 2 * M_PIf;

    if (hd > 2 * M_PIf)
    	hd -= 2 * M_PIf;

    return (int16_t) ((hd * 1800.0 / M_PIf) + magneticDeclination + OFFSET * 10.0f)%3600;
}
