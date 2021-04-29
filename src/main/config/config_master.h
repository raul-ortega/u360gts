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
#include "tracker/defines.h"
#pragma once

// System-wide
typedef struct master_t {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE

    uint8_t mixerMode;
    uint32_t enabledFeatures;
    uint16_t looptime;                      // imu loop time in us
    uint8_t emf_avoidance;                   // change pll settings to avoid noise in the uhf band
    uint8_t i2c_overclock;                  // Overclock i2c Bus for faster IMU readings

    motorMixer_t customMotorMixer[MAX_SUPPORTED_MOTORS];
#ifdef USE_SERVOS
    servoMixer_t customServoMixer[MAX_SERVO_RULES];
#endif
    // motor/esc/servo related stuff
    escAndServoConfig_t escAndServoConfig;
    flight3DConfig_t flight3DConfig;

    uint16_t motor_pwm_rate;                // The update rate of motor outputs (50-498Hz)
    uint16_t servo_pwm_rate;                // The update rate of servo outputs (50-498Hz)

    // global sensor-related stuff

    sensorAlignmentConfig_t sensorAlignmentConfig;
    boardAlignment_t boardAlignment;

    //int8_t yaw_control_direction;           // change control direction of yaw (inverted, normal)
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    uint16_t gyro_lpf;                      // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint16_t gyro_cmpf_factor;              // Set the Gyro Weight for Gyro/Acc complementary filter. Increasing this value would reduce and delay Acc influence on the output of the filter.
    uint16_t gyro_cmpfm_factor;             // Set the Gyro Weight for Gyro/Magnetometer complementary filter. Increasing this value would reduce and delay Magnetometer influence on the output of the filter

    gyroConfig_t gyroConfig;

    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
    uint8_t baro_hardware;                  // Barometer hardware to use

    uint16_t max_angle_inclination;         // max inclination allowed in angle (level) mode. default 500 (50 degrees).
    flightDynamicsTrims_t accZero;
    flightDynamicsTrims_t magZero;

    batteryConfig_t batteryConfig;

    rxConfig_t rxConfig;
    inputFilteringMode_e inputFilteringMode;  // Use hardware input filtering, e.g. for OrangeRX PPM/PWM receivers.

    failsafeConfig_t failsafeConfig;

    /*uint8_t retarded_arm;                   // allow disarm/arm on throttle down + roll left/right
    uint8_t disarm_kill_switch;             // allow disarm via AUX switch regardless of throttle value
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
    uint8_t small_angle;*/

    // mixer-related configuration
    mixerConfig_t mixerConfig;

    airplaneConfig_t airplaneConfig;

#ifdef GPS
    gpsConfig_t gpsConfig;
#endif

    serialConfig_t serialConfig;

    telemetryConfig_t telemetryConfig;

/*#ifdef LED_STRIP
    ledConfig_t ledConfigs[MAX_LED_STRIP_LENGTH];
    hsvColor_t colors[CONFIGURABLE_COLOR_COUNT];
#endif*/

    profile_t profile[MAX_PROFILE_COUNT];
    uint8_t current_profile_index;
    controlRateConfig_t controlRateProfiles[MAX_CONTROL_RATE_PROFILE_COUNT];

/*#ifdef BLACKBOX
    uint8_t blackbox_rate_num;
    uint8_t blackbox_rate_denom;
    uint8_t blackbox_device;
#endif*/

    uint8_t magic_ef;                       // magic number, should be 0xEF
    uint8_t chk;                            // XOR checksum
///////////////////////////////
    // PID
    uint16_t p;
    uint16_t i;
    uint16_t d;
    uint8_t max_pid_error;
    uint16_t max_pid_accumulator;
    uint16_t max_pid_gain;
    uint8_t max_pid_divider;
    // EASING
    uint8_t pan_pin;
    uint16_t pan0;
    uint8_t pan0_calibrated;
    uint8_t mag_calibrated;
    uint8_t min_pan_speed;
    int16_t offset;
    int8_t offset_trim;
    uint8_t tilt_pin;
	uint16_t tilt0;
	uint16_t tilt90;
	uint8_t tilt_max_angle;
	uint8_t easing;
	uint8_t easing_steps;
	uint8_t easing_min_angle;
	uint8_t easing_millis;
	uint16_t easing_last_tilt;
	uint16_t telemetry_protocol;
	uint8_t start_tracking_distance;
	uint8_t start_tracking_altitude;
	uint8_t init_servos;
	uint8_t telemetry_min_sats;
	uint8_t telemetry_provider;
	uint8_t telemetry_home;
	uint8_t gps_min_sats;
	uint8_t min_logic_level;
	// NO PID CONTROL SYSTEM
	float nopid_min_delta;
	uint16_t nopid_max_speed;
	uint8_t nopid_map_angle;
	uint8_t eps;
	epsVectorGain_t eps_gain;
	uint16_t eps_frequency;
	uint8_t eps_interpolation;
	uint8_t eps_interpolation_points;
	uint8_t update_home_by_local_gps;
	uint16_t pan_calibration_pulse;
	uint8_t max_speed_filter;
	uint8_t altitude_priority;
	uint8_t pan_inverted;
	uint8_t restore_last_home;
	int32_t home_lat;
	int32_t home_lon;
	int16_t home_alt;
} master_t;

extern master_t masterConfig;
extern profile_t *currentProfile;
extern controlRateConfig_t *currentControlRateProfile;
