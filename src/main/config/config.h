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

#pragma once

#define MAX_PROFILE_COUNT 3
#define MAX_CONTROL_RATE_PROFILE_COUNT 3
#define ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS 1500


typedef enum {
    FEATURE_VBAT = 1 << 0,
    FEATURE_SERVO_TILT = 1 << 1,
    FEATURE_SOFTSERIAL = 1 << 2,
    FEATURE_GPS = 1 << 3,
    FEATURE_SONAR = 1 << 4,
    FEATURE_TELEMETRY = 1 << 5,
    FEATURE_CURRENT_METER = 1 << 6,
    FEATURE_DISPLAY = 1 << 7,
    FEATURE_BLACKBOX = 1 << 8,
	FEATURE_EASING = 1 << 9,
	FEATURE_NOPID = 1 << 10,
	FEATURE_DEBUG = 1 << 11,
	FEATURE_EPS = 1 << 12
} features_e;

uint16_t pwmPan0;
uint16_t pwmPanCalibrationPulse;

int16_t OFFSET;
int8_t OFFSET_TRIM;
float DECLINATION;

void handleOneshotFeatureChangeOnRestart(void);
void latchActiveFeatures(void);
bool featureConfigured(uint32_t mask);
bool feature(uint32_t mask);
void featureSet(uint32_t mask);
void featureClear(uint32_t mask);
void featureClearAll(void);
uint32_t featureMask(void);

void copyCurrentProfileToProfileSlot(uint8_t profileSlotIndex);

void initEEPROM(void);
void resetEEPROM(void);
void readEEPROM(void);
void readEEPROMAndNotify(void);
void writeEEPROM();
void ensureEEPROMContainsValidData(void);
void saveConfigAndNotify(void);

uint8_t getCurrentProfile(void);
void changeProfile(uint8_t profileIndex);

uint8_t getCurrentControlRateProfile(void);
void changeControlRateProfile(uint8_t profileIndex);

bool canSoftwareSerialBeUsed(void);

uint16_t getCurrentMinthrottle(void);
