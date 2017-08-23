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

// FIXME some of these are flight modes, some of these are general status indicators
typedef enum {
    OK_TO_ARM       = (1 << 0),
    PREVENT_ARMING  = (1 << 1),
    ARMED           = (1 << 2)
} armingFlag_e;

extern uint8_t armingFlags;

#define DISABLE_ARMING_FLAG(mask) (armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask) (armingFlags |= (mask))
#define ARMING_FLAG(mask) (armingFlags & (mask))

typedef enum {
    ANGLE_MODE      = (1 << 0),
    HORIZON_MODE    = (1 << 1),
    MAG_MODE        = (1 << 2),
    BARO_MODE       = (1 << 3),
    GPS_HOME_MODE   = (1 << 4),
    GPS_HOLD_MODE   = (1 << 5),
    HEADFREE_MODE   = (1 << 6),
    UNUSED_MODE     = (1 << 7), // old autotune
    PASSTHRU_MODE   = (1 << 8),
    SONAR_MODE      = (1 << 9),
    FAILSAFE_MODE   = (1 << 10),
    GTUNE_MODE      = (1 << 11),
} flightModeFlags_e;

extern uint16_t flightModeFlags;

#define DISABLE_FLIGHT_MODE(mask) disableFlightMode(mask)
#define ENABLE_FLIGHT_MODE(mask) enableFlightMode(mask)
#define FLIGHT_MODE(mask) (flightModeFlags & (mask))

typedef enum {
    GPS_FIX_HOME   = (1 << 0),
    GPS_FIX        = (1 << 1),
    CALIBRATE_MAG  = (1 << 2),
    SMALL_ANGLE    = (1 << 3),
    FIXED_WING     = (1 << 4),                   // set when in flying_wing or airplane mode. currently used by althold selection code
	BOOT_MODE	   = (1 << 5),
	CALIBRATE_PAN = (1 << 6),
} stateFlags_t;

extern uint8_t stateFlags;

#define DISABLE_STATE(mask) (stateFlags &= ~(mask))
#define ENABLE_STATE(mask) (stateFlags |= (mask))
#define STATE(mask) (stateFlags & (mask))

/*typedef enum {
	homeSet 			= (1 << 0),
	trackingStarted 	= (1 << 1),
	currentState 		= (1 << 2),
	previousState 		= (1 << 3),
	gotAlt 			= (1 << 4),
	gotFix 			= (1 << 5),
	settingHome 		= (1 << 6),
	gotNewHeading			= (1 << 7)
} trackingFlags_t;*/

typedef enum {
	TP_SERVOTEST		= (1 << 0),
	TP_CALIBRATING_MAG	= (1 << 1),
	TP_MFD				= (1 << 2),
	TP_GPS_TELEMETRY	= (1 << 3),
	TP_MAVLINK			= (1 << 4),
	TP_RVOSD			= (1 << 5),
	TP_FRSKY_D			= (1 << 6),
	TP_FRSKY_X 			= (1 << 7),
	TP_LTM				= (1 << 8),
	TP_PITLAB			= (1 << 9),
	TP_PWM360			= (1 << 10),
	TP_LTM_FRSKYD		= (1 << 11),
	TP_CALIBRATING_PAN0	= (1 << 12),
	TP_CALIBRATING_MAXPAN	= (1 << 13),

} trackerProtocolFlags_t;

extern uint16_t trackerProtocolFlags;

#define DISABLE_PROTOCOL(mask) (trackerProtocolFlags &= ~(mask))
#define ENABLE_PROTOCOL(mask) (trackerProtocolFlags |= (mask))
#define PROTOCOL(mask) (trackerProtocolFlags & (mask))

typedef enum {
	SERVOPAN_MOVE 		= (1 << 0),
	SERVOTILT_MOVE		= (1 << 1),
} servotestFlags_t;

extern uint8_t servotestFlags;

#define DISABLE_SERVO(mask) (servotestFlags &= ~(mask))
#define ENABLE_SERVO(mask) (servotestFlags |= (mask))
#define SERVO(mask) (servotestFlags & (mask))

extern uint16_t pwmPan0;

typedef enum {
	HOME_BUTTON = (1 << 0),
	MENU_BUTTON	= (1 << 1),
} buttonsFlags_t;

extern uint8_t buttonsFlags;

#define DISABLE_BUTTON(mask) (buttonsFlags &= ~(mask))
#define ENABLE_BUTTON(mask) (buttonsFlags |= (mask))
#define BUTTON(mask) (buttonsFlags & (mask))


uint16_t enableFlightMode(flightModeFlags_e mask);
uint16_t disableFlightMode(flightModeFlags_e mask);

bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);

void mwDisarm(void);
