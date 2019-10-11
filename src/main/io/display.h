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

//#define ENABLE_DEBUG_OLED_PAGE

typedef enum {
    PAGE_WELCOME,
    PAGE_ARMED,
    PAGE_BATTERY,
    PAGE_TELEMETRY,
	PAGE_CALIBRATING_MAG,
#ifdef GPS
    PAGE_GPS,
#endif
#ifdef ENABLE_DEBUG_OLED_PAGE
    PAGE_DEBUG,
#endif
	PAGE_CLI_MODE,
	PAGE_MENU,
	PAGE_BOOT_MODE
} pageId_e;

typedef enum {
	MENU_IDEL=0,
	MENU_ROOT,
	MENU_CALIBRATE,
	MENU_BATTERY,
	MENU_GPS,
	MENU_TELEMETRY,
	MENU_TELEMETRY_PROTOCOL,
	MENU_TELEMETRY_BAUDRATE,
	/*MENU_SERVOTEST,
	MENU_SET,
	MENU_SOFTSERIAL,*/
	MENU_EPS,
	MENU_EPS_MODE,
	MENU_EPS_DISTANCEGAIN,
	MENU_EPS_FREQUENCY,
	MENU_EASING,
	MENU_EXIT
} menuStates_e;

typedef enum {
	OP_CALIBRATE,
	OP_VBAT,
	OP_GPS,
	OP_TELEMETRY,
	OP_EPS,
	OP_EASING,
	/*OP_SERVOTEST,
	OP_SET,
	OP_SOFTSERIAL,*/
	OP_EXIT
} rootMenu_e;

typedef enum {
	OP_PROTOCOL,
    OP_BAUDRATE,
    OP_TELMETRY_SAVE,
	OP_TELMETRY_EXIT
} telemetryMenu_e;

typedef enum {
	OP_ENABLE,
    OP_DISABLE,
	OP_ENABLEDISABLE_EXIT
} enableDisableMenu_e;

typedef enum {
	OP_MFD,
    OP_GPS_TELEMETRY,
    OP_MAVLINK,
	OP_RVOSD,
	OP_FRSKY_D,
	OP_FRSKY_X,
	OP_LTM,
	OP_PITLAB,
	OP_CROSSFIRE,
    OP_AUTODETECT,
	OP_TELEMETRY_PROTOCOL_EXIT
	/*OP_LTM_FRSKYD*/
} telemetryProtocolMenu_e;

typedef enum {
    OP_AUTO = 0,
	OP_1200,
	OP_2400,
	OP_4800,
    OP_9600,
    OP_19200,
    OP_38400,
    OP_57600,
    OP_115200,
    OP_230400,
    OP_250000,
	OP_TELEMETRY_BAUDRATE_EXIT,
} telemetryBaudlMenu_e;

typedef enum {
    OP_MODE,
	OP_DISTANCEGAIN,
	OP_FREQUENCY,
    OP_EPS_SAVE,
	OP_EPS_EXIT
} epsMenu_e;

typedef enum {
    OP_DISABLED,
	OP_MODE1,
	OP_MODE2,
    OP_MODE3,
	OP_EPS_MODE_EXIT
} epsModeMenu_e;

typedef enum {
    OP_INCREASE,
	OP_DECREASE,
	OP_INCREASEDECREASE_EXIT
} increaseDecreaseMenu_e;

void updateDisplay(void);

void displayShowFixedPage(pageId_e pageId);

void displayEnablePageCycling(void);
void displayDisablePageCycling(void);
void displayResetPageCycling(void);
void displaySetNextPageChangeAt(uint32_t futureMicros);
void updateDisplayProtocolTitle(uint16_t protocol);
void showAutodetectingTitle(uint16_t protocol);
