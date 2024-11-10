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

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "version.h"

#include "build_config.h"

#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "common/printf.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/typeconversion.h"

#ifdef DISPLAY

#include "display_lib.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "rx/rx.h"
#include "io/rc_controls.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"

#ifdef GPS
#include "io/gps.h"
#include "flight/navigation.h"
#endif

#include "config/runtime_config.h"
#include "config/config.h"
#include "display.h"
#include "tracker/defines.h"


extern int32_t telemetry_lat;
extern int32_t telemetry_lon;
extern int16_t telemetry_alt;
extern int16_t telemetry_sats;
extern uint8_t telemetry_failed_cs;
extern uint8_t telemetry_fixtype;
extern uint8_t telemetry_frequency;
extern int16_t telemetry_voltage;
extern positionVector_t targetPosition;
extern positionVector_t trackerPosition;
extern bool gotFix;
extern bool gotTelemetry;
extern bool lostTelemetry;
extern bool homeSet_BY_GPS;
extern bool homeSet_BY_LAST;
extern bool homeSet;
extern bool homeReset;
extern uint16_t pwmPan;
extern uint16_t pwmTilt;
extern uint8_t SERVOTEST_TILT;
extern int8_t OFFSET_TRIM;
extern uint8_t OFFSET_TRIM_STATE;
extern uint8_t EPS_MODE;
extern uint16_t EPS_DISTANCE_GAIN;
extern uint16_t EPS_FREQUENCY;
extern uint16_t rssi;

#define DISPLAY_UPDATE_PERIOD_US    (MICROSECONDS_IN_A_SECOND / 5)
#define PAGE_CYCLE_PERIOD_US        (MICROSECONDS_IN_A_SECOND * 5)
#define PAGE_TOGGLE_PERIOD_US       (MICROSECONDS_IN_A_SECOND / 2)

#define PAGE_TITLE_LINE_COUNT                   1
#define HALF_SCREEN_CHARACTER_COLUMN_COUNT      (SCREEN_CHARACTER_COLUMN_COUNT / 2)
#define IS_SCREEN_CHARACTER_COLUMN_COUNT_ODD    (SCREEN_CHARACTER_COLUMN_COUNT & 1)
#define RX_CHANNELS_PER_PAGE_COUNT              14

typedef enum {
    PAGE_STATE_FLAG_NONE = 0,
    PAGE_STATE_FLAG_CYCLE_ENABLED = (1 << 0),
    PAGE_STATE_FLAG_FORCE_PAGE_CHANGE = (1 << 1)
} pageFlags_e;

typedef struct pageState_s {
    bool pageChanging;
    pageId_e pageId;
    pageId_e pageIdBeforeArming;
    uint8_t pageFlags;
    uint8_t cycleIndex;
    uint32_t nextPageAt;
    uint32_t nextToggleAt;
    uint8_t toggleCounter;
} pageState_t;


static uint8_t u360gts_logo[] = { 128, 32,
    '\x00','\x00','\x07','\x80','\xc0','\xe0','\xf0','\xf8', // 0x0008
    '\xf8','\x02','\xe0','\xe0','\x04','\xc0','\xc0','\x02', // 0x0010
    '\x80','\x80','\x02','\x00','\x00','\x6b','\xe0','\xe0', // 0x0018
    '\x03','\x02','\x07','\x07','\x02','\x1e','\x3c','\x38', // 0x0020
    '\xe1','\xe1','\x03','\xf1','\xf3','\xb8','\xb8','\x02', // 0x0028
    '\xf8','\xf0','\xe1','\x01','\x03','\x07','\x0f','\x3c', // 0x0030
    '\xf8','\xf0','\x80','\x00','\x00','\x04','\xf0','\xf0', // 0x0038
    '\x04','\x00','\x00','\x04','\xf0','\xf0','\x03','\x00', // 0x0040
    '\x1c','\x1e','\x1e','\x04','\x9e','\x9e','\x05','\xfe', // 0x0048
    '\xfe','\x04','\x00','\xfc','\xfe','\xfe','\x04','\x9e', // 0x0050
    '\x9e','\x08','\x1e','\x18','\xfe','\xfe','\x04','\x1e', // 0x0058
    '\x1e','\x06','\xfe','\xfe','\x04','\x00','\xf0','\xf0', // 0x0060
    '\x04','\x70','\x70','\x09','\x60','\x60','\x02','\x70', // 0x0068
    '\x70','\x04','\xf0','\xf0','\x04','\x70','\x70','\x05', // 0x0070
    '\x00','\xf0','\xf0','\x03','\x70','\x70','\x08','\xff', // 0x0078
    '\xff','\x02','\xe0','\x00','\x00','\x04','\x3e','\x7f', // 0x0080
    '\x77','\xf7','\xf7','\x02','\xf8','\xfd','\xfe','\xff', // 0x0088
    '\xbf','\x1f','\x0f','\x00','\x00','\x04','\x80','\xff', // 0x0090
    '\xff','\x02','\x3f','\x00','\x00','\x04','\xff','\xff', // 0x0098
    '\x04','\xc0','\xc0','\x04','\xff','\xff','\x03','\x00', // 0x00a0
    '\xc0','\xc0','\x05','\xcf','\xcf','\x05','\xff','\xff', // 0x00a8
    '\x04','\x00','\xff','\xff','\x05','\xc7','\xcf','\xcf', // 0x00b0
    '\x04','\xff','\xff','\x04','\xfe','\xff','\xff','\x04', // 0x00b8
    '\xc0','\xc0','\x06','\xff','\xff','\x04','\x00','\xff', // 0x00c0
    '\xff','\x04','\xc0','\xc2','\xc6','\xc6','\x02','\xce', // 0x00c8
    '\xce','\x03','\xfe','\xfe','\x03','\x00','\x00','\x05', // 0x00d0
    '\xff','\xff','\x04','\x00','\x00','\x06','\xcf','\xcf', // 0x00d8
    '\x03','\xce','\xce','\x04','\xfe','\xfe','\x04','\x00', // 0x00e0
    '\x01','\x07','\x0f','\x1e','\x3c','\x38','\x70','\x70', // 0x00e8
    '\x02','\xe7','\xe7','\x03','\xc7','\xc7','\x03','\xe7', // 0x00f0
    '\xe7','\x03','\x70','\x70','\x02','\x38','\x1c','\x1e', // 0x00f8
    '\x0f','\x07','\x01','\x00','\x00','\x05','\x01','\x01', // 0x0100
    '\x02','\x03','\x03','\x08','\x01','\x00','\x00','\x02', // 0x0108
    '\x01','\x03','\x03','\x0b','\x01','\x00','\x01','\x03', // 0x0110
    '\x03','\x0c','\x01','\x00','\x01','\x03','\x03','\x0c', // 0x0118
    '\x01','\x00','\x01','\x01','\x02','\x03','\x03','\x0a', // 0x0120
    '\x01','\x01','\x02','\x00','\x00','\x05','\x03','\x03', // 0x0128
    '\x04','\x00','\x00','\x06','\x01','\x03','\x03','\x09', // 0x0130
    '\x01',
};

static const char* const pageTitles[] = {
    "U360GTS",
    "ARMED",
    "BATTERY",
    "TELEMETRY",
	"PLEASE WAIT"
#ifdef GPS
    ,"GPS"
#endif
#ifdef ENABLE_DEBUG_OLED_PAGE
    ,"DEBUG"
#endif
	,"CLI MODE"
	,"MAIN MENU"
};

static const pageId_e cyclePageIds[] = {
    PAGE_TELEMETRY,
#ifdef GPS
    PAGE_GPS,
#endif
    PAGE_BATTERY
#ifdef ENABLE_DEBUG_OLED_PAGE
    ,PAGE_DEBUG,
#endif
};

static const char* const telemetry_protocols_Titles[]={
	"SERVOTEST     ",
	"CALIBRATING_MAG",
	"MFD           ",
	"GPS_TELEMETRY ",
	"MAVLINK       ",
	"RVOSD         ",
	"FRSKY_D       ",
	"FRSKY_X       ",
	"LTM           ",
	"PITLAB        ",
	"CROSSFIRE     "
};

static const char* tickerCharacters = "|/-\\"; // use 2/4/8 characters so that the divide is optimal.

#define CYCLE_PAGE_ID_COUNT (sizeof(cyclePageIds) / sizeof(cyclePageIds[0]))
#define TICKER_CHARACTER_COUNT (sizeof(tickerCharacters) / sizeof(char))

// Menu
const char* const rootMenu[] = {
	"CALIBRATE    ",
  	"BATTERY      ",
	"GPS          ",
    "TELEMETRY    ",
	"EPS          ",
	"EASING       ",
	/*"SERVOS       ",
	"SET PARAMS   ",
	"SOFTSERIAL   ",*/
    "SAVE         "
};

static const char * const telemetryMenu[] = {
	"PROTOCOL     ",
    "BAUDRATE     ",
	"SAVE         ",
	"EXIT         "
};

static const char * const enableDisableMenu[] = {
	"ENABLE       ",
    "DISABLE      ",
	"EXIT         "
};

static const char* const telemetryProtocolMenu[] = {
	"MFD          ",
	"GPS TELEMETRY",
	"MAVLINK      ",
	"RVOSD        ",
	"FRSKY D      ",
	"FRSKY X      ",
	"LTM          ",
	"PITLAB       ",
	"CROSSFIRE    ",
	"AUTODETECT   ",
	"EXIT         "
};

static const char* const telemetryBaudrateMenu[] = {
	"AUTO         ",
	"1200         ",
	"2400         ",
	"4800         ",
	"9600         ",
	"19200        ",
	"38400        ",
	"57600        ",
	"115200       ",
	"230400       ",
	"250000       ",
	"EXIT         "
};

static const char* const epsModeMenu[]={
	"DISABLED     ",
	"MODE 1       ",
	"MODE 2       ",
	"MODE 1+2     ",
	"SAVE         ",
	"EXIT         "
};

static const char* const epsMenu[]={
	"MODE         ",
	"DISTANCE GAIN",
	"FREQUENCY    ",
	"SAVE         ",
	"EXIT         "
};

static const char* const epsParamIncreaseDecrease[] = {
	"+10          ",
	"-10          ",
	"EXIT         "
};

static const char* const telemetryFixType[] = {
	"none ",
	"nofix",
	"2d   ",
	"3d   ",
	"dgps ",
	"float",
	"fixed",
	"stati",
	"ppp  "
};


int16_t master_telemetry_protocol;
uint8_t display_type;
static uint32_t nextDisplayUpdateAt = 0;
static bool displayPresent = false;
static rxConfig_t *rxConfig;
static char lineBuffer[SCREEN_CHARACTER_COLUMN_COUNT + 1];
static pageState_t pageState;
uint8_t indexMenuOption = 0;
static uint8_t maxMenuOptions = 5;
uint8_t menuState = 0;


controlRateConfig_t *getControlRateConfig(uint8_t profileIndex);


static void resetDisplay(void) 
{
    displayPresent = OLED_init(display_type);
}

static void LCDprint(uint8_t i) 
{
   OLED_send_char(i);
}

static void padLineBuffer(void)
{
    uint8_t length = strlen(lineBuffer);
    while (length < sizeof(lineBuffer) - 1)
    {
        lineBuffer[length++] = ' ';
    }
    lineBuffer[length] = 0;
}

static void padHalfLineBuffer(void)
{
    uint8_t halfLineIndex = sizeof(lineBuffer) / 2;
    uint8_t length = strlen(lineBuffer);
    while (length < halfLineIndex - 1)
    {
        lineBuffer[length++] = ' ';
    }
    lineBuffer[length] = 0;
}

// LCDbar(n,v) : draw a bar graph - n number of chars for width, v value in % to display
static void drawHorizonalPercentageBar(uint8_t width, uint8_t percent)
{
    uint8_t i, j;

    if (percent > 100)
        percent = 100;

    j = (width * percent) / 100;

    for (i = 0; i < j; i++)
        LCDprint(159); // full

    if (j < width)
        LCDprint(154 + (percent * width * 5 / 100 - 5 * j)); // partial fill

    for (i = j + 1; i < width; i++)
        LCDprint(154); // empty
}

#if 0
void fillScreenWithCharacters()
{
    for (uint8_t row = 0; row < SCREEN_CHARACTER_ROW_COUNT; row++) 
    {
        for (uint8_t column = 0; column < SCREEN_CHARACTER_COLUMN_COUNT; column++) 
        {
            OLED_set_xy(column, row);
            OLED_send_char('A' + column);
        }
    }
}
#endif

static void updateTicker(void)
{
    static uint8_t tickerIndex = 0;
    OLED_set_xy(SCREEN_CHARACTER_COLUMN_COUNT - 1, 0);
    OLED_send_char(tickerCharacters[tickerIndex]);
    if(!lostTelemetry || PROTOCOL(TP_CALIBRATING_MAG)){
		tickerIndex++;
		tickerIndex = tickerIndex % TICKER_CHARACTER_COUNT;
    }
}

static void updateRxStatus(void)
{
    OLED_set_xy(SCREEN_CHARACTER_COLUMN_COUNT - 2, 0);
    char rxStatus = '!';
    if (rxIsReceivingSignal()) {
        rxStatus = 'r';
    } if (rxAreFlightChannelsValid()) {
        rxStatus = 'R';
    }
    OLED_send_char(rxStatus);
}

#if 0
void updateFailsafeStatus(void)
{
    char failsafeIndicator = '?';
    switch (failsafePhase()) 
    {
        case FAILSAFE_IDLE:
            failsafeIndicator = '-';
            break;
        case FAILSAFE_RX_LOSS_DETECTED:
            failsafeIndicator = 'R';
            break;
        case FAILSAFE_LANDING:
            failsafeIndicator = 'l';
            break;
        case FAILSAFE_LANDED:
            failsafeIndicator = 'L';
            break;
        case FAILSAFE_RX_LOSS_MONITORING:
            failsafeIndicator = 'M';
            break;
        case FAILSAFE_RX_LOSS_RECOVERED:
            failsafeIndicator = 'r';
            break;
    }
    OLED_set_xy(SCREEN_CHARACTER_COLUMN_COUNT - 3, 0);
    OLED_send_char(failsafeIndicator);
}
#endif

static void showTitle(void)
{
    OLED_set_line(0);
    // OLED_send_string(pageTitles[pageState.pageId]);
    if (pageState.pageId == PAGE_TELEMETRY)
    {
        int16_t i;
        for (i = 0; i < OP_CROSSFIRE + 3; i++)
        {
            if (master_telemetry_protocol & (1 << i))
            {
                OLED_send_string(telemetry_protocols_Titles[i]);
                if (feature(FEATURE_EPS) && !PROTOCOL(TP_MFD))
                {
                    tfp_sprintf(lineBuffer, " EPS%d", EPS_MODE);
                    OLED_send_string(lineBuffer);
                }
                else
                    OLED_send_string("      ");
                break;
            }
        }
        if (master_telemetry_protocol == 0 && feature(FEATURE_AUTODETECT))
        {
            tfp_sprintf(lineBuffer, "Auto detecting");
            OLED_send_string(lineBuffer);
        }
    }
    else if (pageState.pageId == PAGE_MENU)
    {
        return;
    }
    else if (pageState.pageId == PAGE_BATTERY)
    {
        if (feature(FEATURE_VBAT))
            OLED_send_string(pageTitles[pageState.pageId]);
    }
    else
    {
        OLED_send_string(pageTitles[pageState.pageId]);
    }
}

static void handlePageChange(void)
{
    OLED_clear_display();
    if (pageState.pageId != PAGE_WELCOME) 
    {
        showTitle();
    }
}

static void drawRxChannel(uint8_t channelIndex, uint8_t width)
{
    uint32_t percentage;
    LCDprint(rcChannelLetters[channelIndex]);
    percentage = (constrain(rcData[channelIndex], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
    drawHorizonalPercentageBar(width - 1, percentage);
}

static void displaySetPage(pageId_e pageId)
{
    pageState.pageId = pageId;
    pageState.pageFlags |= PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
}

static void showRxPage(void)
{
    for (uint8_t channelIndex = 0; channelIndex < rxRuntimeConfig.channelCount && channelIndex < RX_CHANNELS_PER_PAGE_COUNT; channelIndex += 2) 
    {
        OLED_set_line((channelIndex / 2) + PAGE_TITLE_LINE_COUNT);
        drawRxChannel(channelIndex, HALF_SCREEN_CHARACTER_COLUMN_COUNT);
        if (channelIndex >= rxRuntimeConfig.channelCount) 
        {
            continue;
        }
        if (IS_SCREEN_CHARACTER_COLUMN_COUNT_ODD) 
        {
            LCDprint(' ');
        }
        drawRxChannel(channelIndex + PAGE_TITLE_LINE_COUNT, HALF_SCREEN_CHARACTER_COLUMN_COUNT);
    }
}



static void showWelcomePage(void)
{
    // Draw the logo
    OLED_Render_RLE_Bitmap(u360gts_logo);

    // Draw the rest of the text around it
    uint8_t rowIndex = 0;

    tfp_sprintf(lineBuffer, "v%s (%s)", FC_VERSION_STRING, shortGitRevision);
    OLED_set_line(rowIndex);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "TARGET: %s", targetName);
    OLED_set_line(++rowIndex);
    OLED_send_string(lineBuffer);

    rowIndex += 6;
    OLED_set_line(rowIndex);
    tfp_sprintf(lineBuffer, "   WWW.U360GTS.COM");
    OLED_send_string(lineBuffer);
}

static void showCalibratingMagPage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;
    OLED_set_line(rowIndex++);
    tfp_sprintf(lineBuffer, "Calibrating mag ...");
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);
}

static void showBootModePage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;
    OLED_set_line(rowIndex++);
    tfp_sprintf(lineBuffer, "Boot mode ...");
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);
}

static void showArmedPage(void)
{
}

void showMainMenuPage()
{
    const char *const(*currentMenu);
    uint8_t menuItems;
    uint8_t maxItems = maxMenuOptions;
    uint8_t menuTitleIndex;

    switch (menuState)
    {
        case MENU_ROOT:
            currentMenu = rootMenu;
            menuItems = OP_EXIT + 1;
            break;
        case MENU_TELEMETRY:
            currentMenu = telemetryMenu;
            menuItems = OP_TELMETRY_EXIT + 1;
            menuTitleIndex = MENU_TELEMETRY - 1;
            break;
        case MENU_TELEMETRY_PROTOCOL:
            currentMenu = telemetryProtocolMenu;
            menuItems = OP_TELEMETRY_PROTOCOL_EXIT + 1;
            menuTitleIndex = MENU_TELEMETRY - 1;
            break;
        case MENU_TELEMETRY_BAUDRATE:
            currentMenu = telemetryBaudrateMenu;
            menuItems = OP_TELEMETRY_BAUDRATE_EXIT + 1;
            menuTitleIndex = MENU_TELEMETRY - 1;
            break;
        case MENU_BATTERY:
            currentMenu = enableDisableMenu;
            menuItems = OP_ENABLEDISABLE_EXIT + 1;
            menuTitleIndex = MENU_BATTERY - 1;
            break;
        case MENU_GPS:
            currentMenu = enableDisableMenu;
            menuItems = OP_ENABLEDISABLE_EXIT + 1;
            menuTitleIndex = MENU_GPS - 1;
            break;
        case MENU_EPS:
            currentMenu = epsMenu;
            menuItems = OP_EPS_EXIT + 1;
            menuTitleIndex = MENU_EPS - 1;
            break;
        case MENU_EPS_MODE:
            currentMenu = epsModeMenu;
            menuItems = OP_EPS_MODE_EXIT + 1;
            menuTitleIndex = MENU_EPS - 1;
            break;
        case MENU_EPS_DISTANCEGAIN:
            currentMenu = epsParamIncreaseDecrease;
            menuItems = OP_INCREASEDECREASE_EXIT + 1;
            menuTitleIndex = MENU_EPS - 1;
            break;
        case MENU_EPS_FREQUENCY:
            currentMenu = epsParamIncreaseDecrease;
            menuItems = OP_INCREASEDECREASE_EXIT + 1;
            menuTitleIndex = MENU_EPS - 1;
            break;
        case MENU_EASING:
            currentMenu = enableDisableMenu;
            menuItems = OP_ENABLEDISABLE_EXIT + 1;
            menuTitleIndex = MENU_EASING - 1;
            break;
        default:
            return;
    }

    // Show Title
    OLED_set_line(0);
    if (menuState > 1)
        tfp_sprintf(lineBuffer, "SETUP > %s", rootMenu[menuTitleIndex - 1]);
    else
        tfp_sprintf(lineBuffer, "SETUP MAIN      ");
    OLED_send_string(lineBuffer);

    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    OLED_set_line(rowIndex++);
    tfp_sprintf(lineBuffer, "-------------------");     // 19 chars
    OLED_send_string(lineBuffer);

    OLED_set_line(rowIndex++);

    if (menuItems < maxMenuOptions)
        maxItems = menuItems;

    for (uint8_t i = 0; i < maxItems; i++)
    {
        OLED_set_line(rowIndex++);
        if (i == 0)
        {
            tfp_sprintf(lineBuffer, "> %s <", currentMenu[(i + indexMenuOption) % (menuItems)]);
            // OLED_send_stringH(lineBuffer,true);
        }
        else
        {
            tfp_sprintf(lineBuffer, "  %s  ", currentMenu[(i + indexMenuOption) % (menuItems)]);
            // OLED_send_string(lineBuffer);
        }
        OLED_send_string(lineBuffer);
    }

    if (currentMenu == epsParamIncreaseDecrease)
    {
        OLED_set_line(rowIndex++);
        if (menuState == MENU_EPS_DISTANCEGAIN)
            tfp_sprintf(lineBuffer, "Dist. gain: %d", EPS_DISTANCE_GAIN);
        else
            tfp_sprintf(lineBuffer, "Frequency: %d", EPS_FREQUENCY);
        OLED_send_string(lineBuffer);
    }
}

static void showCliModePage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    OLED_set_line(rowIndex++);
    tfp_sprintf(lineBuffer, "H: %03d A:%03d  ", trackerPosition.heading / 10, targetPosition.heading / 10);
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "T: %03d  ", SERVOTEST_TILT);
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    OLED_set_line(rowIndex++);

    tfp_sprintf(lineBuffer, "PWM PAN:  %04d uS", pwmPan);
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "PWM TILT: %04d uS", pwmTilt);
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);
}

static void showTelemetryPage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;
    // OLED_set_line(rowIndex++);
    if (!PROTOCOL(TP_MFD))
    {
        if (telemetry_sats > 99)
            telemetry_sats = 99;

        tfp_sprintf(lineBuffer, "Hz : %d  V:%d.%d", telemetry_frequency, telemetry_voltage / 100, telemetry_voltage % 100);
        // tfp_sprintf(lineBuffer, "Hz : %d",telemetry_frequency);
        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);

        tfp_sprintf(lineBuffer, "Sat: %02d %s FCS:%d", telemetry_sats, telemetryFixType[telemetry_fixtype], telemetry_failed_cs);
        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);

        uint16_t lat_a = abs(targetPosition.lat / 1000000);
        uint32_t lat_b = abs(targetPosition.lat % 1000000);

        if (targetPosition.lat < 0)
            tfp_sprintf(lineBuffer, "Lat: -%d.%06d  ", lat_a, lat_b);
        else
            tfp_sprintf(lineBuffer, "Lat: %d.%06d  ", lat_a, lat_b);
        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);

        uint16_t lon_a = abs(targetPosition.lon / 1000000);
        uint32_t lon_b = abs(targetPosition.lon % 1000000);

        if (targetPosition.lon < 0)
            tfp_sprintf(lineBuffer, "Lon: -%d.%06d  ", lon_a, lon_b);
        else
            tfp_sprintf(lineBuffer, "Lon: %d.%06d  ", lon_a, lon_b);
        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);
    }
    else
    {
        //
    }
    char altSgn = (targetPosition.alt < 0) ? '-' : ' ';
    int16_t displayedAlt = (targetPosition.alt < 0) ? -1 * targetPosition.alt : targetPosition.alt;
    tfp_sprintf(lineBuffer, "Alt:%c%d Dis: %d  ", altSgn, displayedAlt, targetPosition.distance);
    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);
    if (OFFSET_TRIM_STATE)
        tfp_sprintf(lineBuffer, "H: %03d A:%03d Of: %d  ", trackerPosition.heading / 10, targetPosition.heading / 10, OFFSET_TRIM);
    else
        tfp_sprintf(lineBuffer, "H: %03d A:%03d Of:<%d> ", trackerPosition.heading / 10, targetPosition.heading / 10, OFFSET_TRIM);

    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    if(!PROTOCOL(TP_MFD)) {
    	if (homeSet_BY_GPS && homeSet)
    		tfp_sprintf(lineBuffer, "HOME SET <GPS>");
    	else if (homeSet_BY_LAST && homeSet)
    	            tfp_sprintf(lineBuffer, "HOME SET <LAST>");
    	else if (homeSet)
    		tfp_sprintf(lineBuffer, "HOME SET <AIRCRAFT>");
    	else if (!homeSet || homeReset)
    		tfp_sprintf(lineBuffer, "HOME NOT SET");
		padLineBuffer();
		i2c_OLED_set_line(rowIndex++);
		i2c_OLED_send_string(lineBuffer);
    }

    if (!feature(FEATURE_GPS) && !feature(FEATURE_VBAT) && !feature(FEATURE_RSSI_ADC) && !(rxConfig->rssi_channel > 0))
        displayDisablePageCycling();
}

static void showProfilePage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    tfp_sprintf(lineBuffer, "Profile: %d", getCurrentProfile());
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    uint8_t currentRateProfileIndex = getCurrentControlRateProfile();
    tfp_sprintf(lineBuffer, "Rate profile: %d", currentRateProfileIndex);
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    controlRateConfig_t *controlRateConfig = getControlRateConfig(currentRateProfileIndex);

    tfp_sprintf(lineBuffer, "RCE: %d, RCR: %d",
                controlRateConfig->rcExpo8,
                controlRateConfig->rcRate8);
    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "RR:%d PR:%d YR:%d",
                controlRateConfig->rates[FD_ROLL],
                controlRateConfig->rates[FD_PITCH],
                controlRateConfig->rates[FD_YAW]);
    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);
}

#ifdef GPS

#define SATELLITE_COUNT (sizeof(GPS_svinfo_cno) / sizeof(GPS_svinfo_cno[0]))
#define SATELLITE_GRAPH_LEFT_OFFSET ((SCREEN_CHARACTER_COLUMN_COUNT - SATELLITE_COUNT) / 2)

static void showGpsPage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    static uint8_t gpsTicker = 0;
    static uint32_t lastGPSSvInfoReceivedCount = 0;
    if (GPS_svInfoReceivedCount != lastGPSSvInfoReceivedCount)
    {
        lastGPSSvInfoReceivedCount = GPS_svInfoReceivedCount;
        gpsTicker++;
        gpsTicker = gpsTicker % TICKER_CHARACTER_COUNT;
    }

    OLED_set_xy(0, rowIndex);
    OLED_send_char(tickerCharacters[gpsTicker]);

    OLED_set_xy(MAX(0, SATELLITE_GRAPH_LEFT_OFFSET), rowIndex++);

    uint32_t index;
    for (index = 0; index < SATELLITE_COUNT && index < SCREEN_CHARACTER_COLUMN_COUNT; index++)
    {
        uint8_t bargraphOffset = ((uint16_t)GPS_svinfo_cno[index] * VERTICAL_BARGRAPH_CHARACTER_COUNT) / (GPS_DBHZ_MAX - 1);
        bargraphOffset = MIN(bargraphOffset, VERTICAL_BARGRAPH_CHARACTER_COUNT - 1);
        OLED_send_char(VERTICAL_BARGRAPH_ZERO_CHARACTER + bargraphOffset);
    }

    char fixChar = STATE(GPS_FIX) ? 'Y' : 'N';
    tfp_sprintf(lineBuffer, "Sat: %d Fix: %c", GPS_numSat, fixChar);
    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Pos: %d/%d Hdop:%d", GPS_coord[LAT] / GPS_DEGREES_DIVIDER, GPS_coord[LON] / GPS_DEGREES_DIVIDER, GPS_hdop);
    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Spd: %d", GPS_speed);
    padHalfLineBuffer();
    OLED_set_line(rowIndex);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "GC: %d", GPS_ground_course);
    padHalfLineBuffer();
    OLED_set_xy(HALF_SCREEN_CHARACTER_COLUMN_COUNT, rowIndex++);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "RX: %d", GPS_packetCount);
    padHalfLineBuffer();
    OLED_set_line(rowIndex);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "ERRs: %d", gpsData.errors, gpsData.timeouts);
    padHalfLineBuffer();
    OLED_set_xy(HALF_SCREEN_CHARACTER_COLUMN_COUNT, rowIndex++);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Dt: %d", gpsData.lastMessage - gpsData.lastLastMessage);
    padHalfLineBuffer();
    OLED_set_line(rowIndex);
    OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "TOs: %d", gpsData.timeouts);
    padHalfLineBuffer();
    OLED_set_xy(HALF_SCREEN_CHARACTER_COLUMN_COUNT, rowIndex++);
    OLED_send_string(lineBuffer);

    strncpy(lineBuffer, gpsPacketLog, GPS_PACKET_LOG_ENTRY_COUNT);
    padHalfLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

#ifdef GPS_PH_DEBUG
    tfp_sprintf(lineBuffer, "Angles: P:%d R:%d", GPS_angle[PITCH], GPS_angle[ROLL]);
    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);
#endif

#if 0
    tfp_sprintf(lineBuffer, "%d %d %d %d", debug[0], debug[1], debug[2], debug[3]);
    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);
#endif
}
#endif

static void showBatteryPage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    if (feature(FEATURE_VBAT))
    {
        tfp_sprintf(lineBuffer, "Volts: %d.%1d Cells: %d", vbat / 10, vbat % 10, batteryCellCount);
        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);

        uint16_t averageCellVoltage = ((float)vbat / batteryCellCount) * 10;
        tfp_sprintf(lineBuffer, "Avg. Cell: %d.%02dv", averageCellVoltage / 100, averageCellVoltage % 100);
        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);

        uint8_t batteryPercentage = calculateBatteryPercentage();
        OLED_set_line(rowIndex++);
        drawHorizonalPercentageBar(SCREEN_CHARACTER_COLUMN_COUNT, batteryPercentage);
    }

    if (feature(FEATURE_CURRENT_METER))
    {
        tfp_sprintf(lineBuffer, "Amps: %d.%2d mAh: %d", amperage / 100, amperage % 100, mAhDrawn);
        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);

        uint8_t capacityPercentage = calculateBatteryCapacityRemainingPercentage();
        OLED_set_line(rowIndex++);
        drawHorizonalPercentageBar(SCREEN_CHARACTER_COLUMN_COUNT, capacityPercentage);
    }

    if (feature(FEATURE_RSSI_ADC) || (rxConfig->rssi_channel > 0))
    {
        if (feature(FEATURE_VBAT))
            OLED_set_line(rowIndex++);

        tfp_sprintf(lineBuffer, "RSSI");
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);

        uint8_t rssiPercentage1 = calculateRssiPercentage();
        uint8_t rssiPercentage2 = ((uint32_t)rssiPercentage1 * 100) / rxConfig->rssi_zoom;
        /*uint8_t rssiA = calculateRssiVoltage() / 10;
        uint8_t rssiB = calculateRssiVoltage() % 10;

        tfp_sprintf(lineBuffer, "Value: %drssi_zoom.%d v %d %%", rssiA, rssiB, rssiPercentage1);*/
        tfp_sprintf(lineBuffer, "Value: %d %%", rssiPercentage1);

        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);

        OLED_set_line(rowIndex++);
        drawHorizonalPercentageBar(SCREEN_CHARACTER_COLUMN_COUNT, rssiPercentage1);

        if (rssiPercentage1 > rxConfig->rssi_zoom)
            rssiPercentage2 = 100;

        OLED_set_line(rowIndex++);
        drawHorizonalPercentageBar(SCREEN_CHARACTER_COLUMN_COUNT, rssiPercentage2);
    }
}

static void showSensorsPage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;
    static const char *format = "%s %5d %5d %5d";

    OLED_set_line(rowIndex++);
    OLED_send_string("        X     Y     Z");

    if (sensors(SENSOR_ACC))
    {
        tfp_sprintf(lineBuffer, format, "ACC", accSmooth[X], accSmooth[Y], accSmooth[Z]);
        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);
    }

    if (sensors(SENSOR_GYRO))
    {
        tfp_sprintf(lineBuffer, format, "GYR", gyroADC[X], gyroADC[Y], gyroADC[Z]);
        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);
    }

#ifdef MAG
    if (sensors(SENSOR_MAG))
    {
        tfp_sprintf(lineBuffer, format, "MAG", magADC[X], magADC[Y], magADC[Z]);
        padLineBuffer();
        OLED_set_line(rowIndex++);
        OLED_send_string(lineBuffer);
    }
#endif

    tfp_sprintf(lineBuffer, format, "I&H", inclination.values.rollDeciDegrees, inclination.values.pitchDeciDegrees, heading);
    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    uint8_t length;

    ftoa(EstG.A[X], lineBuffer);
    length = strlen(lineBuffer);
    while (length < HALF_SCREEN_CHARACTER_COLUMN_COUNT)
    {
        lineBuffer[length++] = ' ';
        lineBuffer[length + 1] = 0;
    }
    ftoa(EstG.A[Y], lineBuffer + length);
    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);

    ftoa(EstG.A[Z], lineBuffer);
    length = strlen(lineBuffer);
    while (length < HALF_SCREEN_CHARACTER_COLUMN_COUNT)
    {
        lineBuffer[length++] = ' ';
        lineBuffer[length + 1] = 0;
    }
    ftoa(smallAngle, lineBuffer + length);
    padLineBuffer();
    OLED_set_line(rowIndex++);
    OLED_send_string(lineBuffer);
}

#ifdef ENABLE_DEBUG_OLED_PAGE
void showDebugPage(void)
{
    uint8_t rowIndex;
    for (rowIndex = 0; rowIndex < 4; rowIndex++) {
        tfp_sprintf(lineBuffer, "%d = %5d", rowIndex, debug[rowIndex]);
        padLineBuffer();
        OLED_set_line(rowIndex + PAGE_TITLE_LINE_COUNT);
        OLED_send_string(lineBuffer);
    }
}
#endif

void showAutodetectingTitle(uint16_t protocol)
{
    master_telemetry_protocol = protocol;
    showTitle();
}

void updateDisplayProtocolTitle(uint16_t protocol)
{
    master_telemetry_protocol = protocol;
    telemetry_failed_cs = 0;
    handlePageChange();
}

void updateDisplay(void)
{
    uint32_t now = micros();
    static uint8_t previousArmedState = 0;

    bool updateNow = (int32_t)(now - nextDisplayUpdateAt) >= 0L;
    if (!updateNow)
    {
        return;
    }

    nextDisplayUpdateAt = now + DISPLAY_UPDATE_PERIOD_US;

    /*//////
    pageState.pageId=PAGE_TELEMETRY;
    showTitle();
    showTelemetryPage();
    updateTicker();
    return;
    //////*/

    bool armedState = ARMING_FLAG(ARMED) ? true : false;
    bool armedStateChanged = armedState != previousArmedState;
    previousArmedState = armedState;
    if (armedState)
    {
        if (!armedStateChanged)
        {
            return;
        }
        pageState.pageIdBeforeArming = pageState.pageId;
        pageState.pageId = PAGE_ARMED;
        pageState.pageChanging = true;
    }
    else
    {
        if (armedStateChanged)
        {
            pageState.pageFlags |= PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
            pageState.pageId = pageState.pageIdBeforeArming;
        }
        pageState.pageChanging = (pageState.pageFlags & PAGE_STATE_FLAG_FORCE_PAGE_CHANGE) ||
                                 (((int32_t)(now - pageState.nextPageAt) >= 0L && (pageState.pageFlags & PAGE_STATE_FLAG_CYCLE_ENABLED)));
        if (pageState.pageChanging && (pageState.pageFlags & PAGE_STATE_FLAG_CYCLE_ENABLED))
        {
            pageState.cycleIndex++;
            pageState.cycleIndex = pageState.cycleIndex % CYCLE_PAGE_ID_COUNT;
            pageState.pageId = cyclePageIds[pageState.cycleIndex];
        }
    }

    if (pageState.pageChanging)
    {
        pageState.pageFlags &= ~PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
        pageState.nextPageAt = now + PAGE_CYCLE_PERIOD_US;
        pageState.nextToggleAt = now + PAGE_TOGGLE_PERIOD_US;
        pageState.toggleCounter = 0;

        // Some OLED displays do not respond on the first initialisation so refresh the display
        // when the page changes in the hopes the hardware responds.  This also allows the
        // user to power off/on the display or connect it while powered.
        resetDisplay();

        if (!displayPresent)
        {
            return;
        }
        handlePageChange();
    }

    if (!displayPresent)
    {
        return;
    }

    if ((int32_t)(now - pageState.nextToggleAt) >= 0L)
    {
        pageState.toggleCounter++;
        pageState.nextToggleAt += PAGE_TOGGLE_PERIOD_US;
    }

    switch (pageState.pageId)
    {
        case PAGE_WELCOME:
            showWelcomePage();
            break;
        case PAGE_BATTERY:
            if (feature(FEATURE_VBAT) || feature(FEATURE_RSSI_ADC) || (rxConfig->rssi_channel > 0))
            {
                showBatteryPage();
            }
            else
            {
                pageState.pageFlags |= PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
            }
            break;
        case PAGE_TELEMETRY:
            // showProfilePage();
            showTelemetryPage();
            break;
        #ifdef GPS
        case PAGE_GPS:
            if (feature(FEATURE_GPS) && !PROTOCOL(TP_MFD))
            {
                showGpsPage();
            }
            else
            {
                pageState.pageFlags |= PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
            }
            break;
        #endif
        #ifdef ENABLE_DEBUG_OLED_PAGE
        case PAGE_DEBUG:
            showDebugPage();
            break;
        #endif
        case PAGE_CALIBRATING_MAG:
            if (PROTOCOL(TP_CALIBRATING_MAG))
            {
                showCalibratingMagPage();
            }
            else
            {
                pageState.pageFlags |= PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
            }
            break;
        case PAGE_CLI_MODE:
            showCliModePage();
            break;
        case PAGE_MENU:
            showMainMenuPage();
            break;
        case PAGE_BOOT_MODE:
            showBootModePage();
            break;
        default:
            break;
    }
    /*if (!armedState) {
        updateFailsafeStatus();
        updateRxStatus();
        updateTicker();
    }*/
    if (pageState.pageId != PAGE_WELCOME)
    {
        updateTicker();
    }

    OLED_flush();
}

void displayInit(rxConfig_t *rxConfigToUse,uint16_t telemetry_protocol, uint8_t oled_type)
{
    delay(200);
    resetDisplay();
    delay(200);

    rxConfig = rxConfigToUse;
    display_type = oled_type;
    master_telemetry_protocol = telemetry_protocol;

    memset(&pageState, 0, sizeof(pageState));
    displaySetPage(PAGE_WELCOME);
    updateDisplay();

    displaySetNextPageChangeAt(micros() + (1000 * 1000 * 5));
}

void displayShowFixedPage(pageId_e pageId)
{
    displaySetPage(pageId);
    displayDisablePageCycling();
    if(pageId==PAGE_BOOT_MODE){
    	updateDisplay();
    }
}

void displaySetNextPageChangeAt(uint32_t futureMicros)
{
    pageState.nextPageAt = futureMicros;
}

void displayEnablePageCycling(void)
{
    pageState.pageFlags |= PAGE_STATE_FLAG_CYCLE_ENABLED;
}

void displayResetPageCycling(void)
{
    pageState.cycleIndex = CYCLE_PAGE_ID_COUNT - 1; // start at first page
}

void displayDisablePageCycling(void)
{
    pageState.pageFlags &= ~PAGE_STATE_FLAG_CYCLE_ENABLED;
}

#endif
