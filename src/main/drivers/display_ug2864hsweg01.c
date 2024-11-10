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
#include <stdlib.h>
#include <memory.h>

#include "platform.h"

#include "bus_i2c.h"
#include "system.h"

#include "display_ug2864hsweg01.h"

#define OLED_SSD_1306_address   0x3C     // OLED at address 0x3C in 7bit
#define OLED_SH_1106_address    0x3C     // OLED at address 0x3C in 7bit

#define OLED_SSD_1306_data      0x40
#define OLED_SH_1106_data       0x40

#define OLED_SSD_1306_command   0x80
#define OLED_SH_1106_command    0x00

#define OLED_SSD_1306_SETLOWCOLUMN 0x00
#define OLED_SH_1106_SETLOWCOLUMN 0x02

#define OLED_SSD_1306_SETHIGHCOLUMN 0x10
#define OLED_SH_1106_SETHIGHCOLUMN 0x10

static uint8_t OLED_address = OLED_SSD_1306_address;
static uint8_t OLED_data = OLED_SSD_1306_data;
static uint8_t OLED_command = OLED_SSD_1306_command;
static uint8_t OLED_setlowcolumn = OLED_SSD_1306_SETLOWCOLUMN;
static uint8_t OLED_sethighcolumn = OLED_SSD_1306_SETHIGHCOLUMN;

/**
 * according to http://www.adafruit.com/datasheets/UG-2864HSWEG01.pdf Chapter 4.4 Page 15
 */

static const uint8_t SSD_1306_init[] = {
    //0xAE, // Display off
    0xD4, // Set Display Clock Divide Ratio / OSC Frequency
    0x80, // Display Clock Divide Ratio / OSC Frequency
    0xA8, // Set Multiplex Ratio
    0x3F, // Multiplex Ratio for 128x64 (64-1)
    0xD3, // Set Display Offset
    0x00, // Display Offset
    0x40, // Set Display Start Line
    0x8D, // Set Charge Pump
    0x14, // Charge Pump (0x10 External, 0x14 Internal DC/DC)
    0xA1, // Set Segment Re-Map
    0xC8, // Set Com Output Scan Direction
    0xDA, // Set COM Hardware Configuration
    0x12, // COM Hardware Configuration
    0x81, // Set Contrast
    0xCF, // Contrast
    0xD9, // Set Pre-Charge Period
    0xF1, // Set Pre-Charge Period (0x22 External, 0xF1 Internal)
    0xDB, // Set VCOMH Deselect Level
    0x40, // VCOMH Deselect Level
    0xA4, // Set all pixels OFF
    0xA6, // Set display not inverted
    0xAF  // Set display On
};

static const uint8_t SH_1106_init[] = {
    //0xAE, // Display off
    0xD5, // Set Display Clock Divide Ratio / OSC Frequency
    0x80, // Display Clock Divide Ratio / OSC Frequency
    0xA8, // Set Multiplex Ratio
    0x3F, // Multiplex Ratio for 128x64 (64-1)
    0xD3, // Set Display Offset
    0x00, // Display Offset
    0x00, // Set Display Start Line
    0x8D, // Set Charge Pump
    0x14, // Charge Pump (0x10 External, 0x14 Internal DC/DC)
    0x20,                    // 0x20
    0x00,
    0xA1, // Set Segment Re-Map
    0xC8, // Set Com Output Scan Direction
    0xDA, // Set COM Hardware Configuration
    0x12, // COM Hardware Configuration
    0x81, // Set Contrast
    0xCF, // Contrast
    0xD9, // Set Pre-Charge Period
    0xF1, // Set Pre-Charge Period (0x22 External, 0xF1 Internal)
    0xDB, // Set VCOMH Deselect Level
    0x40, // VCOMH Deselect Level
    0xA4, // Set all pixels OFF
    0xA6, // Set display not inverted
    0xAF  // Set display On
};


static bool ug2864hsweg01_SendCmd(uint8_t command)
{
    return i2cWrite(OLED_address, OLED_command, command);
}

//static bool ug2864hsweg01_SendData(uint8_t val)
//{
//   return i2cWrite(OLED_address, OLED_data, val);
//}

static bool ug2864hsweg01_SendInitSequence(const uint8_t *list_ptr, uint16_t size)
{
    if (!ug2864hsweg01_SendCmd(0xAE))
    {
        return false;
    }

    for (; size > 0; size--, list_ptr++)
    {
        ug2864hsweg01_SendCmd(*list_ptr);
    }

    return true;
}

// Output buffer to OLED
// Procedure takes ~30ms @400kHz I2C
void ug2864hsweg01_Display(const uint8_t *buffer)
{    
    #define OLED_I2C_BURST_SIZE   16
    for (int8_t i = 0; i < (OLED_HEIGHT / 8); i++)
    {
        ug2864hsweg01_SendCmd(0xB0 + i);           // Set row
        ug2864hsweg01_SendCmd(OLED_setlowcolumn);  // Set lower column address
        ug2864hsweg01_SendCmd(OLED_sethighcolumn); // Set higher column address
        for (uint16_t j = 0; j < (OLED_WIDTH / OLED_I2C_BURST_SIZE); j++)
        {    
            // send a bunch of data in one xmission
            i2cWriteBuffer(OLED_address, 0x40, OLED_I2C_BURST_SIZE, buffer);
            buffer += OLED_I2C_BURST_SIZE;
        }
    }
}


bool ug2864hsweg01_Init(uint8_t oled_type)
{
    if (oled_type == 0) 
    {
        return ug2864hsweg01_SendInitSequence(SSD_1306_init, sizeof(SSD_1306_init));
    }

    OLED_address = OLED_SH_1106_address;
    OLED_data = OLED_SH_1106_data;
    OLED_command = OLED_SH_1106_command;
    OLED_setlowcolumn = OLED_SH_1106_SETLOWCOLUMN;
    OLED_sethighcolumn = OLED_SH_1106_SETHIGHCOLUMN;
    return ug2864hsweg01_SendInitSequence(SH_1106_init, sizeof(SH_1106_init));
}

