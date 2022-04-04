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

// Display size is provided by low-level driver
#include "drivers/display_ug2864hsweg01.h"

#define FONT_WIDTH              5
#define FONT_HEIGHT             7
#define HORIZONTAL_PADDING      1
#define VERTICAL_PADDING        1

#define CHARACTER_WIDTH_TOTAL (FONT_WIDTH + HORIZONTAL_PADDING)
#define CHARACTER_HEIGHT_TOTAL (FONT_HEIGHT + VERTICAL_PADDING)

#define SCREEN_CHARACTER_COLUMN_COUNT (OLED_WIDTH / CHARACTER_WIDTH_TOTAL)
#define SCREEN_CHARACTER_ROW_COUNT (OLED_HEIGHT / CHARACTER_HEIGHT_TOTAL)

#define VERTICAL_BARGRAPH_ZERO_CHARACTER (128 + 32)
#define VERTICAL_BARGRAPH_CHARACTER_COUNT 7

bool OLED_init(uint8_t oled_type);
void OLED_flush(void);
void OLED_clear_display(void);
void OLED_set_xy(uint8_t column, uint8_t row);
void OLED_set_line(uint8_t row);
void OLED_send_charH(unsigned char ascii, bool hightlight);
void OLED_send_char(unsigned char ascii);
void OLED_send_stringH(const char *string, bool hightlight);
void OLED_send_string(const char *string);
void OLED_Render_RLE_Bitmap(uint8_t *bitmap);
