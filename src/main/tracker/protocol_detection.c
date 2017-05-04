/*
 * This file is part of u360gts, aka amv-open360tracker 32bits:
 * https://github.com/raul-ortega/amv-open360tracker-32bits
 *
 * The code below is an adaptation by Raúl Ortega of the original code of Ghettostation antenna tracker
 * https://github.com/KipK/Ghettostation
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

/*

*/
#include "config.h"
#include <stdint.h>
#include <stdbool.h>
#include "config/runtime_config.h"

// machine states
enum protocolDetectionStates {
    IDLE,
    STATE_START,
	STATE_START_FRXKY_D,
	STATE_START_FRXKY_X,
	STATE_START_MAVLINK,
    STATE_DETECTED
  };

static uint8_t parserState = IDLE;
static uint16_t protocolDetected = 0;
static uint8_t packetIndex=0;

uint16_t protocolDetectionParser(uint8_t c) {
		if (parserState == IDLE)
			protocolDetected=0;

		if (parserState == IDLE && c == 0x7E){
			parserState = STATE_START_FRXKY_D;
		} else if (parserState == STATE_START_FRXKY_D && c == 0xFD) {
				protocolDetected = TP_FRSKY_D;
				parserState = STATE_DETECTED;
		} else if (parserState == STATE_START_FRXKY_D && c ==0x10){
			protocolDetected = TP_FRSKY_X;
			parserState = STATE_DETECTED;
		} else if (parserState == IDLE && c == 254){
			parserState = STATE_START_MAVLINK;
			packetIndex = 0;
		} else if (parserState == STATE_START_MAVLINK && packetIndex >= 0 && packetIndex < 5){
			parserState = STATE_START_MAVLINK;
			packetIndex++;
		} else if (parserState == STATE_START_MAVLINK && packetIndex == 5 && (c == 24 || c == 33)){
			protocolDetected = TP_MAVLINK;
			parserState = STATE_DETECTED;
		} else if (parserState == IDLE && (c == '$')) {
	    	parserState = STATE_START;
	    } else if (parserState == STATE_START) {
	    	switch(c){
				case 'T': //LTM $T
					protocolDetected = TP_LTM;
					parserState = STATE_DETECTED;
					break;
				case 'G': //NMEA $GGA
					protocolDetected = TP_GPS_TELEMETRY;
					parserState = STATE_DETECTED;
					break;
				case '1': //RVOSD $1
				case 'R':
				case 'V':
					protocolDetected = TP_RVOSD;
					parserState = STATE_DETECTED;
					break;
				default:
					parserState = IDLE;
	    	}
	    } else if (parserState == STATE_DETECTED && protocolDetected > 0){
	    	DISABLE_PROTOCOL(0b111111111111);
	    	ENABLE_PROTOCOL(protocolDetected);
	    	parserState = IDLE;
	    } else
	    	parserState = IDLE;

	    return (protocolDetected);
}





