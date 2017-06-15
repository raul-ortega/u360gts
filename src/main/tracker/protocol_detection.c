/*
 * This file is part of u360gts, aka amv-open360tracker 32bits:
 * https://github.com/raul-ortega/amv-open360tracker-32bits
 *
 * The code below is an adaptation by Ra�l Ortega of the original code of Ghettostation antenna tracker
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
    DETECTION_STATE_IDLE,
    DETECTION_STATE_START,
	DETECTION_STATE_START_FRXKY,
	DETECTION_STATE_START_MAVLINK,
	DETECTION_STATE_START_MFD,
    DETECTION_STATE_DETECTED
  };

static uint8_t detectionState = DETECTION_STATE_IDLE;
static uint8_t detectionPacketIdex=0;
static uint16_t protocolDetected = 0;

bool detectionIsEnabled = false;

void enableProtocolDetection(void){
	detectionIsEnabled = true;
}

void disableProtocolDetection(void){
	detectionIsEnabled = false;
}

bool isProtocolDetectionEnabled(void){
	return detectionIsEnabled;
}

uint16_t getProtocol(void){
	return protocolDetected;
}

void protocolDetectionParser(uint8_t c){
	if(!detectionIsEnabled)
		return;

	switch(detectionState){
		case DETECTION_STATE_IDLE:
			if (c =='#' || c == 'X') {
				detectionState = DETECTION_STATE_START_MFD;
				detectionPacketIdex = 0;
			} else if (c == 0x7E)
				detectionState = DETECTION_STATE_START_FRXKY;
			else if (c == 254 && detectionPacketIdex > 10) {
				/*detectionState = DETECTION_STATE_START_MAVLINK;*/
				protocolDetected = TP_MAVLINK;
				detectionState = DETECTION_STATE_DETECTED;
			} else if (c == '$'){
				detectionState = DETECTION_STATE_START;
				detectionPacketIdex = 0;
			}
			detectionPacketIdex ++;
			break;
		case DETECTION_STATE_START_MFD:
			if ((c == '#' || c == 'X'))
				detectionPacketIdex++;
			else if (detectionPacketIdex > 5){
				protocolDetected = TP_MFD;
				detectionState = DETECTION_STATE_DETECTED;
			} else
				detectionState = DETECTION_STATE_IDLE;
			break;
		case DETECTION_STATE_START_FRXKY:
			if (c == 0xFD) {
				protocolDetected = TP_FRSKY_D;
				detectionState = DETECTION_STATE_DETECTED;
			} else if (detectionState == DETECTION_STATE_START_FRXKY && c ==0x10){
				protocolDetected = TP_FRSKY_X;
				detectionState = DETECTION_STATE_DETECTED;
				detectionPacketIdex = 0;
			} else if (detectionPacketIdex > 10)
				detectionState = DETECTION_STATE_IDLE;
			break;
		case DETECTION_STATE_START_MAVLINK:
			if (detectionPacketIdex < 5)
				detectionPacketIdex++;
			else if (c == 24 || c == 33){
				protocolDetected = TP_MAVLINK;
				detectionState = DETECTION_STATE_DETECTED;
			} else
				detectionState = DETECTION_STATE_IDLE;
			break;
		case DETECTION_STATE_START:
			detectionPacketIdex++;
			if(c == '$' && detectionPacketIdex == 10 ){
				protocolDetected = TP_PITLAB;
				detectionState = DETECTION_STATE_DETECTED;
				break;
			} else  {
				switch(c){
					case 'T':
						protocolDetected = TP_LTM;
						break;
					case 'G':
						protocolDetected = TP_GPS_TELEMETRY;
						break;
					case '1':
					case 'R':
					case 'V':
						protocolDetected = TP_RVOSD;
						break;
					default:
						detectionState = DETECTION_STATE_IDLE;
						break;
				}
			}
			break;
		case DETECTION_STATE_DETECTED:
			detectionState = DETECTION_STATE_IDLE;
			detectionPacketIdex = 0;
			disableProtocolDetection();
			break;
	}
}
