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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "config/runtime_config.h"

static uint16_t protocolDetected = 0;
uint8_t current_protocol = 0;
uint32_t protocolDetectionTimer = 0;
bool detectionIsEnabled = false;
uint8_t protocol_fixes = 0;

extern bool gotFix;

static const uint16_t protocols[] = {
    TP_MAVLINK,
    TP_FRSKY_X,
    TP_CROSSFIRE,
    TP_MFD,
    TP_LTM,
    TP_GPS_TELEMETRY,
    TP_FRSKY_D,
    TP_PITLAB,
    TP_RVOSD
};

void enableProtocolDetection(uint16_t default_protocol) {
	protocolDetected = 0;
	detectionIsEnabled = true;
	protocol_fixes = 0;
	current_protocol=0;

	//set default protocol
	for(uint8_t i=0; i < sizeof(protocols) / sizeof(uint8_t); i++){
	    if(protocols[i] == default_protocol){
	        current_protocol = i;
	        break;
	    }
	}

	protocolDetectionTimer = 0;
}

void disableProtocolDetection(void){
	detectionIsEnabled = false;
	protocolDetected = 0;
}

bool isProtocolDetectionEnabled(void){
	return detectionIsEnabled;
}

uint16_t getProtocol(void){
	return protocolDetected;
}

void protocolDetectionParser(uint8_t c)
{

    if(protocolDetectionTimer == 0)
        protocolDetectionTimer = millis();

    if(millis() - protocolDetectionTimer >= 3000){
      current_protocol ++;
      if(current_protocol >= sizeof(protocols) / sizeof(uint16_t)) current_protocol = 0;
      protocolDetectionTimer = millis();
      gotFix = false;
    }

    switch(protocols[current_protocol]){
        case TP_MFD:
            mfd_encodeTargetData(c);
            break;
        case TP_GPS_TELEMETRY:
            gps_encodeTargetData(c);
            break;
        case TP_MAVLINK:
            mavlink_encodeTargetData(c);
            break;
        case TP_RVOSD:
            rvosd_encodeTargetData(c);
            break;
        case TP_FRSKY_D:
            frskyd_encodeTargetData(c);
            break;
        case TP_FRSKY_X:
            frskyx_encodeTargetData(c);
            break;
        case TP_LTM:
            ltm_encodeTargetData(c);
            break;
        case TP_PITLAB:
            pitlab_encodeTargetData(c);
            break;
        case TP_CROSSFIRE:
            crossfire_encodeTargetData(c);
            break;
    }

    if(gotFix){
        protocol_fixes ++;
        if(protocol_fixes > 1) {
            protocolDetected = protocols[current_protocol];
            return;
        }
        gotFix = false;

    }
}
