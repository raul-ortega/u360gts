/*
 * This file is part of u360gts, aka amv-open360tracker 32bits:
 * https://github.com/raul-ortega/amv-open360tracker-32bits
 *
 * The code below is an adaptation by Raúl Ortega of the original code written by Samuel Brucksch:
 * https://github.com/SamuelBrucksch/open360tracker
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

#include "config.h"
#include "telemetry.h"

void processFrskyPacket(uint8_t *packet);
void parseTelemHubByte(uint8_t c);
int32_t gpsToLong(int8_t neg, uint16_t bp, uint16_t ap);

// Enumerate FrSky packet codes
#define LINKPKT         0xfe
#define USRPKT          0xfd
#define A11PKT          0xfc
#define A12PKT          0xfb
#define A21PKT          0xfa
#define A22PKT          0xf9
#define RSSI1PKT        0xf7
#define RSSI2PKT        0xf6

#define START_STOP      0x7E
#define BYTESTUFF       0x7d
#define STUFF_MASK      0x20

#define GPS_ALT_BP_ID     0x01
#define GPS_ALT_AP_ID     0x09
#define BARO_ALT_BP_ID    0x10
#define BARO_ALT_AP_ID    0x21
#define GPS_LON_BP_ID     0x12
#define GPS_LAT_BP_ID     0x13
#define GPS_LON_AP_ID     0x1A
#define GPS_LAT_AP_ID     0x1B
#define GPS_LON_EW_ID     0x22
#define GPS_LAT_NS_ID     0x23
#define TEMP2             0x05

#define FRSKY_RX_PACKET_SIZE 11
uint8_t frskyRxBuffer[FRSKY_RX_PACKET_SIZE];

#define FRSKYD_LATLON_DIVIDER 100000

enum FrSkyDataState {
  STATE_DATA_IDLE, STATE_DATA_START, STATE_DATA_IN_FRAME, STATE_DATA_XOR,
};

static uint8_t dataState = STATE_DATA_IDLE;
static uint8_t numPktBytes = 0;

int16_t alt;
uint16_t NS;
uint16_t EW;
uint8_t sats;
uint8_t fix;

uint16_t lat_bp;
uint16_t lon_bp;
uint16_t lat_ap;
uint16_t lon_ap;

int32_t prevTargetLat = 0;
int32_t prevTargetLon = 0;
uint16_t prevLatBp=0;
uint16_t LatBp=0;
uint16_t prevLonBp=0;
uint16_t LonBp=0;
int32_t prevAlt = 0;

uint8_t numBytes;


void frskyd_setLat(void) {
  int32_t value = gpsToLong(NS == 'N' ? 1 : -1, lat_bp, lat_ap);
  if(abs(value)>180*FRSKYD_LATLON_DIVIDER) {
    value=prevTargetLat;
  }
  else {
    prevLatBp = (int16_t)(prevTargetLat/FRSKYD_LATLON_DIVIDER);
    LatBp = (int16_t)(value/FRSKYD_LATLON_DIVIDER);
    if(prevTargetLat > value && (prevLatBp-LatBp)>2 && prevTargetLat>FRSKYD_LATLON_DIVIDER) value=prevTargetLat;
    if(prevTargetLat < value && (LatBp-prevLatBp)>2 && prevTargetLat>FRSKYD_LATLON_DIVIDER) value=prevTargetLat;
    prevTargetLat=value;
  }

  NS = 0;
  telemetry_lat = value*10;
}

void frskyd_setLon(void) {

  int32_t value = gpsToLong(EW == 'E' ? 1 : -1, lon_bp, lon_ap);
  if(abs(value)>180*FRSKYD_LATLON_DIVIDER) {
      value=prevTargetLon;
    }
  else {
    prevLonBp = (int16_t)(prevTargetLon/FRSKYD_LATLON_DIVIDER);
    LonBp=(int16_t)(value/FRSKYD_LATLON_DIVIDER);
    if(prevTargetLon > value && (prevLonBp-LonBp)>2 && prevTargetLon>FRSKYD_LATLON_DIVIDER) value=prevTargetLon;
    if(prevTargetLon < value && (LonBp-prevLonBp)>2 && prevTargetLon>FRSKYD_LATLON_DIVIDER) value=prevTargetLon;
    prevTargetLon=value;
  }
  EW = 0;
  telemetry_lon=value*10;
}

void frskyd_setAlt() {
  if(((prevAlt-alt)>500 || (alt-prevAlt)>500)  && prevAlt > 0 && alt > 0 ) alt=prevAlt;
  prevAlt=alt;
  telemetry_alt = alt;
}

void frskyd_setSats() {
  telemetry_sats = sats;
}

void frskyd_encodeTargetData(uint8_t c) {
  switch (dataState) {
  case STATE_DATA_START:
    if (c == START_STOP)
      break; // Remain in userDataStart if possible 0x7e,0x7e doublet found.

    frskyRxBuffer[numPktBytes++] = c;
    dataState = STATE_DATA_IN_FRAME;
    break;
  case STATE_DATA_IN_FRAME:
    if (c == BYTESTUFF) {
      dataState = STATE_DATA_XOR; // XOR next byte
      break;
    }
    if (c == START_STOP) // end of frame detected
    {
      processFrskyPacket(frskyRxBuffer); // FrskyRxBufferReady = 1;
      dataState = STATE_DATA_IDLE;
      break;
    }
    if (numPktBytes < FRSKY_RX_PACKET_SIZE)
      frskyRxBuffer[numPktBytes++] = c;
    else{
      dataState = STATE_DATA_IDLE;
      break;
    }
    break;
  case STATE_DATA_XOR:
    if (numPktBytes < FRSKY_RX_PACKET_SIZE)
      frskyRxBuffer[numPktBytes++] = c ^ STUFF_MASK;
    else{
      dataState = STATE_DATA_IDLE;
      break;
    }
    dataState = STATE_DATA_IN_FRAME;
    break;
  case STATE_DATA_IDLE:
    if (c == START_STOP) {
      numPktBytes = 0;
      dataState = STATE_DATA_START;
    }
    break;
  }
}

void processFrskyPacket(uint8_t *packet) {
	//get package ID
	switch (packet[0]) {
		case LINKPKT: // A1/A2/RSSI values
			//maybe we can use RSSI here to start an alarm when RSSI level gets low
			break;
		case USRPKT:
			numBytes = packet[1];
			if (numBytes > 6)
			  return;
			for (uint8_t i = 3; i < numBytes+3; i++) {
				/*if(packet[3]=='$')
					ltm_encodeTargetData(packet[i]);
				else*/
					parseTelemHubByte(packet[i]);
			}
		break;
	}
}

typedef enum {
  IDLE = 0, DATA_ID, DATA_LOW, DATA_HIGH, STUFF = 0x80
} STATE;

void parseTelemHubByte(uint8_t c) {
  static uint8_t dataId;
  static uint8_t byte0;
  static STATE state = IDLE;

  if (c == 0x5e) {
    state = DATA_ID;
    return;
  }
  if (state == IDLE) {
    return;
  }
  if (state & STUFF) {
    c = c ^ 0x60;
    state = (STATE) (state - STUFF);
  }
  if (c == 0x5d) {
    state = (STATE) (state | STUFF);
    return;
  }
  if (state == DATA_ID) {
    if (c > 0x3f) {
      state = IDLE;
    } else {
      dataId = c;
      state = DATA_LOW;
    }
    return;
  }
  if (state == DATA_LOW) {
    byte0 = c;
    state = DATA_HIGH;
    return;
  }
  state = IDLE;

  switch (dataId) {
  case GPS_ALT_BP_ID:
    alt = (int16_t)((c << 8) + byte0);
    gotAlt = true;
    break;
  case BARO_ALT_BP_ID:
    alt = (int16_t)((c << 8) + byte0);
    gotAlt = true;
    break;	
  case GPS_LON_BP_ID:
    lon_bp = (c << 8) + byte0;
    break;
  case GPS_LON_AP_ID:
    lon_ap = (c << 8) + byte0;
    break;
  case GPS_LAT_BP_ID:
    lat_bp = (c << 8) + byte0;
    break;
  case GPS_LAT_AP_ID:
    lat_ap = (c << 8) + byte0;
    break;
  case GPS_LON_EW_ID:
    EW = byte0;
    break;
  case GPS_LAT_NS_ID:
    NS = byte0;
    break;
  case TEMP2:
	if(telemetry_provider==1){
		sats = byte0 / 10;
		fix = byte0 % 10;
	} else if(telemetry_provider==2){
		sats = byte0 % 100;
	} else
		sats = byte0;
    break;
  }
	if ((NS == 'N' || NS == 'S') && (EW == 'E' || EW == 'W')) {

		frskyd_setLat();
		frskyd_setLon();
		frskyd_setAlt();
		frskyd_setSats();
		gotFix = true;
	} else
		telemetry_failed_cs++;
}

