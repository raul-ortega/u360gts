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
LTM is an air to ground telemetry, so it's one way only; No ACK & stuffs it just stream datas to the ground at low baudrates.

Depending of flighcontroller implementation & configuration it starts at boot or when armed.

Protocol is binary, using little endian.
There's 3 frames to consider:

.

G Frame (GPS position) (2hz @ 1200 bauds , 5hz >= 2400 bauds): 18BYTES 0x24 0x54 0x47 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0
$ T G --------LAT-------- -------LON--------- SPD --------ALT-------- SAT/FIX CRC
A Frame (Attitude) (5hz @ 1200bauds , 10hz >= 2400bauds): 10BYTES
0x24 0x54 0x41 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0

$ T A --PITCH-- --ROLL--- -HEADING- CRC

S Frame (Sensors) (2hz @ 1200bauds, 5hz >= 2400bauds): 11BYTES
0x24 0x54 0x53 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0

$ T S VBAT(mv) Current(ma) RSSI AIRSPEED ARM/FS/FMOD CRC

values are:

LAT/LON: int32 (cm)
SPD ( speed ) : uint8 ( m/s )
ALT ( alititude ) int32 ( cm )
SATS ( number of sat visible ) 6bits
FIX ( sat fix ) 2 bits ( sats & fix are coupled in the same byte )
PITCH/ROLL/HEADING int16 ( deg , -180/180° range )
VBAT( voltage ) uint16 (mv )
Current uint16 ( ma )
RSSI : uint8 ( in % )
AIRSPEED : uint8 (m/s)
ARM/FS/FMOD: uint8 ARM armed status: 1 bit ( first ) FS : failsafe status : 1 bit ( second one ) FMOD: flight mode : last 6 bits: // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3, // 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, // 12: Circle, 13: RTH, 14: FollowMe, 15: LAND, 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown
Each frame end with a CRC byte, it's a simple XOR from first payload byte to last one ( starting at 4th byte , headers are not xored )

Attached LTM-log with sample attitude data nothing else (changed text document into picture for attaching here)
 */
#include "config.h"
#include "telemetry.h"
#include "Arduino.h"

//
#define LTM_HEADER_START1 0x24 //$
#define LTM_HEADER_START2 0x54 //T
#define LTM_GFRAME 0x47 //G Frame
#define LTM_AFRAME 0x41 //A Frame

#define LTM_GFRAME_LENGTH 18
#define LTM_AFRAME_LENGTH 10

// machine states
enum LtmDataState {
    IDLE,
    STATE_START1,
    STATE_START2,
    STATE_MSGTYPE,
    STATE_DATA
  };

//
static uint8_t LTM_Buffer[LTM_GFRAME_LENGTH-4];
static uint8_t LTM_Index;
static uint8_t LTM_cmd;
static uint8_t LTM_chk;
static uint8_t LTM_read_index;
static uint8_t LTM_frame_length;
static uint8_t dataState = IDLE;

int32_t temp_alt;
uint8_t satsfix;
uint8_t fix_type;

void parseLTM_GFRAME(void);
uint8_t ltmread_u8();
uint16_t ltmread_u16();
uint32_t ltmread_u32();

void ltm_encodeTargetData(uint8_t c) {

	    if (dataState == IDLE && c == '$') {
	      dataState = STATE_START1;
	    }
	    else if (dataState == STATE_START1 && c == 'T') {
	    	dataState=STATE_START2;
	    }
	    else if (dataState == STATE_START2) {
	      switch (c) {
	         case 'G':
	           LTM_frame_length = LTM_GFRAME_LENGTH;
	           dataState = STATE_MSGTYPE;
	           break;
	         case 'A':
		       LTM_frame_length = LTM_AFRAME_LENGTH;
		       dataState = STATE_MSGTYPE;
	           break;
	         case 'S':
	        	 dataState=IDLE;
	        	 return;
	           break;
	         default:
	           dataState = IDLE;
	      }
	      LTM_cmd = c;
	      LTM_Index=0;
	    }
	    else if (dataState == STATE_MSGTYPE) {
		  if(LTM_Index == 0) {
		  LTM_chk = c;
		  }
		  else {
		  LTM_chk ^= c;
		  }
	      if(LTM_Index == LTM_frame_length-4) {   // received checksum byte
	        if(LTM_chk == 0) {
	        	LTM_read_index = 0;
	        	parseLTM_GFRAME();
	            dataState = IDLE;
	        }
	        else {                                                   // wrong checksum, drop packet
	        	dataState = IDLE;
	        	telemetry_failed_cs++;
	        }
	      }
	      else LTM_Buffer[LTM_Index++]=c;
	    }
}

void parseLTM_GFRAME(void) {
  if (LTM_cmd==LTM_GFRAME)
  {
    telemetry_lat = (int32_t)ltmread_u32();
    telemetry_lat = telemetry_lat/10;
    telemetry_lon = (int32_t)ltmread_u32();
    telemetry_lon = telemetry_lon/10;
    telemetry_speed = ltmread_u8();
    temp_alt = (int32_t)ltmread_u32();//10000000;
    telemetry_alt = (int16_t)(temp_alt/100);
    satsfix = ltmread_u8();
    telemetry_sats = (int16_t)((satsfix >> 2) & 0xFF);
    telemetry_fixtype = satsfix & 0b00000011;
    if(telemetry_sats>=5) gotFix = 1;
    gotAlt = true;
  }
  if (LTM_cmd==LTM_AFRAME)
    {
      telemetry_pitch = radians((int16_t)ltmread_u16());
      telemetry_roll =  radians((int16_t)ltmread_u16());
      telemetry_course = (int16_t)ltmread_u16() * 1.0f ;
      telemetry_yaw = radians(telemetry_course);
  }
}
uint8_t ltmread_u8()  {
  return LTM_Buffer[LTM_read_index++];
}

uint16_t ltmread_u16() {
  uint16_t t = ltmread_u8();
  t |= (uint16_t)ltmread_u8()<<8;
  return t;
}

uint32_t ltmread_u32() {
  uint32_t t = ltmread_u16();
  t |= (uint32_t)ltmread_u16()<<16;
  return t;
}



