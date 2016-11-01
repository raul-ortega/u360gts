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

/*
 * http://www.rc-network.de/forum/showthread.php/281496-Graupner-HoTT-Telemetrie-Sensoren-Eigenbau-DIY-Telemetrie-Protokoll-entschl%C3%BCsselt/page21/*

 *
 * http://johnlenfr.1s.fr/index.php?option=com_content&view=article&id=520:telemetrie-graupner-lipometre40-gps&catid=59:lipometre-hott&Itemid=110
 *
 *
 * GPS MODULE #33600
 *
struct HOTT_GPS_MSG{
uint8_t startByte; //#1 Starting byte data == 0x7C
uint8_t sensorID; //#2 Sensor ID GPS Sensor == 0x8A
uint8_t warning_beeps; //#3 Byte 3: 0…= warning beeps
VOICE OR BIP WARNINGS
0x00 00 No alarm
0x01 01 A Min Speed ​​A
0x02 02 B Descent / B 3s -1 m / s
0x03 03 C Descent / s C -10 m / s
0x04 04 D Max Distance D
0x05 05 E
0x06 06 F
0x07 07 G
0x08 08 H
0x09 09 I
0xA 10 J
0xB 11 K
0xC 12 L Max Speed ​​L
0xD 13 M Rate of Climb / 3s M 1 m / s
0xE 14 N Rate of Climb / s N 10 m / s
0xF 15 O Min Old O
0x10 16 P
0x11 17 Q
0x12 18 R
0x13 19 S
0x14 20 T
0x15 21 U
0x16 22 V
0x17 23 W
0x18 24 X
0x19 25 Y
0x1A 26 Z Max Old Z

uint8_t sensorTextID; //#4 160 0xA0 Sensor ID Neu!
uint8_t alarmInverse1; //#5 01 inverse status
uint8_t alarmInverse2; //#6 00 inverse status status 1 = no GPS Signal
uint8_t flightDirection; //#7 119 = Flightdir./dir. 1 = 2°; 0° (North), 9 0° (East), 180° (South), 270° (West)
uint16_t GPSSpeed; //#8 LSB 196 8 = speed / low byte GPS speed 8km / h
//#9 MSB 002 0 = speed / GPS speed high byte
uint8_t LatitudeNS; //#10 000 = N = 48°39’988 / north=0, south=1
uint16_t LatitudeMin; //#11 231 0xE7 = 0x12E7 = 4839
//#12 018 18=0x12
uint16_t LatitudeSec; //#13 171 220 = 0xDC = 0x03DC =0988
//#14 016 3 = 0x03
uint8_t longitudeEW; //#15 000 = E= 9° 25’9360 / east=0, west=1
uint16_t longitudeMin; //#16 150 157 = 0x9D = 0x039D = 0925
//#17 003 3 = 0x03
uint16_t longitudeSec; //#18: 056 144 = 0x90 0x2490 = 9360
//#19 004 36 = 0x24
uint16_t distance; //#20 LSB 027 123 = Distance low byte 6 = 6 m
//#21 MSB 036 35 = Distance high byte
uint16_t altitude; //#22 LSB 243 244 = Altitude low byte 500 = 0m
//#23 MSB 001 1 = Altitude high byte
//#24 LSB 48 = m/s (resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s))
//#25 MSB 117 = High Byte m/s resolution 0.01m
uint16_t climbrate1s; //#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s 120 = 0m/3s
uint8_t GPSNumSat; //#27 GPS.Satelites (number of satelites) (1 byte)
uint8_t GPSFixChar; //#28 GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte)
uint8_t HomeDirection; //#29 HomeDirection (direction from starting point to Model position) (1 byte)
uint8_t angleXdirection; //#30 angle x-direction (1 byte)
uint8_t angleYdirection; //#31 angle y-direction (1 byte)
uint8_t angleZdirection; //#32 angle z-direction (1 byte)
int8_t gps_time_h; //#33 UTC time hours / gyro x low byte (2 bytes)
int8_t gps_time_m; //#34 UTC time minutes / gyro x high byte
int8_t gps_time_s; //#35 UTC time seconds / gyro y low byte (2 bytes)
int8_t gps_time_sss; //#36 UTC time milliseconds / gyro y high byte
int16_t msl_altitude; //#37 mean sea level altitude / gyro z low byte (2 bytes)
//#38 gyro z high byte
uint8_t vibration; //#39: vibration (1 bytes)
uint8_t Ascii4; //#40: 00 ASCII Free Character [4]
uint8_t Ascii5; //#41: 00 ASCII Free Character [5]
uint8_t GPS_fix; //#42: 00 ASCII Free Character [6], we use it for GPS FIX
uint8_t version; //#43: 00 version number
uint8_t endByte; //#44: 0x7D End byte
uint8_t parity; //#45: CHECKSUM Parity Byte
} ;
 */
#include "config.h"
#ifdef HOTT
#include "telemetry.h"

int32_t getTargetLat() {
  return 0;
}

int32_t getTargetLon() {
  return 0;
}

int16_t getTargetAlt() {
  return 0;
}

void encodeTargetData(uint8_t c) {

}



#endif

