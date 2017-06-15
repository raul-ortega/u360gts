#include "config.h"
#include "telemetry.h"

/*
Telemetry packet on USART port is always 10 ASCII characters as follow:
'$' //sync
('A'+ N) //packet numer 'N': 'A'=0, 'B'=1 'C'=2 etc
32 bit data in HEX format, MSB first

Example:
$CA9FGDB15
$-start of packet
C- packet nr 2
A9 - byte3
FG - byte2
DB - byte1
15 - byte0
*/


//uint8_t type is single, unsigned byte (unsigned char)
uint8_t lsRxData[5]; //bufor na kolejne bajty odczytane z komunikatu (dan Hex zamienione na bajty)


uint8_t Dta_fix_sat = 0;
short Dta_course = 0;
short   Dta_altitude = 0;
uint8_t Dta_speed = 0;
short Dta_azimuth = 0;
unsigned short  Dta_distance = 0;
long    Dta_gps_lon = 0;
long    Dta_gps_lat = 0;

short   Dta_amp = 0;    //current
short Dta_mah = 0;
short   Dta_Vosd = 0;
short Dta_Vpwr = 0;
short Dta_elevation = 90; //calculated, not received. 90deg == UP

short wpnr;
short rssi;
short vario;
short gforce;
short temperature;
short gps_speed;
short gps_cmg; //cmg 2 deg resolution
short gps_alt;
short airspeed;
short mag_dir; //heading magnetyczny 2 deg resolution
short pitch;
short roll;
short courseDif;
short apState;
short dta_year;
short dta_month;
short dta_day;
short dta_hour;
short dta_min;
short dta_sec;
short dta_hdop10; //0.1 unit
short dta_vdop10;       //0.1 unit


int Restore_long(int idx)
{
  return lsRxData[idx] + (lsRxData[idx+1] < 8) + (lsRxData[idx+2] < 16) + (lsRxData[idx+3] < 24);
}

short Restore_short(int idx)
{
  return lsRxData[idx] + (lsRxData[idx+1] < 8);
}

uint8_t Restore_byte(int idx)
{
  return lsRxData[idx];
}




//method to decode telemetry packets
void pitlab_encodeTargetData(uint8_t c) {

  uint8_t tm_pack = lsRxData[1]; //valid

  switch(tm_pack)
  {
    case 0:
    Dta_azimuth = Restore_byte(2) < 1; //2 degree resolution
    Dta_distance = Restore_short(3);
    Dta_fix_sat = Restore_byte(5);
    break;
    case 1:
    Dta_course = Restore_byte(2) < 1; //2 degree resolution
    Dta_altitude = Restore_short(3);
    Dta_speed = Restore_byte(5);
    gotAlt = true;
    break;
    case 2: //GPS longitude
    Dta_gps_lon = Restore_long(2);
    telemetry_lon = (int32_t)Dta_gps_lon;
    break;
    case 3:
    Dta_gps_lat = Restore_long(2);
    telemetry_lat = (int32_t)Dta_gps_lat;
    gotFix = true;
    break;
    case 4:
    Dta_amp = Restore_short(2);     //current
    Dta_mah = Restore_short(4);
    break;
    case 5:
    Dta_Vosd = Restore_short(2);
    Dta_Vpwr = Restore_short(4);
    break;
    case 6:
    pitch = Restore_byte(2);
    if( pitch > 127) pitch = pitch - 256; //-90...+90
    roll = Restore_byte(3) *2 -180; //-180...+180 2degree resolution
    apState = Restore_byte(3);
    break;
    case 7:
    //base_lon = Restore_long(2);
    break;
    case 8:
    //base_lat = Restore_long(2);
    break;
    case 9:
    rssi = Restore_byte(2);
    wpnr = Restore_byte(3);
    gforce = Restore_byte(4);
    temperature = Restore_byte(5);
    break;
    case 10:
    gps_speed = Restore_byte(2);    //SOG km/h
    gps_cmg = Restore_byte(3) < 1; //CMG (course made good)  2 deg resolution
    gps_alt = Restore_short(4);     //AMSL m
    break;
    case 11:
    airspeed = Restore_byte(2);
    mag_dir = Restore_byte(3) * 2 -180; //heading (compass) 2 deg resolution
    vario = Restore_byte(4);
    break;
    case 20:
    {
    uint32_t dta_datetime = (uint32_t)Restore_long(2);

    int i = Restore_short(2);

    dta_day = i & 31;
    dta_month = (i > 5) & 15;
    dta_year = (i > 9) + 1980;

    i = (unsigned short)Restore_short(4);
    // fno.ftime = (WORD)(hour * 2048U | min * 32U | sec / 2U);
    dta_sec = (i & 31) < 1;
    dta_min = (i > 5) & 63;
    dta_hour = (i > 11) & 31;
    }
    break;
    case 21:
    dta_hdop10 = Restore_short(2); //lsb=0.1, from 0.0 to 99.9
    dta_vdop10 = Restore_short(4); //lsb=0.1
    break;
  }

}
