#include "config.h"
#include "telemetry.h"

int32_t Restore_long(int idx);
int16_t Restore_short(int idx);
uint8_t Restore_byte(int idx);
void pitlab_encodeTargetData(uint8_t c);
void preProcessHexString(void);
void processPitlabFrame(void);
uint8_t hex2int(uint8_t *a, uint8_t len);
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

int32_t gps_lat;
int32_t gps_lon;


//uint8_t type is single, unsigned byte (unsigned char)
uint8_t lsRxData[5]; //bufor na kolejne bajty odczytane z komunikatu (dan Hex zamienione na bajty)
uint8_t hexString[8];

enum PitlabDataState {
    IDLE,
    STATE_START1,
    STATE_START2,
    STATE_DATA
  };

static uint8_t dataState = IDLE;
uint8_t dataIdx=0;

int32_t Restore_long(int idx)
{
  return (int32_t)lsRxData[idx] + ((int32_t)lsRxData[idx+1] << 8) + ((int32_t)lsRxData[idx+2] << 16) + ((int32_t)lsRxData[idx+3] << 24);
}

int16_t Restore_short(int idx)
{
  return (int16_t)lsRxData[idx] + ((int16_t)lsRxData[idx+1]  << 8);
}

uint8_t Restore_byte(int idx)
{
  return lsRxData[idx];
}


void pitlab_encodeTargetData(uint8_t c) {

	if (dataState == IDLE && c == '$') {
		dataIdx = 0;
		dataState = STATE_START1;
		return;
	} else if (dataState == STATE_START1) {
		lsRxData[0] = c - 'A';
		dataState=STATE_START2;
		return;
	} else if (dataState == STATE_START2) {
		hexString[dataIdx++] = c;
		if(dataIdx == 8)
			dataState = STATE_DATA;
	}
	if (dataState == STATE_DATA){
		preProcessHexString();
		processPitlabFrame();
		dataState = IDLE;
	}
}

void preProcessHexString(void){
	uint8_t str_buffer[2];
	uint8_t sIdx = 0;
	for(uint8_t i = 1; i < 5; i++){
		for(uint8_t j = 0; j < 2; ++j){
			str_buffer[j] = hexString[sIdx++];
		}
		lsRxData[5-i] = hex2int(str_buffer,2);
	}
}

void processPitlabFrame(void){
	switch(lsRxData[0])
	{
	case 0:
		telemetry_sats = (uint16_t)Restore_byte(4);
		break;
	case 1:
		telemetry_alt = (int16_t)Restore_short(2);
		gotAlt = true;
		break;
	case 2:
		gps_lon = Restore_long(1);
		telemetry_lon = (int32_t)(round(((double)gps_lon * 100.0)/60.0));
		break;
	case 3:
		gps_lat = Restore_long(1);
		telemetry_lat = (int32_t)(round(((double)gps_lat * 100.0)/60.0));
		if(telemetry_sats >= 5) gotFix = true;
		break;
	}
}

uint8_t hex2int(uint8_t *a, uint8_t len)
{
   uint8_t i;
   int val = 0;

   for(i=0;i<len;i++) {
      if(a[i] <= 57)
       val += (a[i]-48)*(1<<(4*(len-1-i)));
      else
       val += (a[i]-55)*(1<<(4*(len-1-i)));
   }
   return val;
}
