/*
 * This file is part of u360gts, aka amv-open360tracker 32bits:
 * https://github.com/raul-ortega/amv-open360tracker-32bits
 *
 * The code by itself do nothing, it have to be combined with cleanflight
 * source code to be run on NAZE32 based boards for controlling a
 * DIY continuous 360 degree rotating antenna tracker system for FPV.
 *
 * Some functions are adaptations by Ra�l Ortega of the original code written by Samuel Brucksch:
 * https://github.com/SamuelBrucksch/open360tracker *
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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/atomic.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/utils.h"


#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/flash_m25p16.h"
#include "drivers/sonar_hcsr04.h"
#include "rx/rx.h"
#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/serial_cli.h"
#include "io/beeper.h"
#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "tracker/defines.h"
#include "tracker/config.h"
#include "tracker/telemetry.h"
#include "tracker/math.h"
#include "tracker/TinyGPS.h"
#include "tracker/servos.h"
#include "tracker/gps_estimation.h"
#include "tracker/interpolation.h"
#include "tracker/protocol_detection.h"

void calcTilt(void);
void getError(void);
void calculatePID();
void blinkLeds(uint8_t numblinks);
int getHeading(void);
//void calibrate_compass(void);
void servo_tilt_update();
void saveLastTilt(bool writteEeprom);
void tracker_setup();
void tracker_loop();
void updateBatteryStatus(void);
void updateServoTest(void);
void updateDigitalButtons(void);
void updateReadTelemetry(void);
void updateTelemetryLost(void);
void updateTargetPosition(void);
void updateHeading(void);
void updateFixedPages(void);
void updateSetHomeByGPS(void);
void updateMFD(void);
void enterMenuMode(void);
void exitMenuMode(void);
void updateSetHomeButton(void);
void updateMenuButton(void);
void updateTracking(void);
void proccessMenu(uint8_t menuButton);
void processMenuRoot(void);
void processMenuCalibrate(void);
void processMenuTelemetry(void);
void processMenuTelemetryProtocol(void);
void processMenuTelemetryBaudrate(void);
void processMenuFeature(uint16_t featureIndex);
void offsetTrimIncrease(void);
void offsetTrimDecrease(void);
float map(long x, long in_min, long in_max, long out_min, long out_max);
void calcEstimatedPosition();
bool couldLolcalGpsSetHome(bool setByUser);
bool couldTelemetrySetHome();
void updateCalibratePan();
uint16_t calculateDeltaHeading(uint16_t heading1, uint16_t heading2);
void setEpsMode(void);
int16_t getOffset(int16_t offset_master,int8_t offset_trim);
void updateProtocolDetection(void);
void protocolInit(void);
void trackingInit(void);
void telemetryPortInit(void);
void setHomeByLocalGps(positionVector_t *tracker, int32_t lat, int32_t lon, int16_t alt, bool home_updated, bool beep);
//EASING
int16_t _lastTilt;
int16_t tilt;
bool _servo_tilt_has_arrived = true;
uint8_t _tilt_pos = 0;
float _servo_tilt_must_move = -1;
float easingout = 0.0f;
float tiltTarget;

//TIMER/CALIB
unsigned long time;
unsigned long calib_timer;

//HOME
unsigned long home_timer;
unsigned long home_timer_reset;

//PID stuff
long Error[11];
long Accumulator;
long PID;
long Dk;
uint8_t Divider;

//SERVOS PWM
uint16_t pwmPan;
uint16_t pwmTilt;
uint16_t _lastPwmTilt;

uint16_t minPwmPan;
uint16_t maxPwmPan;
int16_t maxDeltaHeading;

//OFFSET TRIM STATE
typedef enum {
	PWMPAN0_UNKNOWN,
	FINDING_OUT_MIN_PWMPAN0,
	MIN_PWMPAN0_FOUND,
	FINDING_OUT_MAX_PWMPAN0,
	MAX_PWMPAN0_FOUND,
	PWMPAN0_CALCULATED_WITH_SUCCESS
};
uint8_t pwmPanState = PWMPAN0_UNKNOWN;
//TARGET/TRACKER POSITION
// The target position (lat/lon)
positionVector_t targetPosition;
// The tracker position (lat/lon)
positionVector_t trackerPosition;
float currentDistance = 0;
float currentSpeed = 0;
//uint16_t eps_frequency = 0;

//TELEMETRY VARS
uint16_t distance;
bool mfdTestMode;
bool gotAlt;
bool gotFix;
bool settingHome;
bool gotTelemetry=false;
bool lostTelemetry=true;

//TRACKER STATE VARS
bool homeSet;
bool homeReset;
bool trackingStarted;
bool currentState;
bool homeButtonCurrentState;
bool previousState;
bool homeButtonPreviousState;
bool gotNewHeading;
bool homeSet_BY_GPS = false;

//TIMERS
unsigned long servoPanTimer = 0;
unsigned long servoPanVarTime = 0;
unsigned long minServoPanVarTime = 0;
unsigned long servoPanTimerStart = 0;
unsigned long debugTimer = 0;
unsigned long easingTimer = 0;
unsigned long lostTelemetry_timer = 0;
unsigned long epsVectorTimer = 0;
unsigned long currentTimeMillis = 0;

//RSSI
extern uint16_t rssi;

//Display
uint8_t displayPageIndex=0;
extern uint8_t menuState;
extern uint8_t indexMenuOption;
uint8_t menuOption;
bool detection_title_updated = false;

//COMMON VARS
serialPort_t *trackerSerial;

//VBAT
static uint32_t vbatLastServiced = 0;
//static uint32_t ibatLastServiced = 0;
/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)

//OFFSET TRIM STATE
typedef enum {
	TRIM_STATE_DISABLED,
	TRIM_STATE_ENABLED,
	DISABLING_TRIM_STATE,
	TRIM_STATE_DISABLED_BY_USER
};

uint8_t OFFSET_TRIM_STATE = TRIM_STATE_DISABLED;
uint8_t EPS_MODE;
uint16_t EPS_DISTANCE_GAIN;
uint16_t EPS_FREQUENCY;
//DEBUG VARS
uint16_t debugIndex=0;

//LAT,LON esttimation
extern epsVector_t targetLast;
extern epsVector_t targetCurrent;
extern epsVector_t targetEstimated;

int16_t  imuheading=0;


extern uint32_t currentTime;
extern uint32_t previousTime;
extern uint16_t cycleTime;

extern float dT;

#ifdef SOFTSERIAL_LOOPBACK
serialPort_t *loopbackPort;
#endif

uint16_t homeButton=0;
uint8_t homeButtonCount=0;

uint16_t calibrateButton=0;
uint8_t calibrateButtonCount=0;

extern int16_t telemetry_sats;

int _contador=0;

extern const uint32_t baudRates[];
extern uint8_t telemetry_provider;
extern int32_t telemetry_lat;
extern int32_t telemetry_lon;

void tracker_setup(void)
{

  protocolInit();

  if(feature(FEATURE_AUTODETECT))
	  enableProtocolDetection();

  trackingInit();

  telemetryPortInit();

  setPrintfSerialPort(trackerSerial);

  LED0_OFF;

  if(masterConfig.init_servos==1) {
	  servosInit();
  }

  if(feature(FEATURE_EASING)) {
	  if(masterConfig.init_servos==1)
		  _lastTilt = 0;
	  else {
		  _lastTilt = masterConfig.easing_last_tilt;
		  _lastPwmTilt = map(_lastTilt, 0, 90, masterConfig.tilt0, masterConfig.tilt90);
	  }
  }



  time = millis();

  #ifdef TRACKER_DEBUG
  	  serialPrint(trackerSerial,"Setup finished\n");
  #endif

  LED0_OFF;
  LED1_OFF;

  blinkLeds(3);

  epsVectorsInit(&targetLast,&targetCurrent,&targetEstimated,masterConfig.eps_interpolation,masterConfig.eps_interpolation_points);

 }

void telemetryPortInit(void){
	trackerSerial = openSerialPort(masterConfig.serialConfig.portConfigs[0].identifier, FUNCTION_NONE, NULL, baudRates[masterConfig.serialConfig.portConfigs[0].msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
}

void trackingInit(void){
	setEpsMode();
	updateEPSParams();

	OFFSET_TRIM = masterConfig.offset_trim;
	OFFSET = getOffset(masterConfig.offset,masterConfig.offset_trim);

	gotAlt = false;
	gotFix = false;

	homeSet = false;
	homeReset = false;

	trackingStarted = false;

	settingHome = false;

	previousState = true;
	homeButtonPreviousState = true;

	currentState = true;
	homeButtonCurrentState = true;

	gotNewHeading = false;

	menuState = 0;

	targetPosition.home_alt = -32768;
}

void tracker_loop(void)
{

	static uint32_t loopTime;
	currentTime = micros();
	if (masterConfig.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0) {
		loopTime = currentTime + masterConfig.looptime;

		// Measure loop rate just after reading the sensors
		currentTime = micros();
		cycleTime = (int32_t)(currentTime - previousTime);
		previousTime = currentTime;

		dT = (float)cycleTime * 0.000001f;

		updateCompass(&masterConfig.magZero);

		updateCalibratePan();

		handleSerial();

		updateDigitalButtons();

		updateBatteryStatus();

		updateServoTest();

		updateReadTelemetry();

		updateTelemetryLost();

		updateProtocolDetection();

		updateTargetPosition();

		updateHeading();

		updateMenuButton();

		updateSetHomeButton();

		updateSetHomeByGPS();

		updateMFD();

		updateTracking();

		//update RSSI every 50Hz
		if (feature(FEATURE_RSSI_ADC) || (masterConfig.rxConfig.rssi_channel > 0)) {
			updateRSSI(currentTime);
		}
	}

        beeperUpdate();


	//update display
    if (feature(FEATURE_DISPLAY)) {
        updateDisplay();
    }

	if(feature(FEATURE_EASING)) {
		if(millis()-easingTimer > masterConfig.easing_millis){
		  servo_tilt_update();
		  easingTimer = millis();
		}
	}

#ifdef GPS
        if (feature(FEATURE_GPS)) {
            gpsThread();
        }
#endif

#ifdef TELEMETRY
	if (!cliMode && feature(FEATURE_TELEMETRY)) {
		telemetryProcess(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
	}
#endif

	/*if(feature(FEATURE_DEBUG) && !cliMode) {
		if(millis()-debugTimer>1000){
			printf("c: %d H: %d\r\n",(BUTTON(MENU_BUTTON)>0),(BUTTON(HOME_BUTTON)>0));
			debugTimer=millis();
		}
	}*/
}

//Tilt angle tiltTarget = atan(alt/dist)
void calcTilt(void) {


  //this will fix an error where the tracker randomly points up when plane is lower than tracker
  if (targetPosition.alt < trackerPosition.alt){
    targetPosition.alt = trackerPosition.alt;
  }

  //prevent division by 0
  if (targetPosition.distance == 0) {
    // in larger altitude 1m distance shouldnt mess up the calculation.
    //e.g. in 100m height and dist 1 the angle is 89.4° which is actually accurate enough
    targetPosition.distance = 1;
  }

  tiltTarget = toDeg(atan((float)(targetPosition.alt - trackerPosition.alt) / targetPosition.distance));

  if (tiltTarget < 0)
    tiltTarget = 0;
  else if (tiltTarget > 90)
    tiltTarget = 90;

  if(feature(FEATURE_EASING)) {
	if(_servo_tilt_has_arrived){
		_servo_tilt_must_move = tiltTarget;
		_servo_tilt_has_arrived = false;
		_tilt_pos = 0;
	}
  }
  else {
	pwmTilt = (uint16_t) map(tiltTarget,0,90,masterConfig.tilt0, masterConfig.tilt90);
    pwmWriteTiltServo(pwmTilt);
  }
}

void getError(void)
{
	int16_t delta;

	if(feature(FEATURE_NOPID)){
		delta = trackerPosition.heading - targetPosition.heading;
	} else {
		delta = targetPosition.heading - trackerPosition.heading;

		for (uint8_t i = 0; i < 10; i++) {
		  Error[i + 1] = Error[i];
		}

	}


	if (delta > 1800)
		delta -= 3600;
	else if (delta <= -1800)
		delta += 3600;
	Error[0] = delta;
}

void calculatePID(void)
{
  if(feature(FEATURE_NOPID)) {
	// Calculate pwmPan without usind PID control system
	int16_t PAN_SPEED;
	if(abs(Error[0]) >= masterConfig.nopid_map_angle*10)
		PAN_SPEED = masterConfig.nopid_max_speed;
	else
		PAN_SPEED = map(abs(Error[0]), 0, masterConfig.nopid_map_angle*10, masterConfig.min_pan_speed, masterConfig.nopid_max_speed);

	if(abs(Error[0]) <= masterConfig.nopid_min_delta)
		pwmPan = masterConfig.pan0;

	if (Error[0] < 0)
		pwmPan = masterConfig.pan0 + PAN_SPEED;
	else if(Error[0] > 0)
		pwmPan = masterConfig.pan0 - PAN_SPEED;

  } else {
	// Calculate pwmPan using PID control system
	Divider = masterConfig.max_pid_divider;
	PID = Error[0] * masterConfig.p;     // start with proportional gain
	Accumulator += Error[0];  // accumulator is sum of errors
	if (Accumulator > masterConfig.max_pid_accumulator)
		Accumulator = masterConfig.max_pid_accumulator;
	if (Accumulator < -1 * masterConfig.max_pid_accumulator)
		Accumulator = -1 * masterConfig.max_pid_accumulator;
	PID += masterConfig.i * Accumulator; // add integral gain and error accumulation
	Dk = masterConfig.d * (Error[0] - Error[10]);
	PID += Dk; // differential gain comes next
	PID = PID >> Divider; // scale PID down with divider
	// limit the PID to the resolution we have for the PWM variable
	if (PID >= masterConfig.max_pid_gain)
		PID = masterConfig.max_pid_gain;

	if (PID <= -1*masterConfig.max_pid_gain)
		PID = -1*masterConfig.max_pid_gain;

	if (Error[0] > masterConfig.max_pid_error) {
		pwmPan = masterConfig.pan0 + PID + masterConfig.min_pan_speed;
	} else if (Error[0] < -1 * masterConfig.max_pid_error) {
		pwmPan = masterConfig.pan0 + PID - masterConfig.min_pan_speed;
	} else {
		pwmPan = masterConfig.pan0;
	}
  }


}

void servo_tilt_update(){
  if(_servo_tilt_must_move < 0) _tilt_pos = 0;
  if(!_servo_tilt_has_arrived && _servo_tilt_must_move > -1){
	if(abs(_lastTilt-_servo_tilt_must_move) > masterConfig.easing_min_angle) {
	  if(_tilt_pos < masterConfig.easing_steps){
		if(_lastTilt <= _servo_tilt_must_move)
		  easingout = _lastTilt + easeTilt(_tilt_pos, 0, _servo_tilt_must_move - _lastTilt, masterConfig.easing_steps);
		else
			easingout = _lastTilt - easeTilt(_tilt_pos, 0, _lastTilt - _servo_tilt_must_move, masterConfig.easing_steps);
			pwmTilt=(int16_t)map(easingout,0,90,masterConfig.tilt0,masterConfig.tilt90);
			if(_lastPwmTilt != pwmTilt)
				pwmWriteTiltServo(pwmTilt);
			_tilt_pos++;
	  }
	  else {
		if(_tilt_pos == masterConfig.easing_steps){
		  pwmTilt = (uint16_t) map(_servo_tilt_must_move,0,90, masterConfig.tilt0, masterConfig.tilt90);
		  pwmWriteTiltServo(pwmTilt);
		  _lastTilt = _servo_tilt_must_move;
		  _tilt_pos=0;
		  _servo_tilt_has_arrived = true;
		  _servo_tilt_must_move = -1;
		}
	  }
	  _lastPwmTilt = pwmTilt;
	}
	else {
		_servo_tilt_has_arrived = true;
		_servo_tilt_must_move = -1;
	}
  }
}

void serialPrintln(void){
  serialPrint(trackerSerial, "\n");
}

void blinkLeds(uint8_t numblinks){

	for(int i=0;i<numblinks;i++) {
		  LED0_ON;
		  LED1_ON;
		  delay(250);
		  LED0_OFF;
		  LED1_OFF;
		  delay(250);
	  }
}


int getHeading(void) {
	static t_fp_vector imuVector;
	imuVector.A[0]=magADC[0];
	imuVector.A[1]=magADC[1];
	imuVector.A[2]=magADC[2];

	OFFSET_TRIM = masterConfig.offset_trim;
	OFFSET = getOffset(masterConfig.offset,masterConfig.offset_trim);

	int imuheading = imuCalculateHeading(&imuVector);

	return imuheading;
}

int16_t getOffset(int16_t offset_master,int8_t offset_trim){
	return offset_master - (int16_t)offset_trim;
}

void updateDigitalButtons(void) {
	//Home Button
	homeButton=GPIO_ReadInputDataBit(GPIOB, Pin_9);
	if(homeButton==0) {
		homeButtonCount++;
	} else if (homeButton==1) {
		homeButtonCount=0;
		DISABLE_BUTTON(HOME_BUTTON);
	}
	if(homeButtonCount > masterConfig.min_logic_level) {
		ENABLE_BUTTON(HOME_BUTTON);
	}
	//Calibrate Button
	calibrateButton=GPIO_ReadInputDataBit(GPIOB, Pin_8);
	if(calibrateButton==0) {
		calibrateButtonCount++;
	} else if (calibrateButton==1) {
		calibrateButtonCount=0;
		DISABLE_BUTTON(MENU_BUTTON);
	}
	if(calibrateButtonCount > masterConfig.min_logic_level) ENABLE_BUTTON(MENU_BUTTON);

}

void setHomeByTelemetry(positionVector_t *tracker, positionVector_t *target) {
  tracker->lat = target->lat;
  tracker->lon = target->lon;
  tracker->alt = target->alt;

  if(feature(FEATURE_DEBUG)){
	  tracker->lat = 47403583; tracker->lon = 8535850; tracker->alt = 474;
  }

  tracker->alt = 0;
  tracker->home_alt = target->alt;

  homeSet = true;
  homeSet_BY_GPS = false;
  homeReset = false;
  home_timer_reset = 0;
  epsVectorLoad(&targetLast,target->lat,target->lon,0,0,0);
  epsVectorLoad(&targetCurrent,target->lat,target->lon,0,0,millis());
}

void setHomeByLocalGps(positionVector_t *tracker, int32_t lat, int32_t lon, int16_t alt, bool home_updated, bool beep) {
  tracker->lat = lat;
  tracker->lon = lon;
  tracker->alt = alt;

  if(feature(FEATURE_DEBUG)) {
	  tracker->lat = 47403583; tracker->lon = 8535850; tracker->alt = 474;
  }

  if(home_updated) {
	  tracker->alt = alt - tracker->home_alt;
  } else {
	  tracker->alt = 0;
	  tracker->home_alt = alt;
  }

  if(targetPosition.home_alt == -32768)
	  targetPosition.alt = 0;

  homeSet = true;
  homeSet_BY_GPS = true;
  homeReset = false;
  home_timer_reset = 0;
  if(!home_updated) {
	  epsVectorLoad(&targetLast,lat,lon,0,0,0);
	  epsVectorLoad(&targetCurrent,lat,lon,0,0,millis());
  }
}

void updateBatteryStatus(void){
    if (feature(FEATURE_VBAT)) {
        if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
            vbatLastServiced = currentTime;
            updateBattery();
        }
    }
}

void updateFixedPages(void){

	displayPageIndex =
			PAGE_GPS * (displayPageIndex == PAGE_TELEMETRY && feature(FEATURE_GPS) && !PROTOCOL(TP_MFD)) + \
			PAGE_BATTERY * ((displayPageIndex == PAGE_TELEMETRY && !feature(FEATURE_GPS) && (feature(FEATURE_VBAT) || feature(FEATURE_RSSI_ADC) || (masterConfig.rxConfig.rssi_channel > 0))) || (displayPageIndex == PAGE_GPS && (feature(FEATURE_VBAT) || feature(FEATURE_RSSI_ADC) || (masterConfig.rxConfig.rssi_channel > 0)))) + \
			PAGE_TELEMETRY * (displayPageIndex == 0);
	if(displayPageIndex !=0 ){
		//Show fixed page
		displayShowFixedPage(displayPageIndex);
	} else  {
		//Activate cycling
		displayResetPageCycling();
		displayEnablePageCycling();
	}
}

void updateServoTest(void){
	if(PROTOCOL(TP_SERVOTEST)){
		 if(SERVO(SERVOPAN_MOVE)) {
			 targetPosition.heading = SERVOTEST_HEADING * 10;
			 DISABLE_SERVO(SERVOPAN_MOVE);
		 }

		 if(SERVO(SERVOTILT_MOVE)) {
			if(feature(FEATURE_EASING) && masterConfig.easing > 0) {//#ifdef TILT_EASING
				  _servo_tilt_must_move = SERVOTEST_TILT;
				  _servo_tilt_has_arrived = false;
			}
			else
			{
				tilt = map(SERVOTEST_TILT, 0, 90, masterConfig.tilt0, masterConfig.tilt90);
				pwmWriteTiltServo(tilt);
			}
			DISABLE_SERVO(SERVOTILT_MOVE);
		}
	}
}

void updateReadTelemetry(void){
	if (serialRxBytesWaiting(trackerSerial)>1){
		uint8_t c = serialRead(trackerSerial);

		evaluateOtherData(trackerSerial,c);

		LED0_ON;

		if(!PROTOCOL(TP_SERVOTEST) && !cliMode)
			encodeTargetData(c);

		if(!PROTOCOL(TP_SERVOTEST)) {
			gotTelemetry = true;
			lostTelemetry = false;
			lostTelemetry_timer = 0;
		}


	} else {
		LED0_OFF;
		gotTelemetry = false;
	}
}

void updateTelemetryLost(void){
	if(lostTelemetry) {
		gotFix = false;
		gotAlt = false;
		return;
	}

	if (!gotTelemetry && lostTelemetry_timer == 0){
		lostTelemetry_timer = millis();
		return;
	}

	if(!gotTelemetry && (millis() - lostTelemetry_timer > 3000)){
		lostTelemetry = true;
		if(feature(FEATURE_AUTODETECT)){
			showAutodetectingTitle(0);
			enableProtocolDetection();
		}
	}
}

void updateTargetPosition(void){
	if(!PROTOCOL(TP_SERVOTEST)){
		if (gotAlt) {

			if(telemetry_sats >= masterConfig.telemetry_min_sats && targetPosition.home_alt == -32768)
				targetPosition.home_alt = getTargetAlt(0);

			targetPosition.alt = getTargetAlt(targetPosition.home_alt);

			if(PROTOCOL(TP_MFD)){
				distance = getDistance();
				targetPosition.heading = getAzimuth() * 10;
			}
			else{
				  //Do nothing
			}

			gotAlt = false;
		}

		if(!PROTOCOL(TP_MFD)){
			if (gotFix) {
				targetPosition.lat = getTargetLat();
				targetPosition.lon = getTargetLon();
				currentDistance = distance_between(targetLast.lat / TELEMETRY_LATLON_DIVIDER_F,targetLast.lon / TELEMETRY_LATLON_DIVIDER_F,targetPosition.lat / TELEMETRY_LATLON_DIVIDER_F,targetPosition.lon / TELEMETRY_LATLON_DIVIDER_F);
				currentTimeMillis = millis();
				currentSpeed = epsVectorSpeed(targetLast.time,currentTimeMillis,currentDistance);
				if(masterConfig.max_speed_filter == 0 || targetCurrent.speed < masterConfig.max_speed_filter){
					epsVectorLoad(&targetCurrent,targetPosition.lat,targetPosition.lon,currentDistance,targetLast.time,currentTimeMillis);//epsVectorSpeed(targetLast.time,currentTimeMillis,currentDistance);
					if(homeSet) {
						if(feature(FEATURE_EPS)) {
							targetCurrent.heading = course_to(targetLast.lat / TELEMETRY_LATLON_DIVIDER_F,targetLast.lon / TELEMETRY_LATLON_DIVIDER_F,targetCurrent.lat / TELEMETRY_LATLON_DIVIDER_F,targetCurrent.lon / TELEMETRY_LATLON_DIVIDER_F);
							if(masterConfig.eps > 1){
								if(!pvFull())
									pvPut(&targetCurrent,1);

								if(masterConfig.eps_interpolation) {
									epsVectorAddPoint(&targetLast,&targetCurrent);
								}
							}
							if(masterConfig.eps == 1 || masterConfig.eps == 3) {
								calcEstimatedPosition();
								epsVectorTimer = millis();
							}
						} else {
							targetPosition.distance = distance_between(trackerPosition.lat / TELEMETRY_LATLON_DIVIDER_F, trackerPosition.lon / TELEMETRY_LATLON_DIVIDER_F, targetPosition.lat / TELEMETRY_LATLON_DIVIDER_F, targetPosition.lon / TELEMETRY_LATLON_DIVIDER_F);
							targetPosition.heading = course_to(trackerPosition.lat / TELEMETRY_LATLON_DIVIDER_F, trackerPosition.lon / TELEMETRY_LATLON_DIVIDER_F, targetPosition.lat / TELEMETRY_LATLON_DIVIDER_F, targetPosition.lon / TELEMETRY_LATLON_DIVIDER_F) * 10.0f;
						}
					}
					epsVectorCurrentToLast(&targetCurrent,&targetLast);
				}
				gotFix = false;
			}
			if(feature(FEATURE_EPS) && masterConfig.eps > 1)
				calcEstimatedPosition();
		}
	}
}

void calcEstimatedPosition(){
	if(homeSet && lostTelemetry == false) {
		if(masterConfig.eps == 1 || (millis()- epsVectorTimer > masterConfig.eps_frequency && masterConfig.eps > 1)){
			epsVectorEstimate(&targetLast,&targetCurrent,&targetEstimated,masterConfig.eps_gain,masterConfig.eps_frequency,masterConfig.eps);
			targetPosition.distance = distance_between(trackerPosition.lat / TELEMETRY_LATLON_DIVIDER_F, trackerPosition.lon / TELEMETRY_LATLON_DIVIDER_F, targetPosition.lat / TELEMETRY_LATLON_DIVIDER_F, targetPosition.lon / TELEMETRY_LATLON_DIVIDER_F);
			targetPosition.heading = course_to(trackerPosition.lat / TELEMETRY_LATLON_DIVIDER_F, trackerPosition.lon / TELEMETRY_LATLON_DIVIDER_F, targetEstimated.lat / TELEMETRY_LATLON_DIVIDER_F, targetEstimated.lon / TELEMETRY_LATLON_DIVIDER_F) * 10.0f;
			epsVectorTimer = millis();
		}
	}
}

void updateHeading(void){
	// we update the heading every 14ms to get as many samples into the smooth array as possible
	if (millis() > time) {
		time = millis() + 14;
		trackerPosition.heading = getHeading();
		gotNewHeading = true;
	}
}

void offsetTrimIncrease(void){
	masterConfig.offset_trim++;
	if(masterConfig.offset_trim > 20)
		masterConfig.offset_trim = 20;
	else
		writeEEPROM();

}
void offsetTrimDecrease(void){
	masterConfig.offset_trim--;
	if(masterConfig.offset_trim < -20)
		masterConfig.offset_trim = -20;
	else
		writeEEPROM();
}

void updateMenuButton(void){
	currentState = !BUTTON(MENU_BUTTON);
	if (currentState != previousState) {
		LED1_TOGGLE;
		// When trim offset mode is disabled this button acts as menu browser
		if(OFFSET_TRIM_STATE == TRIM_STATE_DISABLED || OFFSET_TRIM_STATE == TRIM_STATE_DISABLED_BY_USER) {
			/*
			 * 	SI EL OFFSET EST� MODIFICADO, RESETEARLO
			 */
			if(menuState && !cliMode) {
				//MENU MODE
				if (!currentState && calib_timer == 0) {
					calib_timer = millis();
				} else if (currentState && millis() - calib_timer < 1000) {
					proccessMenu(MENU_BUTTON);
					calib_timer = 0;
				} else if (currentState && millis() - calib_timer < 1500) {
					//button not pressed long enough
					calib_timer = 0;
				} else if (currentState && millis() - calib_timer > 1500) {
					calib_timer = 0;
					exitMenuMode();
				}
			} else if(!menuState){
				//TELEMETRY MODE
				if (!currentState && calib_timer == 0) {
					calib_timer = millis();
				} else if (currentState && millis() - calib_timer < 1000) {
					updateFixedPages();
					calib_timer = 0;
				} else if (currentState && millis() - calib_timer < 1500) {
					//button not pressed long enough
					calib_timer = 0;
				} else if (currentState && millis() - calib_timer > 1500) {
					//start calibration routine if button pressed > 4s and released
					//ENABLE_STATE(CALIBRATE_MAG);
					calib_timer = 0;
					enterMenuMode();
				}
			}
		// When trim offset is enabled, this button decrease the offset one unit
		} else if(OFFSET_TRIM_STATE == TRIM_STATE_ENABLED){
			if (!currentState && calib_timer == 0) {
				calib_timer = millis();
			} else if (currentState && millis() - calib_timer < 1000) {
				offsetTrimDecrease();
				calib_timer = 0;
			} else if (currentState && millis() - calib_timer > 2000) {
				calib_timer = 0;
				//FIX ME: RESET OFFSET TRIM
			}
		}

		previousState = currentState;
	}
}


void updateSetHomeButton(void){
	//If disabling offset trim, wait untill button is released
	if(OFFSET_TRIM_STATE == DISABLING_TRIM_STATE) {
		if(!BUTTON(HOME_BUTTON)) {
			OFFSET_TRIM_STATE = TRIM_STATE_DISABLED_BY_USER;
			home_timer = 0;
			home_timer_reset = 0;
		}
		return;
	}

	homeButtonCurrentState = !BUTTON(HOME_BUTTON);
	if (homeButtonCurrentState != homeButtonPreviousState) {
		LED1_TOGGLE;
		// Modo NO trimado
		if(OFFSET_TRIM_STATE == TRIM_STATE_DISABLED || OFFSET_TRIM_STATE == TRIM_STATE_DISABLED_BY_USER) {
			// Modo MENU
			if(menuState) {
				if (!homeButtonCurrentState && home_timer == 0) {
					home_timer = millis();
				} else if (homeButtonCurrentState && (millis() - home_timer < 1000)) {
					proccessMenu(HOME_BUTTON);
					home_timer = 0;
				}
			// Modo HOME
			} else {
				if (!homeButtonCurrentState && home_timer == 0) {
					home_timer = millis();
				// SET HOME
				} else if (homeButtonCurrentState && (millis() - home_timer < 1000) && !PROTOCOL(TP_MFD)) {
					home_timer = 0;
					// By telemetry if has enought sats
					if(!homeSet && telemetry_sats >= masterConfig.telemetry_min_sats)
						setHomeByTelemetry(&trackerPosition, &targetPosition);
					// By local GPS because telemetry hasn't got enought sats and we don't want wait more time.
					else if(!homeSet && couldLolcalGpsSetHome(true)){
						setHomeByLocalGps(&trackerPosition,GPS_coord[LAT]/10,GPS_coord[LON]/10,GPS_altitude,false,true);
					}
				// RESET HOME
				} else if (homeButtonCurrentState && (millis() - home_timer > 2000) && !PROTOCOL(TP_MFD)) {
					if(homeSet && !homeReset && !trackingStarted){
						homeSet = false;
						homeReset = true;
						home_timer = 0;
						home_timer_reset = 0;
					}
				}
			}
		// Modo TRIMADO OFFSET
		} else if(OFFSET_TRIM_STATE == TRIM_STATE_ENABLED){
			if (!homeButtonCurrentState && home_timer == 0) {
				home_timer = millis();
			// INCREMENTO
			} else if (homeButtonCurrentState && millis() - home_timer < 1000) {
				offsetTrimIncrease();
				home_timer = 0;
			// DECREMENTO
			} else if (homeButtonCurrentState && millis() - home_timer > 2000) {
				home_timer = 0;
				OFFSET_TRIM_STATE = DISABLING_TRIM_STATE;
			}
		}

		homeButtonPreviousState = homeButtonCurrentState;
	}

}

void updateSetHomeByGPS(void){
	if(PROTOCOL(TP_SERVOTEST) || PROTOCOL(TP_MFD) || PROTOCOL(TP_CALIBRATING_MAG) || PROTOCOL(TP_CALIBRATING_PAN0) || PROTOCOL(TP_CALIBRATING_MAXPAN))
		return;
	if(homeReset && lostTelemetry){
			telemetry_lat = 0;
			telemetry_lon = 0;
			telemetry_sats = 0;
	}
	if(homeReset && home_timer_reset == 0 ) {
		home_timer_reset = millis();
	} else if(homeReset && millis()- home_timer_reset < 5000 ) {
		return; 	// We have 5 seconds to set home manually by telemetry
	} else if(homeReset && millis()- home_timer_reset >= 5000 && couldLolcalGpsSetHome(false)){
		homeReset = false;
		home_timer_reset = 0;
		//if((!homeSet || (homeSet && homeSet_BY_GPS)) && feature(FEATURE_GPS) && STATE(GPS_FIX) && (GPS_numSat >= masterConfig.home_min_sats)) // || GPS_numSat>3))
		setHomeByLocalGps(&trackerPosition,GPS_coord[LAT]/10,GPS_coord[LON]/10,GPS_altitude,false,true);
		if(masterConfig.gpsConfig.homeBeeper)
			  beeper(BEEPER_ARMING_GPS_FIX);
	} else if(!homeSet && couldLolcalGpsSetHome(false)) {
		homeReset = true;
		home_timer_reset = 0;
	} else if(masterConfig.update_home_by_local_gps == 1 && homeSet && couldLolcalGpsSetHome(false)){
		setHomeByLocalGps(&trackerPosition,GPS_coord[LAT]/10,GPS_coord[LON]/10,GPS_altitude,true,false);
	}
}

bool couldLolcalGpsSetHome(bool setByUser){
	if(feature(FEATURE_DEBUG)){
		  return true;
	}
	return ((setByUser && GPS_numSat >= 4) || (!setByUser && GPS_numSat >= masterConfig.gps_min_sats)) && feature(FEATURE_GPS) && STATE(GPS_FIX);
}

void updateMFD(void){
	if(PROTOCOL(TP_MFD)){
		if (settingHome) {
			homeSet = true;
			settingHome = 0;
		}

		if (mfdTestMode || (homeSet && gotFix)) {
			targetPosition.distance = getDistance();
			targetPosition.alt = getTargetAlt(targetPosition.home_alt);
			targetPosition.heading = getAzimuth() * 10;
			gotFix = false;
		}

		if ((mfdTestMode || homeSet) && gotNewHeading) {
			getError();
			calculatePID();
			pwmWritePanServo(pwmPan);
			calcTilt();
			gotNewHeading = false;
		}
	}
}

void updateTracking(void){
	if(!PROTOCOL(TP_MFD) && !PROTOCOL(TP_CALIBRATING_MAG) && !PROTOCOL(TP_CALIBRATING_PAN0) && !PROTOCOL(TP_CALIBRATING_MAXPAN) && masterConfig.pan0_calibrated==1) {
		if(PROTOCOL(TP_SERVOTEST)) {
			homeSet = true;
			trackingStarted = true;
		} else {
			trackingStarted = (homeSet && ((targetPosition.distance >= masterConfig.start_tracking_distance) || \
					(targetPosition.distance < masterConfig.start_tracking_distance && (targetPosition.alt - trackerPosition.alt) >= masterConfig.start_tracking_altitude))) ;
		}

		if(trackingStarted)  {
			if(lostTelemetry == true && !cliMode){
				pwmWritePanServo(masterConfig.pan0);
				return;
			}
			if(!PROTOCOL(TP_SERVOTEST))
				calcTilt();

			if (gotNewHeading) {
				  getError();
				  calculatePID();
				  pwmWritePanServo(pwmPan);
				  gotNewHeading = false;
			}

			if(!(OFFSET_TRIM_STATE == TRIM_STATE_DISABLED_BY_USER))
				OFFSET_TRIM_STATE = TRIM_STATE_ENABLED;

		} else {
			OFFSET_TRIM_STATE = TRIM_STATE_DISABLED;
			if(homeSet && !cliMode )
				pwmWritePanServo(masterConfig.pan0);
		}
	}
}

void enterMenuMode(void){
	saveLastTilt(false);
	menuState = MENU_ROOT;
	indexMenuOption=0;
	displayShowFixedPage(PAGE_MENU);
}
void exitMenuMode(void) {
	menuState = MENU_IDEL;
	displayResetPageCycling();
	displayEnablePageCycling();
}

void processMenuRoot(void){
	menuOption = indexMenuOption % (OP_EXIT+1);
	if(menuOption == OP_EXIT) {
		writeEEPROM();
		systemReset();
	} else if(menuOption == OP_VBAT)
		menuState = MENU_BATTERY;
	else if(menuOption == OP_CALIBRATE)
		processMenuCalibrate();
	else if(menuOption == OP_GPS)
		menuState = MENU_GPS;
	else if(menuOption == OP_EPS)
		menuState = MENU_EPS;
	else if(menuOption == OP_EASING)
		menuState = MENU_EASING;
	else if(menuOption == OP_TELEMETRY)
		menuState = MENU_TELEMETRY;
	else
		menuState = MENU_ROOT;
}

void processMenuBattery(void){
	processMenuFeature(FEATURE_VBAT);
}

void processMenuGPS(void){
	processMenuFeature(FEATURE_GPS);
}

void processMenuEPS(void){
	/*processMenuFeature(FEATURE_EPS);*/
		menuOption = indexMenuOption % (OP_EPS_EXIT+1);
	if(menuOption == OP_EPS_EXIT){
		menuState = MENU_ROOT;
		indexMenuOption = OP_EXIT;
		return;
	}
	if(menuOption == OP_EPS_SAVE) {
		writeEEPROM();
		systemReset();
	} else if(menuOption == OP_MODE)
		menuState = MENU_EPS_MODE;
	else if(menuOption == OP_DISTANCEGAIN)
		menuState = MENU_EPS_DISTANCEGAIN;
	else if(menuOption == OP_FREQUENCY)
		menuState = MENU_EPS_FREQUENCY;
	else
		menuState = MENU_EPS;
}

void processMenuEPSMode(void){
	menuOption = indexMenuOption % (OP_EPS_MODE_EXIT+1);
	if(menuOption == OP_EPS_MODE_EXIT)
		menuState = MENU_EPS;
	else if(menuOption >= OP_DISABLED && menuOption <= OP_MODE3){
		masterConfig.eps = menuOption;
		setEpsMode();
		if(masterConfig.eps == OP_DISABLED)
			featureClear(FEATURE_EPS);
		else
			featureSet(FEATURE_EPS);
		menuState = MENU_EPS;
	} else
		menuState = MENU_EPS;
	indexMenuOption = OP_EPS_SAVE;
}

void processMenuEPSIncreasDecreaseParamValue(uint16_t *param){
	menuOption = indexMenuOption % (OP_INCREASEDECREASE_EXIT+1);
	if(menuOption == OP_INCREASEDECREASE_EXIT)
		menuState = MENU_EPS;
	else {
		switch(menuOption){
		case OP_INCREASE:
			*param +=10;
			break;
		case OP_DECREASE:
			*param -=10;
			break;
		}
		if(*param < 1)
			*param = 1;
		if(*param > 1000)
			*param = 1000;
		updateEPSParams();
		//menuState = MENU_EPS;
	}
	indexMenuOption = menuOption; //OP_EPS_SAVE;
}

void processMenuEASING(void){
	processMenuFeature(FEATURE_EASING);
}

void processMenuFeature(uint16_t featureIndex){
	menuOption = indexMenuOption % (OP_ENABLEDISABLE_EXIT+1);
	if(menuOption == OP_ENABLE)
		featureSet(featureIndex);
	else if(menuOption == OP_DISABLE)
		featureClear(featureIndex);
	menuState=MENU_ROOT;
	indexMenuOption = OP_EXIT;
}

void processMenuCalibrate(void){
	exitMenuMode();
	pwmPan0 = masterConfig.pan0;
	pwmPanCalibrationPulse = masterConfig.pan_calibration_pulse;
	ENABLE_STATE(CALIBRATE_MAG);
}

void processMenuTelemetry(void){
	menuOption = indexMenuOption % (OP_TELMETRY_EXIT+1);
	if(menuOption == OP_TELMETRY_EXIT){
		menuState = MENU_ROOT;
		indexMenuOption = OP_EXIT;
		return;
	}
	if(menuOption == OP_TELMETRY_SAVE) {
		writeEEPROM();
		systemReset();
	} else if(menuOption == OP_PROTOCOL)
		menuState = MENU_TELEMETRY_PROTOCOL;
	else if(menuOption == OP_BAUDRATE)
		menuState = MENU_TELEMETRY_BAUDRATE;
	else
		menuState = MENU_TELEMETRY;
}

void processMenuTelemetryProtocol(void){
	menuOption = indexMenuOption % (OP_TELEMETRY_PROTOCOL_EXIT+1);
	if(menuOption == OP_TELEMETRY_PROTOCOL_EXIT)
		menuState = MENU_TELEMETRY;
	else if(menuOption == OP_AUTODETECT){
		featureSet(FEATURE_AUTODETECT);
		menuState = MENU_TELEMETRY;
	} else {
		featureClear(FEATURE_AUTODETECT);
		masterConfig.telemetry_protocol = (1 << ( 2 + menuOption));
		menuState = MENU_TELEMETRY;
	}
	indexMenuOption = OP_TELMETRY_SAVE;
}

void processMenuTelemetryBaudrate(void){
	menuOption = indexMenuOption % (OP_TELEMETRY_BAUDRATE_EXIT+1);
	if(menuOption == OP_TELEMETRY_BAUDRATE_EXIT)
		menuState = MENU_TELEMETRY;
	else {
		masterConfig.serialConfig.portConfigs[0].msp_baudrateIndex = menuOption;//(1 << 2+menuOption);
		menuState = MENU_TELEMETRY;
	}
	indexMenuOption = OP_TELMETRY_SAVE;
}

void proccessMenu(uint8_t menuButton) {

	if(menuButton == MENU_BUTTON) {
		indexMenuOption++;
		showMainMenuPage();
	} else if(menuButton == HOME_BUTTON) {
		// root
		if(menuState == MENU_ROOT) {
			processMenuRoot();
		} else if(menuState == MENU_BATTERY) {
			processMenuBattery();
		} else if(menuState == MENU_GPS) {
			processMenuGPS();
		} else if(menuState == MENU_EPS) {
			processMenuEPS();
		} else if(menuState == MENU_EPS_MODE) {
			processMenuEPSMode();
		} else if(menuState == MENU_EPS_DISTANCEGAIN) {
			processMenuEPSIncreasDecreaseParamValue(&masterConfig.eps_gain.distance);
		} else if(menuState == MENU_EPS_FREQUENCY) {
			processMenuEPSIncreasDecreaseParamValue(&masterConfig.eps_frequency);
		}else if(menuState == MENU_EASING) {
			processMenuEASING();
		// Telemetry
		} else if(menuState == MENU_TELEMETRY) {
			processMenuTelemetry();
		// Telemetry Protocol
		} else if(menuState == MENU_TELEMETRY_PROTOCOL) {
			processMenuTelemetryProtocol();
		// Telemetry Baudrate
		} else if (menuState == MENU_TELEMETRY_BAUDRATE) {
			processMenuTelemetryBaudrate();
		}
		if(menuState != MENU_EPS_DISTANCEGAIN && menuState != MENU_EPS_FREQUENCY)
			indexMenuOption=0;
		displayShowFixedPage(PAGE_MENU);
	}

}

float map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void saveLastTilt(bool writteEeprom){
	if(feature(FEATURE_EASING)) {
		masterConfig.easing_last_tilt = _lastTilt;
		if(writteEeprom) writeEEPROM();
	}
}

void updateCalibratePan()
{
	uint16_t deltaHeading;
	float slope;
	// ENABLE CALIBRATING PAN0 PROCCESS
    if (STATE(CALIBRATE_PAN)) {
    	servoPanTimer = millis();
    	targetPosition.heading = 0;
        DISABLE_STATE(CALIBRATE_PAN);
        ENABLE_PROTOCOL(TP_CALIBRATING_PAN0);
        pwmPan = masterConfig.pan_calibration_pulse;
        pwmWriteServo(panServo, pwmPan);
        masterConfig.pan0_calibrated = 0;
        minPwmPan = 1500;
        maxPwmPan = 1500;
        maxDeltaHeading = 0;
        if(masterConfig.mag_calibrated == 0) {
                	ENABLE_STATE(CALIBRATE_MAG);
        }
        pwmPanState = FINDING_OUT_MIN_PWMPAN0;
        return;
     }

    //CALIBRATING PWMPAN0
    if(PROTOCOL(TP_CALIBRATING_PAN0) && !PROTOCOL(TP_CALIBRATING_MAG)){
    	if(masterConfig.pan0_calibrated == 0){
			// CALCULATING MIN AND MAX PAN0
			if(millis() - servoPanTimer > 100 && (pwmPanState == FINDING_OUT_MIN_PWMPAN0 || pwmPanState == FINDING_OUT_MAX_PWMPAN0)){
				trackerPosition.heading = getHeading();
				deltaHeading = calculateDeltaHeading(trackerPosition.heading,targetPosition.heading);
				if (deltaHeading > 2){
					// SERVO IS STILL MOVING
					targetPosition.heading = trackerPosition.heading;
					if(pwmPanState == FINDING_OUT_MIN_PWMPAN0) {
						pwmPan++;
						if(pwmPan > maxPwmPan + 100)
							pwmPan = minPwmPan - 100;
					} else if(pwmPanState == FINDING_OUT_MAX_PWMPAN0){
						pwmPan--;
						if(pwmPan < minPwmPan - 100)
							pwmPan = maxPwmPan + 100;
					}
					pwmWriteServo(panServo, pwmPan);
				} else {
					// SERVO SEEMS TO BE STOPPED
					if(pwmPanState == FINDING_OUT_MIN_PWMPAN0){
						pwmPanState = MIN_PWMPAN0_FOUND;
						minPwmPan = pwmPan;
						printf("min %d\n", minPwmPan);
					} else if(pwmPanState == FINDING_OUT_MAX_PWMPAN0){
						pwmPanState = MAX_PWMPAN0_FOUND;
						maxPwmPan = pwmPan;
						if(maxPwmPan < minPwmPan)
							maxPwmPan = minPwmPan;
						printf("max %d\n", maxPwmPan);
					}
				}
				servoPanTimer = millis();
			}
			//CHECK IF MIN AND MAX PAN0 HAS BEEN WELL CALIBRATED 3 SECONDS LATER
			if(pwmPanState == MIN_PWMPAN0_FOUND || pwmPanState == MAX_PWMPAN0_FOUND) {
				if(millis() - servoPanTimer > 3000){
					trackerPosition.heading = getHeading();
					// due to interference the magnetometer could oscillate while the servo is stopped
					deltaHeading = calculateDeltaHeading(trackerPosition.heading,targetPosition.heading);
					if (deltaHeading > 50){
						// SERVO IS STILL MOVING
						targetPosition.heading = trackerPosition.heading;
						if(pwmPanState == MIN_PWMPAN0_FOUND) {
							pwmPanState = FINDING_OUT_MIN_PWMPAN0;
							pwmPan = masterConfig.pan_calibration_pulse;
						} else if(pwmPanState == MAX_PWMPAN0_FOUND){
							pwmPanState = FINDING_OUT_MAX_PWMPAN0;
							pwmPan = 1500 + (1500 - masterConfig.pan_calibration_pulse);
						}
						pwmWriteServo(panServo, pwmPan);
					}
					else {
						// SERVO IS STOPED
						if(pwmPanState == MIN_PWMPAN0_FOUND){
							//targetPosition.heading = trackerPosition.heading;
							pwmPanState = FINDING_OUT_MAX_PWMPAN0;
							pwmPan = 1500 + (1500 - masterConfig.pan_calibration_pulse);
							pwmWriteServo(panServo, pwmPan);
						} else if(pwmPanState == MAX_PWMPAN0_FOUND) {
							// CALIBRATION FIHISHED WITH SUCCESS
							pwmPanState = PWMPAN0_CALCULATED_WITH_SUCCESS;
							masterConfig.mag_calibrated = 1;
							masterConfig.pan0_calibrated = 1;
							DISABLE_PROTOCOL(TP_CALIBRATING_PAN0);
							masterConfig.min_pan_speed = (uint16_t)(maxPwmPan - minPwmPan)/2.0f;
							masterConfig.pan0 = minPwmPan + masterConfig.min_pan_speed;
							printf("Calibration has finished with success:\n");
							printf("  set pan0=%d\n", masterConfig.pan0);
							printf("  set min_pan_speed=%d\n", masterConfig.min_pan_speed);
							printf("  set pan0_calibrated=%d\n", masterConfig.mag_calibrated);
							saveConfigAndNotify();
							// ACTIVATE MAX PWMPAN CALCULATION
							/*ENABLE_PROTOCOL(TP_CALIBRATING_MAXPAN);
							pwmPan = masterConfig.pan0 - 600;
							pwmWriteServo(panServo, pwmPan);*/
							trackerPosition.heading = getHeading();
							targetPosition.heading = trackerPosition.heading;
						}
					}
					servoPanTimer = millis();
				}

			}
    	}



    }


    /*// CALIBRATE MAX PAN
	if(PROTOCOL(TP_CALIBRATING_MAXPAN) && !PROTOCOL(TP_CALIBRATING_MAG)) {
		trackerPosition.heading = getHeading();
		deltaHeading = calculateDeltaHeading(trackerPosition.heading,targetPosition.heading);
		if(deltaHeading > 450){
			servoPanVarTime = millis() - servoPanTimer;
			slope = (servoPanVarTime/10.0f);
			if(slope > 0.0f)
				printf("%d, %d,%d\r\n", pwmPan,servoPanVarTime,deltaHeading);
			// Enviar nuevo pulso
			pwmPan += 10;
			if(pwmPan > masterConfig.pan0 + 600) {
				//Calcular m�ximo y finalizar
				masterConfig.max_pid_gain = maxPwmPan;
				pwmWriteServo(panServo, masterConfig.pan0);
				DISABLE_PROTOCOL(TP_CALIBRATING_MAXPAN);
				saveConfigAndNotify();
			} else {
				pwmWriteServo(panServo, pwmPan);
				trackerPosition.heading = getHeading();
				targetPosition.heading = trackerPosition.heading;
				servoPanTimer = millis();
			}
		}
	}*/
}

uint16_t calculateDeltaHeading(uint16_t heading1, uint16_t heading2){

	int32_t deltaHeading;

	deltaHeading = heading1 - heading2;

	if(heading1 < heading2)
		deltaHeading = deltaHeading + 3600;

	if (deltaHeading > 1800)
		deltaHeading -= 3600;
	else if (deltaHeading < -1800)
		deltaHeading += 3600;

	return (uint16_t) abs(deltaHeading);
}

void setEpsMode(void){
	EPS_MODE =  masterConfig.eps;
}

void updateEPSParams(){
	EPS_DISTANCE_GAIN = masterConfig.eps_gain.distance;
	EPS_FREQUENCY = masterConfig.eps_frequency;
}

void updateProtocolDetection(void){
	uint16_t protocol;

	if(!feature(FEATURE_AUTODETECT) || cliMode)
		return;

	protocol = getProtocol();

	if(protocol == 0 && !detection_title_updated ){
		detection_title_updated = true;
		updateDisplayProtocolTitle(protocol);
		return;
	}


	if(protocol == masterConfig.telemetry_protocol && isProtocolDetectionEnabled() && !lostTelemetry){
		showAutodetectingTitle(protocol);
		if(PROTOCOL(TP_MFD))
			settingHome = true;
		return;
	}

	if(protocol != masterConfig.telemetry_protocol && protocol > 0) {
		masterConfig.telemetry_protocol = protocol;
		protocolInit();
		trackingInit();
		if(PROTOCOL(TP_MFD))
			settingHome = true;
		updateDisplayProtocolTitle(protocol);
		detection_title_updated = false;
	}


}

void protocolInit(void){
	DISABLE_PROTOCOL(0b1111111111111);
	switch(masterConfig.telemetry_protocol) {
	  case TP_SERVOTEST:
		ENABLE_PROTOCOL(TP_SERVOTEST);
		break;
	  case TP_MFD:
		ENABLE_PROTOCOL(TP_MFD);
		featureClear(FEATURE_EPS);
		featureClear(FEATURE_GPS);
		mfdTestMode = false;
		break;
	  case TP_GPS_TELEMETRY:
		ENABLE_PROTOCOL(TP_GPS_TELEMETRY);
		break;
	  case TP_MAVLINK:
		ENABLE_PROTOCOL(TP_MAVLINK);
		break;
	  case TP_RVOSD:
		ENABLE_PROTOCOL(TP_RVOSD);
		break;
	  case TP_FRSKY_D:
		ENABLE_PROTOCOL(TP_FRSKY_D);
		telemetry_provider = masterConfig.telemetry_provider;
		break;
	  case TP_FRSKY_X:
		ENABLE_PROTOCOL(TP_FRSKY_X);
		telemetry_provider = masterConfig.telemetry_provider;
		break;
	  case TP_LTM:
		ENABLE_PROTOCOL(TP_LTM);
		break;
	  case TP_LTM_FRSKYD:
		ENABLE_PROTOCOL(TP_LTM_FRSKYD);
		break;
	  case TP_PITLAB:
		ENABLE_PROTOCOL(TP_PITLAB);
		break;
	}
}
