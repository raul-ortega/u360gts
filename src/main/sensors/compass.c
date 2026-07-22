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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/printf.h"
#include "common/maths.h"

#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS 1
// Compile nanoprintf in this translation unit.
#define NANOPRINTF_IMPLEMENTATION
#include "common/nanoprintf.h"

#include "drivers/sensor.h"
#include "drivers/compass.h"
#include "drivers/compass_hmc5883l.h"
#include "drivers/compass_qmc5883l.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/pwm_output.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/compass.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "tracker/config.h"
#include "tracker/servos.h"

#include "io/display.h"

#ifdef NAZE
#include "hardware_revision.h"
#endif

//Soft Iron - 2x2 mAtrice		Hard Iron
//Soft Iron - 2x2 matrice		Hard Iron
//0.985875331	-0.1674808396	-0.0825
//0.200471249	1.1800732518	0.1140
//#define TM_11  0.985875331f
//#define TM_12  -0.1674808396f
//#define TM_21  0.200471249f
//#define TM_22  1.1800732518f
//#define TM_11  0.512116f
//#define TM_12  -1.110875f
//#define TM_21  0.908144f
//#define TM_22  0.418656f
#define COMPASS_UPDATE_FREQUENCY_10HZ 100000
#define MAG_SAMPLES 32              // 32 samples in array
#define MAG_SAMPLES_F 32.0f         // 32 samples in array
#define ADC_SAMPLE_P1 5   		    // 5 adc samples to initial point 1 
#define MAG_ANGLE_THRESHOLD 9.0f    // Mathematically should be 11 ( 360 degrees/MAG_SAMPLES )
#define MAG_DISTANCE_THRESHOLD 10.0f // Ensures sufficient distance for accurate angle calculations
#define N_DECIMAL_POINTS_PRECISION (1000000)   // n = 3. Three decimal points for OLED printing

mag_t mag;                   // mag access functions

extern uint32_t currentTime; // FIXME dependency on global variable, pass it in instead.
extern uint8_t  pwmPanPin;
extern uint16_t pwmPan0;
extern uint16_t pwmPanCalibrationPulse;
extern uint8_t  cliMode;

//extern float    Rollt;  // in flight/imu.h
//extern float    Pitcht;  // in flight/imu.h

int16_t magADC[XYZ_AXIS_COUNT];
sensor_align_e magAlign = 0;   // use DEFAULT board alignment

// globals to pass to display module
uint16_t calib_rotation = 0;
float	 radius=0;
float 	 eValScale = 0;
float    phase = 0;
float 	 sd1=0;
float    sd2=0;

float    softIron[2][2] = {	{1.0f, 0.0f},
							{0.0f, 1.0f} 
							};

#ifdef MAG
static uint8_t  magInit = 0;


// process data
struct magProcessing {
  int16_t rawX;
  int16_t rawY;
  // at origin
  float magX;
  float magY;
  float evb11;
  float evb12;
  float evb21;
  float evb22;
  // distance
  float dist;
};

// store points 
struct magADCPoint {
   int x[3];
   int y[3];  
};


void compassInit(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    LED1_ON;
    mag.init();
    LED1_OFF;
    magInit = 1;
}

void updateCompass(flightDynamicsTrims_t *magZero)
{
    static uint32_t nextUpdateAt, notifyUpdateAt = 0, tCal = 0;
    static uint32_t axis;
	
    static flightDynamicsTrims_t magZeroTempMin;
    static flightDynamicsTrims_t magZeroTempMax;

	static char  	buf[100];	// for npf_snprintf()
    static uint16_t evSample;
	static uint32_t adc_sample; 
	
	static struct   magProcessing mp[MAG_SAMPLES];
    static struct   magADCPoint point;
	static float 	sum_vec_angle =0;	
	
	static float    vec_angle_p01=0;
	static float    vec_angle_p12=0;
	static float    delta_angle=0;
    static float    dist=0;

	
	//static uint8_t  evSet=0;   // EigenValues set indicator
	//static float    Xhd; // temp hack
	//static float    Yhd;

    if(detectedSensors[SENSOR_INDEX_MAG] != MAG_NONE && magInit)
	{
		// lock at update frequency
		if ((int32_t)(currentTime - nextUpdateAt) < 0)
			return;
		// HMC5883 runs at 15Htz continous mode, QMC at 200Htz continous mode
		nextUpdateAt = currentTime + COMPASS_UPDATE_FREQUENCY_10HZ;
		
		// populate magADC
		if (!mag.read(magADC))
		{
			// haven't seen this happen
			printf("mag.read() i2c failed\n");
			return;
		}
		// forced to default alignment, need to fix magAlign to read masterconfig
		alignSensors(magADC, magADC, magAlign); 
		
		// calibrate mag triggered
		if (STATE(CALIBRATE_MAG)) {
			
			tCal = nextUpdateAt;
						
			for (axis = 0; axis < 3; axis++) {
				magZero->raw[axis] = 0;
				magZeroTempMin.raw[axis] = magADC[axis];
				magZeroTempMax.raw[axis] = magADC[axis];
			}
			
			notifyUpdateAt = 0;
			calib_rotation=0;
			adc_sample = 0;
			evSample = 0;
			sum_vec_angle = 0;
			//evSet = 0;		// clear eigen values state
			softIron[0][0] = 1.0f;  
			softIron[0][1] = 0.0f;
			softIron[1][0] = 0.0f;
			softIron[1][1] = 1.0f;
			
			DISABLE_STATE(CALIBRATE_MAG);	
			ENABLE_PROTOCOL(TP_CALIBRATING_MAG);
			displayShowFixedPage(PAGE_CALIBRATING_MAG);
			//displayDisablePageCycling(); // PAGE_STATE_FLAG_CYCLE_ENABLED off, covered in the displayShowFixedPage() function call
			//displayResetPageCycling();   // cycle index to page 1, repeated later on
			// trigger rotation
			pwmWriteServo(pwmPanPin, pwmPanCalibrationPulse);
		} // end STATE(CALIBRATE_MAG)

		// clear calibration notification screen
		if (notifyUpdateAt != 0 && currentTime > notifyUpdateAt)
		{
			// display
			if(cliMode) {
			    displayShowFixedPage(PAGE_CLI_MODE);
			}
			  else {
				displayResetPageCycling();   // page 1 - telemetry
			    displayEnablePageCycling();  // enable page cycle
				notifyUpdateAt = 0;
			 }
		}
		
		// regular compass read - apply calibration adjustments
		if (0 == tCal) {
			
			// centre
			magADC[X] -= magZero->raw[X];
			magADC[Y] -= magZero->raw[Y];
			magADC[Z] -= magZero->raw[Z];
			// print transformed compass X,Y
			//printf("x2=%d,y2=%d\n", (int)((float)magADC[X]*softIron[0][0]+(float)magADC[Y]*softIron[1][0]), (int)((float)magADC[Y]*softIron[1][1]+(float)magADC[X]*softIron[0][1]) );
			// print current Xh,Yh tilt (from imu.c as a hack)
			//npf_snprintf(buf, sizeof(buf),"%f",Rollt);
			//printf("Rollt=%s,", buf);
			//npf_snprintf(buf, sizeof(buf),"%f",Pitcht);
			//printf("Pitcht=%s\n", buf);
			
			// apply transformation matrix
			magADC[X] = (int)((float)magADC[X]*softIron[0][0]+(float)magADC[Y]*softIron[1][0]);
			magADC[Y] = (int)((float)magADC[Y]*softIron[1][1]+(float)magADC[X]*softIron[0][1]);
		} // end normal read
		
		// 
		// calibration process
		//
		if (tCal != 0) {
			
			// maximum 15 seconds, or mag samples buffer full, or rotated more than 380 degrees
			if ( ((nextUpdateAt - tCal) < 15000000) && evSample < MAG_SAMPLES && sum_vec_angle < 380.0f ) 
			{
            
			//printf("sample %i taken with nextSampleAt=%i\n", adc_sample, nextSampleAt);
            //printf("raw x1=%d y1=%d\n", magADC[0], magADC[1]);

            // prime (P0)
            if ( 0 == adc_sample ) {   
			   point.x[0] = magADC[0];
               point.y[0] = magADC[1];
			   // printf("Got P0\n");
            }

            // Prime (P1), 5 samples = 0.5 seconds
            if ( ADC_SAMPLE_P1 == adc_sample )  {
                 point.x[1] = magADC[0];
                 point.y[1] = magADC[1];
                 vec_angle_p01 = atan2f( point.y[1] - point.y[0], point.x[1] - point.x[0]); 
              
                 if ( vec_angle_p01 < 0) 
                 {
                   vec_angle_p01 += 2.0f * M_PIf; // add 360
                 }
                 vec_angle_p01 *= (180.0f/M_PIf);
                 //printf("Got P1, angle between p0 and p1 = %f\n", vec_angle_p01);
            }
			
            // P2 .. Pn 
            if ( adc_sample > ADC_SAMPLE_P1 )
            {
                point.x[2] = magADC[0];
                point.y[2] = magADC[1];  
                vec_angle_p12 = atan2f( point.y[2] - point.y[1], point.x[2] - point.x[1] ); 
              
                if ( vec_angle_p12 < 0) 
                {
                     vec_angle_p12  += M_PIf * 2.0f; // add 2PI
                }
                vec_angle_p12 *= (180.0f/M_PIf);

				// print every Mag Reading
                printf("ts=%u, p2x=%d, p2y=%d, p1x=%d, p1y=%d,", currentTime, point.x[2], point.y[2], point.x[1], point.y[1]);
                //printf("Angle between p1 and p2=%f,", vec_angle_p12 );
                npf_snprintf(buf, sizeof(buf), "%f",vec_angle_p12);
				printf(" vap12=%s,",buf); // vector angle12
				
				dist = sqrtf( powf(point.x[2]-point.x[1],2) + powf(point.y[2]-point.y[1],2) );
                npf_snprintf(buf, sizeof(buf), "%f",dist);
				printf(" dist=%s,", buf); // distance between p1 and p2
				
				//printf(" dist=%f\n", dist );
                // could use - atan2(sin(x-y), cos(x-y))
                delta_angle = 180.0f - fabsf(fmodf(fabsf(vec_angle_p01 - vec_angle_p12), 360.0f) - 180.0f);
				//printf("Delta Angle=%f,", delta_angle );
                npf_snprintf(buf, sizeof(buf),"%f",delta_angle);
				printf(" da=%s\n", buf); // delta angle
				//printf(" using Vector Angle p01 =%f\n", vec_angle_p01);
              
                // greater than threshold angle and greater than a threshold distance to avoid noise
                if ( delta_angle  > MAG_ANGLE_THRESHOLD && dist > MAG_DISTANCE_THRESHOLD )
                {
					// start to accrue angle after sample zero
                    if ( evSample > 0 ) {
					sum_vec_angle += delta_angle;
					}
                    //printf("  Triggered > n degrees with\n");
                    //printf("  Vector Angle p01 =%f\n", vec_angle_p01);
                    //printf("  Vector Angle p12 =%f\n", vec_angle_p12);
					//printf("  Sum Vector Angle=%f\n", sum_vec_angle);
                    //printf("  Sample %i\n", adc_sample);
                    
                    // update stack
                    point.x[0] = point.x[1];
                    point.y[0] = point.y[1];
                    point.x[1] = point.x[2];
                    point.y[1] = point.y[2];  
                    // update reference angle
                    vec_angle_p01 = vec_angle_p12;
                  
                    // load post processing structure with raw ADC data
                    mp[evSample].rawX = point.x[2];
                    mp[evSample].rawY = point.y[2];
                    
					// print sample capture
				    printf("evSample=%d,", evSample);
					npf_snprintf(buf, sizeof(buf), "%f",sum_vec_angle);
				    printf("sum_vec_angle=%s\n", buf);

					evSample++;
                }
            } 
          
            // LED0_TOGGLE;
			for (axis = 0; axis < 3; axis++) {
			  if (magADC[axis] < magZeroTempMin.raw[axis])
			      magZeroTempMin.raw[axis] = magADC[axis];
			  if (magADC[axis] > magZeroTempMax.raw[axis])
				     magZeroTempMax.raw[axis] = magADC[axis];  
			}
				adc_sample++;
	     	} // end sample capture loop
			else 
			{
				// calibration calculations
				tCal = 0;
				
				// set magZero (basic calibration)
				for (axis = 0; axis < 3; axis++) {
					magZero->raw[axis] = (magZeroTempMin.raw[axis] + magZeroTempMax.raw[axis]) / 2; 
					printf("magZeroTempMin.raw[%u]=%d\n",axis,magZeroTempMin.raw[axis]);
					printf("magZeroTempMax.raw[%u]=%d\n",axis,magZeroTempMax.raw[axis]);
					printf("MagZero[%u]=%d\n",axis,magZero->raw[axis]);
				}
				
				// buffer filled over at least 360 degrees
				//if ( sum_vec_angle >= 360.0f && sum_vec_angle <= 400.0f && (MAG_SAMPLES == evSample) )	
			    // at least 340 degrees of rotation
			    if ( sum_vec_angle >= 340.0f )
				{
				// advanced calibration
				float sum_evb11 = 0;
				float sum_evb12 = 0;
				float sum_evb21 = 0;
				float sum_evb22 = 0;
				float trB = 0;
				float detB = 0;
				float eValX = 0;
				float eValY = 0;
				// float eValScale = 0; // moved to globals
				float eVecX = 0;
				float eVecY = 0;
				float primary = 0;
				float secondary = 0;
				float phase1 = 0;
				float phase2 = 0;
				//float phase = 0; // moved to globals
				uint16_t samples = evSample;
				
				evSample=0;				
				while ( evSample < samples)
				{ 
				  // adjust offsets
				  mp[evSample].magX = (mp[evSample].rawX - magZero->raw[X]) / 1000.0f;  // milliGaus
				  mp[evSample].magY = (mp[evSample].rawY - magZero->raw[Y]) / 1000.0f;

				  //npf_snprintf( buf, sizeof(buf),"evSample=%d,magX=%f,magY=%f", evSample, mp[evSample].magX , mp[evSample].magY  );
                  //printf("%s\n", buf);
				  
				  // matrix values
				  mp[evSample].evb11 = mp[evSample].magX * mp[evSample].magX;
				  mp[evSample].evb12 = mp[evSample].magX * mp[evSample].magY;
				  mp[evSample].evb21 = mp[evSample].magX * mp[evSample].magY;
				  mp[evSample].evb22 = mp[evSample].magY * mp[evSample].magY;

				  npf_snprintf( buf, sizeof(buf),"evSample=%d,evb11=%f,evb12=%f,ev21=%f,ev22=%f", evSample, mp[evSample].evb11, mp[evSample].evb12, mp[evSample].evb21, mp[evSample].evb22 );
                  printf("%s\n", buf);

				  // aggregate as we go
				  sum_evb11 += mp[evSample].magX * mp[evSample].magX;
				  sum_evb12 += mp[evSample].magX * mp[evSample].magY;
				  sum_evb21 += mp[evSample].magX * mp[evSample].magY;
				  sum_evb22 += mp[evSample].magY * mp[evSample].magY;

				  mp[evSample].dist = 0.0f;
				  /*if ( sample > 0)
				  {
					  mp[sample].dist = sqrtf( powf(mp[sample].magX - mp[sample-1].magX,2) + powf(mp[sample].magY - mp[sample-1].magY,2));
				  }
						printf("dist=%f\n" ,mp[sample].dist ); */
				  evSample++;
				}
				
				  // Co-variance matrix
				  // divide by number of samples
				  sum_evb11 /= (float)samples; //MAG_SAMPLES_F;
				  sum_evb12 /= (float)samples; //MAG_SAMPLES_F;MAG_SAMPLES_F;
				  sum_evb21 /= (float)samples; //MAG_SAMPLES_F;MAG_SAMPLES_F;
				  sum_evb22 /= (float)samples; //MAG_SAMPLES_F;MAG_SAMPLES_F;
				
				  // Trace of Matrix A -(x^2 + y^2)
				  trB = -(sum_evb11+sum_evb22);
				  // Determinant of Matrix A - x^2*y^2 - x*y*x*y
				  detB = sum_evb11*sum_evb22 - sum_evb21*sum_evb12 ;
				  // Eigen Values gain (lamba1, lamba2)
				  eValX = (-trB + sqrtf( powf(trB,2) - 4*detB ))/2.0f;
				  eValY = (-trB - sqrtf( powf(trB,2) - 4*detB ))/2.0f;
				  // Aspect ratio
				  eValScale = sqrtf(eValX/eValY);
				  // Normalized Eigen Vectors of unit length 1 (rotation)
				  eVecX = sum_evb12 / (eValX - sum_evb11);
				  eVecY = sum_evb12 / (eValY - sum_evb11);
				  				  
				   // Calculations
				  npf_snprintf(buf, sizeof(buf), "%f",sum_evb11);
				  printf("EVB11=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f",sum_evb12);
				  printf("EVB12=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f",sum_evb21);
				  printf("EVB21=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f",sum_evb22);
				  printf("EVB22=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f",trB);
				  printf("TR(B)=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f",detB);
				  printf("DET(B)=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f",eValX);
				  printf("EVAL(X)=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f",eValY);
				  printf("EVAL(Y)=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f",eValScale);
				  printf("EVSCALE=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f",eVecX);
				  printf("EVECX=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f",eVecY);
				  printf("EVECY=%s\n", buf);
				  
				  // 0 to 180 degrees - x-axis length - length of primary axis
				  // 90 to 270 degrees - y axis length - length of secondary axis
                  // was previoulsy
 				  // primary  = sqrtf(powf(mp[0].magX-mp[14].magX,2) + powf(mp[0].magY-mp[14].magY,2));
                  // secondary= sqrtf(powf(mp[7].magX-mp[22].magX,2) + powf(mp[7].magY-mp[22].magY,2));

                  // singular radius values, roots of the eigen values (points on circumference of a circle)
                  primary = sqrtf(2.0f*eValX);
				  secondary = sqrtf(2.0f*eValY);

				  if (signbit(eValX - sum_evb11) != signbit(sum_evb12)) 
				  { 
					phase1 = atan2f(1.0f,eVecX)*180.0f/ M_PIf -180.0f; }
				  else
				  { 
					phase1 = atan2f(1.0f,eVecX)*180.0f/ M_PIf; }

				  if (signbit(eValY - sum_evb11) != signbit(sum_evb12)) 
				  { 
					phase2 = atan2f(1.0f,eVecY)*180.0f/ M_PIf -180.0f; }
				  else
				  { 
					phase2 = atan2f(1.0f,eVecY)*180.0f/ M_PIf; }

				  if ( primary > secondary )
					{ phase = phase1; }
					 else
					{ phase = phase2; }
		  
				  //printf("primary=%f\n",primary);
				  npf_snprintf(buf, sizeof(buf), "%f",primary);
				  printf("primary=%s\n", buf);
				  
				  //printf("secondary=%f\n",secondary);
				  npf_snprintf(buf, sizeof(buf), "%f",secondary);
				  printf("secondary=%s\n", buf);
				  
				  //printf("phase=%f\n",phase);
				  npf_snprintf(buf, sizeof(buf), "%f", phase1);
				  printf("phase1=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f", phase2);
				  printf("phase2=%s\n", buf);
				  
				  npf_snprintf(buf, sizeof(buf), "%f", phase);
				  printf("phase=%s\n", buf);
				  
				  // calculate soft iron correction matrix, 
				  // scale factor depends on phase
				  // rotate anticlockwise
				  // [cos,sin]
				  // [-sin,cos ]
				  // scale
				  // [S,0]
				  // [0,S]
				  
				  // calculate soft iron correction matrix
				  if ( primary > secondary )
				  { 
					// rotation, scaling in the y direction
					softIron[0][0] = cosf(phase*M_PIf/180.0f);  
					softIron[0][1] = sinf(phase*M_PIf/180.0f);
					softIron[1][0] = -sinf(phase*M_PIf/180.0f)*eValScale;
					softIron[1][1] = cosf(phase*M_PIf/180.0f)*eValScale;
					radius=primary;
				  }
				   else 
				  {  
			        // rotation, scaling in the x direction
					softIron[0][0] = cosf(phase*M_PIf/180.0f)*eValScale; 
					softIron[0][1] = sinf(phase*M_PIf/180.0f)*eValScale;
					softIron[1][0] = -sinf(phase*M_PIf/180.0f);
					softIron[1][1] = cosf(phase*M_PIf/180.0f);
					radius=secondary;
				  }
				   
				  //printf("softironA00=%f  ",softIron[0][0]);
				  npf_snprintf(buf, sizeof(buf), "%f",softIron[0][0]);
				  printf("softIron[0][0]=%s, ", buf);
				  
				  //printf("softironA01=%f\n",softIron[0][1]);
				  npf_snprintf(buf, sizeof(buf), "%f",softIron[0][1]);
				  printf("softIron[0][1]=%s\n", buf);
				  
				  //printf("softironA10=%f  ",softIron[1][0]);
				  npf_snprintf(buf, sizeof(buf), "%f",softIron[1][0]);
				  printf("softIron[1][0]=%s, ", buf);
				  
				  //printf("softironA11=%f  ",softIron[1][1]);
				  npf_snprintf(buf, sizeof(buf), "%f",softIron[1][1]);
				  printf("softIron[1][1]=%s\n", buf);
				
				  // calculate before and after standard deviations
				  // back test the correction matrix
				  npf_snprintf(buf, sizeof(buf),"%f",radius);
				  printf("circle radius=%s\n", buf);
				  
				  float sum_diff1=0;
				  float sum_diff2=0;
				  evSample=0;
				  while ( evSample < samples )
					  {
						sum_diff1 += powf( sqrtf( powf(mp[evSample].magX,2) + powf(mp[evSample].magY,2) ) - radius, 2);
						sum_diff2 += powf( sqrtf( powf(mp[evSample].magX*softIron[0][0] + mp[evSample].magY*softIron[1][0],2) + 
										          powf(mp[evSample].magY*softIron[1][1] + mp[evSample].magX*softIron[0][1],2) ) - radius, 2);
						// dump data
						npf_snprintf( buf, sizeof(buf),"%f,%f,%f,%f", mp[evSample].magX, mp[evSample].magY, 
						                                              mp[evSample].magX*softIron[0][0] + mp[evSample].magY*softIron[1][0],
																	  mp[evSample].magY*softIron[1][1] + mp[evSample].magX*softIron[0][1] );
						printf("%s\n", buf);									   
								
						evSample++;
					  }
				  // standard deviations
				  sd1 = sqrtf(sum_diff1/((float)samples-1));  //(MAG_SAMPLES-1));
				  sd2 = sqrtf(sum_diff2/((float)samples-1));  //(MAG_SAMPLES-1));
				  // log standard deviaton before and after
				  printf("Raw STD=%d Corrected STD=%d\n",(int)roundf(sd1*1000),(int)roundf(sd2*1000));				  
					  	  		  
				  // 10 second hold time
				  notifyUpdateAt = currentTime + 10000000;
				  displayShowFixedPage(PAGE_CALIBRATING_MAG_SUCCESS);
				  DISABLE_PROTOCOL(TP_CALIBRATING_MAG);
				  // reset hardware
				  pwmWriteServo(pwmPanPin, pwmPan0);	
				  // write to eeprom
				  saveConfigAndNotify();
				}
				// failed advance calibration step
				else {
					printf("Failed to rotated 340 degress within time period\n");
					// 4 second hold time
					notifyUpdateAt = currentTime + 4000000;
					calib_rotation=(uint16_t)sum_vec_angle;
					displayShowFixedPage(PAGE_CALIBRATING_MAG_FAILED);			
					DISABLE_PROTOCOL(TP_CALIBRATING_MAG);
					// reset hardware
				    pwmWriteServo(pwmPanPin, pwmPan0);
				    saveConfigAndNotify();
				}
			} // end calibration calculations
		}
    }
}
#endif
