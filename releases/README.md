#Releases

## Development 8.0.0 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-8.0.0.zip).

### New features

* RSSI signal strength is shown on OLED display. It allows reading analogic signal up to 3.3 volts through RC_CH 2 (RC channel 2). This new feature has to be activated with the **feature RSSI_ADC** command. On the display is shown a bar with the percentage of signal. Parameter rssi_scale must be set to adjust the máximum voltage value to 100%. Parameter rssi_zoom is used to show more detail when the signal strength is lower than its value.

### Bugfixes

* Longitude minus sign not displayed when the aircraft was flying east of meridian 0.

## Stable 7.3.3 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-7.3.3.zip).

### Bugfixes

* After landing the aircraft the antenna tracker keeps spinning until it is powered off. Other times, when the aircarft flies over the vertical of the antenna tracker it stopped tracking. Both issues were caused because the altitude were not taked into acoount to detecet if the aircraft is whithin the tracking distance limits. In this version has been added the parameter start_tracking_altitude to control if the aircraft has landed. Its default value is set to 2 (2 meters). 

## Development 7.3.0 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-7.3.0.zip).

### New features

* The easing function Out Cubic has been added. This new function provides a smoother movement of the tilt servo. Read more information about the Ease Out Cubic function at [http://easings.net/](http://easings.net/)

## Development 7.2.2 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-7.2.2.zip).

### Bugfixes

* Default looptime has been set to 100 to solve the overrun on uart when receiving telemetry at frecuency higer than 2 Hz. It has been tested up to 10 Hz with GPS Telemetry and there are no errors in checksums. In order to get buttons working well, the parameter min_logic_level has been set to 60 by default.

## Development 7.2.1 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-7.2.1.zip).

### Bugfixes

* Solved the erratic movements when using Local GPS and update_home_by_local_gps is set to ON for continuous home position updating.

## Development 7.2.0 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-7.2.0.zip).

### New features

####EPS has now 3 modes

* Mode 1: it is the first implementation in version 5.0.: only param eps_distance_gain is needed. In this mode, every time a position data is received a estimation is calculated.

* Mode 2: it is the current EPS system, parameters eps_distance_gain and eps_frequency are mandatory. In this mode a estimation is calculated every time the clock reach the eps_frequency. No estimation is calculated when a position data is received.

* Mode 3 is a mix of mode 1 and mode 2: estimations are calculated every time the timer reach the eps_frequency. The timer is reset every time a new position data is received.

####EPS on menu display

A new menu for EPS has been implemented. It allows:

* Disabling EP feature.

* Setting the EPS mode.

* Changing the values or parameters eps_distance_gain and eps_frequency.

## Development 7.1.0 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-7.1.0.zip).

### New features

* Added parameter pan_calibration_pulse to set the pwm pulse to which the calibration starts.
* Default parameter values changed.

## Stable 7.0.1 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-7.0.1.zip).

### Bugfixes

* After landing the plane below the limit of the starting tracking distance the trakcer coninued spining.

* Latitude and Longitude data are discarded from GPRMC frames when using GPS_TELEMETRY protocol to prevent errors when estimating positions.

### New features
	
* NEMA GNGGA (GLONNAS GGA from M8M devices) frames supported in Local GPS, as well as in GPS Telemetry protocol.

* EPS system accuracy improved with the introduction an interpolation function. Now the antenna tracker can estimate new positions based on more positions than the two latest, taking into account heading and speed variations.

* A position filtering based on speed has been introduced. It discards potentially erroneous positions (very userf with FrSky D protocol) when the maximum speed limit is exceeded.

* calibrate pan command introduced. It automates the search of pan0 and min_pan_speed values.

* New param update_home_by_local_gps added to enable/disable the coninuous reseting of the HOME position when using Local GPS.

* New data field FCS (Failed Checksum) on display for GPS_TELEMETRY protocol. It counts the number of bad checksums.

## Stable 6.0.8 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-6.0.8.zip)
