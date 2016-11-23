#Releases

## Development 7.2.0 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-7.2.0.zip).

### New features

####EPS has now 3 modes

* Mode 1: it is the first implementation in version 5.0.: only param eps_distance_gain is needed. In this mode, every time a position data is received a estimation is calculated.

* Mode 2: it is the current EPS system, parameters eps_distance_gain and eps_frequency are mandatory. In this mode a estimation is calculated every time the clock reach the eps_frequency. No estimation is calculated when a position data is received.

* Mode 3 is a mix of mode 1 and mode 2: estimations are calculated every time the timer reach the eps_frequency. The timer is reset every time a new position data is received.

####EPS on menú display

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
