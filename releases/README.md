#Releases

##Stable 7.0.0 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-7.0.0.zip).

###Bugfixes

-After landing the plane below the limit of the starting tracking distance the trakcer coninued spining.

-Latitude and Longitude data are discarded from GPRMC frames when using GPS_TELEMETRY protocol to prevent errors when estimating positions.
	
	###New features
	
    -NEMA GNGGA (GLONNAS GGA from M8M devices) frames supported in Local GPS, as well as in GPS Telemetry protocol.
	
	-EPS system accuracy improved with the introduction an interpolation function. Now the antenna tracker can estimate new positions based on more positions than the two latest, taking into account heading and speed variations.
	
    -A position filtering based on speed has been introduced. It discards potentially erroneous positions (very userf with FrSky D protocol) when the maximum speed limit is exceeded
	
    -calibrate pan command introduced. It automates the search of pan0 and min_pan_speed values.

	-New param update_home_by_local_gps added to enable/disable the coninuous reseting of the HOME position when using Local GPS.
	
	-New data field FCS (Failed Checksum) on display for GPS_TELEMETRY protocol. It counts the number of bad checksums.

##Stable 6.0.8 [DOWNLOAD](https://github.com/raul-ortega/u360gts/blob/master/releases/amv-open360tracker_NAZE-6.0.8.zip)
