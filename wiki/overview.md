# Features

* 360 degree continous rotation.
* Multiprotocol.
* Automatic protocol detection.
* Acceps up to 10 Hz of input telemetry frequency.
* Protocol conversion and fordwarding.
* Fully and easy configurable with [u360gts-configurator](https://github.com/raul-ortega/u360gts-configurator) (cross platform) and serial console (CLI mode).
* Tilt easing.
* Automatic home position (with local GPS).
* Detailled status data on OLED display.
* Battery monitoring.
* RSSI signal strength on OLED display.
* Setup menu on OLED display.
* Acurate PID control system.
* Position estimation system.
* Sound alarms through out a buzzer.
* 4 Serial ports, with dynamic assignment.


**360 DEGREE CONTINOUS ROTATION**

With this firmware you can move your antenna continually in a range of 360 degrees, without the need of moving back. Using an slipring and 360 degree servos, or normal servos modified to be able of doing it, the firware will give the orders for reaching the target in an acurate and fast way.

**MULTIPROTOCOL AND AUTOMATIC PROTOCOL DETECTION**

This firmware provides an **all in one antenna tracker controller system**, it is able to work with several telemetry protocols. You can configure your antenna tracker to detect automátically the telemetry protocol that your aircraft is streaming (if you configured the appropieated baud rate, otherwhise you have to select the apropiated baud rate in the setup menu on the OLED display once you are at the flying field). Now you have one antenna tracker system for all your aircrafts!

These are the protocols that are supported (and automatically detected):

- **MFD** 
- **GPS TELEMETRY (DIRECT NMEA from GPS)**
- **MAVLINK**
- **RVOSD**
- **FRSKY D**
- **FRSKY X (SmartPort)**
- **Ardupilot PASSTHROUGH Telemetry
- **LTM (Light Telemetry)**
- **PITLAB**

**INPUT TELEMETRY AT HIGH FREQUENCY**

This antenna tracker system is capable of reading input telemetry packets sended from the aircraft at a frequency up to 10 times per seconds.

**PROTOCOL CONVERSION AND FORWARDING**

With this firmware, you have the possibility to convert the input telemetry datato differents protocols formats, and fordward the frames to externals devices. 

- **MAVLINK** 
- **MFD**
- **NMEA**

Examples:

* Your aircrafat sends GPS direct telemetry frames to the antenna tracker, and it converts and send mavlink packets to Mission Planner or Droidplanner app.
* Your aircrafat sends GPS direct telemetry frames to the antenna tracker, and it converts and send NMEA GPGGA and GPRMC frames to Oruxmaps app.
* The received telemetry data is converted to MFD protocol to manage an MFD antenna tracker. 

**GRAPHICAL CONFIGURATOR**

Yo can configure and interact with the antenna tracker through the [u360gts-configurator](https://github.com/raul-ortega/u360gts-configurator), a software which will facilitate setting parameters, control operations, and test its behavior by simulation.

**COMMAND LINE INTERFACE**

Yo also can configure and interact with the antenna tracker through a Command Line Interface (CLI) from any serial console, as well as for example some app over Bluetooth.

**TILT EASING**

The tilt  movement has been improved by adding easing effects at the beginning and smoothing at the end. This will avoid damaging the tilt servo and other mechanisms when using heavy and larger antennas. This feature doesn't affect the accuracy and speed in the movements of the pan servo.

**AUTOMATIC HOME POSITION**

You can connect a NMEA or UBLOX GPS device to the controller to automatically set the home position before starting the tracking. Nonetheless, the user has full control and can reject the home position to get a more accurate one.

**OLED DISPLAY**

Telemetry data, local gps status, battery monitoring and other usefull information, are displayed on an OLED display.

Now you can select the ingoing telemetry protocol,baud rate, enable/disable features, and other parameter settings through the setup menu on the OLED display.

**RSSI SIGNAL STRENGTH ON DISPLAY**

The rssi signal can be provided from video or control receiver. The percentage of signal strength is shown on the display with a higer level of detail when the value is lower than a prefixed value by the user.

**POSITION ESTIMATION SYSTEM**

It implements a position estimation system which provides 3 estimation modes. It is very useful in case of telemetry at low frecuency.

**4 SERIAL PORTS**

These STM32 microprocessor series based boards incorporate 2 UARTS (serial ports 0 and 1). This firmware provides the way to activate 2 extra virtual ports (softserial) which can be used for different purposes. They can be assigned dynamically for the different functions the antenna tracker can perform.
