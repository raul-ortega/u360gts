# Features

**360 DEGREE CONTINOUS ROTATION**

With this firmware you can move your antenna continuously in a range of 360°, without the need of moving back. Using a slipring and 360 degree servos, your antenna will be pointed at your aircraft in an accurate and fast way.

**MULTIPROTOCOL AND AUTOMATIC PROTOCOL DETECTION**

This firmware provides an **all in one antenna tracker controller system**; it is able to work with several telemetry protocols. You can configure your antenna tracker to automatically detect the telemetry protocol that your aircraft is streaming (on the premise that you've already selected the correct baud rate). Now you have a single antenna tracker system for all your aircrafts!

These are the protocols that are supported and automatically detected:

- **MFD** 
- **GPS TELEMETRY** (DIRECT NMEA from GPS)
- **MAVLINK**
- **RVOSD**
- **FRSKY D**
- **FRSKY X** (SmartPort)
- **Ardupilot** (Passthrough)
- **LTM**
- **PITLAB**

**INPUT TELEMETRY AT HIGH FREQUENCY**

This antenna tracker system is capable of reading input telemetry packets sent from your aircraft at a frequency of up to 10 times per seconds.

**PROTOCOL CONVERSION AND FORWARDING**

With this firmware you have the possibility to convert the input telemetry data to differents protocols formats, and forward the new frames to externals devices. 

Supported output protocols:

- **MAVLINK** 
- **MFD**
- **NMEA**

Examples:

* Your aircraft sends raw GPS telemetry frames to the antenna tracker, which converts them and sends MAVLink packets to a computer running Mission Planner or to a smartphone running the Droidplanner app.
* Your aircraft sends raw GPS telemetry frames to the antenna tracker, which converts them and sends NMEA GPGGA and GPRMC frames to Oruxmaps app.
* The received telemetry data is converted to the MFD protocol to manage an MFD antenna tracker.

**GRAPHICAL CONFIGURATOR**

Yo can configure and interact with the antenna tracker through the [u360gts-configurator](https://github.com/raul-ortega/u360gts-configurator), a software which will facilitate setting parameters, control operations, and test its behavior via simulations.

**COMMAND LINE INTERFACE**

You can also configure and interact with the antenna tracker through a Command Line Interface (CLI) from any serial console, including connecting directly via Bluetooth _(depends on your FC and the wiring of the components)_.

**TILT EASING**

The tilt movement has been improved by adding easing effects at the beginning and smoothing at the end. This will avoid damaging the tilt servo and other mechanisms when using heavy and larger antennas. This feature doesn't affect the accuracy and speed of the movements of the pan servo.

**AUTOMATIC HOME POSITION**

You can connect an NMEA or UBLOX GPS to the controller to automatically set the home position without your aircraft needing a valid GPS fix. It will still be possible to reset the home position in case you move the tracker around after powering it up.

**OLED DISPLAY AND BUTTONS**

An OLED display can be connected to show telemetry data, local gps status, battery monitoring and other useful informations. Using two buttons it will also be possible to graphically change some parameters without having to use a computer.

**RSSI SIGNAL STRENGTH ON DISPLAY**

The RSSI value of the signal can be provided from an external (video or R/C) receiver. The percentage of signal strength is shown on the display with a higer level of detail when the value is lower than a prefixed value by the user.

**POSITION ESTIMATION SYSTEM**

An integrated position estimation system can predict the immediate course of the aircraft; useful to avoid break ups in case of low-frequency telemetry.

**4 SERIAL PORTS**

Most of the supported STM32 boards incorporate 2 UARTS. This firmware provides a way to activate 2 extra virtual ports _(softserial)_ which can be used for various purposes. They can be assigned dynamically for the different functions the antenna tracker can perform.

[<< Go back](README.md)
