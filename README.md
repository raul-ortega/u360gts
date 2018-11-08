# u360gts (amv-open360tracker 32bits)

This is the 32 bits version of the "continuous 360 degree rotating antenna tracker system" for FPV. This project has been developed and maintained by users of the [FPV spanish community](http://www.aeromodelismovirtual.com/showthread.php?t=34530).

Please, we encourage you to read all the documentation before using this firmware in your devices, otherwhise they could be dammaged. In the wiki you'll find detailled information about how to install and configure it with success.

## Hardware platform

This firmware has been developed for controllers based on STM32F series microprocessors, which fit the technical specifications of the popular NAZE32 flight controller. By now, it has been tested on the **Flip32** flight controller which incorporates the magnetometer, but it could work on other NAZE32 based boards like with external magnetometer.

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

Take a look to [wiki/overview.md](https://github.com/raul-ortega/u360gts/blob/master/wiki/overview.md) to read more about the features listed above.

[See more information about this project](http://www.u360gts.com/)

# u360gts Firmware Releases

[https://github.com/raul-ortega/u360gts/releases](https://github.com/raul-ortega/u360gts/releases)
