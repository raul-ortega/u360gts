# u360gts (amv-open360tracker 32bits)

u360gts is a complete DIY software and hardware antenna tracker solution for drones (UAVs, RPAs profesionals and FPV hobbyists). We tried to make it as universal as possible in terms of being compatible with the most popular F1/F3 flight controllers, radio and video systems. It is the perfect option if you need to track different aircrafts with different telemetry protocols. Firmware and hardware design have been developed by members of FPV community and are released for free under the GPLv3 license.

You are encouraged to read all the [documentation](docs/README.md) before using this firmware in your devices in order to prevent damages, unexpected behavior and misconfiguration.

## Hardware platform

This firmware has been developed for controllers based on STM32 F1 and F3 series microprocessors, such as the popular NAZE32 and its clones. By now it has been tested on other boards such as the Flip32 and a barebone Bluepill. Flight controllers **without** an onboard magnetometer are to be preferred.

# Features

* 360° continous rotation.
* Multiprotocol.
* Automatic protocol detection.
* Up to 10 Hz telemetry input.
* Protocol conversion and forwarding.
* Configurable via [u360gts-configurator](https://github.com/raul-ortega/u360gts-configurator) (cross platform) and serial console (CLI mode).
* Tilt easing.
* Automatic home position.
* Detailed status data on OLED display.
* Battery monitoring.
* RSSI signal strength on OLED display.
* Setup menu on OLED display.
* Accurate PID control system.
* Position estimation system.
* Sound alarms via buzzer.
* 4 Serial ports, with dynamic assignment.

Take a look at the [overview](docs/overview.md) to read more about the features listed above.

For more infos on the project refer to the [official website](http://www.u360gts.com/).

# Releases

Check out the [releases](https://github.com/raul-ortega/u360gts/releases) page.

# How To Get Help

- [Ask in u360gts facebook group](https://www.facebook.com/groups/u360gts/)
- [Post in the spanish community (español)](http://www.zonafpv.com/foro/estacion-de-tierra/u360gts-seguidor-de-antena-de-rotacion-continua-360o/)
- [Post in Jelle737's Build Log u360gts: 360° antenna tracker on rcgropus](https://www.rcgroups.com/forums/showthread.php?2964122-u360gts-360%C2%B0-antenna-tracker)
- [Submit a new issue](https://github.com/raul-ortega/u360gts/issues)