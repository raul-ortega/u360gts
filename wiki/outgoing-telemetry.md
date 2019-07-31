# Outgoing Telemetry: translating packets and relaying

This antenna tracker system incorporates a funtionality which allows the translation of received telemetry packets to a different protocol and relay them packets throughout a differnet serial port to feed other devices, like ground stations or mobile applications.

Some examples:

* The aircraft sends MAVLINK telemetry to the tracker, and the tracker translates it to NMEA and is relayed through bluetooth to Oruxmaps mobile app on Android devices.
* The aircraft sends SmartPort telemetry to the tracker, and the tracker translates it to MAVLINK and is relayed through bluetooth to Mission Planner.

Those are only two examples, other combinations and applications are possible.

In order to use this funtionality telemetry feature has to be enabled:

```
feature telemetry

save

```

If your board has no free uarts because and it supports softserial, you want to enable it:

```
feature softserial

save
```

And finally, you have to configure the communication parameters:

```
serial port_number protocol_number baudrate1 baudrate2 baudrate3 baudrate4

save
```

This command is similar to the serial command used to configure the GPS and the ingoing telemetry, the differences are that you have to specify a port number, a protocol number and the baudrate. The value indicated in baudrate3 is the value that will be take into account  for the outgoing telemetry, the other ones doesn't matter their values, you can set all them with the same value.

Note: As a general rule only the proccesed packets for tracking pursposes may be translated: GPS position data, altitude and number of satellites. For MFD is sent Dinstance, Altitude and  Azimut. Some other data might be sent depending of the ingoing telemetry protocol used and particular settings. As an exaple, for NMEA are exported the course and the ground speed. For mavlink, attitude data is also relayed Pith, Roll y Yaw).

**Supported protocols**

Those are the numbers of the supported outgoing protocols.

* 256 for MFD
* 512 for MAVLINK
* 1024 for NMEA
* 2048 for LTM

**Examples**

- LTM 9600 bauds on softserial 1:
```
serial 30 2048 9600 9600 9600 9600
```
- MAVLINK 57600 bauds on uart2:
```
serial 1 512 57600 57600 57600 57600
```


-MAVLINK 57600 bauds:

**Supported External Applications**

These applications have been tested:

* Mission Planner (mavlink)
* Droid Planner / Tower (mavlink)
* Oruxmpas (NMEA)
* EZ-GUI (LTME)

[<< Go back](https://github.com/raul-ortega/u360gts/blob/master/wiki/index.md)
