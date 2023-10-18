## INCOMING TELEMETRY: PROTOCOLS

The u360gts firmware is an antenna tracker control system capable of decoding the most popular radio control telemetry protocols.

When you're out in the field, all you need to do is change the protocol and transmission speed (baud rate) through the configuration menu on the OLED display.

### Supported Protocols

- MFD
- GPS_TELEMETRY (direct NMEA from a GPS device)
- MAVLINK (v1)
- RVOSD
- FRSKY_D
- FRSKY_X (SMARTPORT)
- LTM (Light Telemetry)
- CROSSFIRE
- PITLAB


The default protocol when the system is first started is GPS TELEMETRY, with a baud rate of 115200. Our antenna tracker will be ready to receive direct telemetry in NMEA format, which can be provided by any compatible GPS module. In your aircraft, you can have a GPS module that sends data frames to the tracker through a telemetry serial link, such as those provided by OpenLRS systems.

You can select the desired protocol from the configuration menu mode on the display or from CLI mode.

[<< Go back](README.md)
