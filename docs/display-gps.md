# Display: GPS screen

When FEATURE_GPS is enabled the display will show the GPS screen to show all information relative to the local GPS.

**Sat:** Numbers of satellites
**Fix:** "Y" indicates possition fixed, and "N" indicate no fix.
**Pos:** LAT/LON Position (degrees only).
**Spd:** GPS speed
**GC:** Ground Course
**RX:** Count of received packets
**ERRs:** Count of packets with errors
**Dt:** Time elapsed (milliseconds) until receiving new data
**TOs:** Count of timeouts.

And finally, it shows a line where each symbol represents the status of lays packets received:

**?:** Packet has errors
**!:** Packet has been ignored
**>:** Packet has been skipped
**g:** NMEA GGA packet received
**r:** NMEA RMC packet received
**O:** UBLOX SOL packet received
**S:** UBLOX STATUS packet received
**I:** UBLOX VSINFO packet received
**P:** UBLOX PSLLH packet received
**V:** UBLOX VELNED packet received

[<< Go back](README.md)
