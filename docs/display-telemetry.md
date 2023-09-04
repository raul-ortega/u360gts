# Display: Telemetry screen

The Telemetry screen will show information about telemtry packets recieved from the aircraft/drone.

- **Hz:** is the frequency at witch telemetry packets with data position are received. 0 means that frequency is under 1 Hz.
- **Sat:** number of satellites
- **FCS:** is the count of packets rejected because a bad CRC has been detected.
- **Lat:** Latitude
- **Lon:** Longitude
- **Alt:** Altitude from home possition
- **Dis:** Distance from home possition
- **H:** Tracker heading (Angle relative to north that the tracker is pointing to).
- **A:** Target heading (Angle relative to north that the tracker should be pointing to).
- **Of:** Offset trim value (min value -20º, max value 20º). If the user press the buttons during tracking the heading will be changed the heading will be modified as many degrees as this value indicates to try to get a better video signal.
- **HOME SET:** <GPS> indicates that the home position (tracker home position) is set by the local GPS. <AIRCRAFT> indicates that the home possition has been set from by the telemetry received from the aircraft/dron.

[<< Go back](README.md)
