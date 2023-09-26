## Automatic Home from Telemetry

Thanks to this new feature local GPS is needed no more for auto tracking (unless you are planning to pilot your aircraft from a mobile platform like a car or a ship, in that case local GPS is mandatory becaus the tracker home position have to be updated continiously), and no button have to be pressed in order to set home position from telemetry ().

In order to use this feature you have to disable local GPS and set AUTO mode to telemetry_home parameter:

```
feature -GSP
set telemetry_home=AUTO
```

Note: If you want to set home manually, leave telemetry_home parameter value as DEFAULT.

You also have to set telemetry_min_sats parameter with the desired number of sats as the condition necessary for home position to be set:

```
set telemetry_min_sats=6
```

Please, be aware that the tracker will never know which is the home position that the autopilot has set for the aircraft because no frame from the telemetry stream that might carry it on is processed.

The procedure to set home automatically is described bellow:

0. Tracker and Aircraft have no power.
1. User powers the aircraft.
2. Aircraft set its own home position.
3. User checks information displayed on his ground control software or OSD data.
4. User thiks that he has a good and stable gps signal and decide that it is a good moment to power on the tracker system.
5. The tracker receives telemetry data from the aircraft and if the "min number of sats" condition is overcome it will use the next lat/lon data arrived as home position.
