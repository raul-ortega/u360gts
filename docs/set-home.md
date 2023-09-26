# Set Home Possition

There are different procedures to set the home position of your antenna tracker, depending if home button is used, a local GPS is present, home position from telemetry is configured or restore home position from last known feature is enabled.

1. Automatic home from Local GPS
1. Update Home Continuously From Local GPS
1. Manual home from Local GPS
1. Automatic home from Telemetry
1. Manual home from Telemetry
1. Reset Home
1. Restore from Last Known

## Automatic HOME from Local GPS

When a local GPS is present, **FEATURE GPS** is**enabled**, and the GPS receives signals from more than a predetermined number of satellites set in **gps_min_sats**, the HOME position is established without the need to press the HOME button.

Let's look at a practical example of automatic HOME establishment:

1. The tracker is in telemetry mode.

2. I observe the GPS page on the display.
   It's active, but it hasn't received satellite signals yet.

3. After a few seconds (or minutes), it displays a fix with 8 satellites
   and coordinates.

4. When switching to the telemetry page, the message appears
   "HOME SET <GPS>"

5. I increase the throttle of the simulator (or initiate the takeoff of our model aircraft).

6. The (virtual or real) aircraft moves away and exceeds the minimum distance.

7. Tracking begins.

## Update Home Continuously From Local GPS

When the aircraft is in flight and tracking is being performed, u360gts firmware allows the antenna tracker to be moved to a different location than when the home position was established. This functionality is activated by setting update_home_by_local_gps parameter. Ones enabled, the home possition will be continuosly reset using Local GPS position. This allows you to move on foot, or even in a vehicle carrying the antenna tracker.

## Manual Home With Local GPS

In previous firmware versions, when adding a local GPS, it was not possible to activate the HOME position manually because its presence triggered automatic activation. If for some reason the GPS didn't receive signals from a sufficient number of satellites, or the minimum value set through the configuration was not met, it was not possible to initiate tracking of the model aircraft. This issue has been resolved in recent versions by incorporating a new algorithm that allows the establishment of the home position both automatically and manually.

Let's imagine we have a LOCAL GPS, and the GPS doesn't lock onto the preconfigured number of satellites, let's say 8, because of poor satellite reception today. However, it manages to lock onto 5 or 6, which we might consider sufficient, and we don't want to wait for it to lock onto 8 or more, or because it might be impossible due to cloudy skies.

In a situation like the one described, it's possible to set the home position with a single press of the HOME button. With this button press, you accept the 5 or 6 satellites that the local GPS is receiving, and the position is fixed.

Here's an example of setting the HOME position:

1. The tracker is in telemetry mode, the local GPS doesn't lock onto satellites, and
   the message "HOME NOT SET" is displayed.

2. I inject telemetry from the simulator (or receive it from the model aircraft).

3. I observe the telemetry display, and it shows 11 satellites,
   which is greater than or equal to the minimum number.

4. I press the button once (a single press).

5. The display shows "HOME SET <AIRCRAFT>"

6. I increase the throttle of the simulator (or initiate the takeoff of our model aircraft).

7. The (virtual or real) aircraft moves away and exceeds the minimum distance.

8. Tracking begins.

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

## Resetting the HOME Position

To reset the HOME position, press and hold the HOME button for 3 seconds or more.

The process of establishing the HOME position will start again, allowing you to activate it manually or automatically.

## Restore Last Known Home Position

If enabled, the antenna tracker will store the home position in the non volatile memory once it has been established. If durin tracking a power reset ocurrs (e.g the users replaces the battery), the home position will be restored (HOME SET LAST will be shown on display) and tracking will continue normally without the need of landing.

This functionality may be enabled by setting **restore_last_home** parameter to ON. After enabled, home_lat, home_lon and home_alt provide 0 values by default. Once home position is set, either from telemetry, gps or by pressing the home button, the antenna tracker will store the home position values on those parameters. After a power reset, the antenna tracker will read the stored values and restore the home position.

**Note:** stored lat/lon/alt values may be changed if desired before flying from a different location, the user may press home button for more than 3 seconds in order to get a new home position from the incoming telemetry stream or from local gps.
