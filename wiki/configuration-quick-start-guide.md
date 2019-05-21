## QUICK START GUIDE

Once you have assembled your antenna tracker you have to do a initial setup in order to get it working. Try to apply these steps in same order to do it with success:

1.- **Enable OLED display** (Features --> Display).

2.- **Configure pan0** value (the stop pulse for pan servo).

You may use up/down arrows until pan servo stops. If it stops within a range, you must caluclate the midpoint: pan0 = ( A + B ) / 2 where A and B are the stop pulses between both extrems.

The above two steps may be performed in a single one by clicking on "Calibrate pan" button. This is also useful in order to set min_pan_speed parameter, which is the increment necesary to reach the limits of the stop pulse range, so the pan servo may start moving.

3.- **Calibrate Compasss** by clicking "Calibrate mag".

The antenna tracker will spin for 10 seconds in the same direction in order to perform the calibration. The direction of rotation depends on the value of calibration pulse parameter. This pulse is the same one used for servo pan calibration.

After configuring pan0 and calibrating the magnetometer, we have to be sure that the corresponding "Calibrated" checkboxes are activated, otherwise the tracker will not move. These checkboxes are useful if we want to test the telemetry but we still do not want to move the servos.

4.- **Configure OFFSET** parameter.

If when giving the command to point north the tracker remains some degrees out of phase, yo must set offset parameter with that value (whithin 0 to 360 degrees). You may adjust it with the up and down arrows as well.

5.- **Configure tilt parameters**, so that the servo tilt remains horizontal (tilt0) and vertical (tilt90).

6.- **Select protocol and baud rate** to which you want the tracker to work.

AUTODETECT feature may be activated in order to detect the protocol automatically. It is very useful if you have several aircrafts with different telemetry system.

The baud rate selected must be the same as the one you have configured for your bluetooth modules. Also remember that once you modify the baud rate, next time you want to connect from the configurator you have to to select same baud rate value from the drop-down list before clicking on "Connect" button.

7.- **Configure automatic home**.

In case of not using Local GPS, and avoiding to use buttons to set home position, configure the following parameters in the Telemetry section:

Min Sats: 6 minimum, the higher, if your GPS takes many satellites, the better.
Home: AUTO (From telemetry).

8.- **Configure start traking** parameters.

The default values for distance and altitude might be a bit high. Try 5 for distance and 2 for altitude. Remember that tracking will start when the aircraft has overcome that fence. Once it lands and returns within those limits, the tracking ends.

While the aircraft is within those limits, the menu and home buttons function as menu and home, so you may access the menu and change options, set and reset home, or change and set the data screen shown on the display. But when the aircraft is out of those limits, the tracking starts, and buttons now perform the function of offset trim, which acts on the parameter offset_trim (between -20 and 20 degrees). This value is displayed on the OLED (telemetry screen).

9.- **Configure PID**

By default, the antenna tracker has feature NOPID enabled. It works out of the box, and it is more than probability that it is not necessary to configure any of its parameters. With NOPID control enabled the tracking will experience a slight lag with respect the aircraft, especially if it receives position packets at a low frequency.

Yo can use NOPID control for simulations and first real flights. But if you want more precision you will have to properly configure the PID for pan servo. The will get into action once feature NOPID is disabled.
