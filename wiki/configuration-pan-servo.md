## Automatic Pan Servo Calibration

For automatic pan servo calibration process, the magnetometer sensor is required, and also a value for calibration pulse, which should be lower than the spected stop pulse. By default calibration pulse is 1400 because we spect a value arround 1500 for the central stop pulse. If you suspect that you have a central stop pulse different than the default one, you may change it to a more properly value. If your servo has a high dynamic range, then try set the calibration pulse not too far away from the suspected central stop pulse, in this way your servo will spin slower but during less time than with a  further calibration pulse value (passing through the whole dynamic range is not needed for this calibration process).

In some cases interference on the magnetometer may lead to a long time being required to detect that the servo has stopped, due to the precision used to detect it. If this is the case, try calibration away from external electromagnetic fields. Also try to place the magnetometer as far away as possible from metal objects, such as screws, servos, ..., and especially power cables.

If the process take too much time and you want to interrupt, you need to cut power and click on disconnect button.

### First step, mag calibration:

The controller sends the calibration pulse to the servo, which starts spinning counter clock wise. Durig 10 seconds the controller retrieves data from the sensor, and calculates magzero x, y and z values. Then the controller sends the stop pulse to the servo, and if its value has been properly configured previously, it stops spinning. However, if the stop pulse was no properly configured previously, the tracker continues spinning. In both cases, after 10 seconds, the mag calibration process has finished, and the controller jumps to second step. 

Note: When calibration pulse is lower than central stop pulse, the tracker spins counter clock wise for the calibration process. And when calibration pulse is greater than stop pulse, the tracker spins clock wise. If the tracker does not spin as described during compass calibration process, then your pan servop is reversed, and you need to solve it by a hardware mod, which consist on inverting the polarity of the wires directly connected to the motor (do the mod under your own risk).

### Second step, central stop pulse (pan0) calibration:

The controller sends again the calibration pulse, and the tracker starts or continues spinning, depending on previous step. In each algorithm cycle the value of the pulse is decreased, and as the value gets closer to the central stop pulse servo speed decreases.

At the same time the controller retrieves heading angles values and calculates how small the difference between consecutive values is. When this difference is equal or less than a specific threshold (0.2 degrees) the controller assumes that it has stopped spinning, and stores the last sent pulse as the lower value within a range of stop pulses. Now, the controller waits 3 seconds, and measure again heading value. If a difference greater than 5 degrees is detected then the controller assumes that the tracker is still in movement, and sends again the calibration pulse. The process is repeated as necessary until the lower stop pulse value is calculated (e.g. min_pan0 = 1511 usec)

Once the lower value for the stop pulse range has been calculated, the controller sends a calibration pulse higher than the spected central stop pulse. The valuee of the new calibration pulse is calculated as follows:

**calibration pulse = central stop pulse + (central stop pulse - calibration pulse) = 1500 + (1500 - 1400) = 1600 usec**

This time the tracker starts spinning clock wise. Again, as the value gets closer to the central stop pulse servo speed decreases. If the controller detects that the servo has stoped (heading difference of 0.2 degrees), it stores the last sent pulse as the higher value for the range of stop pulses. If after 3 secods a movement is detected (headding difference greater than 5 degrees) the controller sends again calibration pulse. The process is repeated as necessary until the higher stop pulse value is calculated (e.g. max_pan0 = 1527 usec).

Now that we have the value of both limits of the range, the central stop pulse (pan0) is calculated as follows:

**pan0 = min_pan 0 + (max_pan0 - min_pan0) / 2 = 1511 + (1527 - 1511) / 2 = 1511 + 8 = 1519 usec**

During the process values for central stop pulse are automatically updated several times, and entries are shown on log panel.

### Third step, minimun increment of speed to start movement (min_pan_speed):

Now the algorithm calculates the value of min_pan_speed parameter as follows:

**min_pan_speed = (1527 - 1511) / 2 = 8 usec**

and the configurator updates its value.

### Final step

Finally the tracker has sttoped spinning, and some beeps confirms that the automatic pan servo calibration process has finished.

## Manual Pan Servo Calibration
	
Calibration pulse is not an stop pulse (min or max), it is only a pulse we need to start movement for the calibration, it has nothing to do with calculations. If you think that the pan0 (stop pulse) value might be arround 1500, we set calibration pulse as 1400. This calibration pulse is used only during automatic calibration proccess.

During automatic calibration, the tracker will try to figure out the min_pan0 pulse (pulse lower than pan0 where the tracker stops) and max_pulse (pulse higher than pan0 where the tracker stops). This is because the servo stops movement whithing a range of pulses.

For a better precission you may caluculate pan0 and min_pan_speed manualy. You should figure out the min_pan0 and max_pan0 pulses using the up/down arrows of stop pulse field of the GUI, and then do the calculations bellow.

e.g.:

	min_pan0 = 1495 
	max_pan0 = 1530

Lets calculate the stop pulse:

	pan0 = min_pan 0 + (max_pan0 - min_pan0) / 2

	pan0 = 1495 + (1530 - 1495) / 2 = 1512.5

	Rounded pan0 = 1513

And finally the minimun increment to start moving:

	min_pan_speed = (max_pan0 - pan0) / 2 

	min_pan_speed = (1530 - 1513) / 2 = 8.5

	Rounded min_pan_speed = 9
