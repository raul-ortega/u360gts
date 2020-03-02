## Compass

u360gts requires a compass sensor for tracking. When a compass is pressent, is calibrated and all configurator parameters which affects its behavior are properly configured, the antenna tracker will move automaticaly its heading to try to point to the gps position received from aircraft.


## Supported Sensors

In the moment of writting this document, u360gts supports only HMC5883L compass sensor. Supported boards with built in mag are HMC5883L. But if you need to install an external compass, you must select products with the chip marked as L883. Avoid buying sensors with chip marked as AD because they are not supported yet. To be sure about what you are buying you must ask the seller, because he might be providing not supported chips while in the product description is specified HMC5883L.

<img src="https://github.com/raul-ortega/u360gts/blob/master/wiki/img/supported_mag.jpg" width="525" />

u360gts firmware can detect the compass. To check if the compass of your board (or the external one) is the supported one, in CLI window of configurator type the command "status". The response will show HMC5883 if it is detected.

<img src="https://github.com/raul-ortega/u360gts/blob/master/wiki/img/CLI_status_mag.jpg" width="433" />


## Wiring

Take a look to [wiring schematics](https://github.com/raul-ortega/u360gts/blob/master/wiki/install-wiring-schematics.md) to get more information about how to connect an external mag.


## Configuration

### Calibration

u360gts locks pan servo movement if the compass is not pressent or if its not calibrated. You may calibrate the compass at home for the first time, but some times might be necesary to calibrate it again in the flight field.

Note: Before mag calibration you should [configure the pan servo](https://github.com/raul-ortega/u360gts/blob/master/wiki/configuration-pan-servo.md).

To calibrate the compass from configurator go to Configuration -> Mag, and clic on Calibrate Mag button.

<img src="https://github.com/raul-ortega/u360gts/blob/master/wiki/img/mag_configuration.jpg" width="527" />

The controller will send the calibration pulse to the servo, and it will start spinning. During 10 seconds the controller will retrieve data from the sensor and will calculate magzero x, y and z values. Then the controller sends the stop pulse to the servo, it will stop spinning and calibration will be finished.

If you are using configurator version 4.0 or higher, than the "Calibration done" toggle button should be shown as on. If you are using an older version of the configurator, then check the box "calibrated".

**Note:** When calibration pulse is lower than central stop pulse, the tracker spins **counter clock wise** for the calibration process. And when calibration pulse is greater than stop pulse, the tracker spins clock wise. **If the tracker does not spin as described during compass calibration process**, then your **pan servop is reversed**. You may solve it executing these commands from CLI:

```
set pan_inverted=on
save
```

Alternatively you may do a hardware mod which consist on inverting the polarity of the wires directly connected to the motor (do the mod under your own risk).**

### Offset

When entering in configuration mode, the tracker tries to point to 0 degrees (north). If it does not, but you configured pan servo and and calibrated the mag, you will need to adjust offset parameter.

If you mounted the controller with the magnetometer aligned to north, you may not need to touch this parameter, unless you want to adjust it even better.

But if you mounted the controller in a different position, because you needed to place the micro usb connector to the side of the enclosure, then you need to configure this parameter.

e.g set offset to 90 if the board/mag is rotated 90 degrees.

### External Mag Alignment

When using external mag, you have to be sure that the magnetometer is aligned with the arrow of the board pointing to the front and it is not placed upside down. If it is rotated and/or flipped, then you have to configure the align_mag parameter with one of this values:

DEFAULT
CW0
CW90
CW180
CW270
CW0FLIP
CW90FLIP
CW180FLIP
CW270FLIP


### Magnetic Declination

To change magnetic declination you have to set correct declination of your spesific location, which can be found [here](www.magnetic-declination.com).

If your magnetic declination readings are e.g. +3° 34' , the value entered in the u360gts configurator is 334. For west declination, use a minus value, e.g. for 1° 32' W, the value entered in the u360gts configurator is -132. In all cases (both CLI and GUI), the least significant digits are minutes, not decimal degrees.

[<< Go back](https://github.com/raul-ortega/u360gts/blob/master/wiki/index.md)
