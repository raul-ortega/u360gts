## Compass

u360gts requires a compass sensor for tracking. When a compass is pressent, is calibrated and all configurator parameters which affects its behavior are properly configured, the antenna tracker will move automaticaly its heading to try to point to the gps position received from aircraft.


## Supported Sensors

In the moment of writing this document, u360gts supports HMC5883L and QMC5583L compass sensors. To be sure about what you are buying you must ask the seller, because he might be providing not supported chips while in the product description is specified the supported ones.

<!-- img src="img/supported_mag.jpg" width="525" /-->

u360gts firmware can detect the compass. To check if the compass of your board (or the external one) is supported, in CLI window of configurator type the command "status". The response will show HMC5883 or QMC5883 if detected.

<img src="img/CLI_status_mag.jpg" width="433" />


## Wiring

Take a look to [wiring schematics](install-wiring-schematics.md) to get more information about how to connect an external mag.


## Configuration

### Automatic Detection

#### NAZE Controller

When using a controller configured as NAZE, the system will attempt to detect magnetometers in the following priority order:

1. **QMC5883L:** If present, it is selected as the active sensor.

2. **HMC5883:** If the QMC5883L is not available, this sensor is selected if present.

3. **No sensor detected:** If none of the above sensors are present, the system will proceed without a MAG sensor.

#### SPRACINGF3 Controller

If the configured controller is SPRACINGF3, the system will try to detect the following magnetometers in this order:

1. **QMC5883L:** If present, it is selected as the active sensor.

2. **HMC5883:** If the QMC5883L is not available, this sensor is selected if present.

3. **AK8975:** If the two previous sensors are not available, this sensor is selected if present.

4. **No sensor detected:** If none of the sensors are present, the system will proceed without a MAG sensor.

#### BLUEPILL Controller

For a controller configured as BLUEPILL, the system follows a similar detection logic to the SPRACINGF3, with the following priority order:

1. **QMC5883L:** If present, it is selected as the active sensor.

2. **HMC5883:** If the QMC5883L is not available, this sensor is selected if present.

3. **AK8975:** If the two previous sensors are not available, this sensor is selected if present.

4. **No sensor detected:** If none of the sensors are present, the system will proceed without a MAG sensor.

### Manual Configuration

The system also allows manual configuration of the magnetometer type to be used via the `mag_hardware` value. The following describes what happens for each possible value:

- **`MAG_DEFAULT (0):`** The system follows the automatic detection logic described for each controller.

- **`MAG_NONE (1):`** The system is forced not to use any magnetometer, ignoring any present sensor.

- **`MAG_HMC5883 (2):`** The HMC5883 sensor is directly selected if present. If not available, the system will not use a magnetometer.

- **`MAG_AK8975 (3):`** The AK8975 sensor is directly selected if present. If not available, the system will not use a magnetometer.

- **`MAG_QMC5883L (4):`** The QMC5883L sensor is directly selected if present. If not available, the system will not use a magnetometer.

### External Magnetometer with Built-In Magnetometer

When using an external magnetometer with a controller that already has a built-in magnetometer, it is necessary to physically disable the built-in magnetometer by cutting the traces to prevent it from being detected. This is because the built-in magnetometer shares the same I2C address as the external one, which can cause conflicts.

### I2C Errors

If the magnetometer is not present/detected, it is possible that there are I2C errors. In this case, the errors will be displayed in the CLI (Command Line Interface). These errors indicate that there may be issues with the communication between the controller and the sensor, and should be addressed by checking the wiring and sensor connections.

### Calibration

u360gts locks pan servo movement if the compass is not pressent or if its not calibrated. You may calibrate the compass at home for the first time, but some times might be necesary to calibrate it again in the flight field.

Note: Before mag calibration you should [configure the pan servo](configuration-pan-servo.md).

To calibrate the compass from configurator go to Configuration -> Mag, and clic on Calibrate Mag button.

<img src="img/mag_configuration.jpg" width="527" />

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

When using external mag, you have to be sure that the magnetometer is aligned with the arrow of the board pointing to the front and it is not placed upside down. If it is rotated and/or flipped, then you have to configure the align_mag parameter with one of these values:

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

To change magnetic declination you have to set correct declination of your spesific location, which can be found [here](http://www.magnetic-declination.com).

If your magnetic declination readings are e.g. +3° 34' , the value entered in the u360gts configurator is 334. For west declination, use a minus value, e.g. for 1° 32' W, the value entered in the u360gts configurator is -132. In all cases (both CLI and GUI), the least significant digits are minutes, not decimal degrees.

[<< Go back](README.md)
