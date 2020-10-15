# USB Flashing

## Charging-Only Cables

If you see no signs of life on your host computer when you plug in your board, check your cable with your mobile phone or some other USB device - some charging cables have only the power pins connected. These will power up the board, so the leds light up, but the host computer will not react to the device at all. You need a proper USB cable to connect your board to the u360gts configurator.

## Platform Specific: Windows

u360gts configurator can have problems accessing USB devices on Windows. A solution could be to replace the ST driver with a libusb driver.The easiest way to do that is to download [Zadig](http://zadig.akeo.ie/). 
With the board connected and in bootloader mode: 
* Open Zadig
* Choose Options > List All Devices
* Select `STM32 BOOTLOADER` in the device list
* Choose `WinUSB (v6.x.x.x)` in the right hand box
![Zadig Driver Procedure](assets/images/zadig-dfu.png)
* Click Replace Driver
* Restart Chrome (make sure it is completely closed, logout and login if unsure)
* Now the DFU device should be seen by Configurator

## Platform Specific: Linux

Linux requires udev rules to allow write access to USB devices for users. An example shell command to acheive this on Ubuntu is shown here:
```
(echo '# DFU (Internal bootloader for STM32 MCUs)'
 echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0664", GROUP="plugdev"') | sudo tee /etc/udev/rules.d/45-stdfu-permissions.rules > /dev/null
```

This assigns the device to the plugdev group(a standard group in Ubuntu). To check that your account is in the plugdev group type `groups` in the shell and ensure plugdev is listed. If not you can add yourself as shown (replacing `<username>` with your username):
```
sudo usermod -a -G plugdev <username>
```

If you see your ttyUSB device disappear right after the board is connected, chances are that the ModemManager service (that handles network connectivity for you) thinks it is a GSM modem. If this happens, you can issue the following command to disable the service:
```
sudo systemctl stop ModemManager.service 
```

If your system lacks the systemctl command, use any equivalent command that works on your system to disable services. You can likely add your device ID to a blacklist configuration file to stop ModemManager from touching the device, if you need it for cellural networking, but that is beyond the scope of u360gts documentation.

If you see the ttyUSB device appear and immediately disappear from the list in u360gts configurator when you plug in your flight controller via USB, chances are that NetworkManager thinks your board is a GSM modem and hands it off to the ModemManager daemon as the flight controllers are not known to the blacklisted