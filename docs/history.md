## History

u360gts has been developed and maintained by users of the [FPV spanish community](http://www.aeromodelismovirtual.com/showthread.php?t=34530).

This project was born as **amv-open360tracker**, forked from the original project open360tracker developed by Samuel Brucksch. The firmware was originally designed to be run on Atmega328p and and Atmega2560 (8-bit AVR Microcontrollers used with Arduino) and was intended to be flashed on the popular flight controllers Crius SE, as well as Arduino UNO, Nano and Mega using external IMU. The main idea was to build a DIY antenna tracker system by reusing those old flight controllers and the electronics devices we had left inside that box...

Users were requesting more and more functions, protocols... the code was growing and growing, and those microcontrollers had a big limitation on memory and hardware. So Raúl Ortega, the main contibutor of the firmware, decided to migrate to 32 bit processors while keeping in mind the same idea of reusing old electronics, thus creating **amv-open360tracker-32bits**. The firmware had been developed for controllers based on STM32F series microprocessors, which fit the technical specifications of the popular NAZE32 flight controllers (Flip32, NAZE32 rev5/rev6...).

Later on Raúl Ortega decided to create a graphical user interface to configure the tracker and thought that it would be good to merge both projects under the name **u360gts** (u360gts for the firmware and u360gts-configurator for the configuration tool), and created a website as showcase and starting point for new users: [http://www.u360gts.com](http://www.u360gts.com).

[<< Go back](README.md)
