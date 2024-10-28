**Configuring Serial Port for Local GPS**

Once **feature GPS** is enabled, the u360gts will try to talk with GPS through the default uart port for it. u360gts allows to connect GPS device in any free uart or softserial port.

In order to asign a different port than default, the user should execute a serial command in CLI window:

```
serial port_number protocol_number baudrate1 baudrate2 baudrate3 baudrate4

save
```

For example, if we want to use uart2 (serial 1) at a baud rate of 9600.

  serial **1** **2** 115200 **9600** 115200 115200

  save

As shown, we use 1 for port_number, 2 for protocol_number, and 9600 for baudreate2 params.

<i>**Notes:**
  - Param protocol_number set to 2 means that serial 1 is used for GPS.</i>
  - Params baudrate1, baudrate3 and baudrate4 are not used for GPS configuration, its values are not taken into account.</i>
  - All parameters have to be set, but we only port_number, protocol_number and baudrate2 will be take into account by u360gts firware.
  
Once the serial port is asgined, the user may change baudrate from grafical user interfece by setting the dropdwon list box for Baud Rate.

[<< Go back](configuration-gps.md)
