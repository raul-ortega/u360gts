## Configuration of Serial Port 0 (UART1)

After the first boot, our controller can only work with serial port 0 (UART1) at 115200 baud, initially dedicated to input telemetry.

To change the baud rate of serial port 0 (UART1), execute the following command:

```
set telemetry_baud=value
```

In the parameters section, you'll find the values and their corresponding baud rates.

It's also possible to configure the port with the following command:

```
serial 0 1 baudrate baudrate baudrate baudrate
```

In this case, "baudrate" should be replaced with the desired value. The "serial" command must include all six parameters; none can be omitted. For example, if you want to use UART1 at 9600 baud:

```
set telemetry_baud=2
```

Or alternatively:

```
serial 0 1 9600 9600 9600 9600
```

The value "0" indicates serial port number 0 (UART1).
The value "1" indicates that it is dedicated to telemetry.
The first "9600" value indicates that telemetry will operate at 9600 baud.
The remaining values are not considered for telemetry but are necessary to successfully execute the command.

[<< Go back](README.md)
