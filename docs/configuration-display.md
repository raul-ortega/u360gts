## Configuring the OLED Display 

Please refer to the [display supported hardware](display-supported-hardware.md) section to get information about supported OLED display modules.

### Installation

Please refer to the connection diagram in the wiring schematics section to connect the OLED Display to the controller.
Configuration

### Enabling

To activate the display, enter CLI mode and execute the command:

```
feature display
```

To deactivate it:

```
feature -display
```

### Oled type:

To select the SSD1306 displayset oled_type parameter to 0 (default):

```
set oled_type=0
```
To select the SH1106 display set oled_type parameter to 1:

```
set oled_type=1
```

[<< Go back](README.md)
