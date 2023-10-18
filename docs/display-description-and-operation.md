## DISPLAY

The u360gts firmware allows you to use OLED display devices on your antenna tracker to view information that helps you monitor the status of your model tracking, decide the best time to start your flight, check battery status, and more.

Additionally, you can interact with the antenna tracker for basic configuration and adjustment tasks by using the MENU and HOME buttons.

### Installation

Please refer to the connection diagram in the [wiring schematics](install-wiring-schematics.md) section to connect the OLED Display to the controller.

### Configuration

To activate the display, enter CLI mode and execute the command:

```
feature display
```

To deactivate it:

```
feature -display
```

### Operation

The display presents information on different pages, which can remain fixed or cycle through them:

- Home Page
- CLI Mode (configuration)
- Calibration Page
- Telemetry Page (input)
- Local GPS Status
- Battery Monitoring
- 
[<< Go back](README.md)
