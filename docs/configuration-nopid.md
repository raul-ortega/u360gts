## No-PID Servo PAN Control System

The u360gts firmware also includes a servo PAN control system that does not use PID (which was used in the 8-bit version). This system is more intuitive to configure than the traditional PID system. If you're using a slow servo, this system can improve tracker tracking by performing more precise and fluid movements. Even when using fast servos, this system also allows for configuration.

### Operation

The system proportionally corrects the difference between the tracker's heading and the model aircraft's heading, mapping this difference to a range of PWM pulses for the servo based on the configuration parameter values. When there's a large angle to cover between the tracker's heading and the aircraft's heading, the tracker moves quickly, and when it's smaller, it moves more slowly. This movement is always proportional, preventing oscillations as the destination is never exceeded.

### Activating NOPID

To activate this feature, you can use the command:

```
feature nopid
```

To deactivate the feature:

```
faeture -nopid
```

### Configuration Parameters

Below are the parameters exclusively used to configure the servo PAN control system:

- `nopid_min_delta`: This value is the minimum angle in degrees between the tracker's heading and the model aircraft's heading. If the difference is greater than this value, the tracker moves. If it's equal to or less than this value, the tracker stops.

- `nopid_max_speed`: The maximum amount in milliseconds by which the pan0 pulse must be increased to make it move. A higher value results in faster movements, while a lower value results in slower movements.

- `nopid_map_angle`: The pulse sent to the PAN servo is calculated as a mapping function between 0 and the value (in degrees) indicated in this parameter. If the difference between the tracker's heading and the model aircraft's heading is equal to or greater than this angle, the pulse is increased by the value indicated in `nopid_max_speed`. If the difference is less, then the increment value is calculated as a mapping function, where the minimum increment is 0, and the maximum is `nopid_max_speed`. A higher value for this parameter results in slower and smoother movement, while a lower value results in faster and more abrupt movements.

In addition, please take note of the following parameter:

- `min_pan_speed`: The minimum amount in milliseconds by which the pan0 pulse must be increased to make the servo move.

Note: The `min_pan_speed` parameter is common to both control systems and serves the same function.

[<< Go back](README.md)
