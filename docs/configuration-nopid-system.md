## NOPID System

u360gts firmware provides by default a system to control the movement of pan servo, which is more user friendly to configure than PID control systems.

**NOPID** system adds corrections in a proportional way to the difference between the aircraft and tracker headings, mapping the difference on a range of PWM pulses
 taking into account the velues of its configuration parameters.
 
For big angles the tracker moves fast, and in the same way, and for small angles the movement is slower, but always it moves proportionally, avoiding oscillations because the target never is overcome.

**Configuration Parameters**

- **nopid_min_delta**: The min angle (degrees) between tracker and aircraft heading. If the difference is greater than this value then the servo moves the tracker. f it is lower or equal then the servo stops.
    
- **nopid_max_speed**: Maximun increment (microseconds) of pan0 pulse in order to start moving. The higher its value the faster it moves. The lower its value, the movement will be slower.
    
- **nopid_map_angle**: The pulse sent to the pan servo is calculated as a mapping function between 0 and the value (degrees) indicated in this parameter. If the difference between the heading of the tracker and heading of the aircraft is equal to or greater than this angle, the pulse is increased by the value indicated in nopid_max_speed. If the difference is smaller, then the increment is calculated as a mapping function, where the minimum increment is 0 and the maximum is nopid_max_speed. The higher the value of this parameter, the slower and smoother the movement will be. The lower the value, the faster and sharper the movements will be.

Also has to be taken into account the following parameter:

- **min_pan_speed**: Minimum increment (microseconds) of pan0 pulse in order to start the movement.

[<< Go back](README.md)
