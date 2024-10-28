## PID Control

u360gts provides a PID control system in order to calculate the pwm pulses sent to pan servo. It makes possible the antenna tracker to point towards the target with precisión and speed.


**Disabling NOPID**

By default, after flashing the firmware, the  PID control do no get into action because NOPID control is enabled. NOPID control is less acurate and casuses some lag to reach the target, although position data packets are received at a good frequency.

For disabling NOPID control, in u360gts-configurator, uncheck NOPID feature checkbox, or execute these commands on CLI window:

```
feature -NOPID
save
```

**Configuring PID**

The default values of P, I and D, as well as the rest of the configuration parameters, are configured for a modified TowerPro MG996R servo to rotate 360º. These values will depend on many factors: resistors used for the mod, friction between pieces of mechanics, gear ratio, ...

If the tracker oscillates trying to reach the target, or it does not reach, or overcomes it, then PID values have to be tuned.

```
The technique consists in increasing P value up to the tracker slightly oscillates arround the target.

Once achieved, P value has to be lowered until oscillations desapear within a security margin.

Then, do increase I value until you see that it bounces on the target, but cushionning.

Finally, do increase the D to get it to react vividly, but always without oscillating around the target.
```

[<< Go back](README.md)
