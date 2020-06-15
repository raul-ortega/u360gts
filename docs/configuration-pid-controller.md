## PID Control

El firmware u360gs utiliza un sistema de control PID para calcular el pulso que debe suministrar al servo PAN para

Los valores de P, I y D por defecto, al igual que el resto de parámetros de configuración, están ajustados para un servo TowerPro MG996R modificado para que gire 360º. Éstos valores van a depender de muchos factores: las resistencias que hayamos usado, la mecánica usada en el tracker, cuan ajustados estén todos los elementos, holguras, etc...

Si tu tracker oscila para alcancar el objetivo, se pasa del objetivo ligeramente, o no llega necesitas ajustar estos parámetros.

A continuación se muestra una explicación práctica de como ajustar los PIDs (gracais al  compañero Simba):

u360gts provides a PID control system in order to calculate the pwm pulses sent to pan servo. It makes possible that the antenna tracker to point towards the target with precisión and speed.


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