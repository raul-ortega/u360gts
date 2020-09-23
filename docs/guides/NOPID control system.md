# Sistema de Control de Servo PAN sin PIDS

Se ha incluìdo de forma experimental un sistema de control del servo PAN que no usa PID. Si usas un servo lento, este sistema podrìa mejorar el seguimiento del tracker, realizando movimientos màs precisos y fluìdos. El sistema es màs intiutivo de configurar que el sistema PID tradicional.
El sistema realiza una correcciòn de forma proporcional del àngulo de error entre el heading del tracker y el heading del aeromodelo, mapeando dicho error sobre un rango de pulsos PWM para el servo en base a los siguientes paràmetros:

- **NO_PID_CONTROL:** Descomentando este paràmetro desactivamos el sistema PID tracicional y activamos el nuevo sistema de control PAN.
- **MIN_DELTA:** Angulo mìnimo en grados entre el heading del tracker y el del aeromodelo, si es mayor que este àngulo movemos el tracker.
- **MIN_PAN_SPEED:** Cantidad mìnima en milisegundos que hay que incrementar el pulso del PAN_0 para que se mueva. Este paràmetro es comùn a ambos sistemas de control y està localizado en el config.h donde siempre ha estado.
- **MAX_PAN_SPEED:** Cantidad màxima en milisegundos que hay que incrementar el pulso del PAN_0 para que se mueva.
- **MAP_ANGLE:** àngulo en grados a partir del cual se empieza a mapear el error del àngulo al gradiente de pulsos.