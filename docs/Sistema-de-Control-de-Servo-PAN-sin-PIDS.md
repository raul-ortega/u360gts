# Sistema de Control de Servo PAN sin PIDS

Se ha incluído de forma experimental un sistema de control del servo PAN que no usa PID. Si usas un servo lento, este sistema podría mejorar el seguimiento del tracker, realizando movimientos más precisos y fluídos. El sistema es más intiutivo de configurar que el sistema PID tradicional.
El sistema realiza una corrección de forma proporcional del ángulo de error entre el heading del tracker y el heading del aeromodelo, mapeando dicho error sobre un rango de pulsos PWM para el servo en base a los siguientes parámetros:

- **NO_PID_CONTROL:** Descomentando este parámetro desactivamos el sistema PID tracicional y activamos el nuevo sistema de control PAN.
- **MIN_DELTA:** Angulo mínimo en grados entre el heading del tracker y el del aeromodelo, si es mayor que este ángulo movemos el tracker.
- **MIN_PAN_SPEED:** Cantidad mínima en milisegundos que hay que incrementar el pulso del PAN_0 para que se mueva. Este parámetro es común a ambos sistemas de control y está localizado en el config.h donde siempre ha estado.
- **MAX_PAN_SPEED:** Cantidad máxima en milisegundos que hay que incrementar el pulso del PAN_0 para que se mueva.
- **MAP_ANGLE:** Ángulo en grados a partir del cual se empieza a mapear el error del ángulo al gradiente de pulsos.