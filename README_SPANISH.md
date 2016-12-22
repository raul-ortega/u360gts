# amv-open360tracker 32bits 

Ésta es la versión de 32 bits del firmware para el sistema universal de antena tracker con rotación contínua de 360º para drones. Este proyecto está desarrollado y mantenido por los miembros de la [comunidad española de FPV](http://www.aeromodelismovirtual.com/showthread.php?t=34530).

Por favor, te rogamos que leas con atención toda la información que suministramos, así como la de los componentes que utilices, para abordar con éxito el montaje, instalación y configuración de tu antena tracker, de lo contrario los dispositivos y elementos electrónicos y/o mecánicos podrían sufrir graves daños. También te pedimos que hagas un uso de tu antena tracker responsable y dentro de la legalidad vigente.

Este firmware es software libre, puede ser descargado y utilizado de forma gratuita, y redistribuído, con una única restricción: cualquier versión del firmware debe ser distribuída en los mismos términos y condiciones de uso gratuíto y distribución. Los autores se resevan todos los demás derechos sobre este software. Se distrubuye tal cual es, úsalo bajo tu propio riesgo y responsabilidad.

## Plataforma hardware

Este firmware ha sido desarrollado para su utilización controladoras basadas microprocesadores de la serie STM32F, cuyas especificaciones técnicas encajan con la popular controladora de vuelo NAZE32 para drones. En la actualidad, este firmware ha sido testeado en las controladoras Flip32 y NAZE32, ambas con magnetómetro incorporado, pero podría ser utilizada en otras controladoras basadas en NAZE32 con magentómetro externo (aún no testado).

# Características

* 360 grados de rotación contínua.
* Soporta múltiples protocolos de telemetría.
* Realiza conversión de protocolo y reenvío a aplicaciones externas.
* Completamente configurable desde [u360gts-configurator](https://github.com/raul-ortega/u360gts-configurator) (cross platform) y consola serie (modo CLI).
* Efecto de amortiguación para servo tilt.
* Establecimiento automático de la posición home con GPS local.
* Información detallada del estado en display OLED.
* Menú de configuración en display OLED.
* Sistema de control PID para movimientos precisos del servo PAN. 
* Hasta 4 puertos serie ( 2 en 8 bits), con asignación dinámica (32 bits). 

**ROTACIÓN CONTÍUNA DE 360º**

Podrás mover tú antena de forma contínua en un rango de 360 grados sin tener que retroceder hacia atrás. Con la utilziación de un anillo colector (slip ring) y un servo de 360 grados, o uno normal modificado a tal efecto, el firmware es capaz de enviar órdenes para alcanzar al objetivo de forma rápida y precisa.

**MULTIPROTOCOLO**

Este antena tracker es un sistema todo en uno y universal, es capaz de decodificar diferentes protocolos de telemetría de los más populares sistemas de control de vuelo y radio control. Cuando estás en el campo de vuelo, sólo necesitas cambiar el protocolo y la velocidad de transmisión (baud rate) a través del menú de configuración en el display OLED

Protocolos soportados:

- **MFD** 
- **NMEA directo desde GPS**
- **MAVLINK**
- **RVOSD**
- **FRSKY D**
- **FRSKY X (Smartport)**
- **LTM (Light Telemetry)**

**AMORTIGUACIÓN EN SERVO TILT**

Se aplica efecto de amortiguación (configurable) para evitar daños en el servo tilt y otros mecanismos cuando se utiliza con antenas de grandes dimensiones. 


**POSICIÓN HOME AUTOMÁTICA**

Soporta dispositivos GPS UBLOX y NMEA para el establecimiento automático e la posición HOME del antena tracker.


**INFORMACIÓN EN DISPLAY**

Muestra información detallada sobre el estado del seguimiento, estado del GPS local (sólo 32 bits), monitorización de la batería, y menú de configuración (sólo 32 bits).


**4 PUERTOS SERIE**

Con la versión de 32 bits es posible configurar hasta 4 puertos serie (2 uart y 2 virtuales), con asignación dinámica de funciones (por ejemplo para gestión del GPS local o reenvío de telemetría a aplicaciones externas).


**COMUNICACIÓN CON APLICACIONES EXTERNAS**

Convierte la telemetría de entrada a diferentes formatos de telemetría de salida para conectar con aplicaciones externas: MAVLINK, NMEA, MFD. Podrás monitorizar el movimiento del aeromodelo con aplicaciones como Missión Planner, Oruxmaps, Droidplanner/Tower, etc...

**CONFIGURADOR MULTI PLATAFORMA**

Podrás configurar e inteactuar con tu antenna tracker a través de [u360gts-configurator](https://github.com/raul-ortega/u360gts-configurator), una aplicación multi plataforma para Google Chrome que facilita la configuración de los parámetros, las operaciones de control, así como realizar pruebas mediante simulación.

**CONFIGURADOR POR LÍNEA DE COMANDOS**

También permite la configuración por interfaz de línea de comandos desde consola serie.

# Versiones del Firmware u360gts

[https://github.com/raul-ortega/u360gts/releases](https://github.com/raul-ortega/u360gts/releases)

Encontrarás información detallada sobre la instalación y configuración del firmware en la [WIKI](https://github.com/raul-ortega/amv-open360tracker-32bits/wiki).
