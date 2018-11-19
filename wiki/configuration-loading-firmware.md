## Loading firmware

In this moment the u360gts-configurator tools can not be used to flash the firmware on the board, but it can help to force the board into boot mode.

There are serveral options:

- Using the STM Micro electronics tool [Flash Loader Demonsrator](https://www.st.com/en/development-tools/flasher-stm32.html)
- Using Betaflight/iNav configurator.

**Note: This wiki will focuse only on STM Micro electronics tool.**

Please, follow this instructions in orther to install the firmware on the board with success.

Download first the firmware [latest release](https://github.com/raul-ortega/u360gts/releases/latest).

```
1.- Coloca el jumper en los pines boot.
2.- Conecta el cable Micro USB a la controladora y al PC.
3.- Abre el programa Flash Loader Demonstrator
4.- Sigue las instrucciones del modo boot del manual de la NAZE32.
5.- Cierra el programa Flash Loader Demonstrator.
6.- Desconecta el cable Micro USB.
7.- Quita el Jumper.
8.- Vuelve a conectar el cable Micro USB.
```

[<< Go back](https://github.com/raul-ortega/u360gts/blob/master/wiki/index.md)
