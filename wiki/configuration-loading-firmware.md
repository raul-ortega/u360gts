## Loading firmware

In this moment the u360gts-configurator tools can not be used to flash the firmware on the board, but it can help to force the board into boot mode.

There are serveral options:

- Using the STM Micro electronics tool [Flash Loader Demonsrator](https://www.st.com/en/development-tools/flasher-stm32.html)
- Using Betaflight/iNav configurator.

**Note: This wiki will focuse only on STM Micro electronics tool.**

Please, follow this instructions in orther to install the firmware on the board with success.

Download first the firmware [latest release](https://github.com/raul-ortega/u360gts/releases/latest).

### Flashing for the first time

1. Download the hex file for your controller board.
2. Turn on your controller board in "boot mode". For NAZE32/Flip32 you have to bridge boot pins before giving power.
3. Run Flash Loader Demonstrator (download from [here](https://www.st.com/en/development-tools/flasher-stm32.html).)
4. Select port, baud rate, and next.
5. Browse and select the hex file, select Global Erase and Jump to user program options, and next.
6. After flashing close Flash Loader Demonstrator.
7. Run u360gts configurator, select 115200 bauds, press Connect and  CLI ENTER buttons.

### Upgrading from previous version

1. Download the hex file for your controller board.
2. Select baud rate, connect and press CLI ENTER button
3. Go to Cli Mode tab, press BACKUP Config and save the configuration file.
4. Press BOOT MODE button.
5. Run Flash Loader Demonstrator (download from [here](https://www.st.com/en/development-tools/flasher-stm32.html).)
6. Select port, baud rate, and next.
7. Browse and select the hex file, select Global Erase and Jump to user program options, and next.
8. After flashing close Flash Loader Demonstrator.
9. Go to u360gts configurator, select 115200 bauds, press Connect and  CLI ENTER buttons.
10. Go to Cli Mode tab and press RESTORE CONFIG.
11. Save and you are done.

After this you are ready to connect and enter CLI Mode in order to configure the new features.

## Configuration



[<< Go back](https://github.com/raul-ortega/u360gts/blob/master/wiki/index.md)
