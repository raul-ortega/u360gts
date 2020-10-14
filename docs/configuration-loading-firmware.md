## Loading the Firmware

### Configurator Tool

**u360gts configurator** is the official tool for supported boardas, it can be downloaded from [release page](https://github.com/raul-ortega/u360gts-configurator/releases/latest).
It is a crossplatform tool that can be run on Windows, MacOS and Linux machines, and **standalone application** 

Download the binaries for your operating system and install it following the wizard instructions (Linux users must follow nw.js [instructions](https://github.com/nwjs/nw.js#documents) in order to build and/or run the configurator source code).

#### Flashing the Firmware

Please, follow this instructions for firmware flashing:

1.- Download and isntall u360gts configurator.
2.- Launch the application after installation.
3.- Jumper the boot pins of the board.
4.- Connect the usb cable to your computer.

   If [STM32 drivers](https://zadig.akeo.ie/) for the board are already installed, the new COM port will be shown and selected in the dropdown ports list.
   
5.- Click on **Flash Firmware** tab of left panel.
6.- **Choose** your **board**.
7.- **Choose** the lastest **firmware version** available.
8.- Click on **Load Firmware Online**.

   During firmware downloading process the button will be disabled, once downloaded the button will be back to orange again and release info will be shown (read it carefully).
	
9.- Enable **Full chip erase** option.
10.- Enable **Manual baud rate** and select **115200**.
11.- Click on **Flash Firmware** button.

   The proggress bar animation will start and will show **Flashing** while loading the firmware to the board. After that **Verifying** will be shown and finally, if everything went well, **Programming successful** message will apear on the bar.
   
   If something went wrong an error message will be shown on the bar. Please read carefully **Warning** and **Recovery/Lost communication** notes on screen to get it solved.
   
12.- Disconnect the usb cable.
13.- **Remove the jumper** from the boot pins.

#### Connecting for the First Time.

Please, follow this instructions for connecting to the board from u360gts configurator:

1.- Connect the **usb cable** to your computer.

   If [STM32 drivers](https://zadig.akeo.ie/) for the board are already installed, the new COM port will be shown and selected in the dropdown ports list.
   
2.- Select **115200** baud rate from the dropdwon list.
3.- Click on **Connect** button (top right corner).

   If everything went well **Configuration tab** will be shown, and board name, firmware version and date/time will be shown on the *Log* window. You are ready for configuring your u360gts antenna tracker.

### Alternative Flashing Methods.

- Using the STM Micro Electronics tool [Flash Loader Demonsrator](https://www.st.com/en/development-tools/flasher-stm32.html)
- Using Betaflight configurator or iNav configurator.

In this cases you must [download](https://github.com/raul-ortega/u360gts/releases/latest) first u360gts firmware for your board.

#### FLASH LOADER DEMONSTRATOR

#### Flashing for the First Time

1. Download the hex file for your controller board from [here](https://github.com/raul-ortega/u360gts/releases/latest).
2. Turn on your controller board in "boot mode". For NAZE32, Flip32 and SP Racinfg F3 boards you have to bridge boot pins before giving power.
3. Run Flash Loader Demonstrator (download from [here](https://www.st.com/en/development-tools/flasher-stm32.html).)
4. Select port, baud rate, and next.
5. Browse and select the hex file, select Global Erase and Jump to user program options, and next.
6. After flashing close Flash Loader Demonstrator.
7. Run u360gts configurator, select 115200 bauds, and press Connect button.

[<< Go back](README.md)
