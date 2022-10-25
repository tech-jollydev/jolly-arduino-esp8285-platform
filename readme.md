# Arduino core for the Jolly's ESP8285

### Contents
  - [Documentation](#documentation)
  - [Issue/Bug report](#issuebug-report)

### Documentation

**Getting Started**

Jolly is a new electronic module for makers and Arduino UNO lovers. 
The module has integrated Wi-Fi, to infinitely extend the potential of your projects into the new world of IoT.
You only need to remove the ATMEGA328P microcontroller, replace it with the Jolly module and you will have an IoT board with Wi-Fi.
To date, all the existing projects are mutually compatible.

This is an advanced platform that allows you to reprogram the ESP8285 onboard of the Jolly module. You can use it if you want to customize the Wi-Fi chip behaviour.

You can use the "Burn bootloader" feature of the standard Jolly platform if you need to restore the default MCU functionalities (factory pre-compiled firmware). You can find more information [here](https://github.com/tech-jollydev/jolly-arduino-platform/blob/master/README.md).

**Installing (Windows, Linux and macOS)**

Make sure your Arduino IDE version is 1.8.13 or higher.

To install the Jolly advanced platform in the Arduino IDE it is necessary to perform the following steps:
  - select the Preferences tab in the file menu
  - add (if not present) the following links to the **Additional Boards Manager URLs**
    https://tech-jollydev.github.io/package_jolly_index.json
    https://espressif.github.io/arduino-esp32/package_esp32_index.json
    https://arduino.esp8266.com/stable/package_esp8266com_index.json
  - select the board menu in the tools menu and finally select the boards manager option
  - by typing **jolly** in the search bar the **Jolly advanced** platform should appear
  - now the platform can be installed

**Upload the firmware**

In order to succesfully program esp8285, the microcontroller needs to be in boot mode.

For this reason, every time an upload needs to be performed, there is a specific procedure to follow:
  - press the boot button located on the Jolly module
  - while holding it pressed, press the reset button on the Arduino UNO for 1 second, then release it
  - keep holding the boot button for 1 second longer, then release it ![](/boot_sequence.png) 
  - now the Jolly is entered in the boot mode for the ESP8285 and it can be programmed

### Issue/Bug report
You can use **Issues** section of this repository to report an error. Before reporting an issue, make sure you've searched for similar one that was already created.
Finally, and only for severe issues, write an e-mail to tech@jolly-dev.com