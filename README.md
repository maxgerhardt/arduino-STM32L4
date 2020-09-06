# Arduino Core for STM32L4 based boards

## PlatformIO integration 

If you wish to use this core using PlatformIO, create a new PlatformIO with any board. Then edit the `platformio.ini` so that it points to the modified `ststm32` platform and this package. An example project is given at https://github.com/maxgerhardt/pio-stm32l4core-example. 

Available boards: 
* dragonfly_l496rg
* dragonfly_l476re
* butterfly_l433cc
* ladybug_l432kc
* grumpyoldpizza_nucleo_l432kc
* grumpyoldpizza_nucleo_l476rg

Further configuration options for clock speed, USB support and DOSFS can be activated by modifying the `platformio.ini` 

### Clock speed

Add the changed value via

```
board_build.f_cpu = 80000000L
```

as e.g. shown in [docs](https://docs.platformio.org/en/latest/boards/ststm32/nucleo_l476rg.html#configuration). 

### USB
Add a `-D <flag>` to the `build_flags = ..` list in the `platformio.ini`. Supported flags are: 

* Serial / USB CDC: `PIO_FRAMEWORK_ARDUINO_ENABLE_CDC`
* Serial + Mass Storage: `PIO_FRAMEWORK_ARDUINO_ENABLE_CDC_AND_MSC`
* Serial + Keyboard + Mouse: `PIO_FRAMEWORK_ARDUINO_ENABLE_CDC_AND_HID`
* Serial + Mass Storage + Keyboard + Mouse: `PIO_FRAMEWORK_ARDUINO_ENABLE_CDC_AND_MSC_AND_HID`

Not setting any special flag will result in the "No USB" setting.

### DOSFS
* SFLASH (QSPI): `PIO_FRAMEWORK_ARDUINO_DOSFS_SFLASH_QSPI`
* SDCARD (SPI): `PIO_FRAMEWORK_ARDUINO_DOSFS_SDCARD_SPI`
* SDCARD (SDIO Default Speed):`PIO_FRAMEWORK_ARDUINO_DOSFS_SDCARD_SDIO_DEFAULT_SPEED`
* SDCARD (SDIO High Speed): `PIO_FRAMEWORK_ARDUINO_DOSFS_SDCARD_SDIO_HIGH_SPEED`

### Example configuration:

```ini
[env:dragonfly_l476re]
platform = https://github.com/maxgerhardt/platform-ststm32.git
framework = arduino
platform_packages = 
     framework-arduinoststm32-stm32l4@https://github.com/maxgerhardt/arduino-STM32L4.git
board = dragonfly_l476re
build_flags = 
    -D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC
```

## Supported boards

### Tlera Corp
 * [Dragonfly-STM32L476RE](https://www.tindie.com/products/TleraCorp/dragonfly-stm32l476-development-board)
 * [Butterfly-STM32L433CC](https://www.tindie.com/products/TleraCorp/butterfly-stm32l433-development-board)
 * [Ladybug-STM32L432KC](https://www.tindie.com/products/TleraCorp/ladybug-stm32l432-development-board)

### STMicroelectronics
 * NUCLEO-L432
 * NUCLEO-L476

## Installing

### Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (at least version v1.6.8)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add ```https://grumpyoldpizza.github.io/arduino-STM32L4/package_STM32L4_boards_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "STM32L4 Boards by Tlera Corp"
 6. Select your STM32L4 board from the Tools -> Board menu

#### OS Specific Setup

##### Linux

 1. Go to ~/.arduino15/packages/grumpyoldpizza/hardware/stm32l4/```<VERSION>```/drivers/linux/
 2. sudo cp *.rules /etc/udev/rules.d
 3. reboot

#####  Windows

###### STM32 BOOTLOADER driver setup for Tlera Corp boards

 1. Download [Zadig](http://zadig.akeo.ie)
 2. Plugin STM32L4 board and toggle the RESET button while holding down the BOOT button
 3. Let Windows finish searching for drivers
 4. Start ```Zadig```
 5. Select ```Options -> List All Devices```
 6. Select ```STM32 BOOTLOADER``` from the device dropdown
 7. Select ```WinUSB (v6.1.7600.16385)``` as new driver
 8. Click ```Replace Driver```

###### USB Serial driver setup for Tlera Corp boards (Window XP / Windows 7 only)

 1. Go to ~/AppData/Local/Arduino15/packages/grumpypoldpizza/hardware/stm32l4/```<VERSION>```/drivers/windows
 2. Right-click on ```dpinst_x86.exe``` (32 bit Windows) or ```dpinst_amd64.exe``` (64 bit Windows) and select ```Run as administrator```
 3. Click on ```Install this driver software anyway``` at the ```Windows Security``` popup as the driver is unsigned

###### ST-LINK V2.1 driver setup for NUCLEO boards

 1. Plugin NUCLEO board
 2. Download and install [ST-Link USB Drivers](http://www.st.com/en/embedded-software/stsw-link009.html)

### From git (for core development)

 1. Follow steps from Board Manager section above
 2. ```cd <SKETCHBOOK>```, where ```<SKETCHBOOK>``` is your Arduino Sketch folder:
  * OS X: ```~/Documents/Arduino```
  * Linux: ```~/Arduino```
  * Windows: ```~/Documents/Arduino```
 3. Create a folder named ```hardware```, if it does not exist, and change directories to it
 4. Clone this repo: ```git clone https://github.com/grumpyoldpizza/arduino-STM32L4.git grumpyoldpizza/stm32l4```
 5. Restart the Arduino IDE

## Recovering from a faulty sketch for Tlera Corp Boards

 Sometimes a faulty sketch can render the normal USB Serial based integration into the Arduindo IDE not working. In this case plugin the STM32L4 board and toggle the RESET button while holding down the BOOT button and program a known to be working sketch to go ack to a working USB Serial setup.

## Credits

This core is based on the [Arduino SAMD Core](https://github.com/arduino/ArduinoCore-samd)

