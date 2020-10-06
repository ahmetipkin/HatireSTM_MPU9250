# Hatire project with MPU9250
Hat Tracking implementation compatible with 'Hatire' protocol using MPU9250. A sort-of port of the HatireSTM with MPU9250

Based on OpenTrack's Hatire plugin and Arduino counterpart; I've implemented this. Sensor fusion work is heavily based on (if not directly taken) [Kris Winer's work here](https://github.com/kriswiner/MPU9250/).
 
I burned all my BNO055 sensors; so needed one to work with the project. :)

## How to use

Just flash using Sloeber or Arduino IDE. It requires STM32 Core: https://github.com/stm32duino/Arduino_Core_STM32

Use OpenTrack or similar software and chose Hatire or Hat as 'plugin source'.

The chip will not submit input until it calibrates itself after 'I' command. Button 32 (maple's onboard button) can be used to 'bail' from initial calibration. In normal run; it invokes calibration by hand. Of course; command 'C' does work too.

Enjoy!


## Used Libraries

* SparkFun's MPU9250 library; but heavily modified. Used custom version is also available here.
* EEPROM (for saving calibration)

