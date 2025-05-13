# ESP32_4ToF_WiFi
 An OpenLCB/LCC node for handling 4 Time of Flight sensors designed to run on an Arduino Nano ESP32 and has been developed on the Arduino IDE.

## Functions
- The node connects to the JMRI OpenLCB/LCC hub over WiFi.
- When a hub connection is made the Nano's blue LED lights.
- The ESP32 connects to a PCA9546A (https://www.adafruit.com/product/5664) I2C 4 way multiplexor
to allow up to 4 VL6180 (https://www.adafruit.com/product/3316) time of flight (ToF) sensors can be connected.
- The hardware is designed to allow an LTC4311 (https://www.adafruit.com/product/4756) I2C extender to be used.
- JMRI's OpenLCB/LCC configuraion tool can be used to define 4 distance thresholds for each of the 4 ToF sensors.
- Each distance threshold comprises a near and far distance and near and far events.
- When a sensor detects an object closer than the near distance it sends the near event.
- When a sensor detects an object further away than the far distance it sends the far event.
- These events can be used in JMRI to create a JMRI sensor object for each of the 4 distance thresholds.
- When the node is initially connected it reports the current state of all thresholds.
- The node responds to a query from JMRI to give the current state of the selected sensor.

## Library Dependents
- [OpenLCB_Single_Thread](https://github.com/openlcb/OpenLCB_Single_Thread)
- [Adafruit_VL6180X](https://github.com/adafruit/Adafruit_VL6180X)

## Configuration
In file ESP32_4ToF_WiFi.ino;-
- '#define NODE_ADDRESS  5,1,1,1,0x91,0x05'. This is the 6 byte LCC node address which must be unique.
- '#define RESET_TO_FACTORY_DEFAULTS 0'. This must be set to 1 at least once to force the EEPROM to reset to factory defaults.
- 'const char* ssid = "RPi-JMRI";'. This is the network name for connecting to JMRI.
- 'const char* password = "rpI-jmri";'. This is the network password.
- 'const char* openLCB_can  = "openlcb-can";'. This is the default for JMRI.
