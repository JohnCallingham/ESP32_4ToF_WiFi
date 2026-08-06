# ESP32_4ToF_WiFi

This is a program to create an [OpenLCB/LCC](https://openlcb.org/) node. It was developed using PlatformIO to run on an Arduino Nano ESP32. The node is designed to connect over WiFi to the LCC hub provided by JMRI.

It is part of a group of node types which use the same codebase. The other node types are;-
- [ESP32_2Servo_2Frog_2TOTI_WiFi](https://github.com/JohnCallingham/ESP32_2Servo_2Frog_2TOTI_WiFi)
- [ESP32_6TOTI_WiFi](https://github.com/JohnCallingham/ESP32_6TOTI_WiFi)

## General functionality

All of the node types which share the common codebase provide the following functionality;-
- Responds to consumed events and sends produced events, depending on the specific features of the node.
- When initially connected to JMRI's LCC hub the node sends the state of all events so that JMRI knows the current state of the node.
- Responds to queries from JMRI.
- Allows the user to configure the ESP32's built in RGB LED to indicate various states of the node.
- Allows the user to start various testing cycles for the node.
- Allows for remote configuration and remote firmware updates.

## Specific functionality for this node type

1. Allows for four ToF sensors to be connected.
2. Each sensor has four thresholds.
3. As the sensors do not provide a stable reading over time, each threshold has two parameters - a near threshold and a far threshold.
4. An event is sent when the sensor detects that an object has become closer to it than the near threshold.
5. An event is sent when the sensor detects that an object has become futher away than the far threshold.
5. The configuration interace allows the user to specify the near and far thresholds by selecting a base threshold and a hysterisis value.
6. The near threshold is calculated by subtracting half of the hysterisis value from the base threshold.
7. The far threshold is calculated by adding half of the hysterisis value to the base threshold.

## Software components
This software uses the following components;-
- [OpenLCB_Single_Thread](https://github.com/openlcb/OpenLCB_Single_Thread)
- [ESP32WiFiGC](https://github.com/JohnCallingham/ESP32WiFiGC)
- [LCC_CONFIGURATION](https://github.com/JohnCallingham/LCC_CONFIGURATION)
- [LCC_TOF_SENSOR](https://github.com/JohnCallingham/LCC_TOF_SENSOR.git)

The following software components are dependencies of one or more of the above components;-
- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
- [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
- [Adafruit_VL53L0X](https://github.com/adafruit/Adafruit_VL53L0X.git)
- [Adafruit_VL6180X](https://github.com/adafruit/Adafruit_VL6180X.git)
- [AduinoJson](https://github.com/bblanchon/ArduinoJson)
- [LCC_NODE_CONFIGURATION_BASE](https://github.com/JohnCallingham/LCC_NODE_COMPONENT_BASE)
- [I2C_PERIPHERAL](https://github.com/JohnCallingham/I2C_PERIPHERAL)

PlatformIO's Library Dependency Finder handles the downloading of all required dependencies.
