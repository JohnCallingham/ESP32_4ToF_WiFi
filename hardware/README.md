## Power

Power is supplied by full wave rectifying the signal from the DCC bus using a bridge rectifier. This DC voltage is applied directly to the Arduino Nano ESP32 power input pin, VIN, which accepts a voltage in the range of 6-21 V.


## I2C

The PCB is fitted with an [I2C extender board](https://www.adafruit.com/product/4756) which is based on the LTC4311 IC.

An I2C extender cable connects from the extender board to a [four port I2C multiplexor](https://www.adafruit.com/product/5664) based on the PCA9546 IC.

Any combination of [VL6180](https://www.adafruit.com/product/3316) or [VL53L0X](https://www.adafruit.com/product/3317) time of flight sensors may be connected to the multiplexor ports by I2C extender cables as required. The code in main.cpp will need to be updated to match the type of sensor plugged into each multiplexor port as shown below;-

```
// Create four Adafruit_VL6180X or Adafruit_VL53L0X objects.
Adafruit_VL6180X sensor0 = Adafruit_VL6180X(); // VL6180
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X(); // VL53L0X
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X(); // VL53L0X
Adafruit_VL6180X sensor3 = Adafruit_VL6180X(); // VL6180

// Create four ToFSensor objects.
ToFSensorVL6180 tofSensor0(0, &sensor0);
ToFSensorVL53L0X tofSensor1(1, &sensor1);
ToFSensorVL53L0X tofSensor2(2, &sensor2);
ToFSensorVL6180 tofSensor3(3, &sensor3);
```

