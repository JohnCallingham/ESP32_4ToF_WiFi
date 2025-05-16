#include "ToFSensor.h"

ToFSensor::ToFSensor() {
  vl = Adafruit_VL6180X();
}

void ToFSensor::begin() {
  Wire.begin();
}

int ToFSensor::read(uint8_t sensor) {

  // Check for a valid port value.
  if (sensor >= NUM_SENSOR) return -1;

  // Switch the mux to this port.
  Wire.beginTransmission(MULTIPLEXER_I2C_ADDRESS);
  Wire.write(1 << sensor);
  Wire.endTransmission();

  // Check for a sensor on this port.
  if (! vl.begin()) {
    // There is no sensor on this port.
    return -1;
  }
  
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    return range;
  } else {
    return 255;
  }
}
