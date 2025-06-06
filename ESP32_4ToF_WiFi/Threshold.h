#ifndef Threshold_h
#define Threshold_h

#include <Arduino.h>
#include "Global.h"

// typedef enum {
//   Near = 0,
//   Far = 1
// } State_t;
enum class State { Unknown, Near, Far };

typedef struct {
  uint8_t sensor;
  uint8_t threshold;
  uint8_t valueNear;
  uint8_t valueFar;
  uint16_t eventIndexNear;
  uint16_t eventIndexFar;
  // State_t currentState;
  State currentState;
} threshold_t;

typedef struct { 
  struct {
    threshold_t threshold[NUM_THRESHOLD];
  } sensor[NUM_SENSOR];
} ThresholdArray_t; 

class Threshold {
  public:
    //Threshold();
    /**
     * Stores the threshold in the array of thresholds.
     * Called at startup and when config value changed.
     */
    void set(threshold_t threshold); // one threshold used at startup and when config value changed

    /**
     * Compares range with the current state for this threshold for this sensor and
     * Returns the index of the event to send or -1 if there is no event to be sent.
     */
    int check(uint8_t sensor, uint8_t threshold, uint8_t range);

    /**
     * Returns the current state for the threshold which contains index.
     */
    State getStateForEventIndex(uint16_t index);

    /**
     * Set the state of the thresholds for this sensor.
     */
    void setInitialState(uint8_t sensor, int range);

    /**
     * Checks the current state for the threshold for the sensor.
     * Returns the index of the event to send or -1 if there is no event to be sent.
     */
    // int getCurrentEvent(uint8_t sensor, uint8_t threshold);
    int getEventIndexForCurrentState(uint8_t sensor, uint8_t threshold);

    /**
     * Displays all sensor information to the serial port.
     */
    void print();

  private:
    ThresholdArray_t thresholds;

};

#endif
