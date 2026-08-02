//==============================================================
// ESP32_4ToF_Wifi based on Pico_8ServoWifiGC
// Modified John Callingham 2025, added;-
// - removed all servo code.
// - added four Tine of Flight sensors.
// - each sensor has four distance thresholds.
// - events produced when a threshold is passed.
// MOdified DPH 2024
// Copyright 2019 Alex Shepherd and David Harris
//==============================================================
//// Debugging -- uncomment to activate debugging statements:
    // dP(x) prints x, 
    // dPH(x) prints x in hex, 
    // dPS(string,x) prints string and x
// #define DEBUG Serial
#define OLCB_NO_BLUE_GOLD
#define NOCAN

#include <Arduino.h>
#include "credentials.h"
// #include "ESP32WiFiGC_V2.h"
// #include "ESP32WiFiGC_V3.h"
#include "ESP32WiFiGC_V4.h"
// #include "ToFSensor.h"
#include "ToFSensorVL6180.h"
#include "ToFSensorVL53X0L.h"
#include "configurationPreferences.h"

// Board definitions
#define MANU "J Callingham"  // The manufacturer of node
#define MODEL "ESP32_4ToF_Wifi" // The model of the board
#define HWVERSION "0.1"   // Hardware version
#define SWVERSION "1.0.0"   // Software version

// To Reset the Node Number, Uncomment and edit the next line
// #define NODE_ADDRESS  5,1,1,1,0x91,0x05
#define NODE_ADDRESS  5,1,1,1,0x91,0x00  // Default node ID which will be changed by the remote configuration software.

// Set to 1 to Force Reset EEPROM to Factory Defaults 
// Need to do this at least once.  
#define RESET_TO_FACTORY_DEFAULTS 0

#define NUM_SENSOR 4
#define NUM_THRESHOLD 4

#define NUM_RGB_LED 3 // Red, green and blue.
#define RGB_LED_RED 0 // The index to the RGB_LEDs array.
#define RGB_LED_GREEN 1 // The index to the RGB_LEDs array.
#define RGB_LED_BLUE 2 // The index to the RGB_LEDs array.

// User defs - moved to Global.h so they are available in other files.
#define NUM_EVENT NUM_SENSOR * NUM_THRESHOLD * 2 // Each threshold has two events, one for each threshold being passed.

/**
 * Definitions of the LED configuration property values.
 */
#define LED_CONFIG_NOT_CONFIGURED 0
#define LED_CONFIG_HUB_STATE 1
#define LED_CONFIG_SENSOR_0_THRESHOLD_0 2
#define LED_CONFIG_SENSOR_0_THRESHOLD_1 3
#define LED_CONFIG_SENSOR_0_THRESHOLD_2 4
#define LED_CONFIG_SENSOR_0_THRESHOLD_3 5
#define LED_CONFIG_SENSOR_1_THRESHOLD_0 6
#define LED_CONFIG_SENSOR_1_THRESHOLD_1 7
#define LED_CONFIG_SENSOR_1_THRESHOLD_2 8
#define LED_CONFIG_SENSOR_1_THRESHOLD_3 9
#define LED_CONFIG_SENSOR_2_THRESHOLD_0 10
#define LED_CONFIG_SENSOR_2_THRESHOLD_1 11
#define LED_CONFIG_SENSOR_2_THRESHOLD_2 12
#define LED_CONFIG_SENSOR_2_THRESHOLD_3 13
#define LED_CONFIG_SENSOR_3_THRESHOLD_0 14
#define LED_CONFIG_SENSOR_3_THRESHOLD_1 15
#define LED_CONFIG_SENSOR_3_THRESHOLD_2 16
#define LED_CONFIG_SENSOR_3_THRESHOLD_3 17

// These have all been changed from 8 bit to 16 bit variables so that the following debounce delay variables, which are 16 bit, are aligned to a 16 bit boundary.
uint16_t ledRedConfiguration;
uint16_t ledGreenConfiguration;
uint16_t ledBlueConfiguration;
bool ledConfigHubConnected = false;

// Forward declarations
uint8_t getLEDState(int switchInput);

#include "mdebugging.h"           // debugging
#include "processor.h"
#include "processCAN.h"
#include "OpenLCBHeaderJC.h"

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

// Declare an array of pointers to the ToFSensor objects.
ToFSensor *tofSensor[NUM_SENSOR];

// CDI (Configuration Description Information) in xml, must match MemStruct
// See: http://openlcb.com/wp-content/uploads/2016/02/S-9.7.4.1-ConfigurationDescriptionInformation-2016-02-06.pdf
extern "C" {
    #define N(x) xN(x)     // allow the insertion of the value (x) ..
    #define xN(x) #x       // .. into the CDI string. 
const char configDefInfo[] PROGMEM =
// ===== Enter User definitions below =====
  CDIheader R"(
    <group replication=')" N(NUM_RGB_LED) R"('>
      <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
      <name>LED Control</name>
      <repname>Red</repname><repname>Green</repname><repname>Blue</repname>

      <int size='2'>
        <name>LED function</name>
        <default>0</default>
        <map>
          <relation><property>0</property><value>Not configured</value></relation>
          <relation><property>1</property><value>Hub status</value></relation>
          <relation><property>2</property><value>Sensor 0, Threshold 0 status</value></relation>
          <relation><property>3</property><value>Sensor 0, Threshold 1 status</value></relation>
          <relation><property>4</property><value>Sensor 0, Threshold 2 status</value></relation>
          <relation><property>5</property><value>Sensor 0, Threshold 3 status</value></relation>
          <relation><property>6</property><value>Sensor 1, Threshold 0 status</value></relation>
          <relation><property>7</property><value>Sensor 1, Threshold 1 status</value></relation>
          <relation><property>8</property><value>Sensor 1, Threshold 2 status</value></relation>
          <relation><property>9</property><value>Sensor 1, Threshold 3 status</value></relation>
          <relation><property>10</property><value>Sensor 2, Threshold 0 status</value></relation>
          <relation><property>11</property><value>Sensor 2, Threshold 1 status</value></relation>
          <relation><property>12</property><value>Sensor 2, Threshold 2 status</value></relation>
          <relation><property>13</property><value>Sensor 2, Threshold 3 status</value></relation>
          <relation><property>14</property><value>Sensor 3, Threshold 0 status</value></relation>
          <relation><property>15</property><value>Sensor 3, Threshold 1 status</value></relation>
          <relation><property>16</property><value>Sensor 3, Threshold 2 status</value></relation>
          <relation><property>17</property><value>Sensor 3, Threshold 3 status</value></relation>
        </map>
      </int>
    </group>

    <group>
        <name>Node Configuration</name>
        <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
        <group replication=')" N(NUM_SENSOR) R"('>
            <name>ToF Sensors</name>
            <repname>1</repname>
            <string size='24'><name>Description</name></string>
            <group replication=')" N(NUM_THRESHOLD) R"('>
                <name>Thresholds</name>
                <repname>1</repname>
                <int size='2'>
                  <name>Threshold (mm)</name>
                  <min>0</min>
                  <max>500</max>
                  <hints><slider tickSpacing='100' immediate='yes' showValue='true'></slider></hints>
                </int>
                <int size='2'>
                  <name>Hysterisis (mm)</name>
                  <min>10</min>
                  <max>100</max>
                  <hints><slider tickSpacing='30' immediate='yes' showValue='true'></slider></hints>
                </int>
                <eventid>
                  <name>Near EventID</name>
                  <description>Sent when an object moves nearer to the sensor than the near threshold</description>
                </eventid>
                <eventid>
                  <name>Far EventID</name>
                  <description>Sent when an object moves further away from the sensor than the far threshold</description>
                </eventid>
            </group>
        </group>
    </group>
    )" CDIfooter;
// ===== Enter User definitions above =====
} // end extern

// ===== MemStruct =====
//   Memory structure of EEPROM, must match CDI above
    typedef struct { 
          EVENT_SPACE_HEADER eventSpaceHeader; // MUST BE AT THE TOP OF STRUCT - DO NOT REMOVE!!!
          
          char nodeName[20];  // optional node-name, used by ACDI
          char nodeDesc[24];  // optional node-description, used by ACDI
          // ===== Enter User definitions below =====

          struct {
            uint16_t ledConfiguration; // Changed from 8 bit to 16 bit so that all variables align to a 16 bit boundary.
          } RGB_LEDs[NUM_RGB_LED];

          struct {
            char sensordesc[24];        // description of this Sensor
            struct {
              // uint8_t thresholdNear;       // threshold for ON event
              // uint8_t thresholdFar;       // Threshold for OFF event
              // uint8_t threshold;       // threshold for ON event
              // uint8_t hysterisis;       // Threshold for OFF event ??? comment to be changed
              uint16_t threshold;       // threshold for ON event
              uint16_t hysterisis;       // Threshold for OFF event ??? comment to be changed
              EventID eidNear;       // ON eventID
              EventID eidFar;       // OFF eventID
            } threshold[NUM_THRESHOLD];
          } sensor[NUM_SENSOR];
      // ===== Enter User definitions above =====
      // items below will be included in the EEPROM, but are not part of the CDI
      //uint8_t servoLastAngle[NUM_SERVOS];
    } MemStruct;      // type definition

void userInitAll() {
  NODECONFIG.put(EEADDR(nodeName), ESTRING("ESP32_4ToF_Wifi"));
  NODECONFIG.put(EEADDR(nodeDesc), ESTRING("ESP32_4ToF_Wifi"));
  
  for (uint8_t i = 0; i < NUM_SENSOR; i++) {
    NODECONFIG.put(EEADDR(sensor[i].sensordesc), ESTRING(""));

    NODECONFIG.update16(EEADDR(RGB_LEDs[RGB_LED_RED].ledConfiguration), LED_CONFIG_NOT_CONFIGURED); // The red LED default is not configured.
    NODECONFIG.update16(EEADDR(RGB_LEDs[RGB_LED_GREEN].ledConfiguration), LED_CONFIG_NOT_CONFIGURED); // The green LED default is not configured.
    NODECONFIG.update16(EEADDR(RGB_LEDs[RGB_LED_BLUE].ledConfiguration), LED_CONFIG_HUB_STATE); // The blue LED default is to show the hub connection status.

    // Set sensible default thresholds.
    // NODECONFIG.put(EEADDR(sensor[i].threshold[0].threshold), (uint8_t) 25);
    // NODECONFIG.put(EEADDR(sensor[i].threshold[0].hysterisis), (uint8_t) 10);
    // NODECONFIG.put(EEADDR(sensor[i].threshold[1].threshold), (uint8_t) 55);
    // NODECONFIG.put(EEADDR(sensor[i].threshold[1].hysterisis), (uint8_t) 10);
    // NODECONFIG.put(EEADDR(sensor[i].threshold[2].threshold), (uint8_t) 95);
    // NODECONFIG.put(EEADDR(sensor[i].threshold[2].hysterisis), (uint8_t) 10);
    // NODECONFIG.put(EEADDR(sensor[i].threshold[3].threshold), (uint8_t) 155);
    // NODECONFIG.put(EEADDR(sensor[i].threshold[3].hysterisis), (uint8_t) 10);
    NODECONFIG.update16(EEADDR(sensor[i].threshold[0].threshold), (uint16_t) 25); // cast needed ??
    NODECONFIG.update16(EEADDR(sensor[i].threshold[0].hysterisis), (uint16_t) 10);
    NODECONFIG.update16(EEADDR(sensor[i].threshold[1].threshold), (uint16_t) 55);
    NODECONFIG.update16(EEADDR(sensor[i].threshold[1].hysterisis), (uint16_t) 10);
    NODECONFIG.update16(EEADDR(sensor[i].threshold[2].threshold), (uint16_t) 95);
    NODECONFIG.update16(EEADDR(sensor[i].threshold[2].hysterisis), (uint16_t) 10);
    NODECONFIG.update16(EEADDR(sensor[i].threshold[3].threshold), (uint16_t) 155);
    NODECONFIG.update16(EEADDR(sensor[i].threshold[3].hysterisis), (uint16_t) 10);
  }
}

extern "C" {
    // ===== eventid Table =====
    // useful macros to help fill the table

    // Each threshold has two events.
    #define REG_THRESHOLD(s,p) PEID(sensor[s].threshold[p].eidNear), PEID(sensor[s].threshold[p].eidFar)

    // Each sensor has four thresholds.
    #define REG_SENSOR(s) REG_THRESHOLD(s,0), REG_THRESHOLD(s,1), REG_THRESHOLD(s,2), REG_THRESHOLD(s,3)

    //  Array of the offsets to every eventID in MemStruct/EEPROM/mem, and P/C flags
    const EIDTab eidtab[NUM_EVENT] PROGMEM = {
        // There are four sensors.
        REG_SENSOR(0), REG_SENSOR(1), REG_SENSOR(2), REG_SENSOR(3)
    };
    
    // SNIP Short node description for use by the Simple Node Information Protocol
    // See: http://openlcb.com/wp-content/uploads/2016/02/S-9.7.4.3-SimpleNodeInformation-2016-02-06.pdf
    // extern const char SNII_const_data[] PROGMEM = "\001" MANU "\000" MODEL "\000" HWVERSION "\000" OlcbCommonVersion ; // last zero in double-quote
    extern const char SNII_const_data[] PROGMEM = "\001" MANU "\000" MODEL "\000" HWVERSION "\000" SWVERSION ; // last zero in double-quote
} // end extern "C"

// PIP Protocol Identification Protocol uses a bit-field to indicate which protocols this node supports
// See 3.3.6 and 3.3.7 in http://openlcb.com/wp-content/uploads/2016/02/S-9.7.3-MessageNetwork-2016-02-06.pdf
uint8_t protocolIdentValue[6] = {   //0xD7,0x58,0x00,0,0,0};
        pSimple | pDatagram | pMemConfig | pPCEvents | !pIdent    | pTeach     | !pStream   | !pReservation, // 1st byte
        pACDI   | pSNIP     | pCDI       | !pRemote  | !pDisplay  | !pTraction | !pFunction | !pDCC        , // 2nd byte
        0, 0, 0, 0                                                                                           // remaining 4 bytes
    };

/**
 * userState() is called when JMRI queries the state of an event index.
 */
enum evStates { VALID=4, INVALID=5, UNKNOWN=7 };
uint8_t userState(uint16_t index) {
  // Determine which sensor has this event index.
  for (uint8_t i=0; i<NUM_SENSOR; i++) {
    if (tofSensor[i]->eventIndexMatches(index)) {
      // This sensor has this event index.
      return tofSensor[i]->eventIndexMatchesCurrentState(index) ? VALID : INVALID;
    }
  }

  return UNKNOWN; // In case index is not recognised.
}

void sendEventCallbackFunction(uint16_t eventIndexToSend) {
  if (hubConnected) {
    Serial.printf("\n%6ld sendEventCallbackFunction() called. event index=0x%02X", millis(), eventIndexToSend);
    OpenLcb.produce(eventIndexToSend);
  }
}

void sendInitialEvents() {
  for (uint8_t i=0; i<NUM_SENSOR; i++) {
    tofSensor[i]->sendEventsForCurrentState();
  }
}

// ===== Process Consumer-eventIDs =====
void pceCallback(uint16_t index) {
// Invoked when an event is consumed; drive pins as needed
// from index of all events.
// Sample code uses inverse of low bit of pattern to drive pin all on or all off.
// The pattern is mostly one way, blinking the other, hence inverse.
//
  //dPS((const char*)"\npceCallback: Event Index: ", index);
  dP("\neventid callback: index="); dP((uint16_t)index);
}

void produceFromInputs() {
}

void userSoftReset() {}
void userHardReset() {}

// Callback from a Configuration write
// Use this to detect changes in the ndde's configuration
// This may be useful to take immediate action on a change.
void userConfigWritten(uint32_t address, uint16_t length, uint16_t func)
{
  dPS("\nuserConfigWritten: Addr: ", (uint32_t)address); 
  dPS("  Len: ", (uint16_t)length); 
  dPS("  Func: ", (uint8_t)func);

  // Need to do an EEPROM commit as this doesn't always appear to happen!!
  EEPROM.commit();

  // Update the LED's configuration as it may have been changed.
  ledRedConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_RED].ledConfiguration));
  ledGreenConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_GREEN].ledConfiguration));
  ledBlueConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_BLUE].ledConfiguration));

  // Update the ToF sensor's properties as they may have changed. TO DO: need to test this !!!!
  for (uint8_t i=0; i<NUM_SENSOR; i++) {
    for (uint8_t j=0; j<NUM_THRESHOLD; j++) {
      tofSensor[i]->updateValueAndHysterisis(j,
        // NODECONFIG.read(EEADDR(sensor[i].threshold[j].threshold)), // need to use read16
        // NODECONFIG.read(EEADDR(sensor[i].threshold[j].hysterisis)) // need to use read16
        NODECONFIG.read16(EEADDR(sensor[i].threshold[j].threshold)), // need to use read16
        NODECONFIG.read16(EEADDR(sensor[i].threshold[j].hysterisis)) // need to use read16
      );
    }
  }
}

void initialiseRGBLEDs() {
  // Configure the built in RGB LEDs and turn them off.
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);

  // Determine how the RGB LEDs are configured.
  ledRedConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_RED].ledConfiguration));
  ledGreenConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_GREEN].ledConfiguration));
  ledBlueConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_BLUE].ledConfiguration));
}

void initialiseToFSensors() {
  int retVal;
  bool noDevices = true;
  bool muxConnected = false;

  // Start the I2C bus.
  if (Wire.begin()) {
    Serial.printf("\n%6ld I2C initialised successfully", millis());
  } else {
    Serial.printf("\n%6ld I2C initialisation failed", millis());
    return;
  }

  // Reset the mux in case any port was selected before the ESP32 was reset.
  Wire.beginTransmission(MULTIPLEXER_I2C_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  for (int i2cAddress = 0x01; i2cAddress < 0x7F; i2cAddress++) {
    Wire.beginTransmission(i2cAddress);
    retVal = Wire.endTransmission();

    if (retVal == 0) {
      Serial.printf("\n%6ld Found device at I2C address 0x%02X directly connected", millis(), i2cAddress);
      noDevices = false;
      if (i2cAddress == MULTIPLEXER_I2C_ADDRESS) {
        muxConnected = true;
      }
    }
  }

  if (noDevices) {
    Serial.printf("\n%6ld No I2C devices directly connected", millis());
  }

  // Store pointers to the ToFSensor objects in the tofSensor array.
  tofSensor[0] = &tofSensor0;
  tofSensor[1] = &tofSensor1;
  tofSensor[2] = &tofSensor2;
  tofSensor[3] = &tofSensor3;

  // Initialise all tofSensor objects.
  for (uint8_t i=0; i<NUM_SENSOR; i++) {
    tofSensor[i]->setSendEventCallbackFunction(sendEventCallbackFunction);

    for (uint8_t j=0; j<NUM_THRESHOLD; j++) {
      // uint16_t value = NODECONFIG.read(EEADDR(sensor[i].threshold[j].threshold)); // need to use read16
      // uint8_t hysterisis = NODECONFIG.read(EEADDR(sensor[i].threshold[j].hysterisis)); // need to use read16
      uint16_t value = NODECONFIG.read16(EEADDR(sensor[i].threshold[j].threshold)); // need to use read16
      uint16_t hysterisis = NODECONFIG.read16(EEADDR(sensor[i].threshold[j].hysterisis)); // need to use read16
      uint16_t eventIndexNear = (i * NUM_THRESHOLD * 2) + (j * 2) + 0;
      uint16_t eventIndexFar = (i * NUM_THRESHOLD * 2) + (j * 2) + 1;
      tofSensor[i]->addThreshold(j, value, hysterisis, eventIndexNear, eventIndexFar);
    }

    // Need to do this after all thresholds have been added.
    tofSensor[i]->initialise(muxConnected);

    // If this sensor is connected update the CDI data.
    if (tofSensor[i]->sensorConnected) {
      NODECONFIG.put(EEADDR(sensor[i].sensordesc), ESTRING(String(tofSensor[i]->sensorType())));
    }
  }
}

// Was "NodeID nodeid(NODE_ADDRESS);" which was moved here for version 0.1.19.
// The actual value for Node ID is now set in setup() using data from Preferences or
// uses NODE_ADDRESS if not available in Preferences.
NodeID nodeid;

// The following #include needs nodeid to be already declared.
#include "OpenLCBMid.h"   // Essential - do not move or delete

// ==== Setup does initial configuration ======================
void setup() {
  Serial.begin(115200);

  // Delay to allow Serial port to be established.
  delay(1000);

  // temp for testing -- allows CoolTerm to be connected.
  delay(4000);

  Serial.printf("\n%6ld starting program", millis());
  Serial.printf("\n%6ld            Model: ", millis()); Serial.print(MODEL);
  Serial.printf("\n%6ld Software version: ", millis()); Serial.print(SWVERSION);
  Serial.printf("\n%6ld Compilation date: ", millis()); Serial.print(__DATE__);
  Serial.printf("\n%6ld Compilation time: ", millis()); Serial.print(__TIME__);

  // Create a ConfigurationOTA object and pass in the required parameters.
  ConfigurationOTA configurationOTA;
  configurationOTA.setCredentials(credentials); // A pointer to the credentials data in credentials.h
  configurationOTA.setTimeout(1000); // The 1000 mS timeout is used when connecting to one of potentially many WiFi hubs as not every WiFi hub may be available
  configurationOTA.setCurrentVersion(SWVERSION); // The currently running version of firmware
  configurationOTA.setDefaultNodeID(NodeID(NODE_ADDRESS)); // Used if a Node ID cannot be obtained

  // Connect to a WiFi hub, download the json configuration file and perform all configuration.
  configurationOTA.doConfiguration();

  // Update nodeid according to the Node ID stored in Preferences.
  // If there is no Node ID stored, then use the default of NODE_ADDRESS.
  nodeid = ConfigurationPreferences::getNodeID(NodeID(NODE_ADDRESS));

  // Initialise Olcb with the node id from Preferences.
  Olcb_init(nodeid, RESET_TO_FACTORY_DEFAULTS);

  initialiseRGBLEDs();
  initialiseToFSensors();

  Serial.printf("\n%6ld Initialisation finished", millis());
}

// ==== Loop ==========================
void loop() {
  // Do OpenLCB/LCC processing.
  Olcb_process();

  // Process any changes to sensor/threshold state and send the appropriate event if required.
  for (uint8_t i=0; i<NUM_SENSOR; i++) {
    tofSensor[i]->loop();
  }

  // connectHub(ConfigurationPreferences::getWiFiSSID(), ConfigurationPreferences::getWiFiPassword());

  /**
   * Connect to the OpenLCB/LCC hub and reconnect if contact has been lost.
   */
  if (hubConnectionMade(ConfigurationPreferences::getWiFiSSID(), ConfigurationPreferences::getWiFiPassword())) {
    ledConfigHubConnected = true; // Turn the blue LED on if configured.

    // This is required so that JMRI is initialised if JMRI starts after the node has started.
    sendInitialEvents();
  }

  if (hubConnectionLost(ConfigurationPreferences::getWiFiSSID(), ConfigurationPreferences::getWiFiPassword())) {
    ledConfigHubConnected = false; // Turn the blue LED off if configured.
  }

  /**
   * Control the LEDs.
   */
  digitalWrite(LED_RED, getLEDState(ledRedConfiguration));
  digitalWrite(LED_GREEN, getLEDState(ledGreenConfiguration));
  digitalWrite(LED_BLUE, getLEDState(ledBlueConfiguration));
}

/**
 * Returns LOW or HIGH to control one of the RGB LEDs.
 */
uint8_t getLEDState(int switchInput) {
  uint8_t ledState;

  switch (switchInput) {
    case LED_CONFIG_NOT_CONFIGURED:
      ledState = HIGH;
      break;
    case LED_CONFIG_HUB_STATE:
      ledState = ledConfigHubConnected ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_0_THRESHOLD_0:
      ledState = tofSensor[0]->isThresholdActive(0) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_0_THRESHOLD_1:
      ledState = tofSensor[0]->isThresholdActive(1) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_0_THRESHOLD_2:
      ledState = tofSensor[0]->isThresholdActive(2) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_0_THRESHOLD_3:
      ledState = tofSensor[0]->isThresholdActive(3) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_1_THRESHOLD_0:
      ledState = tofSensor[1]->isThresholdActive(0) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_1_THRESHOLD_1:
      ledState = tofSensor[1]->isThresholdActive(1) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_1_THRESHOLD_2:
      ledState = tofSensor[1]->isThresholdActive(2) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_1_THRESHOLD_3:
      ledState = tofSensor[1]->isThresholdActive(3) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_2_THRESHOLD_0:
      ledState = tofSensor[2]->isThresholdActive(0) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_2_THRESHOLD_1:
      ledState = tofSensor[2]->isThresholdActive(1) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_2_THRESHOLD_2:
      ledState = tofSensor[2]->isThresholdActive(2) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_2_THRESHOLD_3:
      ledState = tofSensor[2]->isThresholdActive(3) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_3_THRESHOLD_0:
      ledState = tofSensor[3]->isThresholdActive(0) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_3_THRESHOLD_1:
      ledState = tofSensor[3]->isThresholdActive(1) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_3_THRESHOLD_2:
      ledState = tofSensor[3]->isThresholdActive(2) ? LOW : HIGH;
      break;
    case LED_CONFIG_SENSOR_3_THRESHOLD_3:
      ledState = tofSensor[3]->isThresholdActive(3) ? LOW : HIGH;
      break;
  }
  
  return ledState;
}
