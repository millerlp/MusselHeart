#ifndef UserTypes_h
#define UserTypes_h
#include "Arduino.h"
// User data types.  Modify for your data items.


#define FILE_BASE_NAME "heartdata"
const uint8_t NUM_SENSORS = 4; 
struct data_t {
  uint32_t time;
  uint16_t heartSensors[NUM_SENSORS];
};
void acquireData(data_t* data);
void printData(Print* pr, data_t* data);
void printHeader(Print* pr);
void userSetup();
#endif  // UserTypes_h
