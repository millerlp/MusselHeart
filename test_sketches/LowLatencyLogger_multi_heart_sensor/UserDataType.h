#ifndef UserDataType_h
#define UserDataType_h
const uint8_t NUM_SENSORS = 4;
struct data_t {
  unsigned long time;
  unsigned short heartSensors[NUM_SENSORS];
};
#endif  // UserDataType_h
