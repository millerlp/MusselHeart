#include "UserTypes.h"
#include "MAX30105.h" // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "heartRate.h" // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library

// User data functions.  Modify these functions for your data items

// Create an array to hold numbers of good sensor channels (up to 8)
byte goodSensors[] = {127,127,127,127,127,127,127,127};
// Start time for data
static uint32_t startMicros;


#define TCAADDR 0x70
MAX30105 particleSensor;

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// Acquire a data record.
void acquireData(data_t* data) {
  data->time = micros();
  for (byte i = 0; i < NUM_SENSORS; i++){
    if (goodSensors[i] != 127) {
      tcaselect(i);
      data->heartSensors[i] = particleSensor.getIR(); // Read the sensor
    } else {
      // If no sensor on this channel, just record a zero
      data->heartSensors[i] = 0;
    }
  }
}

// Print a data record.
void printData(Print* pr, data_t* data) {
  if (startMicros == 0) {
    startMicros = data->time;
  }
  pr->print(data->time - startMicros);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pr->write(',');
    pr->print(data->heartSensors[i]);
  }
  pr->println();
}

// Print data header.
void printHeader(Print* pr) {
  startMicros = 0;
  pr->print(F("micros"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    pr->print(F(",Sensor"));
    pr->print(i);
  }
  pr->println();
}

// Sensor setup
void userSetup() {
  Wire.setSCL(19); // Teensy3.5 Wire SCL pin on daughterboard RevA
  Wire.setSDA(18); // Teensy3.5 Wire SDA pin on daughterboard RevA
  Wire.begin();
  // Scan for heart rate sensors
  for (byte i = 0; i < 8; i++){
    // Set the TCA I2C multiplexer to channel i
    tcaselect(i);
    delayMicroseconds(20);
    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    {
      // If sensor is present, mark it in the goodSensors array
      goodSensors[i] = i;
    }
  
    byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
    byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 215; //Options: 69, 118, 215, 411
    int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  } // end of initial sensor scan

  // Print available sensors
  for (byte i = 0; i < 8; i++){
    if (goodSensors[i] != 127) {
      Serial.print("Sensor");
      Serial.print(goodSensors[i]+1);
      Serial.print("\t"); 
    }
  }
  Serial.println();
}
