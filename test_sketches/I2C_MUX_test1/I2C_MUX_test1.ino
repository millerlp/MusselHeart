
/*  Working example for 2 sensors attached to daughterboard RevA & Teensy3.5
 *  Open Serial Plotter to see output from the 2 sensors. Sensor 1 should only
 *  have the red LED visible (+IR), Sensor 2 should have the green LED visible. 
 * 
 *  2020-06-20
 * 
 * 
 */

#include <Wire.h>
#include "MAX30105.h" // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "heartRate.h" // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library

#define TCAADDR 0x70
MAX30105 particleSensor;




void setup() {

  Wire.setSCL(19); // Teensy3.5 Wire SCL pin
  Wire.setSDA(18); // Teensy3.5 Wire SDA pin
  Wire.begin();

  // Set the TCA I2C multiplexer to channel 0
  tcaselect(0);
  
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }


    //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 3200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings


  // Enable 2nd sensor
  tcaselect(1);
  ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  Serial.print("Sensor1\tSensor2");
}

void loop() {
  tcaselect(0);
  Serial.print(particleSensor.getIR()); //Send raw data to plotter
  Serial.print("\t");
  tcaselect(1);
  Serial.println(particleSensor.getIR()); //Send raw data to plotter
}




void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
