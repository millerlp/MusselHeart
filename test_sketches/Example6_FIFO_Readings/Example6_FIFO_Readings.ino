/*
  MAX30105 Breakout: Take readings from the FIFO
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  Outputs all Red/IR/Green values at 25Hz by polling the FIFO

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
 
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
*/

#include <Wire.h>
#include "MAX30105.h"

#define TCAADDR 0x70 // I2C address for the I2C multiplexer chip

MAX30105 particleSensor;

long startTime;
long samplesTaken = 0; //Counter for calculating the Hz or read rate

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD) == false) //Use default I2C port, 100kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  // Define pins for the I2C multiplexer attached to IR sensors
  Wire.setSCL(19);
  Wire.setSDA(18);
  Wire.begin(); // initialize I2C communication for MUX & sensors

  //Setup to sense up to 18 inches, max LED brightness
  byte ledBrightness = 30; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 215; //Options: 69, 118, 215, 411
  int adcRange = 2048; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
//  particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive

  tcaselect(0); // Just probe the 1st channel

  startTime = millis();
}

void loop()
{
  particleSensor.check(); //Check the sensor, read up to 3 samples

  while (particleSensor.available()) //do we have new data?
  {
    samplesTaken++;

    Serial.print(" R[");
    Serial.print(particleSensor.getFIFORed());
    Serial.print("] IR[");
    Serial.print(particleSensor.getFIFOIR());
    Serial.print("] G[");
    Serial.print(particleSensor.getFIFOGreen());
    Serial.print("] Hz[");
    Serial.print((float)samplesTaken / ((millis() - startTime) / 1000.0), 2);
    Serial.print("]");

    Serial.println();

    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }
}



// Select the correct I2C channel on the I2C multiplexer
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
