/*
 * Use this to visualize a MAX30105 heart rate sensor
 * attached to a Teensy3.5 via the heart rate daughterboard RevB
 * 
 * Below you can change the value of QUICKSAMPLE to 0 or 1, where
 * 1 implements the quick sample routine that clears FIFO before taking
 * a reading, and 0 just reads out the entire FIFO each time and reports the
 * last most recent value. Both are only executing every 50ms, so in this
 * sketch quick sampling isn't actually any faster than non-quick sample. 
 * 
 * You can also uncomment the #define SLEEP line to enable the shutdown and
 * wakeup functions for the MAX30105. Currently these seem to bork the data
 * from the sensor, just giving oscillating values. 

*/

#include <Wire.h>
#include "MAX30105.h"

#define NUM_SENSORS 1
#define QUICKSAMPLE 1// Set to 1 to enable quick sample, set to 0 to disable quick sample
//#define SLEEP  // Comment this out to disable sleep routine


MAX30105 particleSensor;
// sensor configurations
byte ledBrightness = 0x1F; //Options: 0=Off to 255=fully on
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411, units microseconds. Applies to all active LEDs
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

#define TCAADDR 0x70

unsigned int myIntervalMS = 50;  // units milliseconds
unsigned long myMillis;

void setup()
{
  Serial.begin(115200);
  while(!Serial); // wait for serial port to connect
  Wire.begin();

  //Sensor setup
  for (byte i = 0; i < NUM_SENSORS; i++) {
    tcaselect(i);
    delayMicroseconds(20);

    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) { // connect sensor to I2C bus
      particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
//      particleSensor.enableDIETEMPRDY(); //enable temp ready interrupt. Required to log temp, but each read takes 29ms
      particleSensor.disableDIETEMPRDY(); //disable temp ready interrupt.
      // Tweak individual settings
      //particleSensor.setPulseAmplitudeRed(0x00); // essentially turn off red LED to save power, we only want IR LED. **** commented for testing only
      particleSensor.setPulseAmplitudeIR(ledBrightness); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)
      particleSensor.setPulseWidth(pulseWidth); //Options: 69, 118, 215, 411. Higher values = more sensitivity
//      triggerTemperatureSample(); // Start the temperature sample (wait 29 ms before attempting to read)
    }
  }
  //Arduino plotter auto-scales annoyingly. To get around this, pre-populate
  //the plotter with 500 of an average reading from the sensor

  //Take an average of IR readings at power up
  const byte avgAmount = 64;
  long baseValue = 0;
  for (byte x = 0 ; x < avgAmount ; x++)
  {
    baseValue += particleSensor.getIR(); //Read the IR value
  }
  baseValue /= avgAmount;

  //Pre-populate the plotter so that the Y scale is close to IR values
  for (int x = 0 ; x < 500 ; x++)
    Serial.println(baseValue);

  myMillis = millis();  
}

//----------------------------------------------------
void loop()
{
  if ( (millis() - myMillis) >= myIntervalMS) {
    myMillis = millis();
//    Serial.println(particleSensor.getIR()); //Send raw data to plotter
    for (byte i = 0; i < NUM_SENSORS; i++) {
        tcaselect(i);
#ifdef SLEEP        
        particleSensor.wakeUp(); // Wake up sensor to take sample
        delay(40); // Give the chip a chance to wake up
#endif        

#if (QUICKSAMPLE == 1)        
        Serial.print(quickSampleIR()); // custom function, see bottom of this file. This waits long enough to gather one sample
#elif (QUICKSAMPLE == 0)
        Serial.print(particleSensor.getIR()); // The slow way
#endif
        
#ifdef SLEEP
          particleSensor.shutDown(); // shut down sensor once sample is taken to save power
#endif

        Serial.println();
      }

   
  }

}


//-----------------------------------------------
// Custom sampling function for the MAX30105, calling functions in the MAX30105.h library
uint32_t quickSampleIR(void) {
  // Clear the MAX30105 FIFO buffer so that there will only be one new sample to read
  particleSensor.clearFIFO();
  // Multiply pulseWidth by 2 because we always have to wait for the Red LED to sample first
  // before the IR LED gets sampled. Also account for time taken for any sample averages.
  // Then add on 696us for the delay between two LED channels ("slot timing", see datasheet Table 14)
  // and add on a buffer of 50 more microseconds just for safety's sake
  delayMicroseconds( (pulseWidth * 2 * sampleAverage) + 696 + 50) ;
  return (particleSensor.getIR());
} // end of quickSampleIR function


//******************************
void tcaselect(uint8_t i) {
  if (i > NUM_SENSORS) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


//---------------------------------------------
// Set up to read Die Temperature
// After calling this function, wait at least 29 milliseconds before
// querying for a temperature value with the readTemperatureSample() function
void triggerTemperatureSample(void) {
  // Remember: DIE_TEMP_RDY interrupt must be enabled to read temperatures
  //See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19
  // MAX30105 I2C address is 0x57 (always)
  // MAX30105_DIETEMPCONFIG register address is 0x21
  // And send 0x01 to enable a single temperature read
  particleSensor.writeRegister8(0x57, 0x21, 0x01);
}

//-----------------------------------
// Check for new die temperature reading
// This function should be run at least 29 milliseconds after calling
// the triggerTemperatureSample() function which starts the temperature sample

float readTemperatureSample(void) {
  // Read die temperature register (integer)
  int8_t tempInt = particleSensor.readRegister8(0x57, 0x1F);
  uint8_t tempFrac = particleSensor.readRegister8(0x57, 0x20); //Causes the clearing of the DIE_TEMP_RDY interrupt

  // Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}
