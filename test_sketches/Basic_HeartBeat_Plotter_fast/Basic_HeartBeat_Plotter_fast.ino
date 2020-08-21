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
 * wakeup functions for the MAX30105. 
 * 

*/

#include <Wire.h>
#include "MAX30105.h"

#define NUM_SENSORS 1
#define QUICKSAMPLE 1// Set to 1 to enable quick sample, set to 0 to disable quick sample
#define SLEEP  // Comment this out to disable sleep routine


MAX30105 particleSensor;
// sensor configurations
byte ledBrightness = 0x1F; //Options: 0=Off to 255=fully on
byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
int sampleRate = 1000; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411, units microseconds. Applies to all active LEDs
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

#define TCAADDR 0x70

unsigned int myIntervalMS = 50;  // units milliseconds
unsigned long myMillis;

void setup()
{
  Serial.begin(115200);
  while(!Serial); // wait for serial port to connect
  Wire.setSCL(19); // Teensy3.5 Wire SCL pin on daughterboard RevA
  Wire.setSDA(18); // Teensy3.5 Wire SDA pin on daughterboard RevA
  Wire.begin();

  //Sensor setup
  for (byte i = 0; i < NUM_SENSORS; i++) {
    tcaselect(i);
    delayMicroseconds(20);

    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) { // connect sensor to I2C bus
      particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
//      particleSensor.enableDIETEMPRDY(); //enable temp ready interrupt. Required to log temp, but each read takes 29ms
//      particleSensor.disableDIETEMPRDY(); //disable temp ready interrupt.
      // Tweak individual settings
      particleSensor.setPulseAmplitudeRed(0x00); // essentially turn off red LED to save power, we only want IR LED. **** commented for testing only
      particleSensor.setPulseAmplitudeIR(ledBrightness); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)
//      particleSensor.setPulseWidth(pulseWidth); //DANGER - this seems to screw things up! Options: 69, 118, 215, 411. Higher values = more sensitivity
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

    for (byte i = 0; i < NUM_SENSORS; i++) {
        tcaselect(i); // choose sensor channel

        // This section only executes if the #define SLEEP line is uncommented above
#ifdef SLEEP        
        particleSensor.wakeUp(); // Wake up sensor to take sample
        delay(40); // Give the chip a chance to wake up
#endif        

      // This section uses either the quicksample function or traditional getIR() function
      // depending on the value assigned to #define QUICKSAMPLE up above
#if (QUICKSAMPLE == 1)        
        Serial.print(quickSampleIR()); // custom function, see bottom of this file. This waits long enough to gather one sample
#elif (QUICKSAMPLE == 0)
        Serial.print(particleSensor.getIR()); // The slow way
#endif

        // This section only executes if the #define SLEEP line is uncommented above
#ifdef SLEEP
          particleSensor.shutDown(); // shut down sensor once sample is taken to save power
#endif

        Serial.println();
      }

  }

}  // end of main loop


//******************************
void tcaselect(uint8_t i) {
  if (i > NUM_SENSORS) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

//-----------------------------------------------
// Custom sampling function for the MAX30105, calling functions in the MAX30105.h library
uint32_t quickSampleIR(void) {
  // Clear the MAX30105 FIFO buffer so that there will only be one new sample to read
  particleSensor.clearFIFO();
  // Implement a delay for new samples to be collected in the FIFO. If sample averaging
  // is used, this loop will need to execute multiple times to allow the multiple samples
  // to be collected
  for (int i = 0; i < sampleAverage; i++){
    switch (sampleRate) {
      case 50:
        delay(20);
        break;
      case 100:
        delay(10);
        break;
      case 200:
        delay(5);
        break;
      case 400:
        delayMicroseconds(2500);
        break;
      case 800:
        delayMicroseconds(1250);
        break;
      case 1000:
        delay(1);
        break;
      case 1600:
        delayMicroseconds(625);
        break;
      case 3200:
        delayMicroseconds(313);
        break;
    }
    // Now add in the delay for the actual LED flashes to happen
    switch (pulseWidth) {
      // 69, 118, 215, 411
      case 69:
        delayMicroseconds( pulseWidth + 427 + 50);
        break;
      case 118:
        delayMicroseconds( pulseWidth + 525 + 50);
        break;
      case 215:
        delayMicroseconds( pulseWidth + 720 + 50);
        break;
      case 411:
        delayMicroseconds( pulseWidth + 1107 + 50);
        break;
    }
  } // end up of delay loop
  // Query the FIFO buffer on the sensor for the most recent IR value
  return (particleSensor.getIR());
} // end of quickSampleIR function
