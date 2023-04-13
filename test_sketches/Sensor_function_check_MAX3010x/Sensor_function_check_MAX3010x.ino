/* 2021-03-04 L Miller 
 * Scan for functional MAX3010x heart rate sensors and print results to
 * OLED screen. Tested with Heart Rate Daughterboard revB and Teensy 3.5. 
 * Good for checking functioning of newly built heart sensors
 * 
*/

#include <Wire.h>
#include "MAX30105.h" // https://github.com/millerlp/SparkFun_MAX3010x_Sensor_Library
#include "heartRate.h" // https://github.com/millerlp/SparkFun_MAX3010x_Sensor_Library
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii

#define TCAADDR 0x70  // I2C address for the TCA9548 I2C multiplexer, used to select MAX3010x sensors

// MAX3010x sensor settings
MAX30105 max3010x; // create MAX3010x object
#define MAX_SENSORS 8  // Leave this set at 8, even if fewer than 8 sensors are attached
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32. We use immediate read, so no point to averaging
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green. We use Red+IR for heart stuff
byte sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 215; //Options: 69, 118, 215, 411. Higher values = more sensitivity
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
// Define IR led brightness setting for each of the 8 channels
// Options: 0=off to 255=fully on, try 10-30 initially. Too high will make noisy signal
//          Channel =      1   2   3   4   5   6   7   8
byte IRledBrightness[] = {30, 30, 30, 30, 30, 30, 30, 30};
byte REDledBrightness = 1; // low value of 0 shuts it off, 1 is barely on
#define MULTIPLE_I2C_PORTS 1  // for ssd1306Ascii library, tell it there are multiple I2C ports available

SSD1306AsciiWire oled(Wire1); // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here

// Create an array to hold numbers of good sensor channels (up to 8)
byte goodSensors[] = {127,127,127,127,127,127,127,127};
byte numgoodSensors = 0;
unsigned int myIntervalMS = 50;  // units milliseconds
unsigned long myMillis;

//------------------------------------------------------------------------
void setup() {
  Serial.begin(57600); // Speed doesn't actually matter for Teensy3
  // Start the I2C instance on SCL0/SDA0 of the Teensy3.5 (connected to TCA9548 and heart sensors
  Wire.setSCL(19); // Teensy3.5 Wire SCL pin on daughterboard RevA
  Wire.setSDA(18); // Teensy3.5 Wire SDA pin on daughterboard RevA
  Wire.begin();
  
  Wire1.begin(); // Start I2C on SCL1/SDA1 of Teensy3.5, for the OLED display
  Wire1.setSCL(37);
  Wire1.setSDA(38);
  Wire1.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS1);  
  oled.setFont(Adafruit5x7);    
  oled.clear(); 
  oled.home();
  oled.set2X();
  oled.print("Hello");
  oled.println();
  delay(1000);

  //*******************************
  // MAX30105 Sensor setup. This function will scan for avaialble
  // sensors, and set sampling parameters for those that are found
  // This will also store which channels are working, so that only
  // those channels are sampled and stored in the data files
  scanSetupSensors(); // See function near bottom of this file
  
//  for (byte i = 0; i < 8; i++){
//      // Set the TCA I2C multiplexer to channel 0
//    tcaselect(i);
//    delayMicroseconds(20);
////    if (max3010x.begin(Wire, 400000)) // Fast I2C, only works with short sensor leads (<5m)
//    // Use default I2C port, I2C_SPEED_STANDARD (100kHz) to accomodate sensors
//    // on the end of very long wire leads (tested up to 10 meters)
//    if (max3010x.begin(Wire, I2C_SPEED_STANDARD)) 
//    {
//      // If sensor is present, mark it in the goodSensors array
//      goodSensors[i] = i;
//      numgoodSensors++;
//    } else {
//      delay(5);
//      // Try a second time to see if the sensor shows up
//      if(max3010x.begin(Wire, I2C_SPEED_STANDARD)){        
//        goodSensors[i] = i;
//        numgoodSensors++;
//      }
//    }
//
//    //Configure sensor with these initial settings
//    max3010x.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 
//    max3010x.disableDIETEMPRDY(); //disable temp ready interrupt.
//    // Tweak individual settings
//    max3010x.setPulseAmplitudeRed(0x01); // essentially turn off red LED to save power, we only want IR LED.
//    max3010x.setPulseAmplitudeIR(ledBrightness); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)
//    max3010x.setPulseWidth(pulseWidth); //Options: 69, 118, 215, 411. Higher values = more sensitivity
//    
//  }

  if (numgoodSensors == 0){
    oled.println("No sensors");
    oled.println("found");
    Serial.println("No sensors found");
    while(1);
  } else {
      // Print available sensors
    for (byte i = 0; i < 8; i++){
      if (goodSensors[i] != 127) {
        Serial.print("Sensor");
        Serial.print(goodSensors[i]+1); // Start labeling at 1 instead of 0
        Serial.print("\t");
      }
    }
    Serial.println();
    oled.print("Found ");
    oled.println(numgoodSensors);
    oled.println("sensors");
    delay(2000);
  }

  oled.clear(); 
  oled.home();
  oled.set1X();

  myMillis = millis();
}  // end of setup loop

//****** main loop ***********
void loop() {
  unsigned long currentMillis = millis(); // update currentMillis
  if ( (currentMillis - myMillis) >= myIntervalMS) {
      myMillis = currentMillis; // update myMillis
    for (byte i = 0; i < 8; i++){
      if (goodSensors[i] != 127) {
        tcaselect(i);
  //      uint32_t sensorVal = max3010x.getIR();
        max3010x.clearFIFO();
        while(max3010x.check() < 1){} // Idle here until a value is ready
        uint32_t sensorVal = quickSampleIR();
        Serial.print(sensorVal); //Send raw data to plotter
        Serial.print("\t");
        printSensorOLED(i, sensorVal); // Use function below to print sensor values to OLED screen
      } 
    }
    Serial.println();
  }

}



//**************************************
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

//****************************************
void printSensorOLED(uint8_t i, uint32_t sensorValue){
  // This is based on using a 5x7 font in 1x mode (each character is 5 columns (pixels) wide)
  switch(i){
    case 0:
    oled.clearField(0,0,55); // Clear 55 pixels = 11 characters x 5 pixels per character)
    oled.setCursor(0,0);
    oled.print("1: ");
    oled.print(sensorValue);
    break;
    case 1:
    oled.clearField(65,0,55); // start at 65th column (13 characters x 5 pixels per character)
    oled.setCursor(65,0);
    oled.print("2: ");
    oled.print(sensorValue);
    break;  
    case 2:
    oled.clearField(0,2,55);
    oled.setCursor(0,2);
    oled.print("3: ");
    oled.print(sensorValue);
    break;
    case 3:
    oled.clearField(65,2,55);
    oled.setCursor(65,2);
    oled.print("4: ");
    oled.print(sensorValue);
    break;  
    case 4:
    oled.clearField(0,4,55);
    oled.setCursor(0,4);
    oled.print("5: ");
    oled.print(sensorValue);
    break;
    case 5:
    oled.clearField(65,4,55);
    oled.setCursor(65,4);
    oled.print("6: ");
    oled.print(sensorValue);
    break;
    case 6:
    oled.clearField(0,6,55);
    oled.setCursor(0,6);
    oled.print("7: ");
    oled.print(sensorValue);
    break;
    case 7:
    oled.clearField(65,6,55);
    oled.setCursor(65,6);
    oled.print("8: ");
    oled.print(sensorValue);
    break;
  }
  
}

//-----------------------------------------------
//-------------------------------------------------------------
// A function to scan for available MAX30105 sensors
// Saves a record of which sensors returned a valid signature (goodSensors[])
// Saves a count of the number of good sensors (numgoodSensors)
// Sets up each sensor using the user's parameters defined at the top of this program

void scanSetupSensors (void) {
  numgoodSensors = 0; // reset to zero each time this function is called
  for (byte i = 0; i < MAX_SENSORS; i++) {
    tcaselect(i);
    delayMicroseconds(20);
    goodSensors[i] = 127; // Reset this value before scanning for the sensor
    if (max3010x.begin(Wire, I2C_SPEED_STANDARD)) //Use default I2C port, 100kHz speed
    {
      // If sensor is present, mark it in the goodSensors array
      goodSensors[i] = i;
      numgoodSensors++;
    } else {
      // If sensor didn't show up, wait a bit and try a 2nd time
      delay(5);
      if (max3010x.begin(Wire, I2C_SPEED_STANDARD)) {
        goodSensors[i] = i;
        numgoodSensors++;
      }
    }
    // If the sensor was marked good, set it up for our sampling needs
    if (goodSensors[i] != 127) {
      max3010x.setup(IRledBrightness[i], sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
      max3010x.enableDIETEMPRDY(); //enable temp ready interrupt. Required to log temp, but each read takes 29ms
      // Tweak individual settings
      max3010x.setPulseAmplitudeRed(REDledBrightness); // essentially turn off red LED to save power, we only want IR LED. **** commented for testing only
      max3010x.setPulseAmplitudeIR(IRledBrightness[i]); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)
      // TEST =  disable all Slots, then enable IR led on Slot 1
//      max3010x.disableSlots();
//      max3010x.enableSlot(1, 0x02);  // 0x02 represents the IR LED (LED2, page 21 of MAX30102 data sheet)
//      max3010x.enableSlot(2, 0x00); // 0x00 means shut off the LED
    }
  }
}


//---------------------------------------------------------------------------------------------

// Custom sampling function for the MAX30105, calling functions in the MAX30105.h library
uint32_t quickSampleIR(void) {
  //  digitalWriteFast(IRDELAYPIN, HIGH);  // troubleshooting, can comment out
  // Clear the MAX30105 FIFO buffer so that there will only be one new sample to read
//  max3010x.clearFIFO();
  uint16_t ledSampleTime; // units will be microseconds

  if (sampleAverage > 1) {

    // Implement a delay for new samples to be collected in the FIFO. If sample averaging
    // is used, a loop will need to execute multiple times to allow the multiple samples
    // to be collected
    for (int avg = 0; avg < sampleAverage; avg++) {
      // Now add in the delay for the actual LED flashes to happen
      switch (pulseWidth) {
        // options for pulseWidth: 69, 118, 215, 411 microseconds
        case 69:
          ledSampleTime = pulseWidth + 358 + pulseWidth;
          delayMicroseconds(ledSampleTime);
          break;
        case 118:
          ledSampleTime = pulseWidth + 407 + pulseWidth;
          delayMicroseconds(ledSampleTime);
          break;
        case 215:
          ledSampleTime = pulseWidth + 505 + pulseWidth;
          delayMicroseconds(ledSampleTime);
          break;
        case 411:
          ledSampleTime = pulseWidth + 696 + pulseWidth;
          delayMicroseconds(ledSampleTime);
          break;
        default:
          ledSampleTime = pulseWidth + 505 + pulseWidth; // use 215 pulsewidth as default
          delayMicroseconds(ledSampleTime);
          break;
      }
      // If more than one sample is being averaged, you need to
      // further wait for the start of the next sampling cycle,
      // which is determined by the sampleRate and how long it just
      // took for the red and IR leds to be sampled. This won't
      // execute on the last sample of the average because it isn't
      // needed after the IR LED is read for the last time
        if (avg < (sampleAverage - 1)) {
          switch (sampleRate) {
            case 50:
              delayMicroseconds(20000 - ledSampleTime);
              break;
            case 100:
              delayMicroseconds(10000 - ledSampleTime);
              break;
            case 200:
              delayMicroseconds(5000 - ledSampleTime);
              break;
            case 400:
              delayMicroseconds(2500 - ledSampleTime);
              break;
            case 800:
              delayMicroseconds(1250 - ledSampleTime);
              break;
            case 1000:
              delayMicroseconds(1000 - ledSampleTime);
              break;
            case 1600:
              delayMicroseconds(625 - ledSampleTime);
              break;
            case 3200:
              delayMicroseconds(313 - ledSampleTime);
              break;
            default:
              delayMicroseconds(5000 - ledSampleTime);
              break;
          } // end of switch statement
      } // end of if statement
    } // end up of delay loop
  } else {
    // If we're not averaging, just delay a bit for the red + IR pulses to happen
    delayMicroseconds( (pulseWidth * 2 + 696) ) ;
  }
  delayMicroseconds(50); // Add in a little extra delay just to be safe
  // Query the FIFO buffer on the sensor for the most recent IR value
  //  digitalWriteFast(IRDELAYPIN, LOW); // troubleshooting, can comment out
  //  digitalWriteFast(IRPIN, HIGH); // troubleshooting, can comment out
//  uint32_t tempIR = max3010x.getFIFOIR();
  // If sampleAverage is >1, the sensor should be internally averaging the
  // readings, and this getIR() function will return the most recent averaged value
  uint32_t tempIR = max3010x.getIR();
  //  digitalWriteFast(IRPIN, LOW);  // troubleshooting, can comment out
  return (tempIR);

} // end of quickSampleIR function
