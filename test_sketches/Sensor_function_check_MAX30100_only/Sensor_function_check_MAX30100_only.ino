/* 2023-01-18 L Miller
 * Scan for functional MAX30100 (not 30105 or 30101) heart rate sensors and print results to
 * OLED screen. Tested with Heart Rate Daughterboard revB and Teensy 3.5. 
 * Good for checking functioning of newly built heart sensors
 * 
*/

#include <Wire.h>
#include "MAX30100.h" // https://github.com/millerlp/Arduino-MAX30100
//#include "heartRate.h" // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii


#define TCAADDR 0x70  // I2C address for the TCA9548 I2C multiplexer
MAX30100 heartSensor; // create MAX30100 object called heartSensor

// Particle sensor settings
//byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA (full on)
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32. We use immediate read, so no point to averaging
//byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green. We use Red+IR for heart stuff
byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 215; //Options: 69, 118, 215, 411. Higher values = more sensitivity
//int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

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
  
  for (byte i = 0; i < 1; i++){
//  for (byte i = 0; i < 8; i++){
      // Set the TCA I2C multiplexer to channel 0
    tcaselect(i);
    delayMicroseconds(20);
//    if (particleSensor.begin(Wire, 400000)) // Fast I2C, only works with short sensor leads (<5m)
    // Use default I2C port, I2C_SPEED_STANDARD (100kHz) to accomodate sensors
    // on the end of very long wire leads (tested up to 10 meters)
    if ( heartSensor.begin() ) 
    {
      // If sensor is present, mark it in the goodSensors array
      goodSensors[i] = i;
      numgoodSensors++;
    } else {
      delay(5);
      // Try a second time to see if the sensor shows up
      if( heartSensor.begin() )
      {        
        goodSensors[i] = i;
        numgoodSensors++;
      }
    }

    //Configure sensor with these initial settings
    heartSensor.setLedsPulseWidth(MAX30100_SPC_PW_1600US_16BITS);
    heartSensor.setSamplingRate(MAX30100_SAMPRATE_50HZ); // Slowest rate is 50Hz
    heartSensor.setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_0MA ); // (IR current, Red LED current)
    heartSensor.setHighresModeEnabled(false);
    
    // Tweak individual settings
//    particleSensor.setPulseAmplitudeRed(0x01); // essentially turn off red LED to save power, we only want IR LED.
//    particleSensor.setPulseAmplitudeIR(ledBrightness); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)
//    particleSensor.setPulseWidth(pulseWidth); //Options: 69, 118, 215, 411. Higher values = more sensitivity
    
  }

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
    for (byte i = 0; i < 1; i++){ // for testing only channel 0
//    for (byte i = 0; i < 8; i++){
      if (goodSensors[i] != 127) {
        tcaselect(i);
  //      uint32_t sensorVal = particleSensor.getIR();
//        uint32_t sensorVal = quickSampleIR();  // 32-bit number for MAX30105, MAX30101
        uint16_t sensorVal = quickSampleIR(); // 16-bit number for MAX30100
        Serial.print(millis()); Serial.print("\t");
        Serial.print(sensorVal); //Send raw data to plotter
        Serial.print("\t");
//        printSensorOLED(i, sensorVal); // Use function below to print sensor values to OLED screen
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
// Custom sampling function for the MAX30100, calling functions in the MAX30100.h library
uint16_t quickSampleIR(void) {
  uint16_t ir, red;
  // Clear the MAX30100 FIFO buffer so that there will only be one new sample to read
  heartSensor.resetFifo(); 
  // Multiply pulseWidth by 2 because we always have to wait for the Red LED to sample first
  // before the IR LED gets sampled. Then account for time taken for any sample averages, and
  // add on a buffer of 50 more microseconds just for safety's sake
  delayMicroseconds( (pulseWidth * 2 * sampleAverage) + 50) ;
  heartSensor.update(); // get a new reading
  heartSensor.getRawValues(&ir, &red);  // extract the most recent reading
  return(ir); // only return the IR channel value
} // end of quickSampleIR function
