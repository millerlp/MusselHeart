/* 2020-07-09 L Miller
 * Basic demonstration of reading multiple MAX30105 sensors and outputting
 * raw values to the OLED display screen (and serial monitor). Tested with
 * heart rate daughterboard revB and Teensy 3.5.
 * 
*/

#include <Wire.h>
#include "MAX30105.h" // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "heartRate.h" // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii


#define TCAADDR 0x70  // I2C address for the TCA9548 I2C multiplexer
MAX30105 particleSensor; // create MAX30105 object called particleSensor

#define MULTIPLE_I2C_PORTS 1  // for ssd1306Ascii library, tell it there are multiple I2C ports available

SSD1306AsciiWire oled(Wire1); // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here

// Create an array to hold numbers of good sensor channels (up to 8)
byte goodSensors[] = {127,127,127,127,127,127,127,127};


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
  
  for (byte i = 0; i < 8; i++){
      // Set the TCA I2C multiplexer to channel 0
    tcaselect(i);
    delayMicroseconds(20);
//    if (particleSensor.begin(Wire, 400000))
    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    {
      // If sensor is present, mark it in the goodSensors array
      goodSensors[i] = i;
    } else {
      delay(5);
      if(particleSensor.begin(Wire, I2C_SPEED_FAST)){
        // Try a second time
        goodSensors[i] = i;
      }
    }
    // Setup to sense a nice looking saw tooth on the plotter
    byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
    byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 215; //Options: 69, 118, 215, 411
    int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings


  }

  oled.clear(); 
  oled.home();
  oled.set1X();
  // Print available sensors
  for (byte i = 0; i < 8; i++){
    if (goodSensors[i] != 127) {
      Serial.print("Sensor");
      Serial.print(goodSensors[i]+1); // Start labeling at 1 instead of 0
      Serial.print("\t");
      
    }
  }
  Serial.println();




}

void loop() {

  for (byte i = 0; i < 8; i++){
    if (goodSensors[i] != 127) {
      tcaselect(i);
      uint32_t sensorVal = particleSensor.getIR();
      Serial.print(sensorVal); //Send raw data to plotter
      Serial.print("\t");
      printSensorOLED(i, sensorVal); // Use function below to print sensor values to OLED screen
    } 
  }
  Serial.println();
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
    oled.clearField(0,0,30); // Clear 30 pixels = 6 characters x 5 pixels per character)
    oled.setCursor(0,0);
    oled.print(sensorValue);
    break;
    case 1:
    oled.clearField(40,0,30); // start at 40th column (8 characters x 5 pixels per character)
    oled.setCursor(40,0);
    oled.print(sensorValue);
    break;  
    case 2:
    oled.clearField(0,2,30);
    oled.setCursor(0,2);
    oled.print(sensorValue);
    break;
    case 3:
    oled.clearField(40,2,30);
    oled.setCursor(40,2);
    oled.print(sensorValue);
    break;  
    case 4:
    oled.clearField(0,4,30);
    oled.setCursor(0,4);
    oled.print(sensorValue);
    break;
    case 5:
    oled.clearField(40,4,30);
    oled.setCursor(40,4);
    oled.print(sensorValue);
    break;
    case 6:
    oled.clearField(0,6,30);
    oled.setCursor(0,6);
    oled.print(sensorValue);
    break;
    case 7:
    oled.clearField(40,6,30);
    oled.setCursor(40,6);
    oled.print(sensorValue);
    break;
  }
  
}
