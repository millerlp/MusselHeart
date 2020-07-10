
//#include <i2c_t3.h>  // Special I2C lib for Teensy3 https://github.com/nox771/i2c_t3
#include <Wire.h>
#include "MAX30105.h" // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "heartRate.h" // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii


#define TCAADDR 0x70
MAX30105 particleSensor;
SSD1306AsciiWire oled; // create OLED display object, using I2C Wire
#define I2C_ADDRESS1 0x3C // for oled

// Create an array to hold numbers of good sensor channels (up to 8)
byte goodSensors[] = {127,127,127,127,127,127,127,127};


void setup() {
  Serial.begin(57600); // Speed doesn't actually matter for Teensy3
  Wire.setSCL(19); // Teensy3.5 Wire SCL pin on daughterboard RevA
  Wire.setSDA(18); // Teensy3.5 Wire SDA pin on daughterboard RevA
  Wire.begin();
  Wire1.begin();
  oled.begin(&Adafruit128x64, I2C_ADDRESS1);
  oled.set400kHz();  
  oled.setFont(Adafruit5x7);    
  oled.clear(); 
  oled.print(F("Hello"));

  for (byte i = 0; i < 8; i++){
      // Set the TCA I2C multiplexer to channel 0
    tcaselect(i);
    delayMicroseconds(20);
//    if (particleSensor.begin(Wire, 400000))
    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    {
      // If sensor is present, mark it in the goodSensors array
      goodSensors[i] = i;
    }
        //Setup to sense a nice looking saw tooth on the plotter
    byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
    byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 215; //Options: 69, 118, 215, 411
    int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings


  }

  // Print available sensors
  for (byte i = 0; i < 8; i++){
    if (goodSensors[i] != 127) {
      Serial.print("Sensor");
      Serial.print(goodSensors[i]+1);
      Serial.print("\t");
    }
  }
  Serial.println();

  
      // Setup for Master mode, pins 18/19, external pullups, 400kHz, 200ms default timeout
//    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
//    Wire.setDefaultTimeout(200000); // 200ms
    // For Teensy3.5 Wire, Wire1, Wire2 available


//  Serial.print("Sensor1\tSensor2");
}

void loop() {

  for (byte i = 0; i < 8; i++){
    if (goodSensors[i] != 127) {
      tcaselect(i);
      Serial.print(particleSensor.getIR()); //Send raw data to plotter
      Serial.print("\t");
    } 
  }
  Serial.println();
}




void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
