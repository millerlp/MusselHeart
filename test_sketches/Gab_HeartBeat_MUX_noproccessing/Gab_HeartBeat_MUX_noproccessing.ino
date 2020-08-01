/* . GKalbach July 2020
       sketch written for Teensy 3.5 Heart Rate Rev B
       initial sketch goals are to iterate through all
       multiplexed sensors and write data to SD card

       not included in this sketch: processing of IR data

       desired improvements: file names with dates, OLED screen printing

*/

#include "MAX30105.h"    // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "heartRate.h"   // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include <TimeLib.h>     // https://github.com/PaulStoffregen/Time
#include <Wire.h>
#include <SD.h>          // Must use SD library from https://github.com/PaulStoffregen/SD for Teensy

MAX30105 particleSensor;
#define TCAADDR 0x70

// time components
TimeElements tm;
time_t myTime;
unsigned long millisVal;

//SD components
//#define FILE_BASE_NAME "HR_log" dont think i can use this since SD.h has a file name character limit
const int chipSelect = BUILTIN_SDCARD;
File myFile; //SD card object
char fileName[19];


void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for serial port to connect
  Wire.begin(); // initialize I2C communication

  //RTC setup
  setSyncProvider(getTeensy3Time); // tell TimeLib to use Teensy's RTC for timekeeping
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  delay(100);
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");
    digitalClockDisplay(myTime); Serial.println();
//  Serial.println(timeStatus());

  millisVal = millis();

  //Sensor setup
  for (byte i = 0; i < 7; i++) {
    tcaselect(i);
    delayMicroseconds(20);

    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) { // connect sensor to I2C bus

      // sensor configurations
      byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
      byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
      byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
      byte sampleRate = 3200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
      int pulseWidth = 411; //Options: 69, 118, 215, 411
      int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

      particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
      particleSensor.enableDIETEMPRDY(); //enable temp ready interrupt. Required to log temp
    }
  }

  //SD setup
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization Failed!");
    return;
  }
  Serial.println("Initialization done.");


  // Testing new SD Naming
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  digitalClockDisplay(myTime);
  // digital clock display of the time
  sprintf(fileName, "%4d%02d%02d.CSV", year(), month(), day());

  if (SD.exists(fileName)) {
    Serial.print("Opening ");
    Serial.println(fileName);
    Serial.println("Appending existing file.");
  }
  else {
    Serial.println("File does not exist. Creating new file.");
    myFile = SD.open(fileName, FILE_WRITE);
    Serial.print("Opening ");
    Serial.println(fileName);
  }

  // End of new testing

}

void loop() {
  if ((millis() - millisVal) > 1000) {
    millisVal = millis(); // update millisVal
  }
  // if the file is available, write to it
  if (SD.exists(fileName)) {
    myTime = Teensy3Clock.get();
    myFile.print(myTime); myFile.print(",");
    myFile.print(year()); myFile.print("-"); myFile.print(month()); myFile.print("-");myFile.print(day());
    myFile.print(" "); myFile.print(hour()); myFile.print(":");myFile.print(minute());myFile.print(":");myFile.print(second());
    myFile.print(",");

    // loop through sensors in sequence
    for (byte i = 0; i < 7; i++) {
      tcaselect(i);
      myFile.print(particleSensor.getIR());
      myFile.print(", ");
      myFile.print(particleSensor.readTemperature());
      myFile.print(", ");
      if (i > 5) { //start new line after the last sensor
        myFile.print(particleSensor.getIR());
        myFile.print(", ");
        myFile.print(particleSensor.readTemperature());
        myFile.println(", ");
      }
    }
  }
  else {
    Serial.println("error opening file.");
  }
 // LPM: Deleted a } here that was ending the loop early
myFile.flush(); // force a write to the SD card

delay(1000); // testing, feel free to delete this
}


//******************************
void tcaselect(uint8_t i) {
  if (i > 6) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
//******************************
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
//******************************
void digitalClockDisplay(time_t theTime) {
  // digital clock display of the time
  Serial.print(year(theTime));
  Serial.print(F("-")); 
  Serial.print(month(theTime));   
  Serial.print(F("-"));
  Serial.print(day(theTime));
  Serial.print(" ");
  Serial.print(hour(theTime));
  printDigits(minute(theTime));
  printDigits(second(theTime)); 
  Serial.println(); 
}
//****************************
void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
