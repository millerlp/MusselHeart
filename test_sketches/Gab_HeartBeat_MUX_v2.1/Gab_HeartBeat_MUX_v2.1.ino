
/* . GKalbach July 2020
 *    updated 2020-08-01
 *    
       sketch written for Teensy 3.5 Heart Rate Rev B
       initial sketch goals are to iterate through all
       multiplexed sensors and write data to SD card

       desired improvements:  OLED screen printing

*/

#include "MAX30105.h"         // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "heartRate.h"        // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "SSD1306Ascii.h"     // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <TimeLib.h>          // https://github.com/PaulStoffregen/Time
#include <Wire.h>
#include <SD.h>               // Must use SD library from https://github.com/PaulStoffregen/SD for Teensy

#define NUM_SENSORS 8  // Change this if you have fewer than 8 sensors attached, to skip over higher unused channels

MAX30105 particleSensor;
#define TCAADDR 0x70

// time components
TimeElements tm;
time_t myTime;
//unsigned long millisVal;  // not being used currently that I can see

unsigned int myIntervalMS = 50;  // units milliseconds
unsigned long myMillis;
int myCounter;

//SD components
//#define FILE_BASE_NAME "HR_log" dont think i can use this since SD.h has a file name character limit
const int chipSelect = BUILTIN_SDCARD;
File myFile; //SD card object
char fileName[19];

// OLED components
#define SCREEN_WIDTH 128 // OLED pixel width
#define SCREEN_HEIGHT 64 // OLED pixel height
#define OLED_RESET     4 // OLED reset pin num
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

//  millisVal = millis();  // not necessary, millisVal isn't currently being used for anything
  myMillis = millis();
  myCounter = 0;

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


  // SD Naming
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
    // file header
    if (SD.exists(fileName)) {
      printFileHeader(); // see function printFileHeader() at bottom of this file
    }
  }
}

void loop() {

  // This if statement will update millisVal whenever 1000 milliseconds have elapsed (1 second)
  // but the millisVal value isn't getting used anywhere else? So I don't think it's necessary
  
//  if ((millis() - millisVal) > 1000) {
//    millisVal = millis(); // update millisVal
//  }

  // This if statement below will be true whenever the current millis() value is more than myIntervalMS
  // larger than myMillis
  if ( (millis() - myMillis) >= myIntervalMS) {
    // Always update myMillis whenever the if statement successfully executes so that the next
    // time around the if statement is comparing the time difference to this current time at the
    // start of the sampling cycle
    myMillis = millis();
    myCounter++; // increment here, since we want to mark how many times (20 times) we go through
    // the sampling cycle, rather than how many times each sensor gets sampled
    // if the file is available, write to it
    if (SD.exists(fileName)) {
      myTime = Teensy3Clock.get();
      myFile.print(myMillis); myFile.print(","); // keep track of start of sampling round
      myFile.print(myTime); myFile.print(",");
      myFile.print(year()); myFile.print("-"); myFile.print(month()); myFile.print("-"); myFile.print(day());
      myFile.print(" "); myFile.print(hour()); myFile.print(":"); myFile.print(minute()); myFile.print(":"); myFile.print(second());
      myFile.print(",");

      // loop through sensors in sequence. If you have fewer than 8 sensors attached, change the value 
      // of NUM_SENSORS at the top of this file, and make sure the sensors are plugged into the lower
      // number ports available (1,2,3 etc.) rather than skipping over ports and plugging into higher
      // port numbers (7, 8 etc.)
      for (byte i = 0; i < NUM_SENSORS; i++) {
        tcaselect(i);
        myFile.print(particleSensor.getIR());
        myFile.print(", ");
        myFile.print(particleSensor.readTemperature());
        if (i < (NUM_SENSORS-1)) {
          myFile.print(", ");
//          myCounter++; // counting here would increment by 1 each time a single sensor gets read, rather
                          // than when the whole set of sensors gets read (i.e. it counts too fast here)
        }
        if (i >= (NUM_SENSORS-1) ) { //start new line after the last sensor
          myFile.print(", ");  // Modified to record millis value at end of cycle
          myFile.println(millis()); // Record the end time of the sampling cycle
//          myCounter++; // Don't need to increment this here
        }
      }
    }
    else {
      Serial.println("error opening file.");
    }
    //myFile.flush(); // force a write to the SD card every second
    //Serial.println("Data written to SD card.");
  }
//  myMillis = millis(); // do this up at the start of the if statement, so that we're always
  // counting from the start of the last sampling round. Sampling rounds can vary in duration
  // so we don't want to count from the end of the sampling round, because that time varies. 

  // I'm not sure this if statement below is necessary in practice. The SD library should take care of 
  // flushing data to the card on its own whenever it accumulates enough data. You'd only want this
  // to force a flush if you really want to be sure a flush happens on a regular (frequent) interval, 
  // which isn't really necessary. 
  if (myCounter > 19) {
    myFile.flush(); // force a write to the SD card every second
    Serial.println("Data written to SD card. myCounter reset.");
    myCounter = 0;
  }

}  // end of main loop


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
void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


//******************************
// Function to print the file header row. This will only print as many sensor columns as 
// there are sensors (defined by the variable NUM_SENSORS). 
// If you have fewer than 8 sensors on the board,
// make sure they are all plugged in to the lowest available port numbers (1, 2, 3 etc.), 
// rather than skipping over ports and plugging into ports 7 or 8 etc. 
void printFileHeader() {
      myFile.print("millis,");  // LPM added this
      myFile.print("UnixTime"); myFile.print(",");
      myFile.print("DateTime"); myFile.print(",");
      for (int i = 1; i <= NUM_SENSORS; i++) {
        myFile.print("Sensor");
        myFile.print(i);
        myFile.print("IR,Sensor");
        myFile.print(i);
        myFile.print("TempC,");    
      }
      myFile.println("endMillis"); // LPM added this
}
