
/* . GKalbach July 2020
      updated 2020-08-20 by LPM

       sketch written for Teensy 3.5 Heart Rate Rev B
       initial sketch goals are to iterate through all
       multiplexed sensors and write data to SD card.
       Separate files are written for IR data (20Hz sampling rate)
       and temperature data (0.03Hz sampling rate)

       This version uses the MAX30105 shutdown feature, but this can be
       disabled by commenting out the line #define SLEEP near the top of
       this file


*/

#include "MAX30105.h"         // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
//#include "heartRate.h"        // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "SSD1306Ascii.h"     // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include "TimeLib.h"          // https://github.com/PaulStoffregen/Time
#include <Wire.h>
#include "SdFat.h"            // https://github.com/greiman/SdFat
#include "EEPROM.h"

#define NUM_SENSORS 8  // Change this if you have fewer than 8 sensors attached, to skip over higher unused channels

#define SLEEP // comment this line out to disable shutdown/wakeup of the MAX30105 sensors
//#define SERIALPLOTTER // comment this line out to shut off serial plotter output

MAX30105 particleSensor;
// sensor configurations
byte REDledBrightness = 0x01; // low value of 0x00 shuts it off, 0x01 is barely on
byte IRledBrightness = 0x1F; //Options: 0=0x00 to 0xFF=fully on
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 215; //Options: 69, 118, 215, 411, units microseconds. Applies to all active LEDs
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

#define TCAADDR 0x70

// time components
TimeElements tm;
time_t myTime;
uint8_t oldDay;
uint8_t oldMinute;
unsigned int myIntervalMS = 50;  // units milliseconds
unsigned long myMillis;
unsigned long lastWriteMillis;
unsigned long tempMillis;
unsigned int tempIntervalMS = 30000; //units milliseconds
int myCounter;
bool readTempsFlag = false; // Used to trigger a temperature readout
bool temp0Flag = false; // Used to mark a temperature readout at 0 seconds
bool temp30Flag = false; // Used to mark a temperature readout at 30 seconds

//SD components
SdFatSdio SD; // Uses Teensy's built-in SD card slot
File myFile; //SD card object 1 (IR data)
File myFile2; //SD card object 2 (Temp data)
//const int chipSelect = BUILTIN_SDCARD;  // not used with SdFat library

// Declare initial name for output files written to SD card
char filename[] = "YYYYMMDD_HHMM_00_SN00_IR.csv";
char filename2[] = "YYYYMMDD_HHMM_00_SN00_TEMP.csv";
// Placeholder serialNumber. Use the program serial_number_generator.ino to set the serial number
char serialNumber[] = "SN00"; // This number will be overwritten with the board's actual serial number
bool serialValid = false; // Flag to show whether the serialNumber value is real or just zeros
char ir[] = "IR"; //end of IR file name
char temp[] = "TEMP"; // end of temperature file name

// OLED components
#define MULTIPLE_I2C_PORTS 1 // for ssd1306Ascii library, multiple I2C ports available
#define SCREEN_TIMEOUT 10 //Seconds before OED display shuts off
#define SCREEN_WIDTH 128 // OLED pixel width
#define SCREEN_HEIGHT 64 // OLED pixel height
#define OLED_RESET     4 // OLED reset pin num
//SSD1306AsciiWire oled(Wire1); // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3c //OLED address
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
 


void setup() {
  Serial.begin(115200);
  //while (!Serial); // wait for serial port to connect. Sketch won't run unless serial monitor is open with this line
  Wire.setSCL(19);
  Wire.setSDA(18);
  Wire.begin(); // initialize I2C communication for MUX & sensors
  
   
    /*
    Wire1.begin(); // for OLED display
    Wire1.setSCL(37);
    Wire1.setSDA(38);
    Wire1.setClock(400000L);
    oled.begin(&Adafruit128x64, I2C_ADDRESS1);
    oled.setFont(Adafruit5x7);
    oled.clear();
    oled.print("Hello");
  */
  //RTC setup
  setSyncProvider(getTeensy3Time); // tell TimeLib to use Teensy's RTC for timekeeping
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  delay(100);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
#ifndef SERIALPLOTTER  
    Serial.println("RTC has set the system time");
#endif    
  }
#ifndef SERIALPLOTTER
  digitalClockDisplay(myTime); Serial.println();
#endif  
  oldDay = day(myTime); // Store current day

  // Grab the serial number from the EEPROM memory
  // This will be in the format "SNxx". The serial number
  // for a board can be permanently set with the separate
  // program 'serial_number_generator.ino' available in
  // one of the subfolders of the MusselHeart software
  EEPROM.get(0, serialNumber);
  if (serialNumber[0] == 'S') {
    serialValid = true; // set flag
  }

  myMillis = millis();
  lastWriteMillis = myMillis;
  myCounter = 0;

  //Sensor setup
  for (byte i = 0; i < NUM_SENSORS; i++) {
    tcaselect(i);
    delayMicroseconds(20);

    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) { // connect sensor to I2C bus
      particleSensor.setup(IRledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
      particleSensor.enableDIETEMPRDY(); //enable temp ready interrupt. Required to log temp, but each read takes 29ms
      //      particleSensor.disableDIETEMPRDY(); //disable temp ready interrupt.
      // Tweak individual settings
      particleSensor.setPulseAmplitudeRed(REDledBrightness); // essentially turn off red LED to save power, we only want IR LED. **** commented for testing only
      particleSensor.setPulseAmplitudeIR(IRledBrightness); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)
//      particleSensor.setPulseWidth(pulseWidth); //DANGER - this screws things up. Options: 69, 118, 215, 411. Higher values = more sensitivity
      triggerTemperatureSample(); // Start the temperature sample (wait 29 ms before attempting to read)
    }
  }

  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
#ifndef SERIALPLOTTER
  digitalClockDisplay(myTime); // digital clock display of the time

  //SD setup
  Serial.print("Initializing SD card...");
#endif  
  if (!SD.begin()) {
    SD.initErrorHalt("SdFatSdio begin() failed");
  }
#ifndef SERIALPLOTTER
  Serial.println("Initialization done.");
#endif  
  // SD Naming IR file
  initFileName(SD, myFile, myTime, filename, serialValid, serialNumber);
#ifndef SERIALPLOTTER
  Serial.print("Using IR file ");
  Serial.println(filename);
#endif  
  // SD Naming Temp file
  initTempFileName(SD, myFile2, myTime, filename2, serialValid, serialNumber, temp);
#ifndef SERIALPLOTTER
  Serial.print("Using Temp file ");
  Serial.println(filename2);
#endif  
  delay(30); // Just make sure the first temperature sample has time to happen
}  // end of setup loop


//-------------------------------------------
void loop() {

  // This if statement below will be true whenever the current millis() value is more than myIntervalMS
  // larger than myMillis
  if ( (millis() - myMillis) >= myIntervalMS) {
    // Always update myMillis whenever the if statement successfully executes so that the next
    // time around the if statement is comparing the time difference to this current time at the
    // start of the sampling cycle
    myMillis = millis();
    myCounter++; // increment here, since we want to mark how many times (20 times) we go through
    // the sampling cycle, rather than how many times each sensor gets sampled
    myTime = Teensy3Clock.get();

    // If a new day has started, create a new output file
    if ( oldDay != day(myTime) ) {
      oldDay = day(myTime); // update oldDay value to the new day
      // Close the current file
      myFile.close();
      // Start a new file
      initFileName(SD, myFile, myTime, filename, serialValid, serialNumber);
    }

    // Reopen logfile. If opening fails, notify the user
    if (!myFile.isOpen()) {
      if (!myFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
        Serial.println("Could not reopen file");
      }
    }
    if (myFile.isOpen()) {
      myFile.print(myMillis); myFile.print(","); // keep track of start of sampling round
      myFile.print(myTime); myFile.print(",");
      myFile.print(year(myTime)); myFile.print("-"); myFile.print(month(myTime)); myFile.print("-"); myFile.print(day(myTime));
      myFile.print(" "); myFile.print(hour(myTime)); myFile.print(":"); myFile.print(minute(myTime)); myFile.print(":"); myFile.print(second(myTime));
      myFile.print(",");

      // loop through sensors in sequence. If you have fewer than 8 sensors attached, change the value
      // of NUM_SENSORS at the top of this file, and make sure the sensors are plugged into the lower
      // number ports available (1,2,3 etc.) rather than skipping over ports and plugging into higher
      // port numbers (7, 8 etc.)
      for (byte i = 0; i < NUM_SENSORS; i++) {
        tcaselect(i);
#ifdef SLEEP        
        particleSensor.wakeUp(); // Wake up sensor to take sample
        delayMicroseconds(10); // Give the chip a chance to wake up
#endif        
        uint32_t tempIR = quickSampleIR();
        myFile.print(tempIR);
#ifdef SERIALPLOTTER        
        Serial.print(tempIR);
#endif        
//        myFile.print(quickSampleIR()); // custom function, see bottom of this file. This waits long enough to gather one sample
        
        if (readTempsFlag == false) {
#ifdef SLEEP          
          // If we don't need to read the temperatures on this round, shut the sensor back down
          particleSensor.shutDown(); // shut down sensor once sample is taken to save power
#endif
        }
        if (i < (NUM_SENSORS - 1)) {
          myFile.print(",");
#ifdef SERIALPLOTTER
          Serial.print("\t");
#endif
        }
        if (i >= (NUM_SENSORS - 1) ) { //start new line after the last sensor
          myFile.print(",");  // Modified to record millis value at end of cycle
          myFile.println(millis()); // Record the end time of the sampling cycle
          myFile.close();
#ifdef SERIALPLOTTER
          Serial.println();
#endif
        }
      }
    }
    else {
      Serial.println("error opening file.");
    }
    
    // Routine to read temperatures if the readTempsFlag has been set true elsewhere
    if (readTempsFlag == true){
      tempMillis = millis(); // update this for the file
        myTime = Teensy3Clock.get();
        // If a new day has started, create a new output file
        if ( oldDay != day(myTime) ) {
          oldDay = day(myTime); // update oldDay value to the new day
          // Close the current file
          myFile2.close();
          // Start a new file
          initTempFileName(SD, myFile2, myTime, filename, serialValid, serialNumber, temp);
        }
    
        // Reopen logfile. If opening fails, notify the user
        if (!myFile2.isOpen()) {
          if (!myFile2.open(filename2, O_RDWR | O_CREAT | O_AT_END)) {
            Serial.println("Could not reopen file");
          }
        }
        if (myFile2.isOpen()) {
          myFile2.print(tempMillis); myFile2.print(",");
          myFile2.print(myTime); myFile2.print(",");
          myFile2.print(year(myTime)); myFile2.print("-"); myFile2.print(month(myTime)); myFile2.print("-"); myFile2.print(day(myTime));
          myFile2.print(" "); myFile2.print(hour(myTime)); myFile2.print(":"); myFile2.print(minute(myTime)); myFile2.print(":"); myFile2.print(second(myTime));
          myFile2.print(",");
    
          // loop through sensors in sequence. If you have fewer than 8 sensors attached, change the value
          // of NUM_SENSORS at the top of this file, and make sure the sensors are plugged into the lower
          // number ports available (1,2,3 etc.) rather than skipping over ports and plugging into higher
          // port numbers (7, 8 etc.)
          for (byte i = 0; i < NUM_SENSORS; i++) {
            tcaselect(i);
            myFile2.print(readTemperatureSample()); myFile2.print(",");
#ifdef SLEEP            
            particleSensor.shutDown(); // shut down sensor to save power
#endif
          }
//          Serial.println("Temperature reading taken");
          myFile2.println(millis());
          myFile2.close();
          readTempsFlag = false; // reset this flag so this section doesn't execute again until triggered
        }
        else {
          Serial.println("error opening file.");
        }
      }

    
  } // End of IR sampling routine (and temperature readout routine)

  
  //----------------------------------------
  // Temperature sampling check
  // Check if it has been long enough for a new temperature reading to be needed (i.e. 30 seconds).
  // If it is, wake up the sensors and initiate a temperature reading, but we won't actually read
  // the temperatures until the next time through the IR readout section above. This should give
  // sufficient time for the sensors to read their temperatures (about 29 milliseconds) when our
  // IR sampling interval is larger than that (usually 50 milliseconds)
  if ( (second(myTime) == 0) & (temp0Flag == false) ){
    // Trigger when seconds value == 0
    readTempsFlag = true;
    temp0Flag = true; // set true to stop repeated measures in the same second
    temp30Flag = false; // set false to allow next sample at 30 seconds
    for (byte i = 0; i < NUM_SENSORS; i++) {
      tcaselect(i);
#ifdef SLEEP      
      particleSensor.wakeUp(); // wake up sensor
      delayMicroseconds(10);
#endif
      triggerTemperatureSample(); // start temp sample so that it's ready by the next cycle through the main loop  
      // We won't put the sensor back to sleep on this cycle so that it can complete its temperature reading
    }
  }
  if ( (second(myTime) == 30) & (temp30Flag == false) ){
    // Trigger when seconds value == 30
    readTempsFlag = true;
    temp30Flag = true; // set true to stop repeated measures in the same second
    temp0Flag = false; // set false to allow next sample at 0 seconds
    for (byte i = 0; i < NUM_SENSORS; i++) {
      tcaselect(i);
#ifdef SLEEP      
      particleSensor.wakeUp(); // wake up sensor
      delayMicroseconds(10);
#endif      
      triggerTemperatureSample(); // start temp sample so that it's ready by the next cycle through the main loop  
      // We won't put the sensor back to sleep on this cycle so that it can complete its temperature reading
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

//*********************************************
// Function to create a fileName based on the current time
void initFileName(SdFatSdio& SD, File& myFile, time_t time1, char *filename, bool serialValid, char *serialNumber) {

  char buf[5];
  // integer to ascii function itoa(), supplied with numeric year value,
  // a buffer to hold output, and the base for the conversion (base 10 here)
  itoa(year(time1), buf, 10);
  // copy the ascii year into the filename array
  for (byte i = 0; i < 4; i++) {
    filename[i] = buf[i];
  }
  // Insert the month value
  if (month(time1) < 10) {
    filename[4] = '0';
    filename[5] = month(time1) + '0';
  } else if (month(time1) >= 10) {
    filename[4] = (month(time1) / 10) + '0';
    filename[5] = (month(time1) % 10) + '0';
  }
  // Insert the day value
  if (day(time1) < 10) {
    filename[6] = '0';
    filename[7] = day(time1) + '0';
  } else if (day(time1) >= 10) {
    filename[6] = (day(time1) / 10) + '0';
    filename[7] = (day(time1) % 10) + '0';
  }
  // Insert an underscore between date and time
  filename[8] = '_';
  // Insert the hour
  if (hour(time1) < 10) {
    filename[9] = '0';
    filename[10] = hour(time1) + '0';
  } else if (hour(time1) >= 10) {
    filename[9] = (hour(time1) / 10) + '0';
    filename[10] = (hour(time1) % 10) + '0';
  }
  // Insert minutes
  if (minute(time1) < 10) {
    filename[11] = '0';
    filename[12] = minute(time1) + '0';
  } else if (minute(time1) >= 10) {
    filename[11] = (minute(time1) / 10) + '0';
    filename[12] = (minute(time1) % 10) + '0';
  }
  // Insert another underscore after time
  filename[13] = '_';
  // If there is a valid serialnumber, insert it into
  // the file name in positions 17-20.
  if (serialValid) {
    byte serCount = 0;
    for (byte i = 17; i < 21; i++) {
      filename[i] = serialNumber[serCount];
      serCount++;
    }
  }
  // insert _IR in positions 21-23 of the file name
  filename[21] = '_';
  byte irCount = 0;
  for (byte i = 22; i < 24; i++) {
    filename[i] = ir[irCount];
    irCount++;
  }
  // Next change the counter on the end of the filename
  // (digits 14+15) to increment count for files generated on
  // the same day. This shouldn't come into play
  // during a normal data run, but can be useful when
  // troubleshooting.
  for (uint8_t i = 0; i < 100; i++) {
    filename[14] = i / 10 + '0';
    filename[15] = i % 10 + '0';

    if (!SD.exists(filename)) {
      // when SD.exists() returns false, this block
      // of code will be executed to open a file with this new filename
      myFile = SD.open(filename, O_RDWR | O_CREAT | O_AT_END);
      //      if (!myFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      //        Serial.println("Didn't open myFile initially");
      //        delay(5);
      //      }
      break; // Break out of the for loop when the
      // statement if(!myFile.exists())
      // is finally false (i.e. you found a new file name to use).
    } // end of if(!sd.exists())
  } // end of file-naming for loop
  //------------------------------------------------------------
  // Write 1st header line
  myFile.print("startMillis,");
  myFile.print("UnixTime"); myFile.print(",");
  myFile.print("DateTime"); myFile.print(",");
  for (int i = 1; i <= NUM_SENSORS; i++) { // Write column names for each sensor
    myFile.print("Sensor"); myFile.print(i); myFile.print("IR,");

  }
  myFile.println("endMillis");

  // Update the file's creation date, modify date, and access date.
  myFile.timestamp(T_CREATE, year(time1), month(time1), day(time1),
                   hour(time1), minute(time1), second(time1));
  myFile.timestamp(T_WRITE, year(time1), month(time1), day(time1),
                   hour(time1), minute(time1), second(time1));
  myFile.timestamp(T_ACCESS, year(time1), month(time1), day(time1),
                   hour(time1), minute(time1), second(time1));
  myFile.close(); // force the data to be written to the file by closing it
} // end of initFileName function
//*********************************************
// Function to create a fileName for Temp file based on the current time
void initTempFileName(SdFatSdio& SD, File& myFile2, time_t time1, char *filename2, bool serialValid, char *serialNumber, char *temp) {

  char buf[5];
  // integer to ascii function itoa(), supplied with numeric year value,
  // a buffer to hold output, and the base for the conversion (base 10 here)
  itoa(year(time1), buf, 10);
  // copy the ascii year into the filename array
  for (byte i = 0; i < 4; i++) {
    filename2[i] = buf[i];
  }
  // Insert the month value
  if (month(time1) < 10) {
    filename2[4] = '0';
    filename2[5] = month(time1) + '0';
  } else if (month(time1) >= 10) {
    filename2[4] = (month(time1) / 10) + '0';
    filename2[5] = (month(time1) % 10) + '0';
  }
  // Insert the day value
  if (day(time1) < 10) {
    filename2[6] = '0';
    filename2[7] = day(time1) + '0';
  } else if (day(time1) >= 10) {
    filename2[6] = (day(time1) / 10) + '0';
    filename2[7] = (day(time1) % 10) + '0';
  }
  // Insert an underscore between date and time
  filename2[8] = '_';
  // Insert the hour
  if (hour(time1) < 10) {
    filename2[9] = '0';
    filename2[10] = hour(time1) + '0';
  } else if (hour(time1) >= 10) {
    filename2[9] = (hour(time1) / 10) + '0';
    filename2[10] = (hour(time1) % 10) + '0';
  }
  // Insert minutes
  if (minute(time1) < 10) {
    filename2[11] = '0';
    filename2[12] = minute(time1) + '0';
  } else if (minute(time1) >= 10) {
    filename2[11] = (minute(time1) / 10) + '0';
    filename2[12] = (minute(time1) % 10) + '0';
  }
  // Insert another underscore after time
  filename2[13] = '_';
  // If there is a valid serialnumber, insert it into
  // the file name in positions 17-20.
  if (serialValid) {
    byte serCount = 0;
    for (byte i = 17; i < 21; i++) {
      filename2[i] = serialNumber[serCount];
      serCount++;
    }
  }
  // insert _TEMP in positions 21-25 of the file name
  filename2[21] = '_';
  byte tempCount = 0;
  for (byte i = 22; i < 26; i++) {
    filename2[i] = temp[tempCount];
    tempCount++;
  }

  // Next change the counter on the end of the filename
  // (digits 14+15) to increment count for files generated on
  // the same day. This shouldn't come into play
  // during a normal data run, but can be useful when
  // troubleshooting.
  for (uint8_t i = 0; i < 100; i++) {
    filename2[14] = i / 10 + '0';
    filename2[15] = i % 10 + '0';

    if (!SD.exists(filename2)) {
      // when SD.exists() returns false, this block
      // of code will be executed to open a file with this new filename
      myFile2 = SD.open(filename2, O_RDWR | O_CREAT | O_AT_END);
      //      if (!myFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      //        Serial.println("Didn't open myFile initially");
      //        delay(5);
      //      }
      break; // Break out of the for loop when the
      // statement if(!myFile.exists())
      // is finally false (i.e. you found a new file name to use).
    } // end of if(!sd.exists())
  } // end of file-naming for loop
  //------------------------------------------------------------
  // Write 1st header line for temp file
  myFile2.print("startMillis"); myFile2.print(",");
  myFile2.print("UnixTime"); myFile2.print(",");
  myFile2.print("DateTime"); myFile2.print(",");
  for (int i = 1; i <= NUM_SENSORS; i++) { // Write column names for each sensor
    myFile2.print("Sensor"); myFile2.print(i); myFile2.print("TempC,");
    }
  myFile2.println("endMillis");  

  // Update the file's creation date, modify date, and access date.
  myFile2.timestamp(T_CREATE, year(time1), month(time1), day(time1),
                    hour(time1), minute(time1), second(time1));
  myFile2.timestamp(T_WRITE, year(time1), month(time1), day(time1),
                    hour(time1), minute(time1), second(time1));
  myFile2.timestamp(T_ACCESS, year(time1), month(time1), day(time1),
                    hour(time1), minute(time1), second(time1));
  myFile2.close(); // force the data to be written to the file by closing it
} // end of initTempFileName function

//-----------------------------------------------



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

//-----------------------------------------------
// Custom sampling function for the MAX30105, calling functions in the MAX30105.h library
uint32_t quickSampleIR(void) {
  // Clear the MAX30105 FIFO buffer so that there will only be one new sample to read
  particleSensor.clearFIFO();
  uint16_t ledSampleTime; // units will be microseconds
  
  
  // Implement a delay for new samples to be collected in the FIFO. If sample averaging
  // is used, a loop will need to execute multiple times to allow the multiple samples
  // to be collected
  
  // Start by waiting the maximum time needed for the 1st sample to be taken, based 
  // on the sample rate
  switch (sampleRate) {
    // Options 50, 100, 200, 400, 800, 1000, 1600, 3200 samples per second
    case 50:
      delay(20); // units milliseconds
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
    default:
      delay(5);
      break;
  }
   
  for (int avg = 0; avg < sampleAverage; avg++){
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
    }
    // If more than one sample is being averaged, you need to
    // further wait for the start of the next sampling cycle,
    // which is determined by the sampleRate and how long it just
    // took for the red and IR leds to be sampled. This won't 
    // execute on the last sample of the average because it isn't
    // needed after the IR LED is read for the last time
    if (avg < (sampleAverage - 1)) {
      switch (sampleRate){
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
  delayMicroseconds(50); // Add in a little extra delay just to be safe
  // Query the FIFO buffer on the sensor for the most recent IR value
  return (particleSensor.getIR());
} // end of quickSampleIR function
