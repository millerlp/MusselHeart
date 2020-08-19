
/* . GKalbach July 2020
      updated 2020-08-15 by GMK

       sketch written for Teensy 3.5 Heart Rate Rev B
       initial sketch goals are to iterate through all
       multiplexed sensors and write data to SD card.
       Separate files are written for IR data (20Hz sampling rate)
       and temperature data (0.03Hz sampling rate)


*/

#include "MAX30105.h"         // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "heartRate.h"        // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include "SSD1306Ascii.h"     // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <TimeLib.h>          // https://github.com/PaulStoffregen/Time
#include <Wire.h>
#include "SdFat.h"            // https://github.com/greiman/SdFat
#include "EEPROM.h"

#define NUM_SENSORS 2  // Change this if you have fewer than 8 sensors attached, to skip over higher unused channels

MAX30105 particleSensor;
// sensor configurations
byte ledBrightness = 0x1F; //Options: 0=Off to 255=fully on
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411, units microseconds. Applies to all active LEDs
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

#define TCAADDR 0x70

// time components
TimeElements tm;
time_t myTime;
uint8_t oldDay;
//unsigned long millisVal;  // not being used currently that I can see

unsigned int myIntervalMS = 50;  // units milliseconds
unsigned long myMillis;
unsigned long lastWriteMillis;
unsigned long tempMillis;
unsigned int tempIntervalMS = 30000; //units milliseconds
int myCounter;

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
  Wire.begin(); // initialize I2C communication for MUX & sensors
  /*
    Wire.setSCL(19);
    Wire.setSDA(18);
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
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");
  digitalClockDisplay(myTime); Serial.println();
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
      particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
      particleSensor.enableDIETEMPRDY(); //enable temp ready interrupt. Required to log temp, but each read takes 29ms
      //      particleSensor.disableDIETEMPRDY(); //disable temp ready interrupt.
      // Tweak individual settings
      //particleSensor.setPulseAmplitudeRed(0x00); // essentially turn off red LED to save power, we only want IR LED. **** commented for testing only
      particleSensor.setPulseAmplitudeIR(ledBrightness); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)
      particleSensor.setPulseWidth(pulseWidth); //Options: 69, 118, 215, 411. Higher values = more sensitivity
      triggerTemperatureSample(); // Start the temperature sample (wait 29 ms before attempting to read)
    }
  }

  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  digitalClockDisplay(myTime); // digital clock display of the time

  //SD setup
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    SD.initErrorHalt("SdFatSdio begin() failed");
  }
  Serial.println("Initialization done.");
  // SD Naming IR file
  initFileName(SD, myFile, myTime, filename, serialValid, serialNumber);
  Serial.print("Using IR file ");
  Serial.println(filename);
  // SD Naming Temp file
  initTempFileName(SD, myFile2, myTime, filename2, serialValid, serialNumber, temp);
  Serial.print("Using Temp file ");
  Serial.println(filename2);
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
        particleSensor.wakeUp(); // wake up sensor to take sample
        //        myFile.print(particleSensor.getIR()); // old method
        delay(10);
        myFile.print(quickSampleIR()); // custom function, see bottom of this file
        myFile.print(",");
        particleSensor.shutDown(); // shut down sensor once sample is taken to save power
        //        myFile.print(particleSensor.readTemperature()); // old method
        if (i < (NUM_SENSORS - 1)) {
          myFile.print(",");
        }
        if (i >= (NUM_SENSORS - 1) ) { //start new line after the last sensor
          myFile.print(",");  // Modified to record millis value at end of cycle
          myFile.println(millis()); // Record the end time of the sampling cycle
          myFile.close();
        }
      }
    }
    else {
      Serial.println("error opening file.");
    }
  }

  // temp file
  if ( (millis() - tempMillis) >= tempIntervalMS) {
    // Always update tempMillis whenever the if statement successfully executes so that the next
    // time around the if statement is comparing the time difference to this current time at the
    // start of the sampling cycle
    tempMillis = millis();
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
        particleSensor.wakeUp(); // wake up sensor
        triggerTemperatureSample(); // start temp sample, wait 30ms to read
        delay(40);
        myFile2.print(readTemperatureSample()); myFile2.print(",");
        Serial.println("Temperature reading taken");
        particleSensor.shutDown(); // shut down sensor to save power
        //        myFile.print(particleSensor.readTemperature()); // old method
      }
      myFile2.println(millis());
      myFile2.close();
    }
    else {
      Serial.println("error opening file.");
    }
  }

  // I'm not sure this if statement below is necessary in practice. The SD library should take care of
  // flushing data to the card on its own whenever it accumulates enough data. You'd only want this
  // to force a flush if you really want to be sure a flush happens on a regular (frequent) interval,
  // which isn't really necessary.
  /*
  if (myCounter > 19) {
    myFile.flush(); // force a write to the SD card every second
    //    Serial.print("Millis since last card write: ");
    Serial.println( (millis() - lastWriteMillis)); // Time difference since last card flush, ideally = 1000
    //    Serial.println("\t Data written to SD card. myCounter reset.");
    lastWriteMillis = millis(); // Update lastWriteMillis
    myCounter = 0;
  }
*/
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
// Custom sampling function for the MAX30105, calling functions in the MAX30105.h library
uint32_t quickSampleIR(void) {
  // Clear the MAX30105 FIFO buffer so that there will only be one new sample to read
  particleSensor.clearFIFO();
  // Multiply pulseWidth by 2 because we always have to wait for the Red LED to sample first
  // before the IR LED gets sampled. Then account for time taken for any sample averages, and
  // add on a buffer of 50 more microseconds just for safety's sake
  delayMicroseconds( (pulseWidth * 2 * sampleAverage) + 50) ;
  return (particleSensor.getIR());
} // end of quickSampleIR function


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
