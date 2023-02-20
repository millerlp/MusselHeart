/* HeartBeat_interval_sample.ino

    A version of the main datalogging program to work with
    Teensy3.5 and heart rate daughterboard RevB.

    Updated 2023-02-18 to fix the missed-sample issue and lengthen
    sampling period to 60 seconds from 30 seconds.   
    This version still has an issue with values from one channel
    bleeding over into the neighboring channel occasionally. This
    may be due to the multiplexer not changing channels successfully
    sometimes. 

    This version will sample the heart rate sensors at 10Hz
    for 60 seconds, then take a round of temperature readings,
    and put everything to sleep for the remainder of the sample interval
    using the Snooze library, in an attempt to lower the overall
    power usage.
  
    NOTE: If this program is running on the Teensy, the serial
    connection will drop out every time a new set of samples starts,
    and you won't be able to send a new program to the device easily. 
    To upload the new program, hit the upload button here, and then on
    the Teensy you need to hit the reset button the Teensy itself
    (not the reset button on the daughterboard).

*/

#include "MAX30105.h"         // https://github.com/millerlp/SparkFun_MAX3010x_Sensor_Library
#include "Snooze.h"   // https://github.com/duff2013/Snooze
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include "SSD1306Ascii.h"     // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <Wire.h>
#include "SdFat.h"            // https://github.com/greiman/SdFat (compiles with SdFat 2.1.2)
#include "EEPROM.h"

#define MAX_SENSORS 8  // Leave this set at 8, even if fewer than 8 sensors are attached
#define FAST_SAMPLE_INTERVAL_MS 100 // units millisecond - this sets sampling rate when active
#define INTERVAL_MINUTES 1 // Interval between sampling bouts, in minutes (i.e. 2, 5, etc)
#define SAMPLING_LENGTH_SEC 55 // units seconds - how many seconds worth of samples will be collected in a minute
bool readTempsFlag = false;


//------------------------------------------
// Debugging stuff, used with logic analyzer
bool scopePinState = LOW; // for debugging
int scopePin0 = 30; // for debugging

//--------------------------------------
// MAX30105 sensor parameters
MAX30105 max3010x;
// Create an array to hold numbers of good sensor channels (up to 8)
byte goodSensors[] = {127, 127, 127, 127, 127, 127, 127, 127};
byte numgoodSensors = 0;
// sensor configurations
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green. Only use 2
byte REDledBrightness = 0; // low value of 0 shuts it off, 1 is barely on
//byte IRledBrightness = 20; //Options: 0=off to 255=fully on, try 10-30 initially. Too high will make noisy signal
// Define IR led brightness setting for each of the 8 channels
// Options: 0=off to 255=fully on, try 10-30 initially. Too high will make noisy signal
//          Channel =      1   2   3   4   5   6   7   8
byte IRledBrightness[] = {60, 60, 60, 60, 60, 60, 60, 60};

byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32, but only use 1. The others are too slow
int pulseWidth = 215; //Options: 69, 118, 215, 411, units microseconds. Applies to all active LEDs. Recommend 215
// For 118us, max sampleRate = 1000; for 215us, max sampleRate = 800, for 411us, max sampleRate = 400
int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384. 4096 is standard


#define TCAADDR 0x70 // I2C address for the I2C multiplexer chip
//-------------------------------
TimeElements tm; // Create a TimeElements object
time_t myTime;
uint8_t oldDay;
uint8_t oldMinute;

//-----------------------------
// Snooze library setup
SnoozeTimer timer;  // LowPowerTimer millisecond timer object
SnoozeAlarm  alarm; // RTC alarm object
SnoozeBlock config_teensy35(timer, alarm); // wake from ms timer or RTC
SnoozeBlock config_teensy35_2(alarm);  // wake from RTC alarm only
int who; // used to track source of Snooze wakeup
//---------------------------------------
// OLED components
#define MULTIPLE_I2C_PORTS 1 // for ssd1306Ascii library, multiple I2C ports available
#define SCREEN_TIMEOUT 10 //Seconds before OLED display shuts off
#define SCREEN_WIDTH 128 // OLED pixel width
#define SCREEN_HEIGHT 64 // OLED pixel height
SSD1306AsciiWire oled(Wire1); // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C //OLED address

//----------------------------------------------
//SD components
const uint8_t SD_CS_PIN = SDCARD_SS_PIN; // Set up to use Teensy 3.5 onboard SD card slot
#define SD_FAT_TYPE 3; // For use with SdFat-beta 2.1.4-beta3 or SdFat 2.1.2
SdFs sd; 
FsFile IRFile; //SD card object 1 (IR data) 
FsFile TEMPFile; //SD card object 2 (Temp data) 
//SdFatSdio sd; // Uses Teensy's built-in SD card slot
//File IRFile; //SD card object 1 (IR data)
//File TEMPFile; //SD card object 2 (Temp data)
// Declare initial name for output files written to SD card
char filename[] = "YYYYMMDD_HHMM_00_SN00_IR.csv";
char filename2[] = "YYYYMMDD_HHMM_00_SN00_TEMP.csv";
// Placeholder serialNumber. Use the program serial_number_generator.ino to set the serial number
char serialNumber[] = "SN00"; // This number will be overwritten with the board's actual serial number
bool serialValid = false; // Flag to show whether the serialNumber value is real or just zeros
char ir[] = "IR"; //end of IR file name
char temp[] = "TEMP"; // end of temperature file name
//-------------------------------------------------
// Setup RGB LED on daughterboard
int REDLED = 7; //  Teensy Pin D7
int GRNLED = 6; //  Teensy Pin D6
int BLUELED = 5; //  Teensy Pin D5
#define COMMON_ANODE  // For RGB LED

//---------------------------------------------
// Battery monitor stuff
const int BATT_MONITOR_EN = 17; // Digital pin to enable battery voltage monitor circuit
const int BATT_MONITOR = A10;  //  Analog pin to read battery voltage
double dividerRatio = 5.7; // Ratio of voltage divider (47k + 10k) / 10k = 5.7
int resolutionADC = 1024;
// The refVoltage is coming from a MAX6103 precision voltage reference chip, so it should be 3.000
double refVoltage = 3.00; // Voltage at AREF pin on Teensy
double batteryVolts = 0; // Estimated battery voltage returned from readBatteryVoltage function
//----------------------------------------------

//-----------------------------------

void setup() {
  Serial.begin(115200);
  //**************************
  // RGB LED setup
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);
  digitalWrite(REDLED, HIGH); // for common anode LED, set high to shut off
  digitalWrite(GRNLED, HIGH);
  digitalWrite(BLUELED, HIGH);
  // Flash green quickly to denote restart
  for (int i = 0; i < 10; i++) {
    setColor(0, 255, 0);
    delay(20);
    setColor(0, 0, 0);
    delay(20);
  }
  //***************************
  // Debugging pins
  pinMode(scopePin0, OUTPUT);
  digitalWriteFast(scopePin0, scopePinState);
  //**************************
  // Battery monitor pins
  analogReference(EXTERNAL);
  analogReadResolution(10); // set 10 bit resolution
  pinMode(BATT_MONITOR, INPUT);
  pinMode(BATT_MONITOR_EN, OUTPUT);
  //***********************
  // Define pins for the I2C multiplexer attached to IR sensors
  Wire.setSCL(19);
  Wire.setSDA(18);
  Wire.begin(); // initialize I2C communication for MUX & sensors
  //**********************************
  // OLED I2C setup
  Wire1.begin(); // for OLED display
  Wire1.setSCL(37);
  Wire1.setSDA(38);
  Wire1.setClock(400000L);  // 400 kHz fast I2C
  oled.begin(&Adafruit128x64, I2C_ADDRESS1);
  Wire1.beginTransmission(I2C_ADDRESS1);
  Wire1.write(0x80); // oled set to Command mode (0x80) instead of data mode (0x40)
  Wire1.write(0xAF); // oled command 0xAF should power back up
  Wire1.endTransmission();
  delay(50);
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.println("Hello");

  //*************************************
  // RTC setup
  setSyncProvider(getTeensy3Time); // tell TimeLib to use Teensy's RTC for timekeeping
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  delay(100);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
    oled.println("RTC not set!");
  } else {
    Serial.println("RTC has set the system time");
  }
  digitalClockDisplay(myTime); Serial.println();
  oldDay = day(myTime); // Store current day
  // Show date and time on OLED display
  printTimeOLED(myTime);
  oled.println();
  //**********************************
  // Grab the serial number from the EEPROM memory
  // This will be in the format "SNxx". The serial number
  // for a board can be permanently set with the separate
  // program 'serial_number_generator.ino' available in
  // one of the subfolders of the MusselHeart software
  EEPROM.get(0, serialNumber);
  if (serialNumber[0] == 'S') {
    serialValid = true; // set flag
  }
  //*******************************
  // MAX30105 Sensor setup. This function will scan for avaialble
  // sensors, and set sampling parameters for those that are found
  // This will also store which channels are working, so that only
  // those channels are sampled and stored in the data files
  scanSetupSensors(); // See function near bottom of this file
  if (numgoodSensors == 0) {
    oled.println("No sensors");
    oled.println("found");
  } else {
    oled.print("Found ");
    oled.print(numgoodSensors);
    oled.println(" sensors:");
    for (byte i = 0; i < MAX_SENSORS; i++) {
      if (goodSensors[i] != 127) {
        oled.print(goodSensors[i] + 1); // Start labeling at 1 instead of 0
        oled.print(" ");
      }
    }
    oled.println();
  }
  // Shut the sensors back down for now
  for (byte i = 0; i < MAX_SENSORS; i++) {
    if (goodSensors[i] != 127) {
      tcaselect(i);
      max3010x.shutDown(); // shut down sensor to save power
    }
  }

  //********************************
  // SD card setup
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  digitalClockDisplay(myTime); // digital clock display of the time to Serial
  //SD setup
  Serial.print("Initializing SD card...");
  if (!sd.begin(SdioConfig(FIFO_SDIO))) {
    oled.println("No SD card found");
    sd.initErrorHalt("SdFatSdio begin() failed");
    setColor(255, 0, 0);
  }
  Serial.println("Initialization done.");
  // SD Naming IR file
  initFileName(sd, IRFile, myTime, filename, serialValid, serialNumber);
  Serial.print("Using IR file ");
  Serial.println(filename);
  // SD Naming Temperature file
  initTempFileName(sd, TEMPFile, myTime, filename2, serialValid, serialNumber, temp);
  Serial.print("Using Temp file ");
  Serial.println(filename2);
  //*****************************
  // Check raw battery voltage, see function at bottom of this file
  batteryVolts = readBatteryVoltage(BATT_MONITOR_EN, BATT_MONITOR, dividerRatio, refVoltage, resolutionADC);
  oled.print("Battery: ");
  oled.print(batteryVolts, 3);
  oled.println("V");
  //**********************************
  // Report settings
//  Serial.print("Sampling interval: "); Serial.print(SAMPLE_INTERVAL_SEC); Serial.println(" secs");
  Serial.print("Sampling interval: "); Serial.print(INTERVAL_MINUTES); Serial.println(" minutes");
  Serial.print("Sampling duration: "); Serial.print(SAMPLING_LENGTH_SEC); Serial.println(" secs");
//  oled.print("Interval: "); oled.print(SAMPLE_INTERVAL_SEC); oled.println(" secs");
  oled.print("Interval: "); oled.print(INTERVAL_MINUTES); oled.println(" mins");
  oled.print("Duration: "); oled.print(SAMPLING_LENGTH_SEC); oled.println(" secs");

  
  delay(4000);
  //*********************************
  // Idle while waiting for a new minute to start
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
//  while (second(myTime) != 0) {
//    alarm.setAlarm(myTime + 1); // set alarm 1 second in future
//    Snooze.deepSleep( config_teensy35); // sleep until second rolls over
//    oled.clear();
//    printTimeOLED(Teensy3Clock.get());
//    oled.println();
//    oled.print("Waiting...");
//    myTime = Teensy3Clock.get(); // update myTime
//  }

  while (1) {
    alarm.setAlarm(myTime + 1); // set alarm 1 second in future
    Snooze.deepSleep( config_teensy35); // sleep until second rolls over
    oled.clear();
    printTimeOLED(Teensy3Clock.get());
    oled.println();
    oled.print("Waiting...");
    myTime = Teensy3Clock.get(); // update myTime

    if ( ((minute(myTime) % INTERVAL_MINUTES) == 0) & (second(myTime) == 0) ){
      break; // break out of while loop
    }
  }

  
  // If we exited the while loop above, a new minute has just
  // started
  oled.clear();
  // Manual shut down of SSD1306 oled display driver
//  Wire1.beginTransmission(0x3C); // oled1 display address
//  Wire1.write(0x80); // oled set to Command mode (0x80) instead of data mode (0x40)
//  Wire1.write(0xAE); // oled command to power down (0xAF should power back up)
//  Wire1.endTransmission(); // stop transmitting

} // end of setup()

void loop() {
  /*  You should arrive at the start of this main loop if a new minute has just started
   *   This will usually be because the RTC alarm has been set to wake the Teensy at the
   *   start of a new minute
   *   
   *  TODOs in this main loop:
      1. Check current time from RTC. If new day, generate new files
      2. If seconds 0-SAMPLING_LENGTH_SEC of the minute, run the IntervalTimer to
      generate 10Hz interrupts that will trigger a round of heart
      rate sampling
      3. If seconds SAMPLING_LENGTH_SEC is reached, take a set of temperature readings
      to write to the 2nd file, along with battery voltage
      4. Shut down heart sensors and Teensy3.5, using rtcAlarm() function
      from Snooze library to trigger a wake up at the start of the next
      minute
  */
  myTime = Teensy3Clock.get();
//  digitalWriteFast(scopePin0, HIGH); // debugging, can comment out
//  scopePinState = HIGH; // debugging, can comment out

  time_t currTime = myTime; // Copy for later
  // If a new day has started, create a new output file
  if ( oldDay != day(myTime) ) {
    oldDay = day(myTime); // update oldDay value to the new day
    // Re-scan the available sensors each new day to make sure none have dropped off
    // and are screwing up the data collection.
    scanSetupSensors(); // See function near bottom of this file

    // Close the current IR file
    IRFile.close();
    // Start a new IR file
    initFileName(sd, IRFile, myTime, filename, serialValid, serialNumber);
    // Close the current temperature file
    TEMPFile.close();
    // Start a new temperature file
    initTempFileName(sd, TEMPFile, myTime, filename2, serialValid, serialNumber, temp);
  }

  // Create the sample buffer  [row][column]
  uint32_t sampleBuffer [SAMPLING_LENGTH_SEC * (FAST_SAMPLE_INTERVAL_MS / 10)] [MAX_SENSORS] = { {}, {} };
  // Create buffer to hold the millis value at the start of each sampling cycle (10Hz cycles)
  uint32_t millisStartBuffer [SAMPLING_LENGTH_SEC * (FAST_SAMPLE_INTERVAL_MS / 10)] = {}; 
  // Create buffer to hold the millis value at the end of each sampling cycle (10Hz cycles)
  uint32_t millisEndBuffer [SAMPLING_LENGTH_SEC * (FAST_SAMPLE_INTERVAL_MS / 10)] = {};
  // Store the POSIX time of each cycle of samples (will repeat the same time value until new second elapses)
  time_t startTimeStamp [SAMPLING_LENGTH_SEC * (FAST_SAMPLE_INTERVAL_MS / 10)] = {}; 
  
  uint16_t loopCount = 0;  // Used for counting the SAMPLING_LENGTH_SEC * 10 Hz number of sampling loops

  
  for (byte channel = 0; channel < MAX_SENSORS; channel++) {
    if (goodSensors[channel] != 127) {
      tcaselect(channel);
      delayMicroseconds(5);
      max3010x.wakeUp();
      delayMicroseconds(5);
    }
  }
  
    
  while ( loopCount < (SAMPLING_LENGTH_SEC * (FAST_SAMPLE_INTERVAL_MS / 10)) ) {
    elapsedMillis sampleTimer = 0;
    startTimeStamp[loopCount] = myTime;
    // Start each time through this loop by reawakening IR sensors
      
    millisStartBuffer[loopCount] = millis(); // Store the current millis value at start of a sample cycle
      
    for (byte channel = 0; channel < MAX_SENSORS; channel++) {
      if (goodSensors[channel] != 127) {
          tcaselect(channel);
          while(max3010x.check() < 0) {}; // wait for a new sample to appear    
          // Calling getIR() should get the most recent value from the buffer of values
          sampleBuffer[loopCount][channel] = max3010x.getIR();  // modify getIR in the library to remove safeCheck() function    
//          printSensorOLED(channel, sampleBuffer[loopCount][channel]); // testing only
      }
    } // End of looping through the 8 channels


    millisEndBuffer[loopCount] = millis(); // Store the millis value at the end of a single sample cycle
    loopCount++;  // Increment the loop counter

    while ( sampleTimer < FAST_SAMPLE_INTERVAL_MS ) {
      // chill out waiting for next sample interval
      // It would be nice to sleep the Teensy here, but
      // the milliseconds timer loses a bit of time each
      // time it goes in/out of sleep modes, and the 
      // sampleTimer value stops updating
    }
       
    readTempsFlag = true;
    myTime = Teensy3Clock.get(); // update myTime

  }  // end of while loop up to SAMPLING_LENGTH_SEC seconds
  // while loop quits after SAMPLING_LENGTH_SEC seconds of IR samples have been taken
  
  //***************************************************************************
  // Write the sample buffers to the SD card
  // Reopen IR logfile. If opening fails, notify the user
  if (!IRFile.isOpen()) {
    if (!IRFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      Serial.println("Could not reopen file");
    }
  }

  for (uint16_t writeLoop = 0; writeLoop < (SAMPLING_LENGTH_SEC * (FAST_SAMPLE_INTERVAL_MS / 10)); writeLoop++){
    IRFile.print(millisStartBuffer[writeLoop]); IRFile.print(","); // keep track of millis() value at start of sampling round
    IRFile.print(startTimeStamp[writeLoop]); IRFile.print(",");  // print POSIX time value (one long number, seconds since 1970-1-1 00:00)
    IRFile.print(year(startTimeStamp[writeLoop])); IRFile.print("-"); IRFile.print(month(startTimeStamp[writeLoop])); IRFile.print("-"); IRFile.print(day(startTimeStamp[writeLoop]));
    IRFile.print(" "); IRFile.print(hour(startTimeStamp[writeLoop])); IRFile.print(":"); IRFile.print(minute(startTimeStamp[writeLoop])); IRFile.print(":"); IRFile.print(second(startTimeStamp[writeLoop]));
    IRFile.print(","); 
    // Write the samples, this will always write 8 channels of data even if fewer than 8 sensors are present
    for (byte channel = 0; channel < MAX_SENSORS; channel++){
      IRFile.print(sampleBuffer[writeLoop][channel]);
      IRFile.print(",");     
    }
    IRFile.print(millisEndBuffer[writeLoop]);
    IRFile.println();
  }
  IRFile.close(); // close this file for now

  //**********************************************************
  // Temperature sampling
  // We arrive here when the RTC reports a seconds value of SAMPLING_LENGTH_SEC or greater
  // At this point we stop sampling the IR sensors and instead
  // take one round of temperature values if the readTempsFlag
  // is currently true. It will be set to false after reading
  // one set of temperature values at the 30 second mark
//  if ( (second(myTime) >= SAMPLING_LENGTH_SEC) & (readTempsFlag == true) ) {
  if ( readTempsFlag == true ) {
    
    readTempsFlag = false;  // Set false so that this only runs once per minute
    for (byte i = 0; i < MAX_SENSORS; i++) {
      if (goodSensors[i] != 127) {
        tcaselect(i);
        max3010x.wakeUp(); // wake up sensor
        delayMicroseconds(10);
        triggerTemperatureSample(); // start temp sample so that it's ready by the
                                    // next cycle through the main loop
        // We won't put the sensor back to sleep on this cycle so that 
        // it can complete its temperature reading
      }
    }
    timer.setTimer(30);  // units milliseconds
    who = Snooze.deepSleep( config_teensy35 ); // Sleep for a few ms
    // This should give sufficient time for the sensors to read
    // their temperatures (about 29 milliseconds)
    unsigned long tempMillis = millis(); // update this for the file
    myTime = Teensy3Clock.get();

    // Reopen logfile. If opening fails, notify the user
    if (!TEMPFile.isOpen()) {
      if (!TEMPFile.open(filename2, O_RDWR | O_CREAT | O_AT_END)) {
        Serial.println("Could not reopen file");
      }
    }
    if (TEMPFile.isOpen()) {
      TEMPFile.print(tempMillis); TEMPFile.print(",");
      TEMPFile.print(myTime); TEMPFile.print(",");
      TEMPFile.print(year(myTime)); TEMPFile.print("-"); TEMPFile.print(month(myTime)); TEMPFile.print("-"); TEMPFile.print(day(myTime));
      TEMPFile.print(" "); TEMPFile.print(hour(myTime)); TEMPFile.print(":"); TEMPFile.print(minute(myTime)); TEMPFile.print(":"); TEMPFile.print(second(myTime));
      TEMPFile.print(",");

      float temperatures [ MAX_SENSORS] = {};
      
      // Loop through sensors in sequence.
      for (byte i = 0; i < MAX_SENSORS; i++) {
        if (goodSensors[i] != 127) {
          tcaselect(i);
          temperatures[i] = readTemperatureSample();
//          TEMPFile.print(readTemperatureSample()); TEMPFile.print(",");
          max3010x.shutDown(); // shut down sensor to save power
        }
      }

      for (byte i = 0; i < MAX_SENSORS; i++){
        TEMPFile.print(temperatures[i]); TEMPFile.print(",");
      }
      
      TEMPFile.print(millis());
      TEMPFile.print(",");
      // Read battery voltage and add to the file
      batteryVolts = readBatteryVoltage(BATT_MONITOR_EN, BATT_MONITOR, dividerRatio, refVoltage, resolutionADC);
      TEMPFile.println(batteryVolts, 3); // Write value to file
      TEMPFile.close();

    }
    else {
      Serial.println("error opening temperature file.");
    }
    // Flash to let user know we just finished the SAMPLING_LENGTH_SEC seconds of IR sampling 
    // and the temperature readings, and will be going to sleep next
    for (int i = 0; i < 4; i++) {
      setColor(0, 255, 0);
      delay(5);
      setColor(0, 0, 0);
      delay(5);
    }
  } // end of if (myTime == SAMPLING_LENGTH_SEC & readTempsFlag == true) section

  //*****************************************************
  // After the temperature samples have been taken, spend the
  // rest of the interval in Snooze to save power

  // Here we assume the currTime value still has a time stored from
  // when the previous minute turned over at the start of the main
  // loop, so we add SAMPLE_INTERVAL_SEC (usually 60 seconds)
  // to it to create an alarm (1 minute) in the future from currTime
  // which should cause the device to reawaken exactly when the real
  // time clock rolls over at the start of the next minute HH:MM:00
//  alarm.setAlarm( currTime + SAMPLE_INTERVAL_SEC );
  alarm.setAlarm( currTime + (INTERVAL_MINUTES * 60) );
  // Hibernate using the config_teensy35_2 configuration which only
  // listens for the RTC Alarm (ignores the faster timer wakeup)
  who = Snooze.hibernate( config_teensy35_2 );
  // When we re-awaken, go back to the top of the main loop and start
  // again
 
  
}  // end of main loop


//*************************************************************************************
// Utility functions
//*************************************************************************************

//******************************
// Select the correct I2C channel on the I2C multiplexer
void tcaselect(uint8_t i) {
  if (i > MAX_SENSORS) return;
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
  printDigits(month(theTime));
  Serial.print(F("-"));
  printDigits(day(theTime));
  Serial.print(" ");
  printDigits(hour(theTime));
  Serial.print(":");
  printDigits(minute(theTime));
  Serial.print(":");
  printDigits(second(theTime));
  Serial.println();
}
//****************************
void printDigits(int digits) {
  // utility function for digital clock display: 
  // prints a leading 0 on time values less than 10
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

//-------------------------------------------
// LED color-setting function. Feed it 3 values for
// red, green, blue, between 0 to 256
void setColor(int red, int green, int blue)
{
  // Brightness values run from 0 to 255, but a value of 256
  // is needed to fully shut the LED off on Teensy
#ifdef COMMON_ANODE
  red = 256 - red;
  green = 256 - green;
  blue = 256 - blue;
#endif
  analogWrite(REDLED, red);
  analogWrite(GRNLED, green);
  analogWrite(BLUELED, blue);
}

//*********************************************
// Function to create a fileName based on the current time
void initFileName(SdFs& sd, FsFile& IRFile, time_t time1, char *filename, bool serialValid, char *serialNumber) {

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

    if (!sd.exists(filename)) {
      // when sd.exists() returns false, this block
      // of code will be executed to open a file with this new filename
      IRFile = sd.open(filename, O_RDWR | O_CREAT | O_AT_END);
      //      if (!IRFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      //        Serial.println("Didn't open IRFile initially");
      //        delay(5);
      //      }
      break; // Break out of the for loop when the
      // statement if(!IRFile.exists())
      // is finally false (i.e. you found a new file name to use).
    } // end of if(!sd.exists())
  } // end of file-naming for loop
  //------------------------------------------------------------
  // Write 1st header line
  IRFile.print("startMillis,");
  IRFile.print("UnixTime"); IRFile.print(",");
  IRFile.print("DateTime"); IRFile.print(",");
  for (int i = 0; i < MAX_SENSORS; i++) { // Write column names for each sensor
      IRFile.print("Sensor"); IRFile.print(i + 1); IRFile.print("IR,");
  }
  IRFile.println("endMillis");

  // Update the file's creation date, modify date, and access date.
  IRFile.timestamp(T_CREATE, year(time1), month(time1), day(time1),
                   hour(time1), minute(time1), second(time1));
  IRFile.timestamp(T_WRITE, year(time1), month(time1), day(time1),
                   hour(time1), minute(time1), second(time1));
  IRFile.timestamp(T_ACCESS, year(time1), month(time1), day(time1),
                   hour(time1), minute(time1), second(time1));
  IRFile.close(); // force the data to be written to the file by closing it
} // end of initFileName function
//*********************************************
// Function to create a fileName for Temp file based on the current time
void initTempFileName(SdFs& sd, FsFile& TEMPFile, time_t time1, char *filename2, bool serialValid, char *serialNumber, char *temp) {

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
  // the same minute. This shouldn't come into play
  // during a normal data run, but can be useful when
  // troubleshooting.
  for (uint8_t i = 0; i < 100; i++) {
    filename2[14] = i / 10 + '0';
    filename2[15] = i % 10 + '0';

    if (!sd.exists(filename2)) {
      // when sd.exists() returns false, this block
      // of code will be executed to open a file with this new filename
      TEMPFile = sd.open(filename2, O_RDWR | O_CREAT | O_AT_END);
      //      if (!IRFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      //        Serial.println("Didn't open IRFile initially");
      //        delay(5);
      //      }
      break; // Break out of the for loop when the
      // statement if(!IRFile.exists())
      // is finally false (i.e. you found a new file name to use).
    } // end of if(!sd.exists())
  } // end of file-naming for loop
  //------------------------------------------------------------
  // Write 1st header line for temp file
  TEMPFile.print("startMillis"); TEMPFile.print(",");
  TEMPFile.print("UnixTime"); TEMPFile.print(",");
  TEMPFile.print("DateTime"); TEMPFile.print(",");
  for (int i = 0; i < MAX_SENSORS; i++) { // Write column names for each sensor
      TEMPFile.print("Sensor"); TEMPFile.print(i + 1); TEMPFile.print("TempC,");
  }
  TEMPFile.print("endMillis");  TEMPFile.print(",");
  TEMPFile.println("Battery.V");

  // Update the file's creation date, modify date, and access date.
  TEMPFile.timestamp(T_CREATE, year(time1), month(time1), day(time1),
                     hour(time1), minute(time1), second(time1));
  TEMPFile.timestamp(T_WRITE, year(time1), month(time1), day(time1),
                     hour(time1), minute(time1), second(time1));
  TEMPFile.timestamp(T_ACCESS, year(time1), month(time1), day(time1),
                     hour(time1), minute(time1), second(time1));
  TEMPFile.close(); // force the data to be written to the file by closing it
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
  max3010x.writeRegister8(0x57, 0x21, 0x01);
}

//-----------------------------------
// Check for new die temperature reading
// This function should be run at least 29 milliseconds after calling
// the triggerTemperatureSample() function which starts the temperature sample

float readTemperatureSample(void) {
  // Read die temperature register (integer)
  int8_t tempInt = max3010x.readRegister8(0x57, 0x1F);
  uint8_t tempFrac = max3010x.readRegister8(0x57, 0x20); //Causes the clearing of the DIE_TEMP_RDY interrupt

  // Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}

//-----------------------------------------------
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
//  uint32_t tempIR = max3010x.getFIFOIR();
  //  digitalWriteFast(IRPIN, LOW);  // troubleshooting, can comment out
  return (tempIR);

} // end of quickSampleIR function

///--------------------------------------------------------------

void printTimeOLED(time_t theTime) {
  //------------------------------------------------
  // printTimeOLED function takes a time_t object from
  // the real time clock and prints the date and time
  // to the OLED object.
  oled.print(year(theTime), DEC);
  oled.print('-');
  if (month(theTime) < 10) {
    oled.print("0");
  }
  oled.print(month(theTime), DEC);
  oled.print('-');
  if (day(theTime) < 10) {
    oled.print("0");
  }
  oled.print(day(theTime), DEC);
  oled.print(' ');
  if (hour(theTime) < 10) {
    oled.print("0");
  }
  oled.print(hour(theTime), DEC);
  oled.print(':');
  if (minute(theTime) < 10) {
    oled.print("0");
  }
  oled.print(minute(theTime), DEC);
  oled.print(':');
  if (second(theTime) < 10) {
    oled.print("0");
  }
  oled.print(second(theTime), DEC);
  // You may want to print a newline character
  // after calling this function i.e. Serial.println();
}


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

//--------------------------------------------------------------------
//------------readBatteryVoltage-------------------
// readBatteryVoltage function. This will read the AD convertor
// and calculate the approximate battery voltage (before the
// voltage regulator). Returns a floating point value for
// voltage.
float readBatteryVoltage (int BATT_MONITOR_EN, int BATT_MONITOR, double dividerRatio, double refVoltage, int resolutionADC) {
  // Turn on the battery voltage monitor circuit
  digitalWrite(BATT_MONITOR_EN, HIGH);
  delay(1);
  // Read the analog input pin
  unsigned int rawAnalog = 0;
  analogRead(BATT_MONITOR); // This initial value is ignored
  delay(3); // Give the ADC time to stablize
  // Take 4 readings
  for (byte i = 0; i < 4; i++) {
    rawAnalog = rawAnalog + analogRead(BATT_MONITOR);
    delay(2);
  }
  // Do a 2-bit right shift to divide rawAnalog
  // by 4 to get the average of the 4 readings
  rawAnalog = rawAnalog >> 2;
  // Shut off the battery voltage sense circuit
  digitalWrite(BATT_MONITOR_EN, LOW);
  // Convert the rawAnalog count value (0-4096) into a voltage
  // Relies on global variables dividerRatio and refVoltage
  double reading = rawAnalog * dividerRatio * refVoltage / (double)resolutionADC;
  return reading; // return voltage result
}


//****************************************
void printSensorOLED(uint8_t i, uint32_t sensorValue){
  // This is based on using a 5x7 font in 1x mode (each character is 5 columns (pixels) wide)
  switch(i){
    case 0:
//    oled.clearField(0,0,55); // Clear 55 pixels = 11 characters x 5 pixels per character)
    oled.clearField(0,0,99); // Clear 55 pixels = 11 characters x 8 pixels per character)
    oled.setCursor(0,0);
    oled.print("1: ");
    if(sensorValue > 0){
      oled.print(sensorValue);  // just print regular values
    } else if (sensorValue == 0){
      oled.print(sensorValue); oled.print("     <-"); // help visually identify the zero
    }
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
