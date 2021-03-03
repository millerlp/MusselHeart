/* HeartBeat_interval_sample.ino
 *  
 *  A version of the main datalogging program to work with
 *  Teensy3.5 and heart rate daughterboard RevB. 
 *  
 *  This version will sample the heart rate sensors at 10Hz 
 *  for 30 seconds, then take a round of temperature readings,
 *  and put everything to sleep for the remainder of the minute
 *  using the Snooze library, in an attempt to lower the overall
 *  power usage.
 *  
 */

#include "MAX30105.h"         // https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#include <Snooze.h>   // https://github.com/duff2013/Snooze
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include "SSD1306Ascii.h"     // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <Wire.h>
#include "SdFat.h"            // https://github.com/greiman/SdFat
#include "EEPROM.h"

#define MAX_SENSORS 8  // Leave this set at 8, even if fewer than 8 sensors are attached
#define FAST_SAMPLE_INTERVAL_MS 100 // units millisecond - this sets sampling rate when active
#define SAMPLE_INTERVAL_SEC 60 // units seconds - this sets how long between sampling bouts
bool readTempsFlag = false;

//------------------------------------------
// Debugging stuff, used with logic analyzer
bool scopePinState = LOW; // for debugging
int scopePin0 = 30; // for debugging

//--------------------------------------
// MAX30105 sensor parameters
MAX30105 particleSensor;
// Create an array to hold numbers of good sensor channels (up to 8)
byte goodSensors[] = {127,127,127,127,127,127,127,127};
byte numgoodSensors = 0;
// sensor configurations
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green. Only use 2
byte REDledBrightness = 1; // low value of 0 shuts it off, 1 is barely on
//byte IRledBrightness = 20; //Options: 0=off to 255=fully on, try 10-30 initially. Too high will make noisy signal
// Define IR led brightness setting for each of the 8 channels
// Options: 0=off to 255=fully on, try 10-30 initially. Too high will make noisy signal
//          Channel =      1   2   3   4   5   6   7   8
byte IRledBrightness[] = {20, 20, 20, 20, 20, 20, 20, 20};
byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32, but only use 1, 2, or 4. 4 is preferred
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
SnoozeBlock config_teensy35(timer, alarm);
SnoozeBlock config_teensy35_2(alarm);
//SnoozeBlock config_teensy35(alarm);   // wake from RTC alarm only
int who; // used to track source of Snooze wakeup
//---------------------------------------
// OLED components
#define MULTIPLE_I2C_PORTS 1 // for ssd1306Ascii library, multiple I2C ports available
#define SCREEN_TIMEOUT 10 //Seconds before OLED display shuts off
#define SCREEN_WIDTH 128 // OLED pixel width
#define SCREEN_HEIGHT 64 // OLED pixel height
//#define OLED_RESET     4 // OLED reset pin num
SSD1306AsciiWire oled(Wire1); // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C //OLED address

//----------------------------------------------
//SD components
SdFatSdio SD; // Uses Teensy's built-in SD card slot
File IRFile; //SD card object 1 (IR data)
File TEMPFile; //SD card object 2 (Temp data)
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
  for (int i = 0; i<10; i++){
    setColor(0,255,0);
    delay(20);
    setColor(0,0,0);
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
  if (numgoodSensors == 0){
    oled.println("No sensors");
    oled.println("found");
  } else {
    oled.print("Found ");
    oled.print(numgoodSensors);
    oled.println(" sensors:");
    for (byte i = 0; i < MAX_SENSORS; i++){
      if (goodSensors[i] != 127){
        oled.print(goodSensors[i]+1); // Start labeling at 1 instead of 0
        oled.print(" ");
      }
    }
    oled.println();
  }
  // Shut the sensors back down for now
  for (byte i = 0; i < MAX_SENSORS; i++) {
    if (goodSensors[i] != 127){
      tcaselect(i);        
      particleSensor.shutDown(); // shut down sensor to save power
    }
  }
  
  //********************************
  // SD card setup
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  digitalClockDisplay(myTime); // digital clock display of the time to Serial
  //SD setup
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    oled.println("No SD card found");
    SD.initErrorHalt("SdFatSdio begin() failed");
    setColor(255,0,0);
  }
  Serial.println("Initialization done.");
  // SD Naming IR file
  initFileName(SD, IRFile, myTime, filename, serialValid, serialNumber);
  Serial.print("Using IR file ");
  Serial.println(filename);
    // SD Naming Temperature file
  initTempFileName(SD, TEMPFile, myTime, filename2, serialValid, serialNumber, temp);
  Serial.print("Using Temp file ");
  Serial.println(filename2);
  //*****************************
  // Check raw battery voltage, see function at bottom of this file
  batteryVolts = readBatteryVoltage(BATT_MONITOR_EN,BATT_MONITOR,dividerRatio,refVoltage,resolutionADC);
  oled.print("Battery: ");
  oled.print(batteryVolts,3);
  oled.println("V");
  //*********************************
  // Idle while waiting for a new minute to start
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  while (second(myTime) != 0){
    alarm.setAlarm(myTime + 1); // set alarm 1 second in future
    Snooze.deepSleep( config_teensy35); // sleep until second rolls over
    oled.clear();
    printTimeOLED(Teensy3Clock.get());
    oled.println();
    oled.print("Waiting...");
    myTime = Teensy3Clock.get(); // update myTime
  }
  // If we exited the while loop above, a new minute has just
  // started
  oled.clear();
  // Manual shut down of SSD1306 oled display driver
  Wire1.beginTransmission(0x3C); // oled1 display address
  Wire1.write(0x80); // oled set to Command mode (0x80) instead of data mode (0x40)
  Wire1.write(0xAE); // oled command to power down (0xAF should power back up)
  Wire1.endTransmission(); // stop transmitting
  
} // end of setup()

void loop() {
  /* TODOs:
   *  1. Check current time from RTC. If new day, open new files
   *  2. If seconds 0-30 of the minute, run the IntervalTimer to 
   *  generate 10Hz interrupts that will trigger a round of heart 
   *  rate sampling
   *  3. If seconds 30 is reached, take a set of temperature readings 
   *  to write to the 2nd file, along with battery voltage
   *  4. Shut down heart sensors and Teensy3.5, using rtcAlarm() function
   *  from Snooze library to trigger a wake up at the start of the next 
   *  minute
   */
  myTime = Teensy3Clock.get();
  digitalWriteFast(scopePin0, HIGH); // debugging, can comment out
  scopePinState = HIGH; // debugging, can comment out
  
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
    initFileName(SD, IRFile, myTime, filename, serialValid, serialNumber);
    // Close the current temperature file
    TEMPFile.close();
    // Start a new temperature file
    initTempFileName(SD, TEMPFile, myTime, filename2, serialValid, serialNumber, temp);
  }
  
  while ( second(myTime) < 30 ) {
    elapsedMillis sampleTimer = 0;
     // Start each time through this loop by reawakening IR sensors
     // Reopen IR logfile. If opening fails, notify the user
    if (!IRFile.isOpen()) {
      if (!IRFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
        Serial.println("Could not reopen file");
      }
    }
     if (IRFile.isOpen()){
       unsigned long myMillis = millis();        
       IRFile.print(myMillis); IRFile.print(","); // keep track of millis() value at start of sampling round
       IRFile.print(myTime); IRFile.print(",");  // print POSIX time value (one long number, seconds since 1970-1-1 00:00)
       IRFile.print(year(myTime)); IRFile.print("-"); IRFile.print(month(myTime)); IRFile.print("-"); IRFile.print(day(myTime));
       IRFile.print(" "); IRFile.print(hour(myTime)); IRFile.print(":"); IRFile.print(minute(myTime)); IRFile.print(":"); IRFile.print(second(myTime));
       IRFile.print(",");
     
       for (byte i = 0; i < MAX_SENSORS; i++) {
        if (goodSensors[i] != 127) {
          tcaselect(i);
          particleSensor.wakeUp(); // Wake up sensor to take sample
          delayMicroseconds(10); // Give the chip a chance to wake up      
  //          digitalWriteFast(IRPIN, HIGH); // troubleshooting, can comment out
          uint32_t tempIR = quickSampleIR();
  //          digitalWriteFast(IRPIN, LOW); // troubleshooting, can comment out
          IRFile.print(tempIR);
          particleSensor.shutDown(); // shut down sensor once sample is taken to save power
  
          if (i < (MAX_SENSORS - 1)) {
              IRFile.print(",");
          }
        }
        if (i >= (MAX_SENSORS - 1) ) { //start new line after the last sensor
            IRFile.print(",");  // Modified to record millis value at end of cycle
            IRFile.println(millis()); // Record the end time of the sampling cycle
            
        }
      }
     } else {
      Serial.println("error opening file.");
     }
     while ( sampleTimer < FAST_SAMPLE_INTERVAL_MS){
        // chill out waiting for next sample interval
     }
     // debugging chunk, can comment out
     digitalWriteFast(scopePin0, !scopePinState);
     if (scopePinState == HIGH){
        scopePinState = LOW;
      } else {
        scopePinState = HIGH;
      }
      // end of debugging chunk
     readTempsFlag = true;
     myTime = Teensy3Clock.get(); // update myTime
     
  }  // end of while loop up to 30 seconds
  // while loop quits after 30 seconds of IR samples have been taken
  
  

  //**********************************************************
  // Temperature sampling 
  // We arrive here when the RTC reports a seconds value of 30
  // At this point we stop sampling the IR sensors and instead 
  // take one round of temperature values if the readTempsFlag 
  // is currently true. It will be set to false after reading
  // one set of temperature values at the 30 second mark
  if ( (second(myTime) == 30) & (readTempsFlag == true) ) {
    IRFile.close(); // close this file for now  
    readTempsFlag = false;  // Set false so that this only runs once per minute
    for (byte i = 0; i < MAX_SENSORS; i++) {
      if (goodSensors[i] != 127){
        tcaselect(i);    
        particleSensor.wakeUp(); // wake up sensor
        delayMicroseconds(10);     
        triggerTemperatureSample(); // start temp sample so that it's ready by the next cycle through the main loop  
        // We won't put the sensor back to sleep on this cycle so that it can complete its temperature reading
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
  
      // Loop through sensors in sequence. 
      for (byte i = 0; i < MAX_SENSORS; i++) {
        if (goodSensors[i] != 127){
          tcaselect(i);
          TEMPFile.print(readTemperatureSample()); TEMPFile.print(",");          
          particleSensor.shutDown(); // shut down sensor to save power
        }
      }
      TEMPFile.print(millis());
      TEMPFile.print(",");
      // Read battery voltage and add to the file
      batteryVolts = readBatteryVoltage(BATT_MONITOR_EN,BATT_MONITOR,dividerRatio,refVoltage,resolutionADC);
      TEMPFile.println(batteryVolts,3); // Write value to file
      TEMPFile.close();  
       
    }
    else {
      Serial.println("error opening temperature file.");
    }
    for (int i = 0; i<4; i++){
      setColor(0,255,0);
      delay(5);
      setColor(0,0,0);
      delay(5);
    }
  } // end of if (myTime == 30 & readTempsFlag == true) section
  
  //*****************************************************
  // After the temperature samples have been taken, spend the
  // rest of the minute in Snooze to save power
  
  alarm.setAlarm( currTime + SAMPLE_INTERVAL_SEC );
  // Hibernate using the config_teensy35_2 configuration which only
  // listens for the RTC Alarm (ignores the faster timer wakeup)
  who = Snooze.hibernate( config_teensy35_2 );
  // When we re-awaken, go back to the top of the main loop and start
  // again
  digitalWriteFast(scopePin0, LOW); // debugging, can comment out
  scopePinState = LOW; // debugging, can comment out
}  // end of main loop


//******************************
// Utility functions
//******************************

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
  // utility function for digital clock display: prints preceding colon and leading 0
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
void initFileName(SdFatSdio& SD, File& IRFile, time_t time1, char *filename, bool serialValid, char *serialNumber) {

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
      IRFile = SD.open(filename, O_RDWR | O_CREAT | O_AT_END);
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
    if (goodSensors[i] != 127) { // Only print column headers for active sensors
      IRFile.print("Sensor"); IRFile.print(goodSensors[i]+1); IRFile.print("IR,");
    }
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
void initTempFileName(SdFatSdio& SD, File& TEMPFile, time_t time1, char *filename2, bool serialValid, char *serialNumber, char *temp) {

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
      TEMPFile = SD.open(filename2, O_RDWR | O_CREAT | O_AT_END);
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
    if (goodSensors[i] != 127){
      TEMPFile.print("Sensor"); TEMPFile.print(goodSensors[i]+1); TEMPFile.print("TempC,");
    }
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
//  digitalWriteFast(IRDELAYPIN, HIGH);  // troubleshooting, can comment out
  // Clear the MAX30105 FIFO buffer so that there will only be one new sample to read
  particleSensor.clearFIFO();
  uint16_t ledSampleTime; // units will be microseconds
 
  // Implement a delay for new samples to be collected in the FIFO. If sample averaging
  // is used, a loop will need to execute multiple times to allow the multiple samples
  // to be collected
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
//  digitalWriteFast(IRDELAYPIN, LOW); // troubleshooting, can comment out
//  digitalWriteFast(IRPIN, HIGH); // troubleshooting, can comment out
  uint32_t tempIR = particleSensor.getIR();
//  digitalWriteFast(IRPIN, LOW);  // troubleshooting, can comment out
  return (tempIR);
  
} // end of quickSampleIR function

///--------------------------------------------------------------

void printTimeOLED(time_t theTime){
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
  if (hour(theTime) < 10){
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

void scanSetupSensors (void){
  numgoodSensors = 0; // reset to zero each time this function is called
  for (byte i = 0; i < MAX_SENSORS; i++) {
    tcaselect(i);
    delayMicroseconds(20);
    goodSensors[i] = 127; // Reset this value before scanning for the sensor
    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
      {
        // If sensor is present, mark it in the goodSensors array
        goodSensors[i] = i;
        numgoodSensors++;
      } else {
        // If sensor didn't show up, wait a bit and try a 2nd time
        delay(5);
        if(particleSensor.begin(Wire, I2C_SPEED_FAST)){
          goodSensors[i] = i;
          numgoodSensors++;
        }
      }
      // If the sensor was marked good, set it up for our sampling needs
    if (goodSensors[i] != 127){
      particleSensor.setup(IRledBrightness[i], sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
      particleSensor.enableDIETEMPRDY(); //enable temp ready interrupt. Required to log temp, but each read takes 29ms
      // Tweak individual settings
      particleSensor.setPulseAmplitudeRed(REDledBrightness); // essentially turn off red LED to save power, we only want IR LED. **** commented for testing only
      particleSensor.setPulseAmplitudeIR(IRledBrightness[i]); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)
    }
  }
}

//--------------------------------------------------------------------
//------------readBatteryVoltage-------------------
// readBatteryVoltage function. This will read the AD convertor
// and calculate the approximate battery voltage (before the
// voltage regulator). Returns a floating point value for
// voltage.
float readBatteryVoltage (int BATT_MONITOR_EN, int BATT_MONITOR, double dividerRatio, double refVoltage,int resolutionADC){
    // Turn on the battery voltage monitor circuit
    digitalWrite(BATT_MONITOR_EN, HIGH);
    delay(1);
    // Read the analog input pin
    unsigned int rawAnalog = 0;
    analogRead(BATT_MONITOR); // This initial value is ignored
    delay(3); // Give the ADC time to stablize
    // Take 4 readings
    for (byte i = 0; i<4; i++){
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
