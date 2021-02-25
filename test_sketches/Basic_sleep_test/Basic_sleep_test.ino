/* Basic sleep test
 * 
 * You can use Snooze.sleep(), Snooze.deepSleep(), or
 * Snooze.hibernate() to put the Teensy to sleep
 * 
 * Current version: Wakes based on a real time clock alarm
 * TODO: Figure out if the millisecond timer also only starts
 * counting from when it goes to sleep, or it can be set to 
 * run continuously and wake on a specified count. 
 * 
 * To trigger a wakeup at a specific real world time, use
 * the alarm.setAlarm() function, and feed it a time_t value
 * So if you want to wake 5 seconds in the future, you can
 * get the current time with myTime = Teensy3Clock.get() and 
 * then feed that value + 5 seconds to alarm.setAlarm(myTime+5);
 * Then the next time the device goes to Snooze, it will be set to
 * awaken at that new timestamp 5 seconds in the future. 
 * 
 */

#include <Snooze.h>   // https://github.com/duff2013/Snooze
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include "SSD1306Ascii.h"     // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <Wire.h>

TimeElements tm; // Create a TimeElements object
time_t myTime;
unsigned long millisVal;

//SnoozeTimer timer;  // LowPowerTimer millisecond timer object
SnoozeAlarm  alarm; // RTC alarm object
//SnoozeBlock config_teensy35(timer, alarm);
SnoozeBlock config_teensy35(alarm);   // wake from RTC alarm only
unsigned long awakeTime = 3000;


// OLED components
#define MULTIPLE_I2C_PORTS 1 // for ssd1306Ascii library, multiple I2C ports available
#define SCREEN_TIMEOUT 10 //Seconds before OLED display shuts off
#define SCREEN_WIDTH 128 // OLED pixel width
#define SCREEN_HEIGHT 64 // OLED pixel height
//#define OLED_RESET     4 // OLED reset pin num
SSD1306AsciiWire oled(Wire1); // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C //OLED address
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
//  Serial.begin(115200);
//  while(!Serial); // wait for serial monitor to connect
//  Serial.println("Hello");
  delay(200);
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
// Teensy 3.x/LC Set Low Power Timer wake up in Milliseconds.
// MAX: 65535ms
//  timer.setTimer(5000);// milliseconds
//  alarm.setRtcTimer(0, 0, 5);// hour, min, sec - wakes up this long after going to sleep

  Wire1.begin(); // for OLED display
  Wire1.setSCL(37);
  Wire1.setSDA(38);
  Wire1.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS1);
  Wire1.beginTransmission(I2C_ADDRESS1);
  Wire1.write(0x80); // oled set to Command mode (0x80) instead of data mode (0x40)
  Wire1.write(0xAF); // oled command 0xAF should power back up
  Wire1.endTransmission();
  delay(50);
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.println("Hello");
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  alarm.setAlarm(myTime + 5); // Set an alarm 5 seconds in the future
  printTimeOLED(myTime); // digital clock display of the time to OLED display
  delay(5000); // Give time for user to read OLED screen
  oled.clear();
  time_t oldSeconds = second(myTime);
  while(second(Teensy3Clock.get()) != 0){
    // Wait for a new minute to roll over (seconds value becomes 0)
    if (second(Teensy3Clock.get()) != oldSeconds){
      myTime = Teensy3Clock.get();
      oldSeconds = second(myTime);
      oled.clear();
      oled.println("Wait for rollover");
      printTimeOLED(myTime); 
    }

  }
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  alarm.setAlarm(myTime + 60); // Set an alarm x seconds in the future
  oled.clear();
  printTimeOLED(myTime);
  
}

void loop() {
  /********************************************************
    feed the sleep function its wakeup parameters. Then go
    to deepSleep (or hibernate, or just sleep).
  ********************************************************/
  int who;
  who = Snooze.deepSleep( config_teensy35 );// go to sleep and return module that woke processor
  elapsedMillis waketime = 0; // reset time awake counter
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  alarm.setAlarm(myTime + 60); // Set next alarm x seconds in the future
  oled.clear();
  printTimeOLED(myTime); // digital clock display of the time to OLED display
  if (who == 35) { // rtc wakeup value
    for (int i = 0; i < 4; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
    oled.println();
    oled.print("RTC alarm");
  }
  if (who == 36) { // lptmr wakeup value
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
    oled.println();
    oled.print("timer alarm");
  }

  while (waketime < awakeTime){
    // Do nothing here
  }
  
//  Serial.print(F("RTC time: "));
//    digitalClockDisplay(now());
//  Serial.print("Wake trigger = ");
//  Serial.println(who);
//  delay(200);
}



//------------------------------------------------------------
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

//--------------------------------------------------------------------
void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

//----------------------------------------------
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}


//----------------------------------------------------------------------
void printTimeOLED(time_t theTime){
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
