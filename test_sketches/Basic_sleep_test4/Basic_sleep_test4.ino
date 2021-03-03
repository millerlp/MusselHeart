/* Basic sleep test4
 * 
 * You can use Snooze.sleep(), Snooze.deepSleep(), or
 * Snooze.hibernate() to put the Teensy to sleep
 * 
 * Current version: Wakes and hibernates on a 10Hz (100ms)
 * interval, writing loop time updates to the OLED screen
 * 
 * Also toggles pin 30 so you can read out the actual loop time
 * with a digital logic analyzer. You will find that the actual
 * loop time is slightly longer than the millis() function reports
 * because of some missed ticks during the going-to-sleep and 
 * wakeup phases. In deepSleep this is around 1 to 1.5ms jitter
 * in the sleep duration. This can be partially compensated by 
 * shortening the desired sleep duration by 1 millisecond, but there's
 * still some jitter in that fractional millisecond that can't be
 * accounted for, and therefore you'll get accumulated error through
 * time if you rely entirely on the sleep timer function. 
 * 
 * TODO: Reincorporate the RTC wakeup on each minute rollover
 * and use that to trigger a bout of sampling for 30 sec.
 * 
 * TODO: Confirm with a logic analyzer that these loops are
 * actually cycling at the rate the program claims. May need to
 * switch calculations to microseconds if possible?
 * 
 * To trigger a wakeup at a specific real world time, use
 * the alarm.setAlarm() function, and feed it a time_t value
 * So if you want to wake 5 seconds in the future, you can
 * get the current time with myTime = Teensy3Clock.get() and 
 * then feed that value + 5 seconds to alarm.setAlarm(myTime+5);
 * Then the next time the device goes to Snooze, it will be set to
 * awaken at that new timestamp 5 seconds in the future. 
 * 
 * Notes: https://github.com/luni64/TeensyTimerTool gives access
 * to FTM module (flexible timer module) on Teensy3.x, and FTM
 * should be active in VLPR (reduced CPU) and VLPW (sleep) modes (not VLPS, LLS - deepSleep, VLLSx)
 * 
 */

#include <Snooze.h>   // https://github.com/duff2013/Snooze
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include "SSD1306Ascii.h"     // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <Wire.h>

#define SAMPLE_INTERVAL 100  // sampling interval, milliseconds


const int scopePin0 = 30; // pin will be toggled for scope/logic analyzer usage
const int scopePin1 = 31; // pin will be toggled for scope/logic analyzer usage
const int scopePin2 = 32; // pin will be toggled for scope/logic analyzer usage
volatile bool scopePinState = LOW;

TimeElements tm; // Create a TimeElements object
time_t myTime;
unsigned long millisVal;

SnoozeTimer timer;  // LowPowerTimer millisecond timer object
SnoozeAlarm  alarm; // RTC alarm object
SnoozeBlock config_teensy35(timer, alarm);
//SnoozeBlock config_teensy35(alarm);   // wake from RTC alarm only
unsigned long awakeTime = 3000;
int who; // used to track source of Snooze wakeup


// OLED components
#define MULTIPLE_I2C_PORTS 1 // for ssd1306Ascii library, multiple I2C ports available
#define SCREEN_TIMEOUT 10 //Seconds before OLED display shuts off
#define SCREEN_WIDTH 128 // OLED pixel width
#define SCREEN_HEIGHT 64 // OLED pixel height
//#define OLED_RESET     4 // OLED reset pin num
SSD1306AsciiWire oled(Wire1); // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C //OLED address
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
unsigned long t1Millis = 0; // time at start of loop
unsigned long t2Millis = 0; // time at end of doing stuff, just before sleep
unsigned long t3Millis = 0; // time at end snooze, at end of main loop
unsigned long previoust1Millis = 0; // copy of t1Millis from prior loop iteration
int sleepMillis = 0; // milliseconds for sleep timer to run -- this value will be changed on each loop


//*******************************************
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(scopePin0, OUTPUT);
  pinMode(scopePin1, OUTPUT);
  pinMode(scopePin2, OUTPUT);
  digitalWriteFast(scopePin0, scopePinState);
  digitalWriteFast(scopePin1, LOW);
  digitalWriteFast(scopePin2, LOW);
//  Serial.begin(115200);
//  while(!Serial); // wait for serial monitor to connect
//  Serial.println("Hello");
  delay(200);
  // set the Time library to use Teensy 3.5's RTC to keep time
  setSyncProvider(getTeensy3Time);

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
  time_t oldSeconds = second(myTime);
  t1Millis = millis();
  t2Millis = t1Millis;
  t3Millis = t1Millis;
  previoust1Millis = t1Millis;
  oled.clear();
  printTimeOLED(myTime);
  oled.println();

  timer.setTimer(2000);  // Set a long interval so that this doesn't fire during setup
  
  while ( second(Teensy3Clock.get()) == oldSeconds){
    // Wait for a new second to roll over
  }
  
  alarm.setAlarm(Teensy3Clock.get() + 1);
  // Go to sleep and wake up 1 second in the future
  who = Snooze.deepSleep( config_teensy35 );  

}  // end of setup loop

void loop() {
  t1Millis = millis(); // get time at start of doin' stuff
  digitalWriteFast(scopePin0, scopePinState);
  if (scopePinState == HIGH){
    scopePinState = LOW;
  } else {
    scopePinState = HIGH;
  }
  myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
  alarm.setAlarm(myTime + 1); // Set next alarm x seconds in the future
  if (who == 35) { // rtc wakeup value
    
    digitalWriteFast(scopePin1, HIGH);
    delay(1);
//    oled.println();
//    oled.print("RTC alarm");
    digitalWriteFast(scopePin1, LOW);
  }


  
//  unsigned long workDuration = t2Millis - previoust1Millis;
//  unsigned long sleepDuration = t3Millis - t2Millis;
  unsigned long loopDuration = t3Millis - previoust1Millis;
  oled.clear();
  printTimeOLED(myTime); // digital clock display of the time to OLED display
  oled.println();
  oled.println(t3Millis);
  oled.println("Work  Sleep   Total");
//  oled.print(workDuration);
  oled.print("      ");
//  oled.print(sleepDuration);
  oled.print("      ");
  oled.print(loopDuration);



  if (who == 36) { // lptmr wakeup value
//    oled.println();
//    oled.print("timer alarm");
  }


    // End of work section
    //------------------------------------
    
    t2Millis = millis();
    previoust1Millis = t1Millis;
    sleepMillis = SAMPLE_INTERVAL - (t2Millis - t1Millis) - 1;
//    sleepMillis = 50;
    timer.setTimer(sleepMillis);  // Update time needed to sleep, milliseconds
//    digitalWriteFast(scopePin0, LOW); // mark end of work section
//    digitalWriteFast(scopePin2, HIGH);  // mark start of sleep
    // Finish the main loop by going to sleep for the remaining time
    who = Snooze.deepSleep( config_teensy35 );// go to sleep and return module that woke processor
      // accuracy seems to actually be lower in 'sleep' than 'deepSleep' or 'hibernate'
//    digitalWriteFast(scopePin2, LOW); // mark end of sleep
    t3Millis = millis(); // record the time when we reawaken and end the loop
//    digitalWriteFast(scopePin1, LOW); // mark end of loop
    
} // end of main loop()



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
