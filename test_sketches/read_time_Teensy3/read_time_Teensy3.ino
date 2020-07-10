#include <TimeLib.h>  // Download from https://github.com/PaulStoffregen/Time

TimeElements tm; // Create a TimeElements object

time_t myTime;
unsigned long millisVal;

void setup() {
  Serial.begin(115200); // Speed doesn't matter for Teensy3 devices
  while (!Serial); // Wait for serial port to connect
  Serial.println(F("Hello"));
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  
  millisVal = millis();
}

void loop() {
if ( (millis() - millisVal) >  1000){
    millisVal = millis(); // Update millisVal
    
    myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
    Serial.print(F("RTC time: "));
    digitalClockDisplay(now());
  }

}

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

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
