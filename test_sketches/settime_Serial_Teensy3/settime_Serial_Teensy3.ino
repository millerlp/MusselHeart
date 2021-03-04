/*	settime_Serial_Teensy3.ino
	Luke Miller 2020

  A program to let the user type in a date and time in the 
  Serial monitor that will then be programmed into the onboard
  Real Time Clock on a Teensy 3.0+ board (tested on 3.5). This
  will allow you to set a date and time of your choosing rather
  than relying on the local computer time. This may be useful 
  particularly if you want to set your Teensy real time clock to 
  be in a different time zone (such as Greenwich Mean Time) rather
  than you local time zone. 
	
	To use this program, upload it to your board, then open the
	serial monitor. Make sure the serial monitor 
	is set to send a 'newline' when you hit return (see the 
	menu in the lower right of the Arduino serial monitor window)
	
	You will be prompted to enter a date and time in the format:
			YYYY MM DD HH MM SS
	using 24hr time format.
	For example, July 8, 2015 at 3:12:30 PM (afternoon) would 
	be entered as:
	2015 7 8 15 12 30
	
	Type that in to the serial monitor window, then wait to hit
	Enter until you reach exactly that time. The new time will be
	uploaded immediately, and the result will be printed to the
	serial monitor. If you miss your time, you can enter a 
	new date and time in the same format and try again without
	needing to reboot. 

*/

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

	
	Serial.println(F("Enter a new date and time in the following format")); 
	Serial.println(F("all on one line: "));
	Serial.println(F("\tYYYY MM DD HH MM SS"));
	Serial.println(F("and hit enter when ready to set time"));
	
	millisVal = millis();
	
}

void loop() {
	if ( (millis() - millisVal) >  1000){
		millisVal = millis(); // Update millisVal
    
		myTime = Teensy3Clock.get(); // Read current time from the Teensy rtc
		Serial.print(F("RTC time: "));
    digitalClockDisplay(now());
	}
	// When the user has entered a date and time value in the serial 
	// monitor and hit enter, the following section will execute.
	while (Serial.available() > 0) {
		// Expect the year first
		tm.Year = Serial.parseInt() - 1970; // Year is defined as an offset from 1970
		// Expect month next
		tm.Month = Serial.parseInt();
		// Expect day next
		tm.Day = Serial.parseInt();
		// Expect hour next
		tm.Hour = Serial.parseInt();
		// Expect minute next
		tm.Minute = Serial.parseInt();
		// Expect second next
		tm.Second = Serial.parseInt();
		
		// When the enter symbol '\n' comes along, convert the 
		// values to a DateTime object and set the clock
		if (Serial.read() == '\n'){

			myTime = makeTime(tm); // Update myTime with the new date+time from Serial
      Serial.println(F("Setting time"));
      setTime(myTime); // Set the time value in the current program
      Teensy3Clock.set(now()); // Set the time value on the Teensy RTC hardware
      

      
		}
	}
}



void digitalClockDisplay(time_t theTime) {
  // digital clock display of the time
  Serial.print(year(theTime));
  Serial.print(F("-")); 
  printDigits(month(theTime));
//  Serial.print(month(theTime));   
  Serial.print(F("-"));
  printDigits(day(theTime));
//  Serial.print(day(theTime));
  Serial.print(" ");
  Serial.print(hour(theTime)); Serial.print(":");
  printDigits(minute(theTime)); Serial.print(":");
  printDigits(second(theTime)); 
  Serial.println(); 
}


void printDigits(int digits){
  // utility function for digital clock display: prints leading 0
  if(digits < 10) {
    Serial.print('0');
  }
  Serial.print(digits);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
