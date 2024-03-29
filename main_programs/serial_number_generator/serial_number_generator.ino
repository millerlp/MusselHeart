/* serial_number_generator.ino
	Luke Miller 2020-08-04
	
	For MusselHeart Teensy3.5 boards. 
	
	This sketch is used to burn a (hopefully) unique serial 
	number to the EEPROM of a Teensy3.5.
	
	You must manually enter the serial number to be 
	programmed below.
		
	After writing the serial number to the start of
	the eeprom (starting at address 0), it reads the
	value back and prints it out to the serial monitor.

*/
#include <EEPROM.h>

// serial number to write to chip. Always increment this value and save this file
char serialnumber[]="SN17";

void setup(){
	Serial.begin(57600);
  while (!Serial); // wait for serial port to connect
	Serial.println("Hello");


	Serial.print("Writing serial number: ");
	Serial.println(serialnumber);
	
	// Put the contents of serialnumber in EEPROM starting at
	// address 0
	EEPROM.put(0, serialnumber);
	
	char output[sizeof(serialnumber)];
	EEPROM.get(0, output);
	Serial.print("Read serial number: ");
	Serial.println(output);
			
}

void loop() {
	// Do nothing in main loop.
	while(1);
}
