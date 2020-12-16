/* LED_brightness_load.ino
  Luke Miller 2020-12-16
  
  For MusselHeart Teensy3.5 boards. 
  
  This sketch is used to store a set of 8 IR LED
  brightness settings that will be unique to the 
  Teensy3.5 board. 
  
  You must manually enter the brightness settings to be 
  programmed below.
    
  After writing the values to the eeprom after the
  serial number location (addressess 0-3), 
  it reads the values back and prints them to the serial monitor.

*/
#include <EEPROM.h>

// serial number to write to chip. Always increment this value and save this file
byte IRledBrightness[] = {10, 20, 30, 40, 50, 60, 70, 80};
char serialnumber[]="SN01";

void setup(){
  Serial.begin(57600);
  while (!Serial); // wait for serial port to connect
  Serial.println("Hello");


  Serial.println("Writing IR brightness values: ");
  for (byte i = 0; i<8; i++){
    Serial.print(IRledBrightness[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  // Put the contents of IRledBrightness in EEPROM starting at
  // address 4
  EEPROM.put(4, IRledBrightness);

  // Read back the EEPROM contents
  char serialNum[sizeof(serialnumber)];
  EEPROM.get(0, serialNum);
  byte output[sizeof(IRledBrightness)];
  EEPROM.get(4, output);
  
  // Print results to Serial Monitor
  Serial.println(serialNum);
  Serial.print("Read back values: ");
  for (byte i = 0; i<8; i++){
    Serial.print(output[i]);
    Serial.print(" ");
  }
  Serial.println();
      
}

void loop() {
  // Do nothing in main loop.
  while(1);
}
