/*
  SD card read/write

 This example shows how to read and write data to and from an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
//#include <SD.h>
#include "SdFat.h"
const uint8_t SD_CS_PIN = SDCARD_SS_PIN; // Set up to use Teensy 3.5 onboard SD card slot
//#define SD_FAT_TYPE 3; // For use with SdFat-beta 2.1.4-beta3 or SdFat 2.1.2
//SdFs SD; 
//FsFile myFile;
//SdFatSdio SD; // Uses Teensy's built-in SD card slot
//File myFile;

//#define SD_FAT_TYPE 0;
//SdExFat sd;
//ExFile myFile;

#define SD_FAT_TYPE 3;
SdFs sd;
FsFile myFile;



void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  if (!sd.begin( SdioConfig(FIFO_SDIO)) ) {
    sd.initErrorHalt("initialization failed!");

  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = sd.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = sd.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void loop() {
  // nothing happens after setup
}
