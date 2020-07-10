// Blink.ino
/* Basic test of the RGB LED on HeartRate daughterboard  
 *  RevB hardware. Uses a common anode led APTF1616SEEZGKQBKC
 *  from Kingbright.
 *  
 *  
 */

int REDLED = 7; // PB1
int GRNLED = 6; // PD5
int BLUELED = 5; // PD6

#define COMMON_ANODE

void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);  
  digitalWrite(REDLED, HIGH); // for common anode LED, set high to shut off
  digitalWrite(GRNLED, HIGH);
  digitalWrite(BLUELED, HIGH);  
}

void loop() {
  setColor(10, 0, 0);  // red
  delay(1000);
  setColor(0, 10, 0);  // green
  delay(1000);
  setColor(0, 0, 10);  // blue
  delay(1000);
//  setColor(115, 127, 0);  // yellow
//  delay(1000);  
//  setColor(80, 0, 80);  // purple
//  delay(1000);
//  setColor(0, 127, 120);  // aqua
//  delay(1000);
  setColor(0,0,0);
  delay(3000);
}

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
