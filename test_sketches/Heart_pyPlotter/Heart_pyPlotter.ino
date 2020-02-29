/*
 * Testing a python plotter with Teensy 3.5
 * 
 */

unsigned long timer = 0;
long loopTime = 50000;   // microseconds
float val = 2.0; // Reminder: on Teensy float and double are different sizes
float stepSize = 10.0;
bool climb = true;
#define ledPin 13

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
  Serial.begin(9600); // speed doesn't matter for Teensy3.5
  while (!Serial) {
    // Wait for serial connection to establish
  }
//  Serial.println("Hello");
  digitalWrite(ledPin, LOW);
  timer = micros();
}
 
void loop() {
  timeSync(loopTime);

  if (climb & (val < 512.0) ){
    val = val + stepSize;
  } else if ( climb & (val >= 512.0) ){
    climb = false; // reverse direction on next loop
  } else if ( !climb & (val > 0.0) ) {
    val = val - stepSize;
  } else if ( !climb & (val <= 0.0 ) ) {
    climb = true; // reverse direction on next loop
  }

  sendToPC(&val);
//  Serial.println(val);
//  sendToPC(val);
  digitalWrite(ledPin, !digitalRead(ledPin));

}
 
void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}
 
// On Teensy, a float is 4 bytes, while a double is 8 bytes
void sendToPC(float* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}

// On Teensy, a double is 8 bytes
void sendToPC(double* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 8);
}
