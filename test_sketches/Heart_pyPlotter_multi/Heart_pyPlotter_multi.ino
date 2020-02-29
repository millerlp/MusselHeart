/*
 * Testing a python plotter with Teensy 3.5
 * Works along with Heart_multiPlotter1.py python script
 * 
 */

unsigned long timer = 0;
long loopTime = 20000;   // microseconds
float val1 = 2.0; // Reminder: on Teensy float and double are different sizes
float val2 = 40.0;
float val3 = 80.0;
float stepSize = 10.0;
float minVal = 0.0;
float maxVal = 512.0;
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

  val1 = rampFunc(val1, stepSize, minVal, maxVal, climb);
  val2 = rampFunc(val2, stepSize, minVal, maxVal, climb);
  val3 = rampFunc(val3, stepSize, minVal, maxVal, climb);
  if (val1 > maxVal | val2 > maxVal | val3 > maxVal) climb = !climb;
  if (val1 < minVal | val2 < minVal | val3 < minVal) climb = !climb;
//  Serial.print(val1);
//  Serial.print(", ");
//  Serial.print(val2);
//  Serial.print(", ");
//  Serial.println(val3);
  sendToPC(&val1, &val2, &val3);
//  Serial.println(val);
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
void sendToPC(float* data1, float* data2, float* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}


//***************
float rampFunc(float val, float stepSize, float minVal, float maxVal, bool climb) {
  if (climb & (val < maxVal) ){
    val = val + stepSize;
  } else if ( !climb & (val > minVal) ) {
    val = val - stepSize;
  } 
  return val;
}
