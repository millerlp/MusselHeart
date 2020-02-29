long loopTime = 1000;   // milliseconds
//double val = 0.0; // Note that for 32-bit Teensy float is 4-byte, double is 8-byte
float val = 0.0; // Note that for 32-bit Teensy float is 4-byte, double is 8-byte
#define ledPin 13

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
  Serial.begin(9600); // speed shouldn't matter for Teensy3.5
  while (!Serial) {
    // Wait for serial connection to establish
  }
//  Serial.println("Hello"); // Works fine on Teensy or Uno, when uncommented
  digitalWrite(ledPin, LOW);
}
 
void loop() {
  delay(loopTime);
  sendToPC(&val);

  val = val + 1.0;
  digitalWrite(ledPin, !digitalRead(ledPin));

}
 
 
 // Version for sending a 4-byte floating point value
void sendToPC(float* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}

// Version for sending 8-byte double precision floating point number
// But note that double is only larger on 32-bit ARM Teensy, not 8-bit AVR
void sendToPC(double* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 8);
}
