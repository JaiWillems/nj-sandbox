#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 4);

byte receivedNumber;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  delay (1000);
  while (!mySerial.available()) {
    Serial.println("Serial unavailable");
  }

  if (mySerial.available()) {
    receivedNumber = mySerial.read();
    Serial.print("Received: ");
    Serial.println(receivedNumber);
    // receivedNumber++;
    // mySerial.write(receivedNumber);
    // Serial.print("Sent: ");
    // Serial.println(receivedNumber);
    // delay(1000); // Small delay to avoid too fast sending
  } 
}