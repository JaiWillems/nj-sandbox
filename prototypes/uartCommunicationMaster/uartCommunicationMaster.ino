#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 4);

byte numberToSend = 0;


void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  mySerial.write(numberToSend);
  Serial.print("Sent: ");
  Serial.println(numberToSend);

  delay(1000);

  // while (!mySerial.available()) {
  //   // Wait for response
  // }

  // if (mySerial.available()) {
  //   numberToSend = mySerial.read();
  //   Serial.print("Received: ");
  //   Serial.println(numberToSend);
  //   numberToSend++;
  //   delay(1000);
  // }
}