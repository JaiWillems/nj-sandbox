#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>

const byte START_MARKER = 255;

const int RX_pin = 9;
const int TX_PIN = 10;

const int CE_PIN = 5;
const int CSN_PIN = 6;

const int LED_PIN = 2;
const int LED_PIN2 = 4;

SoftwareSerial flightControllerComms(9, 10);

const byte readAddress[6] = "00001";
RF24 radio(CE_PIN, CSN_PIN);

struct ControlPacket {
  int thrustInput;
  int yawInput;
  int pitchInput;
  int rollInput;
};

ControlPacket controlPacket;

void setup() {
  radio.begin();
  radio.openReadingPipe(0, readAddress);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
  pinMode(LED_PIN, OUTPUT);

  flightControllerComms.begin(9600);
}

void loop() {
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_PIN2, LOW);
  if (radio.available()) {
    radio.read(&controlPacket, sizeof(controlPacket));
    digitalWrite(LED_PIN, HIGH);
  }
  flightControllerComms.write(START_MARKER);
  int bytesSent= flightControllerComms.write(
  (char*)&controlPacket,
  sizeof(controlPacket)
  );
  if (bytesSent >= sizeof(&controlPacket)){
    digitalWrite(LED_PIN2, HIGH);
  }
  

  delay (10);
}
