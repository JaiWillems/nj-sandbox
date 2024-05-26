#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>

const int RX_pin = 9;
const int TX_PIN = 10;

const int CE_PIN = 5;
const int CSN_PIN = 6;

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

  flightControllerComms.begin(9600);
}

void loop() {

  if (radio.available()) {
    radio.read(&controlPacket, sizeof(controlPacket));
  }
  
  if (flightControllerComms.available()){
    flightControllerComms.write((char*)&controlPacket); 
  }
}
