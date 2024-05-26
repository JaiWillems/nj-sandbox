#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int CE_PIN = 5;
int CSN_PIN = 6;

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
}

void loop() {

  if (radio.available()) {
    radio.read(&controlPacket, sizeof(controlPacket));
  }

  int thrustInput = controlPacket.thrustInput;
  int yawInput = controlPacket.yawInput;
  int pitchInput = controlPacket.pitchInput;
  int rollInput = controlPacket.rollInput;
}
