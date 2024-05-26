#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int CE_PIN = 7;
int CSN_PIN = 8;

const byte readAddress[6] = "00001";

RF24 radio(CE_PIN, CSN_PIN);

struct ControlPacket {
  uint16_t thrustInput;
  uint16_t yawInput;
  uint16_t pitchInput;
  uint16_t rollInput;
};

ControlPacket controlPacket;

void setup() {
  
  Serial.begin(9600);

  radio.begin();
  radio.openReadingPipe(0, readAddress);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  
  if (radio.available()) {
    radio.read(&controlPacket, sizeof(ControlPacket));
  }

  uint16_t thrustInput = controlPacket.thrustInput;
  uint16_t yawInput = controlPacket.yawInput;
  uint16_t pitchInput = controlPacket.pitchInput;
  uint16_t rollInput = controlPacket.rollInput;

  Serial.print("Thrust:\t"); Serial.print(thrustInput); Serial.print("\t");
  Serial.print("Yaw:\t"); Serial.print(yawInput); Serial.print("\t");
  Serial.print("Pitch:\t"); Serial.print(pitchInput); Serial.print("\t");
  Serial.print("Roll:\t"); Serial.println(rollInput);
}
