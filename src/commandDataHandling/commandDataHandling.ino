#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>

const byte START_MARKER = 255;

const int RX_PIN = 9;
const int TX_PIN = 10;

const int CE_PIN = 5;
const int CSN_PIN = 6;

SoftwareSerial flightControllerComms(
  RX_PIN,
  TX_PIN
);

const byte readAddress[6] = "00001";
RF24 radio(CE_PIN, CSN_PIN);

struct ControlCommands {
  float throttle;
  float yaw;
  float pitch;
  float roll;
};

ControlCommands controlCommands;

int FLIGHT_CONTROLLER_RESET_PIN = 3;

void setup() {

  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, readAddress);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  flightControllerComms.begin(9600);
}

void loop() {

  if (radio.available()) {
    radio.read(&controlCommands, sizeof(controlCommands));
  }
  flightControllerComms.write(START_MARKER);
  flightControllerComms.write(
    (char*)&controlCommands,
    sizeof(controlCommands)
  );
  delay (10);
}
