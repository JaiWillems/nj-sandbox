#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int COMMANDING_FREQUENCY_HZ = 100;

const int THRUST_AXIS_PIN = A1;
const int YAW_AXIS_PIN = A2;
const int PITCH_AXIS_PIN = A4;
const int ROLL_AXIS_PIN = A3;

const int CE_PIN = 7;
const int CSN_PIN = 8;

const byte writeAddress[6] = "00001";

RF24 radio(CE_PIN, CSN_PIN);

struct ControlCommands {
  float throttle;
  float yaw;
  float pitch;
  float roll;
};

void setup() {
  
  pinMode(THRUST_AXIS_PIN, INPUT);
  pinMode(YAW_AXIS_PIN, INPUT);
  pinMode(PITCH_AXIS_PIN, INPUT);
  pinMode(ROLL_AXIS_PIN, INPUT);

  radio.begin();
  radio.openWritingPipe(writeAddress);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  
  int thrustInput = analogRead(THRUST_AXIS_PIN);
  int yawInput = analogRead(YAW_AXIS_PIN);
  int pitchInput = analogRead(PITCH_AXIS_PIN);
  int rollInput = analogRead(ROLL_AXIS_PIN);

  struct ControlCommands controlCommands = {
    thrustInput,
    yawInput,
    pitchInput,
    rollInput
  };

  radio.write(&controlCommands, sizeof(controlCommands));

  delay(1000 / COMMANDING_FREQUENCY_HZ);
}
