#include "Drone.h"

Drone::Drone(
  int motorOnePin,
  int motorTwoPin,
  int motorThreePin,
  int motorFourPin,
  int minMotorInput,
  int maxMotorInput
) {
  _motorOne.attach(
    motorOnePin,
    minMotorInput,
    maxMotorInput,
    true,
    false
  );
  _motorTwo.attach(
    motorTwoPin,
    minMotorInput,
    maxMotorInput,
    false,
    false
  );
  _motorThree.attach(
    motorThreePin,
    minMotorInput,
    maxMotorInput,
    false,
    true
  );
  _motorFour.attach(
    motorFourPin,
    minMotorInput,
    maxMotorInput,
    true,
    true
  );
}

void Drone::sendControlInputs(
  int throttleInput,
  int yawInput,
  int pitchInput,
  int rollInput
) {
  _motorOne.setSpeed(
    throttleInput,
    yawInput,
    pitchInput,
    rollInput
  );
  _motorTwo.setSpeed(
    throttleInput,
    yawInput,
    pitchInput,
    rollInput
  );
  _motorThree.setSpeed(
    throttleInput,
    yawInput,
    pitchInput,
    rollInput
  );
  _motorFour.setSpeed(
    throttleInput,
    yawInput,
    pitchInput,
    rollInput
  );
}
