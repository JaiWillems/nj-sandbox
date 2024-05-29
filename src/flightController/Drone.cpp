#include "Drone.h"

const bool MOTOR_ONE_BOW = true;
const bool MOTOR_ONE_PORT = false;

const bool MOTOR_TWO_BOW = false;
const bool MOTOR_TWO_PORT = false;

const bool MOTOR_THREE_BOW = false;
const bool MOTOR_THREE_PORT = true;

const bool MOTOR_FOUR_BOW = true;
const bool MOTOR_FOUR_PORT = true;

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
    maxMotorInput
  );
  _motorTwo.attach(
    motorTwoPin,
    minMotorInput,
    maxMotorInput
  );
  _motorThree.attach(
    motorThreePin,
    minMotorInput,
    maxMotorInput
  );
  _motorFour.attach(
    motorFourPin,
    minMotorInput,
    maxMotorInput
  );
  _minMotorInput = minMotorInput;
  _maxMotorInput = maxMotorInput;
}

void Drone::sendControlInputs(
  int throttleInput,
  int yawInput,
  int pitchInput,
  int rollInput
) {
  _motorOne.setSpeed(
    mixControlInputs(
      MOTOR_ONE_BOW,
      MOTOR_ONE_PORT,
      throttleInput,
      yawInput,
      pitchInput,
      rollInput
    )
  );
  _motorTwo.setSpeed(
    mixControlInputs(
      MOTOR_TWO_BOW,
      MOTOR_TWO_PORT,
      throttleInput,
      yawInput,
      pitchInput,
      rollInput
    )
  );
  _motorThree.setSpeed(
    mixControlInputs(
      MOTOR_THREE_BOW,
      MOTOR_THREE_PORT,
      throttleInput,
      yawInput,
      pitchInput,
      rollInput
    )
  );
  _motorFour.setSpeed(
    mixControlInputs(
      MOTOR_FOUR_BOW,
      MOTOR_FOUR_PORT,
      throttleInput,
      yawInput,
      pitchInput,
      rollInput
    )
  );
}

int Drone::mixControlInputs(
  bool bow,
  bool port,
  float throttleInput,
  float yawInput,
  float pitchInput,
  float rollInput
) {
  bool motorCcw = isMotorCcw(bow, port);

  int signedYawInput = motorCcw ? yawInput : -yawInput;
  int signedPitchInput = bow ? pitchInput : -pitchInput;
  int signedRollInput = port ? rollInput : -rollInput;

  float motorInput = throttleInput;
  motorInput = motorInput + signedYawInput;
  motorInput = motorInput + signedPitchInput;
  motorInput = motorInput + signedRollInput;

  return motorInput;
}

bool Drone::isMotorCcw(bool bow, bool port) {
  return (bow && !port) || (!bow && port);
}
