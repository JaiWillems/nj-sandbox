/*
BSD 3-Clause License

Copyright (c) 2024, Nishant Kumar, Jai Willems

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "Drone.h"

const bool MOTOR_ONE_BOW = true;
const bool MOTOR_ONE_PORT = false;

const bool MOTOR_TWO_BOW = false;
const bool MOTOR_TWO_PORT = false;

const bool MOTOR_THREE_BOW = false;
const bool MOTOR_THREE_PORT = true;

const bool MOTOR_FOUR_BOW = true;
const bool MOTOR_FOUR_PORT = true;

const int MOTOR_ARM_TIME = 5000;

void Drone::setup(
  int motorOnePin,
  int motorTwoPin,
  int motorThreePin,
  int motorFourPin
) {
  _motorOne.attach(
    motorOnePin
  );
  _motorTwo.attach(
    motorTwoPin
  );
  _motorThree.attach(
    motorThreePin
  );
  _motorFour.attach(
    motorFourPin
  );
}

void Drone::arm() {
  _motorOne.arm();
  _motorTwo.arm();
  _motorThree.arm();
  _motorFour.arm();
  
  delay(MOTOR_ARM_TIME);
}

void Drone::sendControlInputs(
  ControlCommands controlCommands
) {
  _motorOne.setSpeed(
    mixControlInputs(
      MOTOR_ONE_BOW,
      MOTOR_ONE_PORT,
      controlCommands
    )
  );
  _motorTwo.setSpeed(
    mixControlInputs(
      MOTOR_TWO_BOW,
      MOTOR_TWO_PORT,
      controlCommands
    )
  );
  _motorThree.setSpeed(
    mixControlInputs(
      MOTOR_THREE_BOW,
      MOTOR_THREE_PORT,
      controlCommands
    )
  );
  _motorFour.setSpeed(
    mixControlInputs(
      MOTOR_FOUR_BOW,
      MOTOR_FOUR_PORT,
      controlCommands
    )
  );
}

int Drone::mixControlInputs(
  bool bow,
  bool port,
  ControlCommands controlCommands
) {
  float throttleInput = controlCommands.throttle;
  float yawInput = controlCommands.yaw;
  float pitchInput = controlCommands.pitch;
  float rollInput = controlCommands.roll;

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
