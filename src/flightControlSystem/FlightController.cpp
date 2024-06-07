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

#include "FlightController.h"

void FlightController::setup() {
  flightModeFSM = flightModeFSM;

  _altitudeController.initialize(
    ALTITUDE_KP,
    ALTITUDE_KI,
    ALTITUDE_KD,
    MIN_THROTTLE_PID_INPUT,
    MAX_THROTTLE_PID_INPUT
  );
  _yawController.initialize(
    YAW_KP,
    YAW_KI,
    YAW_KD,
    MIN_MOTOR_PID_INPUT,
    MAX_MOTOR_PID_INPUT
  );
  _pitchController.initialize(
    PITCH_KP,
    PITCH_KI,
    PITCH_KD,
    MIN_MOTOR_PID_INPUT,
    MAX_MOTOR_PID_INPUT
  );
  _rollController.initialize(
    ROLL_KP,
    ROLL_KI,
    ROLL_KD,
    MIN_MOTOR_PID_INPUT,
    MAX_MOTOR_PID_INPUT
  );

  _altitudeController.begin();
  _yawController.begin();
  _pitchController.begin();
  _rollController.begin();
}

ControlCommands FlightController::compute(
  StateVector referenceState,
  StateVector measuredState
) {
  ControlCommands result;

  if (flightModeFSM.isMode(TAKEOFF)) {
    result.throttle = (1 + TAKEOFF_GAIN) *
      MOTOR_INPUT_TO_HOVER;
  }
  else if (flightModeFSM.isMode(LANDING)) {
    result.throttle = (1 - LANDING_GAIN) *
      MOTOR_INPUT_TO_HOVER;
  }
  else {
    result.throttle = MOTOR_INPUT_TO_HOVER +
      _altitudeController.compute(
        referenceState.altitude,
        measuredState.altitude
      );
  }

  result.yaw = _yawController.compute(
    0,
    0,
    referenceState.yawRate
  );

  result.pitch = _pitchController.compute(
    referenceState.pitch,
    measuredState.pitch,
    measuredState.pitchRate
  );

  result.roll = _rollController.compute(
    referenceState.roll,
    measuredState.roll,
    measuredState.rollRate
  );

  return result;
}
