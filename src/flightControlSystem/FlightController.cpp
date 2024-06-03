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

const int MIN_ESC_INPUT_FROM_PID = -50;
const int MAX_ESC_INPUT_FROM_PID = 50;

const float ALTITUDE_KP = 25.0;
const float ALTITUDE_KI = 0.0;
const float ALTITUDE_KD = 0.0;

const float YAW_RATE_KP = 1.0;
const float YAW_RATE_KI = 0.0;
const float YAW_RATE_KD = 0.0;

const float PITCH_KP = 1.0;
const float PITCH_KI = 0.0;
const float PITCH_KD = 0.0;

const float PITCH_RATE_KP = 1.0;
const float PITCH_RATE_KI = 0.0;
const float PITCH_RATE_KD = 0.0;

const float ROLL_KP = 1.0;
const float ROLL_KI = 0.0;
const float ROLL_KD = 0.0;

const float ROLL_RATE_KP = 1.0;
const float ROLL_RATE_KI = 0.0;
const float ROLL_RATE_KD = 0.0;

void FlightController::begin(
  bool stabilizeMode
) {
  _stabilizeMode = stabilizeMode;

  _altitudeController.initialize(
    ALTITUDE_KP,
    ALTITUDE_KI,
    ALTITUDE_KD,
    MIN_ESC_INPUT_FROM_PID,
    MAX_ESC_INPUT_FROM_PID
  );
  _yawRateController.initialize(
    YAW_RATE_KP,
    YAW_RATE_KI,
    YAW_RATE_KD,
    MIN_ESC_INPUT_FROM_PID,
    MAX_ESC_INPUT_FROM_PID
  );
  _pitchController.initialize(
    PITCH_KP,
    PITCH_KI,
    PITCH_KD,
    MIN_PITCH_RATE_DEG_PER_SEC,
    MAX_PITCH_RATE_DEG_PER_SEC
  );
  _pitchRateController.initialize(
    PITCH_RATE_KP,
    PITCH_RATE_KI,
    PITCH_RATE_KD,
    MIN_ESC_INPUT_FROM_PID,
    MAX_ESC_INPUT_FROM_PID
  );
  _rollController.initialize(
    ROLL_KP,
    ROLL_KI,
    ROLL_KD,
    MIN_ROLL_RATE_DEG_PER_SEC,
    MAX_ROLL_RATE_DEG_PER_SEC
  );
  _rollRateController.initialize(
    ROLL_RATE_KP,
    ROLL_RATE_KI,
    ROLL_RATE_KD,
    MIN_ESC_INPUT_FROM_PID,
    MAX_ESC_INPUT_FROM_PID
  );

  _altitudeController.begin();
  _yawRateController.begin();
  _pitchController.begin();
  _pitchRateController.begin();
  _rollController.begin();
  _rollRateController.begin();
}

FlightControllerResult FlightController::compute(
  StateVector referenceState,
  StateVector measuredState
) {
  FlightControllerResult result;

  result.throttle = _altitudeController.compute(
    referenceState.altitude,
    measuredState.altitude
  );

  result.yaw = _yawRateController.compute(
    referenceState.yawRate,
    measuredState.yawRate
  );

  float referencePitchRate = referenceState.pitchRate;
  if (_stabilizeMode) {
    referencePitchRate = _pitchController.compute(
      referenceState.pitch,
      measuredState.pitch
    );
  }
  result.pitch = _pitchController.compute(
    referencePitchRate,
    measuredState.pitchRate
  );

  float referenceRollRate = referenceState.rollRate;
  if (_stabilizeMode) {
    referenceRollRate = _pitchController.compute(
      referenceState.roll,
      measuredState.roll
    );
  }
  result.roll = _rollController.compute(
    referenceRollRate,
    measuredState.rollRate
  );

  return result;
}
