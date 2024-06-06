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

#include "PidController.h"

void PidController::initialize(
  float kp,
  float ki,
  float kd,
  float minLimit,
  float maxLimit
) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _minLimit = minLimit;
  _maxLimit = maxLimit;
}

void PidController::begin() {
  _previousTime = millis();
}

float PidController::compute(
  float reference,
  float measured
) {
  float deltaTime = getDeltaTime();

  float currentError = reference - measured;
  _integralError = _integralError +
    currentError * deltaTime;
  float derivativeError = (currentError -
    _previousError) / deltaTime;

  _previousError = currentError;

  return getInput(
    currentError,
    _integralError,
    derivativeError
  );
}

float PidController::getInput(
  float error,
  float integralError,
  float derivativeError
) {
  return saturate(
    _kp * error +
    _ki * integralError +
    _kd * derivativeError
  );
}

float PidController::saturate(
  float input
) {
  return constrain(
    input,
    _minLimit,
    _maxLimit
  );
}

float PidController::compute(
  float reference,
  float measured,
  float measuredDerivative
) {
  float deltaTime = getDeltaTime();

  float currentError = reference - measured;
  _integralError = _integralError +
    currentError * deltaTime;

  _previousError = currentError;

  return getInput(
    currentError,
    _integralError,
    measuredDerivative
  );
}

float PidController::getDeltaTime() {
  float currentTime = millis();
  float deltaTime = (currentTime - _previousTime) / 1000;
  _previousTime = currentTime;
  return deltaTime;
}
