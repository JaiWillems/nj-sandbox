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
  float currentTime = millis();
  float deltaTime = (currentTime - _previousTime) / 1000;

  float currentError = reference - measured;
  _integralError = _integralError + currentError * deltaTime;
  float derivativeError = (currentError - _previousError) / deltaTime;
  
  _previousTime = currentTime;
  _previousError = currentError;

  return constrain(
    _kp * currentError + _ki * _integralError + _kd * derivativeError,
    _minLimit,
    _maxLimit
  );
}
