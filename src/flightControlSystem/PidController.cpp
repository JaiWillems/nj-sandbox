#include "PidController.h"

PidController::PidController(
  float kp,
  float ki,
  float kd,
  float minLimit,
  float maxLimit
) {
  _controller.tune(kp, ki, kd);
  _controller.limit(minLimit, maxLimit);
  _controller.minimize(1);
}

void PidController::begin() {
  _controller.begin();
}

float PidController::compute(
  float reference,
  float measured
) {
  _controller.setpoint(reference);
  return _controller.compute(measured);
}
