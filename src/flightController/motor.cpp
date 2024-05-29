#include "Arduino.h"
#include "motor.h"
#include <Servo.h>

Motor::Motor(
  int pin,
  int minInput,
  int maxInput,
  bool bow,
  bool port
) {
  _motor.attach(
    pin,
    minInput,
    maxInput
  );
  _minInput = minInput;
  _maxInput = maxInput;
  _bow = bow;
  _port = port;
}

void Motor::setSpeed(
  float thrustContribution,
  float yawContribution,
  float pitchContribution,
  float rollContribution
) {
  float motorInput = mixMotorInputs(
    thrustContribution,
    yawContribution,
    pitchContribution,
    rollContribution
  );
  float validMotorInput = constrain(
    motorInput,
    _minInput,
    _maxInput
  );
  _motor.write(validMotorInput);
}

float Motor::mixMotorInputs(
  float throttle,
  float yaw,
  float pitch,
  float roll
) {
  float motorInput = throttle;
  motorInput = motorInput + (isMotorCcw() ? yaw : -yaw);
  motorInput = motorInput + ((_bow) ? pitch : -pitch);
  motorInput = motorInput + ((_port) ? roll : -roll);
  return motorInput;
}

bool Motor::isMotorCcw() {
  return (_bow && !_port) || (!_bow && _port);
}
