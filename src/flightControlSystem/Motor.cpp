#include "Arduino.h"
#include "Motor.h"
#include <Servo.h>

void Motor::attach(
  int pin,
  int minInput,
  int maxInput
) {
  _motor.attach(
    pin,
    minInput,
    maxInput
  );
  _minInput = minInput;
  _maxInput = maxInput;
}

void Motor::setSpeed(
  int input
) {
  _motor.write(
    constrain(
      input,
      _minInput,
      _maxInput
    )
  );
}
