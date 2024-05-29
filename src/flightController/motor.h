#include <Arduino.h>
#include <Servo.h>

class Motor {
  public:
    void attach(
      int pin,
      int minInput,
      int maxInput
    );
    void setSpeed(
      int input
    );
  private:
    Servo _motor;
    int _minInput;
    int _maxInput;
};
