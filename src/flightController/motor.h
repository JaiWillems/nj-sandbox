#include <Arduino.h>
#include <Servo.h>

class Motor {
  public:
    Motor(
      int pin,
      int minInput,
      int maxInput,
      bool bow,
      bool port
    );
    void setSpeed(
      float thrustContribution,
      float yawContribution,
      float pitchContribution,
      float rollContribution
    );
  private:
    Servo _motor;
    int _minInput;
    int _maxInput;
    bool _bow;
    bool _port;
    float mixMotorInputs(
      float throttle,
      float yaw,
      float pitch,
      float roll
    );
};
