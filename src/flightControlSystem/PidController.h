#include <Arduino.h>

class PidController {
  public:
    void initialize(
      float kp,
      float ki,
      float kd,
      float minLimit,
      float maxLimit
    );
    void begin();
    float compute(
      float reference,
      float measured
    );
  private:
    float _kp;
    float _ki;
    float _kd;
    float _minLimit;
    float _maxLimit;
    float _previousTime;
    float _previousError;
    float _integralError;
};
