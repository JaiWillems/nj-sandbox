#include <PIDController.h>

class PidController {
  public:
    PidController(
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
    PIDController _controller;
};