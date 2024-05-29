#include "Motor.h"

class Drone {
  public:
    Drone(
      int motorOnePin,
      int motorTwoPin,
      int motorThreePin,
      int motorFourPin,
      int minMotorInput,
      int maxMotorInput
    );
    void sendControlInputs(
      int throttleInput,
      int yawInput,
      int pitchInput,
      int rollInput
    );
  private:
    Motor _motorOne;
    Motor _motorTwo;
    Motor _motorThree;
    Motor _motorFour;
};
