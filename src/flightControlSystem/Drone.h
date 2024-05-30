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
    int _minMotorInput;
    int _maxMotorInput;
    int mixControlInputs(
      bool bow,
      bool port,
      float throttleInput,
      float yawInput,
      float pitchInput,
      float rollInput
    );
    bool isMotorCcw(
      bool bow,
      bool port
    );
};
