#include <Servo.h>

Servo ESC;

int escPin = 9;
int escMinPulseWidth = 1000;
int escMaxPulseWidth = 2000;
int escAngleMin = 0;
int escAngleMax = 180;

int potentiometerPin = A0;
int potentiometerValueMin = 0;
int potentiometerValueMax = 1023;
int potentiometerValue;

void setup() {
  ESC.attach(
    escPin,
    escMinPulseWidth,
    escMaxPulseWidth
  );
}

void loop() {
  potentiometerValue = analogRead(potentiometerPin);
  potentiometerValue = map(
    potentiometerValue,
    potentiometerValueMin,
    potentiometerValueMax,
    escAngleMin,
    escAngleMax
  );
  ESC.write(potentiometerValue);
}
