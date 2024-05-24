#include <Servo.h>

Servo esc;

int escPin = 9;
int escMinPulseWidth = 1000;
int escMaxPulseWidth = 2000;
int escAngleMin = 0;
int escAngleMax = 180;
int escValue;

int potentiometerPin = A0;
int potentiometerValueMin = 0;
int potentiometerValueMax = 1023;
int potentiometerValue;

void setup() {
  Serial.begin(9600);
  esc.attach(
    escPin,
    escMinPulseWidth,
    escMaxPulseWidth
  );
}

void loop() {
  potentiometerValue = analogRead(potentiometerPin);
  Serial.println(potentiometerValue);

  escValue = map(
    potentiometerValue,
    potentiometerValueMin,
    potentiometerValueMax,
    escAngleMin,
    escAngleMax
  );

  esc.write(escValue);
}
