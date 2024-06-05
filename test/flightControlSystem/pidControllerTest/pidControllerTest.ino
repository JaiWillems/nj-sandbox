/*
BSD 3-Clause License

Copyright (c) 2024, Nishant Kumar, Jai Willems

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <util/atomic.h>
#include "PidController.h"

const int ENC_A_PIN = 3;
const int ENC_B_PIN = 2;
const int MOTOR_PWM_PIN = 5;
const int IN1_PIN = 8;
const int IN2_PIN = 7;

const float KP = 1.0;
const float KI = 0.0;
const float KD = 0.0;

const int MIN_LIMIT = 0;
const int MAX_LIMIT = 1;

PidController pid;

volatile int measuredCounter = 0;
long previousTime = 0;

void setup() {

  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A_PIN),
    readEncoder,
    RISING
  );

  pid.initialize(
    KP,
    KI,
    KD,
    MIN_LIMIT,
    MAX_LIMIT
  );
  pid.begin();
}

void loop() {

  int measured = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    measured = measuredCounter;
  }

  int reference = 250 * sin(previousTime / 1e6);
  setMotor(
    pid.compute(reference, measured),
    MOTOR_PWM_PIN,
    IN1_PIN,
    IN2_PIN
  );

  Serial.print(reference);
  Serial.print(" ");
  Serial.print(measured);
  Serial.println();

  previousTime = micros();
}

void setMotor(
    int pwmVal,
    int pwmPin,
    int in1Pin,
    int in2Pin
) {
  analogWrite(pwmPin, fabs(pwmVal));
  if (pwmVal > 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  else {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENC_B_PIN);
  if (b > 0) {
    measuredCounter++;
  }
  else {
    measuredCounter--;
  }
}
