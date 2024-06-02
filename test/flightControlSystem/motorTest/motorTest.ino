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

#include "Motor.h"

const int MOTOR_ONE_PIN = 10;
const int MOTOR_TWO_PIN = 9;
const int MOTOR_THREE_PIN = 11;
const int MOTOR_FOUR_PIN = 6;
const int MIN_INPUT = 1000;
const int MAX_INPUT = 2000;

const int ATTACH_DELAY = 1500;
const int ARM_DELAY = 5000;
const int SMALL_DELAY = 2000;

const int MOTOR_SPEED = 20;

Motor motorOne;
Motor motorTwo;
Motor motorThree;
Motor motorFour;

void setup() {

  delay(ATTACH_DELAY);

  motorOne.attach(
    MOTOR_ONE_PIN,
    MIN_INPUT,
    MAX_INPUT
  );
  motorTwo.attach(
    MOTOR_TWO_PIN,
    MIN_INPUT,
    MAX_INPUT
  );
  motorThree.attach(
    MOTOR_THREE_PIN,
    MIN_INPUT,
    MAX_INPUT
  );
  motorFour.attach(
    MOTOR_FOUR_PIN,
    MIN_INPUT,
    MAX_INPUT
  );

  delay(ARM_DELAY);

  motorOne.arm();
  motorTwo.arm();
  motorThree.arm();
  motorFour.arm();
}

void loop() {
  delay(SMALL_DELAY);
  motorOne.setSpeed(MOTOR_SPEED);
  delay(SMALL_DELAY);
  motorTwo.setSpeed(MOTOR_SPEED);
  delay(SMALL_DELAY);
  motorThree.setSpeed(MOTOR_SPEED);
  delay(SMALL_DELAY);
  motorFour.setSpeed(MOTOR_SPEED);
}
