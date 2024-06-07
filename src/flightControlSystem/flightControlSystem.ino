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

#include "Settings.h"
#include "Types.h"
#include "Drone.h"
#include "Ultrasonic.h"
#include "Imu.h"
#include "UartCommunications.h"
#include "FlightController.h"

// *** PINS ***

const int MOTOR_ONE_PIN = 10;
const int MOTOR_TWO_PIN = 9;
const int MOTOR_THREE_PIN = 11;
const int MOTOR_FOUR_PIN = 6;

const int ULTRASONIC_TRIG_PIN = 12;
const int ULTRASONIC_ECHO_PIN = 13;

const int UART_RX_PIN = 3;
const int UART_TX_PIN = 4;

// *** INITIALIZATIONS ***

Drone drone;
Ultrasonic ultrasonic;
Imu imu;
UartCommunications uartCommunications;
FlightController flightController;

ControlCommands controlCommands;

// *** OTHER ***

float previousAltitude;
int previousTime;

void setup() {

  drone.setup(
    MOTOR_ONE_PIN,
    MOTOR_TWO_PIN,
    MOTOR_THREE_PIN,
    MOTOR_FOUR_PIN,
    MIN_MOTOR_INPUT,
    MAX_MOTOR_INPUT
  );
  drone.arm();

  flightController.setup();

  ultrasonic.setup(
    ULTRASONIC_TRIG_PIN,
    ULTRASONIC_ECHO_PIN
  );
  ultrasonic.calibrate();

  imu.setup();
  if (!imu.initializeDmp()) {
    flightController.flightModeFSM.setMode(
      ERROR
    );
  }
  imu.calibrate(IMU_CALIBRATION_LOOPS);

  uartCommunications.setup(
    UART_RX_PIN,
    UART_TX_PIN,
    UART_BAUD_RATE
  );
}

void loop() {
  if (!flightController.flightModeFSM.isMode(ERROR)) {
    if (uartCommunications.isPacketAvailable()) {
      controlCommands = uartCommunications.deserialize();
    }

    StateVector referenceState = getReferenceState();
    StateVector measuredState = getMeasuredState();

    drone.sendControlInputs(
      flightController.compute(
        referenceState,
        measuredState
      )
    );
  }
}

StateVector getReferenceState() {
  StateVector referenceState;
  float referenceAltitudeRate = intToFloatRangeMap(
    controlCommands.throttle,
    MIN_THROTTLE_STICK_POSITION,
    MAX_THROTTLE_STICK_POSITION,
    MIN_ALTITUDE_RATE_M_PER_SEC,
    MAX_ALTITUDE_RATE_M_PER_SEC
  );
  referenceState.altitude = previousAltitude +
    getDeltaTime() * referenceAltitudeRate;
  previousAltitude = referenceState.altitude;
  referenceState.yawRate = map(
    controlCommands.yaw,
    MIN_YAW_STICK_POSITION,
    MAX_YAW_STICK_POSITION,
    MIN_YAW_RATE_DEG_PER_SEC,
    MAX_YAW_RATE_DEG_PER_SEC
  );
  referenceState.pitch = map(
    controlCommands.pitch,
    MIN_PITCH_STICK_POSITION,
    MAX_PITCH_STICK_POSITION,
    MIN_PITCH_DEG,
    MAX_PITCH_DEG
  );
  referenceState.roll = map(
    controlCommands.roll,
    MIN_ROLL_STICK_POSITION,
    MAX_ROLL_STICK_POSITION,
    MIN_ROLL_DEG,
    MAX_ROLL_DEG
  );
  return referenceState;
}

float intToFloatRangeMap(
  int value,
  int minIn,
  int maxIn,
  float minOut,
  float maxOut
) {
  return (maxOut - minOut) * value / (maxIn - minIn) + minOut;
}

float getDeltaTime() {
  int currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000;
  previousTime = currentTime;
  return deltaTime;
}

StateVector getMeasuredState() {
  StateVector measuredState;
  measuredState.altitude = ultrasonic.getRelativeDistance();
  imu.getRotationRates(
    &measuredState.rollRate,
    &measuredState.pitchRate,
    &measuredState.yawRate
  );
  uint8_t fifoBuffer[64];
  if (imu.getCurrentFIFOPacket(fifoBuffer)) {
    imu.getYawPitchRollFromDmp(
      fifoBuffer,
      &measuredState.yaw,
      &measuredState.pitch,
      &measuredState.roll
    );
  }
  return measuredState;
}
