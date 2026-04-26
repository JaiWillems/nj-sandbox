/*
BSD 3-Clause License

Copyright (c) 2026, Nishant Kumar, Jai Willems

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

#include "Drone.h"

void Drone::setup(
    uint8_t motorOnePin,
    uint8_t motorTwoPin,
    uint8_t motorThreePin,
    uint8_t motorFourPin
) {
    _motorOne.attach(
        motorOnePin
    );
    _motorTwo.attach(
        motorTwoPin
    );
    _motorThree.attach(
        motorThreePin
    );
    _motorFour.attach(
        motorFourPin
    );
}

void Drone::arm() {
    _motorOne.arm();
    _motorTwo.arm();
    _motorThree.arm();
    _motorFour.arm();
  
    delay(MOTOR_ARM_TIME);
}

void Drone::sendFlightInputs(
    FlightInputs flightInputs
) {
    const float a1 = flightInputs.U1 / (4 * KF);
    const float a2 = flightInputs.U2 / (2 * sqrt(2) * ARM_LENGTH * KF);
    const float a3 = flightInputs.U3 / (2 * sqrt(2) * ARM_LENGTH * KF);
    const float a4 = flightInputs.U4 / (4 * KM);
    
    _motorOne.setSpeed(
        a1 - a2 - a4 - a4
    );
    _motorTwo.setSpeed(
        a1 - a2 + a3 + a4
    );
    _motorThree.setSpeed(
        a1 + a2 + a3 - a4
    );
    _motorFour.setSpeed(
        a1 + a2 - a3 + a4
    );
}
