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

#include "FlightController.h"

void FlightController::begin(){
    _altitudeRateController.initialize(
        ALTITUDE_RATE_KP,
        ALTITUDE_RATE_KI,
        ALTITUDE_RATE_KD,
        MIN_MOTOR_INPUTS,
        MAX_MOTOR_INPUTS
    );
    _yawRateController.initialize(
        YAW_RATE_KP,
        YAW_RATE_KI,
        YAW_RATE_KD,
        MIN_MOTOR_INPUTS,
        MAX_MOTOR_INPUTS
    );
    _pitchController.initialize(
        PITCH_KP,
        PITCH_KI,
        PITCH_KD,
        MIN_MOTOR_INPUTS,
        MAX_MOTOR_INPUTS
    );
    _rollController.initialize(
        ROLL_KP,
        ROLL_KI,
        ROLL_KD,
        MIN_MOTOR_INPUTS,
        MAX_MOTOR_INPUTS
    );

    _altitudeRateController.begin();
    _yawRateController.begin();
    _pitchController.begin();
    _rollController.begin();
}

FlightInputs FlightController::compute(
    UserInputs userInputs,
    StateEstimation state
){
    FlightInputs flightInputs;
    flightInputs.U1 = _altitudeRateController.compute(
        userInputs.altitudeRate,
        state.altitudeRate
    ) + GRAVITATIONAL_ACCELERATION * DRONE_MASS;
    flightInputs.U2 = _rollController.compute(
        userInputs.roll,
        state.roll,
        state.rollRate
    );
    flightInputs.U3 = _pitchController.compute(
        userInputs.pitch,
        state.pitch,
        state.pitchRate
    );
    flightInputs.U4 = _yawRateController.compute(
        userInputs.yawRate,
        state.yawRate
    );

    return flightInputs;
}
