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

#include "Configuration.h"
#include "Settings.h"
#include "Types.h"
#include "Utils.h"
#include "Transmitter.h"

Transmitter transmitter;
ControlSignals averageSignals;

void setup() {
    pinMode(THRUST_AXIS_PIN, INPUT);
    pinMode(YAW_AXIS_PIN, INPUT);
    pinMode(PITCH_AXIS_PIN, INPUT);
    pinMode(ROLL_AXIS_PIN, INPUT);

    averageSignals = calculateAverageSignals();

    Serial.begin(9600);
    Serial.print(averageSignals.throttle);
    Serial.print("\t");
    Serial.print(averageSignals.yaw);
    Serial.print("\t");
    Serial.print(averageSignals.pitch);
    Serial.print("\t");
    Serial.println(averageSignals.roll);

    transmitter.setup(
        CE_PIN,
        CSN_PIN,
        SPI_SPEED,
        WRITE_ADDRESS
    );
}

void loop() {
    FlightInputs flightInputs = cubicMapInputs(
        calibrateSignals(
            readControlSignals(),
            averageSignals
        )
    );

    transmitter.write(
        flightInputs
    );

    delay(1000 / COMMANDING_FREQUENCY_HZ);
}

ControlSignals readControlSignals() {
    return {
        analogRead(THRUST_AXIS_PIN),
        analogRead(YAW_AXIS_PIN),
        analogRead(PITCH_AXIS_PIN),
        analogRead(ROLL_AXIS_PIN)
    };
}

ControlSignals calculateAverageSignals() {
    uint16_t throttle = 0;
    uint16_t yaw = 0;
    uint16_t pitch = 0;
    uint16_t roll = 0;

    for (int i = 0; i < CALIBRATION_ITERATIONS; i ++) {
        ControlSignals signals = readControlSignals();

        throttle += signals.throttle;
        yaw += signals.yaw;
        pitch += signals.pitch;
        roll += signals.roll;
    }
    
    return {
        throttle / CALIBRATION_ITERATIONS,
        yaw / CALIBRATION_ITERATIONS,
        pitch / CALIBRATION_ITERATIONS,
        roll / CALIBRATION_ITERATIONS
    };
}

ControlSignals calibrateSignals(
    ControlSignals rawSignals,
    ControlSignals averageSignals
) {
    // TODO: Implement.
    return rawSignals;
}

FlightInputs cubicMapInputs(
    ControlSignals controlSignals
) {
    return {
        cubicMapInput(
            controlSignals.throttle,
            MAX_Z_DOT,
            MIN_Z_DOT
        ),
        cubicMapInput(
            controlSignals.yaw,
            MIN_YAW_RATE,
            MAX_YAW_RATE
        ),
        cubicMapInput(
            controlSignals.pitch,
            MIN_PITCH,
            MAX_PITCH
        ),
        cubicMapInput(
            controlSignals.roll,
            MAX_ROLL,
            MIN_ROLL
        )
    };
}
