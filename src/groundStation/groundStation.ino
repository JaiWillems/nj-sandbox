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

bool droneState = false; // On is True, off is False.
uint8_t buttonHistory = 0b00000000;
uint8_t stateSwitchMask = 0b01111111;

void setup() {
    pinMode(THRUST_AXIS_PIN, INPUT);
    pinMode(YAW_AXIS_PIN, INPUT);
    pinMode(PITCH_AXIS_PIN, INPUT);
    pinMode(ROLL_AXIS_PIN, INPUT);

    pinMode(SWITCH_LEFT_PIN, INPUT_PULLUP);
    pinMode(SWITCH_RIGHT_PIN, INPUT_PULLUP);
    
    averageSignals = calculateAverageSignals();

    transmitter.setup(
        CE_PIN,
        CSN_PIN,
        SPI_SPEED,
        WRITE_ADDRESS
    );
}

void loop() {
    if (simultaneousJoystickPress()) {
        droneState = !droneState;
    }

    UserInputs userInputs = cubicMapInputs(
        calibrateSignals(
            readControlSignals(),
            averageSignals
        )
    );

    transmitter.write(
        userInputs,
        droneState
    );

    delay(1000 / COMMANDING_FREQUENCY_HZ);
}

bool simultaneousJoystickPress() {
    uint8_t leftSwitchState = digitalRead(SWITCH_LEFT_PIN);
    uint8_t rightSwitchState = digitalRead(SWITCH_RIGHT_PIN);
    uint8_t bothSwitchesPressed = !(leftSwitchState || rightSwitchState);
    
    buttonHistory = buttonHistory << 1;
    buttonHistory = buttonHistory | bothSwitchesPressed;
    
    return !(buttonHistory ^ stateSwitchMask);
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
    uint16_t lx = 0;
    uint16_t ly = 0;
    uint16_t rx = 0;
    uint16_t ry = 0;

    for (int i = 0; i < CALIBRATION_ITERATIONS; i ++) {
        ControlSignals signals = readControlSignals();

        lx += signals.LX;
        ly += signals.LY;
        rx += signals.RX;
        ry += signals.RY;
    }
    
    return {
        lx / CALIBRATION_ITERATIONS,
        ly / CALIBRATION_ITERATIONS,
        rx / CALIBRATION_ITERATIONS,
        ry / CALIBRATION_ITERATIONS
    };
}

ControlSignals calibrateSignals(
    ControlSignals rawSignals,
    ControlSignals averageSignals
) {
    return {
        calibrateSignal(
            rawSignals.LX,
            averageSignals.LX
        ),
        calibrateSignal(
            rawSignals.LY,
            averageSignals.LY
        ),
        calibrateSignal(
            rawSignals.RX,
            averageSignals.RX
        ),
        calibrateSignal(
            rawSignals.RY,
            averageSignals.RY
        )
    };
}

int16_t calibrateSignal(
    int16_t rawSignal,
    int16_t averageSignal
) {
    if (rawSignal > averageSignal) {
        return linearMap(
            rawSignal,
            averageSignal,
            MAX_CONTROL_INPUT,
            MID_CONTROL_INPUT,
            MAX_CONTROL_INPUT
        );
    } else {
        return linearMap(
            rawSignal,
            MIN_CONTROL_INPUT,
            averageSignal,
            MIN_CONTROL_INPUT,
            MID_CONTROL_INPUT
        );
    }
}

int16_t linearMap(
    int16_t value,
    int16_t minInputValue,
    int16_t maxInputValue,
    int16_t minOutputValue,
    int16_t maxOutputValue
) {
    float slope = (float) (minOutputValue - maxOutputValue) / (minInputValue - maxInputValue);
    return slope * (value - minInputValue) + minOutputValue;
}


UserInputs cubicMapInputs(
    ControlSignals controlSignals
) {
    return {
        cubicMapInput(
            controlSignals.LX,
            MAX_Z_DOT,
            MIN_Z_DOT
        ),
        cubicMapInput(
            controlSignals.LY,
            MIN_YAW_RATE,
            MAX_YAW_RATE
        ),
        cubicMapInput(
            controlSignals.RX,
            MAX_PITCH,
            MIN_PITCH
        ),
        cubicMapInput(
            controlSignals.RY,
            MAX_ROLL,
            MIN_ROLL
        )
    };
}
