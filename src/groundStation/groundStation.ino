/*
BSD 3-Clause License

Copyright (c) 2025, Nishant Kumar, Jai Willems

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

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(
    CE_PIN,
    CSN_PIN,
    SPI_SPEED
);

void setup() {
    pinMode(THRUST_AXIS_PIN, INPUT);
    pinMode(YAW_AXIS_PIN, INPUT);
    pinMode(PITCH_AXIS_PIN, INPUT);
    pinMode(ROLL_AXIS_PIN, INPUT);

    radio.begin();
    radio.openWritingPipe(WRITE_ADDRESS);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
}

void loop() {
    FlightInputs flightInputs = mapInputs(
        readControlInputs()
    );

    radio.write(
        &flightInputs,
        sizeof(flightInputs)
    );

    delay(1000 / COMMANDING_FREQUENCY_HZ);
}

ControlInputs readControlInputs() {
    return {
        analogRead(THRUST_AXIS_PIN),
        analogRead(YAW_AXIS_PIN),
        analogRead(PITCH_AXIS_PIN),
        analogRead(ROLL_AXIS_PIN)
    };
}

FlightInputs mapInputs(
    ControlInputs controlInputs
) {
    return {
        mapInput(
            controlInputs.throttle,
            MIN_THROTTLE_AUTHORITY,
            MAX_THROTTLE_AUTHORITY
        ),
        mapInput(
            controlInputs.yaw,
            MIN_YAW_AUTHORITY,
            MAX_YAW_AUTHORITY
        ),
        mapInput(
            controlInputs.pitch,
            MIN_PITCH_AUTHORITY,
            MAX_PITCH_AUTHORITY
        ),
        mapInput(
            controlInputs.roll,
            MIN_ROLL_AUTHORITY,
            MAX_ROLL_AUTHORITY
        )
    };
}
