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

#include "Transmitter.h"

void Transmitter::setup(
    uint8_t cePin,
    uint8_t csnPin,
    uint8_t spiSpeed,
    byte writeAddress[6]
) {
    _transmitter = new RF24(
        cePin,
        csnPin,
        spiSpeed
    );
    _transmitter->begin();
    _transmitter->openWritingPipe(writeAddress);
    _transmitter->setPALevel(RF24_PA_MIN);
    _transmitter->stopListening();
}

void Transmitter::write(
    FlightInputs flightInputs,
    bool droneState
) {
    struct DataPackage {
        int8_t throttle;
        int8_t yaw;
        int8_t pitch;
        int8_t roll;
        int8_t droneState;
    };

    DataPackage dataPackage = {
        100 * flightInputs.throttle,
        100 * flightInputs.yaw,
        100 * flightInputs.pitch,
        100 * flightInputs.roll,
        droneState
    };

    _transmitter->write(
        &dataPackage,
        sizeof(dataPackage)
    );
}