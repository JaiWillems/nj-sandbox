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
#include "Drone.h"
#include "UartCommunications.h"
#include "Ultrasonic.h"

#include <SoftwareSerial.h>

UartCommunications uartCommunications;
Ultrasonic altimiter;
Drone drone;
FlightInputs flightInputs;

void setup() {
    Serial.begin(9600);
    
    uartCommunications.setup(
        UART_RX_PIN,
        UART_TX_PIN,
        UART_BAUD_RATE
    );

    altimiter.setup(
        ULTRASONIC_TRIG_PIN,
        ULTRASONIC_ECHO_PIN
    );

    drone.setup(
        MOTOR_ONE_PIN,
        MOTOR_TWO_PIN,
        MOTOR_THREE_PIN,
        MOTOR_FOUR_PIN
    );
}

void loop() {
    if (uartCommunications.available()) {
        flightInputs = uartCommunications.read();
    }

    drone.sendFlightInputs(
        flightInputs
    );

    float distance = altimiter.getAbsoluteDistance();
    Serial.println(distance);

    delay(1000 / COMMANDING_FREQUENCY_HZ);
}
