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
#include "Lights.h"
#include "Receiver.h"
#include "UartCommunications.h"

Lights lights;
Receiver receiver;
UartCommunications uartCommunications;

void setup() {
    lights.setup(
        NAV_LIGHT_ONE_PIN,
        NAV_LIGHT_TWO_PIN,
        NAV_LIGHT_THREE_PIN,
        NAV_LIGHT_FOUR_PIN
    );

    receiver.setup(
        CE_PIN,
        CSN_PIN,
        SPI_SPEED,
        READ_ADDRESS
    );

    uartCommunications.setup(
        UART_RX_PIN,
        UART_TX_PIN,
        UART_BAUD_RATE
    );
}

void loop() {
    lights.blinkingRefresh();

    if (receiver.available()) {
        uartCommunications.write(
            receiver.read()
        );
    }

    delay(1000 / COMMANDING_FREQUENCY_HZ);
}
