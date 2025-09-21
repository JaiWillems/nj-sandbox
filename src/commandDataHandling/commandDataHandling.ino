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

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>

const int COMMANDING_FREQUENCY_HZ = 75;

struct ControlCommands {
  float throttle;
  float yaw;
  float pitch;
  float roll;
};

ControlCommands controlCommands;

const int CE_PIN = 5;
const int CSN_PIN = 6;
const uint32_t SPI_SPEED = 4000000;
const byte readAddress[6] = "00001";

RF24 radio(
  CE_PIN,
  CSN_PIN,
  SPI_SPEED
);

const int RX_PIN = 9;
const int TX_PIN = 10;
const int UART_BAUD_RATE = 9600;
const byte START_MARKER = 255;

SoftwareSerial uartCommunications(
  RX_PIN,
  TX_PIN
);

void setup() {
  radio.begin();
  radio.openReadingPipe(0, readAddress);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  uartCommunications.begin(UART_BAUD_RATE);
}

void loop() {
  if (radio.available()) {
    radio.read(
      &controlCommands,
      sizeof(controlCommands)
    );
  }

  uartCommunications.write(START_MARKER);
  uartCommunications.write(
    (char*)&controlCommands,
    sizeof(controlCommands)
  );

  delay(1000 / COMMANDING_FREQUENCY_HZ);
}
