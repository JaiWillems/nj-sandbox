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

#ifndef Configuration_h
#define Configuration_h

// *** MOTORS ***

const uint8_t MOTOR_ONE_PIN = 10;
const bool MOTOR_ONE_BOW = true;
const bool MOTOR_ONE_PORT = false;

const uint8_t MOTOR_TWO_PIN = 9;
const bool MOTOR_TWO_BOW = false;
const bool MOTOR_TWO_PORT = false;

const uint8_t MOTOR_THREE_PIN = 11;
const bool MOTOR_THREE_BOW = false;
const bool MOTOR_THREE_PORT = true;

const uint8_t MOTOR_FOUR_PIN = 6;
const bool MOTOR_FOUR_BOW = true;
const bool MOTOR_FOUR_PORT = true;

const uint16_t MOTOR_ARM_TIME = 5000;

// *** UART ***

const uint8_t UART_RX_PIN = 3;
const uint8_t UART_TX_PIN = 4;
const uint16_t UART_BAUD_RATE = 9600;
const uint8_t START_MARKER = 255;


#endif
