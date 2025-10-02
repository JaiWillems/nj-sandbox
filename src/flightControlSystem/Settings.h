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

#ifndef Settings_h
#define Settings_h

// *** MOTOR ***

const uint16_t MOTOR_INPUT_TO_HOVER = 450;

// *** CONTROL MAPPING ***

const uint16_t MIN_THROTTLE_STICK_POSITION = 0;
const uint16_t MAX_THROTTLE_STICK_POSITION = 1023;
const float MIN_ALTITUDE_RATE_M_PER_SEC = -0.5;
const float MAX_ALTITUDE_RATE_M_PER_SEC = 0.5;

const uint16_t MIN_YAW_STICK_POSITION = 0;
const uint16_t MAX_YAW_STICK_POSITION = 1023;
const int16_t MIN_YAW_RATE_DEG_PER_SEC = -180;
const int16_t MAX_YAW_RATE_DEG_PER_SEC = 180;

const uint16_t MIN_PITCH_STICK_POSITION = 1023;
const uint16_t MAX_PITCH_STICK_POSITION = 0;
const int8_t MIN_PITCH_DEG = -60;
const int8_t MAX_PITCH_DEG = 60;

const uint16_t MIN_ROLL_STICK_POSITION = 0;
const uint16_t MAX_ROLL_STICK_POSITION = 1023;
const int8_t MIN_ROLL_DEG = -60;
const int8_t MAX_ROLL_DEG = 60;

// *** SENSOR ***

const uint8_t IMU_CALIBRATION_LOOPS = 6;
const uint16_t UART_BAUD_RATE = 9600;

// *** PID ***

const uint16_t MIN_THROTTLE_PID_INPUT = -500;
const uint16_t MAX_THROTTLE_PID_INPUT = 500;
const uint8_t MIN_MOTOR_PID_INPUT = -50;
const uint8_t MAX_MOTOR_PID_INPUT = 50;

const float TAKEOFF_GAIN = 0.1;
const float LANDING_GAIN = 0.1;

const float ALTITUDE_KP = 25.0;
const float ALTITUDE_KI = 0.0;
const float ALTITUDE_KD = 0.0;

const float YAW_KP = 1.0;
const float YAW_KI = 0.0;
const float YAW_KD = 0.0;

const float PITCH_KP = 1.0;
const float PITCH_KI = 0.0;
const float PITCH_KD = 0.0;

const float ROLL_KP = 1.0;
const float ROLL_KI = 0.0;
const float ROLL_KD = 0.0;

#endif