/*
BSD 3-Clause License

Copyright (c) 2024, Nishant Kumar, Jai Willems

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

const int MIN_MOTOR_INPUT = 1000;
const int MAX_MOTOR_INPUT = 2000;

// *** CONTROL MAPPING ***

const int MIN_THROTTLE_STICK_POSITION = 0;
const int MAX_THROTTLE_STICK_POSITION = 1023;
const float MIN_ALTITUDE_RATE_M_PER_SEC = -0.5;
const float MAX_ALTITUDE_RATE_M_PER_SEC = 0.5;

const int MIN_YAW_STICK_POSITION = 0;
const int MAX_YAW_STICK_POSITION = 1023;
const int MIN_YAW_RATE_DEG_PER_SEC = -180;
const int MAX_YAW_RATE_DEG_PER_SEC = 180;

const int MIN_PITCH_STICK_POSITION = 1023;
const int MAX_PITCH_STICK_POSITION = 0;
const int MIN_PITCH_ANGLE_DEG = -60;
const int MAX_PITCH_ANGLE_DEG = 60;
const int MIN_PITCH_RATE_DEG_PER_SEC = -180;
const int MAX_PITCH_RATE_DEG_PER_SEC = 180;

const int MIN_ROLL_STICK_POSITION = 0;
const int MAX_ROLL_STICK_POSITION = 1023;
const int MIN_ROLL_ANGLE_DEG = -60;
const int MAX_ROLL_ANGLE_DEG = 60;
const int MIN_ROLL_RATE_DEG_PER_SEC = -180;
const int MAX_ROLL_RATE_DEG_PER_SEC = 180;

// *** SENSOR ***

const int IMU_CALIBRATION_LOOPS = 6;
const int UART_BAUD_RATE = 9600;

// *** PID ***

const int MIN_ESC_INPUT_FROM_PID = -50;
const int MAX_ESC_INPUT_FROM_PID = 50;

const float ALTITUDE_KP = 25.0;
const float ALTITUDE_KI = 0.0;
const float ALTITUDE_KD = 0.0;

const float YAW_RATE_KP = 1.0;
const float YAW_RATE_KI = 0.0;
const float YAW_RATE_KD = 0.0;

const float PITCH_KP = 1.0;
const float PITCH_KI = 0.0;
const float PITCH_KD = 0.0;

const float PITCH_RATE_KP = 1.0;
const float PITCH_RATE_KI = 0.0;
const float PITCH_RATE_KD = 0.0;

const float ROLL_KP = 1.0;
const float ROLL_KI = 0.0;
const float ROLL_KD = 0.0;

const float ROLL_RATE_KP = 1.0;
const float ROLL_RATE_KI = 0.0;
const float ROLL_RATE_KD = 0.0;

#endif