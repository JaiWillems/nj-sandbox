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

// *** GENERAL ***

const uint8_t COMMANDING_FREQUENCY_HZ = 75;

// *** CONTROL MAPPING ***

const uint16_t MIN_THROTTLE_AUTHORITY = 0;
const uint16_t MAX_THROTTLE_AUTHORITY = 1000;

const uint8_t YAW_AUTHORITY = 100;
const int16_t MIN_YAW_AUTHORITY = -YAW_AUTHORITY;
const int16_t MAX_YAW_AUTHORITY = YAW_AUTHORITY;

const uint8_t PITCH_AUTHORITY = 100;
const int16_t MIN_PITCH_AUTHORITY = PITCH_AUTHORITY;
const int16_t MAX_PITCH_AUTHORITY = -PITCH_AUTHORITY;

const uint8_t ROLL_AUTHORITY = 100;
const int16_t MIN_ROLL_AUTHORITY = -ROLL_AUTHORITY;
const int16_t MAX_ROLL_AUTHORITY = ROLL_AUTHORITY;

#endif