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

#include "Imu.h"

void Imu::setup() {
  Wire.begin();
  _mpu.initialize();
}

bool Imu::testConnection() {
  return _mpu.testConnection();
}

bool Imu::initializeDmp() {
  uint8_t result = _mpu.dmpInitialize();
  if (result == 0) {
    _mpu.setDMPEnabled(true);
  }
  return result == 0;
}

void Imu::calibrate(uint8_t loops) {
  _mpu.CalibrateAccel(loops);
  _mpu.CalibrateGyro(loops);
}

void Imu::getRotationRates(
  int16_t* x,
  int16_t* y,
  int16_t* z
) {
  _mpu.getRotation(x, y, z);
  (*y) = -(*y);
  (*z) = -(*z);
}

uint8_t Imu::getCurrentFIFOPacket(
  uint8_t* fifoBuffer
) {
  return _mpu.dmpGetCurrentFIFOPacket(
    fifoBuffer
  );
}

void Imu::getYawPitchRollFromDmp(
  uint8_t* fifoBuffer,
  float* yaw,
  float* pitch,
  float* roll
) {
  Quaternion orientation;
  VectorFloat gravity;
  float ypr[3];

  _mpu.dmpGetQuaternion(
    &orientation,
    fifoBuffer
  );
  _mpu.dmpGetGravity(
    &gravity,
    &orientation
  );
  _mpu.dmpGetYawPitchRoll(
    ypr,
    &orientation,
    &gravity
  );

  (*yaw) = ypr[0] * RAD_TO_DEG;
  (*pitch) = ypr[1] * RAD_TO_DEG;
  (*roll) = ypr[2] * RAD_TO_DEG;
}
