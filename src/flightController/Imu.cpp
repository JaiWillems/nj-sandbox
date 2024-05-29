#include "Imu.h"

void Imu::initialize() {
  Wire.begin();
  _mpu.initialize();
}

bool Imu::testConnection() {
  return _mpu.testConnection();
}

uint8_t Imu::initializeDmp() {
  uint8_t result = _mpu.dmpInitialize();
  if (result == 0) {
    _mpu.setDMPEnabled(true);
  }
  return result;
}

void Imu::calibrate(uint8_t loops) {
  _mpu.CalibrateAccel(loops);
  _mpu.CalibrateGyro(loops);
}

void Imu::getRotationRates(
  uint16_t* x,
  uint16_t* y,
  uint16_t* z
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
  uint8_t fifoBuffer,
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
