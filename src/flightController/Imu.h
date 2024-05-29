#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

class Imu {
  public:
    // Imu();
    void initialize();
    bool testConnection();
    uint8_t initializeDmp();
    void calibrate(uint8_t loops);
    void getRotationRates(
      uint16_t* x,
      uint16_t* y,
      uint16_t* z
    );
    uint8_t getCurrentFIFOPacket(
      uint8_t* fifoBuffer
    );
    void getYawPitchRollFromDmp(
      uint8_t fifoBuffer,
      float* yaw,
      float* pitch,
      float* roll
    );
  private:
    MPU6050 _mpu;
};