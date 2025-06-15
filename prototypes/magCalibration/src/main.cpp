#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define ACCEL_CONF 0x1C
#define GYRO_CONF 0x1B
#define INT_PIN_CFG 0x37
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

#define QMC5883_ADDR 0x0D
#define MAG_XOUT_L 0x00
#define MAG_YOUT_L 0x02
#define MAG_ZOUT_L 0x04
#define SET_MODE 0x09
#define PERIOD_REG 0x0B

#define CALIBRATION_SAMPLES 1000
float accelXOffset = 0;
float accelYOffset = 0;
float accelZOffset = 0;
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

float offsetMagX = 0;
float offsetMagY = 0; 
float offsetMagZ = 0;
float scaleMagX = 0;
float scaleMagY = 0;
float scaleMagZ = 0;
float yawReference = 0;

const int numberReadings = 100;
float magXSum[numberReadings];
float magYSum[numberReadings];
float yawSum[numberReadings];
int index = 0;

float magneticDeclination = 10.0;

void IMUsetup();
void IMUconfig();
void IMUcalibrate();
int16_t readValues(uint8_t addr, uint8_t start, int type);
void setI2CBypassEnabled();
void magnetCalibrate();
float calculateMovingAverage(float sum[], int sumLength, float sensorReading, int &index);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  IMUsetup();
  IMUconfig();
  IMUcalibrate();
  magnetCalibrate();
  yawReference += magneticDeclination;
}

void loop() {
  float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
  float magX, magY, magZ;

  accelX = (readValues(MPU_ADDR, ACCEL_XOUT_H, 0) - accelXOffset) / 16384.0;
  accelY = (readValues(MPU_ADDR, ACCEL_YOUT_H, 0) - accelYOffset) / 16384.0;
  accelZ = (readValues(MPU_ADDR, ACCEL_ZOUT_H, 0) - accelZOffset) / 16384.0;

  gyroX = (readValues(MPU_ADDR, GYRO_XOUT_H, 0) - gyroXOffset) / 131.0;
  gyroY = (readValues(MPU_ADDR, GYRO_YOUT_H, 0) - gyroYOffset) / 131.0;
  gyroZ = (readValues(MPU_ADDR, GYRO_ZOUT_H, 0) - gyroZOffset) / 131.0;

  magX = (readValues(QMC5883_ADDR, MAG_XOUT_L, 1) - offsetMagX) * scaleMagX;
  magY = (readValues(QMC5883_ADDR, MAG_YOUT_L, 1) - offsetMagY) * scaleMagY;
  magZ = (readValues(QMC5883_ADDR, MAG_ZOUT_L, 1) - offsetMagZ) * scaleMagZ;

  calculateMovingAverage(magXSum, numberReadings, magX, index);
  calculateMovingAverage(magYSum, numberReadings, magY, index);

  float magYaw = atan2(magY, magX) * 180.0 / PI;
  
  if (magYaw < 0) {
    magYaw += 360;
  } else if (magYaw > 360) {
    magYaw -= 360;
  }

  Serial.println(magYaw);
  delay(100);
}


void IMUsetup() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);

  setI2CBypassEnabled();
}

void IMUconfig() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONF);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONF);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x03);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(SMPLRT_DIV);
  Wire.write(0x09);
  Wire.endTransmission(true);

  setI2CBypassEnabled();

  Wire.beginTransmission(QMC5883_ADDR);
  Wire.write(PERIOD_REG);
  Wire.write(0x01);
  Wire.endTransmission(true);

  Wire.beginTransmission(QMC5883_ADDR);
  Wire.write(SET_MODE);
  Wire.write(0x19);
  Wire.endTransmission(true);
}

int16_t readValues(uint8_t addr, uint8_t start, int type) {
  Wire.beginTransmission(addr);
  Wire.write(start);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 2, true);
  uint8_t highByte, lowByte;
  if (type == 0) {
    highByte = Wire.read();
    lowByte = Wire.read();
  } else {
    lowByte = Wire.read();
    highByte = Wire.read();
  }
  int16_t value = (highByte << 8) | lowByte;
  return value;
}

void setI2CBypassEnabled() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02);
  Wire.endTransmission(true);
}

void IMUcalibrate() {
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t ax = readValues(MPU_ADDR, ACCEL_XOUT_H, 0);
    int16_t ay = readValues(MPU_ADDR, ACCEL_YOUT_H, 0);
    int16_t az = readValues(MPU_ADDR, ACCEL_ZOUT_H, 0);
    int16_t gx = readValues(MPU_ADDR, GYRO_XOUT_H, 0);
    int16_t gy = readValues(MPU_ADDR, GYRO_YOUT_H, 0);
    int16_t gz = readValues(MPU_ADDR, GYRO_ZOUT_H, 0);

    accelXOffset += ax;
    accelYOffset += ay;
    accelZOffset += az;
    gyroXOffset += gx;
    gyroYOffset += gy;
    gyroZOffset += gz;
  }

  accelXOffset /= CALIBRATION_SAMPLES;
  accelYOffset /= CALIBRATION_SAMPLES;
  accelZOffset /= CALIBRATION_SAMPLES;
  gyroXOffset /= CALIBRATION_SAMPLES;
  gyroYOffset /= CALIBRATION_SAMPLES;
  gyroZOffset /= CALIBRATION_SAMPLES;

  accelZOffset -= 16384.0;
}

void magnetCalibrate() {
  float minMagX = 10000.0, maxMagX = -10000.0;
  float minMagY = 10000.0, maxMagY = -10000.0;
  float minMagZ = 10000.0, maxMagZ = -10000.0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float magX = readValues(QMC5883_ADDR, MAG_XOUT_L, 1);
    float magY = readValues(QMC5883_ADDR, MAG_YOUT_L, 1);
    float magZ = readValues(QMC5883_ADDR, MAG_ZOUT_L, 1);
    minMagX = min(minMagX, magX);
    maxMagX = max(maxMagX, magX);
    minMagY = min(minMagY, magY);
    maxMagY = max(maxMagY, magY);
    minMagZ = min(minMagZ, magZ);
    maxMagZ = max(maxMagZ, magZ);
  }
  offsetMagX = (minMagX + maxMagX) / 2.0;
  offsetMagY = (minMagY + maxMagY) / 2.0;
  offsetMagZ = (minMagZ + maxMagZ) / 2.0;

  float averageMagX = (maxMagX - minMagX) / 2.0;
  float averageMagY = (maxMagY - minMagY) / 2.0;
  float averageMagZ = (maxMagZ - minMagZ) / 2.0;
  float average = (averageMagX + averageMagY + averageMagZ) / 3.0;
  scaleMagX = average / averageMagX;
  scaleMagY = average / averageMagY;
  scaleMagZ = average / averageMagZ;

  yawReference = atan2(averageMagY, averageMagX) * 180.0 / PI;
}


float calculateMovingAverage(float sum[], int sumLength, float sensorReading, int &index)
{
  float totalSum = 0;
  if (index < sumLength)
  {
    sum[index] = sensorReading;
    index++;
  }
  else
  {
    for (int element = 1; element < sumLength; element++)
    {
      sum[element - 1] = sum[element];
    }
    sum[sumLength - 1] = sensorReading;
  }

  for (int element = 0; element < sumLength; element++)
  {
    totalSum += sum[element];
  }
  float accelAverage = totalSum / sumLength;

  return accelAverage;
}