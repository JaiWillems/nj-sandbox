#include <Arduino.h>
#include <Wire.h>

#define GY87_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define ACCEL_CONF 0x1C
#define GYRO_CONF 0x1B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

#define CALIBRATION_SAMPLES 1000
float accelXOffset = 0;
float accelYOffset = 0;
float accelZOffset = 0;
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;
float gyroAngleX, gyroAngleY;

const int numberReadings = 20;
float accelXSum[numberReadings];
float accelYSum[numberReadings];
float accelZSum[numberReadings];
float gyroXSum[numberReadings];
float gyroYSum[numberReadings];
float gyroZSum[numberReadings];
float kalmanPitchSum[numberReadings];
float kalmanRollSum[numberReadings];
int index = 0;

float kalmanPitch = 0;
float uncertaintyPitch = 4;
float kalmanRoll = 0;
float uncertaintyRoll = 4;
float controlMatrix = 0.005;
float processNoiseVariance = 4.0;
float measurementNoiseVariance = 3.0;
float kalmanState[] = {0, 0};

void IMUsetup();
void IMUconfig();
void IMUcalibrate();
int16_t readValues(uint8_t addr, uint8_t start);
float calculateMovingAverage(float sum[], int sumLength, float sensorReading, int &index);
void kalmanFilter(float state, float uncertainty, float accelInput, float gyroInput);
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  IMUsetup();
  delay(100);
  IMUconfig();
  IMUcalibrate();
}

void loop()
{
  float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
  float filterAccelX, filterAccelY, filterAccelZ, filterGyroX, filterGyroY, filterGyroZ;
  accelX = (readValues(GY87_ADDR, ACCEL_XOUT_H) - accelXOffset) / 16384.0;
  accelY = (readValues(GY87_ADDR, ACCEL_YOUT_H) - accelYOffset) / 16384.0;
  accelZ = (readValues(GY87_ADDR, ACCEL_ZOUT_H) - accelZOffset) / 16384.0;

  gyroX = (readValues(GY87_ADDR, GYRO_XOUT_H) - gyroXOffset) / 131.0;
  gyroY = (readValues(GY87_ADDR, GYRO_YOUT_H) - gyroYOffset) / 131.0;
  gyroZ = (readValues(GY87_ADDR, GYRO_ZOUT_H) - gyroZOffset) / 131.0;

  filterAccelX = calculateMovingAverage(accelXSum, numberReadings, accelX, index);
  filterAccelY = calculateMovingAverage(accelYSum, numberReadings, accelY, index);
  filterAccelZ = calculateMovingAverage(accelZSum, numberReadings, accelZ, index);

  filterGyroX = calculateMovingAverage(gyroXSum, numberReadings, gyroX, index);
  filterGyroY = calculateMovingAverage(gyroYSum, numberReadings, gyroY, index);
  filterGyroZ = calculateMovingAverage(gyroZSum, numberReadings, gyroZ, index);

  float accelRoll = atan2(filterAccelY, filterAccelZ) * 180.0 / PI;
  float accelPitch = atan2((-filterAccelX), sqrt(filterAccelY * filterAccelY + filterAccelZ * filterAccelZ)) * 180.0 / PI;

  kalmanFilter(kalmanPitch, uncertaintyPitch, accelPitch, filterGyroY);
  kalmanPitch = kalmanState[0];
  uncertaintyPitch = kalmanState[1];
  kalmanFilter(kalmanRoll, uncertaintyRoll, accelRoll, filterGyroX);
  kalmanRoll = kalmanState[0];
  uncertaintyRoll = kalmanState[1];

  Serial.print(kalmanPitch);
  Serial.print(" ");
  Serial.print(kalmanRoll);
  Serial.print(" ");

  float filterKalmanPitch = calculateMovingAverage(kalmanPitchSum, numberReadings, kalmanPitch, index);
  float filterKalmanRoll = calculateMovingAverage(kalmanRollSum, numberReadings, kalmanRoll, index);
  Serial.print(filterKalmanPitch);
  Serial.print(" ");
  Serial.println(filterKalmanRoll);
}
void IMUsetup()
{
  Wire.beginTransmission(GY87_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
}
void IMUconfig()
{
  Wire.beginTransmission(GY87_ADDR);
  Wire.write(ACCEL_CONF);
  Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.beginTransmission(GY87_ADDR);
  Wire.write(GYRO_CONF);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.beginTransmission(GY87_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.beginTransmission(GY87_ADDR);
  Wire.write(SMPLRT_DIV);
  Wire.write(0x09);
  Wire.endTransmission(true);
}
int16_t readValues(uint8_t addr, uint8_t start)
{
  Wire.beginTransmission(addr);
  Wire.write(start);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 2, true);
  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();
  float accel = (highByte << 8) | lowByte;
  return accel;
}

void IMUcalibrate()
{
  for (int element = 0; element < CALIBRATION_SAMPLES; element++)
  {
    int16_t ax = readValues(GY87_ADDR, ACCEL_XOUT_H);
    int16_t ay = readValues(GY87_ADDR, ACCEL_YOUT_H);
    int16_t az = readValues(GY87_ADDR, ACCEL_ZOUT_H);
    int16_t gx = readValues(GY87_ADDR, GYRO_XOUT_H);
    int16_t gy = readValues(GY87_ADDR, GYRO_YOUT_H);
    int16_t gz = readValues(GY87_ADDR, GYRO_ZOUT_H);

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
  delay(10);
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

void kalmanFilter(float state, float uncertainty, float accelInput, float gyroInput)
{
  state = state + controlMatrix * gyroInput;
  uncertainty = uncertainty + (controlMatrix * controlMatrix + processNoiseVariance * processNoiseVariance);
  float kalmanGain = uncertainty / (uncertainty + measurementNoiseVariance * measurementNoiseVariance);
  state = state + kalmanGain * (accelInput - state);
  uncertainty = (1 - kalmanGain) * uncertainty;

  kalmanState[0] = state;
  kalmanState[1] = uncertainty;
}