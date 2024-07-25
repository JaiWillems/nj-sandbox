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

const int numberReadings = 20;
float accelXSum[numberReadings];
float accelYSum[numberReadings];
float accelZSum[numberReadings];
float gyroXSum[numberReadings];
float gyroYSum[numberReadings];
float gyroZSum[numberReadings];
int index = 0;

void IMUsetup();
void IMUconfig();
void IMUcalibrate();
int16_t readValues(uint8_t addr, uint8_t start);
float calculateMovingAverage(float sum[], int sumLength, float sensorReading, int &index);
float complementaryFilter(float accelAngle, float gyroAngle, float dt, float alpha);

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

  // Serial.print(filterAccelX);
  // Serial.print(",");
  // Serial.print(filterAccelY);
  // Serial.print(",");
  // Serial.print(filterAccelZ);
  // Serial.print(",");
  // Serial.println(filterGyroX);
  // Serial.print(",");
  // Serial.print(filterGyroY);
  // Serial.print(",");
  // Serial.println(filterGyroZ);

  // float accelPitch = atan2(filterAccelY, filterAccelZ) * 180.0 / 3.14;
  // float accelRoll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / 3.14;
  float accelRoll = atan2(filterAccelY, sqrt(pow(filterAccelX, 2) + pow(filterAccelZ, 2))) * 180.0 / PI;
  float accelPitch = atan2(-1 * filterAccelX, sqrt(pow(filterAccelY, 2) + pow(filterAccelZ, 2))) * 180.0 / PI;

  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float alpha = 0.98;
  float anglePitch = complementaryFilter(accelPitch, filterGyroX, dt, alpha);
  float angleRoll = complementaryFilter(accelRoll, filterGyroY, dt, alpha);

  // Serial.print(anglePitch);
  // Serial.print(",");
  // Serial.println(angleRoll);

  Serial.print(accelPitch);
  Serial.print(",");
  Serial.println(angleRoll);

  delay(100);
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

float complementaryFilter(float accelAngle, float gyroAngle, float dt, float alpha)
{
  static float angle = 0;
  float gyro = gyroAngle * dt;
  float accelerationContribution = (1 - alpha) * accelAngle;
  float gyroContribution = alpha * (angle + gyro);
  angle = accelerationContribution + gyroContribution;
  return angle;
}