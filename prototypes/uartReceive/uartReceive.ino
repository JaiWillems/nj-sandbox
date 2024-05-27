#include <PIDController.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SoftwareSerial.h>

int PID_FREQUENCY_HZ = 100;

const int START_MARKER = 255;

int RX_PIN = 3;
int TX_PIN = 4;

int throttleStickSetting;
int rollStickSetting;
int pitchStickSetting;
int yawStickSetting;

struct ControlPacket {
  int thrustInput;
  int yawInput;
  int pitchInput;
  int rollInput;
};

ControlPacket controlPacket;

SoftwareSerial cdhComms(RX_PIN, TX_PIN);

void setup() {
  Serial.begin(9600);
  cdhComms.begin(9600);
}

void loop() {

  byte* structStart = reinterpret_cast<byte*>(&controlPacket);
  if (cdhComms.available() > sizeof(controlPacket) + 1) {
    Serial.println("Comms Available...");
    byte data = cdhComms.read();

    if (data == START_MARKER) {
      Serial.println("Start Marker Found...");
      for (byte n = 0; n < sizeof(controlPacket); n++) {
        *(structStart + n) = cdhComms.read();
      }
      while (cdhComms.available() > 0) {
        byte dumpTheData = cdhComms.read();
      }

      throttleStickSetting = controlPacket.thrustInput;
      rollStickSetting = controlPacket.rollInput;
      pitchStickSetting = controlPacket.pitchInput;
      yawStickSetting = controlPacket.yawInput;
    }
  }

  Serial.print("Throttle:\t"); Serial.print(throttleStickSetting); Serial.print("\t");
  Serial.print("Roll:\t"); Serial.print(rollStickSetting); Serial.print("\t");
  Serial.print("Pitch:\t"); Serial.print(pitchStickSetting); Serial.print("\t");
  Serial.print("Yaw:\t"); Serial.println(yawStickSetting);

  delay(1000 / PID_FREQUENCY_HZ);
}
