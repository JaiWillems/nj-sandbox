#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SoftwareSerial.h>

#include "PidController.h"
#include "motor.h"

// *** DRONE LEVEL SETTINGS ***

bool STABILIZE_MODE = true;

// *** MOTOR SETTINGS ***

int MIN_ESC_INPUT = 1000;
int MAX_ESC_INPUT = 2000;
int MIN_ESC_INPUT_FROM_PID = -50;
int MAX_ESC_INPUT_FROM_PID = 50;

int MOTOR_ONE_ESC_PIN = 10;
int MOTOR_TWO_ESC_PIN = 9;
int MOTOR_THREE_ESC_PIN = 11;
int MOTOR_FOUR_ESC_PIN = 6;

// *** COMMS SETTINGS ***

const int START_MARKER = 255;
int RX_PIN = 3;
int TX_PIN = 4;

// *** THROTTLE SETTINGS ***

int MIN_THROTTLE_STICK_POSITION = 0;
int MAX_THROTTLE_STICK_POSITION = 1023;
int MIN_THROTTLE = MIN_ESC_INPUT;
int MAX_THROTTLE = MIN_ESC_INPUT + 0.85 * (MAX_ESC_INPUT - MIN_ESC_INPUT);

// *** ROLL SETTINGS ***

int MIN_ROLL_STICK_POSITION = 0;
int MAX_ROLL_STICK_POSITION = 1023;
int MIN_ROLL_ANGLE_DEG = -60;
int MAX_ROLL_ANGLE_DEG = 60;
int MIN_ROLL_RATE_DEG_PER_SEC = -180;
int MAX_ROLL_RATE_DEG_PER_SEC = 180;

// *** PITCH SETTINGS ***

int MIN_PITCH_STICK_POSITION = 1023;
int MAX_PITCH_STICK_POSITION = 0;
int MIN_PITCH_ANGLE_DEG = -60;
int MAX_PITCH_ANGLE_DEG = 60;
int MIN_PITCH_RATE_DEG_PER_SEC = -180;
int MAX_PITCH_RATE_DEG_PER_SEC = 180;

// *** YAW SETTINGS ***

int MIN_YAW_STICK_POSITION = 0;
int MAX_YAW_STICK_POSITION = 1023;
int MIN_YAW_RATE_DEG_PER_SEC = -180;
int MAX_YAW_RATE_DEG_PER_SEC = 180;

// *** PID SETTINGS ***

int PID_FREQUENCY_HZ = 100;

float ROLL_ANGLE_KP = 1.0;
float ROLL_ANGLE_KI = 0.0;
float ROLL_ANGLE_KD = 0.0;

float PITCH_ANGLE_KP = 1.0;
float PITCH_ANGLE_KI = 0.0;
float PITCH_ANGLE_KD = 0.0;

float ROLL_RATE_KP = 1.0;
float ROLL_RATE_KI = 0.0;
float ROLL_RATE_KD = 0.0;

float PITCH_RATE_KP = 1.0;
float PITCH_RATE_KI = 0.0;
float PITCH_RATE_KD = 0.0;

float YAW_RATE_KP = 1.0;
float YAW_RATE_KI = 0.0;
float YAW_RATE_KD = 0.0;

// *** END OF SETTINGS ***

struct ControlPacket {
  int thrustInput;
  int yawInput;
  int pitchInput;
  int rollInput;
};

ControlPacket controlPacket;

SoftwareSerial cdhComms(RX_PIN, TX_PIN);

int throttleStickSetting = 512;
int rollStickSetting = 512;
int pitchStickSetting = 512;
int yawStickSetting = 512;

MPU6050 imu;
float measuredRollAngle;
float measuredPitchAngle;
int16_t measuredRollRate;
int16_t measuredPitchRate;
int16_t measuredYawRate;

Motor motorOne(
  MOTOR_ONE_ESC_PIN,
  MIN_ESC_INPUT,
  MAX_ESC_INPUT,
  true,
  false
);
Motor motorTwo(
  MOTOR_TWO_ESC_PIN,
  MIN_ESC_INPUT,
  MAX_ESC_INPUT,
  false,
  false
);
Motor motorThree(
  MOTOR_THREE_ESC_PIN,
  MIN_ESC_INPUT,
  MAX_ESC_INPUT,
  false,
  true
);
Motor motorFour(
  MOTOR_FOUR_ESC_PIN,
  MIN_ESC_INPUT,
  MAX_ESC_INPUT,
  true,
  true
);

PidController rollAngleController(
  ROLL_ANGLE_KP,
  ROLL_ANGLE_KI,
  ROLL_ANGLE_KD,
  MIN_ROLL_RATE_DEG_PER_SEC,
  MAX_ROLL_RATE_DEG_PER_SEC
);
PidController pitchAngleController(
  PITCH_ANGLE_KP,
  PITCH_ANGLE_KI,
  PITCH_ANGLE_KD,
  MIN_PITCH_RATE_DEG_PER_SEC,
  MAX_PITCH_RATE_DEG_PER_SEC
);
PidController rollRateController(
  ROLL_RATE_KP,
  ROLL_RATE_KI,
  ROLL_RATE_KD,
  MIN_ESC_INPUT_FROM_PID,
  MAX_ESC_INPUT_FROM_PID
);
PidController pitchRateController(
  PITCH_RATE_KP,
  PITCH_RATE_KI,
  PITCH_RATE_KD,
  MIN_ESC_INPUT_FROM_PID,
  MAX_ESC_INPUT_FROM_PID
);
PidController yawRateController(
  YAW_RATE_KP,
  YAW_RATE_KI,
  YAW_RATE_KD,
  MIN_ESC_INPUT_FROM_PID,
  MAX_ESC_INPUT_FROM_PID
);

void setup() {

  Serial.begin(9600);
  cdhComms.begin(9600);

  Wire.begin();
  imu.initialize();

  if (imu.testConnection()) {
    Serial.print("IMU Connection Successful.");
  }
  else {
    Serial.print("IMU Connection Failed...");
  }

  if (imu.dmpInitialize() == 0) {
      imu.CalibrateAccel(6);
      imu.CalibrateGyro(6);
      imu.setDMPEnabled(true);
  }
  else {
    Serial.println("DMP Initialization Failure...");
  }

  if (STABILIZE_MODE) {
    rollAngleController.begin();
    pitchAngleController.begin();
  }
  rollRateController.begin();
  pitchRateController.begin();
  yawRateController.begin();
}

void loop() {

  byte* structStart = reinterpret_cast<byte*>(&controlPacket);
  if (cdhComms.available() > sizeof(controlPacket) + 1) {

    byte data = cdhComms.read();

    if (data == START_MARKER) {

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

  imu.getRotation(
    &measuredRollRate,
    &measuredPitchRate,
    &measuredYawRate
  );

  // Negate pitch/yaw rates to get in the drone frame.
  measuredPitchRate = -measuredPitchRate;
  measuredYawRate = -measuredYawRate;

  uint8_t fifoBuffer[64];
  if (imu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // TODO: Handle dmpGetCurrentFIFOPacket function
    // when it hangs.

    Quaternion orientation;
    VectorFloat gravity;
    float ypr[3];

    imu.dmpGetQuaternion(
      &orientation,
      fifoBuffer
    );
    imu.dmpGetGravity(
      &gravity,
      &orientation
    );
    imu.dmpGetYawPitchRoll(
      ypr,
      &orientation,
      &gravity
    );

    measuredPitchAngle = ypr[1] * RAD_TO_DEG;
    measuredRollAngle = ypr[2] * RAD_TO_DEG;
  }

  int throttleContribution = map(
    throttleStickSetting,
    MIN_THROTTLE_STICK_POSITION,
    MAX_THROTTLE_STICK_POSITION,
    MIN_THROTTLE,
    MAX_THROTTLE
  );

  float referenceRollRate;
  if (STABILIZE_MODE) {
    int referenceRollAngle = map(
      rollStickSetting,
      MIN_ROLL_STICK_POSITION,
      MAX_ROLL_STICK_POSITION,
      MIN_ROLL_ANGLE_DEG,
      MAX_ROLL_ANGLE_DEG
    );
    referenceRollRate = rollAngleController.compute(
      referenceRollAngle,
      measuredRollAngle
    );
  }
  else {
    referenceRollRate = map(
      rollStickSetting,
      MIN_ROLL_STICK_POSITION,
      MAX_ROLL_STICK_POSITION,
      MIN_ROLL_RATE_DEG_PER_SEC,
      MAX_ROLL_RATE_DEG_PER_SEC
    );
  }
  float rollContribution = rollRateController.compute(
    referenceRollRate,
    measuredRollRate
  );

  float referencePitchRate;
  if (STABILIZE_MODE) {
    int referencePitchAngle = map(
      pitchStickSetting,
      MIN_PITCH_STICK_POSITION,
      MAX_PITCH_STICK_POSITION,
      MIN_PITCH_ANGLE_DEG,
      MAX_PITCH_ANGLE_DEG
    );
    referencePitchRate = pitchAngleController.compute(
      referencePitchAngle,
      measuredPitchAngle
    );
  }
  else {
    referencePitchRate = map(
      pitchStickSetting,
      MIN_PITCH_STICK_POSITION,
      MAX_PITCH_STICK_POSITION,
      MIN_PITCH_RATE_DEG_PER_SEC,
      MAX_PITCH_RATE_DEG_PER_SEC
    );
  }
  float pitchContribution = pitchRateController.compute(
    referencePitchRate,
    measuredPitchRate
  );

  int referenceYawRate = map(
    yawStickSetting,
    MIN_YAW_STICK_POSITION,
    MAX_YAW_STICK_POSITION,
    MIN_YAW_RATE_DEG_PER_SEC,
    MAX_YAW_RATE_DEG_PER_SEC
  );
  float yawContribution = yawRateController.compute(
    referenceYawRate,
    measuredYawRate
  );

  motorOne.setSpeed(
    throttleContribution,
    yawContribution,
    pitchContribution,
    rollContribution
  );
  motorTwo.setSpeed(
    throttleContribution,
    yawContribution,
    pitchContribution,
    rollContribution
  );
  motorThree.setSpeed(
    throttleContribution,
    yawContribution,
    pitchContribution,
    rollContribution
  );
  motorFour.setSpeed(
    throttleContribution,
    yawContribution,
    pitchContribution,
    rollContribution
  );

  delay(1000 / PID_FREQUENCY_HZ);
}
