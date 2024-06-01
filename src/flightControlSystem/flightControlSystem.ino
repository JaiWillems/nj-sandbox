#include <SoftwareSerial.h>
#include <BMP180.h>
#include <Wire.h>
#include "Imu.h"
#include "PidController.h"
#include "Drone.h"


// *** DRONE LEVEL SETTINGS ***

bool STABILIZE_MODE = true;

// *** MOTOR SETTINGS ***

int MIN_ESC_INPUT = 1000;
int MAX_ESC_INPUT = 2000;
int MIN_ESC_INPUT_FROM_PID = -50;
int MAX_ESC_INPUT_FROM_PID = 50;

int MIN_THROTTLE_INPUT_FROM_PID= MIN_ESC_INPUT- MAX_ESC_INPUT;
int MAX_THROTTLE_INPUT_FROM_PID= MAX_ESC_INPUT- MIN_ESC_INPUT;

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
int MIN_ALTITUDE_RATE_M_PER_SEC = -2;
int MAX_ALTITUDE_RATE_M_PER_SEC = 2;
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

float ALTITUDE_KP = 1.0;
float ALTITUDE_KI = 0.0;
float ALTITUDE_KD = 0.0;

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

Imu imu;
float measuredYawAngle;
float measuredPitchAngle;
float measuredRollAngle;
int16_t measuredYawRate;
int16_t measuredPitchRate;
int16_t measuredRollRate;

BMP180 bmp(BMP180_ULTRAHIGHRES);

float groundLevelPressure;
float measuredPressure;
float measuredAltitude;

float referenceAltitude;
double previousTime = 0.0;
double currentTime = 0.0;

Drone drone(
  MOTOR_ONE_ESC_PIN,
  MOTOR_TWO_ESC_PIN,
  MOTOR_THREE_ESC_PIN,
  MOTOR_FOUR_ESC_PIN,
  MIN_ESC_INPUT,
  MAX_ESC_INPUT
);

PidController altitudeController;
PidController rollAngleController;
PidController pitchAngleController;
PidController rollRateController;
PidController pitchRateController;
PidController yawRateController;

void setup() {

  Serial.begin(9600);
  cdhComms.begin(9600);

  imu.initialize();
  if (imu.testConnection()) {
    Serial.print("IMU Connection Successful.");
  }
  else {
    Serial.print("IMU Connection Failed...");
  }
  if (bmp.begin()){
    Serial.print("BMP Connection Successful...");
  }
  else{
    Serial.print("BMP Connection Failed...");
  }
  if (imu.initializeDmp() == 0) {
    imu.calibrate(6);
  }
  else {
    Serial.println("DMP Initialization Failure...");
  }

  altitudeController.initialize(
    ALTITUDE_KP,
    ALTITUDE_KI,
    ALTITUDE_KD,
    MIN_THROTTLE_INPUT_FROM_PID,
    MAX_THROTTLE_INPUT_FROM_PID
  );
  altitudeController.begin();
  if (STABILIZE_MODE) {
    rollAngleController.initialize(
      ROLL_ANGLE_KP,
      ROLL_ANGLE_KI,
      ROLL_ANGLE_KD,
      MIN_ROLL_RATE_DEG_PER_SEC,
      MAX_ROLL_RATE_DEG_PER_SEC
    );
    rollAngleController.begin();
    pitchAngleController.initialize(
      PITCH_ANGLE_KP,
      PITCH_ANGLE_KI,
      PITCH_ANGLE_KD,
      MIN_PITCH_RATE_DEG_PER_SEC,
      MAX_PITCH_RATE_DEG_PER_SEC
    );
    pitchAngleController.begin();
  }
  rollRateController.begin();
  pitchRateController.initialize(
    PITCH_RATE_KP,
    PITCH_RATE_KI,
    PITCH_RATE_KD,
    MIN_ESC_INPUT_FROM_PID,
    MAX_ESC_INPUT_FROM_PID
  );
  pitchRateController.begin();
  yawRateController.initialize(
    YAW_RATE_KP,
    YAW_RATE_KI,
    YAW_RATE_KD,
    MIN_ESC_INPUT_FROM_PID,
    MAX_ESC_INPUT_FROM_PID
  );
  yawRateController.begin();

  groundLevelPressure = bmp.getPressure();
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

  imu.getRotationRates(
    &measuredRollRate,
    &measuredPitchRate,
    &measuredYawRate
  );

  uint8_t fifoBuffer[64];
  if (imu.getCurrentFIFOPacket(fifoBuffer)) {
    // TODO: Handle dmpGetCurrentFIFOPacket function
    // when it hangs.

    imu.getYawPitchRollFromDmp(
      fifoBuffer,
      &measuredYawAngle,
      &measuredPitchAngle,
      &measuredRollAngle
    );
  }

  measuredPressure = bmp.getPressure();
  measuredAltitude = 4430 * (1- pow(
    (measuredPressure/groundLevelPressure),
    (1/5.255)));

  int referenceAltitudeRate = map(
    throttleStickSetting,
    MIN_THROTTLE_STICK_POSITION,
    MAX_THROTTLE_STICK_POSITION,
    MIN_ALTITUDE_RATE_M_PER_SEC,
    MAX_ALTITUDE_RATE_M_PER_SEC
  );
  currentTime= millis();
  double deltaTime = (currentTime - previousTime)/1000;
  referenceAltitude = referenceAltitude + deltaTime * 
    referenceAltitudeRate;
  previousTime= currentTime;

  float throttleInput = altitudeController.compute(
    referenceAltitude,
    measuredAltitude
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
  float rollInput = rollRateController.compute(
    referenceRollRate,
    measuredRollRate
  );

  float referencePitchRate;
  if (STABILIZE_MODE) {
    float referencePitchAngle = map(
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
  float pitchInput = pitchRateController.compute(
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
  float yawInput = yawRateController.compute(
    referenceYawRate,
    measuredYawRate
  );

  drone.sendControlInputs(
    throttleInput,
    yawInput,
    pitchInput,
    rollInput
  );

  delay(1000 / PID_FREQUENCY_HZ);
}
