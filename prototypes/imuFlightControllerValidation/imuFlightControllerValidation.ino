#include <PIDController.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// *** DEBUG SETTINGS ***

bool PRINT_IMU_DATA = true;
bool PLOT_RATE_DATA = false;
bool PLOT_ANGLE_DATA = false;
bool PLOT_ANGLE_PID_DATA = false;
bool PRINT_YAW_RATE_PID_INFORMATION = false;
bool PRINT_MOTOR_OUTPUTS = false;

// *** DRONE LEVEL SETTINGS ***

bool STABILIZE_MODE = true;

// *** MOTOR SETTINGS ***

int MIN_ESC_INPUT = 1000;
int MAX_ESC_INPUT = 2000;
int MIN_ESC_INPUT_FROM_PID = -100;
int MAX_ESC_INPUT_FROM_PID = 100;

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

int MIN_PITCH_STICK_POSITION = 0;
int MAX_PITCH_STICK_POSITION = 1023;
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

int PID_FREQUENCY_HZ = 10;

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

MPU6050 imu;
int16_t measuredRollRate;
int16_t measuredPitchRate;
int16_t measuredYawRate;
float measuredRollAngle;
float measuredPitchAngle;

PIDController rollAngleController;
PIDController pitchAngleController;
PIDController rollRateController;
PIDController pitchRateController;
PIDController yawRateController;

void setup() {

  Serial.begin(115200);

  imu.initialize();

  if (imu.dmpInitialize() == 0) {
      imu.CalibrateAccel(6);
      imu.CalibrateGyro(6);
      imu.setDMPEnabled(true);
  }
  else {
    Serial.print("DMP Initialization Failed");
  }

  if (STABILIZE_MODE) {
    initializePidController(
      &rollAngleController,
      ROLL_ANGLE_KP,
      ROLL_ANGLE_KI,
      ROLL_ANGLE_KD,
      MIN_ROLL_RATE_DEG_PER_SEC,
      MAX_ROLL_RATE_DEG_PER_SEC
    );
    initializePidController(
      &pitchAngleController,
      PITCH_ANGLE_KP,
      PITCH_ANGLE_KI,
      PITCH_ANGLE_KD,
      MIN_PITCH_RATE_DEG_PER_SEC,
      MAX_PITCH_RATE_DEG_PER_SEC
    );
  }

  initializePidController(
    &rollRateController,
    ROLL_RATE_KP,
    ROLL_RATE_KI,
    ROLL_RATE_KD,
    MIN_ESC_INPUT_FROM_PID,
    MAX_ESC_INPUT_FROM_PID
  );
  initializePidController(
    &pitchRateController,
    PITCH_RATE_KP,
    PITCH_RATE_KI,
    PITCH_RATE_KD,
    MIN_ESC_INPUT_FROM_PID,
    MAX_ESC_INPUT_FROM_PID
  );
  initializePidController(
    &yawRateController,
    YAW_RATE_KP,
    YAW_RATE_KI,
    YAW_RATE_KD,
    MIN_ESC_INPUT_FROM_PID,
    MAX_ESC_INPUT_FROM_PID
  );

  Serial.println("End of Setup");
}

void loop() {

  // Set from CD&H inputs.
  int throttleStickSetting = 512;
  int rollStickSetting = 512;
  int pitchStickSetting = 512;
  int yawStickSetting = 512;

  uint8_t fifoBuffer[64];
  if (imu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

    Quaternion orientation;
    VectorFloat gravity;
    float ypr[3];

    imu.getRotation(
      &measuredRollRate,
      &measuredPitchRate,
      &measuredYawRate
    );

    // Need to negate pitch and yaw rates to get in drone frame.
    measuredPitchRate = -measuredPitchRate;
    measuredYawRate = -measuredYawRate;

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
  else {
    Serial.println("No IMU Data");
  }

  if (PRINT_IMU_DATA) {
    Serial.print("Pitch:\t"); Serial.print(measuredPitchAngle); Serial.print("\t");
    Serial.print("Roll:\t"); Serial.print(measuredRollAngle); Serial.print("\t");
    Serial.print("Yaw Rate:\t"); Serial.print(measuredYawRate); Serial.print("\t");
    Serial.print("Pitch Rate:\t"); Serial.print(measuredPitchRate); Serial.print("\t");
    Serial.print("Roll Rate:\t"); Serial.println(measuredRollRate);
  }
  if (PLOT_RATE_DATA) {
    Serial.print(measuredYawRate);
    Serial.print(" ");
    Serial.print(measuredPitchRate);
    Serial.print(" ");
    Serial.println(measuredRollRate);
  }
  if (PLOT_ANGLE_DATA) {
    Serial.print(measuredPitchAngle);
    Serial.print(" ");
    Serial.println(measuredRollAngle);
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
    referenceRollRate = computePidIteration(
      &rollAngleController,
      referenceRollAngle,
      measuredRollAngle
    );
    if (PLOT_ANGLE_PID_DATA) {
      Serial.print(referenceRollRate);
      Serial.print(" ");
    }
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
  float rollContribution = computePidIteration(
    &rollRateController,
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
    referencePitchRate = computePidIteration(
      &pitchAngleController,
      referencePitchAngle,
      measuredPitchAngle
    );
    if (PLOT_ANGLE_PID_DATA) {
      Serial.println(referencePitchRate);
    }
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
  float pitchContribution = computePidIteration(
    &pitchRateController,
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
  float yawContribution = computePidIteration(
    &yawRateController,
    referenceYawRate,
    measuredYawRate
  );

  if (PRINT_YAW_RATE_PID_INFORMATION) {
    Serial.print("Yaw Contribution:\t"); Serial.print(yawContribution); Serial.print("\t");
    Serial.print("Reference:\t"); Serial.print(referenceYawRate); Serial.print("\t");
    Serial.print("Measured:\t"); Serial.println(measuredYawRate);
  }

  if (PRINT_MOTOR_OUTPUTS) {
    Serial.print("Motor 1: ");
    Serial.print(
      mixMotorInputs(
        true,
        false,
        throttleContribution,
        yawContribution,
        pitchContribution,
        rollContribution
      )
    );
    Serial.print("\t");

    Serial.print("Motor 2: ");
    Serial.print(
      mixMotorInputs(
        false,
        false,
        throttleContribution,
        yawContribution,
        pitchContribution,
        rollContribution
      )
    );
    Serial.print("\t");

    Serial.print("Motor 3: ");
    Serial.print(
      mixMotorInputs(
        false,
        true,
        throttleContribution,
        yawContribution,
        pitchContribution,
        rollContribution
      )
    );
    Serial.print("\t");

    Serial.print("Motor 4: ");
    Serial.print(
      mixMotorInputs(
        true,
        true,
        throttleContribution,
        yawContribution,
        pitchContribution,
        rollContribution
      )
    );
    Serial.print("\t");

    Serial.println();
  }

  delay(1000 / PID_FREQUENCY_HZ);
}

void initializePidController(
  PIDController* pid,
  float kp,
  float ki,
  float kd,
  float minLimit,
  float maxLimit
) {
  (*pid).begin();
  (*pid).tune(kp, ki, kd);
  (*pid).limit(minLimit, maxLimit);
  (*pid).minimize(1);
}

float computePidIteration(
  PIDController* pid,
  float reference,
  float measured
) {
  (*pid).setpoint(reference);
  return (*pid).compute(measured);
}

float mixMotorInputs(
  bool bow,
  bool port,
  float throttle,
  float yaw,
  float pitch,
  float roll
) {
  bool isCcwMotor = (bow && !port) || (!bow && port);

  float motorInput = throttle;
  motorInput = motorInput + ((isCcwMotor) ? yaw : -yaw);
  motorInput = motorInput + ((bow) ? pitch : -pitch);
  motorInput = motorInput + ((port) ? roll : -roll);

  return constrain(
    motorInput,
    MIN_ESC_INPUT,
    MAX_ESC_INPUT
  );
}
