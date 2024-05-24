#include <PIDController.h>
#include <Servo.h>

int MIN_ESC_INPUT = 1000;
int MAX_ESC_INPUT = 2000;

int MOTOR_ONE_PIN = 10;
int MOTOR_TWO_PIN = 9;
int MOTOR_THREE_PIN = 11;
int MOTOR_FOUR_PIN = 6;

bool STABILIZE_MODE = true;

int MIN_THROTTLE_STICK_POSITION = 0;  // Needs to be set.
int MAX_THROTTLE_STICK_POSITION = 0;  // Needs to be set.
int MIN_THROTTLE = 0;
int MAX_THROTTLE = MIN_ESC_INPUT + 0.9 * (MAX_ESC_INPUT - MIN_ESC_INPUT);

int MIN_ROLL_STICK_POSITION = 0;  // Needs to be set.
int MAX_ROLL_STICK_POSITION = 0;  // Needs to be set.
int MIN_ROLL_ANGLE_DEG = -60;
int MAX_ROLL_ANGLE_DEG = 60;
int MIN_ROLL_RATE_DEG_PER_SEC = -180;
int MAX_ROLL_RATE_DEG_PER_SEC = 180;

int ROLL_ANGLE_KP = 0;  // Needs to be set.
int ROLL_ANGLE_KI = 0;  // Needs to be set.
int ROLL_ANGLE_KD = 0;  // Needs to be set.

int ROLL_RATE_KP = 0;  // Needs to be set.
int ROLL_RATE_KI = 0;  // Needs to be set.
int ROLL_RATE_KD = 0;  // Needs to be set.

int MIN_PITCH_STICK_POSITION = 0;  // Needs to be set.
int MAX_PITCH_STICK_POSITION = 0;  // Needs to be set.
int MIN_PITCH_ANGLE_DEG = -60;
int MAX_PITCH_ANGLE_DEG = 60;
int MIN_PITCH_RATE_DEG_PER_SEC = -180;
int MAX_PITCH_RATE_DEG_PER_SEC = 180;

int PITCH_ANGLE_KP = 0;  // Needs to be set.
int PITCH_ANGLE_KI = 0;  // Needs to be set.
int PITCH_ANGLE_KD = 0;  // Needs to be set.

int PITCH_RATE_KP = 0;  // Needs to be set.
int PITCH_RATE_KI = 0;  // Needs to be set.
int PITCH_RATE_KD = 0;  // Needs to be set.

int MIN_YAW_STICK_POSITION = 0;  // Needs to be set.
int MAX_YAW_STICK_POSITION = 0;  // Needs to be set.
int MIN_YAW_RATE_DEG_PER_SEC = -180;
int MAX_YAW_RATE_DEG_PER_SEC = 180;

int YAW_RATE_KP = 0;  // Needs to be set.
int YAW_RATE_KI = 0;  // Needs to be set.
int YAW_RATE_KD = 0;  // Needs to be set.

Servo motorOneEsc;
Servo motorTwoEsc;
Servo motorThreeEsc;
Servo motorFourEsc;

PIDController rollAngleController;
PIDController pitchAngleController;
PIDController rollRateController;
PIDController pitchRateController;
PIDController yawRateController;

void setup() {

  motorOneEsc.attach(
    MOTOR_ONE_PIN,
    MIN_ESC_INPUT,
    MAX_ESC_INPUT
  );
  motorTwoEsc.attach(
    MOTOR_TWO_PIN,
    MIN_ESC_INPUT,
    MAX_ESC_INPUT
  );
  motorThreeEsc.attach(
    MOTOR_THREE_PIN,
    MIN_ESC_INPUT,
    MAX_ESC_INPUT
  );
  motorFourEsc.attach(
    MOTOR_FOUR_PIN,
    MIN_ESC_INPUT,
    MAX_ESC_INPUT
  );

  if (STABILIZE_MODE) {
    initializePidController(
      rollAngleController,
      ROLL_ANGLE_KP,
      ROLL_ANGLE_KI,
      ROLL_ANGLE_KD,
      MIN_ROLL_RATE_DEG_PER_SEC,
      MAX_ROLL_RATE_DEG_PER_SEC
    );
    initializePidController(
      pitchAngleController,
      PITCH_ANGLE_KP,
      PITCH_ANGLE_KI,
      PITCH_ANGLE_KD,
      MIN_PITCH_RATE_DEG_PER_SEC,
      MAX_PITCH_RATE_DEG_PER_SEC
    );
  }

  initializePidController(
    rollRateController,
    ROLL_RATE_KP,
    ROLL_RATE_KI,
    ROLL_RATE_KD,
    MIN_ESC_INPUT,
    MAX_ESC_INPUT
  );
  initializePidController(
    pitchRateController,
    PITCH_RATE_KP,
    PITCH_RATE_KI,
    PITCH_RATE_KD,
    MIN_ESC_INPUT,
    MAX_ESC_INPUT
  );
  initializePidController(
    yawRateController,
    YAW_RATE_KP,
    YAW_RATE_KI,
    YAW_RATE_KD,
    MIN_ESC_INPUT,
    MAX_ESC_INPUT
  );
}

void loop() {

  // Set from CD&H inputs.
  int throttleStickSetting = 0;
  int rollStickSetting = 0;
  int pitchStickSetting = 0;
  int yawStickSetting = 0;

  // Set from IMU reading.
  int measuredRollAngle = 0;
  int measuredPitchAngle = 0;
  int measuredRollRate = 0;
  int measuredPitchRate = 0;
  int measuredYawRate = 0;

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
      rollAngleController,
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
  float rollContribution = computePidIteration(
    rollRateController,
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
      pitchAngleController,
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
  float pitchContribution = computePidIteration(
    pitchRateController,
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
    yawRateController,
    referenceYawRate,
    measuredYawRate
  );

  writeToMotor(
    motorOneEsc,
    true,
    false,
    throttleContribution,
    yawContribution,
    pitchContribution,
    rollContribution
  );
  writeToMotor(
    motorTwoEsc,
    false,
    false,
    throttleContribution,
    yawContribution,
    pitchContribution,
    rollContribution
  );
  writeToMotor(
    motorThreeEsc,
    false,
    true,
    throttleContribution,
    yawContribution,
    pitchContribution,
    rollContribution
  );
  writeToMotor(
    motorFourEsc,
    true,
    true,
    throttleContribution,
    yawContribution,
    pitchContribution,
    rollContribution
  );
}

void initializePidController(
  PIDController pid,
  float kp,
  float ki,
  float kd,
  float minLimit,
  float maxLimit
) {
  pid.begin();
  pid.tune(kp, ki, kd);
  pid.limit(minLimit, maxLimit);
  pid.minimize(1);
}

float computePidIteration(
  PIDController pid,
  float reference,
  float measured
) {
  pid.setpoint(reference);
  return pid.compute(measured);
}

void writeToMotor(
  Servo motorEsc,
  bool bow,
  bool port,
  float throttle,
  float yaw,
  float pitch,
  float roll
) {
  motorEsc.write(
    mixMotorInputs(
      bow,
      port,
      throttle,
      yaw,
      pitch,
      roll
    )
  );
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
  motorInput = motorInput + (isCcwMotor) ? yaw : -yaw;
  motorInput = motorInput + (bow) ? pitch : -pitch;
  motorInput = motorInput + (port) ? roll : -roll;

  return motorInput;
}
