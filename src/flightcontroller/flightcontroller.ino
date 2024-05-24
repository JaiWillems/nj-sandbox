#include <PIDController.h>
#include <Servo.h>

int ESC_INPUT_MIN = 1000;
int ESC_INPUT_MAX = 2000;

int MOTOR_ONE_PIN = 10;
int MOTOR_TWO_PIN = 9;
int MOTOR_THREE_PIN = 11;
int MOTOR_FOUR_PIN = 6;

int MIN_THROTTLE_STICK_POSITION = 0;  // Needs to be set.
int MAX_THROTTLE_STICK_POSITION = 0;  // Needs to be set.
int MIN_THROTTLE = 0;
int MAX_THROTTLE = ESC_INPUT_MIN + 0.9 * (ESC_INPUT_MAX - ESC_INPUT_MIN);

int MIN_ROLL_STICK_POSITION = 0;  // Needs to be set.
int MAX_ROLL_STICK_POSITION = 0;  // Needs to be set.
int MIN_ROLL_RATE_DEG_PER_SEC = -180;
int MAX_ROLL_RATE_DEG_PER_SEC = 180;

int ROLL_RATE_KP = 0;  // Needs to be set.
int ROLL_RATE_KI = 0;  // Needs to be set.
int ROLL_RATE_KD = 0;  // Needs to be set.

int MIN_PITCH_STICK_POSITION = 0;  // Needs to be set.
int MAX_PITCH_STICK_POSITION = 0;  // Needs to be set.
int MIN_PITCH_RATE_DEG_PER_SEC = -180;
int MAX_PITCH_RATE_DEG_PER_SEC = 180;

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

PIDController rollRateController;
PIDController pitchRateController;
PIDController yawRateController;

void setup() {

  motorOneEsc.attach(
    MOTOR_ONE_PIN,
    ESC_INPUT_MIN,
    ESC_INPUT_MAX
  );
  motorTwoEsc.attach(
    MOTOR_TWO_PIN,
    ESC_INPUT_MIN,
    ESC_INPUT_MAX
  );
  motorThreeEsc.attach(
    MOTOR_THREE_PIN,
    ESC_INPUT_MIN,
    ESC_INPUT_MAX
  );
  motorFourEsc.attach(
    MOTOR_FOUR_PIN,
    ESC_INPUT_MIN,
    ESC_INPUT_MAX
  );

  rollRateController.begin();
  rollRateController.tune(
    ROLL_RATE_KP,
    ROLL_RATE_KI,
    ROLL_RATE_KD
  );
  rollRateController.limit(
    MIN_ROLL_RATE_DEG_PER_SEC,
    MAX_ROLL_RATE_DEG_PER_SEC
  );
  rollRateController.minimize(1);

  pitchRateController.begin();
  pitchRateController.tune(
    PITCH_RATE_KP,
    PITCH_RATE_KI,
    PITCH_RATE_KD
  );
  pitchRateController.limit(
    MIN_PITCH_RATE_DEG_PER_SEC,
    MAX_PITCH_RATE_DEG_PER_SEC
  );
  pitchRateController.minimize(1);

  yawRateController.begin();
  yawRateController.tune(
    YAW_RATE_KP,
    YAW_RATE_KI,
    YAW_RATE_KD
  );
  yawRateController.limit(
    MIN_YAW_RATE_DEG_PER_SEC,
    MAX_YAW_RATE_DEG_PER_SEC
  );
  yawRateController.minimize(1);
}

void loop() {

  // Set from CD&H inputs.
  int throttleStickSetting = 0;
  int rollStickSetting = 0;
  int pitchStickSetting = 0;
  int yawStickSetting = 0;

  // Set from IMU reading.
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

  int referenceRollRate = map(
    rollStickSetting,
    MIN_ROLL_STICK_POSITION,
    MAX_ROLL_STICK_POSITION,
    MIN_ROLL_RATE_DEG_PER_SEC,
    MAX_ROLL_RATE_DEG_PER_SEC
  );
  rollRateController.setpoint(referenceRollRate);
  float rollContribution = rollRateController.compute(
    measuredRollRate
  );

  int referencePitchRate = map(
    pitchStickSetting,
    MIN_PITCH_STICK_POSITION,
    MAX_PITCH_STICK_POSITION,
    MIN_PITCH_RATE_DEG_PER_SEC,
    MAX_PITCH_RATE_DEG_PER_SEC
  );
  pitchRateController.setpoint(referencePitchRate);
  float pitchContribution = pitchRateController.compute(
    measuredPitchRate
  );

  int referenceYawRate = map(
    yawStickSetting,
    MIN_YAW_STICK_POSITION,
    MAX_YAW_STICK_POSITION,
    MIN_YAW_RATE_DEG_PER_SEC,
    MAX_YAW_RATE_DEG_PER_SEC
  );
  yawRateController.setpoint(referenceYawRate);
  float yawContribution = yawRateController.compute(
    measuredYawRate
  );

  motorOneEsc.write(
    throttleContribution + yawContribution + pitchContribution - rollContribution
  );
  motorTwoEsc.write(
    throttleContribution - yawContribution - pitchContribution - rollContribution
  );
  motorThreeEsc.write(
    throttleContribution + yawContribution - pitchContribution + rollContribution
  );
  motorFourEsc.write(
    throttleContribution - yawContribution + pitchContribution + rollContribution
  );
}
