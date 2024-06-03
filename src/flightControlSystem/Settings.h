
#ifndef Settings_h
#define Settings_h

// *** DRONE LEVEL ***

// bool STABILIZE_MODE = false;

// *** MOTOR ***

const int MIN_MOTOR_INPUT = 1000;
const int MAX_MOTOR_INPUT = 2000;

// *** CONTROL MAPPING ***

const int MIN_THROTTLE_STICK_POSITION = 0;
const int MAX_THROTTLE_STICK_POSITION = 1023;
const float MIN_ALTITUDE_RATE_M_PER_SEC = -0.5;
const float MAX_ALTITUDE_RATE_M_PER_SEC = 0.5;

const int MIN_YAW_STICK_POSITION = 0;
const int MAX_YAW_STICK_POSITION = 1023;
const int MIN_YAW_RATE_DEG_PER_SEC = -180;
const int MAX_YAW_RATE_DEG_PER_SEC = 180;

const int MIN_PITCH_STICK_POSITION = 1023;
const int MAX_PITCH_STICK_POSITION = 0;
const int MIN_PITCH_ANGLE_DEG = -60;
const int MAX_PITCH_ANGLE_DEG = 60;
const int MIN_PITCH_RATE_DEG_PER_SEC = -180;
const int MAX_PITCH_RATE_DEG_PER_SEC = 180;

const int MIN_ROLL_STICK_POSITION = 0;
const int MAX_ROLL_STICK_POSITION = 1023;
const int MIN_ROLL_ANGLE_DEG = -60;
const int MAX_ROLL_ANGLE_DEG = 60;
const int MIN_ROLL_RATE_DEG_PER_SEC = -180;
const int MAX_ROLL_RATE_DEG_PER_SEC = 180;

// *** SENSOR ***

const int IMU_CALIBRATION_LOOPS = 6;
const int UART_BAUD_RATE = 9600;

// *** PID ***

const int MIN_ESC_INPUT_FROM_PID = -50;
const int MAX_ESC_INPUT_FROM_PID = 50;

const float ALTITUDE_KP = 25.0;
const float ALTITUDE_KI = 0.0;
const float ALTITUDE_KD = 0.0;

const float YAW_RATE_KP = 1.0;
const float YAW_RATE_KI = 0.0;
const float YAW_RATE_KD = 0.0;

const float PITCH_KP = 1.0;
const float PITCH_KI = 0.0;
const float PITCH_KD = 0.0;

const float PITCH_RATE_KP = 1.0;
const float PITCH_RATE_KI = 0.0;
const float PITCH_RATE_KD = 0.0;

const float ROLL_KP = 1.0;
const float ROLL_KI = 0.0;
const float ROLL_KD = 0.0;

const float ROLL_RATE_KP = 1.0;
const float ROLL_RATE_KI = 0.0;
const float ROLL_RATE_KD = 0.0;

#endif