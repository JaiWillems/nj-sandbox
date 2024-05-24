#include <util/atomic.h>
#include <PIDController.h>

#define ENC_A_PIN 3
#define ENC_B_PIN 2
#define MOTOR_PWM_PIN 5
#define IN1_PIN 8
#define IN2_PIN 7

PIDController pid;

float KP = 4;
float KD = 0.025;
float KI = 0.025;

volatile int positionCounter = 0;
long previousTime = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A_PIN),
    readEncoder,
    RISING
  );

  pid.begin();
  pid.tune(KP, KI, KD);
  pid.limit(-255, 255);
  pid.minimize(1);
}

void loop() {
  
  int setpoint = 250 * sin(previousTime / 1e6);
  pid.setpoint(setpoint);

  int position = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    position = positionCounter;
  }
  setMotor(
    pid.compute(position),
    MOTOR_PWM_PIN,
    IN1_PIN,
    IN2_PIN
  );

  Serial.print(setpoint);
  Serial.print(" ");
  Serial.print(position);
  Serial.println();

  previousTime = micros();
}

void setMotor(int pwmVal, int pwmPin, int in1Pin, int in2Pin) {
  analogWrite(pwmPin, fabs(pwmVal));
  if (pwmVal > 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  else {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENC_B_PIN);
  if (b > 0) {
    positionCounter++;
  }
  else {
    positionCounter--;
  }
}
