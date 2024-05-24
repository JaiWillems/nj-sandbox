#include <Servo.h>
Servo ESC_FL;
Servo ESC_FR;
Servo ESC_BL;
Servo ESC_BR;


int FL_pin = 6;
int FR_pin = 10;
int BR_pin = 9;
int BL_pin = 11;
int speed= 50;
int on_period= 1000;

void setup() {
  ESC_FL.attach(FL_pin, 1000, 2000);
  ESC_BL.attach(BL_pin, 1000, 2000);
  ESC_BR.attach(BR_pin, 1000, 2000);
  ESC_FR.attach(FR_pin, 1000, 2000);

  delay(1000);
  ESC_FL.write(10);
  ESC_BL.write(10);
  ESC_FR.write(10);
  ESC_BR.write(10);
  delay(500);
  ESC_FL.write(0);
  ESC_BL.write(0);
  ESC_FR.write(0);
  ESC_BR.write(0);
  delay(500);
  ESC_FL.write(10);
  ESC_BL.write(10);
  ESC_FR.write(10);
  ESC_BR.write(10);
  delay(500);
  ESC_FL.write(0);
  ESC_BL.write(0);
  ESC_FR.write(0);
  ESC_BR.write(0);
  delay(500);
  ESC_FL.write(10);
  ESC_BL.write(10);
  ESC_FR.write(10);
  ESC_BR.write(10);
  delay(500);
  ESC_FL.write(0);
  ESC_BL.write(0);
  ESC_FR.write(0);
  ESC_BR.write(0);
  delay(500);
}

void loop() {
  ESC_FL.write(speed-15);
  ESC_BL.write(speed);
  ESC_FR.write(speed);
  ESC_BR.write(speed+15);
  delay(on_period);
  ESC_FL.write(0);
  ESC_BL.write(0);
  ESC_FR.write(0);
  ESC_BR.write(0);
  delay(6000);
  on_period+=1000;
}