#include <Servo.h>
Servo ESC_BR;

int BR_pin= 9;


void setup() {
  ESC_BR.attach(BR_pin,1000,2000);

  delay(1000);
  ESC_BR.write(10);
  delay(500);
  ESC_BR.write(0);
  delay(500);
  ESC_BR.write(10);
  delay(500);
  ESC_BR.write(0);
  delay(500);
  ESC_BR.write(10);
  delay(500);
  ESC_BR.write(0);
  delay(500);
  
  
}

void loop() {
  ESC_BR.write(15);
  delay(1000); 
  ESC_BR.write(0);
  delay(5000);
}