int x1_in = A0;
int y1_in= A1;
int x2_in = A2;
int y2_in = A3;
int joystick1x_val, joystick1y_val, joystick2x_val, joystick2y_val;

int joystick1_switch = 2;
int joystick2_switch = 3;


void setup() {
  pinMode(joystick1_switch, INPUT);
  pinMode(joystick2_switch, INPUT);
  Serial.begin(9600);
}

void loop() {
  joystick1x_val= analogRead(x1_in);
  joystick1y_val= analogRead(y1_in);
  joystick2x_val= analogRead(x2_in);
  joystick2y_val= analogRead(y2_in);

  Serial.print("X for joystick 1= ");
  Serial.println(joystick1x_val);
  Serial.print("Y for joystick 1= ");
  Serial.println(joystick1y_val);
  Serial.print("X for joystick 2= ");
  Serial.println(joystick2x_val);
  Serial.print("Y for joystick 2= ");
  Serial.println(joystick2y_val);
  delay(2000);
}
