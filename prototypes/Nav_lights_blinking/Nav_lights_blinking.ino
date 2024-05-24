int led_pin=2;
int led_pin2=4;
int led_pin3=7;
int led_pin4=8;


void setup() {
  pinMode(led_pin,OUTPUT);
  pinMode(led_pin2, OUTPUT);
  pinMode(led_pin3, OUTPUT);
  pinMode(led_pin4, OUTPUT);
}

void loop(){
digitalWrite(led_pin, HIGH);
digitalWrite(led_pin2, HIGH);
digitalWrite(led_pin3, HIGH);
digitalWrite(led_pin4,HIGH);
delay(1000);
digitalWrite(led_pin, LOW);
digitalWrite(led_pin2, LOW);
digitalWrite(led_pin3,LOW);
digitalWrite(led_pin4,LOW);
delay(500);
}

