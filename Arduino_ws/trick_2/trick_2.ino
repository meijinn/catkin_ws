#include <Servo.h>

#define SERVO_PIN 10
int TRIG = 3;
int ECHO = 2;
int count = 0;


const int servoDO = 0;
const int servoNEW = 120;

double duration = 0;
double distance = 0;
double speed_of_sound = 331.5 + 0.6 * 20;


Servo trick_servo;


void setup(){
  trick_servo.attach(SERVO_PIN);
}


float getRange_Ultrasound()
{
  pinMode(TRIG, OUTPUT);
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  pinMode(ECHO, INPUT);
  duration = pulseIn(ECHO, HIGH);
  
  if (duration > 0) {
    duration = duration / 2; // 往路にかかった時間
    distance = duration * speed_of_sound * 100 / 1000000;
    return distance;
  }
}

void loop(){
  float dist = getRange_Ultrasound();
  int border = 10;
  
    
  if(dist < border){
    count = 1;
  }

  if(count == 1){
    delay(200);
    trick_servo.write(servoDO);
    delay(2000);
    trick_servo.write(servoNEW);
    count = 0;   
  }else{
    trick_servo.write(servoNEW);
  }
}
