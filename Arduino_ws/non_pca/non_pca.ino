#include <Servo.h>

int counter = 0;
int switchas;
int switchals;
int pulse;

#define esc 5
#define ser 3
#define switchA 6
#define switchB 7
#define PULSEMIN 1100 //正転
#define PULSEMAX 1900 //逆転
#define PULSENEU 1500 //静止時

Servo speedcontroller;
Servo servo;

void setup(){
  pinMode(switchA,INPUT);
  pinMode(switchB,INPUT);
  Serial.begin(9600);
  switchals = digitalRead(switchA);
  speedcontroller.attach(esc);
  servo.attach(ser);
}

void loop(){
  switchas = digitalRead(switchA);
  
  if(switchas != switchals){
    if(digitalRead(switchB)!= switchas){
      counter ++;
    } else{
      counter --;
    }

    pulse = map(counter, 0, 400, PULSENEU, PULSEMIN);
    Serial.print("Pulse: ");
    Serial.println(pulse);
    speedcontroller.write(pulse);
    servo.write(pulse);
  }
  switchals = switchas;
}
