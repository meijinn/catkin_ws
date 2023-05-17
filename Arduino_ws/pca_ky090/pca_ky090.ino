#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int counter = 0;
int switchas;
int switchals;
int pulse;

#define switchA 6
#define switchB 7
#define PULSEMIN 1100 //正転
#define PULSEMAX 1900 //逆転
#define PULSENEU 1500 //静止時

int esc_pin = 0;
int servo_pin = 1;

void setup(){
  pinMode(switchA,INPUT);
  pinMode(switchB,INPUT);
  Serial.begin(9600);
  switchals = digitalRead(switchA);
  pwm.begin();         // 初期設定
  pwm.setPWMFreq(56.94);  // PWM周期を56.94Hzに設定(57.67Hz) → プロポの実測は57.50Hz 
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
    pwm.writeMicroseconds(esc_pin, 1520);
    pwm.writeMicroseconds(servo_pin, pulse);
  }
  switchals = switchas;
}
