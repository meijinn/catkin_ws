#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int counter = 0;
int switchas;
int switchals;
int pulse = 1156;
int f_in;

#define switchA 6
#define switchB 7

#define FMIN 50
#define FMAX 240

int esc_pin = 0;
int servo_pin = 1;

void setup(){
  pinMode(switchA,INPUT);
  pinMode(switchB,INPUT);
  Serial.begin(9600);
  switchals = digitalRead(switchA);
  pwm.begin();         // 初期設定
  pwm.setPWMFreq(56.94);
}

void loop(){
  switchas = digitalRead(switchA);
  
  if(switchas != switchals){
    if(digitalRead(switchB)!= switchas){
      counter ++;
    } else{
      counter --;
    }
    f_in = map(counter, 0, 400, FMIN, FMAX);
    pwm.setPWMFreq(f_in);  // PWM周期を56.94Hzに設定(57.67Hz) → プロポの実測は57.50Hz 
    Serial.print("Frequency: ");
    Serial.println(f_in);
    pwm.writeMicroseconds(esc_pin, pulse);
  }
  switchals = switchas;
}
