#include <Wire.h>                     // ライブラリのインクルード
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // PCA9685のI2Cアドレスを指定

int pulse;

#define SERVOMIN 1100    // 最小パルス幅(μs)
#define SERVOMAX 1900   // 最大パルス幅(μs)
#define SERVONEU 1500

int Servo_pin = 0;      // サーボ接続ピンを0番に

void setup() {
  pwm.begin();         // 初期設定
  pwm.setPWMFreq(57.67);  // PWM周期を50Hzに設定
  delay(1000);
}

void loop() {

    pulse = map(pulse, 0, 400, SERVONEU, SERVOMAX);
    Serial.print("Pulse: ");
    Serial.println(pulse);
    pwm.writeMicroseconds(Servo_pin, pulse);        // サーボを動作させる
    delay(1000);
}
