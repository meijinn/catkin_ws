#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define sampling_time 1000000
#define PPR 1024
#define d 6
#define k 0.001885
#define A 0.6

const int ESC_NEUT = 1520;
const int SER_NEUT = 1500;
int esc_pin = 0;
int servo_pin = 1;
int esc, servo;
ros::NodeHandle nh;
std_msgs::Float32 encoder_msg;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

byte encoder_a = 2;                     //A相pin2
byte encoder_b = 3;                     //B相pin3
byte encoder_z = 4;                     //Z相pin4
long encoder_cnt=0;                     //エンコーダカウント用変数
int  encoder_rotate_cnt=0;              //エンコーダ回転数カウント用変数

float a, rpm, km, km_new, last_micros, current_micros;
long t, t_old, count, count_old;

void servoCb(const std_msgs::UInt16MultiArray& cmd_msg) {

  if(cmd_msg.data[1] == 0){
     cmd_msg.data[1] = ESC_NEUT;
  }
  if(cmd_msg.data[1] == 0 && cmd_msg.data[0] == 0){
    cmd_msg.data[0] = SER_NEUT;
    cmd_msg.data[1] = ESC_NEUT;
  }
  servo = cmd_msg.data[0];
  esc = cmd_msg.data[1];

  pwm.writeMicroseconds(servo_pin, servo);
  pwm.writeMicroseconds(esc_pin, esc);
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servoCb);
ros::Publisher chatter("speed",&encoder_msg);

void setup() {
  pinMode(encoder_a,INPUT_PULLUP);      //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(encoder_b,INPUT_PULLUP);      //B相用信号入力　内蔵プルアップ有効
  pinMode(encoder_z,INPUT_PULLUP);      //Z相用信号入力　内蔵プルアップ有効
  /* デジタル入力割り込みをA相に設定、立ち上がりエッジでencoder_pulse関数を呼び出し  */
  attachInterrupt(digitalPinToInterrupt(encoder_a), encoder_pulse, RISING);

  pwm.begin();
  pwm.setPWMFreq(56.94);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  Wire.setClock(400000);
}

void loop() {
  float vel = velocity_cal(encoder_cnt);
  encoder_msg.data = vel;
  chatter.publish(&encoder_msg);
  Serial.println(vel);
  nh.spinOnce();
  delay(1);
}

void encoder_pulse() {
  if(digitalRead(encoder_b)==0){
    encoder_cnt--;                    //エンコーダカウントをインクリメント
    if(digitalRead(encoder_z)==0){    //Z相がLOWのとき（原点にいるとき）
      encoder_rotate_cnt--;           //回転数をインクリメント
    }
  }else{
    encoder_cnt++;                    //エンコーダカウントをデクリメント
    if(digitalRead(encoder_z)==0){    //Z相がLOWのとき（原点にいるとき）
      encoder_rotate_cnt++;           //回転数をデクリメント
    }
  }
}

float velocity_cal(long cnt){
  
  //現在時間を取得
  current_micros = micros();
  count = encoder_cnt;                //カウントをエンコーダ関数から取得
  
  a = (float)(sampling_time*(count - count_old))/PPR;
  rpm = 60*a/(current_micros - last_micros)*6.4/6;
  km = d*k*rpm;
  km_new = A*km_new+(1-A)*km;
  count_old = count;
  last_micros = current_micros;
  return km_new;
}
