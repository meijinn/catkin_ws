#include <Arduino.h>

byte encoder_a = 2;                     //A相pin2
byte encoder_b = 3;                     //B相pin3
byte encoder_z = 4;                     //Z相pin4
int  encoder_cnt=0;                     //エンコーダカウント用変数
int  encoder_rotate_cnt=0;              //エンコーダ回転数カウント用変数

void setup() {
  pinMode(encoder_a,INPUT_PULLUP);      //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(encoder_b,INPUT_PULLUP);      //B相用信号入力　内蔵プルアップ有効
  pinMode(encoder_z,INPUT_PULLUP);      //Z相用信号入力　内蔵プルアップ有効

  /* デジタル入力割り込みをA相に設定、立ち上がりエッジでencoder_pulse関数を呼び出し  */
  attachInterrupt(digitalPinToInterrupt(encoder_a), encoder_pulse, RISING);
  Serial.begin(9600);
}

void loop() {
  Serial.println(encoder_cnt);       //エンコーダカウントをPCに出力   
  delay(10);
}

void encoder_pulse() {
  if(digitalRead(encoder_b)==0){
    encoder_cnt++;                    //エンコーダカウントをインクリメント
    if(digitalRead(encoder_z)==0){    //Z相がLOWのとき（原点にいるとき）
      encoder_rotate_cnt++;           //回転数をインクリメント
    }
  }else{
    encoder_cnt--;                    //エンコーダカウントをデクリメント
    if(digitalRead(encoder_z)==0){    //Z相がLOWのとき（原点にいるとき）
      encoder_rotate_cnt--;           //回転数をデクリメント
    }
  }
}