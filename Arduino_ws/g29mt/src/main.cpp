#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


const int ESC_NEUT = 1520;
const int SER_NEUT = 1500;
int esc_pin = 0;
int servo_pin = 1;
int esc, servo;
ros::NodeHandle nh;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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

void setup(){
  pwm.begin();
  pwm.setPWMFreq(56.94);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  Wire.setClock(400000);
}


void loop(){
  nh.spinOnce();
  delay(1);
}
