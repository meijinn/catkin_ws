#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int esc_pin = 0;
int servo_pin = 1;
ros::NodeHandle nh;

int esc, servo;

void servoCb(const std_msgs::UInt16MultiArray& cmd_msg) {

  if(cmd_msg.data[1] == 0){
     cmd_msg.data[1] = 1520;
  }
  if(cmd_msg.data[1] == 0 && cmd_msg.data[0] == 0){
    cmd_msg.data[0] = 1500;
    cmd_msg.data[1] = 1520;
  }
  // servo.write(cmd_msg.data[0]);
  // speedcontroller.write(cmd_msg.data[1]);
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
  if(!nh.connected()){
    pwm.writeMicroseconds(servo_pin, 1500);
    pwm.writeMicroseconds(esc_pin, 1520);
  }
  nh.spinOnce();
  delay(1);
}
