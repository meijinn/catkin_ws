
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

Servo servo;
ros::NodeHandle nh;

void servoCb( const std_msgs::UInt16MultiArray& cmd_msg) {
  servo.write(cmd_msg.data[0]);
  analogWrite(11, cmd_msg.data[1]);
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servoCb);

void setup(){
  pinMode(11, OUTPUT);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  servo.attach(9);
}


void loop(){
  nh.spinOnce();
  delay(1);
}
