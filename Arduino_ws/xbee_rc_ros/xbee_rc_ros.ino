#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>


Servo servo;
Servo speedcontroller;
ros::NodeHandle nh;


void servoCb( const std_msgs::UInt8MultiArray& cmd_msg) {
  if(cmd_msg.data[1] == 0){
    cmd_msg.data[1] = 93;
  }
  if(cmd_msg.data[1] == 0 && cmd_msg.data[0] == 0){
    cmd_msg.data[0] = 90;
    cmd_msg.data[1] = 93;
  }
  servo.write(cmd_msg.data[0]);
  speedcontroller.write(cmd_msg.data[1]);
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub("servo", servoCb);


void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  servo.attach(10);
  speedcontroller.attach(6);
}


void loop(){
  if(!nh.connected()){
   servo.write(90);
   speedcontroller.write(93);
  }
  nh.spinOnce();
  delay(1);
}
