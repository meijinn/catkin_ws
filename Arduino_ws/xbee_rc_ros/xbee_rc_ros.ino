
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>

Servo servo;
Servo speedcontroller;

//int TRIG = 3;
//int ECHO = 2;
//const int BRAKE = 93;


//double duration = 0;
//double distance = 0;
//double speed_of_sound = 331.5 + 0.6 * 18; // 25℃の気温の想定

ros::NodeHandle nh;

void servoCb( const std_msgs::UInt8MultiArray& cmd_msg) {

  if(cmd_msg.data[1] == 0){
    cmd_msg.data[1] = 93;
  }
  if(cmd_msg.data[1] == 0 && cmd_msg.data[0] == 0){
    cmd_msg.data[0] = 90;
    cmd_msg.data[1] = 93;
  }
  
//  float dist = getRange_Ultrasound();
//  int border = 30;
//
//  if(cmd_msg.data[1] <= 90){
//    border = 40;
//  }
//  else if(cmd_msg.data[1] <= 88){
//    border = 60;
//  }
//  
//  if(dist < border){
//    cmd_msg.data[1] += 6;
//    if(cmd_msg.data[1] >= 103 && dist <= 30){
//      cmd_msg.data[1] -= 6;
//    }
//  }

  
  servo.write(cmd_msg.data[0]);
  speedcontroller.write(cmd_msg.data[1]); 
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub("servo", servoCb);

//float getRange_Ultrasound()
//{
//  pinMode(TRIG, OUTPUT);
//  digitalWrite(TRIG, LOW);
//  delayMicroseconds(2);
//  digitalWrite(TRIG, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(TRIG, LOW);
//  pinMode(ECHO, INPUT);
//  duration = pulseIn(ECHO, HIGH);
//  
//  if (duration > 0) {
//    duration = duration / 2; // 往路にかかった時間
//    distance = duration * speed_of_sound * 100 / 1000000;
//    return distance;
//  }
//}

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(10);
  speedcontroller.attach(6);
}


void loop(){
  nh.spinOnce();
  delay(1);
}
