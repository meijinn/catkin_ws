#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/************************************************************
 * Macro
************************************************************/
#define ESC_PIN 0
#define SERVO_PIN 1

const int ESC_NEUT = 1530;
const int SER_NEUT = 1540;
const double serA = -1104.25;
const double serB = 1499.7;
const double escA = -570;
const double escB = 1567;

/************************************************************
 * Global variables
************************************************************/

int esc_pulse, servo_pulse = 0;
double wheel_rad = 0.06;
double wheelbase = 0.257;

double speed_ang = 0.0;
double speed_lin = 0.0;

// ROS
ros::NodeHandle nh;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/************************************************************
 * Function
************************************************************/
void messageCb(const geometry_msgs::Twist& msg) {

  servo_pulse = (int)(-940/0.8*msg.angular.z+1087);
  esc_pulse = (int)(-114/0.2*msg.linear.x+1567);
  
  pwm.writeMicroseconds(ESC_PIN, esc_pulse);
  pwm.writeMicroseconds(SERVO_PIN, servo_pulse);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

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
