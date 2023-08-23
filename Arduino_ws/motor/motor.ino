#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/***********************************************************************
 * Macro
 **********************************************************************/
#define ESC_PIN 0
#define SERVO_PIN 1

const int ESC_NEUT = 1530;
const int SER_NEUT = 1500;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
/***********************************************************************
 * Global variables
 **********************************************************************/

int esc_pulse = 0;
int servo_pulse = 0;

//wheel_rad is the wheel radius ,wheel_sep is separation
double wheel_rad = 0.06;
double wheel_sep = 0.32;

double speed_ang = 0.0;
double speed_lin = 0.0;

/***********************************************************************
 * Function
 **********************************************************************/
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  
  /* change calculation method to ackermann steering geometry */
  esc_pulse = (int)(speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  servo_pulse = (int)(speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

}

// ROS
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

/***********************************************************************
 * prottype
 **********************************************************************/
void Motors_init();
void ESC_Write(int ESC_Pulse_Width);
void Servo_Write(int Servo_Pulse_Width);

void setup(){
  Motors_init();
  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  ESC_Write(esc_pulse);
  Servo_Write(servo_pulse);
  nh.spinOnce();
}

/***********************************************************************
 * Function
 **********************************************************************/
void Motors_init(){
    pwm.begin();
    pwm.setPWMFreq(56.94);
    nh.getHardware()->setBaud(115200);
    Wire.setClock(400000);
}

void ESC_Write(int ESC_Pulse_Width){
    // esc_pulse to signal calculation
    pwm.writeMicroseconds(ESC_PIN, ESC_Pulse_Width);
}

void Servo_Write(int Servo_Pulse_Width){
    // servo_pulse to signal calculation
    pwm.writeMicroseconds(SERVO_PIN, Servo_Pulse_Width);
}