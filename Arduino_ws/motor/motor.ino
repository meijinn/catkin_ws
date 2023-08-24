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
const double serA = -1104.25;
const double serB = 1499.7;
const double escA = -570;
const double escB = 1567;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
/***********************************************************************
 * Global variables
 **********************************************************************/

int esc_pulse = 0;
int servo_pulse = 0;

//wheel_rad is the wheel radius ,wheelbase is separation
double wheel_rad = 0.06;
double wheelbase = 0.257;

double speed_ang = 0.0;
double speed_lin = 0.0;

/***********************************************************************
 * ROS CallBack
 **********************************************************************/
void messageCb( const geometry_msgs::Twist& msg){
  // steering_angle [rad]?
  // speed_lin [m/s]
  if(msg.linear.x == 0.0){
    speed_ang = 0.0;
  }
  else{
    speed_ang = atan(wheelbase*msg.angular.z/msg.linear.x);
  }
  speed_lin = msg.linear.x;
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
  ESC_Write(speed_lin);
  Servo_Write(speed_ang);
  nh.spinOnce();
  delay(1);
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

void ESC_Write(int speed_lin){
  /* esc_pulse to signal calculation */
  esc_pulse = (int)(escA*speed_lin+escB);
  pwm.writeMicroseconds(ESC_PIN, esc_pulse);
}

void Servo_Write(int speed_ang){
  /* steering adjast */
  if(speed_ang < -0.4)
    speed_ang = -0.435;
  if(speed_ang > 0.4)
    speed_ang = 0.416;
  
  /* servo_pulse to signal calculation */
  servo_pulse = (int)(serA*speed_ang+serB);
  pwm.writeMicroseconds(SERVO_PIN, servo_pulse);
}
