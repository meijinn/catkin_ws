#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/***********************************************************************
 * Macro
 **********************************************************************/
#define ESC_PIN 0    // ESC   pin asign
#define SERVO_PIN 1  // Servo pin asign

const double ESC_NEUT = 1582;//1598.2;//1582;//1577.9;
const int SER_NEUT = 1540;
const double ESC_FORWARD_PULSE = 1512;
const double ESC_BACK_PULSE = 1624;
const double VEL_FORWARD = 0.05;         //  Navigation Forward Velocity
const double VEL_BACK = -0.03; //-0.015; //  Navigation Back    Velocity

const double TH_RIGHT_LIMIT = -0.35;   //  Navigation Theta Right Limit
const double TH_LEFT_LIMIT = 0.35;     //  Navigation Theta Left  Limit

const double serA = -1104.25;         //  Nothing to do Because it has optimized by taken the average.
const double serB = 1499.7;           //  Same as above.

/* Pulse Calculation Multiplier */ 
const double escA = (ESC_FORWARD_PULSE - ESC_BACK_PULSE)/(VEL_FORWARD-VEL_BACK);
const double escB = (VEL_FORWARD*ESC_BACK_PULSE-VEL_BACK*ESC_FORWARD_PULSE)/(VEL_FORWARD-VEL_BACK);

//const double escA = -555;
//const double escB = 1568.5;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/***********************************************************************
 * Global variables
 **********************************************************************/
int esc_pulse, servo_pulse = 0;

// wheel_rad is the wheel radius ,wheelbase is separation
double wheel_rad = 0.06;
double wheelbase = 0.257;

// speed_angle [rad/s] speed_lin [m/s]
double speed_ang = 0.0;
double speed_lin = 0.0;

/***********************************************************************
 * ROS CallBack
 **********************************************************************/
void messageCb( const geometry_msgs::Twist& msg){
  // steering_angle [rad]
  // speed_lin [m/s]
  if(msg.linear.x == 0.0){
    speed_ang = msg.angular.z;
  }
  else{
    speed_ang = atan(wheelbase*msg.angular.z/msg.linear.x);
  }
  speed_lin = msg.linear.x;
}

//  ROS
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

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
    Wire.setClock(400000);
    nh.getHardware()->setBaud(115200);
}

void ESC_Write(double speed_lin){
  /* esc_pulse to signal calculation */
  if(speed_lin > 0 && speed_lin < VEL_FORWARD)
    speed_lin = VEL_FORWARD;
  if(speed_lin < 0 && speed_lin > VEL_BACK)
    speed_lin = VEL_BACK;
  esc_pulse = (int)(escA*speed_lin+escB);
  
  if(speed_lin < 0){
    pwm.writeMicroseconds(ESC_PIN, (int)(ESC_NEUT+40));//35
    pwm.writeMicroseconds(ESC_PIN, esc_pulse);
  }
  else{
    pwm.writeMicroseconds(ESC_PIN, esc_pulse);
  }
}

void Servo_Write(double speed_ang){
  /* steering adjast */
  if(speed_ang < TH_RIGHT_LIMIT)
    speed_ang = TH_RIGHT_LIMIT;//-0.035;
  if(speed_ang > TH_LEFT_LIMIT)
    speed_ang = TH_LEFT_LIMIT;//+0.016;
  
  /* servo_pulse to signal calculation */
  servo_pulse = (int)(serA*speed_ang+serB);
  pwm.writeMicroseconds(SERVO_PIN, servo_pulse);
}
