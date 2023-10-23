#include <ros/ros.h>
#include <sensor_msgs/Joy.h>  
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <string.h>

#define VMAX 0.1

const double neut = 0.0;
const double low = 0.05;
const double second = 0.068;
const double third = 0.075;
const double fourth = 0.08;
const double top = 0.09;
const double high_top = VMAX;
const double back_vel = -VMAX/2;
double forward, back = 0.0;

geometry_msgs::Twist controller;

int shift_count = 0;
int shift_state = 0;
int last_shift_state = 0;
int clutch;
char hantei[10];
double gas_in, brake_in = 0.0;
double gas, brake = 0.0;
double vel = 0.0;
double crb1 = 1;
double crb2 = 2;

void joy_callback(const sensor_msgs::Joy &joy_msg){

  double vel;
  // 処理内容を記述 踏むと1になるからps3に-1をかける
  //shift_count = shift_count + joy_msg.buttons[4] - joy_msg.buttons[5];
  if(joy_msg.axes[1] > 0 && joy_msg.axes[1] < 0.5){
    strcpy(hantei, "Good");
  }else if (joy_msg.axes[1] > 0){
    strcpy(hantei, "OK");
  }else{
    strcpy(hantei, "NG");
  }
  
  //MT-パドルシフトの処理
  shift_state = joy_msg.buttons[4] - joy_msg.buttons[5];
    //チャタリング防止処理
  if(shift_state != last_shift_state){
    if(shift_state == 1){
      shift_count++;
    }
    if(shift_state == -1){
      shift_count--;
    }
  }
  last_shift_state = shift_state;

    //7速とか-2速とかがないように例外を外す
  if (shift_count > 6){
    shift_count = 6;
  }
  if (shift_count < -1){
    shift_count = -1;
  }

  //MT-Hパターンの処理
  for (int i = 13; i <= 17; i++){
    if (joy_msg.buttons[12] == 1 && joy_msg.axes[1] > 0 && joy_msg.axes[1] < 0.5){
      shift_count = 1;//lowギアの半クラッチの処理
    }else if(joy_msg.buttons[i] == 1 && joy_msg.axes[1] > 0){//クラッチが踏まれててかつHパターンの入力がある。
        shift_count = i-11;//second 2速 ~ overtop 6速までの処理
      }else if(joy_msg.buttons[18] == 1 && joy_msg.axes[1] > 0){
        shift_count = -1;//バックギアの処理
      }
    }
  //ニュートラルの処理
  if(joy_msg.buttons[12] + joy_msg.buttons[13] + joy_msg.buttons[14] + joy_msg.buttons[15]
      + joy_msg.buttons[16] + joy_msg.buttons[17] + joy_msg.buttons[18] == 0 && joy_msg.axes[1] > 0){
        shift_count = 0;
      }
    
  //ATモード
  if (joy_msg.buttons[2] == 1){
    shift_count = 6;
    clutch = joy_msg.axes[1];
  }
  if (joy_msg.buttons[0] == 1){
    shift_count = -1;
  }

  //ギア数によるモーターの回転数の場合分け
  switch (shift_count)
  {
  case 0:
    forward = neut; back = back_vel;
    break;
  case 1:
    if(joy_msg.axes[2] >= -1 && joy_msg.axes[2] <= 1){
      forward = low; back = back_vel;
    }
    break;
  case 2:
    forward = second; back = back_vel;
    break;
  case 3:
    forward = third; back = back_vel;
    break;
  case 4:
    forward = fourth; back = back_vel;
    break;
  case 5:
    forward = top; back = back_vel;
    break;
  case 6:
    forward = high_top; back = back_vel;
    break;
  case -1:
    forward = back_vel; back = -back_vel;
    break;
  default:
    break;
  }
  
  gas_in = (joy_msg.axes[2]+crb1)/crb2;
  brake_in = (joy_msg.axes[3]+crb1)/crb2;
  gas = forward*gas_in;
  brake = back*brake_in;
  
  vel = gas+brake;

  if (shift_count >= 0 && vel < 0){
    vel = 0.0;
  }
  if (shift_count < 0 && vel > 0){
    vel = 0.0;
  }
  if (vel > 0.1){
    vel = 0.1;
  }
  if (vel < -0.1){
    vel = -0.1;
  }

  controller.linear.x = vel;
  if(controller.linear.x < 0){
    controller.angular.z = -0.5*joy_msg.axes[0];
  }
  else{
    controller.angular.z = 0.5*joy_msg.axes[0];
  }
  controller.linear.z = (int)(joy_msg.buttons[1]);
  controller.linear.y = (int)(joy_msg.buttons[3]);

  controller.angular.y = shift_count;
  
  // printf("\rSteering:%lf ",controller.angular.z);
  // printf("Throttle:%lf ",controller.linear.x);
  // printf("Gear:%.0lf ",controller.linear.y);
  // printf("clutch:%s    \r",hantei);
  // fflush(stdout);
}

// void chatterCb(const std_msgs::Float32& encoder_msg){
//   ROS_INFO("speed: %f", encoder_msg.data);
// }

int main(int argc, char **argv){

  ros::init(argc, argv, "joy_pub_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("joy", 10, joy_callback);

  ros::Publisher controller_pub = nh.advertise<geometry_msgs::Twist>("controller", 10);
  ros::Rate loop_rate(100);

  while (ros::ok()){
    controller_pub.publish(controller);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}