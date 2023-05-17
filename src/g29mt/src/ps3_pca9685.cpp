#include <ros/ros.h>
#include <sensor_msgs/Joy.h>  
#include <std_msgs/UInt16MultiArray.h>
#include <unistd.h>
#include <string.h>


const int ESC_NEUT = 1520;
const int SER_NEUT = 1500;
const int BRAKE = 1762;
const int seven = 1510;
const int eight = 1506;
const int nine = 1502;
const int ten = 1498;
const int eleven = 1492;
const int twelve = 1484;
const int BACK  = 1624;


std_msgs::UInt16MultiArray servo;
int shift_count = 0;
int shift_state = 0;
int last_shift_state = 0;
int clutch;
char hantei[10];
float gas_in, brake_in;
float ster_in;
int limit = ESC_NEUT;
int throttle = ESC_NEUT;
int ster = SER_NEUT;

void joy_callback(const sensor_msgs::Joy &joy_msg){

  gas_in = joy_msg.axes[5];
  brake_in = joy_msg.axes[2];
  ster_in = joy_msg.axes[0];
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
  shift_state = joy_msg.buttons[0] - joy_msg.buttons[3];
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

  //ATモード
  if (joy_msg.buttons[2] == 1){
    shift_count = 6;
    clutch = joy_msg.axes[1];
  }
  if (joy_msg.buttons[1] == 1){
    shift_count = -1;
  }

  //ギア数によるESCパルスのlimitの場合分け
  switch (shift_count)
  {
  case 0:
    limit = ESC_NEUT;
    break;
  case 1:
    limit = seven;
    break;
  case 2:
    limit = eight;
    break;
  case 3:
    limit = nine;
    break;
  case 4:
    limit = ten;
    break;
  case 5:
    limit = eleven;
    break;
  case 6:
    limit = twelve;
    break;
  case -1:
    limit = BACK;
    break;
  default:
    break;
  }

  int a = (ESC_NEUT-limit)/2;
  int b = ESC_NEUT-a;

  //アクセルオン前進時    throttle = ESC_NEUT;    throttle = ESC_NEUT;
  if(shift_count > 0){
      throttle = gas_in*a+b;
  }

  //バックギア後退時
  if (shift_count < 0 && gas_in >= 0){
      throttle = limit;
  }

  //前進時ブレーキ
  if(brake_in < 0 && shift_count > 0){
    throttle = BRAKE;
  }

  if(brake_in < 0 && shift_count == 0){
    throttle = ESC_NEUT;
  }

  if(brake_in < 0.5 && shift_count < 0){
    throttle = ESC_NEUT;
  }

  ster = ster_in*(-400)+1500;
  servo.data[0] = ster;
  servo.data[1] = throttle;

  printf("\rSteering:%d ",ster);
  printf("Throttle:%d ",throttle);
  printf("Gear:%d ",shift_count);
  printf("clutch:%s    \r",hantei);
  fflush(stdout);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "joy_pub_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("joy", 10, joy_callback);

  ros::Publisher servo_pub = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1);
  ros::Rate loop_rate(100);
  // ros::Publisher controller_pub = nh.advertise<std_msgs::UInt8MultiArray>("controller", 1);
  // ros::Rate loop_rate(100);

  while (ros::ok())
  {
    servo.data.resize(2);
    //controller_pub.publish(controller);
    servo_pub.publish(servo);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}