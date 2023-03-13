#include <ros/ros.h>
#include <sensor_msgs/Joy.h>  
#include <std_msgs/UInt8MultiArray.h>
#include <unistd.h>
#include <string.h>

std_msgs::UInt8MultiArray controller;

int shift_count = 0;
int shift_state = 0;
int last_shift_state = 0;
int clutch;
char hantei[10];
float gas_in, brake_in = 0;
int crb1 = 1;
int crb2 = 2;

int m,n,l,k = 0;
int i = 0;
int j = 1;

void joy_callback(const sensor_msgs::Joy &joy_msg){

  int throttle = 93;
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
  // if (joy_msg.buttons[12] == 1 && joy_msg.axes[1] > 0){
  //   shift_count = 1;
  // }else if(joy_msg.buttons[13] == 1 && joy_msg.axes[1] > 0){
  //   shift_count = 2;
  // }else if(joy_msg.buttons[14] && joy_msg.axes[1] > 0)

  //ギア数によるモーターの回転数の場合分け
  switch (shift_count)
  {
  case 0:
    m = 0; n = 0; l = 0;
    break;
  case 1:
    if(joy_msg.axes[2] >= -1 && joy_msg.axes[2] <= 1){
      m = -3; n = 94; l = 1; k = 4; i = 4;
    }
    break;
  case 2:
    m = -4; n = 94; l = 1; k = 4; i = 4;
    break;
  case 3:
    m = -5; n = 94; l = 1; k = 4; i = 4;
    break;
  case 4:
    m = -6; n = 94; l = 1; k = 4; i = 4;
    break;
  case 5:
    m = -7; n = 94; l = 1; k = 4; i = 4;
    break;
  case 6:
    m = -8; n = 94; l = 1; k = 4; i = 4;

    break;
  case -1:
    m =  9; n = 93; l = 1; k = -4; i = 4;
    //m =  1; n = 103; l = 1; k = -1; i = -1; j = 2;
    // if((joy_msg.axes[1] > 0 && joy_msg.axes[1] < 0.5) || (last_shift_state == 1) && (shift_count == -1)){
    //     m = 0; n = 0; l = 0;
    // }
    break;
  default:
    break;
  }
  gas_in = (joy_msg.axes[2]+crb1)/crb2;
  brake_in = joy_msg.axes[3];

  float gas = (gas_in*m+n)/l;
  float brake = (brake_in*k+i)/j;
  
  throttle = gas+brake;
  if (throttle > 103){
    throttle = 103;
  }
  
  //float gas = joy_msg.axes[2]*(-1)+93;//low
  //float gas = (joy_msg.axes[2]*-3+183)/2;//second
  //float gas = joy_msg.axes[2]*-3+92;//third
  //float gas = (joy_msg.axes[2]*-7+183)/2;//fourth
  //float gas = (joy_msg.axes[2]*-11+183)/2;//top
  //float gas = joy_msg.axes[2]*-6+93;//overtop
  //float brake = (joy_msg.axes[3]*21*-1+207)/2; //default
  //float brake = joy_msg.axes[3]*10+103;//low
  //float brake = joy_msg.axes[3]*10+103;//second
  //float brake = (joy_msg.axes[3]*21+203)/2;//third
  //float brake = joy_msg.axes[3]*11+102;//fourth
  //float brake = joy_msg.axes[3]*(11)+101;//top
  //float brake = (joy_msg.axes[3]*21+195)/2;//overtop
  controller.data[0] = ((joy_msg.axes[0]*(-1)*180)+180)/2;
  controller.data[1] = throttle;
  //ROS_INFO("steering:%d",steering);   // スティック0の状態を表示 (-1 ～ 1)
  //ROS_INFO("steering:%d",controller.data[0]);  // ボタン0の状態を表示 (0 or 1)
  //ROS_INFO("throttle:%d",controller.data[1]);
  //ROS_INFO("shift_count:%d",shift_count);
  printf("\rSteering:%d ",controller.data[0]);
  printf("Throttle:%d ",controller.data[1]);
  printf("Gear:%d ",shift_count);
  printf("clutch:%s    \r",hantei);
  fflush(stdout);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "joy_pub_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("joy", 10, joy_callback);

  ros::Publisher controller_pub = nh.advertise<std_msgs::UInt8MultiArray>("controller", 1);
  ros::Rate loop_rate(100);
  // ros::Publisher controller_pub = nh.advertise<std_msgs::UInt8MultiArray>("controller", 1);
  // ros::Rate loop_rate(100);

  while (ros::ok())
  {
    controller.data.resize(2);
    //controller_pub.publish(controller);
    controller_pub.publish(controller);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}