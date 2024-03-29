#include <ros/ros.h>
#include <sensor_msgs/Joy.h>  
#include <std_msgs/UInt8MultiArray.h>
#include <unistd.h>
#include <string.h>

std_msgs::UInt8MultiArray servo;

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

  //ギア数によるモーターの回転数の場合分け
  switch (shift_count)
  {
  case 0:
    m = 0; n = 0; l = 0;
    break;
  case 1:
    // m = 3; n = 94; l = 1; k = -4; i = 4;
    m = 7; n = 87, l = 1; k = -4; i = 4; //80 15 km/h
    break;
  case 2:
    // m = 4; n = 94; l = 1; k = -4; i = 4; //90
    m = 27; n = 67; l = 1; k = -7; i = 7; //40 20 km/h
    break;
  case 3:
    // m = 5; n = 94; l = 1; k = -4; i = 4; //89
    //m = 12; n = 82; l = 1; k = -7; i = 7; //70
    m = 37; n = 57; l = 1; k = -7; i = 7; //20
    break;
  case 4:
    //m = 6; n = 94; l = 1; k = -4; i = 4; //88
    //m = 17; n = 77; l = 1; k = -7; i = 7; //60
    m = 42; n = 52; l = 1; k = -7; i = 7; //10
    break;
  case 5:
    //m = 7; n = 94; l = 1; k = -4; i = 4; //87
    // m = 22; n = 72; l = 1; k = -7; i = 7; //50
    m = 46; n = 48; l = 1; k = -7; i = 7; //2
    break;
  case 6:
    m = 6; n = 92; l = 1; k = -4; i = 4; //86
    break;
  case -1:
    m = -9; n = 103; l = 1; k = 2; i = -2;
    //m =  1; n = 103; l = 1; k = -1; i = -1; j = 2;
    // if((joy_msg.axes[1] > 0 && joy_msg.axes[1] < 0.5) || (last_shift_state == 1) && (shift_count == -1)){
    //     m = 0; n = 0; l = 0;
    // }
    break;
  default:
    break;
  }
  gas_in = joy_msg.axes[5];
  brake_in = joy_msg.axes[2];

  float gas = (gas_in*m+n)/l;
  float brake = (brake_in*k+i)/j;
  
  throttle = gas+brake;
  if (throttle > 103){
    throttle = 103;
  }

  servo.data[0] = ((joy_msg.axes[0]*(-1)*180)+180)/2;
  servo.data[1] = throttle;

  printf("\rSteering:%d ",servo.data[0]);
  printf("Throttle:%d ",servo.data[1]);
  printf("Gear:%d ",shift_count);
  printf("clutch:%s    \r",hantei);
  fflush(stdout);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "joy_pub_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("joy", 10, joy_callback);

  ros::Publisher servo_pub = nh.advertise<std_msgs::UInt8MultiArray>("servo", 1);
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