#include <ros.h>
#include <std_msgs/Float32.h>
#include <DHT.h>

#define DHTPIN 3
#define DHTTYPE 11

DHT dht (DHTPIN, DHTTYPE);

ros::NodeHandle nh;

std_msgs::Float32 temp_msg;
ros::Publisher chatter("temp", &temp_msg);

void setup(){
  nh.initNode();
  nh.advertise(chatter);
  dht.begin();
}

void loop(){
  delay(500);

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if(isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  temp_msg.data = t;
  chatter.publish( &temp_msg );

  nh.spinOnce();
  delay(1);
}
