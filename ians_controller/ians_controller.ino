#include "Motors.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <Encoder.h>

ros::NodeHandle nh;

std_msgs::Int16 lwheel_msg, rwheel_msg;
ros::Publisher lwheel("lwheel", &lwheel_msg);
ros::Publisher rwheel("rwheel", &rwheel_msg);

int16_t lencVal = 0;
int16_t rencVal = 0;

//Motors motors(32,33,31,34);
Encoder leftEnc(32,33);
Encoder rightEnc(31,34);
 
void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(lwheel);
  nh.advertise(rwheel);
}

void loop() {
  lencVal = leftEnc.read();
  rencVal = rightEnc.read();
  lwheel_msg.data = lencVal/4;
  lwheel.publish(&lwheel_msg);
  rwheel_msg.data = rencVal/4;
  rwheel.publish(&rwheel_msg);
  nh.spinOnce();
  delay(100);
}
