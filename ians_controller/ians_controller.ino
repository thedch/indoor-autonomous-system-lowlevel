#include "Motors.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <Encoder.h>

//Motor Pin Defines
#define RIGHT_PWM_PIN 2
#define RIGHT_MOTOR_EN1 24
#define RIGHT_MOTOR_EN2 25
#define LEFT_PWM_PIN 23
#define LEFT_MOTOR_EN1 22
#define LEFT_MOTOR_EN2 21
#define LEFT_ENCODER_PIN1 32
#define LEFT_ENCODER_PIN2 33
#define RIGHT_ENCODER_PIN1 31
#define RIGHT_ENCODER_PIN2 34

//ROS FUNCTIONS & VARIABLES
ros::NodeHandle nh;
std_msgs::Int16 lwheel_msg, rwheel_msg;
ros::Publisher lwheel("lwheel", &lwheel_msg);
ros::Publisher rwheel("rwheel", &rwheel_msg);
void rosEncoderPublisher();

//MOTOR FUNCTIONS & VARIABLES 
Motors motors(RIGHT_PWM_PIN,RIGHT_MOTOR_EN1,RIGHT_MOTOR_EN2,LEFT_PWM_PIN,LEFT_MOTOR_EN1,LEFT_MOTOR_EN2);
Encoder leftEnc(LEFT_ENCODER_PIN1,LEFT_ENCODER_PIN2);
Encoder rightEnc(RIGHT_ENCODER_PIN1,RIGHT_ENCODER_PIN2);
int16_t lencVal = 0;
int16_t rencVal = 0;

 
void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(lwheel);
  nh.advertise(rwheel);
}

void loop() {
//  motors.rightMotorForward(255);
//  motors.leftMotorReverse(150);
//  delay(1000);
//  motors.rightMotorReverse(150);
//  motors.leftMotorForward(255);
//  delay(1000);
  rosEncoderPublisher();
}


void rosEncoderPublisher(){
  lencVal = leftEnc.read();
  rencVal = rightEnc.read();
  lwheel_msg.data = lencVal/4;
  lwheel.publish(&lwheel_msg);
  rwheel_msg.data = rencVal/4;
  rwheel.publish(&rwheel_msg);
  nh.spinOnce();
  delay(100);
}

