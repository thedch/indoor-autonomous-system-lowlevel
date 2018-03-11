#include "Motors.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <Encoder.h>

// Motor Pin Defines
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
#define BAUD_RATE 9600

// MOTOR FUNCTIONS & VARIABLES 
Motors motors(RIGHT_PWM_PIN,RIGHT_MOTOR_EN1,RIGHT_MOTOR_EN2,LEFT_PWM_PIN,LEFT_MOTOR_EN1,LEFT_MOTOR_EN2);
Encoder leftEnc(LEFT_ENCODER_PIN1,LEFT_ENCODER_PIN2);
Encoder rightEnc(RIGHT_ENCODER_PIN1,RIGHT_ENCODER_PIN2);
int16_t lencVal = 0;
int16_t rencVal = 0;
std_msgs::Int16 test1;
std_msgs::Int16 test2;
int motorGo = 0;

// ROS FUNCTIONS & VARIABLES
ros::NodeHandle nh;
std_msgs::Int16 lwheel_msg, rwheel_msg;
ros::Publisher lwheel("lwheel", &lwheel_msg);
ros::Publisher rwheel("rwheel", &rwheel_msg);
void rosEncoderPublisher();

void lmotorSub( const std_msgs::Float32& msg){
  if(msg.data > 0){
    motors.leftMotorForward(msg);
  }
  else if(msg.data < 0){
    std_msgs::Float32 temp;
    temp.data = abs(msg.data);
    motors.leftMotorReverse(temp);
  }
  // TODO: Add motors_off()
}

void rmotorSub( const std_msgs::Float32& msg){
  if(msg.data > 0){
    motors.rightMotorForward(msg);
  }
  else if(msg.data < 0){
    std_msgs::Float32 temp;
    temp.data = abs(msg.data);
    motors.rightMotorReverse(temp);
  }
}

ros::Subscriber<std_msgs::Float32> lmotor_sub("lmotor", &lmotorSub);
ros::Subscriber<std_msgs::Float32> rmotor_sub("rmotor", &rmotorSub);
 
void setup() {
  Serial.begin(BAUD_RATE);
  // ROS Node Setup
  nh.initNode();
  nh.advertise(lwheel);
  nh.advertise(rwheel);
  nh.subscribe(lmotor_sub);
  nh.subscribe(rmotor_sub);
}


void loop() {
  // Sit and spin and wait for message publications from the Pi
  nh.spinOnce();
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
