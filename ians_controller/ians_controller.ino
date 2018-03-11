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

//MOTOR FUNCTIONS & VARIABLES 
Motors motors(RIGHT_PWM_PIN,RIGHT_MOTOR_EN1,RIGHT_MOTOR_EN2,LEFT_PWM_PIN,LEFT_MOTOR_EN1,LEFT_MOTOR_EN2);
Encoder leftEnc(LEFT_ENCODER_PIN1,LEFT_ENCODER_PIN2);
Encoder rightEnc(RIGHT_ENCODER_PIN1,RIGHT_ENCODER_PIN2);
int16_t lencVal = 0;
int16_t rencVal = 0;
std_msgs::Int16 test1;
std_msgs::Int16 test2;

//ROS FUNCTIONS & VARIABLES
ros::NodeHandle nh;
std_msgs::Int16 lwheel_msg, rwheel_msg;
ros::Publisher lwheel("lwheel", &lwheel_msg);
ros::Publisher rwheel("rwheel", &rwheel_msg);
void rosEncoderPublisher();

void messageCb( const std_msgs::Int16& msg){
  digitalWrite(13, HIGH-digitalRead(13));
}
ros::Subscriber<std_msgs::Int16> motor_sub("lmotor", &messageCb);

 
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(lwheel);
  nh.advertise(rwheel);
  nh.subscribe(motor_sub);
}


void loop() {
  motors.leftMotorForward(255);
  motors.rightMotorForward(100);
  delay(1000);
  motors.leftMotorReverse(100);
  motors.rightMotorReverse(255);
  delay(1000);
//  nh.spinOnce();
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

