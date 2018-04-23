#include "Motors.h"
#include "PID_velocity.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <Encoder.h>
#include <Arduino.h>

// Motor Pin Defines
#define LEFT_PWM_PIN 2
#define LEFT_MOTOR_EN1 24
#define LEFT_MOTOR_EN2 25
#define RIGHT_PWM_PIN 23
#define RIGHT_MOTOR_EN1 22
#define RIGHT_MOTOR_EN2 21
#define LEFT_ENCODER_PIN1 32
#define LEFT_ENCODER_PIN2 33
#define RIGHT_ENCODER_PIN1 34
#define RIGHT_ENCODER_PIN2 31
#define BAUD_RATE 9600

// MOTOR FUNCTIONS & VARIABLES
Motors motors(RIGHT_PWM_PIN, RIGHT_MOTOR_EN1, RIGHT_MOTOR_EN2, LEFT_PWM_PIN, LEFT_MOTOR_EN1, LEFT_MOTOR_EN2);
Encoder lmotor_encoder(LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2);
Encoder rmotor_encoder(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2);
PID_velocity l_pid;
PID_velocity r_pid;
int16_t lenc_val = 0; // TODO: What is this for?
int16_t renc_val = 0;

// ROS FUNCTIONS & VARIABLES
ros::NodeHandle nh;
std_msgs::Int16 lwheel_msg, rwheel_msg;
ros::Publisher lwheel("lwheel", &lwheel_msg);
ros::Publisher rwheel("rwheel", &rwheel_msg);
void ROS_encoder_publisher();

// Callback headers to be used when a ROS topic publish is received
void lmotor_callback(const std_msgs::Float32& msg);
void rmotor_callback(const std_msgs::Float32& msg);
void lwheel_vtarget_callback(const std_msgs::Float32& msg);
void rwheel_vtarget_callback(const std_msgs::Float32& msg);
void encoder_reset_callback(const std_msgs::Empty& reset_msg);

// Subscribers to ROS topics
ros::Subscriber<std_msgs::Float32> lmotor_sub("lmotor", &lmotor_callback);
ros::Subscriber<std_msgs::Float32> rmotor_sub("rmotor", &rmotor_callback);
ros::Subscriber<std_msgs::Float32> lwheel_vtarget_sub("lwheel_vtarget", &lwheel_vtarget_callback);
ros::Subscriber<std_msgs::Float32> rwheel_vtarget_sub("rwheel_vtarget", &rwheel_vtarget_callback);
ros::Subscriber<std_msgs::Empty> reset_encoder_sub("reset_encoders", &encoder_reset_callback);

void setup() {
  Serial.begin(BAUD_RATE);
  // ROS Node Setup
  nh.initNode();
  nh.advertise(lwheel);
  nh.advertise(rwheel);
  nh.subscribe(lmotor_sub);
  nh.subscribe(rmotor_sub);
  nh.subscribe(lwheel_vtarget_sub);
  nh.subscribe(rwheel_vtarget_sub);
  nh.subscribe(reset_encoder_sub);
}

// Main Loop
void loop() {
  ROS_encoder_publisher();
  // Sit and spin and wait for message publications from the Pi
  nh.spinOnce();
}

void ROS_encoder_publisher() {
  // Send the odom to the Pi for the nav stack
  lwheel_msg.data = (lmotor_encoder.read() / 4);
  rwheel_msg.data = (rmotor_encoder.read() / 4);
  lwheel.publish(&lwheel_msg);
  rwheel.publish(&rwheel_msg);
  // Update the PID controller with the current odom
  l_pid.wheelCallback(lwheel_msg.data);
  r_pid.wheelCallback(rwheel_msg.data);

  delay(100); // TODO: What is this?
}

void lmotor_callback(const std_msgs::Float32& msg) {
  if (msg.data > 0) {
    motors.left_motor_forward(msg);
  } else if (msg.data < 0) {
    std_msgs::Float32 temp;
    temp.data = abs(msg.data);
    motors.left_motor_reverse(temp);
  } else {
    motors.left_motor_brake();
  }
}

void rmotor_callback(const std_msgs::Float32& msg) {
  if (msg.data > 0) {
    motors.right_motor_forward(msg);
  } else if (msg.data < 0) {
    std_msgs::Float32 temp;
    temp.data = abs(msg.data);
    motors.right_motor_reverse(temp);
  } else {
    motors.right_motor_brake();
  }
}

void lwheel_vtarget_callback(const std_msgs::Float32& msg) {
  l_pid.pid_target = msg.data;
  l_pid.calc_velocity();
  l_pid.do_pid();
  l_pid.pid_spin();
}

void rwheel_vtarget_callback(const std_msgs::Float32& msg) {
  r_pid.pid_target = msg.data;
  r_pid.calc_velocity();
  r_pid.do_pid();
  // TODO: pid_spin calls do_pid, why are we calling both?
  r_pid.pid_spin();
}


void encoder_reset_callback(const std_msgs::Empty& reset_msg) {
  lenc_val = 0;
  renc_val = 0;
  lmotor_encoder.write(0);
  rmotor_encoder.write(0);
}
