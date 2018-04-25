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
//#define LEFT_MOTOR_EN1 25
//#define LEFT_MOTOR_EN2 24
#define RIGHT_PWM_PIN 23
//#define RIGHT_MOTOR_EN1 22
//#define RIGHT_MOTOR_EN2 21
#define RIGHT_MOTOR_EN1 21
#define RIGHT_MOTOR_EN2 22
#define LEFT_ENCODER_PIN1 32
#define LEFT_ENCODER_PIN2 33
#define RIGHT_ENCODER_PIN1 34
#define RIGHT_ENCODER_PIN2 31
#define BAUD_RATE 9600

// MOTOR FUNCTIONS & VARIABLES
Motors right_motor(RIGHT_PWM_PIN, RIGHT_MOTOR_EN1, RIGHT_MOTOR_EN2);
Motors left_motor(LEFT_PWM_PIN, LEFT_MOTOR_EN1, LEFT_MOTOR_EN2);
Encoder lmotor_encoder(LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2);
Encoder rmotor_encoder(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2);
//PID_velocity l_pid(LEFT_PWM_PIN, LEFT_MOTOR_EN1, LEFT_MOTOR_EN2);  //PID controller that creates own motor object
//PID_velocity r_pid(RIGHT_PWM_PIN, RIGHT_MOTOR_EN1, RIGHT_MOTOR_EN2);
PID_velocity l_pid;
PID_velocity r_pid;
IntervalTimer encoder_timer; //interrupt to publish encoder values at 10hz
int16_t lenc_val = 0; // initialize encoder values
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
  //Encoder Interrupt set up
  encoder_timer.begin(ROS_encoder_publisher,100000);
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
  l_pid.cumulative_enc_val(lwheel_msg.data);
  r_pid.cumulative_enc_val(rwheel_msg.data);
}

void lmotor_callback(const std_msgs::Float32& msg) {
  left_motor.motor_cmd(msg);
}

void rmotor_callback(const std_msgs::Float32& msg) {
  right_motor.motor_cmd(msg);
}

void lwheel_vtarget_callback(const std_msgs::Float32& msg) {
  l_pid.pid_spin(msg);
}

void rwheel_vtarget_callback(const std_msgs::Float32& msg) {
  r_pid.pid_spin(msg);
}

void encoder_reset_callback(const std_msgs::Empty& reset_msg) {
  lenc_val = 0;
  renc_val = 0;
  lmotor_encoder.write(0);
  rmotor_encoder.write(0);
}
