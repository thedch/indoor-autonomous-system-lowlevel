#include "Motors.h"
#include "PID_velocity.h"
#include "IMU.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <Encoder.h>
#include <Arduino.h>

// Motor Pin Defines
#define RIGHT_PWM_PIN 2
#define RIGHT_MOTOR_EN1 24
#define RIGHT_MOTOR_EN2 25
#define LEFT_PWM_PIN 23
#define LEFT_MOTOR_EN1 21
#define LEFT_MOTOR_EN2 22
#define LEFT_ENCODER_PIN1 34
#define LEFT_ENCODER_PIN2 33
#define RIGHT_ENCODER_PIN1 31
#define RIGHT_ENCODER_PIN2 32
#define BAUD_RATE 9600

//PID Constants
#define KP 600
#define KI 200
#define KD 0
#define TIMEOUT_TICKS 40

#define ENCODER_RATE 7 // Milliseconds
#define PID_RATE ENCODER_RATE*2 // Milliseconds

// MOTOR FUNCTIONS & VARIABLES
Encoder lmotor_encoder(LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2);
Encoder rmotor_encoder(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2);
PID_velocity l_pid(LEFT_PWM_PIN, LEFT_MOTOR_EN1, LEFT_MOTOR_EN2, KD, KP, KI, TIMEOUT_TICKS);  // PID controller that creates own motor object
PID_velocity r_pid(RIGHT_PWM_PIN, RIGHT_MOTOR_EN1, RIGHT_MOTOR_EN2, KD, KP, KI, TIMEOUT_TICKS);
IntervalTimer encoder_timer; // interrupt to publish encoder values at 10hz
IntervalTimer PID_timer; // interrupt to check PID loop 
int16_t lenc_val = 0; // initialize encoder values
int16_t renc_val = 0;

//IMU
//IMU robot_imu;

// ROS FUNCTIONS & VARIABLES
ros::NodeHandle nh;
std_msgs::Int16 lwheel_msg, rwheel_msg;
//sensor_msgs::Imu IMU_msg;
//sensor_msgs::MagneticField MF_msg;
ros::Publisher lwheel("lwheel", &lwheel_msg);
ros::Publisher rwheel("rwheel", &rwheel_msg);
//ros::Publisher imu_data("imu_data", &IMU_msg);
//ros::Publisher compass("compass", &MF_msg);
void ROS_publisher();
void run_PID();

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

std_msgs::Float32 l_wheel_target;
std_msgs::Float32 r_wheel_target;

void setup() {
  Serial.begin(BAUD_RATE);
  // Encoder Interrupt set up
  encoder_timer.begin(ROS_publisher, ENCODER_RATE*1000); // Convert to microseconds
  PID_timer.begin(run_PID, PID_RATE*1000); // Convert to microseconds
  
  // ROS Node Setup
  nh.initNode();
  nh.advertise(lwheel);
  nh.advertise(rwheel);
//  nh.advertise(imu_data);
//  nh.advertise(compass);
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

void run_PID() {  
  r_wheel_target.data = 0.3;
  l_wheel_target.data = 0.3; // TODO: These should get set in a callback, not hardcoded
  r_pid.pid_spin(r_wheel_target);
  l_pid.pid_spin(l_wheel_target);
}

void ROS_publisher() {
  // Send the odom to the Pi for the nav stack
  lwheel_msg.data = (lmotor_encoder.read() / 4);
  rwheel_msg.data = (rmotor_encoder.read() / 4);
  lwheel.publish(&lwheel_msg);  
  rwheel.publish(&rwheel_msg);
  //Publish IMU data
  //IMU_msg = robot_imu.read_IMUmsg_data();
  //imu_data.publish(&IMU_msg);
  //MF_msg = robot_imu.read_compass();
//  compass.publish(&MF_msg);
  // Update the PID controller with the current odom
  l_pid.cumulative_enc_val(lwheel_msg.data);
  r_pid.cumulative_enc_val(rwheel_msg.data);
}

void lmotor_callback(const std_msgs::Float32& msg) {
  l_pid.test_motor_control(msg);
}

void rmotor_callback(const std_msgs::Float32& msg) {
  r_pid.test_motor_control(msg);
}

void lwheel_vtarget_callback(const std_msgs::Float32& msg) {
  l_pid.pid_spin(msg);
  // TODO: Update a global command variable here, and let the interrupt call the PID loop that references the command variable 
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
