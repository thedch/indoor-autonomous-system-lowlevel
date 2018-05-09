#include "motors.h"
#include "PID_velocity.h"
#include "IMU.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <Encoder.h>
#include <Arduino.h>
#include <tuple>

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
#define BAUD_RATE 115200

// PID params
#define KP 600
#define KI 200
#define KD 0
#define TIMEOUT_TICKS 40
#define L_TICKS_PER_METER 1527.88
#define R_TICKS_PER_METER 1527.88

#define ENCODER_RATE 7 // Milliseconds
#define PID_RATE ENCODER_RATE*2 // Milliseconds

// MOTOR FUNCTIONS & VARIABLES
Encoder lmotor_encoder(LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2);
Encoder rmotor_encoder(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2);

// PID controller that creates own motor object
PID_velocity l_pid(LEFT_PWM_PIN, LEFT_MOTOR_EN1, LEFT_MOTOR_EN2, KD, KP, KI, L_TICKS_PER_METER);
PID_velocity r_pid(RIGHT_PWM_PIN, RIGHT_MOTOR_EN1, RIGHT_MOTOR_EN2, KD, KP, KI, R_TICKS_PER_METER);

IntervalTimer encoder_timer; // interrupt to publish encoder values
IntervalTimer PID_timer; // interrupt to check PID loop

int l_halt_highlevel = 0; // motor stalled flags
int r_halt_highlevel = 0;

//IMU
IMU robot_imu;

// ROS FUNCTIONS & VARIABLES
ros::NodeHandle nh;
std_msgs::Int16 lwheel_msg, rwheel_msg;
std_msgs::Float32 quatz_msg, quatw_msg;
ros::Publisher lwheel("lwheel", &lwheel_msg);
ros::Publisher rwheel("rwheel", &rwheel_msg);
ros::Publisher quatz("quat_z", &quatz_msg);
ros::Publisher quatw("quat_w", &quatw_msg);
void ROS_publisher();
void run_PID();

// Callback headers to be used when a ROS topic publish is received
void lwheel_vtarget_callback(const std_msgs::Float32& msg);
void rwheel_vtarget_callback(const std_msgs::Float32& msg);
void encoder_reset_callback(const std_msgs::Empty& reset_msg);

// Subscribers to ROS topics
ros::Subscriber<std_msgs::Float32> lwheel_vtarget_sub("lwheel_vtarget", &lwheel_vtarget_callback);
ros::Subscriber<std_msgs::Float32> rwheel_vtarget_sub("rwheel_vtarget", &rwheel_vtarget_callback);
ros::Subscriber<std_msgs::Empty> reset_encoder_sub("reset_encoders", &encoder_reset_callback);

void setup() {
    Serial.begin(BAUD_RATE);
    // Interrupt set up
    encoder_timer.begin(ROS_publisher, ENCODER_RATE * 1000); // Convert to microseconds
    PID_timer.begin(run_PID, PID_RATE * 1000); // Convert to microseconds

    // ROS Node Setup
    nh.initNode();
    nh.advertise(lwheel);
    nh.advertise(rwheel);
    nh.advertise(quatz);
    nh.advertise(quatw);
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
    r_pid.pid_spin();
    l_pid.pid_spin();
}

void ROS_publisher() {
    // Send the odom to the Pi for the nav stack
    lwheel_msg.data = (lmotor_encoder.read() / 4);
    rwheel_msg.data = (rmotor_encoder.read() / 4);

    // check for motor stall
    l_halt_highlevel = l_pid.check_motor_stall(lwheel_msg.data);
    r_halt_highlevel = r_pid.check_motor_stall(rwheel_msg.data);

    static int encoder_counter = 0;
    encoder_counter++;
    if (encoder_counter > 14) {
        lwheel.publish(&lwheel_msg);
        rwheel.publish(&rwheel_msg);
        encoder_counter = 0;
        // Publish IMU data
        auto quat_data = robot_imu.read_IMUmsg_data();
        quatw_msg = std::get<0>(quat_data);
        quatz_msg = std::get<1>(quat_data);
        quatw.publish(&quatw_msg);
        quatz.publish(&quatz_msg);
    }

    // Update the PID controller with the current odom
    l_pid.cumulative_enc_val(lwheel_msg.data);
    r_pid.cumulative_enc_val(rwheel_msg.data);
}

void lwheel_vtarget_callback(const std_msgs::Float32& msg) {
    if ((r_halt_highlevel == 1) || (l_halt_highlevel == 1)) { // Check for either left or right motor stall to stop left motor
        l_pid.pid_target = 0;
    } else if ((r_halt_highlevel == 2) || (l_halt_highlevel == 2)) {
        l_pid.pid_target = -0.3;
    } else {
        l_pid.pid_target = msg.data;
    }
}

void rwheel_vtarget_callback(const std_msgs::Float32& msg) {
    if ((r_halt_highlevel == 1) || (l_halt_highlevel == 1)) { // Check for either left or right motor stall to stop right motor
        r_pid.pid_target = 0;
    } else if ((r_halt_highlevel == 2) || (l_halt_highlevel == 2)) {
        r_pid.pid_target = -0.3;
    } else {
        r_pid.pid_target = msg.data;
    }
}

void encoder_reset_callback(const std_msgs::Empty& reset_msg) {
    lmotor_encoder.write(0);
    rmotor_encoder.write(0);
}
