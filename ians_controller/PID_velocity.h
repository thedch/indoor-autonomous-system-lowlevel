/*
  PID_velocity.h - This library is used as a feedback control loop for the
  motors. It uses encoder values to calculate velocity of the robot. It calculates 
  error from desired velocity and measured and applies correction based on proportional,
  intergral and derivative terms.
  Dependencies : ros.h - This library provides Float32 ROS message type. motors.h - used to apply motor
  commands.  
  This data type is passed through to set PWM duty cyle.
  Created by Juan Huerta, Daniel Hunter
*/

#ifndef PID_VELOCITY_H
#define PID_VELOCITY_H

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <algorithm>
#include "Arduino.h"
#include "motors.h"

#define ROLLING_PTS 10

class PID_velocity
{
        motors motor;
        float pid_error;
        float pid_motor;
        float vel;
        float pid_integral;
        float pid_derivative;
        float pid_previous_error;
        float wheel_prev; // previous encoder total ticks
        float wheel_latest; // latest encoder total ticks
        int wheel_mult;
        int prev_encoder;
        float pid_Kp;
        float pid_Ki;
        float pid_Kd;
        int out_min;
        int out_max;
        int rate;
        int ticks_per_meter;
        double velocity_threshold;
        int encoder_min;
        int encoder_max;
        unsigned long prev_pid_time;
        unsigned long then; // This is in milliseconds
        // float dt; // This is in seconds
        int encoder_low_wrap;
        int encoder_high_wrap;
        int rolling_pts;
        float prev_vel[ROLLING_PTS] = { 0 };

    public:
        PID_velocity(int PWM_PIN, int MOTOR_EN1, int MOTOR_EN2, float Kd, float Kp, float Ki, float ticks_per_m);
        float pid_target;
        void calc_velocity();
        void append_vel(double val);
        void calc_rolling_vel();
        void do_pid();
        void cumulative_enc_val(int enc);
        void test_motor_control(std_msgs::Float32 msg);        
        void pid_spin();
        int check_motor_stall(float curr_encoder_val);
};

#endif
