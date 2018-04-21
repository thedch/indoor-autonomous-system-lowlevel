/*
  Motors.h - This library is used to set motor speed and direction
  Dependencies : ros.h - This library provides Float32 ROS message type. 
  This data type is passed through to set PWM duty cyle.
  Created by Juan Huerta, Kevin Beher, Kelvin Silva  
*/

#ifndef PID_VELOCITY_H
#define PID_VELOCITY_H

#include <deque>

#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <ros.h>
#include "Arduino.h"


class PID_velocity
{
  private:
    int pid_error;
    int pid_target;
    int pid_motor;
    int pid_vel;
    int pid_intergral;
    int pid_derivative;
    int pid_previous_error;
    int long wheel_prev; // previous encoder total ticks
    int long wheel_latest; // latest encoder total ticks
    int wheel_mult;
    int prev_encoder;
    int pid_Kp;
    int pid_Ki;
    int pid_Kd;
    int out_min;
    int out_max;
    int rate;
    int rolling_pts;
    int timeout_ticks;
    int ticks_per_meter;
    // velocity_threshold;
    int encoder_min;
    int encoder_max;
    unsigned long prev_pid_time;
    unsigned long then; // This is in milliseconds!
    // float dt; // This is in seconds!
    int encoder_low_wrap;
    int encoder_high_wrap;
    // pid_prev_vel = [0.0] * rolling_pts;   
  
  
  public:
    PID_velocity();
    void calc_velocity();
    void append_vel(int val);
    void calc_rolling_vel();
    void do_pid();
    void wheelCallback(int encoder);
    void targetCallback(msg);
};

#endif
