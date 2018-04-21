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
    wheel_prev;
    wheel_latest;    
    wheel_mult;
    prev_encoder;
    prev_pid_time;
    int pid_Kp;
    int pid_Ki;
    int pid_Kd;
    int out_min;
    int out_max;
    int rate;
    int rolling_pts;
    int timeout_ticks;
    int ticks_per_meter;
    velocity_threshold;
    int encoder_min;
    int encoder_max;
    float then;
    float dt_duration; // This is in milliseconds!
    float dt; // This is in seconds!
    encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min;
    // pid_prev_vel = [0.0] * rolling_pts;
    wheel_latest;
      
  
  
  public:
    PID_velocity();
    void calc_velocity();
    void append_vel(int val);
    void calc_rolling_vel();
    void do_pid();
    void wheelCallback(msg);
    void targetCallback(msg);
};

#endif
