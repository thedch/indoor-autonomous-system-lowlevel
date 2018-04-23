/*
  Motors.h - This library is used to set motor speed and direction
  Dependencies : ros.h - This library provides Float32 ROS message type.
  This data type is passed through to set PWM duty cyle.
  Created by Juan Huerta, Kevin Beher, Kelvin Silva
*/

#ifndef PID_VELOCITY_H
#define PID_VELOCITY_H

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include "Arduino.h"

#define ROLLING_PTS 5

class PID_velocity
{
    private:
        float pid_error;
        float pid_motor;
        float pid_vel;
        float pid_intergral;
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
        int timeout_ticks;
        int ticks_per_meter;
        double velocity_threshold;
        int encoder_min;
        int encoder_max;
        int ticks_since_target;
        unsigned long prev_pid_time;
        unsigned long then; // This is in milliseconds!
        // float dt; // This is in seconds!
        int encoder_low_wrap;
        int encoder_high_wrap;
        int rolling_pts;
        float prev_vel[ROLLING_PTS];

    public:
        PID_velocity();
        void calc_velocity();
        void append_vel(double val);
        void calc_rolling_vel();
        void do_pid();
        void wheelCallback(int enc);
        void targetCallback(std_msgs::Float32 msg);
        int pid_target;
        void pid_spin();
};

#endif
