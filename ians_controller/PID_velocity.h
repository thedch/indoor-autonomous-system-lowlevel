/*
  Motors.h - This library is used to set motor speed and direction
  Dependencies : ros.h - This library provides Float32 ROS message type. 
  This data type is passed through to set PWM duty cyle.
  Created by Juan Huerta, Kevin Beher, Kelvin Silva  
*/

#ifndef PID_VELOCITY_H
#define PID_VELOCITY_H

#include "Arduino.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>


class PID_velocity
{
  private:
    unsigned long prev_pid_time;
    int pid_error;
    int pid_target; 
    
  
  
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
