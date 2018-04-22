/*
  Motors.h - This library is used to set motor speed and direction
  Dependencies : ros.h - This library provides Float32 ROS message type. 
  This data type is passed through to set PWM duty cyle.
  Created by Juan Huerta, Kevin Beher, Kelvin Silva  
*/

#ifndef PID_VELOCITY_H
#define PID_VELOCITY_H

//#include <vector>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>


#include "Arduino.h"
#include "MovingAverage.cpp"

//class MovingAverage{
//    private:
//        std::vector<double> *q;
//        double sum;
//    public:
//        MovingAverage(int rolling_history_size){
//            sum = 0;
//            q = new std::vector<double>(rolling_history_size);
//        }
//        ~MovingAverage(){
//            delete q;
//        }
//        void push(double v){
//            if(q->size() == q->capacity()){
//                double t = (double) q->front();
//                sum-=t;
//                q->erase(q->begin());
//            }
//            q->push_back(v);
//            sum+=(double)v;
//        }
//        double size(){
//            return q->size();
//        }
//        double mean(){
//           //return sum/size();
//           return 10;
//        }
//};

class PID_velocity
{
  private:
//    double pid_error;
//    double pid_motor;
//    double pid_vel;
//    double pid_intergral;
//    double pid_derivative;
//    double pid_previous_error;
//    double wheel_prev; // previous encoder total ticks
//    double wheel_latest; // latest encoder total ticks
//    int wheel_mult;
//    int prev_encoder;
//    double pid_Kp;
//    double pid_Ki;
//    double pid_Kd;
//    int out_min;
//    int out_max;
//    int rate;
//    int timeout_ticks;
//    int ticks_per_meter;
//    double velocity_threshold;
//    int encoder_min;
//    int encoder_max;
//    int ticks_since_target;
//    unsigned long prev_pid_time;
//    unsigned long then; // This is in milliseconds!
//    // float dt; // This is in seconds!
//    int encoder_low_wrap;
//    int encoder_high_wrap;
    MovingAverage *pid_prev_vel;
    int rolling_pts;
  
  public:
    PID_velocity();
//    void calc_velocity();
//    void append_vel(double val);
//    void calc_rolling_vel();
//    void do_pid();
//    void wheelCallback(int enc);
//    void targetCallback(std_msgs::Float32 msg);
//    int pid_target;
//    void pid_spin();
};

#endif
