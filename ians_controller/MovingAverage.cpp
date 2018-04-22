/*
  Motors.h - This library is used to set motor speed and direction
  Dependencies : ros.h - This library provides Float32 ROS message type. 
  This data type is passed through to set PWM duty cyle.
  Created by Juan Huerta, Kevin Beher, Kelvin Silva  
*/

#ifndef MOVINGAVERAGE_CPP
#define MOVINGAVERAGE_CPP

#include <vector>

#include "Arduino.h"

class MovingAverage{
    private:
        std::vector<double> *q;
        double sum;
    public:
        MovingAverage(int rolling_history_size){
            sum = 0;
            q = new std::vector<double>(rolling_history_size);
        }
        ~MovingAverage(){
            delete q;
        }
        void push(double v){
            if(q->size() == q->capacity()){
                double t = (double) q->front();
                sum-=t;
                q->erase(q->begin());
            }
            q->push_back(v);
            sum+=(double)v;
        }
        double size(){
            return q->size();
        }
        double mean(){
           //return sum/size();
           return 10;
        }
};

#endif
