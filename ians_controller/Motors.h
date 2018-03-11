/*
  Motors.h - Library for Reading and reseting encoder values
  values from two motors.
  Dependencies : Encoder.h - Used to count pulses of encoders
  Created by Juan Huerta, Kevin Behr, Kelvin Silva  
*/
#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <Encoder.h>

/*
 Author: Juan Huerta, Kevin Behr
| Param: None
| Return: int[] representing the cumulative encoder pulses for each
|       wheel since last read
int[] readEncoders();

| Author: Juan Huerta, Kevin Behr
| Param: None
| Remark: Reset encoder values to zero
void resetEncoders();
*/

class Motors
{
  private:
    int Int16ToPWM(std_msgs::Int16 motorSpeed);
    int rpwmPin;
    int rmDirec1;
    int rmDirec2;
    int lpwmPin;
    int lmDirec1;
    int lmDirec2;   
  
  public:
    Motors(int rightPwmPin,int rightMotorDirectionPin1,int rightMotorDirectionPin2,int leftPwmPin, int leftMotorDirectionPin1, int leftMotorDirectionPin2);
    void rightMotorForward(int motorSpeed);
    void rightMotorReverse(int motorSpeed);
    void leftMotorForward(int motorSpeed);
    void leftMotorReverse(int motorSpeed);   
};

#endif

