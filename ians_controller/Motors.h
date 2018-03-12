/*
  Motors.h - This library is used to set motor speed and direction
  Dependencies : ros.h - This library provides Float32 ROS message type. 
  This data type is passed through to set PWM duty cyle.
  Created by Juan Huerta, Kevin Beher, Kelvin Silva  
*/

#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"
#include <ros.h>
#include <std_msgs/Float32.h>

/*
| Author: Juan Huerta
| Param: Digital Pins values on the Teensy used as PWM and 
|	motor direction and direction enables
| Remark: This constructor sets the pin numbers given as private variables
|	for the class
Motors(int rightPwmPin,int rightMotorDirectionPin1,int rightMotorDirectionPin2,
int leftPwmPin, int leftMotorDirectionPin1, int leftMotorDirectionPin2);

| Author: Juan Huerta
| Param: motorSpeed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the right
|	motor and sets directions pins to rotate motors forward
void rightMotorForward(std_msgs::Float32 motorSpeed);

| Author: Juan Huerta
| Param: motorSpeed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the right
|	motor and sets directions pins to rotate motors in reverse
void rightMotorReverse(std_msgs::Float32 motorSpeed);

| Author: Juan Huerta
| Param: motorSpeed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the left
|	motor and sets directions pins to rotate motors forward
void leftMotorForward(std_msgs::Float32 motorSpeed);

| Author: Juan Huerta
| Param: motorSpeed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the left
|	motor and sets directions pins to rotate motors in reverse
void leftMotorReverse(std_msgs::Float32 motorSpeed);
\end{minted}

| Author: Juan Huerta
| Return: Void
| Remark: This function sets motor direction pins to zero
|	and PWM duty cycle to zero to brake left motor
void leftMotorBrake();

| Author: Juan Huerta
| Return: Void
| Remark: This function sets motor direction pins to zero
|	and PWM duty cycle to zero to brake right motor
void rightMotorBrake();
*/

class Motors
{
  private:
    int rpwmPin;
    int rmDirec1;
    int rmDirec2;
    int lpwmPin;
    int lmDirec1;
    int lmDirec2;   
  
  public:
    Motors(int rightPwmPin,int rightMotorDirectionPin1,int rightMotorDirectionPin2,int leftPwmPin, int leftMotorDirectionPin1, int leftMotorDirectionPin2);
    void rightMotorForward(std_msgs::Float32 motorSpeed);
    void rightMotorReverse(std_msgs::Float32 motorSpeed);
    void leftMotorForward(std_msgs::Float32 motorSpeed);
    void leftMotorReverse(std_msgs::Float32 motorSpeed);
    void leftMotorBrake();
    void rightMotorBrake();       
};

#endif
