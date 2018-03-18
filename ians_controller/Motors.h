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
|    motor direction and direction enables
| Remark: This constructor sets the pin numbers given as private
|    variables of the class
Motors(int right_pwm_Pin,int rmotor_direction_pin1,
int rmotor_direction_pin2, int left_pwm_pin,
int lmotor_direction_pin1, int lmotor_direction_pin2);

| Author: Juan Huerta
| Param: motor_speed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the right
|    motor and sets directions pins to rotate motors forward
void Motors::right_motor_forward(std_msgs::Float32 motor_speed);

| Author: Juan Huerta
| Param: motor_speed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the right
|    motor and sets directions pins to rotate motors in reverse
void Motors::right_motor_reverse(std_msgs::Float32 motor_speed);

| Author: Juan Huerta
| Param: motor_speed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the left
|    motor and sets directions pins to rotate motors forward
void Motors::left_motor_forward(std_msgs::Float32 motor_speed);

| Author: Juan Huerta
| Param: motor_speed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the left
|    motor and sets directions pins to rotate motors in reverse
void Motors::left_motor_reverse(std_msgs::Float32 motor_speed);

| Author: Juan Huerta
| Param: None
| Return: Void
| Remark: This function sets motor direction pins to zero
|    and PWM duty cycle to zero to brake left motor
void Motors::left_motor_brake();

| Author: Juan Huerta
| Param: None
| Return: Void
| Remark: This function sets motor direction pins to zero
|    and PWM duty cycle to zero to brake right motor
void Motors::right_motor_brake();

*/

class Motors
{
  private:
    int r_pwm_pin;
    int rm_dir1;
    int rm_dir2;
    int l_pwm_pin;
    int lm_dir1;
    int lm_dir2;   
  
  public:
    Motors(int right_pwm_Pin, int rmotor_direction_pin1, int rmotor_direction_pin2, int left_pwm_pin, int lmotor_direction_pin1, int lmotor_direction_pin2);
    void right_motor_forward(std_msgs::Float32 motor_speed);
    void right_motor_reverse(std_msgs::Float32 motor_speed);
    void left_motor_forward(std_msgs::Float32 motor_speed);
    void left_motor_reverse(std_msgs::Float32 motor_speed);
    void left_motor_brake();
    void right_motor_brake();       
};

#endif
