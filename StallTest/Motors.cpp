/*
  Motors.cpp -
  Created by Juan Huerta and Kevin Beher
*/

#include "Arduino.h"
#include "Motors.h"

Motors::Motors(int right_pwm_pin, int rmotor_direction_pin1, int rmotor_direction_pin2, int left_pwm_pin, int lmotor_direction_pin1, int lmotor_direction_pin2){
  pinMode(right_pwm_pin, OUTPUT); //right motor PWM pin
  pinMode(rmotor_direction_pin1, OUTPUT); //right motor direction en1
  pinMode(rmotor_direction_pin2, OUTPUT); //right motor direction en2
  pinMode(left_pwm_pin, OUTPUT); //left motor PWM pin
  pinMode(lmotor_direction_pin1, OUTPUT); //left motor direction en1
  pinMode(lmotor_direction_pin2, OUTPUT); //left motor direction en2
  r_pwm_pin = right_pwm_pin;
  rm_dir1 = rmotor_direction_pin1;
  rm_dir2 = rmotor_direction_pin2;
  l_pwm_pin = left_pwm_pin;
  lm_dir1 = lmotor_direction_pin1;
  lm_dir2 = lmotor_direction_pin2;   
}

void Motors::right_motor_forward(std_msgs::Float32 motor_speed){
  analogWrite(r_pwm_pin, motor_speed.data);
  digitalWrite(rm_dir1, LOW);
  digitalWrite(rm_dir2, HIGH);
}

void Motors::right_motor_reverse(std_msgs::Float32 motor_speed){
  analogWrite(r_pwm_pin, motor_speed.data);
  digitalWrite(rm_dir1, HIGH);
  digitalWrite(rm_dir2, LOW);
}

void Motors::right_motor_brake(){
  analogWrite(r_pwm_pin, 0);
  digitalWrite(rm_dir1, LOW);
  digitalWrite(rm_dir2, LOW);
}

void Motors::left_motor_forward(std_msgs::Float32 motor_speed){
  analogWrite(l_pwm_pin, motor_speed.data);
  digitalWrite(lm_dir1, HIGH);
  digitalWrite(lm_dir2, LOW);
}

void Motors::left_motor_reverse(std_msgs::Float32 motor_speed){
  analogWrite(l_pwm_pin, motor_speed.data);
  digitalWrite(lm_dir1, LOW);
  digitalWrite(lm_dir2, HIGH);
}

void Motors::left_motor_brake(){
  analogWrite(l_pwm_pin, 0);
  digitalWrite(lm_dir1, LOW);
  digitalWrite(lm_dir2, LOW);
}
