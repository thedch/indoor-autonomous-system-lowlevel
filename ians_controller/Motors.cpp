/*
  Motors.cpp -
  Created by Juan Huerta and Kevin Behr
*/

// Encoder Interface //
// Kevin Beher, Juan Huerta//

#include "Arduino.h"
#include "Motors.h"

Motors::Motors(int rightPwmPin,int rightMotorDirectionPin1,int rightMotorDirectionPin2,int leftPwmPin, int leftMotorDirectionPin1, int leftMotorDirectionPin2){
  pinMode(rightPwmPin, OUTPUT); //right motor PWM pin
  pinMode(rightMotorDirectionPin1, OUTPUT); //right motor direction en1
  pinMode(rightMotorDirectionPin2, OUTPUT); //right motor direction en2
  pinMode(leftPwmPin, OUTPUT); //left motor PWM pin
  pinMode(leftMotorDirectionPin1, OUTPUT); //left motor direction en1
  pinMode(leftMotorDirectionPin2, OUTPUT); //left motor direction en2
  rpwmPin = rightPwmPin;
  rmDirec1 = rightMotorDirectionPin1;
  rmDirec2 = rightMotorDirectionPin2;
  lpwmPin = leftPwmPin;
  lmDirec1 = leftMotorDirectionPin1;
  lmDirec2 = leftMotorDirectionPin2;   
}

void Motors::rightMotorForward(int motorSpeed){
//  int pwmSpeed = Int16ToPWM(motorSpeed);
  analogWrite(rpwmPin,motorSpeed);
  digitalWrite(rmDirec1,LOW);
  digitalWrite(rmDirec2,HIGH);
}

void Motors::rightMotorReverse(int motorSpeed){
//  int pwmSpeed = Int16ToPWM(motorSpeed);
  analogWrite(rpwmPin,motorSpeed);
  digitalWrite(rmDirec1,HIGH);
  digitalWrite(rmDirec2,LOW);
}

void Motors::leftMotorForward(int motorSpeed){
//  int pwmSpeed = Int16ToPWM(motorSpeed);
  analogWrite(lpwmPin,motorSpeed);
  digitalWrite(lmDirec1,LOW);
  digitalWrite(lmDirec2,HIGH);
}

void Motors::leftMotorReverse(int motorSpeed){
//  int pwmSpeed = Int16ToPWM(motorSpeed);
  analogWrite(lpwmPin,motorSpeed);
  digitalWrite(lmDirec1,HIGH);
  digitalWrite(lmDirec2,LOW);
}

int Motors::Int16ToPWM(std_msgs::Int16 motorSpeed){
  int temp = 0;
  temp = ((motorSpeed.data * 256) / 65535);
  return temp;  
}

