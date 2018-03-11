/*
  Motors.cpp -
  Created by Juan Huerta and Kevin Behr
*/

// Encoder Interface //
// Kevin Beher, Juan Huerta//

#include "Arduino.h"
#include "Motors.h"

Motors::Motors(int rightPwmPin,int rightMotorEn1,int rightMotorEn2,int leftPwmPin, int leftMotorEn1, int leftMotorEn2){
  pinMode(rightPwmPin, OUTPUT); //right motor PWM pin
  pinMode(rightMotorEn1, OUTPUT); //right motor direction en1
  pinMode(rightMotorEn2, OUTPUT); //right motor direction en2
  pinMode(leftPwmPin, OUTPUT); //left motor PWM pin
  pinMode(leftMotorEn1, OUTPUT); //left motor direction en1
  pinMode(leftMotorEn2, OUTPUT); //left motor direction en2
  rpwmPin = rightPwmPin;
  rmEn1 = rightMotorEn1;
  rmEn2 = rightMotorEn2;
  lpwmPin = leftPwmPin;
  lmEn1 = leftMotorEn1;
  lmEn2 = leftMotorEn2; 
  
}

void Motors::rightMotorForward(int motorSpeed){
  analogWrite(rpwmPin,motorSpeed);
  digitalWrite(rmEn1,LOW);
  digitalWrite(rmEn2,HIGH);
}

void Motors::rightMotorReverse(int motorSpeed){
  analogWrite(rpwmPin,motorSpeed);
  digitalWrite(rmEn1,HIGH);
  digitalWrite(rmEn2,LOW);
}

void Motors::leftMotorForward(int motorSpeed){
  analogWrite(lpwmPin,motorSpeed);
  digitalWrite(lmEn1,LOW);
  digitalWrite(lmEn2,HIGH);
}

void Motors::leftMotorReverse(int motorSpeed){
  analogWrite(lpwmPin,motorSpeed);
  digitalWrite(lmEn1,HIGH);
  digitalWrite(lmEn2,LOW);
}

