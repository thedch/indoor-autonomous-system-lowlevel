#include "Arduino.h"
#include "Motors.h"
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#define LEFT_ENCODER_PIN1 34
#define LEFT_ENCODER_PIN2 33
#define RIGHT_ENCODER_PIN1 31
#define RIGHT_ENCODER_PIN2 32
#define RIGHT_PWM_PIN 2
#define RIGHT_MOTOR_EN1 24
#define RIGHT_MOTOR_EN2 25
#define LEFT_PWM_PIN 23
#define LEFT_MOTOR_EN1 21
#define LEFT_MOTOR_EN2 22

Encoder myEnc1(LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2); // Left Encoder
Encoder myEnc2(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2); // Right Encoder

Motors left_motor(LEFT_PWM_PIN,LEFT_MOTOR_EN1,LEFT_MOTOR_EN2);
Motors right_motor(RIGHT_PWM_PIN,RIGHT_MOTOR_EN1,RIGHT_MOTOR_EN2);

int ticksR = 0;
int ticksL = 0;
// Declaration of variables for state machine.
int timeR = 0; // Time variable for right wheel
int newTicksR = 0; // Used to capture initial right wheel position
int timeL = 0;
int newTicksL = 0;

enum RightWheelState { // States for right wheel
  MovingR,
  StalledR,
  TurnOffR
};
enum RightWheelState RightWheelCurrentState = MovingR;

enum LeftWheelState { // States for left wheel
  MovingL,
  StalledL,
  TurnOffL
};
enum LeftWheelState LeftWheelCurrentState = MovingL;

long oldRightWheelPosition = -999;  // From encoders test on Arduino example.
long oldLeftWheelPosition  = -999;

std_msgs::Float32 right_motorspeed;
std_msgs::Float32 left_motorspeed;


void setup() {  
 Serial.begin(9600);
 right_motorspeed.data = 128 ;
 left_motorspeed.data = 128;
 left_motor.motor_cmd(left_motorspeed);
 right_motor.motor_cmd(right_motorspeed);
}
 
void loop() {
  

  long RightWheelPosition = myEnc2.read(); // Grabs current encoder reading.
  long LeftWheelPosition = myEnc1.read();
  Serial.println("Begin State");
//  delay(1000);

  
  if (RightWheelPosition != oldRightWheelPosition) {
    oldRightWheelPosition = RightWheelPosition;
    ticksR = RightWheelPosition  / 4;
  }
  
  if (LeftWheelPosition != oldLeftWheelPosition) {
    oldLeftWheelPosition = LeftWheelPosition;
    ticksL = LeftWheelPosition  / 4;
  }


///// begin right wheel state machine ///
  switch (RightWheelCurrentState) {
    case (MovingR):
      Serial.println("Moving R");
      newTicksR = ticksR;
     
      if  (right_motorspeed.data != 0)  { // transistions to next state is always true. This state initializes the time.
        Serial.println(right_motorspeed.data);
        RightWheelCurrentState = StalledR;
        timeR = millis();
      }
      break;

    case (StalledR): // Checks if parameters of stall are met.
      Serial.println("StalledR");
      if ( (ticksR == newTicksR) && (right_motorspeed.data != 0) && ( (millis() - timeR) > 1000) ) {
        RightWheelCurrentState = TurnOffR;
      }
      if (ticksR != newTicksR) {
       RightWheelCurrentState = MovingR;
      }
      break;
      
    case (TurnOffR):
      Serial.print("You're in right wheel turn off state.");
      Serial.print("\r\n");
      left_motorspeed.data = 0;
      right_motorspeed.data = 0;
      left_motor.motor_cmd(left_motorspeed);
      right_motor.motor_cmd(right_motorspeed);
      break;
  
    default:
     break;
}

/// begin left wheel state machine ///
//switch (LeftWheelCurrentState) {
//case (MovingL):
//  Serial.println("MovingL");
//  newTicksL = ticksL;
//  if ( (left_motorspeed.data != 0) && (newTicksL == ticksL) ) { // transistions to next state is always true. This state initializes the time.
//  LeftWheelCurrentState == StalledR;
//  timeL = millis();
//  }
//  break;
//
//case (StalledL): // Checks if parameters of stall are met.
//    Serial.println("StalledL");
//    if ( (ticksL == newTicksL) && (left_motorspeed.data != 0) && ( (millis() - timeL) > 1000) ) {
//    LeftWheelCurrentState = TurnOffL;
//  }
//if (ticksL != newTicksL) {
//  LeftWheelCurrentState = MovingL;
//}
//break;
//case (TurnOffL):
//  Serial.print("You're in left wheel turn off state.");
//  Serial.print("\r\n");
//  left_motorspeed.data = 0;
//  right_motorspeed.data = 0;
//  left_motor.motor_cmd(left_motorspeed);
//  right_motor.motor_cmd(right_motorspeed);
//
//  break;
//  
//  default:
//  break;
//}
}
