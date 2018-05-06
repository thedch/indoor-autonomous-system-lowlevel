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
int timer = 0; // Time variable for wheel
int newTicksR = 0; // Used to capture initial right wheel position
int newTicksL = 0; // Used to capture initial left wheel position

enum WheelState { // States for right wheel
  Moving,
  Stalled,
  TurnOff,
  GoBack
};
enum WheelState WheelCurrentState = Moving;

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
  
  if (RightWheelPosition != oldRightWheelPosition) {
    oldRightWheelPosition = RightWheelPosition;
    ticksR = RightWheelPosition  / 4;
  }
  
  if (LeftWheelPosition != oldLeftWheelPosition) {
    oldLeftWheelPosition = LeftWheelPosition;
    ticksL = LeftWheelPosition  / 4;
  }


///// begin wheel state machine ///
  switch (WheelCurrentState) {
    case (Moving):
      newTicksR = ticksR;
      newTicksL = ticksL;
      if  ((right_motorspeed.data != 0) || (left_motorspeed.data != 0))  { // This state initializes the time and captures wheel position.
        WheelCurrentState = Stalled;
        timer = millis();
      }
      break;

    case (Stalled): // Checks if parameters of stall are met.
      Serial.println("Stalled");
      if ( (((ticksR == newTicksR) && (right_motorspeed.data != 0)) || ((ticksL == newTicksL) && (left_motorspeed.data != 0))) && ((millis() - timer) > 1000) ) {
        WheelCurrentState = TurnOff;
        timer = millis();
      }
      if ((ticksR != newTicksR) && (ticksL != newTicksL)) {
        WheelCurrentState = Moving;
      }
      break;
      
    case (TurnOff):
      Serial.print("You're in turn off state.");
      Serial.print("\r\n");
      left_motorspeed.data = 0;
      right_motorspeed.data = 0;
      left_motor.motor_cmd(left_motorspeed);
      right_motor.motor_cmd(right_motorspeed);
      if((millis() - timer) > 2500){
        WheelCurrentState = GoBack;
        timer = millis();        
      }
      break;
    case(GoBack):
      left_motorspeed.data = -50;
      right_motorspeed.data = -50;
      left_motor.motor_cmd(left_motorspeed);
      right_motor.motor_cmd(right_motorspeed);
      if((millis() - timer) > 3000){
        WheelCurrentState = Moving;
        left_motorspeed.data = 0;
        right_motorspeed.data = 0;
        left_motor.motor_cmd(left_motorspeed);
        right_motor.motor_cmd(right_motorspeed);
      }
      break;
    default:
      break;
  }
}
