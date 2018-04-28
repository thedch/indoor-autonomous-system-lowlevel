#include "Arduino.h"
#include "Motors.h"
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Float32.h>

Encoder myEnc1(32, 33); // Left Encoder
Encoder myEnc2(31, 34); // Right Encoder


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
 Serial.begin(38400); 
 right_motorspeed.data = 128 ;
 left_motorspeed.data = 128;
}
 
void loop() {

  long RightWheelPosition = myEnc2.read(); // Grabs current encoder reading.
  long LeftWheelPosition = myEnc1.read();

  
  if (RightWheelPosition != oldRightWheelPosition) {
    oldRightWheelPosition = RightWheelPosition;
    ticksR = RightWheelPosition  / 4;
  }
  
  if (LeftWheelPosition != oldLeftWheelPosition) {
    oldLeftWheelPosition = LeftWheelPosition;
    ticksL = LeftWheelPosition  / 4;
  }


/// begin right wheel state machine ///
switch (RightWheelCurrentState) {
case (MovingR):
  newTicksR = ticksR;
  if ( (right_motorspeed.data != 0) && (newTicksR == ticksR) ) { // transistions to next state is always true. This state initializes the time.
  RightWheelCurrentState == StalledR;
  timeR = millis();
  }
  break;

case (StalledR): // Checks if parameters of stall are met.
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
  right_motorspeed.data = 0;
  break;
  
  default:
  break;
}

/// begin left wheel state machine ///
switch (LeftWheelCurrentState) {
case (MovingL):
  newTicksL = ticksL;
  if ( (left_motorspeed.data != 0) && (newTicksL == ticksL) ) { // transistions to next state is always true. This state initializes the time.
  LeftWheelCurrentState == StalledR;
  timeL = millis();
  }
  break;

case (StalledL): // Checks if parameters of stall are met.
    if ( (ticksL == newTicksL) && (left_motorspeed.data != 0) && ( (millis() - timeL) > 1000) ) {
    LeftWheelCurrentState = TurnOffL;
  }
if (ticksL != newTicksL) {
  LeftWheelCurrentState = MovingL;
}
break;
case (TurnOffL):
  Serial.print("You're in left wheel turn off state.");
  Serial.print("\r\n");
  left_motorspeed.data = 0;
  break;
  
  default:
  break;
}
}
