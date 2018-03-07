/*
  Motors.cpp -
  Created by Juan Huerta and Kevin Behr
*/

// Encoder Interface //
// Kevin Beher, Juan Huerta//

#include "Arduino.h"
#include "Motors.h"
#include <Encoder.h>

Motors::Motors(int lEnc1, int lEnc2, int rEnc1, int rEnc2) : leftEnc(lEnc1, lEnc2), rightEnc(rEnc1, rEnc2){
//  this->leftEnc(lEnc1, lEnc2);
  //this->rightEnc(rEnc1, rEnc2);
  oldPositionL  = -999;
  oldPositionR  = -999;
  degrees_per_tick = 15; 
  AngleL = 0;
  ticksL = 0;
  AngleR = 0;
  ticksR = 0;
}

int* Motors::readEncoders(){
    int* pointer;
    int encVal[2];
    pointer = encVal;
    encVal[0] = leftEnc.read(); //newPosition Left Encoder
    encVal[1] = rightEnc.read(); //newPosition Right Encoder
    //update old position of encoders and convert to Angle
    if (encVal[0] != oldPositionL) {
      oldPositionL = encVal[0];
      ticksL = encVal[0]  / 4;
      AngleL = ticksL * degrees_per_tick;
    }
    if (encVal[1] != oldPositionR) {
      oldPositionR = encVal[1];
      ticksR = encVal[1]  / 4;
      AngleR = ticksR * degrees_per_tick;
    }
  Serial.print("Basic Encoder Test: ");
  Serial.print("Left encoder angle: ");
  Serial.print(AngleL);
  Serial.print(", Right encoder angle: ");
  Serial.print(AngleR);
  Serial.println();
  return pointer;
}

void Motors::resetEncoders(){
    //reset encoder values
    leftEnc.write(0);
    rightEnc.write(0);
    //digitalWrite(13,HIGH);
    ticksL = 0;           
    ticksR = 0;
    AngleL = 0;
    AngleR = 0;
}
