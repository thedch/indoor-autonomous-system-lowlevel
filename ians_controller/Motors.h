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
    Encoder leftEnc;
    Encoder rightEnc;
    int oldPositionL;
    int oldPositionR;
    int degrees_per_tick;
    int AngleL;
    int ticksL;
    int AngleR;
    int ticksR;
  
  public:
    Motors(int lEnc1, int lEnc2, int rEnc1, int rEnc2);
    int* readEncoders();
    void resetEncoders();
};

#endif

