#ifndef IMUDATA_H
#define IMUDATA_H

#include "Arduino.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


class IMUdata
{
  private:
    imu::Vector<3> AngularVel;
    imu::Vector<3> LinearAccel;
    imu::Quaternion quat;
    
  
  public:
    IMUdata();
    void PackData();
};

#endif
