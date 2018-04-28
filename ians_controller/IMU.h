#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


class IMU
{
  private:
    Adafruit_BNO055 bno;
    
  public:
    IMU();
    sensor_msgs::Imu read_IMUmsg_data();
    sensor_msgs::MagneticField read_compass();
};

#endif
