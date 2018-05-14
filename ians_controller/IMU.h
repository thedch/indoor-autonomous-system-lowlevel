/*
  IMU.h - This library is used read absolute orientation (quaternion)
  with respect to north and return the w and z component in a tuple.
  Dependencies : ros.h - This library provides Float32 ROS message type. 
  This data type is passed through to set PWM duty cyle.
  Created by Juan Huerta and Kevin Beher 
*/

#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float32.h>
#include <tuple>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


class IMU
{
    private:
        Adafruit_BNO055 bno;
        void read_compass();

    public:
        IMU();
        std::tuple<std_msgs::Float32, std_msgs::Float32> read_IMUmsg_data();

};

#endif
