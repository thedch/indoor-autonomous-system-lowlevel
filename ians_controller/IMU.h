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
