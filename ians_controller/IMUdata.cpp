#include "Arduino.h"
#include "IMUdata.h"

IMUdata::IMUdata(){

  Adafruit_BNO055 bno = Adafruit_BNO055(55);
  bno.begin();
  bno.setExtCrystalUse(true);
  imu::Vector<3> AngularVel = 0;
  imu::Vector<3> LinearAccel  = 0;
  imu::Quaternion Quat;

}

void IMUdata::PackData(){
     AngularVel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
     LinearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 
     Quat = bno.getQuat();

     sensor_msgs::Imu ImuData;
     ImuData.data.orientaton = Quat;
     ImuData.data.angular_velocity = AngularVel;
     ImuData.data.linear_acceleration = LinearAccel;     
}



