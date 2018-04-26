#include "Arduino.h"
#include "IMU.h"

IMU::IMU(){
  bno = Adafruit_BNO055(55);
  bno.begin();
  bno.setExtCrystalUse(true);
}

sensor_msgs::Imu IMU::read_data(){
  sensor_msgs::Imu IMU_msg;
  imu::Vector<3> angular_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  IMU_msg.angular_velocity.x = (float) angular_vel.x();
  IMU_msg.angular_velocity.y = (float) angular_vel.y();
  IMU_msg.angular_velocity.z = (float) angular_vel.z();
 
  imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  IMU_msg.linear_acceleration.x = (float) linear_accel.x();
  IMU_msg.linear_acceleration.y = (float) linear_accel.y();
  IMU_msg.linear_acceleration.z = (float) linear_accel.z();

  imu::Quaternion quat = bno.getQuat();
  IMU_msg.orientation.w = (float) quat.w();
  IMU_msg.orientation.x = (float) quat.x();
  IMU_msg.orientation.y = (float) quat.y();
  IMU_msg.orientation.z = (float) quat.z();

  return IMU_msg;
}



