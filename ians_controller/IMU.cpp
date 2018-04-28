#include "Arduino.h"
#include "IMU.h"

IMU::IMU(){
  bno = Adafruit_BNO055(55);
  bno.begin();
  bno.setExtCrystalUse(true);
}

sensor_msgs::Imu IMU::read_IMUmsg_data(){
  sensor_msgs::Imu IMU_msg;
  imu::Vector<3> angular_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  IMU_msg.angular_velocity.x = (float) angular_vel.x();
  IMU_msg.angular_velocity.y = (float) angular_vel.y();
  IMU_msg.angular_velocity.z = (float) angular_vel.z();

//  IMU_msg.angular_velocity_covariance.x = {0};
//  IMU_msg.angular_velocity_covariance.y = {0};
//  IMU_msg.angular_velocity_covariance.z = {0};
 
  imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  IMU_msg.linear_acceleration.x = (float) linear_accel.x();
  IMU_msg.linear_acceleration.y = (float) linear_accel.y();
  IMU_msg.linear_acceleration.z = (float) linear_accel.z();

//  IMU_msg.linear_acceleration_covariance.x = {0};
//  IMU_msg.linear_acceleration_covariance.y = {0};
//  IMU_msg.linear_acceleration_covariance.z = {0};

  imu::Quaternion quat = bno.getQuat();
  IMU_msg.orientation.w = (float) quat.w();
  IMU_msg.orientation.x = (float) quat.x();
  IMU_msg.orientation.y = (float) quat.y();
  IMU_msg.orientation.z = (float) quat.z();

//  IMU_msg.orientation_covariance.x = {0};
//  IMU_msg.orientation_covariance.y = {0};
//  IMU_msg.orientation_covariance.z = {0};

  return IMU_msg;
}

sensor_msgs::MagneticField IMU::read_compass(){
  sensor_msgs::MagneticField MF_msg;
  imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  MF_msg.magnetic_field.x = (float) magnetometer.x();
  MF_msg.magnetic_field.y = (float) magnetometer.y();
  MF_msg.magnetic_field.z = (float) magnetometer.z();

//  MF_msg.magnetic_field_covariance.x = {0};
//  MF_msg.magnetic_field_covariance.y = {0};
//  MF_msg.magnetic_field_covariance.z = {0};
  
  return MF_msg;
}



