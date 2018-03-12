# Indoor Autonomous Navigation System Low Level

### Repository Hierarchy

The Indoor Autonmous System Software is contained within three repositories. 

1. [indoor-autonomous-system-cloud](https://github.com/thedch/indoor-autonomous-system-cloud)
1. [indoor-autonomous-system-highlevel](https://github.com/thedch/indoor-autonomous-system-highlevel)
1. [indoor-autonomous-system-lowlevel](https://github.com/thedch/indoor-autonomous-system-lowlevel)

The cloud repository contains code to run the front facing Flask server on GCE. This presents the user with a control panel containing destinations inside the floor plan, and a kill switch.

The high level repository contains ROS code that runs on a Raspberry Pi. This deals with the ROS navigation stack, manual control of the robot, interfacing with the LiDAR, the cloud server, and the low level micro. 

The low level repository contains C++ code that interfaces directly with the sensors and motors to control the direction of the robot and read wheel odometry and IMU data. This data is then passed back to the Raspberry Pi where it is processed by ROS. 

## Setup

In order to connect the Pi to the Teensy, run the following command:

```
rosrun rosserial_python serial_node.py /dev/ttyACM[#]
```

Where the number represents the USB port (usually 0 or 1). An easy way to check that USB port is currently active is `ls /dev | grep ACM`. 

## ians_controller.ino


Dependecies:
Used to read pulses from encoders.
1. Encoders.h(https://github.com/PaulStoffregen/Encoder)

## Motors.h
Motors.h is the API used set motor power and direction of the differential drive robot.
