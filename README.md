# Indoor Autonomous Navigation System Low Level

### Repository Hierarchy

The Indoor Autonmous System Software is contained within three repositories. 

1. indoor-autonomous-system-cloud
1. indoor-autonomous-system-highlevel
1. indoor-autonomous-system-lowlevel

The cloud repository contains code to run the front facing Flask server on GCE. This presents the user with a control panel containing destinations inside the floor plan, and a kill switch.

The high level repository contains ROS code that runs on a Raspberry Pi. This deals with the ROS navigation stack, manual control of the robot, interfacing with the LiDAR, the cloud server, and the low level micro. 

The low level repository contains C++ code that interfaces directly with the sensors and motors to control the direction of the robot and read wheel odometry and IMU data. This data is then passed back to the Raspberry Pi where it is processed by ROS. 
