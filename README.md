# Indoor Autonomous Navigation System Low Level

## Repository Hierarchy

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

## How To echo rostopics

In order to reset encoder values, run the following command:

```
rostopic echo /imu_data
```

## How To Reset Encoder Values Through ROS

In order to reset encoder values, run the following command:

```
rostopic pub reset_encoders std_msgs/Empty --once
```

## ians_controller.ino
ians_controller.ino is the main file that handles calling the Motors API and the built in ROS and Enocder libraries.

Dependecies:

* [ros.h](http://wiki.ros.org/roslib)
* [Encoders.h](https://github.com/PaulStoffregen/Encoder)

### ros.h
ros.h is a ROS library for Arduino that is used to set up the ROS functions that subscribe and publish to topics on the ROS system of the Raspberry Pi. Below is an example on how to create ROS publisher and subscriber functions within a ROS project.

 ```
  #include <ros.h>
  #include <std_msgs/String.h>
  #include <std_msgs/Empty.h>
  // LINK FOR MORE EXAMPLES
  // http://wiki.ros.org/rosserial_arduino/Tutorials/
 
  //ROS Node Handler
  ros::NodeHandle nh;
 
  // Callback headers to be used when a ROS topic publish is received
  void toggle_callback(const std_msgs::Empty& toggle_msg);
  
  /* ROS Subcriber Template Function
     Function can be named anything
     First arguement says it will subscribe to toggle_led topic
     Second arguement is callback that is triggered when a message is published
     to toggle_led topic */
  ros::Subscriber<std_msgs::Empty> toggle_led_sub("toggle_led", &toggle_callback);
  
  /* ROS Publisher Template Function
     Function can be named anything
     First arguement says it will publish to chatter topic
     Second arguement is reference to message instance to be used for publishing */
  std_msgs::String str_msg;
  ros::Publisher chatter_pub("chatter", &str_msg);
  
  // Create hellow world string
  char hello[13] = "hello world!";
   
  void setup()
  {
    pinMode(13,OUTPUT);
    nh.initNode(); // Instantiate node handler
    nh.advertise(chatter_pub); //initialize chatter topic
    nh.subscribe(toggle_led_sub); //initialize toggle_led topic
  }
   
  void loop()
  {
    str_msg.data = hello; // Populate str_msg struct's data field with hello string 
    chatter.publish(&str_msg); // Publish string to chatter topic. Place where you want message to be published
    nh.spinOnce(); // all ROS communication callbacks are handled
    delay(1000); // Delay prevents queue overflow from continuous publishing over serial
  }
  
  //CALLBACK FUNCTION LOGIC
  void message_callback(const std_msgs::Empty& toggle_msg){
      digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  }
```

### Encoders.h
Encoders.h is a library made for the Teensy that is used to count pulses from quadrature encoded signals.
ians_controller.ino uses the functions:

* **Encoder my_enc(pin1, pin2)** - Creates an Encoder object, using 2 pins.
* **my_enc.read()** - Returns the positive or negative accumulated position.
* **my_enc.write(newPosition)** - Set the position to a new value.

## pid_velocity.h
pid_velocity.h is the API used to create a motors object with a PID controller overlayed on top. ians_controller.ino sets the target velocity of the motors in meters per second then uses the encoders library to read current encoder value and call cumulative_enc_val to update the cumulative encoder value. The cumulative encoder value is used in calculating the current velocity and apply correction to the motor.

* **PID_velocity my_pid(PWM_PIN, MOTOR_EN1, MOTOR_EN2, KD, KP, KI, TICKS_PER_METER)** - Creates a Motors object using the pins connected from the H-bridge to the Teensy.
* **my_pid.pid_target = target_value** - Sets the new target for velocity in m/s for the motor
* **my_pid.cumulative_enc_val(enc_value)** - Pass value read from encoder to update cumulative encoder value. 
* **my_pid.pid_spin()** - Calculates current average velocity using encoders and error with respect to pid_target and applies correction to motors

## motors.h
motors.h is the API used set motor power and direction of the differential drive robot.

* **motors my_motor(pwm_Pin, motor_direction_pin1, motor_direction_pin2)** - Creates a Motors object using the pins connected from the H-bridge to the Teensy.
* **my_motor.motor_cmd(motorSpeed)** - Input it is PWM parameter from -255 to 255

### Wheel Stall State Machine
The wheel stall state machine is implemented to protect the hardware from current surges that occur when the wheels stall.
Shown below is the state diagram of how this works.

![Wheel Stall State Machine](./images/WheelStallSM.png)

The first state captures the current encoder position of each wheel and a time reference. The next state will compare real time encoder positions to the snapshots taken in the previous state to determine if a stall has occured. If a stall has occured the high-level commands will be disabled and the robot will turn off its motors. The final state will make the robot go in reverse for a few seconds. After this time has passed the robot will then be able to receive high-level commands and continue it route.

## IMU.h
IMU.h is the API used read absolute orientation of the robot from the IMU.

* **IMU my_IMU** - Creates a Motors object using the pins connected from the H-bridge to the Teensy.
  **my_imu.read_IMUmsg_data()** - Returns absolute orientation quaternion w and z components in a tuple.

