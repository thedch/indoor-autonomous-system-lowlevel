/*
  Motors.cpp -
  Created by Juan Huerta and Kevin Beher
*/

#include "Arduino.h"
#include "motors.h"

/*
| Author: Juan Huerta
| Param: Digital Pins values on the Teensy used as PWM and
|    motor direction and direction enables
| Remark: This constructor sets the pin numbers given as private
|    variables of the class
Motors(int pwm_Pin,int motor_direction_pin1,
int motor_direction_pin2);
*/

Motors::Motors(int pwm_pin, int motor_direction_pin1, int motor_direction_pin2) {
    pinMode(pwm_pin, OUTPUT); //motor PWM pin
    pinMode(motor_direction_pin1, OUTPUT); //motor direction en1
    pinMode(motor_direction_pin2, OUTPUT); //motor direction en2
    PWM_pin = pwm_pin;
    m_dir1 = motor_direction_pin1;
    m_dir2 = motor_direction_pin2;
    timer = 0; // Time variable for wheel
    newTicks = 0; // Used to capture initial right wheel position
    halt_highlevel = 0;
}

/*

*/
void Motors::motor_cmd(std_msgs::Float32 motor_speed) {
    last_motor_cmd = motor_speed.data;
    if (motor_speed.data > 0) { // TODO: No need for a ROS msg here, just take a float
        motor_forward(motor_speed.data);
    } else if (motor_speed.data < 0) {
        motor_reverse(abs(motor_speed.data));
    } else {
        motor_brake();
    }
}

/*
| Author: Juan Huerta
| Param: motor_speed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the right
|    motor and sets directions pins to rotate motors forward
void Motors::motor_forward(float motor_speed);
*/
void Motors::motor_forward(float motor_speed) {
    analogWrite(PWM_pin, motor_speed);
    digitalWrite(m_dir1, HIGH);
    digitalWrite(m_dir2, LOW);
}

/*
| Author: Juan Huerta
| Param: motor_speed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the right
|    motor and sets directions pins to rotate motors forward
void Motors::motor_forward(float motor_speed);
*/
void Motors::motor_reverse(float motor_speed) {
    analogWrite(PWM_pin, motor_speed);
    digitalWrite(m_dir1, LOW);
    digitalWrite(m_dir2, HIGH);
}

/*
| Author: Juan Huerta
| Param: None
| Return: Void
| Remark: This function sets motor direction pins to zero
|    and PWM duty cycle to zero to brake the motor
void Motors::motor_brake();
*/
void Motors::motor_brake() {
    analogWrite(PWM_pin, 0);
    digitalWrite(m_dir1, LOW);
    digitalWrite(m_dir2, LOW);
}

void Motors::check_motor_stall(float curr_enocder_val) {
    Serial.println("Begin State");
    std_msgs::Float32 motorspeed;
///// begin wheel state machine ///
    switch (WheelCurrentState) {
    case (Moving):
        newTicks = curr_enocder_val;
        if  (last_motor_cmd != 0)  { // This state initializes the time and captures wheel position.
            WheelCurrentState = Stalled;
            timer = millis();
        }
        break;

    case (Stalled): // Checks if parameters of stall are met.
        Serial.println("Stalled");
        if ( (curr_enocder_val == newTicks) && (last_motor_cmd != 0) && ((millis() - timer) > 1000) ) {
            WheelCurrentState = TurnOff;
            timer = millis();
        }
        if (curr_enocder_val != newTicks) {
            WheelCurrentState = Moving;
        }
        break;

    case (TurnOff):
        Serial.print("You're in turn off state.");
        halt_highlevel = 1;
        Serial.print("\r\n");
        motorspeed.data = 0;
        motor_cmd(motorspeed);
        if ((millis() - timer) > 2500) {
            halt_highlevel = 2;
            WheelCurrentState = GoBack;
            timer = millis();
        }
        break;
    case (GoBack):
        if ((millis() - timer) > 3000) {
            halt_highlevel = 0;
            WheelCurrentState = Moving;
        }
        break;
    default:
        break;
    }
}
