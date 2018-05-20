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
*/
motors::motors(int pwm_pin, int motor_direction_pin1, int motor_direction_pin2) {
    pinMode(pwm_pin, OUTPUT); //motor PWM pin
    pinMode(motor_direction_pin1, OUTPUT); //motor direction en1
    pinMode(motor_direction_pin2, OUTPUT); //motor direction en2
    PWM_pin = pwm_pin;
    m_dir1 = motor_direction_pin1;
    m_dir2 = motor_direction_pin2;
    timer = 0; // Time variable for wheel stall state machine.
    new_ticks = 0; // Snapshot of encoder value. Used to check for a stall
    halt_highlevel = 0;
}

/*
| Author: Juan Huerta
| Param:  Positive or negative PWM value
| Remark: Calls motor_forward if motor_speed in positive,
|         calls motor_reverse if motor_speed is negative,
|         calls motor_brake if motor_speed is zero
|         updates last_motor_cmd variable with last past motor_speed.
*/
void motors::motor_cmd(float motor_speed) {
    last_motor_cmd = motor_speed;
    if (motor_speed > 0) {
        motor_forward(motor_speed);
    } else if (motor_speed < 0) {
        motor_reverse(abs(motor_speed));
    } else {
        motor_brake();
    }
}

/*
| Author: Juan Huerta
| Param: motor_speed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the right
|    motor and sets directions pins to rotate the motor forward
*/
void motors::motor_forward(float motor_speed) {
    analogWrite(PWM_pin, motor_speed);
    digitalWrite(m_dir1, HIGH);
    digitalWrite(m_dir2, LOW);
}

/*
| Author: Juan Huerta
| Param: motor_speed, 0-255
| Return: Void
| Remark: This function sets PWM duty cycle of the right
|    motor and sets directions pins to rotate the motor in reverse
*/
void motors::motor_reverse(float motor_speed) {
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
*/
void motors::motor_brake() {
    analogWrite(PWM_pin, 0);
    digitalWrite(m_dir1, LOW);
    digitalWrite(m_dir2, LOW);
}

/*
| Author: Kevin Beher and Juan Huerta
| Param: curr_encoder_val
| Return: Void
| Remark: State machine that checks for motor stalling. Does this by checking
|     what the last_motor_cmd is set as and if its not zero and encoder values
|     haven't changed the sets flag that motors have stalled.
*/
void motors::check_motor_stall(float curr_encoder_val) {
    switch (wheel_current_state) {
    case (moving):
        new_ticks = curr_encoder_val;
        if  (last_motor_cmd != 0)  { // This state initializes the time and captures wheel position.
            wheel_current_state = check_for_stall;
            timer = millis();
        }
        break;

    case (check_for_stall): // Checks if parameters of stall are met.
        if ( (curr_encoder_val == new_ticks) && (last_motor_cmd != 0) && ((millis() - timer) > STALL_TIME) ) {
            wheel_current_state = turn_off;
            timer = millis();
        }
        if (curr_encoder_val != new_ticks) {
            wheel_current_state = moving;
        }
        break;

    case (turn_off): // This state disables high level commands and stop motors
        halt_highlevel = 1; // Flag to indicate to turn the motors off.
        Serial.print("\r\n");
        motor_cmd(0);
        if ((millis() - timer) > OFF_TIME) {
            halt_highlevel = 2;  // Flag to indicate motors should go in reverse
            wheel_current_state = moving;
            timer = millis();
        }
        break;
    case (go_back):
        if ((millis() - timer) > BACKUP_TIME) {
            halt_highlevel = 0; // Flag that indicates to enable high level commands
            wheel_current_state = moving;
        }
        break;
    default:
        break;
    }
}
