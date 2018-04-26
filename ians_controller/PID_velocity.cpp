#include "Arduino.h"
#include "PID_velocity.h"

PID_velocity::PID_velocity(int PWM_PIN,int MOTOR_EN1,int MOTOR_EN2,float Kd,float Kp,float Ki,int timeout_tick) : motor(PWM_PIN, MOTOR_EN1, MOTOR_EN2){
    pinMode(13, OUTPUT); // TODO: Is this needed?
    pid_error = 0;
    pid_target = 0;
    pid_motor = 0;
    vel = 0;
    pid_integral = 0;
    pid_derivative = 0;
    pid_previous_error = 0;
    wheel_prev = 0;
    wheel_mult = 0;
    prev_encoder = 0;

    then = millis();
    prev_pid_time = millis();
    pid_Kp = Kp;
    pid_Ki = Ki;
    pid_Kd = Kd;
    out_min = -255;
    out_max = 255;
    rate = 30;
    timeout_ticks = timeout_tick; // TODO: Delete this
    ticks_per_meter = 1527.88; // TODO: This should not be hardcoded, but passed in 
    velocity_threshold = 0.001;
    encoder_min = -32768;
    encoder_max = 32768;
    encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min; // 10k
    encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min; // 22k
    wheel_latest = 0.0;   
//    prev_vel[ROLLING_PTS] = { 0 }; // all elements 0
}

void PID_velocity::calc_velocity() {
    // check how much time has elapsed
    double dt_duration = millis() - then; // In milliseconds
//    Serial.print("Inside calc vel, current dt: ");
//    Serial.println(dt_duration);
    double dt = dt_duration / 1000; // Convert to seconds
    double cur_vel;
    
    if (wheel_latest == wheel_prev) {
        Serial.println("No new wheels ticks found in calc velocity");
        cur_vel = (1.0 / (double)ticks_per_meter) / dt;
//            Serial.print("cur_vel");
//           Serial.println(cur_vel);
        if (abs(cur_vel) < velocity_threshold) { // too slow
//            Serial.println("IF too slow");
            append_vel(0);
            calc_rolling_vel();
        } else { // moving
//            Serial.println("No wheel ticks found but simulated vel is high or something");
            if ((vel >= 0 && vel > cur_vel && cur_vel >= 0) ||
                    (vel < 0 && vel < cur_vel && cur_vel <= 0)) {
                append_vel(cur_vel);
                calc_rolling_vel();
            }
        }
    } else {      
//        Serial.println("Found a tick inside calc velocity, calculating...");
        cur_vel = (wheel_latest - wheel_prev) / dt;
        append_vel(cur_vel);
        calc_rolling_vel();
        wheel_prev = wheel_latest;
        then = millis();
    }    
}

// The prev_vel array is modeled as a FILO queue: elements are added at
// index 0, and fall off the array at index ROLLING_PTS
void PID_velocity::append_vel(double val) {
//    Serial.print("Inside append vel: ");
//    Serial.println(val);
    for (int i = 1; i < ROLLING_PTS; i++) {
        prev_vel[i] = prev_vel[i - 1];
    }
    prev_vel[0] = val;
}

void PID_velocity::calc_rolling_vel() {
    float sum = 0;
    for (int i = 0; i < ROLLING_PTS; i++) {
        sum += prev_vel[i];
    }
    vel = sum / ROLLING_PTS;
}

void PID_velocity::do_pid() {
    unsigned long pid_dt_duration = millis() - prev_pid_time;
    double pid_dt = pid_dt_duration / 1000.0; // Must cast to float, otherwise int division
    prev_pid_time = millis();
    Serial.println("*********");
//    Serial.print("Inside do pid, current dt: (milliseconds) ");
//    Serial.println(pid_dt_duration);

    pid_error = pid_target - vel;
    pid_integral = pid_integral + (pid_error * pid_dt); 
    pid_derivative = (pid_error - pid_previous_error) / pid_dt;
    pid_previous_error = pid_error;
    
    pid_motor = (pid_Kp * pid_error) + (pid_Ki * pid_integral) + (pid_Kd * pid_derivative);

    Serial.print("Your current velocity is ");
    Serial.println(vel,5);
//    Serial.print("pid_error ");
//    Serial.println(pid_error,5);
//    Serial.print("pid_integral ");
//    Serial.println(pid_integral,5);
//    Serial.print("Output to the motor is ");
//    Serial.println(pid_motor,5);
    
    if (pid_motor > out_max) {
        pid_motor = out_max;
        pid_integral = pid_integral - (pid_error * pid_dt); 
    } else if (pid_motor < out_min) {
        pid_motor = out_min;
        pid_integral = pid_integral - (pid_error * pid_dt); 
    } 

    if(pid_target > 0) { //prevents neg output for pos target
      pid_motor = std::max(0.0,pid_motor);
    } else if(pid_target < 0) { // prevents pos output for neg target
      pid_motor = std::min(pid_motor, 0.0);
    } else if (pid_target == 0) {
      pid_motor = 0;
    }
}

void PID_velocity::cumulative_enc_val(int enc) {
    if (enc < encoder_low_wrap && prev_encoder > encoder_high_wrap) {
        wheel_mult++;
    }
    if (enc > encoder_high_wrap && prev_encoder < encoder_low_wrap) {
        wheel_mult--;
    }

    wheel_latest = 1.0 * (enc + wheel_mult * (encoder_max - encoder_min)) / ticks_per_meter;
    prev_encoder = enc;
}

void PID_velocity::pid_spin(std_msgs::Float32 target_msg) { // TODO: Take a float instead of a msg, unpack the message higher up the stack
  if (ticks_since_target > timeout_ticks) { // This will never happen! Delete this
    then = millis();
    Serial.println("Just reset the 'then' variable!");
    
    ticks_since_target = 0;
    // ticks_since_target = timeout_ticks;
    wheel_prev = wheel_latest;

    pid_previous_error = pid_integral = pid_error = pid_derivative = 0;
  }
    

    std_msgs::Float32 motor_msg; // TODO: Put this in a class variable
    pid_target = target_msg.data;
      
    calc_velocity();
    do_pid();
    motor_msg.data = pid_motor;
    motor.motor_cmd(motor_msg);
//      ticks_since_target += 1;
      
//      Serial.print("Current signal to motor ");
//      Serial.println(motor_msg.data);
      
//      if (ticks_since_target == timeout_ticks) {
//          motor_msg.data = 0;
//          motor.motor_cmd(motor_msg);
//      }    
}

void PID_velocity::test_motor_control(std_msgs::Float32 msg){
  motor.motor_cmd(msg);
}

