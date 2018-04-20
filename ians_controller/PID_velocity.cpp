#include "Arduino.h"
#include "Motors.cpp"

PID_velocity::PID_velocity(){
  pid_error = 0;
  pid_target = 0;
  pid_motor = 0;
  pid_vel = 0;
  pid_intergral = 0;
  pid_derivative = 0;
  pid_previous_error = 0;
  wheel_prev = 0;
  wheel_latest = 0;
  something_then = millis();
  wheel_mult = 0;
  prev_encoder = 0;

  prev_pid_time = millis();
  pid_Kp = 10;
  pid_Ki = 10;
  pid_Kd = 0.001;
  out_min = -255;
  out_max = 255;
  rate = 30;
  rolling_pts = 2;
  timeout_ticks = 4;
  ticks_per_meter = 20;
  velocity_threshold = 0.001;
  encoder_min = -32768;
  encoder_max = 32768;
  encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min;
  pid_prev_vel = [0.0] * rolling_pts; /////////////////
  wheel_latest = 0.0;
}

void calc_velocity(){

  
}



void do_pid(){
  unsigned long pid_dt_duration = millis() - prev_pid_time; 
  unsigned float pid_dt = pid_dt_duration * 1000;
  prev_pid_time = millis();

  pid_error = pid_target - pid_vel;
  pid_intergral = pid_intergral + (pid_error * pid_dt); // might need to be pid_dt
  derivative = (pid_error - pid_previous_error) / pid_dt; // migt need to be pid_dt
  pid_previous_error = pid_error;

  pid_motor = (pid_Kp * pid_error) + (pid_Ki * pid_intergral) + (pid_Kd * pid_derivative);

  if(pid_motor > out_max){
    pid_motor = out_max;
    pid_intergral = pid_intergral - (pid_error * pid_dt) //might need to be pid_dt 
  } else if(pid_motor < out_min){
    pid_motor = out_min;
    pid_intergral = pid_intergral - (pid_error * pid_dt) //might need to be pid_dt
  } else if(pid_target == 0){
      pid_motor = 0;
  }  

  Serial.Print("Velocity: ");
  Serial.Print(pid_vel);
  Serial.Print(" Target: ");
  Serial.Print(pid_target);
  Serial.Print(" Error: ");
  Serial.Print(pid_error);
  Serial.Print(" Intergral: ");
  Serial.Print(pid_intergral);
  Serial.Print(" Derivative: ");
  Serial.Print(pid_derivative);
  Serial.Print(" Motor: ");
  Serial.Print(pid_motor);
}

