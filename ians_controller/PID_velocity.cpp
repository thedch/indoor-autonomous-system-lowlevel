#include "Arduino.h"
#include "Motors.cpp"

PID_velocity::PID_velocity() {
  pid_error = 0;
  pid_target = 0;
  pid_motor = 0;
  pid_vel = 0;
  pid_intergral = 0;
  pid_derivative = 0;
  pid_previous_error = 0;
  wheel_prev = 0;
  wheel_latest = 0;
  then = millis();
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
  // velocity_threshold = 0.001;
  encoder_min = -32768;
  encoder_max = 32768;
  encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min; // 10k
  encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min; // 22k
  std::deque<int> pid_prev_vel (8,0); 
  // pid_prev_vel = [0.0] * rolling_pts;
  wheel_latest = 0.0;
}

void PID_velocity::calc_velocity() {  
  // check how much time has elapsed
  float dt_duration = millis() - then; 
  float dt = dt_duration * 1000;

  // if no recent odometry data
    // calc current velocity
    // if below threshold
      // set vel = 0
    // else
      // ??

  // if we HAVE recent odom data
  int cur_vel = (wheel_latest - wheel_prev) / dt
  append_vel(cur_vel);
  calc_rolling_vel();
  wheel_prev = wheel_latest;
  then = millis(); // update last_velocity_timestamp

  // publish current vel?
  
}

void PID_velocity::append_vel(int val) { // Add velocity to history
  pid_prev_vel.push_front(val);
  pid_prev_vel.pop_back();  
}

void PID_velocity::calc_rolling_vel() {
  int sum = 0
  for (int i = 0; i < pid_prev_vel.size(); i++) {
    sum += pid_prev_vel[i];
  }
  return float(sum/pid_prev_vel.size());
}



void PID_velocity::do_pid() {
  unsigned long pid_dt_duration = millis() - prev_pid_time; 
  unsigned float pid_dt = pid_dt_duration / 1000;
  prev_pid_time = millis();

  pid_error = pid_target - pid_vel;
  pid_intergral = pid_intergral + (pid_error * pid_dt); // might need to be pid_dt
  derivative = (pid_error - pid_previous_error) / pid_dt; // migt need to be pid_dt
  pid_previous_error = pid_error;

  pid_motor = (pid_Kp * pid_error) + (pid_Ki * pid_intergral) + (pid_Kd * pid_derivative);

  if (pid_motor > out_max){
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

void PID_velocity::wheelCallback(enc) {
  // int enc = msg.data;
  if (enc < encoder_low_wrap && prev_encoder > encoder_high_wrap) {
    wheel_mult++;
  }
  if (enc > encoder_high_wrap && prev_encoder < encoder_low_wrap) {
    wheel_mult--;
  }

  wheel_latest = 1.0 * (enc + wheel_mult * (encoder_max - encoder_min)) / ticks_per_meter;
  prev_encoder = enc;
}

void PID_velocity::targetCallback(msg) {
  // When we recieve a geo twist, this happens
  target = msg.data;
  calc_velocity();
  do_pid();
}
