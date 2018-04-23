#include "Arduino.h"
#include "PID_velocity.h"

PID_velocity::PID_velocity() {
    // Serial.begin(9600);
    pid_error = 0;
    pid_target = 0;
    pid_motor = 0;
    vel = 0;
    pid_intergral = 0;
    pid_derivative = 0;
    pid_previous_error = 0;
    wheel_prev = 0;
    wheel_latest = 0;
    wheel_mult = 0;
    prev_encoder = 0;

    then = millis();
    prev_pid_time = millis();
    pid_Kp = 10.0f;
    pid_Ki = 10.0f;
    pid_Kd = 0.001f;
    out_min = -255;
    out_max = 255;
    rate = 30;
    timeout_ticks = 4;
    ticks_per_meter = 20;
    velocity_threshold = 0.001;
    encoder_min = -32768;
    encoder_max = 32768;
    encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min; // 10k
    encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min; // 22k
    wheel_latest = 0.0;
    rolling_pts = 2;
    prev_vel[ROLLING_PTS] = { 0 }; // all elements 0
}

void PID_velocity::calc_velocity() {
    // check how much time has elapsed
    double dt_duration = millis() - then;
    double dt = dt_duration / 1000;
    double cur_vel;

    if (wheel_latest == wheel_prev) {
        cur_vel = 1.0 / (double)ticks_per_meter / dt;
        if (abs(cur_vel) < velocity_threshold) { // too slow
            append_vel(0);
            calc_rolling_vel();
        } else { // moving
            if ((vel >= 0 && vel > cur_vel && cur_vel >= 0) ||
                    (vel < 0 && vel < cur_vel && cur_vel <= 0)) {
                append_vel(cur_vel);
                calc_rolling_vel();
            }

        }

    } else {
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
    for (int i = 1; i < ROLLING_PTS; i++) {
        prev_vel[i] = prev_vel[i - 1];
    }
    prev_vel[0] = val;
}

void PID_velocity::calc_rolling_vel() {
    int sum = 0;
    for (int i = 0; i < ROLLING_PTS; i++) {
        sum += prev_vel[i];
    }
    vel = sum / ROLLING_PTS;
}

void PID_velocity::do_pid() {
    unsigned long pid_dt_duration = millis() - prev_pid_time;
    float pid_dt = pid_dt_duration / 1000;
    prev_pid_time = millis();

    pid_error = pid_target - vel;
    pid_intergral = pid_intergral + (pid_error * pid_dt); // might need to be pid_dt
    pid_derivative = (pid_error - pid_previous_error) / pid_dt; // migt need to be pid_dt
    pid_previous_error = pid_error;

    pid_motor = (pid_Kp * pid_error) + (pid_Ki * pid_intergral) + (pid_Kd * pid_derivative);

    if (pid_motor > out_max) {
        pid_motor = out_max;
        pid_intergral = pid_intergral - (pid_error * pid_dt); //might need to be pid_dt
    } else if (pid_motor < out_min) {
        pid_motor = out_min;
        pid_intergral = pid_intergral - (pid_error * pid_dt); //might need to be pid_dt
    } else if (pid_target == 0) {
        pid_motor = 0;
    }
}

void PID_velocity::wheelCallback(int enc) { // TODO: This is not a callback, it's just a function. Rename this.
    if (enc < encoder_low_wrap && prev_encoder > encoder_high_wrap) {
        wheel_mult++;
    }
    if (enc > encoder_high_wrap && prev_encoder < encoder_low_wrap) {
        wheel_mult--;
    }

    wheel_latest = 1.0 * (enc + wheel_mult * (encoder_max - encoder_min)) / ticks_per_meter;
    prev_encoder = enc;
}

void PID_velocity::targetCallback(std_msgs::Float32 msg) {
    // When we receive a geom twist, this happens
    pid_target = msg.data;
    ticks_since_target = 0;
    calc_velocity();
    do_pid();
}

void PID_velocity::pid_spin() {
    then = millis();
    ticks_since_target = timeout_ticks;
    wheel_prev = wheel_latest;

    pid_previous_error = pid_intergral = pid_error = pid_derivative = 0;

    std_msgs::Float32 motor_msg;
    while (ticks_since_target < timeout_ticks) {
        calc_velocity();
        do_pid();
        motor_msg.data = pid_motor;
        //publish motors

        ticks_since_target += 1;
        if (ticks_since_target == timeout_ticks) {
            motor_msg.data = 0;
            //publish motors
        }
    }
}

