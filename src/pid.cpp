#include "pid.h"

float Kp = 0.0027f;
float Kd = 0.00024f;
float Ki = 0.0010f;

float pid_i = 0.0f;
float pid_last_e = 0.0f;
uint32_t pid_last_us = 0;

float target_deg_link_1 = 0.0f;
float target_deg_link_2 = 0.0f;
float applyDeadband(float x, float db) { //range around setpoint pid ignores diviations (tolerance)
  if (fabsf(x) < db) return 0.0f;
  return (x > 0) ? (x - db) / (1.0f - db) : (x + db) / (1.0f - db);
}

float PID_step_position(float target_deg, float meas_deg_in) { // all the PID lives here 
  uint32_t now = micros();
  if (pid_last_us == 0) {
    pid_last_us = now;
  } 
  float dt = (now - pid_last_us) / 1.0e6f;
  if (dt <= 0) {
    dt = 1e-4f;
  }
  pid_last_us = now;   // calculating dt

  float e = target_deg - meas_deg_in; // for Kp

  pid_i += e * dt;
  pid_i = constrain(pid_i, I_MIN, I_MAX); //integrating 

  float dedt = (e - pid_last_e) / dt;
  pid_last_e = e; //for Kd

  float u = Kp * e + Ki * pid_i + Kd * dedt;
  u = constrain(u, -0.6f, 0.6f);
  u = applyDeadband(u, 0.04f);
  return u;
}

void PID_reset() { //resetting PID
  pid_i = 0.0f;
  pid_last_e = 0.0f;
  pid_last_us = micros();
}


