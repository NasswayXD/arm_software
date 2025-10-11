#include "pid.h"

float Kp = 0.0027f;
float Kd = 0.00024f;
float Ki = 0.0010f;

float pid_i = 0.0f;
float pid_last_e = 0.0f;
uint32_t pid_last_us = 0;

float target_deg_link_1 = 0.0f;

float target_deg_link_2 = 0.0f;
float target_deg_base = 0.0f;
float applyDeadband(float x, float db) { 
  if (fabsf(x) < db) return 0.0f;
  return (x > 0) ? (x - db) / (1.0f - db) : (x + db) / (1.0f - db);
}

#include "pid.h"

float PID_step_position_state(float target_deg_in, float meas_deg_in, PIDState& s) {
  uint32_t now = micros();
  if (s.last_us == 0) s.last_us = now;

  float dt = (now - s.last_us) / 1.0e6f;
  if (dt <= 0) dt = 1e-4f;
  s.last_us = now;

  float e = target_deg_in - meas_deg_in;

  s.i += e * dt;
  s.i = constrain(s.i, I_MIN, I_MAX);

  float dedt = (e - s.last_e) / dt;
  s.last_e = e;

  float u = Kp * e + Ki * s.i + Kd * dedt;
  u = constrain(u, -0.6f, 0.6f);
  return applyDeadband(u, 0.02f);
}

void PID_reset_state(PIDState& s) {
  s.i = 0.0f;
  s.last_e = 0.0f;
  s.last_us = micros();
}



