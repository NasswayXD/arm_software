#include "pid.h"
#include <math.h>

float Kp = 0.0027f;
float Kd = 0.00024f;
float Ki = 0.0010f;

static inline float constrainf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float applyDeadband(float x, float db) {
  if (fabsf(x) < db) return 0.0f;
  return (x > 0) ? (x - db) / (1.0f - db) : (x + db) / (1.0f - db);
}

float PID_step_position_state(float target_deg_in, float meas_deg_in, float dt, PIDState& s) {
  if (dt <= 0.0f) dt = 1e-4f;             

  float e = target_deg_in - meas_deg_in;

 
  s.i += e * dt;
  s.i = constrainf(s.i, I_MIN, I_MAX);

 
  float dedt = (e - s.last_e) / dt;
  s.last_e = e;

  float u = Kp * e + Ki * s.i + Kd * dedt;
  u = constrainf(u, -0.6f, 0.6f);
  return applyDeadband(u, 1.0f);
}

void PID_reset_state(PIDState& s) {
  s.i = 0.0f;
  s.last_e = 0.0f;
}
