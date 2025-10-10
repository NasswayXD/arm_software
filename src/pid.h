#pragma once
#include <Arduino.h>


constexpr int PIN_esc_One = 19;
constexpr int PIN_esc_Two = 18;
constexpr int PIN_SDA_One = 21;
constexpr int PIN_SCL_One = 22;
constexpr int PIN_SDA_Two = 25;
constexpr int PIN_SCL_Two = 26;

constexpr uint32_t PRINT_EVERY_MS = 150;
constexpr float I_MIN = -50.0f;
constexpr float I_MAX = +50.0f;


extern float Kp;
extern float Kd;
extern float Ki;

struct PIDState {
  float i;
  float last_e;
  uint32_t last_us;
};




extern float target_deg_link_1;
extern float target_deg_link_2;

float applyDeadband(float x, float db = 0.04f);
float PID_step_position_state(float target_deg, float meas_deg_in, PIDState& s);
void  PID_reset_state(PIDState& s);
