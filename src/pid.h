#pragma once
#include <Arduino.h>


constexpr int PIN_escL = 19;
constexpr int PIN_escR = 18;
constexpr int PIN_SDAL = 21;
constexpr int PIN_SCLL = 22;
constexpr int PIN_SDAR = 17;
constexpr int PIN_SCLR = 16;

constexpr uint32_t PRINT_EVERY_MS = 150;
constexpr float I_MIN = -50.0f;
constexpr float I_MAX = +50.0f;


extern float Kp;
extern float Kd;
extern float Ki;

extern float pid_i;
extern float pid_last_e;
extern uint32_t pid_last_us;

extern float target_deg_link_1;
extern float target_deg_link_2;

float applyDeadband(float x, float db = 0.04f);
float PID_step_position(float target_deg, float meas_deg_in);
void  PID_reset();

