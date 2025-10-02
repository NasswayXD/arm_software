#pragma once
#include <Arduino.h>
#include <Wire.h>

// One instance per AS5600
struct AS5600Enc {
  TwoWire* wire;          // Wire or Wire1
  uint8_t  addr;          // 0x36
  float    gear;          // output = motor/gear

  // state
  int32_t  turns = 0;
  uint16_t last_raw = 0;
  bool     seeded  = false;

  // tiny median filter
  uint16_t r1 = 0, r2 = 0, r3 = 0;
};

// ---- lifecycle ----
void enc_begin(AS5600Enc& e, TwoWire& w, int sda, int scl,
               uint32_t clock_hz = 100000, uint8_t addr = 0x36,
               float gear = 1.0f, bool do_begin = true);
bool enc_seed(AS5600Enc& e);        // call once after boot
bool enc_read(AS5600Enc& e);        // updates e.last_raw/turns

// ---- math helpers ----
inline float rawToDeg(uint16_t raw) { return raw * (360.0f / 4096.0f); }
float enc_motor_deg(const AS5600Enc& e);   // multi-turn motor degrees
float enc_output_deg(const AS5600Enc& e);  // output degrees (geared)

// (kept for your PID interface if you use it)
static inline float angleErrDeg(float target, float meas) { return target - meas; }

