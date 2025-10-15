#pragma once
#include <Arduino.h>
#include <Wire.h>


struct AS5600Enc {
  TwoWire* wire;          
  uint8_t  addr;          
  float    gear;          

  int32_t  turns = 0;
  uint16_t last_raw = 0;
  bool     seeded  = false;

  uint16_t r1 = 0, r2 = 0, r3 = 0;
};


void enc_begin(AS5600Enc& e, TwoWire& w, int sda, int scl,
               uint32_t clock_hz = 100000, uint8_t addr = 0x36,
               float gear = 1.0f, bool do_begin = true);
bool enc_seed(AS5600Enc& e);        
bool enc_read(AS5600Enc& e);        

inline float rawToDeg(uint16_t raw) { return raw * (360.0f / 4096.0f); }
float enc_motor_deg(const AS5600Enc& e);   
float enc_output_deg(const AS5600Enc& e);  

static inline float angleErrDeg(float target, float meas) { return target - meas; }