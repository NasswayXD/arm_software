#include "encoder.h"

static bool i2cReadReg(TwoWire& w, uint8_t addr, uint8_t reg, uint8_t& val, int retries = 3) {
  while (retries--) {
    w.beginTransmission((uint8_t)addr);
    w.write((uint8_t)reg);

    uint8_t err = w.endTransmission(true);
    if (err == 0) {
      uint8_t got = w.requestFrom((uint16_t)addr, (uint8_t)1, (bool)true);
      if (got == 1) { val = w.read(); return true; }
    }

    delay(2);
  }
  return false;
}

static bool readRawAngle12(AS5600Enc& e, uint16_t& raw) {
  uint8_t hi = 0, lo = 0;
  if (!i2cReadReg(*e.wire, e.addr, 0x0C, hi)) return false; 
  if (!i2cReadReg(*e.wire, e.addr, 0x0D, lo)) return false;
  raw = (uint16_t)(((uint16_t)hi << 8) | lo) & 0x0FFF;
  return true;
}
void enc_begin(AS5600Enc& e, TwoWire& w, int sda, int scl,
               uint32_t clock_hz, uint8_t addr, float gear, bool do_begin) {
  e.wire     = &w;
  e.addr     = addr;
  e.gear     = gear;
  e.turns    = 0;
  e.last_raw = 0;
  e.seeded   = false;
  e.r1 = e.r2 = e.r3 = 0;

  if (do_begin) {
    w.begin(sda, scl, clock_hz);
  }
}

bool enc_seed(AS5600Enc& e) {
  uint16_t r;
  if (!readRawAngle12(e, r)) return false;
  e.last_raw = r;
  e.r1 = e.r2 = e.r3 = r;
  e.turns = 0;
  e.seeded = true;
  return true;
}

bool enc_read(AS5600Enc& e) {
  uint16_t r;
  if (!readRawAngle12(e, r)) return false;

  e.r1 = e.r2; e.r2 = e.r3; e.r3 = r;
  uint16_t a = e.r1, b = e.r2, c = e.r3;
  if (a > b) { auto t = a; a = b; b = t; }
  if (b > c) { auto t = b; b = c; c = t; }
  if (a > b) { auto t = a; a = b; b = t; }
  uint16_t rm = b;

  if (!e.seeded) { e.last_raw = rm; e.seeded = true; return true; }

  int d = (int)rm - (int)e.last_raw;

  if (d > 3000 || d < -3000) return false;

  if (d > 2048)  { e.turns--; d -= 4096; }
  if (d < -2048) { e.turns++; d += 4096; }

  if (d > 2048 || d < -2048) return false;

  e.last_raw = rm;
  return true;
}

float enc_motor_deg(const AS5600Enc& e) {
  return e.turns * 360.0f + rawToDeg(e.last_raw);
}

float enc_output_deg(const AS5600Enc& e) {
  return enc_motor_deg(e) / e.gear;
}