#pragma once
#include <Arduino.h>

extern long     turn_count;
extern uint16_t last_raw;

extern const uint8_t AS5600_ADDR;
extern const uint8_t REG_RAW_ANGLE_H;
extern const uint8_t REG_RAW_ANGLE_L;
extern const uint8_t REG_STATUS;



bool readRawAngle12(uint16_t &raw) ;
inline float rawToDeg(uint16_t raw);

static inline float angleErrDeg(float target, float meas) { return target - meas; }

float motorDegrees(uint16_t raw);
float outputDegrees(uint16_t raw);
void updateTurnCounter(uint16_t raw_now);