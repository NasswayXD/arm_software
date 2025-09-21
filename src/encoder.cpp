
#include "encoder.h"
#include <Wire.h>

long     turn_count = 0;
uint16_t last_raw   = 0;

const uint8_t AS5600_ADDR     = 0x36;
const uint8_t REG_RAW_ANGLE_H = 0x0C;
const uint8_t REG_RAW_ANGLE_L = 0x0D;
const uint8_t REG_STATUS      = 0x0B;


bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t &val, int retries=3) { // read the i2c signals 
  while (retries--) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) == 0) {
      if (Wire.requestFrom((int)addr, 1) == 1) {
        val = Wire.read();
        return true;
      }
    }
    delay(2);
  }
  return false;
}

bool readRawAngle12(uint16_t &raw) { //reads the raw angles 
  uint8_t hi, lo;
  if (!i2cReadReg(AS5600_ADDR, REG_RAW_ANGLE_H, hi)) return false;
  if (!i2cReadReg(AS5600_ADDR, REG_RAW_ANGLE_L, lo)) return false;
  raw = ((((uint16_t)hi) << 8) | lo) & 0x0FFF;
  return true;
}


inline float rawToDeg(uint16_t raw) { return (raw * (360.0f / 4096.0f)); } //converst raw to the degrees that we like 

float motorDegrees(uint16_t raw) { //converst raw to the degrees that esp eats 
  return turn_count * 360.0f + rawToDeg(raw);
}

float outputDegrees(uint16_t raw) { //adjust for gear reduction (theoretical gear reduction is 25, but due to torque it is like that, at least it seams like that )
  return motorDegrees(raw) / 38.5f;  // gear scaling
}

void updateTurnCounter(uint16_t raw_now) { // updates turns to have gear reduction consistent  
  int d = (int)raw_now - (int)last_raw;
  if (d > 2048) { 
    turn_count--;
  }    
  if (d < -2048) {
    turn_count++;
  }    
  last_raw = raw_now;
}
