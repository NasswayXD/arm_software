#pragma once
#include <Arduino.h>
#include <cmath>
#define LIMIT_SWITCH_SAFETY_ONE 15
#define LIMIT_SWITCH_SAFETY_TWO 32//link 1
#define LIMIT_SWITCH_SAFETY_THREE 16
#define LIMIT_SWITCH_SAFETY_FOUR 33


bool danger(float target_deg_link_1, float target_deg_link_2);