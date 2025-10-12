#pragma once
#include <tuple>
#include <cmath>
#include <ESP32Servo.h>


extern Servo wrist;
extern float link_1, link_2, wrist_link;

std::tuple<float, float, float> angles(float x, float y, float z, float angle_of_attack);
bool reachable (float x, float y, float z);

void set_wrist(float angle);
