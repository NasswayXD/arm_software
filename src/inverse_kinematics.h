#pragma once
#include <tuple>
#include <cmath>
#include <ESP32Servo.h>


extern Servo wrist;
extern Servo claw;
extern float link_1, link_2, wrist_link;

std::tuple<float, float, float> angles(float x, float y, float z, float angle_of_attack);
bool reachable (float x, float y, float z);
std::tuple<float, float, float>
forward_kinematics(float curr_link1, float curr_link2, float curr_base, float wrist_angle);

void set_wrist(float angle);
void gripper_control(float percentage);