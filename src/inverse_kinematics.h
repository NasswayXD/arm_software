#pragma once
#include <tuple>
#include <cmath>
extern float link_1, link_2;

std::tuple<float, float, float> angles(float x, float y, float z);
bool reachable (float x, float y, float z);


