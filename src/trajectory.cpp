#include "trajectory.h"


Vec3 p_start(0.1f, 0.1f, 0.0f);
Vec3 p_end(0.25f, 0.2f, 0.1f);


Vec3 displacement = p_start - p_end;
float L = displacement.norm();

Vec3 direction = displacement.normalized();