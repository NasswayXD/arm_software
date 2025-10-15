#include "inverse_kinematics.h"
const float pi = 3.14;
Servo wrist;
Servo claw;

float link_1 = 0.2, link_2= 0.15, wrist_link = 0.08; // all the measurements are in m, may be changed later

std::tuple<float, float, float> angles(float x, float y, float z, float angle_of_attack){ // all the IK lives here 

    const float d2r = (pi / 180.0f);
    float aoa = angle_of_attack * d2r;  // degrees -> radians

    x = x - wrist_link * cosf(aoa);
    y = y - wrist_link * sinf(aoa);

    float xy_plane_alpha = atan2f(y, x) * (180.0f / pi);

    float l = sqrt(pow(x,2)+pow(y,2)); 
    float theta = acos(((sqrt(pow(z,2)+pow(l,2)))/2)/link_1)* (180/pi);
    float pheta = atan(z/l)* (180/pi);
    float zl_plane_alpha1 = theta + pheta;
    float zl_plane_alpha2 = pheta -theta;

    return {xy_plane_alpha, zl_plane_alpha1, zl_plane_alpha2};
}
void set_wrist(float angle)
{
    wrist.write(angle);
}  
void gripper_control(float percentage) {
  // 0 = open, 90 = closed
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;
  float deg = (percentage * 90.0f) / 100.0f;  
  claw.write(deg);
}

std::tuple<float, float, float>
forward_kinematics(float curr_link1, float curr_link2, float curr_base, float wrist_angle)
{
    const float d2r = (pi / 180.0f);
    float a1   = curr_link1 * d2r;  
    float a2   = curr_link2 * d2r; 
    float base = curr_base  * d2r; 

    float r = link_1 * cosf(a1) + link_2 * cosf(a2);
    float z = link_1 * sinf(a1) + link_2 * sinf(a2);

    float xw = r * cosf(base);
    float yw = r * sinf(base);

    
    float w = wrist_angle * d2r; 
    float xe = xw + wrist_link * cosf(w);
    float ye = yw + wrist_link * sinf(w);

    float ze = z;

    return {xe, ye, ze};
}
   


bool reachable (float x, float y, float z){ //to check if reachable 
    
    double h = sqrt(pow(x,2)+pow(z,2)+pow(y,2));
    return h<(link_1+link_2);
}