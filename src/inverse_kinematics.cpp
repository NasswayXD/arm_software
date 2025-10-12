#include "inverse_kinematics.h"
const float pi = 3.14;
Servo wrist;
float link_1 = 0.2, link_2= 0.15, wrist_link = 0.08; // all the measurements are in m, may be changed later

std::tuple<float, float, float> angles(float x, float y, float z, float angle_of_attack){ // all the IK lives here 

    x = x-wrist_link*cos(angle_of_attack);
    y = y-wrist_link*sin(angle_of_attack);
    float xy_plane_alpha = atan(y/x) * (180/pi); // angle for base 
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
   


bool reachable (float x, float y, float z){ //to check if reachable 
    
    double h = sqrt(pow(x,2)+pow(z,2)+pow(y,2));
    return h<(link_1+link_2);
}