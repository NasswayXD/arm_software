
#include "safety.h"



bool danger(float target_deg_link_1, float target_deg_link_2) {
  
 
  if (target_deg_link_1 > 180.0f || target_deg_link_2 > 180.0f) return true;
  if (target_deg_link_1 < 0.0f   || target_deg_link_2 < -60.0f) return true;

  return false;
}
