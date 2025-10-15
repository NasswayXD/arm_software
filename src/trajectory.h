#include <math.h>
struct Vec3 {
  float x, y, z;


  Vec3() : x(0), y(0), z(0) {}
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}


  Vec3 operator+(const Vec3& b) const { return {x + b.x, y + b.y, z + b.z}; }
  Vec3 operator-(const Vec3& b) const { return {x - b.x, y - b.y, z - b.z}; }
  Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
  Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }

  Vec3& operator+=(const Vec3& b) { x += b.x; y += b.y; z += b.z; return *this; }

  
  float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
  Vec3 cross(const Vec3& b) const {
    return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
  }

  
  float norm() const { return sqrtf(x*x + y*y + z*z); }
  Vec3 normalized() const {
    float n = norm();
    return (n > 1e-6f) ? *this / n : Vec3(0,0,0);
  }
};


void traj_start(const Vec3& p0, const Vec3& p1, float vmax, float amax);

void traj_start_with_v(const Vec3& p0, const Vec3& p1, float vmax, float amax,float v0, float v_end);



bool traj_step(float dt, Vec3& p_out);


bool  traj_active();
float traj_time();      
float traj_total_time(); 
float traj_progress();   
void  traj_cancel();