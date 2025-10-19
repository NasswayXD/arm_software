#include <math.h>
#include "trajectory.h"

static Vec3  g_p0, g_p1, g_dir; 
static float g_L = 0.0f;        // path length
static float g_vmax = 0.0f;
static float g_amax = 0.0f;
static float g_ta = 0.0f;       // accel time
static float g_tc = 0.0f;       // cruise time
static float g_T  = 0.0f;       // total time
static float g_t  = 0.0f;       // current elapsed time
static bool  g_running = false;


static float g_v0   = 0.0f;     // starting velocity
static float g_vend = 0.0f;     // target end velocity


static inline void eval_scalar(float tt, float& s, float& sdot) {
  if (g_L < 1e-6f || g_T <= 0.0f) { s = 0.0f; sdot = 0.0f; return; }

  const float t1 = g_ta;        // end of accel
  const float t2 = g_ta + g_tc; // end of cruise

  if (tt <= 0.0f) { s = 0.0f; sdot = g_v0; return; }

  if (tt < t1) {
    // accelerate
    s    = g_v0 * tt + 0.5f * g_amax * tt * tt;
    sdot = g_v0 + g_amax * tt;
    return;
  }

  if (tt < t2) {
    // cruise
    const float da = g_v0 * g_ta + 0.5f * g_amax * g_ta * g_ta;
    s    = da + g_vmax * (tt - g_ta);
    sdot = g_vmax;
    return;
  }

  if (tt < g_T) {
    // decelerate
    const float td = (g_T - tt);
    s    = g_L - 0.5f * g_amax * td * td;
    sdot = g_amax * td;
    return;
  }

  s = g_L;
  sdot = g_vend;
}

void traj_start(const Vec3& start, const Vec3& finish, float vmax, float amax) {
  g_p0 = start;
  g_p1 = finish;
  g_vmax = vmax;
  g_amax = amax;

  const Vec3 d = g_p1 - g_p0;
  g_L = d.norm();

  if (g_L < 1e-6f || g_vmax <= 0.0f || g_amax <= 0.0f) {
    g_dir = Vec3(0,0,0);
    g_ta = g_tc = g_T = g_t = 0.0f;
    g_running = false;
    return;
  }

  g_dir = d / g_L;

  const float ta_try = g_vmax / g_amax;
  const float da     = 0.5f * g_amax * ta_try * ta_try;

  if (2.0f * da <= g_L) {
    // trapezoid
    g_ta = ta_try;
    g_tc = (g_L - 2.0f * da) / g_vmax;
    g_T  = 2.0f * g_ta + g_tc;
  } else {
    // triangular
    const float vp = sqrtf(g_amax * g_L);
    g_ta = vp / g_amax;
    g_tc = 0.0f;
    g_T  = 2.0f * g_ta;
  }

  g_t = 0.0f;
  g_running = true;
}

void traj_start_with_v(const Vec3& p0, const Vec3& p1, float vmax, float amax,float v0, float v_end)
{
  g_v0   = fmaxf(0.0f, v0);
  g_vend = fmaxf(0.0f, v_end);
  traj_start(p0, p1, vmax, amax);
}


bool traj_step(float dt, Vec3& p_out) {
  if (!g_running) { p_out = g_p1; return true; }

  if (dt <= 0.0f) dt = 1e-4f;
  g_t += dt;
  if (g_t > g_T) g_t = g_T;

  float s, sdot;
  eval_scalar(g_t, s, sdot);

  
  if (sdot < g_vend) sdot = g_vend;

  p_out = g_p0 + g_dir * s;

  const bool done = (g_t >= g_T - 1e-6f);
  if (done) g_running = false;
  return done;
}

bool  traj_active()     { return g_running; }
float traj_time()       { return g_t; }
float traj_total_time() { return g_T; }

float traj_progress() {
  if (g_L < 1e-6f) return 1.0f;
  float s, v;
  eval_scalar(g_t, s, v);
  float u = s / g_L;
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;
  return u;
}

void traj_cancel() {
  g_t = g_T;
  g_running = false;
}
