#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <tuple>
#include "pid.h"
#include "encoder.h"
#include "inverse_kinematics.h"
#include "safety.h"
#include "trajectory.h"
#include <deque>
Servo escOne, escTwo;


AS5600Enc enc_link1;  
AS5600Enc enc_link2;  
static uint32_t last_us_loop = 0;

PIDState pid1{0,0};
PIDState pid2{0,0};
static float wrist_cmd_deg = 90.0f;
static float grip_cmd_pct  = 0.0f;
static constexpr float VMAX = 0.07f;  
static constexpr float AMAX = 0.022f;
static float base_err_deg = 0.0f;  
float curr_base_angle = 0.0f;
bool danger_on = false;
float zero1 = 0.0f, zero2 = 0.0f;
struct Waypoint { 
  Vec3 p; 
  float aoa;      
  float grip;     
  float vmax, amax, rb;
};
static inline float dist3(const Vec3& a, const Vec3& b){
  const float dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z;
  return sqrtf(dx*dx+dy*dy+dz*dz);
}

static void merge_and_thin_queue(std::deque<Waypoint>& q,float min_dist ,float max_aoa_d  = 5.0f,float max_grip_d = 5.0f)   
{
  if (q.size() < 2) return;

  std::deque<Waypoint> out;
  out.push_back(q.front());

  for (size_t i=1; i<q.size(); ++i) {
    Waypoint& last = out.back();
    const Waypoint& cur = q[i];

    bool very_close   = dist3(last.p, cur.p) < min_dist;
    bool aoa_close    = fabsf(last.aoa  - cur.aoa)  <= max_aoa_d;
    bool grip_close   = fabsf(last.grip - cur.grip) <= max_grip_d;

    if (very_close && aoa_close && grip_close) {
      
      last.p.x = 0.5f*(last.p.x + cur.p.x);
      last.p.y = 0.5f*(last.p.y + cur.p.y);
      last.p.z = 0.5f*(last.p.z + cur.p.z);
      last.aoa  = 0.5f*(last.aoa  + cur.aoa);
      last.grip = 0.5f*(last.grip + cur.grip);
      last.vmax = fminf(last.vmax, cur.vmax);
      last.amax = fminf(last.amax, cur.amax);
      last.rb   = fmaxf(last.rb,   cur.rb);
    } else {
    
      out.push_back(cur);
    }
  }

  q.swap(out);
}

#define LIMIT_SWITCH_CALIBRATION_ONE 32 //link 1
#define LIMIT_SWITCH_CALIBRATION_TWO 33 //link 2

static Vec3 goalP(0.1,0.1,0.1);


static float STEPS_PER_REV = 1600.0f; 
float x=0, y=0, z=0, angle = 0, gripper_angle = 0;
bool calibration_executed_one = false;
bool calibration_executed_two = false;
static bool start_traj_request = false;
static bool  test_all_mode = false;
static int   test_idx_seq  = -1;
static std::deque<Waypoint> q;
static float v0_curr  = 0.0f;   
static float v_end_curr = 0.0f;

struct Sample {
  float t;    
  Vec3  p;   
  float aoa;   
  float grip; 
};

static std::deque<Sample> g_buf;     // buffer
static bool  g_playing    = false;
static float g_play_t     = 0.0f;    

static constexpr float PLAN_DT          = 0.005f; 
static constexpr int   MIN_START_SAMPLES= 60;   
static constexpr int   LOW_WATER        = 60; 
float target_deg_link_1, target_deg_link_2, target_deg_base = 0.0f;
static inline float degToSteps(float deg){ return deg * (STEPS_PER_REV / 360.0f); }
static const float BASE_MAX_STEPS_PER_SEC = 2000.0f;  
static const float BASE_KP_STEPS          = 5.0f;     
static void startNextIfIdle(const Vec3& currP){
  if (!traj_active() && !q.empty()){
    const auto& w0 = q.front();
    float v_through = 0.0f;
    if (q.size() >= 2){
      const auto& w1 = q[1];
      float vblend = sqrtf(w0.amax * w0.rb);
      v_through = fminf(fminf(w0.vmax, w1.vmax), vblend);
    }
  
    traj_start_with_v(currP, w0.p, w0.vmax, w0.amax, v0_curr, v_through);
    v_end_curr = v_through;
  }
}

struct TestCase {
  float x, y, z, angle_deg, gripper_percentage;
};

static const TestCase kTests[5] = {
  
  {0.20f, 0.22f, -0.1f, 25.0f, 90.0f}, 
  {0.20f, 0.22f, -0.1f, 25.0f, 40.0f},  // test1

  {0.1f, 0.22f, 0.2f, 25.0f, 35.0f}, 
  {0.10f, 0.05f, 0.0f, 35.0f, 35.0f},
  {0.10f, 0.05f, 0.0f, 35.0f, 90.0f},
};

//90 is closed
// 0 is open 



static inline void apply_wrist_grip(float aoa_deg, float grip_pct){
  if (fabsf(aoa_deg - wrist_cmd_deg) > 0.1f) {
    set_wrist(aoa_deg);
    wrist_cmd_deg = aoa_deg;
  }
  if (fabsf(grip_pct - grip_cmd_pct) > 0.1f) {
    gripper_control(grip_pct);
    grip_cmd_pct = grip_pct;
  }
}
static void runTestCase(int idx) {
  if (idx < 0 || idx >= 5) { Serial.println("Invalid test index"); return; }
  const auto& t = kTests[idx];

  Serial.print("[TEST] x="); 
  Serial.print(t.x);
  Serial.print(" y=");       
  Serial.print(t.y);
  Serial.print(" z=");       
  Serial.print(t.z);
  Serial.print(" angle=");   
  Serial.println(t.angle_deg);

  if (!reachable(t.x, t.y, t.z)) { 
    Serial.println("Test target out of reach!"); 
    return; 
  }
  

  q.push_back(Waypoint{
    Vec3(t.x, t.y, t.z),
    t.angle_deg,
    t.gripper_percentage,
    VMAX, AMAX,
    0.06f
  });
  merge_and_thin_queue(q, 0.04f,4.0f, 5.0f);

  if (!g_playing && q.size() == 1) {
    apply_wrist_grip(q.front().aoa, q.front().grip);
  }
}
static inline void escOneWriteNorm(float x) {
  x = constrain(x, -0.2f, 0.2f); 
  int us = (int)(1500.0f + 500.0f * x);
  escOne.writeMicroseconds(us);

}

static inline float wrap360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a <    0.0f) a += 360.0f;
  return a;
}
inline void stepPulseActiveHigh(uint32_t us_high = 500, uint32_t us_low = 500) {
  digitalWrite(PUL_PIN, HIGH);           
  delayMicroseconds(us_high);
  digitalWrite(PUL_PIN, LOW);    
  delayMicroseconds(us_low);
}
void base_update_nonblocking(float target_angle_deg, float dt) { //basically we do the same as in trajectory planning
  float err = target_angle_deg - curr_base_angle;
  while (err > 180.0f)  err -= 360.0f;
  while (err < -180.0f) err += 360.0f;

  float cmd_steps_per_s = BASE_KP_STEPS * degToSteps(err);
  if (cmd_steps_per_s >  BASE_MAX_STEPS_PER_SEC) cmd_steps_per_s =  BASE_MAX_STEPS_PER_SEC;
  if (cmd_steps_per_s < -BASE_MAX_STEPS_PER_SEC) cmd_steps_per_s = -BASE_MAX_STEPS_PER_SEC;

  static float step_accum = 0.0f;
  step_accum += cmd_steps_per_s * dt;
  int steps_to_emit = (int)step_accum;
  step_accum -= steps_to_emit;
  if (steps_to_emit == 0) return;

  if (steps_to_emit < 0) { 
    digitalWrite(DIR_PIN, LOW);  
    steps_to_emit = -steps_to_emit; 
  }else{ 
    digitalWrite(DIR_PIN, HIGH); 
  }

  const int MAX_PULSES_PER_LOOP = 50;
  if (steps_to_emit > MAX_PULSES_PER_LOOP) steps_to_emit = MAX_PULSES_PER_LOOP;

  for (int i = 0; i < steps_to_emit; ++i) {
    digitalWrite(PUL_PIN, HIGH);  
    delayMicroseconds(3);
    digitalWrite(PUL_PIN, LOW);   
    delayMicroseconds(3);
  }

  float deg_per_step = 360.0f / STEPS_PER_REV;
  curr_base_angle = wrap360(curr_base_angle + (digitalRead(DIR_PIN)==HIGH ? +1 : -1) * steps_to_emit * deg_per_step);
}

static inline void escTwoWriteNorm(float x) {
  x = constrain(x, -0.2f, 0.2f); 
  int us = (int)(1500.0f + 500.0f * x);
  escTwo.writeMicroseconds(us);

}
static void planner_tick(const Vec3& currP) {
  startNextIfIdle(currP);

  while (g_buf.size() < LOW_WATER && traj_active()) {
    Vec3 p;
    bool seg_done = traj_step(PLAN_DT, p);

    float aoa  = (!q.empty() ? q.front().aoa  : angle);
    float grip = (!q.empty() ? q.front().grip : gripper_angle);

    float tstamp = g_buf.empty() ? PLAN_DT : (g_buf.back().t + PLAN_DT);
    g_buf.push_back(Sample{ tstamp, p, aoa, grip });

    if (seg_done) {
      if (!q.empty()) q.pop_front();  
      v0_curr = v_end_curr;          
      if (q.empty()) v0_curr = 0.0f;   
      startNextIfIdle(p);            
      
      break;                          
    }
  }
}
static void playback_tick(float dt) {
  if (!g_playing) {
    if ((int)g_buf.size() >= MIN_START_SAMPLES) {
      g_playing = true;
      g_play_t  = 0.0f;
    } else {
      return; 
    }
  }

  g_play_t += dt;

  while (!g_buf.empty() && g_buf.front().t <= g_play_t) {
    const Sample& s = g_buf.front();

    auto angs = angles(s.p.x, s.p.y, s.p.z, s.aoa);
    target_deg_base   = std::get<0>(angs);
    target_deg_link_1 = std::get<1>(angs);
    target_deg_link_2 = std::get<2>(angs);

    
  apply_wrist_grip(s.aoa, s.grip);

    g_buf.pop_front();
  }

  if (g_buf.empty() && !traj_active() && q.empty()) {
    g_playing = false;   
  }
}


inline void calibration_link_one(float &curr_deg, float &target){
  Serial.println("EXECUTING CALIBRATION ONE");
  if (digitalRead(LIMIT_SWITCH_CALIBRATION_ONE) == LOW) {
    escOneWriteNorm(0.0f);
    zero1  = curr_deg-7.0f;       
    target = 90.0f;         
    
    calibration_executed_one = true;
    Serial.println("[CAL_LINK_ONE] done: zero1 set, target=90");
  } else {
    escOneWriteNorm(-0.05f); 
  }
}

inline void calibration_link_two(float &curr_deg, float &target){
  Serial.println("EXECUTING CALIBRATION TWO");
  if (digitalRead(LIMIT_SWITCH_CALIBRATION_TWO) == LOW) {
    escTwoWriteNorm(0.0f);
    zero2  = curr_deg+50;      
    target = 90.0f;
    calibration_executed_two = true;
   
    Serial.println("[CAL_LINK_TWO] done: zero2 set, target=90");
  } else {
    escTwoWriteNorm(-0.05f);
  }
}


void setup() {
  Serial.begin(115200);

  
  Wire.end();
  Wire1.end();
  delay(5);


  Wire.begin(PIN_SDA_One, PIN_SCL_One, 100000);   
  Wire1.begin(PIN_SDA_Two, PIN_SCL_Two, 100000);  
  Wire.setTimeOut(50);
  Wire1.setTimeOut(50);

  enc_begin(enc_link1, Wire,  PIN_SDA_One, PIN_SCL_One, 100000, 0x36, 38.0f, false);
  enc_begin(enc_link2, Wire1, PIN_SDA_Two, PIN_SCL_Two, 100000, 0x36, -38.0f, false);
  pinMode(LIMIT_SWITCH_CALIBRATION_ONE, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_CALIBRATION_TWO, INPUT_PULLUP);

  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(PUL_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  if (!enc_seed(enc_link1)) Serial.println("enc_link1 seed failed");
  if (!enc_seed(enc_link2)) Serial.println("enc_link2 seed failed");
  wrist.attach(WRIST);   
  wrist.write(90); 
  claw.attach(CLAW);   
  claw.write(0); 
  escOne.attach(PIN_esc_One);
  escOne.writeMicroseconds(1500);
  escTwo.attach(PIN_esc_Two);
  escTwo.writeMicroseconds(1500);
  delay(3000);

  PID_reset_state(pid1);
  PID_reset_state(pid2);
}


void loop() {
  uint32_t now = micros();
  if (last_us_loop == 0) last_us_loop = now;
  float dt = (float)((uint32_t)(now - last_us_loop)) * 1e-6f; 
  if (dt <= 0.0f) dt = 1e-4f;          
  if (dt > 0.05f) dt = 0.05f;          
  last_us_loop = now;

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("reset")) {
      target_deg_link_1 = 90.0f;
      target_deg_link_2 = 90.0f;
      target_deg_base = 0.0f;
      PID_reset_state(pid1);
      PID_reset_state(pid2);

      q.clear();
      g_buf.clear();
      g_playing = false;
      g_play_t = 0.0f;
      return;
    }
    if (input.equalsIgnoreCase("base")) {
      target_deg_link_1 = 120.0f;
      target_deg_link_2 = -30.0f;
      target_deg_base = -37.0f;
      PID_reset_state(pid1);
      PID_reset_state(pid2);

      q.clear();
      g_buf.clear();
      g_playing = false;
      g_play_t = 0.0f;
      return;
    }
    if (input.equalsIgnoreCase("testall")) {
      q.clear(); g_buf.clear(); g_playing=false; g_play_t=0.0f;
      for (int i=0; i<4; ++i) runTestCase(i);
      merge_and_thin_queue(q, 0.04f, 4.0f, 5.0f);
      return;
    }


    if (input.startsWith("test")) {
      
      int idx = -1;
      if (input.length() == 4) {
        idx = 0; 
      } else {
  
        String tail = input.substring(4);
        tail.trim();
        if (tail.length() == 1 && isDigit(tail[0])) {
          idx = (tail[0] - '1'); 
        }
      }
      if (idx >= 0 && idx < 5) {
        runTestCase(idx);
      } else {
        Serial.println("Usage: test | test1 | test2 | test3 | test4");
      }
      return;
    }
   

    int s1 = input.indexOf(' ');
    int s2 = input.indexOf(' ', s1 + 1);
    int s3 = input.indexOf(' ', s2 + 1);

    if (s1 > 0 && s2 > s1) {
      x = input.substring(0, s1).toFloat();
      y = input.substring(s1 + 1, s2).toFloat();
      z = input.substring(s2 + 1, s3).toFloat();
      angle = input.substring(s3 + 1).toFloat();

      Serial.print("You entered: ");
      Serial.print(x); Serial.print(", ");
      Serial.print(y); Serial.print(", ");
      Serial.print(z); Serial.print(", ");
      Serial.print("Angle: "); Serial.println(angle);
      if (reachable(x, y, z)) {
        q.push_back(Waypoint{
          Vec3(x, y, z),    
          angle,            
          gripper_angle,   
          VMAX, AMAX,
          0.06f              
        });
        merge_and_thin_queue(q,0.04f, 4.0f, 5.0f);  /////////////////////////////////////min distance was increased from 0.012
        if (!g_playing && q.size() == 1) {
          apply_wrist_grip(q.front().aoa, q.front().grip);
        }

        Serial.println("[TRAJ] goal accepted; starting trajectory...");
      } else {
        Serial.println("Too far!");
      }
    } else if (input.length()) {
      Serial.println("Enter: x y z   (or 'home', 'reset')");
    }
  }
  
  bool ok1 = enc_read(enc_link1);
  bool ok2 = enc_read(enc_link2);

  static float deg_now_link1 = 0.0f;
  static float deg_now_link2 = 0.0f;

  if (ok1) deg_now_link1 = enc_output_deg(enc_link1) ;
  if (ok2) deg_now_link2 = enc_output_deg(enc_link2) ;

  float meas1 = deg_now_link1 - zero1;
  float meas2 = deg_now_link2 - zero2;

  auto curr_location = forward_kinematics(meas1, meas2, curr_base_angle, angle);
  float x_curr = std::get<0>(curr_location);
  float y_curr = std::get<1>(curr_location);
  float z_curr = std::get<2>(curr_location);
  Vec3 currP(x_curr, y_curr, z_curr);

  planner_tick(currP);     
  playback_tick(dt);      




  if (!calibration_executed_one) {
    calibration_link_one(deg_now_link1, target_deg_link_1);
      
    
    return;
  }
  if((meas1>80.0f && meas1 < 100.0f)&&(calibration_executed_one && !calibration_executed_two)){ 
   
      
    escOneWriteNorm(0.0f);
      
    calibration_link_two(deg_now_link2, target_deg_link_2);
    return;
    
    
   
    
  }
  if (!danger_on && calibration_executed_one && calibration_executed_two) {
    if (digitalRead(LIMIT_SWITCH_CALIBRATION_ONE) == HIGH &&
        digitalRead(LIMIT_SWITCH_CALIBRATION_TWO) == HIGH) {
      danger_on = true;
      Serial.println("[SAFETY] armed");
    }
  }


  float u_norm_1 = PID_step_position_state(target_deg_link_1, meas1,dt, pid1);
  float u_norm_2 = PID_step_position_state(target_deg_link_2, meas2,dt, pid2);

  if (calibration_executed_one){ 
    if (danger(target_deg_link_1, target_deg_link_2)){
      escOneWriteNorm(0.0f);
      escTwoWriteNorm(0.0f);
      Serial.println("DANGER");
    }else{
      if(fabs(curr_base_angle - target_deg_base) > 0.1f) base_update_nonblocking(target_deg_base, dt);
      
      escOneWriteNorm(u_norm_1);
      escTwoWriteNorm(u_norm_2);

    }
  }

  if (calibration_executed_one&&calibration_executed_two && danger_on){
    if (danger(target_deg_link_1, target_deg_link_2)){
      escOneWriteNorm(0.0f);
      escTwoWriteNorm(0.0f);
      Serial.println("DANGER");
    }
  }
   

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= PRINT_EVERY_MS) {
    lastPrint = millis();
    Serial.print(u_norm_1);         Serial.print("    ");
    Serial.print(u_norm_2);         Serial.print("    ");
    Serial.print(meas1);            Serial.print("    ");
    Serial.print(meas2);            Serial.print("    "); 
    Serial.print(curr_base_angle);  Serial.print("    "); 
    Serial.print("    t1 ");
    Serial.print(target_deg_link_1);Serial.print("  t2 ");
    Serial.print(target_deg_link_2);Serial.print("  t3 ");
    Serial.println(target_deg_base);
    
     
  }

  delay(2);
}

