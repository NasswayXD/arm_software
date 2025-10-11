#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <tuple>
#include "pid.h"
#include "encoder.h"
#include "inverse_kinematics.h"
#include "safety.h"
Servo escOne, escTwo;


AS5600Enc enc_link1;  
AS5600Enc enc_link2;  

PIDState pid1{0,0,0};
PIDState pid2{0,0,0};
float curr_base_angle = 0.0f;
bool danger_on = false;
float zero1 = 0.0f, zero2 = 0.0f;
#define LIMIT_SWITCH_CALIBRATION_ONE 15
#define LIMIT_SWITCH_CALIBRATION_TWO 16
static float STEPS_PER_REV = 800.0f; 
float x=0, y=0, z=0;
bool calibration_executed_one = false;
bool calibration_executed_two = false;
static inline void escOneWriteNorm(float x) {
  x = constrain(x, -0.07, 0.07f); 
  int us = (int)(1500.0f + 500.0f * x);
  escOne.writeMicroseconds(us);

}
static inline float wrap360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a <    0.0f) a += 360.0f;
  return a;
}
void writeBase(float target_angle_deg) {
 
  float delta = target_angle_deg - curr_base_angle;

 
  if (fabs(delta) < 0.01f) return;

 
  if (delta < 0.0f) {
    digitalWrite(DIR_PIN, LOW);   // CW
  } else {
    digitalWrite(DIR_PIN, HIGH);  // CCW
  }

  
  long steps = lroundf(fabs(delta) * (STEPS_PER_REV / 360.0f));

 
  for (long i = 0; i < steps; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(500);
  }

 
  curr_base_angle = wrap360(target_angle_deg);
}
static inline void escTwoWriteNorm(float x) {
  x = constrain(x, -0.08, 0.08f); 
  int us = (int)(1500.0f + 500.0f * x);
  escTwo.writeMicroseconds(us);

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
    zero2  = curr_deg+45.0f;      
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
  pinMode(LIMIT_SWITCH_SAFETY_TWO, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_SAFETY_FOUR, INPUT_PULLUP);
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);
  if (!enc_seed(enc_link1)) Serial.println("enc_link1 seed failed");
  if (!enc_seed(enc_link2)) Serial.println("enc_link2 seed failed");

  escOne.attach(PIN_esc_One);
  escOne.writeMicroseconds(1500);
  escTwo.attach(PIN_esc_Two);
  escTwo.writeMicroseconds(1500);
  delay(3000);

  PID_reset_state(pid1);
  PID_reset_state(pid2);
}

void loop() {

 
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("reset")) {
      target_deg_link_1 = 90.0f;
      target_deg_link_2 = 90.0f;
      target_deg_base = 0.0f;
      PID_reset_state(pid1);
      PID_reset_state(pid2);
      return;
    }
    if (input.equalsIgnoreCase("test")) {
      auto ang = angles(0.1f, 0.1f, 0.1f);    
      float alpha2 = std::get<1>(ang);  
      float alpha3 = std::get<2>(ang);  
      target_deg_link_1 = alpha2;
      target_deg_link_2 = alpha3;
     
      return;
    }

    int s1 = input.indexOf(' ');
    int s2 = input.indexOf(' ', s1 + 1);

    if (s1 > 0 && s2 > s1) {
      x = input.substring(0, s1).toFloat();
      y = input.substring(s1 + 1, s2).toFloat();
      z = input.substring(s2 + 1).toFloat();

      Serial.print("You entered: ");
      Serial.print(x); Serial.print(", ");
      Serial.print(y); Serial.print(", ");
      Serial.println(z);

      if (reachable(x, y, z)) {
        auto ang = angles(x, y, z);        
        float alpha2 = std::get<1>(ang);  
        float alpha3 = std::get<2>(ang);  
        float alpha1 = std::get<0>(ang); 
        target_deg_link_1 = alpha2;
        target_deg_link_2 = alpha3;  
        target_deg_base = alpha1;
        Serial.print("Targets: ");
        Serial.print(target_deg_link_1); Serial.print(" / ");
        Serial.println(target_deg_link_2);
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


  float u_norm_1 = PID_step_position_state(target_deg_link_1, meas1, pid1);
  float u_norm_2 = PID_step_position_state(target_deg_link_2, meas2, pid2);
  if (calibration_executed_one){ //change to calibration_executed_one
  if (danger(target_deg_link_1, target_deg_link_2)){
     escOneWriteNorm(0.0f);
      escTwoWriteNorm(0.0f);
      Serial.println("DANGER");
    }else{
      if(fabs(curr_base_angle - target_deg_base) > 0.1f) writeBase(target_deg_base);
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
    Serial.print(meas1);    Serial.print("    ");
    Serial.print(meas2);   Serial.print("    "); Serial.print(curr_base_angle); Serial.print("    "); Serial.print("    t1 ");
    Serial.print(target_deg_link_1);Serial.print("  t2 ");
    Serial.print(target_deg_link_2);Serial.print("  t3 ");
    Serial.println(target_deg_base);
    
     //Serial.println(zero1);
  }

  delay(2);
}
