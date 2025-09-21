#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "pid.h"
#include "encoder.h"
#include "inverse_kinematics.h"
Servo esc;  
float x, y , z;
static inline void escWriteNorm(float x) {
  x = constrain(x, -0.1f, 0.1f); // at what percentage of full speed the motor is working, now 10% (most optimal at low speed)
  int us = (int)(1500.0f + 500.0f * x); // mapping to the motor range 
  esc.writeMicroseconds(us);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  esc.attach(PIN_ESC);
  esc.writeMicroseconds(1500);   
  delay(3000);
  PID_reset();
}

void loop() {
  if (Serial.available()) {
    
    String input = Serial.readStringUntil('\n');
    input.trim();

   
    if (input.equalsIgnoreCase("reset")) {
      target_deg = 0.0;
      return;
    }

   
    int firstSpace  = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);

    if (firstSpace > 0 && secondSpace > firstSpace) {
      x = input.substring(0, firstSpace).toFloat();
      y = input.substring(firstSpace + 1, secondSpace).toFloat();
      z = input.substring(secondSpace + 1).toFloat();

      Serial.print("You entered: ");
      Serial.print(x); Serial.print(", ");
      Serial.print(y); Serial.print(", ");
      Serial.println(z);

      Serial.println("Enter 3 numbers separated by spaces:");
    }

    if (reachable(x, y, z)) {
      auto [alpha1, alpha2, alpha3] = angles(x, y, z);
      Serial.print("Target is reachable, angles: "); 
      Serial.print(alpha1); Serial.print(", ");
      Serial.print(alpha2); Serial.print(", ");
      Serial.println(alpha3);
      target_deg = alpha2;
    } else {
      Serial.println("Too far!"); 
    }
  }

  uint16_t raw;
  if (readRawAngle12(raw)) {
    updateTurnCounter(raw);
    float deg_now = outputDegrees(raw);

    float u_norm = PID_step_position(target_deg, deg_now);
    escWriteNorm(u_norm);

    // slow telemetry
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= PRINT_EVERY_MS) {
      lastPrint = millis();
      Serial.print(u_norm);
      Serial.print("    ");
      Serial.print(deg_now);
      Serial.print("    target    ");
      Serial.print(target_deg);
      Serial.println();
    }
  }
  delay(2);
}
