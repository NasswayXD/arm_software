#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "pid.h"
#include "encoder.h"
#include "inverse_kinematics.h"

Servo escL, escR, escLL2;

// ------- two I2C controllers + two encoders -------

 
AS5600Enc enc_link1;           // left / link 1 encoder
AS5600Enc enc_link2;           // right / link 2 encoder
// --------------------------------------------------

float x, y, z;

static inline void escLWriteNorm(float x) {
  x = constrain(x, -0.1f, 0.1f);
  int us = (int)(1500.0f + 500.0f * x);
  escL.writeMicroseconds(us);
  escR.writeMicroseconds(us);
}

void setup() {
  Serial.begin(115200);

  // Nuke any previous I2C modes (including accidental slave)
  Wire.end();  
  Wire1.end();  
  delay(5);

  // Bring up both buses explicitly (master @ 100kHz)
  Wire.begin(PIN_SDAL, PIN_SCLL, 100000);  // bus 0
  Wire1.begin(PIN_SDAR, PIN_SCLR, 100000); // bus 1


  // Now initialize encoder structs WITHOUT re-beginning
  enc_begin(enc_link1, Wire,  PIN_SDAL, PIN_SCLL, 100000, 0x36, 38.0f, false);
  enc_begin(enc_link2, Wire1, PIN_SDAR, PIN_SCLR, 100000, 0x36, 38.0f, false);


  auto checkBus = [](const char* name, TwoWire& w) {
  // Quick transaction against a nonsense address to see if the driver rejects it because it's in slave mode.
  w.beginTransmission(0x7F);
  uint8_t err = w.endTransmission(true);    // We only care if the call itself is allowed to run
  if (err == 4 /*OTHER_ERROR*/ ) {
    Serial.printf("%s: beginTransmission blocked → likely in SLAVE mode\n", name);
  } else {
    Serial.printf("%s: beginTransmission OK (err=%u) → master mode\n", name, err);
  }
  // Bonus: on ESP32 getClock()==0 is a red flag too.
  if (w.getClock() == 0) {
    Serial.printf("%s: getClock()==0 → suspicious (not in master?)\n", name);
  }
};


checkBus("I2C0", Wire);
checkBus("I2C1", Wire1);


  // Seed unwrap once (after bus is up)
  if (!enc_seed(enc_link1)) Serial.println("enc_link1 seed failed");
  if (!enc_seed(enc_link2)) Serial.println("enc_link2 seed failed");
  // ------------------------------------------------

  // ⚠️ DO NOT call Wire.begin(...) again; you already initialized I2C0/I2C1 above.
  // Remove these two lines to avoid "Bus is in Slave Mode" / pin errors:
  // Wire.begin(PIN_SDAL, PIN_SCLL);
  // Wire.setClock(100000);

  escL.attach(PIN_escL);
  escL.writeMicroseconds(1500);
  escR.attach(PIN_escR);
  escR.writeMicroseconds(1500);
  delay(3000);
  PID_reset();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("reset")) {
      target_deg_link_1 = 0.0;
      target_deg_link_2 = 0.0;
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
      target_deg_link_1 = alpha2;
      target_deg_link_2 = alpha3;
    } else {
      Serial.println("Too far!");
    }
  }

  // ------- read both encoders (robustly) -------
  bool ok1 = enc_read(enc_link1);
  bool ok2 = enc_read(enc_link2);

  // Use last good values even if one read drops; skip PID if none are valid
  static float deg_now_link1 = 0.0f;
  static float deg_now_link2 = 0.0f;

  if (ok1) deg_now_link1 = enc_output_deg(enc_link1);
  if (ok2) deg_now_link2 = enc_output_deg(enc_link2);
  // ---------------------------------------------

  // Your existing single-link PID drive (uses link1 angle)
  float u_norm_1 = PID_step_position(target_deg_link_1, deg_now_link1);
  escLWriteNorm(u_norm_1);

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= PRINT_EVERY_MS) {
    lastPrint = millis();
    Serial.print(u_norm_1);
    Serial.print("    ");
    Serial.print(deg_now_link1);
    Serial.print("    ");
    Serial.print(deg_now_link2);
    Serial.print("    target    ");
    Serial.print(target_deg_link_1);
    Serial.println();
  }

  delay(2);
}
