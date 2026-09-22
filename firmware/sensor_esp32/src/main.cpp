#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  float laser = random(370, 430) / 10.0;
  float force = random(100, 200) / 100.0;

  Serial.print("laser=");
  Serial.print(laser);

  Serial.print(",force=");
  Serial.println(force);

  delay(100);
}