#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    // Placeholder for the pressure reported by
    // the real Helico controller.
    float pressure = random(180, 260) / 10.0;

    Serial.print("pressure=");
    Serial.println(pressure);

    delay(100);
}