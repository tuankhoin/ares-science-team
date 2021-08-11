#include <Arduino.h>
#include <Wire.h>
// #include <distanceSensor.h>
#include <main.h>

void setup()
{
  using namespace Main;
  Serial.begin(115200);

  // Init the ToF sensor
  tofSensor.init();
  
  Serial.println("new program");
}

void loop()
{
  using namespace Main;

  // Print measurement for ToF sensor
  Serial.println("Distance measurement: " + String(tofSensor.read()) + " mm");
}