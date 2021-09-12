#include <Arduino.h>
#include <Wire.h>
// #include <distanceSensor.h>
#include <main.h>
#include <Servo.h>  

Servo sv;
int curr,d;

void setup()
{
  using namespace Main;
  Serial.begin(115200);

  // Attach servo pin
  sv.attach(9);
  // Init the ToF sensors
  tofSensor.init();
  
  Serial.println("new program");
}

void loop()
{
  using namespace Main;

  curr = sv.read();
  d = tofSensor.read();

  if (d<1000 && d>100 && curr<270) {
    sv.write(curr+1);
  } else if (d<=100 && curr>0) {
    sv.write(curr-1);
  }

  // Print measurement for ToF sensor
  Serial.println("Distance measurement: " + String(d) + " mm");
}