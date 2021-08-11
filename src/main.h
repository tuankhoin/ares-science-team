#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <distanceSensor.h>

namespace Main
{
  distanceSensor tofSensor = distanceSensor();
}