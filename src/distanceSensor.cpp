#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <distanceSensor.h>


void distanceSensor::init()
{
    // This is for communicate with ÌC/TWI devices
    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    sensor.setTimeout(500);
    while (!sensor.init())
    {
        Serial.println("[VL53L1X] Failed to initialise distance sensor...");
    }
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(15000);
    sensor.startContinuous(15);
    Serial.println("[VL53L1X] Distance sensor is running...");
}

uint16_t distanceSensor::read()
{
    return sensor.read();
}
