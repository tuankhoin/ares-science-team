#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

class distanceSensor
{
    private:
        VL53L1X sensor = VL53L1X();
    public:
        void init();
        uint16_t read();
};
