#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>
//#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X S1_distance_sensor;
SFEVL53L1X S2_distance_sensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

int S1_distance;
int S2_distance;

void setup(void)
{
  Wire.begin();

  Serial.begin(9600);

  S1_distance_sensor.begin();
  S2_distance_sensor.begin();
  
  Serial.println("Please remove S2's SDA & SCL pins. Press any key and hit enter to continue.");
  while (Serial.available() == 0);
  Serial.println("Changing S1's I2C address from 82 to 81.");
  S1_distance_sensor.setI2CAddress(81);

  Serial.print("S1's address: ");
  Serial.println(S1_distance_sensor.getI2CAddress());
  Serial.print("S2's address: ");
  Serial.println(S2_distance_sensor.getI2CAddress());  
  
  Serial.println("Please reinsert S2's SDA & SCL pins. Press any key and hit enter to continue.");
  while (Serial.available() <= 1);
  Serial.println("Done"); 
  
}

void loop(void)
{

  // S1
  S1_distance_sensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!S1_distance_sensor.checkForDataReady())
  {
    delay(1);
  }
  S1_distance = S1_distance_sensor.getDistance(); //Get the result of the measurement from the sensor
  S1_distance_sensor.clearInterrupt();
  S1_distance_sensor.stopRanging();

  Serial.print("S1 Distance(mm): ");
  Serial.println(S1_distance);

  

  // S2
  S2_distance_sensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!S2_distance_sensor.checkForDataReady())
  {
    delay(1);
  }
  S2_distance = S2_distance_sensor.getDistance(); //Get the result of the measurement from the sensor
  S2_distance_sensor.clearInterrupt();
  S2_distance_sensor.stopRanging();

  Serial.print("S2 Distance(mm): ");
  Serial.println(S2_distance);

}
