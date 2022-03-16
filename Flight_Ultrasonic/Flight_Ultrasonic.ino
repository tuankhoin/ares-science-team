#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>

#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
SFEVL53L1X S1_distance_sensor;
int S1_distance;

long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

void setup() {
  Wire.begin();
  S1_distance_sensor.begin();
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
}
void loop() {
  // Ultrasonic
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.34 / 2; // Speed of sound wave divided by 2 (go and back)
  
  // S1
  S1_distance_sensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!S1_distance_sensor.checkForDataReady())
  {
    delay(1);
  }
  S1_distance = S1_distance_sensor.getDistance(); //Get the result of the measurement from the sensor
  S1_distance_sensor.clearInterrupt();
  S1_distance_sensor.stopRanging();
  
  // Displays the distance on the Serial Monitor
  Serial.print("US-S1 Distance (mm): ");
  Serial.print(distance);
  Serial.print(" -- ");

  Serial.println(S1_distance);
}
