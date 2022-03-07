#include <Servo.h>

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X



// Stepper motor
// S1 = Stepper for container 
// S2 = Stepper for drill
#define S1_EN_PIN 27
#define S1_STEP_PIN 35
#define S1_DIR_PIN 37
#define S2_EN_PIN 43
#define S2_STEP_PIN 51
#define S2_DIR_PIN 53
#define STEPS_PER_REVOLUTION 200

// Limit switch
#define S1_UPPER_LIMIT_SWITCH_PIN 23
#define S1_LOWER_LIMIT_SWITCH_PIN 25
#define S2_UPPER_LIMIT_SWITCH_PIN 39
#define S2_LOWER_LIMIT_SWITCH_PIN 41

// DC Motor
#define E1 3
#define M1 2

// Servos
#define CONTAINER_SERVO_PIN 9
#define PROBE_SERVO_PIN 10

// Command mapping
#define CONT_UP 'q'
#define CONT_DOWN 'a'
#define CONT_STOP 'w'

#define DRILL_UP 'e'
#define DRILL_DOWN 'd'
#define DRILL_STOP 'r'

#define DRILL_CW 't'
#define DRILL_CCW 'g'
#define DRILL_OFF 'y'

#define CONT_CLOSE 'u'
#define CONT_OPEN 'j'

#define PROBE_UP 'i'
#define PROBE_DOWN 'k'

#define START 'z'
#define STOP_MOTORS 'x'
#define KILL 'c'

#define READY 'p'



// Rangefinder
#define MAX_DIST_ATTEMPTS 100
#define MIN_DIST 500
#define MIN_SEPARATION 100

// Check distance every
#define ITERS_DIST 10000

int iters_count = 1;

SFEVL53L1X S1_distance_sensor;
SFEVL53L1X S2_distance_sensor;

int S1_distance = MIN_DIST + 1;
int S2_distance = MIN_DIST + MIN_SEPARATION + 1;

bool distance_sensors_available = false;

Servo container_servo;
Servo probe_servo;


// Serial monitor input character
char rx_byte;

// Some toggles
bool stepper_1 = false;
bool stepper_2 = false;
bool drill_on = false;
bool accept_input = false;


// set to true when using the serial monitor. 
// set to false when communicating with python.
bool debug = true;

void log_info(String info) {
  if (debug)
    Serial.print(info);
}


bool stepper_runnable(int lower_limit_pin, int upper_limit_pin, int dir_pin, int distance){
  // Return false if the lower limit switch has been hit and the container arm/drill is going down or vice versa with the upper limit switch
  if(digitalRead(lower_limit_pin) == LOW && digitalRead(dir_pin) == HIGH ||
     digitalRead(upper_limit_pin) == LOW && digitalRead(dir_pin) == LOW){
    log_info("Attempting to go past a limit switch!!\n\n");
    return false;
  }

  // Return false if the the container arm/drill is too close to the ground
  if(distance_sensors_available){
    if(distance < MIN_DIST){
      log_info("Attempting to go too close to the ground!!\n\n");
      return false;
    } else if(fabs(S1_distance - S2_distance) < MIN_SEPARATION){
      log_info("Steppers too close to each other!!\n\n");
      return false;
    }
  }
  // Otherwise return true
  return true;
}

void run_stepper(int step_pin){
  // Delay
  delayMicroseconds(400);

  // Rotate 1 step:
  digitalWrite(step_pin, HIGH); // Turn stepper(s) on
  delayMicroseconds(400);     // Delay
  digitalWrite(step_pin, LOW);// Turn stepper off
  delayMicroseconds(400);     // Delay
}

void run_steppers(int step_pin_1, int step_pin_2){
  // Delay
  delayMicroseconds(400);

  // Rotate 1 step:
  // Turn stepper(s) on
  digitalWrite(step_pin_1, HIGH);
  digitalWrite(step_pin_2, HIGH);

  // Delay
  delayMicroseconds(400);

  // Turn stepper(s) off
  digitalWrite(step_pin_1, LOW);
  digitalWrite(step_pin_2, LOW);

  // Delay
  delayMicroseconds(400);
}

void cont_up() {
  // Enable stepper 1
  stepper_1 = true;
  
  // This can't be run every loop otherwise stepper makes a terrible noise
  // Set the spinning direction clockwise (move up):
  digitalWrite(S1_DIR_PIN, LOW);  // stepper driver #1
  digitalWrite(S1_EN_PIN, LOW);
}

void cont_down(){
  // Enable stepper 1
  stepper_1 = true;

  // Set the spinning direction counterclockwise (move down):
  digitalWrite(S1_DIR_PIN, HIGH);   // stepper driver #1
  digitalWrite(S1_EN_PIN, LOW);
}

void drill_up(){
  // Enable stepper 2
  stepper_2 = true;

  // Set the spinning direction clockwise (move up):
  digitalWrite(S2_DIR_PIN, LOW);   // stepper driver #2
  digitalWrite(S2_EN_PIN, LOW);
}


void drill_down(){
  // Enable stepper 2
  stepper_2 = true;

  // Set the spinning direction counterclockwise (move down):
  digitalWrite(S2_DIR_PIN, HIGH);   // stepper driver #2
  digitalWrite(S2_EN_PIN, LOW);
}
    

void drill_cw() {
  // Spin clockwise at full speed
  analogWrite(E1, 255);
  digitalWrite(M1, LOW);
}

void drill_ccw(){
  // Spin clockwise at full speed
  analogWrite(E1, 255);
  digitalWrite(M1, HIGH);
}

void drill_off(){
  // Turn drill off
  analogWrite(E1, LOW);
}


void container_open(){
  container_servo.write(120);   // Rotate to ~180°
  probe_servo.write(120);   // Rotate to 0°
}

void container_close(){
  container_servo.write(0);   // Rotate to 0°
  probe_servo.write(0);   // Rotate to 0°
}

void start() {
  digitalWrite(S1_EN_PIN, LOW);  // Enable stepper driver #1
  digitalWrite(S2_EN_PIN, LOW);  // Enable stepper driver #2
  accept_input = true;  //Allow commands to be input
}

void stop_motors() {
  digitalWrite(S1_EN_PIN, HIGH);  // Disable stepper driver #1
  digitalWrite(S2_EN_PIN, HIGH);  // Disable stepper driver #2

  stepper_1 = false;  // Stop running stepper #1
  stepper_2 = false;  // Stop running stepper #1
  
  drill_off();  // Turn drill off  
  accept_input = false; //Allow commands to be input
}


void setup_distance_sensors(){
  
  if(S1_distance_sensor.begin() == false){
    log_info("Sensor 1 failed to begin. Please check wiring.\n");
    return;
  }
  
 

  log_info("Changing S1's I2C address from 82 to 81.\n");

  S1_distance_sensor.setI2CAddress(81);

  log_info("S1's address: " + String(S1_distance_sensor.getI2CAddress()) + "\n");
  
  
  
  
  log_info("Please insert S2's SDA & SCL pins. Press 'l' and hit enter to continue.\n");

  int attempts;
  for (attempts = 0; Serial.available() <= 0 && attempts < MAX_DIST_ATTEMPTS; attempts++)
    delay(100);
  
  if(attempts == MAX_DIST_ATTEMPTS){
    log_info("Distance sensor setup failed: no command received\n\n");
    return;
  }


   if(S2_distance_sensor.begin() == false){
      log_info("Sensor 2 failed to begin. Please check wiring.\n");
      return;
    }

  
  log_info("S2's address: " + String(S2_distance_sensor.getI2CAddress()) + "\n");

  distance_sensors_available = true;
  
  log_info("Done\n\n");  
}


int get_distance(SFEVL53L1X distance_sensor) {
  int distance;

  // Declare some variables for getting distance safely and reliably
  int num_checks = 1;
  bool data_received = false;
  
  distance_sensor.startRanging(); //Write configuration bytes to initiate measurement
  
  for(int attempts = 0; attempts < MAX_DIST_ATTEMPTS; attempts++){
    if(distance_sensor.checkForDataReady()){
      distance = distance_sensor.getDistance(); //Get the result of the measurement from the sensor
      data_received = true;
      break;
    }
  }
  
  distance_sensor.clearInterrupt();
  distance_sensor.stopRanging();

  if(!data_received)
    distance = -1;
  
  return distance;
}


void pin_info(){
  
  log_info("S1_DIR_PIN : " + String(digitalRead(S1_DIR_PIN)) + ", ");

  log_info("S2_DIR_PIN : " + String(digitalRead(S2_DIR_PIN)) + ", ");

  log_info("container_servo : " + String(container_servo.read()) + ", ");

  log_info("probe_servo : " + String(probe_servo.read()) + "\n\n");
}



void setup() {
  Wire.begin();
  
  // For command inputs
  Serial.begin(9600);

  while(Serial.available()>0){
    char curr_char = Serial.read();
  }

  // Stepper
  pinMode(S1_STEP_PIN, OUTPUT);
  pinMode(S1_DIR_PIN, OUTPUT);
  pinMode(S2_STEP_PIN, OUTPUT);
  pinMode(S2_DIR_PIN, OUTPUT);

  // Limit switch
  pinMode(S1_UPPER_LIMIT_SWITCH_PIN, INPUT);
  pinMode(S1_LOWER_LIMIT_SWITCH_PIN, INPUT);
  pinMode(S2_UPPER_LIMIT_SWITCH_PIN, INPUT);
  pinMode(S2_LOWER_LIMIT_SWITCH_PIN, INPUT);

  // DC motor
  pinMode(M1, OUTPUT);

  // Servo
  container_servo.attach(CONTAINER_SERVO_PIN);
  probe_servo.attach(PROBE_SERVO_PIN);

  // Close container otherwise it will open automatically
  container_close();

  //  Turn everything off to begin with
  stop_motors();
  
  // Do distance sensor calibration
  setup_distance_sensors();

  log_info(String("Commands:\n")+START+String(" - accept commands\n")
            + STOP_MOTORS + String(" - turn off motors and stop accepting commands\n")
            + CONT_UP + String(" - container arm up\n")
            + CONT_DOWN + String(" - container arm down\n")
            + CONT_STOP + String(" - container arm off\n")
            + DRILL_UP + String(" - drill arm up\n")
            + DRILL_DOWN + String(" - drill arm down\n")
            + DRILL_STOP + String(" - stop drill translation")
            + DRILL_CW + String(" - spin drill clockwise\n")
            + DRILL_CCW + String(" - spin drill counter-clockwise\n")
            + DRILL_OFF + String(" - stop spinning drill\n")
            + DRILL_STOP + String(" - drill stop translating\n")
            + CONT_OPEN + String(" - open container\n")
            + CONT_CLOSE + String(" - close container\n\n"));
}

void loop() {    
  if (Serial.available() > 0) {
    // Read and echo command from Serial
    rx_byte = Serial.read();

    log_info("Command: " + String(rx_byte));
    
    if(accept_input){
      switch (rx_byte){
        // Raise container arm
        case CONT_UP:
          cont_up();
          break;
  
        // :Lower container arm
        case CONT_DOWN:
          cont_down();    
          break;
  
        // Stop container arm translation
        case CONT_STOP:
          stepper_1 = false;
          break;
  
        // Raise drill arm
        case DRILL_UP:
          drill_up();
          break;
        
        // Lower drill arm
        case DRILL_DOWN:
          drill_down();
          break;
  
        // Stop drill arm translation
        case DRILL_STOP:
          stepper_2 = false;
          break;
  
        // Spin clockwise
        case DRILL_CW:
          drill_cw();
          break;

        // Spin counter-clockwise
        case DRILL_CCW:
          drill_ccw();
          break;

        // Stop drilling
        case DRILL_OFF:
          drill_off();
          break;

        // Close container
        case CONT_CLOSE:
          container_close();
          break;
        
        // Open container
        case CONT_OPEN:
          container_open();
          break;
          
        // Moisture probe up
        case PROBE_UP:
          break;
          
        // Moisture probe down
        case PROBE_DOWN:
          break;
          
        // Stop motors
        case STOP_MOTORS:
          stop_motors();
          break;

        // Stop everything
        case KILL:
          stop_motors();
          Serial.write(KILL);
          exit(EXIT_FAILURE);
          break;
      }
      log_info("\n\n");
    }
    else if(rx_byte == READY){
      Serial.write(READY);
    }
    else if(rx_byte == START){
      start();
      log_info("\n\n");
    }
    else{
      log_info("  -  NOT ACCEPTED (Press 'p' to send commands))\n\n");
    }
    
    pin_info();
  }

  // Get distance information and reset the counter if it's time to
  // NB: Counter ensures distance isn't calculated every loop so motors don't stall (measuring distance takes time)
  if(distance_sensors_available){
    if(iters_count == ITERS_DIST){
      S2_distance = get_distance(S2_distance_sensor);
      S1_distance = get_distance(S1_distance_sensor);
      
      log_info("S1 distance(mm): " + String(S1_distance));
      log_info(", S2 distance(mm): " + String(S2_distance) + "\n\n");
      iters_count = 1;
    }
    else
      iters_count++; 
  }

  // Update whether stepper can be run based on limit switches & distance 
  if(stepper_1)
    stepper_1 = stepper_runnable(S1_LOWER_LIMIT_SWITCH_PIN, S1_UPPER_LIMIT_SWITCH_PIN, S1_DIR_PIN, S1_distance);
  if(stepper_2)
    stepper_2 = stepper_runnable(S2_LOWER_LIMIT_SWITCH_PIN, S2_UPPER_LIMIT_SWITCH_PIN, S2_DIR_PIN, S2_distance);    


  // Run whatever steppers can be run
  // NB: needed to separate cases of running both simultaneously. 
  // The extra delay of running another motor slows both down and makes a loud noise.
  if(stepper_1 && stepper_2)
    run_steppers(S1_STEP_PIN, S2_STEP_PIN);
  else if(stepper_1)
    run_stepper(S1_STEP_PIN);
  else if(stepper_2)
    run_stepper(S2_STEP_PIN);  
}
