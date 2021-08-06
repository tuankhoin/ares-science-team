#include <Servo.h>

#define LED_INDICATOR 13
#define INDICATOR 12
#define AHEAD 2
#define END -3

#define TRIGGER 0
#define ECHO 1
#define SOUND_TO_CM 0.01723
#define CLOSE_DISTANCE 50 // 10cm in trials, larger distance is put for simplifying demonstration

#define BACKWARD 0
#define FORWARD 1
#define RIGHT -1
#define LEFT 1

#define FLAP 30
#define FULL_LEFT 180
#define MIDDLE 90
#define TURN_ANGLE 45
#define FULL_ROTATION 3000

// ON/OFF
int state = 0;                // On/Off state 
int curr = 0, prev = 0;       // Button states

// Ball dropping order
int currentDrop = 4;          // Current tube's corresponding number of vessels

// Distance measuring system
int cmLeft = 0, cmRight = 0;  // Initial sensor values
int leftSensor[2] = {8,9};    // Left sensor pins
int rightSensor[2] = {7,6};   // Right sensor pins

// Moving system
int dcMotor[2] = {4,5};       // Motor pins

//Servo controllers
Servo turnWheel;              // Attached at front wheel axis
Servo pipeControl;            // Attached as distributor's rotator
Servo flapControl;            // Controls which tube will be opened

void setup() {
  
  // Initiate system
  pinMode(INDICATOR, INPUT);
  pinMode(LED_INDICATOR, OUTPUT);
  pinMode(AHEAD, OUTPUT);
  Serial.begin(9600);

  // Attach and set default position
  turnWheel.attach(11);
  turnWheel.write(MIDDLE);
  pipeControl.attach(10);
  pipeControl.write(MIDDLE);
  flapControl.attach(3);
  flapControl.write(FULL_LEFT);
  
  // Set pins for DC motors
  pinMode(dcMotor[0], OUTPUT);
  pinMode(dcMotor[1], OUTPUT);

}

void loop() {
  curr = digitalRead(INDICATOR);
  
  // Change of button state? If yes, change the on/off state
  if (curr && !prev) {
    state = !state;
    // Notifying
    if (state) {Serial.println("Starting operation. A flagpole is dropped at the starting point!");}
  }
  prev = curr;	// Remember the previous button state
  
  if (state) {
  	digitalWrite(LED_INDICATOR, HIGH); // LED indicating ON state
    operation(); // Operate the system
  } else {
  	digitalWrite(LED_INDICATOR, LOW); // LED off
    // Restarting all axis to default position
    turnWheel.write(MIDDLE);
    pipeControl.write(MIDDLE);
    flapControl.write(FULL_LEFT);
    // Full stop
    stop();
  }
  delay(100);
}

/* Main operations go here */
void operation() {
  cmLeft = readUltrasonic(leftSensor);
  cmRight = readUltrasonic(rightSensor);
  Serial.print("Nearest obstacle on 2 sides: ");
  Serial.print(cmLeft);
  Serial.print(" Left-Right ");
  Serial.println(cmRight);
  
  // Tube detected: Drop the vessels
  if (upClose(cmLeft) || upClose(cmRight)) {
    Serial.println("Tube detected in front!");
    digitalWrite(AHEAD, HIGH);
    stop();

    // If the flagpole at the end zone is detected, device can take a rest now
    if (currentDrop == END) { 
      Serial.println("Task completed!");
      return; 
    }

    // Check on what side was the tube detected
    if (upClose(cmLeft)) {
      // Skip the dropping part if there is no more vessels to drop
      if (currentDrop > 0) {
        directToTube(LEFT);     // Calibration
        dropball();             // Drop the vessels
      }
      turn(RIGHT);              // Make an exit turn
    } else {
      // Same procedure as above, different direction
      if (currentDrop > 0) {        
        directToTube(RIGHT);
        dropball();
      }
      turn(LEFT);
    }
    Serial.println("Let's go bois");
  }
  // Nothing ahead: Keep moving
  else {
    move(FORWARD);
  }
}

/*   Rotates the pipe in a direction until the tube is set directly in front
 *   direction: the direction to rotate to
 */
void directToTube(int direction) {
  Serial.println("Directing the pipe to the tube...");
  int pipeDirection = pipeControl.read();
  while (!(upClose(cmLeft) && upClose(cmRight))){
    pipeDirection += direction;
    pipeControl.write(pipeDirection);
    cmLeft = readUltrasonic(leftSensor);
    cmRight = readUltrasonic(rightSensor);
  }
  Serial.println("Calibration completed!");
}

/*   Dropping the balls to the tube */
void dropball() {
  Serial.print("Dropping the ball. Current vessel is "); 
  Serial.println(currentDrop);
  flapControl.write(currentDrop * FLAP);
  delay(2000);
}

/*   Make a turn to exit the tube
 *   direction: turning direction
 */
void turn(int direction) {
  Serial.println("Operation finished! Now turning...");
  currentDrop--;
  pipeControl.write(MIDDLE);

  turnWheel.write(MIDDLE + TURN_ANGLE*direction);
  move(FORWARD);

  if (currentDrop) {
    // Turn until there is no obstacles ahead
    while(upClose(cmLeft) || upClose(cmRight)) {
      cmLeft = readUltrasonic(leftSensor);
      cmRight = readUltrasonic(rightSensor);
    }
  } else {
    // In case of just finished the task, do a full rotation
    Serial.println("No more vessels. Rotating 180deg to head back!");
    delay(FULL_ROTATION);
  }
  

  stop();
  digitalWrite(AHEAD, LOW);
  turnWheel.write(MIDDLE);
  Serial.println("Journey continues...");
}

/*   Returns the measured sensor value of an ultrasonic sensor
 *
 *   u: the measured ultrasonic sensor
 *   returns: the measured distance of u, in centimeters
 */
int readUltrasonic(int* u) {
  pinMode(u[TRIGGER], OUTPUT);
  pinMode(u[ECHO], INPUT);
  digitalWrite(u[TRIGGER], LOW); 
  delayMicroseconds(2);
  digitalWrite(u[TRIGGER],HIGH); 
  delayMicroseconds(10);
  digitalWrite(u[TRIGGER], LOW);
  return pulseIn(u[ECHO], HIGH) * SOUND_TO_CM;
}

/*   Returns true if a close object is detected from the sensor
 *   val: The distance measured from the sensor
 */
int upClose(int val) {
  return val < CLOSE_DISTANCE;
}

/*   Move the DC motors
 *   forward: true if device is set to move forward, otherwise backwards
 */
void move(int forward) {
  if (!forward) {
    digitalWrite(dcMotor[0], LOW);
    digitalWrite(dcMotor[1], HIGH);
  } else {
    digitalWrite(dcMotor[1], LOW);
    digitalWrite(dcMotor[0], HIGH);
  }
}

/*   Stops the engine   */
void stop() {
  digitalWrite(dcMotor[0], LOW);
  digitalWrite(dcMotor[1], LOW);
}