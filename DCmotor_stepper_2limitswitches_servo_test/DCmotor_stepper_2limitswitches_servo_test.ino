#include <Servo.h>
Servo myservo;

// Servo
#define SERVO_PIN 9

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
int pwm_value = 0;

// Serial monitor input character
char rx_byte;

// Change command flag
boolean change_command;

// Servo container open status flag- initially closed
boolean closed = true;


void setup() {
  // For command inputs
  Serial.begin(9600);

  // Servo
  myservo.attach(SERVO_PIN);  

  // Stepper
  digitalWrite(S1_EN_PIN, HIGH);  // Disable stepper driver so it doesn't turn motor
  digitalWrite(S2_EN_PIN, HIGH);  // Disable stepper driver so it doesn't turn motor
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
}

void loop() {
/*
  // Set the spinning direction counterclockwise (move down):
  digitalWrite(S1_DIR_PIN, HIGH);

  // Spin the stepper motor 10 revolutions fast:
  for (int i = 0; i < 10 * STEPS_PER_REVOLUTION; i++) {
    // These four lines result in 1 step:
    digitalWrite(S1_STEP_PIN, HIGH);
    delayMicroseconds(400);
    digitalWrite(S1_STEP_PIN, LOW);
    delayMicroseconds(400);
  }

  delay(1000);

  // Set the spinning direction clockwise (move up):
  digitalWrite(S1_DIR_PIN, LOW);

  //Spin the stepper motor 10 revolutions fast:
  for (int i = 0; i < 10 * STEPS_PER_REVOLUTION; i++) {
    // These four lines result in 1 step:
    digitalWrite(S1_STEP_PIN, HIGH);
    delayMicroseconds(400);
    digitalWrite(S1_STEP_PIN, LOW);
    delayMicroseconds(400);
  }

  delay(1000);
*/

  
  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();       // get the character
    change_command = true;
  }

  
  // This can't be run every loop otherwise stepper makes a terrible noise
  if (change_command && rx_byte == 'w') {
    Serial.println("Going up");
    // Set the spinning direction clockwise (move up):
    digitalWrite(S1_DIR_PIN, LOW);  // stepper driver #1
    digitalWrite(S2_DIR_PIN, LOW);  // stepper driver #2
    
    change_command = false;
  } else if (change_command && rx_byte == 's') {
    // Set the spinning direction counterclockwise (move down):
    Serial.println("Going down");
    digitalWrite(S1_DIR_PIN, HIGH);   // stepper driver #1
    digitalWrite(S2_DIR_PIN, HIGH);   // stepper driver #2

    change_command = false;
  }


  // Stop everything if 'q' is pressed
  if (rx_byte == 'q') {
    analogWrite(E1, LOW); // No DC motor voltage
    digitalWrite(S1_EN_PIN, HIGH);  // Disable stepper driver #1
    digitalWrite(S2_EN_PIN, HIGH);  // Disable stepper driver #2
  }
  // Otherwise do the action
  else {
    // Servo rotate
    if (rx_byte == 'r') {
      if (closed) {
        myservo.write(120);   // Rotate to ~180°
        closed = false;
      } else {
        myservo.write(0);   // Rotate to 0°
        closed = true;
      }
    }
    
    // Pulsating drill if 'd' is pressed
    if (rx_byte == 'd') {
      Serial.println("Drilling");
  
      if (pwm_value > 255) {
        pwm_value = 0;
      }
      digitalWrite(M1, LOW); // Go forward
      analogWrite(E1, pwm_value); // PWM Speed Control
      pwm_value++;
      delay(30);  // So it can have time to get up-to-speed
  
      // Turn it off
      analogWrite(E1, LOW); // No DC motor voltage
    } 
    
    // Stepper #1: Move up or down if 'w' or s' is pressed respectively unless upper or lower limit is hit respectively
    if ((digitalRead(S1_LOWER_LIMIT_SWITCH_PIN) == HIGH && digitalRead(S1_UPPER_LIMIT_SWITCH_PIN) == HIGH && (rx_byte == 'w' || rx_byte == 's')) ||
              digitalRead(S1_LOWER_LIMIT_SWITCH_PIN) == LOW && rx_byte == 'w' ||
              digitalRead(S1_UPPER_LIMIT_SWITCH_PIN) == LOW && rx_byte == 's') {
      // Turn it on
      digitalWrite(S1_EN_PIN, LOW);  // Enable stepper driver
      
      // These four lines result in 1 step:
      digitalWrite(S1_STEP_PIN, HIGH);
      delayMicroseconds(400);
      digitalWrite(S1_STEP_PIN, LOW);
      delayMicroseconds(400);
  
      // Turn it off
      digitalWrite(S1_EN_PIN, HIGH);  // Disable stepper driver
    }
    
    /*
    // Stepper #2: Move up or down if 'w' or s' is pressed respectively unless upper or lower limit is hit respectively
    if ((digitalRead(S2_LOWER_LIMIT_SWITCH_PIN) == HIGH && digitalRead(S2_UPPER_LIMIT_SWITCH_PIN) == HIGH && (rx_byte == 'w' || rx_byte == 's')) ||
              digitalRead(S2_LOWER_LIMIT_SWITCH_PIN) == LOW && rx_byte == 'w' ||
              digitalRead(S2_UPPER_LIMIT_SWITCH_PIN) == LOW && rx_byte == 's') {
      // Turn it on
      digitalWrite(S2_EN_PIN, LOW);  // Enable stepper driver
      
      // These four lines result in 1 step:
      digitalWrite(S2_STEP_PIN, HIGH);
      delayMicroseconds(400);
      digitalWrite(S2_STEP_PIN, LOW);
      delayMicroseconds(400);
  
      // Turn it off
      digitalWrite(S2_EN_PIN, HIGH);  // Disable stepper driver
    }
    */
  }


}
