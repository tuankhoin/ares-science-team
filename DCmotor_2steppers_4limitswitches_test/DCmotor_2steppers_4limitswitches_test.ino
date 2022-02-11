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

// Serial monitor input character
char rx_byte;

// Change command flag
boolean change_command;

// Toggle drill command flag
boolean drilling = false;


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
  
  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();       // get the character
    change_command = true;
  }

  
  // Set the direction of movement for the steppers. This can't be run every loop otherwise stepper makes a terrible noise
  // S1
  if (change_command && rx_byte == 'w') {
    Serial.println("S1 going up");
    // Set the spinning direction clockwise (move up):
    digitalWrite(S1_DIR_PIN, LOW);  // stepper driver #1
    
    change_command = false;
  } else if (change_command && rx_byte == 's') {
    Serial.println("S1 going down");
    // Set the spinning direction counterclockwise (move down):
    digitalWrite(S1_DIR_PIN, HIGH);   // stepper driver #1

    change_command = false;
  }
  // S2
  else if (change_command && rx_byte == 'r') {
    Serial.println("S2 going up");
    // Set the spinning direction clockwise (move up):
    digitalWrite(S2_DIR_PIN, LOW);  // stepper driver #2

    change_command = false;
  } else if (change_command && rx_byte == 'f') {
    Serial.println("S2 going down");
    // Set the spinning direction counterclockwise (move down):
    digitalWrite(S2_DIR_PIN, HIGH);   // stepper driver #2

    change_command = false;
  }

  // Set drill command
  if (change_command && rx_byte == 'd') {
    if (drilling) {
      drilling = false;
      Serial.println("Drill OFF");
    } else {
      drilling = true;
      Serial.println("Drill ON");
    }

    change_command = false;
  }
  


  // Stop everything if 'q' is pressed
  if (rx_byte == 'q') {
    analogWrite(E1, LOW); // No DC motor voltage
    drilling = false;
    digitalWrite(S1_EN_PIN, HIGH);  // Disable stepper driver #1
    digitalWrite(S2_EN_PIN, HIGH);  // Disable stepper driver #2
  }
    
  // Toggle drill if 'd' is pressed
  if (drilling) {
    // Spin at full speed
    analogWrite(E1, 255);
    //delay(30);  // So it can have time to get up-to-speed
  } else {  
    // Turn it off
    analogWrite(E1, LOW); // No DC motor voltage
  }

  // Servo open
  if (rx_byte == 'o') {
    myservo.write(120);   // Rotate to ~180°
    Serial.println("Opening");
  }
  
  // Servo close
  if (rx_byte == 'c') {
    myservo.write(0);   // Rotate to 0°
    Serial.println("Closing");
  }

  // Stepper #1:
  if (rx_byte == 'e') {
    // Turn it off
    digitalWrite(S1_EN_PIN, HIGH);  // Disable stepper driver
  }
  // Move up or down if 'w' or 's' is pressed respectively unless upper or lower limit is hit respectively
  else if ((digitalRead(S1_LOWER_LIMIT_SWITCH_PIN) == HIGH && digitalRead(S1_UPPER_LIMIT_SWITCH_PIN) == HIGH && (rx_byte == 'w' || rx_byte == 's')) ||
            digitalRead(S1_LOWER_LIMIT_SWITCH_PIN) == LOW && rx_byte == 'w' ||
            digitalRead(S1_UPPER_LIMIT_SWITCH_PIN) == LOW && rx_byte == 's') {
    // Turn it on
    digitalWrite(S1_EN_PIN, LOW);  // Enable stepper driver
    
    // These four lines result in 1 step:
    digitalWrite(S1_STEP_PIN, HIGH);
    delayMicroseconds(400);
    digitalWrite(S1_STEP_PIN, LOW);
    delayMicroseconds(400);
  }
   
  // Stepper #2: 
  if (rx_byte == 't') {
    // Turn it off
    digitalWrite(S2_EN_PIN, HIGH);  // Disable stepper driver
  }
  // Move up or down if 'r' or 'f' is pressed respectively unless upper or lower limit is hit respectively
  else if ((digitalRead(S2_LOWER_LIMIT_SWITCH_PIN) == HIGH && digitalRead(S2_UPPER_LIMIT_SWITCH_PIN) == HIGH && (rx_byte == 'r' || rx_byte == 'f')) ||
            digitalRead(S2_LOWER_LIMIT_SWITCH_PIN) == LOW && rx_byte == 'r' ||
            digitalRead(S2_UPPER_LIMIT_SWITCH_PIN) == LOW && rx_byte == 'f') {
    // Turn it on
    digitalWrite(S2_EN_PIN, LOW);  // Enable stepper driver
    
    // These four lines result in 1 step:
    digitalWrite(S2_STEP_PIN, HIGH);
    delayMicroseconds(400);
    digitalWrite(S2_STEP_PIN, LOW);
    delayMicroseconds(400);
  }
  

}
