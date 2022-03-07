

// Servo
#include <Servo.h>
Servo container_servo;
Servo probe_servo;
#define CONTAINER_SERVO_PIN 9
#define PROBE_SERVO_PIN 10

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

// Toggle drill command status flag
boolean drilling = false;

// Set stepper command status
char S1_status;
char S2_status;


void setup() {
  
  // For command inputs
  Serial.begin(9600); 


  // Servo
  container_servo.attach(CONTAINER_SERVO_PIN);
  container_servo.write(0);   // Rotate to 0° otherwise it will open automatically
  probe_servo.attach(PROBE_SERVO_PIN);
  probe_servo.write(120);   // Rotate to 0° otherwise it will open automatically

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

  
  // Note: Setting the direction of rotation for the steppers can't be run every loop otherwise stepper makes a terrible noise
  // Set S1 command
  if (change_command && rx_byte == 'w') {
    Serial.println("S1 going up");
    // Set the spinning direction clockwise (move up):
    digitalWrite(S1_DIR_PIN, LOW);  // stepper driver #1
    S1_status = 'w';
    
    change_command = false;
  } else if (change_command && rx_byte == 's') {
    Serial.println("S1 going down");
    // Set the spinning direction counterclockwise (move down):
    digitalWrite(S1_DIR_PIN, HIGH);   // stepper driver #1
    S1_status = 's';

    change_command = false;
  } else if (change_command && rx_byte == 'e') {
    Serial.println("S1 stopped");
    S1_status = 'e';

    change_command = false;    
  }
  // Set S2 command
  else if (change_command && rx_byte == 'r') {
    Serial.println("S2 going up");
    // Set the spinning direction clockwise (move up):
    digitalWrite(S2_DIR_PIN, LOW);  // stepper driver #2
    S2_status = 'r';

    change_command = false;
  } else if (change_command && rx_byte == 'f') {
    Serial.println("S2 going down");
    // Set the spinning direction counterclockwise (move down):
    digitalWrite(S2_DIR_PIN, HIGH);   // stepper driver #2
    S2_status = 'f';

    change_command = false;
  } else if (change_command && rx_byte == 't') {
    Serial.println("S2 stopped");
    S2_status = 't';

    change_command = false;    
  }

  // Set drill command
  else if (change_command && rx_byte == 'd') {
    if (drilling) {
      drilling = false;
      Serial.println("Drill OFF");
    } else {
      drilling = true;
      Serial.println("Drill ON");
    }

    change_command = false;
  }

  else if (change_command && rx_byte == 'm') {
    //Khoi put in later
    Serial.println(analogRead(0));
    change_command = false;
  }
  


  // Stop everything if 'q' is pressed
  if (rx_byte == 'q') {
    drilling = false;
    S1_status = 'e';
    S2_status = 't';
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

  // S1:
  if (S1_status == 'e') {
    // Turn it off
    digitalWrite(S1_EN_PIN, HIGH);  // Disable stepper driver
  }
  // Move up or down if 'w' or 's' is pressed respectively unless upper or lower limit is hit respectively
  else if ((digitalRead(S1_LOWER_LIMIT_SWITCH_PIN) == HIGH && digitalRead(S1_UPPER_LIMIT_SWITCH_PIN) == HIGH && (S1_status == 'w' || S1_status == 's')) ||
            digitalRead(S1_LOWER_LIMIT_SWITCH_PIN) == LOW && S1_status == 'w' ||
            digitalRead(S1_UPPER_LIMIT_SWITCH_PIN) == LOW && S1_status == 's') {
    // Turn it on
    digitalWrite(S1_EN_PIN, LOW);  // Enable stepper driver
    delayMicroseconds(400);
    
    // These four lines result in 1 step:
    digitalWrite(S1_STEP_PIN, HIGH);
    delayMicroseconds(400);
    digitalWrite(S1_STEP_PIN, LOW);
    delayMicroseconds(400);

    // Turn it off
    digitalWrite(S2_EN_PIN, HIGH);  // Disable stepper driver
  }
   
  // S2: 
  if (S2_status == 't') {
    // Turn it off
    digitalWrite(S2_EN_PIN, HIGH);  // Disable stepper driver
  }
  // Move up or down if 'r' or 'f' is pressed respectively unless upper or lower limit is hit respectively
  else if ((digitalRead(S2_LOWER_LIMIT_SWITCH_PIN) == HIGH && digitalRead(S2_UPPER_LIMIT_SWITCH_PIN) == HIGH && (S2_status == 'r' || S2_status == 'f')) ||
            digitalRead(S2_LOWER_LIMIT_SWITCH_PIN) == LOW && S2_status == 'r' ||
            digitalRead(S2_UPPER_LIMIT_SWITCH_PIN) == LOW && S2_status == 'f') {
    // Turn it on
    digitalWrite(S2_EN_PIN, LOW);  // Enable stepper driver
    delayMicroseconds(400);
    
    // These four lines result in 1 step:
    digitalWrite(S2_STEP_PIN, HIGH);
    delayMicroseconds(400);
    digitalWrite(S2_STEP_PIN, LOW);
    delayMicroseconds(400);

    // Turn it off
    digitalWrite(S2_EN_PIN, HIGH);  // Disable stepper driver
  }
 
  // Container close
  if (rx_byte == 'c') {
    container_servo.write(0);   // Rotate to 0°
    Serial.println("Closing");
    // probe_servo.write(0);   // Rotate to 0°
  }

  // Container open
  if (rx_byte == 'o') {
    container_servo.write(120);   // Rotate to ~180°
    Serial.println("Opening");
    //probe_servo.write(120);   // Rotate to 0°
  }

  if (change_command && rx_byte == 'p') {
    int current = probe_servo.read();
    probe_servo.write((current > 90) ? 0 : 120);   // Rotate to 0°
    change_command = false;
  }

  
}
