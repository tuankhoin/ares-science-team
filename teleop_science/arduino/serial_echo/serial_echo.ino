int x = 20;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available() > 0) {
    x = Serial.read();
    Serial.write(x);
    Serial.print("hello");
    Serial.readString();
  }
}
