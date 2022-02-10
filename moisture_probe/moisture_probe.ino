#include <SoftwareSerial.h>
#include <Wire.h>


//not sure what these pins
#define RE 8
#define DE 7

//this is the message we send to the probe sensor to elicit a response about the temp, moisture and Electrical content of the soil
const byte temp[] = {0x01, 0x04, 0x0000, 0x0003,0xB00B};

//this array stores the values of the return message from the probe
byte values[10];

//this creates a new serial port
SoftwareSerial mod(2, 3);

void setup()
{
  Serial.begin(9600);
   while(!Serial);

  Serial.println("Modbus moisture sensor");
  mod.begin(9600);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  delay(3000);
}
void loop()
{
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);

  //we write to the probe here
  if (mod.write(temp, sizeof(temp)) == 5)
  {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);

    //in this loop, we assign the elements of the return message to elements of the values array
    for (int i = 0; i < 10; i++)
    {
      Serial.println(i);
      values[i] = mod.read();
      Serial.print(values[i]);
      Serial.println();
    }
  }
  float mositure = float(values[6]) / 10;
  Serial.print("moisture: ");
  Serial.println(moisture);
  
  delay(3000);
}
