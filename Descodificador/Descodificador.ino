#include <Wire.h>


void writeDecoder(boolean S0, boolean S1)
{

  delay(500);
  digitalWrite(15,LOW);
  digitalWrite(3,S0);
  digitalWrite(1,S1);
 
  delay(500);

}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Pinos inicializados");

  pinMode(1,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(15,OUTPUT);

  Serial.println("Pinos definidos");
}

void loop() {
  // put your main code here, to run repeatedly:
  boolean S0 = LOW;
  boolean S1 = LOW;
  writeDecoder(S0,S1);

  Serial.println("Output 1");
  
  delay(1000);
  
  S0 = HIGH;
  S1 = LOW;
  writeDecoder(S0,S1);

  Serial.println("Output 2");

  delay(1000);
  
  S0 = LOW;
  S1 = HIGH;
  writeDecoder(S0,S1);

  Serial.println("Output 3");

  delay(1000);
  
  S0 = HIGH;
  S1 = HIGH;
  writeDecoder(S0,S1);

  Serial.println("Output 4");

  delay(1000);
}
