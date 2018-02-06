
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <Wire.h>

ESP8266WebServer server(80);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Wire.begin(14,12);  
  Serial.println("Wire begin! GPIO9 and GPIO10");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Reading water level sensor");
  bool Bit0 = digitalRead(12);
  bool Bit1 = digitalRead(14);

  Serial.print("Bit0 = ");
  Serial.println(Bit0);
  Serial.print("Bit1 = ");
  Serial.println(Bit1);
  
  if(Bit1 == HIGH && Bit0 == LOW)
  {
    Serial.println("Level 3");
    delay(1000);
  }
  if(Bit1 == HIGH && Bit0 == HIGH)
  {
    Serial.println("Level 2");
    delay(1000);
  }
  if(Bit1 == LOW && Bit0 == LOW)
  {
    Serial.println("Level 1");
    delay(1000);
  }
  if(Bit1 == LOW && Bit0 == HIGH)
  {
    Serial.println("Level 0");
    delay(1000);
  }
} 
