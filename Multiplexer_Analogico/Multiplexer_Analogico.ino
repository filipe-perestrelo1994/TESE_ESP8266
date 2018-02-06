#include <Wire.h>



float AnalogSensorValue(boolean S0)//, boolean S1, boolean S2)
{
   // digitalWrite(12,S1);
  //digitalWrite(13,S2);
  digitalWrite(14,S0);
  int sensorValue = analogRead(A0);
  delay(500);
  
  sensorValue = map(sensorValue, 0, 1023, 0, 1000);
  Serial.print(sensorValue);
  Serial.println(" mV");
  
  return sensorValue;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //pinMode(13,OUTPUT);
  //pinMode(12,OUTPUT);
  pinMode(14,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  bool S0 = LOW;
  //bool S1 = LOW;
  //bool S2 = LOW;
  
  Serial.println("pH:");
  //int pHsensorValue = AnalogSensorValue(S0, S1, S2);
  //float pHsensorValue = (float) analogRead(A0);
  double pH_mV = (double)AnalogSensorValue(S0);//, S1, S2);
  
  Serial.println(pH_mV);

  yield();

  S0 = HIGH;
  //S1 = HIGH;
  //S2 = HIGH;
 

  Serial.println("ORP:");
  int ORPsensorValue = (int) AnalogSensorValue(S0);//, S1, S2);
  //float ORPsensorValue = (float) analogRead(A0);
  Serial.println(ORPsensorValue);

}
