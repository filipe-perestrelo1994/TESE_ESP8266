void setup() {
  // put your setup code here, to run once:


}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.begin(115200);
  Serial.println("Read ADC pin!");
  int ORPsensorValue = analogRead(A0);
  Serial.println(ORPsensorValue);


  if(ORPsensorValue < 650)
  {
    Serial.println("Low ORP value! Add acid!");
  }
  else
  {
    if(ORPsensorValue > 750)
    {
      Serial.println("High ORP value! Add chlorine!");
    }
    else
    {
      Serial.println("ORP stable!");
    }
  } 
  delay(10000);
}
