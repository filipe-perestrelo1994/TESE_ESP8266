
float calculate_pH(int pHsensorValue, float Temperature)
{
  float pH_X;
  float pH_S = 7;
  float Es = 0.165;
  float Ex = 0.440;
  float F = 9.6485309*pow(10,4);
  float R = 8.314510;
  float T = Temperature + 273.15;

  pH_X = pH_S + ((Es - Ex)*F)/(R*T*log(10));
  Serial.print("pH: ");
  Serial.println(pH_X);
   
  return pH_X;
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  float Temperature = 23;
  Serial.println("Read ADC pin!");
  int pHsensorValue = analogRead(A0);
  Serial.println(pHsensorValue);

  calculate_pH(pHsensorValue, Temperature);
  
  delay(10000);
}
