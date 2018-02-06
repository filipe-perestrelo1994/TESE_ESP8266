#include <Wire.h>


boolean readMultiplexer(boolean S0, boolean S1, boolean S2)
{

  digitalWrite(14,S0);
  digitalWrite(12,S1);
  digitalWrite(13,S2);

  delay(500);

  boolean y = digitalRead(4);

  delay(500);

  return y;
}

void defineWaterLevel(boolean water1, boolean water0)
{
  if(water1 == LOW && water0 == HIGH)
  {
      Serial.println("Level 0 - Empty");
      delay(1000);
  }
  
  if(water1 == LOW && water0 == LOW)
  {
      Serial.println("Level 1 - Almost Empty");
      delay(1000);
  }

  if(water1 == HIGH && water0 == HIGH)
  {
      Serial.println("Level 2 - Normal");
      delay(1000);
  }

  if(water1 == HIGH && water0 == LOW)
  {
      Serial.println("Level 3 - Full");
      delay(1000);
  }
  
}

void defineChlorineLevel(boolean chlorine1, boolean chlorine0)
{
  if(chlorine1 == LOW && chlorine0 == HIGH)
  {
    Serial.println("Level 1 - Almost Empty");
    delay(1000);
  }
  
  if(chlorine1 == LOW && chlorine0 == LOW)
  {
    Serial.println("Level 2 - Normal");
    delay(1000);
  }
  
  if(chlorine1 == HIGH && chlorine0 == HIGH)
  {
    Serial.println("Level 0 - Empty");
    delay(1000);
  }
}

void defineAcidLevel(boolean acid1, boolean acid0)
{
  if(acid1 == LOW && acid0 == HIGH)
  {
    Serial.println("Level 1 - Almost Empty");
    delay(1000);
  }
  
  if(acid1 == LOW && acid0 == LOW)
  {
    Serial.println("Level 2 - Normal");
    delay(1000);
  }
  
  if(acid1 == HIGH && acid0 == HIGH)
  {
    Serial.println("Level 0 - Empty");
    delay(1000);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(13,14);
  Wire.begin(12,4);

  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(14,OUTPUT);
  pinMode(4,INPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.println("Reading water sensor");
  
  bool  S0 = LOW;
  bool  S1 = LOW;
  bool  S2 = LOW;

  delay(5000);
  bool  water0 = readMultiplexer(S0,S1,S2);
  
  
  S0 = HIGH;
  S1 = LOW;
  S2 = LOW;

  delay(5000);
  bool  water1 = readMultiplexer(S0,S1,S2);
  
  
  Serial.print("Water0 = ");
  Serial.println(water0);
  Serial.print("Water1 = ");
  Serial.println(water1);

  defineWaterLevel(water1, water0);


  Serial.println("Reading chlorine sensor");
  
  S0 = LOW;
  S1 = HIGH;
  S2 = LOW;

  delay(5000);
  bool chlorine0 = readMultiplexer(S0,S1,S2);
  
  
  S0 = HIGH;
  S1 = HIGH;
  S2 = LOW;

  delay(5000);
  bool chlorine1 = readMultiplexer(S0,S1,S2);
  

  Serial.print("Chlorine0 = ");
  Serial.println(chlorine0);
  Serial.print("Chlorine1 = ");
  Serial.println(chlorine1);

  defineChlorineLevel(chlorine1, chlorine0);

  Serial.println("Reading acid sensor");

  S0 = LOW;
  S1 = LOW;
  S2 = HIGH;

  delay(5000);
  bool acid0 = readMultiplexer(S0,S1,S2);
  
  
  S0 = HIGH;
  S1 = LOW;
  S2 = HIGH;
  
  bool acid1 = readMultiplexer(S0,S1,S2);
  delay(5000);

  Serial.print("Acid0 = ");
  Serial.println(acid0);
  Serial.print("Acid1 = ");
  Serial.println(acid1);

  defineAcidLevel(acid1, acid0);

  
}
