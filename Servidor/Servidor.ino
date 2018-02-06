#include <Adafruit_ADXL345_U.h> //ADXL345
#include <Wire.h> //ADXL345
#include <ESP8266WiFi.h> //ESP8266



//const char* ssid = "Vodafone-D4F7CF";
//const char* password = "54117E6C3E";


const char* ssid = "RICS_LAB";
  const char* password = "rics2014lab";

//const char* ssid = "VFD 600";
//const char* password = "1234567890";

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(80);



String security;
String maintenance;



int setTemperature(int temp_request)
{
  int  temperature = temp_request;
  return temperature;
}


void setup() 
{


   Wire.begin(0,2);

  
  Serial.begin(115200);
  delay(10);

  // prepare GPIO2
  pinMode(2, OUTPUT);
  digitalWrite(2, 0);
  
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.println(WiFi.localIP());
}

void loop() {
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  
  // Wait until the client sends some data
  Serial.println("new client");
  while(!client.available()){
    delay(1);
  }

 String poolname = "qualquer coisa";
 int pHvalue =7.5;
 int ORP = 650;
 int pressure = 1;
 String waterLevel = "Full";
 String chlorineLevel = "Empty";
 String acidLevel = "Empty";
 int temperature = 25;
 
  String jsonAux = "{\"d\":{\"Poolname\":";
  jsonAux += poolname;
  jsonAux += ",\"Temperature\":";
  jsonAux += String(temperature);
  jsonAux += ",\"pH\":";
  jsonAux += String(pHvalue);
  jsonAux += ",\"ORP\":";
  jsonAux += String(ORP);
  jsonAux += ",\"Pressure\":";
  jsonAux += String(pressure);
  jsonAux += ",\"Water level\":";
  jsonAux += waterLevel;
  jsonAux += ",\"Chlorine level\":";
  jsonAux += chlorineLevel;
  jsonAux += ",\"Acid Level\":";
  jsonAux += acidLevel;
  jsonAux += ",\"Security State\":";
  jsonAux += security;
  jsonAux += ",\"Maintenance state\":";
  jsonAux += maintenance;
  jsonAux += "}}\0";


  // Read the request
  String req1 = client.readString();
  String req = client.readStringUntil('=');
  String String_id = client.readStringUntil('&');
  String String_msg = client.readStringUntil('=');
  Serial.println(req1);
  Serial.println(req);
  Serial.println(String_id);
  Serial.println(String_msg);

  int id = String_id.toInt();

  String s = "";
 switch(id)
 {
  case 0: 
    setTemperature(String_msg.toInt());
    s = "Temperature = " + String_msg;
    Serial.println(s);
    client.print(s);
    break;
  
  case 2:
    if(String_msg == "SecurityOn")
    {
      boolean SecurityOn = true;
      security = "Security mode activated";
      Serial.println(security);
      client.print(security);
    }
    else if (String_msg == "SecurityOff")
    {
      boolean SecurityOn = false;
      security = "Security mode deactivated";
      Serial.println(security);
      client.print(security);
    }
    break;
  case 3:
    if(String_msg == "PumpOff")
    {
      maintenance = "Maintenance Suspended";
      Serial.println(maintenance);
      client.print(maintenance);
    }
    else if (String_msg == "PumpOn")
    {
      maintenance = "Maintenance Restarted";
      Serial.println(maintenance);
      client.print(maintenance);
    }
 }
// String poolname = "qualquer coisa";
// int pHvalue =7.5;
// int ORP = 650;
// int pressure = 1;
// String waterLevel = "Full";
// String chlorineLevel = "Empty";
// String acidLevel = "Empty";
// int temperature = 25;
/*
  String jsonAux = "{\"d\":{\"Poolname\":";
  jsonAux += poolname;
  jsonAux += ",\"Temperature\":";
  jsonAux += String(temperature);
  jsonAux += ",\"pH\":";
  jsonAux += String(pHvalue);
  jsonAux += ",\"ORP\":";
  jsonAux += String(ORP);
  jsonAux += ",\"Pressure\":";
  jsonAux += String(pressure);
  jsonAux += "}}\0";
*/
  
  Serial.println(jsonAux);
  char json[500];
  jsonAux.toCharArray(json, jsonAux.length());
  client.print(json);
  
  client.flush();


  // Send the response to the client
  
 // delay(1);
  Serial.println("Client disonnected");

}

