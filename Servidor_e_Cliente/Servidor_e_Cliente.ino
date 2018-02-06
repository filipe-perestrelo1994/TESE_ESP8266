#include "esp_common.h"
#include "gpio.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <Adafruit_ADXL345_U.h> //ADXL345
#include <Wire.h> //ADXL345

//const char* ssid = "VFD 600";
//const char* password = "1234567890";

const char* ssid = "RICS_LAB";
const char* password = "rics2014lab";

//const char* ssid = "Vodafone-D4F7CF";
//const char* password = "54117E6C3E";

/*-----------------------------CLOUD----------------------------*/

#define ORG "rfh2dd"
#define DEVICE_TYPE "ESP8266_01"
#define DEVICE_ID "ESP8266_01"
//#define TOKEN 

#define authMethod "use-token-auth"
#define clientId "d:rfh2dd:ESP8266-Type:ESP8266-1"
#define token "tese12345"
#define server "rfh2dd.messaging.internetofthings.ibmcloud.com"

char topic[] = "iot-2/evt/status/fmt/json";


WiFiClient wifiClient;
PubSubClient client(server, 1883, NULL, wifiClient);
//-------- Customise the above values --------
/*-----------------------------CLOUD-------------------------------*/

/*--------------------------EFEITOS DE TESTE-----------------------*/
float pH = 7.2;
float orp = 700;
float temperature = 27;
float pressure = 1.5;
float water_level = 3;
float chlorine_level = 1;
float acid_level = 2;
/*--------------------------EFEITOS DE TESTE-----------------------*/


/*--------ENVIAR JSONS PARA A CLOUD (40 EM 40 MIN.)----------------*/
void sender_task(void *p)
{
  while(true)
   {
       if (!client.connected())
     {
       Serial.print("Reconnecting client to ");
       Serial.println(server);
       while (!client.connect(clientId, authMethod, token))
       {
         Serial.print(".");
         delay(500);
       }
       Serial.println();
     }
    
     String payload = "{\"d\":{\"pH\":";
     payload += String(pH);
     payload += ",\"ORP\":";
     payload += String(orp);
     payload += ",\"temperature\":";
     payload += String(temperature);
     payload += ",\"pressure\":";
     payload += String(pressure);
      payload += ",\"water level\":";
     payload += String(water_level);
      payload += ",\"chlorine level\":";
     payload += String(chlorine_level);
      payload += ",\"acid level\":";
     payload += String(acid_level);
     payload += "}}";
     
     Serial.print("Sending payload: ");
     Serial.println(payload);
     
     if (client.publish(topic, (char*) payload.c_str())) {
     Serial.println("Publish ok");
     } else {
     Serial.println("Publish failed");
     }
    
     vTaskDelay(2400000);
  }
}
/*--------ENVIAR JSONS PARA A CLOUD (40 EM 40 MIN.)----------------*/
void security_task(void *p)
{
  Serial.println("Security Task Working!!!!!");
}


/*-----------------------------SERVIDOR----------------------------*/

WiFiServer server(80);

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

/*-----------------------------SERVIDOR----------------------------*/


/*----------------------------SETUP-------------------------------*/
void setup() {
 Serial.begin(115200);
 Serial.println();

 Serial.print("Connecting to "); Serial.print(ssid);
 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
 delay(500);
 Serial.print(".");
 } 
 Serial.println("");

 Serial.print("WiFi connected, IP address: "); Serial.println(WiFi.localIP());
}
/*----------------------------SETUP-------------------------------*/
void loop() {

  xTaskCreate(sender_task, (signed char*) "sender_task", 1024, NULL, 1, NULL);
  vTaskStartScheduler();

}

